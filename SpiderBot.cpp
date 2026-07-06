/*
Copyright (C) 2026 aWildKaelin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <stdio.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
//#include "hardware/timer.h"
#include "pico/cyw43_arch.h"
#include <math.h>


//https://github.com/Harbys/pico-ssd1306
#define SSD1306_ASCII_FULL

#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"


#include "rawServo.h"

float maxf(float a, float b){
    return a > b ? a : b;
}
float minf(float a, float b){
    return a < b ? a : b;
}


typedef struct vec3{
    float x, y, z;
}vec3;

typedef struct vec2{
    float x, y;
}vec2;

enum struct moveType{
    trot, crawl, still
};

moveType robotState = moveType::trot;

enum struct emotion{
    normal = 0, happy = 1, dizzy, sad, excited, curious, sleepy, embarrassed, crying
};



#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17

pico_ssd1306::SSD1306* display = nullptr;


void initDisplay(){
    i2c_init(I2C_PORT, 1000000); //Use i2c port with baud rate of 1Mhz

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c


    display = new pico_ssd1306::SSD1306(I2C_PORT, 0x3C, pico_ssd1306::Size::W128xH64);
    if(!display){
        while(true)
        {
            printf("Failed to initialize display!\n");
            sleep_ms(5000);
        }
    }
    display->setOrientation(true);
}

void setFace(emotion face, pico_ssd1306::SSD1306 *display){
    display->clear();

    switch(face){
    case emotion::normal:
        drawText(display, font_16x32, "._.", 40, 16);
        break;
    case emotion::happy:
        drawText(display, font_16x32, "`W`", 40, 16);
        break;
    case emotion::dizzy:
        drawText(display, font_16x32, "@~@", 40, 16);
        break;
    case emotion::sad:
        drawText(display, font_16x32, "-n-", 40, 16);
        break;
    case emotion::excited:
        drawText(display, font_16x32, "^w^", 40, 16);
        break;
    case emotion::curious:
        drawText(display, font_16x32, "`o`", 40, 16);
        break;
    case emotion::sleepy:
        drawText(display, font_16x32, "-w-", 40, 16);
        break;
    case emotion::embarrassed:
        drawText(display, font_16x32, "@///@", 24, 16);
        break;
    case emotion::crying:
        drawText(display, font_16x32, ";-;", 40, 16);
        break;
    }

    display->sendBuffer();
}





// TODO: give this the actual measurements of the robot
#define BOT_WIDTH 80    // mm
#define BOT_LENGTH 130
#define ARM_LENGTH 60.0f //mm
#define SERVO_CNT 8

float chassisPitch = 0, chassisRoll = 0;


#define SERVO_CENTER M_PI_2 // default home position
#define SWING_HEIGHT 40 // raise leg by 40mm
#define SWING_ARC_RAD 1  // swing leg 1 radian forward and back

// on board used for testing, GPIO 14 is completely fried, not even a basic blink program makes it through
uint servoPins[SERVO_CNT] = {6, 7, 8, 9, 10, 11, 12, 13};


typedef struct leg{
    uint phiServo; // forward - back
    uint thetaServo; // up - down

    // set forward-back angle in radians
    void setPhi(float angle){
        setAngleRad(phiServo, angle);
    };

    // set up-down angle in radians
    void setTheta(float angle){
        setAngleRad(thetaServo, angle);
    };


    float phaseOffset = 0;

    // two values representing the same thing in different units
    // height is the offset in mm of the corner of the robot that holds this leg
    // baseAngle is the servo angle that thetaServo needs to hold to achieve height
    float height = 0, baseAngle = M_PI/2;
}leg;



// X forward, Y right, Z up
const vec3 cornerFR = { BOT_LENGTH / 2,  BOT_WIDTH / 2, 0};
const vec3 cornerBR = {-BOT_LENGTH / 2,  BOT_WIDTH / 2, 0};
const vec3 cornerFL = { BOT_LENGTH / 2, -BOT_WIDTH / 2, 0};
const vec3 cornerBL = {-BOT_LENGTH / 2, -BOT_WIDTH / 2, 0};

// forward-back, up-down
leg legFR = {13, 12};
leg legBR = {11, 10};
leg legFL = {9, 8};
leg legBL = {6, 7};

leg* legs[4] = {
    &legFR,
    &legBR,
    &legFL,
    &legBL
};

// helps mirror servos to keep the math identical
vec2 legOrientations[] ={
    vec2{1, -1},
    vec2{1, -1},
    vec2{-1, 1},
    vec2{-1, 1},
};



float getHeightAngle(float targetHeight){
    // Inverse Kinematics iterative approach
    /*
    float tempAngle = 0;
    float temp = 0; //middle
    float range = M_PI_2;

    // like a binary search
    while(fabs(targetHeight - temp) > 5){ // if current is within 5mm of target
        
        if(targetHeight > temp)                 // check if we're over or under
            tempAngle += range * 0.5f;
        else
            tempAngle -= range * 0.5f;

        temp = sinf(tempAngle) * ARM_LENGTH;  // generate condition //expensive operation
            range /= 2;                         // shrink search area
    }

    return tempAngle;
    */
    // TODO: add safety clamp
    float value = maxf(minf(targetHeight / ARM_LENGTH, 1), -1);;
    return asinf(value);
}



double phaseFloat = 0;
float trotOffset[] = {0.5, 0, 0, 0.5};
float crawlOffset[] = {0, 0.5, 0.25, 0.75};


// movement has two steps: swing and return
// during swing, the leg traces a semicircle and brings itself forward
// during return, the leg moves back and pushes the robot forward
// The time for the two steps must always add to one
// I've chosen to return time as 1 - swing time instead of giving it a new variable
const float trotSwingTime = 0.25;
const float crawlSwingTime = 0.2;


// first value is forward-back, 2nd is up-down
vec2 getLegPos(float phase){
    phase = minf(phase, 1);
    if(robotState != moveType::still){
        float swingTime = robotState == moveType::trot ? trotSwingTime : crawlSwingTime;

        if(phase < swingTime){
            float phaseProgress = phase / swingTime;
            float heightAngle = getHeightAngle((sinf(M_PI * phaseProgress)) * SWING_HEIGHT);
            float strideAngle = cosf(M_PI * phaseProgress) * SWING_ARC_RAD;
            return {strideAngle, heightAngle};
        }
        else{
            float phaseProgress = (phase - swingTime) / (1.0f - swingTime);
            float strideAngle = (2 * phaseProgress - 1) * SWING_ARC_RAD;  // the swing goes between 1 and -1 radians, this brings it from -1 back to 1
            return {strideAngle, 0};
        }
    }
    else return {0, 0};
}




// extracts one result out of matrix multiplication
// the corner height after Pitch x Roll
// assumes Z always starts at 0 AND corners are const
float getCornerHeight(vec3 corner, float pitch, float roll){
    float result =  -corner.x * sin(pitch) + 
                    (corner.y * sin(roll) * cos(pitch));

    //TODO: parametrize this clamp
    // makes sure the requested height isn't out of bounds and leaves headroom for walking
    return maxf(minf(result, ARM_LENGTH - 15), -ARM_LENGTH + 5);
}

// mm, deg, deg
void setPose(float height, float pitch, float roll){
    legFR.height = legOrientations[0].x * (height + getCornerHeight(cornerFL, pitch, roll));
    legBR.height = legOrientations[1].x * (height + getCornerHeight(cornerBL, pitch, roll));
    legFL.height = legOrientations[2].x * (height + getCornerHeight(cornerFR, pitch, roll));
    legBL.height = legOrientations[3].x * (height + getCornerHeight(cornerBR, pitch, roll));

    legFL.baseAngle = M_PI_2 + getHeightAngle(legFL.height);
    legFR.baseAngle = M_PI_2 + getHeightAngle(legFR.height);
    legBL.baseAngle = M_PI_2 + getHeightAngle(legBL.height);
    legBR.baseAngle = M_PI_2 + getHeightAngle(legBR.height);
}



void setMovementType(moveType type){
    if(type == moveType::trot){
        for(int i = 0; i < 4; i++){
            legs[i]->phaseOffset = trotOffset[i];
        }
    }
    else if(type == moveType::crawl){
        for(int i = 0; i < 4; i++){
            legs[i]->phaseOffset = crawlOffset[i];
        }
    }
    else{
        for(int i = 0; i < 4; i++){
            legs[i]->phaseOffset = 0;
        }
    }
}



// defined as how much of a step is finished per second
double timestep = 0;
// -100 -> +100
void setSpeed(float speed){

    //TODO: currently doesn't support going backwards, clamped to forward for testing
    speed = minf(100, maxf(0, speed));

    // a timestep of 1 means the robot is going through an entire walk cycle once per second
    // TODO: test and find a decent upper limit
    timestep = speed / 200;
}
//TODO: on 0 throttle for half a second, return to standstill


// yaw rate is a multiplier applied to stride length
// positive makes left stride shorter, turning left, negative shortens right stride
//TODO: implement
void setHeading(float yawRate);




//TODO: repurpose into a generic "RECEIVE" function regardless of receive method
//TODO: finish this
// will integrate with the upcoming crsf parser
void receiveSerial(){

    // the current setup allows receiving messages over serial with USB
    // this will be discarded later
    while (tud_cdc_available())
    {
        // Create a place to hold the incoming message
        static char message[16];
        static unsigned int message_pos = 0;
        // Read the next available byte in the serial receive buffer
        char inByte = getchar();
        // Message coming in (check not terminating character) and guard for over message size
        if (inByte != '\n' && (message_pos < 16 - 1))
        {
            // Add the incoming byte to our message
            message[message_pos] = inByte;
            message_pos++;
        }
        // Full message received...
        else
        {
            switch(message[0]){
            case 's':
                setSpeed(5);
                break;
            }
            message_pos = 0;
        }
    }
}



int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    sleep_ms(5000);

    initServo(servoPins, SERVO_CNT);
    
    for(int i = 0; i < 4; i++){
        legs[i]->setTheta(M_PI_2);
        legs[i]->setPhi(M_PI_2);
    }

    
    initDisplay();
    printf("%i\n", display);

    
    /*
    // loop used to showcase the faces of the robot    
    int emote =  0;
    while(true){
        setFace(emotion(emote), display);
        printf("%i\n", emote);
        emote++;
        emote%=9;
        sleep_ms(2000);
    }
    */

    /*
    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("Your Wi-Fi SSID", "Your Wi-Fi Password", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        while(true){
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            sleep_ms(800);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            sleep_ms(500);
        }
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }
    */
    
    setSpeed(10);
    setPose(50, 0, 0);

    setFace(emotion::happy, display);
    
    setMovementType(moveType::crawl);

    // code used to center the servos in order to mount them properly
    /*
    for(int i = 0; i < 4; i++){
        legs[i]->setTheta(M_PI_2);
        legs[i]->setPhi(M_PI_2);
    }
    while(true){sleep_ms(1000);}
    */
    


    // LED turning off means setup stopped
    // TODO: add #ifndef for pico and pico W boards
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    

    uint dt = 0;
    uint64_t lastTime = time_us_64();
    while (true) {
        //receiveSerial(); //to add: setSpeed, setFace, setPose,

        if(robotState != moveType::still){

            phaseFloat += timestep * (dt / 1000000.0);


            // !!!!!!!!!!!!!!!!!!!!! IMPORTANT NOTE !!!!!!!!!!!!!!!!!!!!! 
            // servos should ALWAYS work in offsets!
            // up-down axis offsets from baseAngle
            // forward-back axis offsets from M_PI_2
            // when assembled, legs should extend out from the center of the servo's range

            while(phaseFloat >= 1.0)
                phaseFloat -= 1.0;
            
            
            for(int i = 0; i < 4; i++){
                double phase = legs[i]->phaseOffset + phaseFloat;
                phase = phase < 1 ? phase : phase - 1;

                vec2 result = getLegPos(phase);
                legs[i]->setPhi(M_PI_2 + (result.x * legOrientations[i].x));
                legs[i]->setTheta(legs[i]->baseAngle + (result.y * legOrientations[i].y));
            }
        }

        // delta time calculation
        dt = uint(time_us_64() - lastTime);
        lastTime = time_us_64();

    }
}