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

float clampMax(float a, float b){
    return a > b ? a : b;
}
float clampMin(float a, float b){
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


#define SERVO_CNT 8

// both axes and regardless of side will make sure that they sit centered when servo is at half
#define SERVO_CENTER 90

// GPIO 14 is completely fried, not even a basic blink program makes it through
uint servoPins[SERVO_CNT] = {6, 7, 8, 9, 10, 11, 12, 13};
//uint servoSlices[SERVO_CNT];


typedef struct leg{
    uint phiServo; // forward - back
    uint thetaServo; // up - down

    void setPhi(float angle){
        setAngleRad(phiServo, angle);
        pose.x = angle;
    };
    void setTheta(float angle){
        setAngleRad(thetaServo, angle);
        pose.y = angle;
    };

    float phaseOffset = 0;

    // stores its own height for reference and the servo angle that represents 0
    // baseAngle is radians and only applies to height, not stride
    float height = 0, baseAngle = M_PI/2;

    // x/y, phi/theta
    vec2 pose = {0, 0};
    vec2 start = {0, 0}, target = {0, 0};
}leg;



#define BOT_WIDTH 80    // mm
#define BOT_LENGTH 130

// This is 
#define ARM_LENGTH 60.0f
// this is the actuator arm of the leg, as the 4 bar linkage acts as a static, 
// constant offset for the robot's height

// X forward, Y right, Z up

const vec3 cornerFL = { BOT_LENGTH / 2, -BOT_WIDTH / 2, 0};
const vec3 cornerFR = { BOT_LENGTH / 2,  BOT_WIDTH / 2, 0};
const vec3 cornerBL = {-BOT_LENGTH / 2, -BOT_WIDTH / 2, 0};
const vec3 cornerBR = {-BOT_LENGTH / 2,  BOT_WIDTH / 2, 0};

leg legFL = {6, 7};
leg legBL = {8, 9};
leg legFR = {10, 11};
leg legBR = {12, 13};

leg* legs[4] = {
    &legFL,
    &legBL,
    &legFR,
    &legBR
};

vec2 legOrientations[] ={
    vec2{-1, 1},
    vec2{-1, 1},
    vec2{1, -1},
    vec2{1, -1}
};


double phaseFloat = 0;
float trotOffset[] = {0.5, 0, 0, 0.5};
float walkOffset[] = {0, 0.25, 0.5, 0.75};



float getHeightAngle(float targetHeight){
    /*
    float tempAngle = 0;
    float temp = 0; //middle
    float range = M_PI_2;

    // like a binary serch
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
    return asin(targetHeight / ARM_LENGTH);
}






float chassisPitch = 0, chassisRoll = 0;

// extracts the corner height out of Pitch x Roll x pos
// assumes Z always starts at 0 AND corners are const
float getCornerHeight(vec3 corner, float pitch, float roll){
    float result =  -corner.x * sin(pitch) + 
                    (corner.y * sin(roll) * cos(pitch));

    return clampMax(clampMin(result, ARM_LENGTH - 15), -ARM_LENGTH + 5);
}


// mm, deg, deg
void setPose(float height, float pitch, float roll){
    legFL.height = legOrientations[0].x * (height + getCornerHeight(cornerFL, pitch, roll));
    legBL.height = legOrientations[1].x * (height + getCornerHeight(cornerBL, pitch, roll));
    legFR.height = legOrientations[2].x * (height + getCornerHeight(cornerFR, pitch, roll));
    legBR.height = legOrientations[3].x * (height + getCornerHeight(cornerBR, pitch, roll));

    // TODO: once i begin testing servos, make the proper baesAgles negative
    legFL.baseAngle = M_PI_2 + getHeightAngle(legFL.height);
    legFR.baseAngle = M_PI_2 + getHeightAngle(legFR.height);
    legBL.baseAngle = M_PI_2 + getHeightAngle(legBL.height);
    legBR.baseAngle = M_PI_2 + getHeightAngle(legBR.height);
}


// TODO: there are 6 transitions which need to be accounted for
// from home to swing
// from home to return
// from swing to return
// from swing to home
// from return to swing
// form return to home
// find a way to account for this, hopefully without edge cases
void setMovementType(moveType type){
    
}


// defined as how much of a step is finished per second
double timestep = 0;
// -100 -> +100
void setSpeed(float speed){
    // timestep is a linear range between -2 and 2, where 2 means switching states twice per second
    // this is probably way too fast, to be tuned later, TODO
    timestep = speed / 50;
}
// on 0 throttle for half a second, return to standstill
// real throttle should lag behind set throttle to prevent sudden shifts

void setHeading(float yawRate);

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




//TODO: repurpose into a generic "RECEIVE" function regardless of receive method
//TODO: finish this
// will integrate with the crsf parser
void receiveSerial(){

    // the current setup allows receiving messages over serial with USB
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

    
    //i2c_init(I2C_PORT, 1000000); //Use i2c port with baud rate of 1Mhz
    
    //gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    //gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    //gpio_pull_up(I2C_SDA);
    //gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c


    //pico_ssd1306::SSD1306 display = pico_ssd1306::SSD1306(I2C_PORT, 0x3C, pico_ssd1306::Size::W128xH64);
    //display.setOrientation(false);

    //setFace(emotion::normal, &display);

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
    setPose(10, 0, 0);

    // code used to center the servos in order to mount them properly
    //legFL.setTheta(M_PI_2);
    //legFL.setPhi(M_PI_2);
    //while(true){sleep_ms(1000);}
    

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);


    // loop used to showcase the faces of the robot
    /*
    int emote =  0;
    while(true){
        setFace(emotion(emote), &display);
        emote++;
        emote%=9;
        sleep_ms(2000);
    }
    */

    

    uint dt = 0;
    uint64_t lastTime = time_us_64();
    while (true) {
        receiveSerial(); //to add: setSpeed, setFace, setPose,

        // the given motion is structured such that positive turns up and negative turns down
        // everything is centered on 90 degrees, center of servo motion
        // this means the only change needed is to turn the offset negative
        // however, base angle does need to be computed differently

        // this sets the servo angles as starting positions + a fraction of the offset needed to reach the target
        // for phi, M_PI_2 is the baseAngle
        for(int i = 0; i < 4; i++){
            legs[i]->setPhi(M_PI_2 + 
                (legOrientations[i].x * 
                    (legs[i]->start.x + 
                        (phaseFloat * 
                            (legs[i]->target.x - legs[i]->start.x)))));


            legs[i]->setTheta(
                legs[i]->baseAngle + 
                    (legOrientations[i].y * 
                        (legs[i]->start.y + 
                            (phaseFloat * 
                                (legs[i]->target.y - legs[i]->start.y)))));
        }

        
        phaseFloat += timestep * (dt / 1000000.0);

        

        // accounts for the program somehow getting stuck
        while(phaseFloat >= 1.0)
            phaseFloat -= 1.0;

        // delta time calculation
        dt = uint(time_us_64() - lastTime);
        lastTime = time_us_64();
    }
}