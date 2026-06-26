/*
Copyright (C) 2026 aWildKaelin

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

float max(float a, float b){
    return a > b ? a : b;
}
float min(float a, float b){
    return a < b ? a : b;
}



typedef struct vec3{
    float x, y, z;
}vec3;

typedef struct pair{
    float first, second;
}pair;

enum struct moveType{
    trot, walk, run, still
};

moveType robotState = moveType::trot;

enum struct moveState{
    home = 0, up = 1, swingF, down, swingB
};

struct moveStruct{
    moveState activeState;
    uint8_t waitTime = 0;
};

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
        pose.first = angle;
    };
    void setTheta(float angle){
        setAngleRad(thetaServo, angle);
        pose.second = angle;
    };

    moveStruct state = {moveState::home};

    // stores its own height for reference and the servo angle that represents 0
    // baseAngle is radians and only applies to height, not stride
    float height = 0, baseAngle = M_PI/2;

    // x/y, phi/theta
    pair pose = {0, 0};
    pair start = {0, 0}, target = {0, 0};
}leg;



#define BOT_WIDTH 80    // mm
#define BOT_LENGTH 130

// THIS IS NOT ENOUGH, its not fully accurate, but its good for testing
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

pair legOrientations[] ={
    pair{-1, 1},
    pair{-1, 1},
    pair{1, -1},
    pair{1, -1}
};


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

// first value is the stride length in radians, 2nd is height offset
pair moveStorage[5] = {{0, 0}, {-1, 40}, {1, 40}, {1, 0}, {-1, 0}};


pair computeNextServoTarget(moveState state, float multiplier = 1){
    if(state == moveState::home){
        return {0, 0.1674480115553};
    }

    int next = ((int)state + 1) % 5;
    if(next == 0) next = 1;
    if(state == moveState::swingB || state == moveState::swingF)
        return {moveStorage[next].first * multiplier, getHeightAngle(moveStorage[next].second)};
    else
        return {moveStorage[next].first, getHeightAngle(moveStorage[next].second)};
}


float chassisPitch = 0, chassisRoll = 0;

// extracts the corner height out of Pitch x Roll x pos
// assumes Z always starts at 0 AND corners are const
float getCornerHeight(vec3 corner, float pitch, float roll){
    float result =  -corner.x * sin(pitch) + 
                    (corner.y * sin(roll) * cos(pitch));

    return max(min(result, ARM_LENGTH - 15), -ARM_LENGTH + 5);
}


// mm, deg, deg
void setPose(float height, float pitch, float roll){
    legFL.height = legOrientations[0].first * (height + getCornerHeight(cornerFL, pitch, roll));
    legBL.height = legOrientations[1].first * (height + getCornerHeight(cornerBL, pitch, roll));
    legFR.height = legOrientations[2].first * (height + getCornerHeight(cornerFR, pitch, roll));
    legBR.height = legOrientations[3].first * (height + getCornerHeight(cornerBR, pitch, roll));

    // TODO: once i begin testing servos, make the proper baesAgles negative
    legFL.baseAngle = M_PI_2 + getHeightAngle(legFL.height);
    legFR.baseAngle = M_PI_2 + getHeightAngle(legFR.height);
    legBL.baseAngle = M_PI_2 + getHeightAngle(legBL.height);
    legBR.baseAngle = M_PI_2 + getHeightAngle(legBR.height);
}


// should reset pose to home and then set phase
void setMovementType(moveType type){
    if(robotState == moveType::still){ // if starting from standstill

    }
    else if(type == moveType::still){   // if going to standstill
        
    }
    else{

    }
}


// defined as how much of a step is finished per second
double timestep = 0;
// -100 -> +100
void setSpeed(float speed){
    // timestep is a linear range between -2 and 2, where 2 means switching states twice per second
    // this is probably way too fast, to be tuned later
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
void receiveSerial(){
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
    
    setSpeed(50);
    setPose(10, 0, 0);
    //legFL.setTheta(M_PI_2);
    //legFL.setPhi(M_PI_2);
    //while(true){sleep_ms(1000);}
    

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);


    /*
    setFace(emotion(emote), &display);
    emote++;
    emote%=9;
    sleep_ms(2000);
    */


    double TheFloat = 0;

    int emote =  0;

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
            legs[i]->setPhi(M_PI_2 + (legOrientations[i].first * (legs[i]->start.first + (TheFloat * (legs[i]->target.first - legs[i]->start.first)))));
            legs[i]->setTheta(legs[i]->baseAngle + (legOrientations[i].second * (legs[i]->start.second + (TheFloat * (legs[i]->target.second - legs[i]->start.second)))));
        }

        
        TheFloat += timestep * (dt / 1000000.0);
        printf("%i %-10f %-10f\n", (int)legFL.state.activeState, TheFloat, legFL.start.first + (TheFloat * (legFL.target.first - legFL.start.first)));

        // phasing system
        if(TheFloat >= 1 && robotState != moveType::still){ // if paused, don't advance state
            // advance to next step

            for(int i = 0; i < 4; i++){
                if(legs[i]->state.waitTime == 0){
                    legs[i]->start = legs[i]->target;
                    legs[i]->target = computeNextServoTarget(legs[i]->state.activeState);
                    // TODO: based on yaw, calculate a multiplier per leg
                    // YAW goes from -1 to 1, where -1 means turning left at max speed and 1 means turning right at max speed
                    // this can be added to a list in sync with legs[i] because each leg has a hardcoded position
                    int next = ((int)legs[i]->state.activeState + 1) % 5;
                    if(next == 0) next = 1;
                    legs[i]->state.activeState = moveState(next);
                }
            }
        }
        while(TheFloat >= 1.0)
            TheFloat -= 1.0;

        dt = uint(time_us_64() - lastTime);
        lastTime = time_us_64();
    }
}