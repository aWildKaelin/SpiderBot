#include "rawServo.h"


void initServo(uint servos[], uint servoCount){

    // I need config because I'm not manually setting up 8 servos across 4 slices
    pwm_config cfg = pwm_get_default_config();

    // this combo does 200Hz
    // Set divider
    pwm_config_set_clkdiv(&cfg, 250.0f);
    // Set TOP value
    pwm_config_set_wrap(&cfg, PWM_TOP);
    //f_pwm = f_sys / (divider * (TOP + 1)), where TOP is the wrap value
    //125000000Hz / (250 * (2499 + 1)) = 200Hz, 5ms period, high refresh rate
    // according to servo spec, this is on the edge of what's allowed
    // I get 500 steps per ms, which means that 1 - 2ms is 500 - 1000 levels
    // most servos have a +- 0.5ms tolerance, which means 0.5ms to 2.5ms


    bool sliceInitialized[8] = {false};

    for(int i = 0; i < servoCount; i++){
        //printf("setting up pin %i\n", servos[i]);
        gpio_set_function(servos[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(servos[i]);

        if(!sliceInitialized[slice]){
            sliceInitialized[slice] = true;
            pwm_init(slice, &cfg, true);
        }
    }
}


void setAngleDeg(uint servoPin, float angle){
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;

    pwm_set_gpio_level(servoPin, SERVO_BOTTOM + (angle * levelsPerDeg));
}

void setAngleRad(uint servoPin, float angle){
    if(angle < 0) angle = 0;
    if(angle > M_PI) angle = M_PI;

    pwm_set_gpio_level(servoPin, SERVO_BOTTOM + (angle * levelsPerRad));
}