#pragma once

#ifndef _RAWSERVO_H
#define _RAWSERVO_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>

#define PWM_TOP 2499
#define SERVO_TOP 2.5f      //milliseconds
#define SERVO_BOTTOM 0.5f   //milliseconds

// 1/freq to get period, then top / period for levels per ms
const float levelsPerMs = PWM_TOP / (1000.0f / (125000000 / (250.0f * (PWM_TOP + 1))));
const float levelsPerDeg = ((levelsPerMs * SERVO_TOP) - (levelsPerMs * SERVO_BOTTOM)) / 180.0f;
const float levelsPerRad = ((levelsPerMs * SERVO_TOP) - (levelsPerMs * SERVO_BOTTOM)) / M_PI;

void initServo(uint servos[], uint servoCount);
void setAngleDeg(uint servoPin, float angle);
void setAngleRad(uint servoPin, float angle);

#endif