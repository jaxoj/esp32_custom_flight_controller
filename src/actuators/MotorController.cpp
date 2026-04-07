#include "MotorController.h"
#include "../config.h"
#include <Arduino.h>

void MotorController::begin()
{
    esc1.attach(ESC_FL, PWM_MIN, PWM_MAX);
    esc2.attach(ESC_FR, PWM_MIN, PWM_MAX);
    esc3.attach(ESC_BL, PWM_MIN, PWM_MAX);
    esc4.attach(ESC_BR, PWM_MIN, PWM_MAX);
}

void MotorController::arm()
{
    esc1.writeMicroseconds(PWM_MIN);
    esc2.writeMicroseconds(PWM_MIN);
    esc3.writeMicroseconds(PWM_MIN);
    esc4.writeMicroseconds(PWM_MIN);
    delay(5000);
}

void MotorController::update(int baseThrottle, float roll, float pitch, float yaw)
{
    int m1 = baseThrottle + roll - pitch + yaw;
    int m2 = baseThrottle - roll - pitch - yaw;
    int m3 = baseThrottle + roll + pitch - yaw;
    int m4 = baseThrottle - roll + pitch + yaw;

    m1 = constrain(m1, PWM_MIN, PWM_MAX);
    m2 = constrain(m2, PWM_MIN, PWM_MAX);
    m3 = constrain(m3, PWM_MIN, PWM_MAX);
    m4 = constrain(m4, PWM_MIN, PWM_MAX);

    esc1.writeMicroseconds(m1);
    esc2.writeMicroseconds(m2);
    esc3.writeMicroseconds(m3);
    esc4.writeMicroseconds(m4);
}