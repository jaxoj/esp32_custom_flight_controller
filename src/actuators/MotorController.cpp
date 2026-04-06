#include "MotorController.h"
#include "../config.h"
#include <Arduino.h>

void MotorController::begin()
{
    esc1.attach(ESC_FL, 1000, 2000);
    esc2.attach(ESC_FR, 1000, 2000);
    esc3.attach(ESC_BL, 1000, 2000);
    esc4.attach(ESC_BR, 1000, 2000);
}

void MotorController::arm()
{
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    delay(3000);
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

    Serial.printf(
        "Roll:%7.2f  Pitch:%7.2f | M1:%4d M2:%4d M3:%4d M4:%4d\n",
        roll, pitch, m1, m2, m3, m4);
}