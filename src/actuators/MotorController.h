#pragma once
#include <ESP32Servo.h>

class MotorController {
public:
    void begin();
    void arm();

    void update(int baseThrottle, float roll, float pitch, float yaw);

private:
    Servo esc1, esc2, esc3, esc4;
};