#include "PID.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float minOut, float maxOut)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _previousError(0),
      _minOut(minOut), _maxOut(maxOut) {}

float PID::compute(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    _integral += error * dt;
    _integral = constrain(_integral, _minOut, _maxOut);

    float derivative = (error - _previousError) / dt;

    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
    output = constrain(output, _minOut, _maxOut);

    _previousError = error;
    return output;
}

void PID::reset() {
    _integral = 0;
    _previousError = 0;
}