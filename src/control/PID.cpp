#include "PID.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float minOut, float maxOut)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _previousError(0),
      _minOut(minOut), _maxOut(maxOut) {}

float PID::compute(float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Calculate and constrain the Integral (Prevents Windup)
    _integral += error * dt;
    _integral = constrain(_integral, _minOut, _maxOut);

    // SAFE Derivative calculation
    float derivative = 0;
    if (dt > 0.0001f) { // Only calculate if time has actually passed
        derivative = (error - _previousError) / dt;
    }

    // Simple Low Pass Filter (adjust 0.7 to 0.9 for more filtering)
    float filterFactor = 0.8f;
    derivative = (_lastDerivative * filterFactor) + (derivative * (1.0f - filterFactor));
    _lastDerivative = derivative;

    // Calculate Final Output
    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
    output = constrain(output, _minOut, _maxOut);

    _previousError = error;

    return output;
}

void PID::reset()
{
    _integral = 0;
    _previousError = 0;
}