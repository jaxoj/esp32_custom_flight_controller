#pragma once

class PID {
public:
    PID(float kp, float ki, float kd, float minOut, float maxOut);

    float compute(float setpoint, float measurement, float dt);

    void reset();

private:
    float _kp, _ki, _kd;
    float _integral;
    float _previousError;
    float _lastDerivative = 0;
    float _minOut, _maxOut;
};