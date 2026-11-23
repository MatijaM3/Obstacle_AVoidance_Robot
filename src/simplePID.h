#pragma once
class PID
{
private:
    float Kp;
    float Ki;
    float Kd;

    float integral = 0.0f;
    float previousError = 0.0f;

public:
    PID(float kp, float ki, float kd);

    float compute(float setpoint, float measured, float dt);
    void reset();
};