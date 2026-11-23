#include "simplePID.h"

PID::PID(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd)
{
}

float PID::compute(float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    float P = Kp * error;

    integral += error * dt;
    float I = Ki * integral;

    float derivative = (error - previousError) / dt;
    float D = Kd * derivative;

    previousError = error;

    return P + I + D;
}
void PID::reset()
{
    integral=0;
    previousError=0;
}