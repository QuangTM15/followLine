#include <Arduino.h>
#include "pid.h"

// PID parameters
float Kp = 10;
float Ki = 0.0;
float Kd = 0.0;

// PID state
float prevError = 0;
float integral = 0;

// compute PID correction
float updatePID(float error)
{
    // proportional
    float P = error;

    // integral
    integral += error;

    // derivative
    float D = error - prevError;

    prevError = error;

    float output =
        Kp * P +
        Ki * integral +
        Kd * D;

    return output;
}

// reset PID values
void resetPID()
{
    prevError = 0;
    integral = 0;
}
