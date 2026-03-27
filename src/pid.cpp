#include <Arduino.h>
#include "sensor.h"
#include "motor.h"
#include "config.h"

// ===== PID =====
static float Kp = 35.0;
static float Kd = 8.0;

static float lastError = 0;

#define MIN_RUNNING_SPEED 120

void pidInit()
{
    lastError = 0;
}

// =====================================================
// APPLY MIN SPEED (chuẩn)
// =====================================================
static int applyMinRunningSpeed(int speed)
{
    if (speed > 0 && speed < MIN_RUNNING_SPEED) return 0;
    if (speed < 0 && speed > -MIN_RUNNING_SPEED) return 0;
    return speed;
}

// =====================================================
// MAIN CONTROL
// =====================================================
void updateLineFollowing()
{
    float position = calculateLinePosition();

    // ===== RECOVERY =====//
    if (abs(position) > 4)
    {
        int fast = 170;
        int slow = 110;

        if (position > 0)
            setMotorSpeed(slow, fast);
        else
            setMotorSpeed(fast, slow);

        return;
    }

    float error = position;

    float derivative = error - lastError;
    float correction = Kp * error + Kd * derivative;

    lastError = error;

    // ===== LIMIT correction =====
    correction = constrain(correction, -150, 150);

    // ===== DYNAMIC SPEED =====
    int baseSpeed = BASE_SPEED - abs(error) * 10;
    baseSpeed = constrain(baseSpeed, 170, BASE_SPEED);

    int left  = baseSpeed - correction;
    int right = baseSpeed + correction;

    left  = constrain(left, -PWM_MAX, PWM_MAX);
    right = constrain(right, -PWM_MAX, PWM_MAX);

    left  = applyMinRunningSpeed(left);
    right = applyMinRunningSpeed(right);

    setMotorSpeed(left, right);
}
