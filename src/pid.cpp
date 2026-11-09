#include <Arduino.h>
#include "sensor.h"
#include "motor.h"
#include "config.h"

static float lastError = 0;

void pidInit()
{
    lastError = 0;
}

// =====================================================
// MAIN CONTROL
// =====================================================
void updateLineFollowing()
{
    float position = calculateLinePosition();

    // ===== RECOVERY =====
    if (abs(position) > RECOVERY_POS_THRESHOLD)
    {
        if (position > 0)
            setMotorSpeed(RECOVERY_SLOW_SPEED, RECOVERY_FAST_SPEED);
        else
            setMotorSpeed(RECOVERY_FAST_SPEED, RECOVERY_SLOW_SPEED);

        return;
    }

    float error = position;

    float derivative = error - lastError;
    float correction = PID_KP * error + PID_KD * derivative;

    lastError = error;

    int left  = BASE_SPEED - correction;
    int right = BASE_SPEED + correction;

    setMotorSpeed(left, right);
}
