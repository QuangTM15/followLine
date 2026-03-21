#include <Arduino.h>
#include "config.h"
#include "motor.h"

// current speed (để smooth)
static int currentLeft = 0;
static int currentRight = 0;

// tốc độ thay đổi mỗi lần (giảm giật)
#define ACC_STEP 10

// =====================================================
// INIT
// =====================================================
void motorInit()
{
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcSetup(1, PWM_FREQ, PWM_RES);

    ledcAttachPin(LEFT_MOTOR_PWM, 0);
    ledcAttachPin(RIGHT_MOTOR_PWM, 1);

    pinMode(LEFT_MOTOR_DIR1, OUTPUT);
    pinMode(LEFT_MOTOR_DIR2, OUTPUT);

    pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

    stopMotors();
}

// =====================================================
// SET DIRECTION + SPEED (gộp lại)
// =====================================================
void setMotorSpeed(int left, int right)
{
    // smooth tăng giảm tốc
    if (left > currentLeft) currentLeft += ACC_STEP;
    else if (left < currentLeft) currentLeft -= ACC_STEP;

    if (right > currentRight) currentRight += ACC_STEP;
    else if (right < currentRight) currentRight -= ACC_STEP;

    // constrain
    currentLeft  = constrain(currentLeft, -PWM_MAX, PWM_MAX);
    currentRight = constrain(currentRight, -PWM_MAX, PWM_MAX);

    // ===== LEFT MOTOR =====
    if (currentLeft > 0)
    {
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
        ledcWrite(0, currentLeft);
    }
    else if (currentLeft < 0)
    {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
        ledcWrite(0, -currentLeft);
    }
    else
    {
        ledcWrite(0, 0);
    }

    // ===== RIGHT MOTOR =====
    if (currentRight > 0)
    {
        digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
        ledcWrite(1, currentRight);
    }
    else if (currentRight < 0)
    {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
        ledcWrite(1, -currentRight);
    }
    else
    {
        ledcWrite(1, 0);
    }
}

// =====================================================
// STOP
// =====================================================
void stopMotors()
{
    currentLeft = 0;
    currentRight = 0;

    ledcWrite(0, 0);
    ledcWrite(1, 0);

    digitalWrite(LEFT_MOTOR_DIR1, LOW);
    digitalWrite(LEFT_MOTOR_DIR2, LOW);

    digitalWrite(RIGHT_MOTOR_DIR1, LOW);
    digitalWrite(RIGHT_MOTOR_DIR2, LOW);
}

void setMotorRaw(int left, int right)
{
    // LEFT
    if (left > 0)
    {
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
        ledcWrite(0, left);
    }
    else if (left < 0)
    {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
        ledcWrite(0, -left);
    }
    else ledcWrite(0, 0);

    // RIGHT
    if (right > 0)
    {
        digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
        ledcWrite(1, right);
    }
    else if (right < 0)
    {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
        ledcWrite(1, -right);
    }
    else ledcWrite(1, 0);
}
