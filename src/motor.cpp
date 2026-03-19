#include <Arduino.h>
#include "config.h"
#include "motor.h"

// initialize motor pins and PWM
void motorInit()
{
    // setup PWM channels
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcSetup(1, PWM_FREQ, PWM_RES);

    // attach PWM pins
    ledcAttachPin(LEFT_MOTOR_PWM, 0);
    ledcAttachPin(RIGHT_MOTOR_PWM, 1);

    // setup direction pins
    pinMode(LEFT_MOTOR_DIR1, OUTPUT);
    pinMode(LEFT_MOTOR_DIR2, OUTPUT);

    pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

    stopMotors();
}

// set motor PWM speed
void setMotorSpeed(int left, int right)
{
    left  = constrain(left, 0, PWM_MAX);
    right = constrain(right, 0, PWM_MAX);

    ledcWrite(0, left);
    ledcWrite(1, right);
}

// set motor direction
void setMotorDirection(int leftDir, int rightDir)
{
    // LEFT MOTOR
    if(leftDir == 1)
    {
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    }
    else if(leftDir == -1)
    {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    }
    else
    {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    }

    // RIGHT MOTOR
    if(rightDir == 1)
    {
        digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    }
    else if(rightDir == -1)
    {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    }
    else
    {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    }
}

// stop both motors
void stopMotors()
{
    ledcWrite(0, 0);
    ledcWrite(1, 0);

    digitalWrite(LEFT_MOTOR_DIR1, LOW);
    digitalWrite(LEFT_MOTOR_DIR2, LOW);

    digitalWrite(RIGHT_MOTOR_DIR1, LOW);
    digitalWrite(RIGHT_MOTOR_DIR2, LOW);
}