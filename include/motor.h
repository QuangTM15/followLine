#ifndef MOTOR_H
#define MOTOR_H

// initialize motors
void motorInit();

// set motor speed (0 → 255)
void setMotorSpeed(int leftSpeed, int rightSpeed);

// set motor direction
// 1 = forward
// -1 = reverse
// 0 = stop
void setMotorDirection(int leftDir, int rightDir);

// stop both motors
void stopMotors();

void setMotorRaw(int left, int right);

#endif
