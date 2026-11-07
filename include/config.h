#ifndef CONFIG_H
#define CONFIG_H

// ================= SENSOR =================
#define NUM_SENSORS 6

// Analog pins (sensor)
const int sensorPins[NUM_SENSORS] = {33, 34, 35, 25, 26, 27};

// ================= MOTOR =================
// Left motor
#define LEFT_MOTOR_PWM 18
#define LEFT_MOTOR_DIR1 17
#define LEFT_MOTOR_DIR2 5

#define MOTOR_STBY 23

// Right motor
#define RIGHT_MOTOR_PWM 19
#define RIGHT_MOTOR_DIR1 16
#define RIGHT_MOTOR_DIR2 4

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RES 8
#define PWM_MAX 180

// ================= SPEED =================
#define BASE_SPEED 95

// ================= ANALOG =================
#define SENSOR_THRESHOLD 2000
#define SENSOR_MAX 4095

#endif
