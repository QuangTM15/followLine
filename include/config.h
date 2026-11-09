#ifndef CONFIG_H
#define CONFIG_H

// ================= SENSOR =================
#define NUM_SENSORS 6

// Analog pins (sensor)
const int sensorPins[NUM_SENSORS] = {33, 34, 35, 25, 26, 27};

// ================= MOTOR =================
// Left motor
#define LEFT_MOTOR_PWM 18
#define LEFT_MOTOR_DIR1 5
#define LEFT_MOTOR_DIR2 17

#define MOTOR_STBY 23

// Right motor
#define RIGHT_MOTOR_PWM 19
#define RIGHT_MOTOR_DIR1 16
#define RIGHT_MOTOR_DIR2 4

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RES 8
#define PWM_MAX 180

// ================= SPEED & PID =================
#define BASE_SPEED              90
#define PID_KP                  35.0f
#define PID_KD                  8.0f

// Recovery khi lệch khỏi line nhiều
#define RECOVERY_POS_THRESHOLD  4.0f
#define RECOVERY_FAST_SPEED     180
#define RECOVERY_SLOW_SPEED     20

// ================= ANALOG =================
#define SENSOR_THRESHOLD 2000
#define SENSOR_MAX 4095
#define SENSOR_ACTIVE_THRESHOLD 0.5f

// ================= HARDWARE =================
#define LED_PIN 2

// ================= MAP & NAVIGATION =================
#define DIR_LEFT  -1
#define DIR_RIGHT  1
#define MAX_STEP   8

const int mapLeft[MAX_STEP]  = {0, 0, DIR_LEFT,  DIR_RIGHT, DIR_RIGHT, DIR_LEFT,  DIR_RIGHT, 0};
const int mapRight[MAX_STEP] = {0, 0, DIR_RIGHT, DIR_LEFT,  DIR_LEFT,  DIR_RIGHT, DIR_LEFT,  0};

// Các mốc ngã tư (step) trên sa bàn
#define STEP_PASS_FIRST     1
#define STEP_DETECT_MAP     2
#define STEP_FOLLOW_MAP     3
#define STEP_BOTTLE_BRANCH  4
#define STEP_RETURN_1       5
#define STEP_RETURN_2       6
#define STEP_FINAL_STOP     7

// ================= INTERSECTION DETECTION =================
#define DETECT_DELAY                  200
#define INTERSECTION_MIN_ACTIVE       4
#define INTERSECTION_MIN_SUM          3.0f
#define INTERSECTION_STABLE_COUNT     3
#define OUT_INTERSECTION_MAX_ACTIVE   2
#define OUT_INTERSECTION_STABLE_COUNT 3

// ================= MAP SPEEDS & DELAYS =================
// Xử lý khi gặp ngã tư
#define INTERSECTION_STOP_DELAY   10

// Đi thẳng qua ngã đầu tiên (cnt == 1)
#define STEP1_FORWARD_SPEED       BASE_SPEED
#define STEP1_FORWARD_DELAY       10

// Đi thẳng thoát khỏi ngã tư (forwardUntilClear)
#define FORWARD_CLEAR_SPEED       BASE_SPEED

// Hàm rẽ dò line (turn)
#define TURN_LEFT_SPEED           110
#define TURN_RIGHT_SPEED          110
#define TURN_PIVOT_SPEED          -110
#define TURN_LINE_STABLE_COUNT    3

// Hàm rẽ vượt qua line hiện tại sang nhánh mới (turn2)
#define TURN2_SPEED               130
#define TURN2_PIVOT_SPEED        -130
#define TURN2_LINE_STABLE_COUNT   3

// Nhánh đâm tới chai & lùi về (FINAL_FORWARD & FINAL_BACK)
#define FINAL_FORWARD_DURATION    1500
#define FINAL_BACK_SPEED         -BASE_SPEED
#define FINAL_BACK_DETECT_DELAY   100
#define FINAL_BACK_STOP_DELAY     INTERSECTION_STOP_DELAY

#endif
