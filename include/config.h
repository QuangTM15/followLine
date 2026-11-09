
#ifndef CONFIG_H
#define CONFIG_H

// ================= SENSOR =================
#define NUM_SENSORS 6

const int sensorPins[NUM_SENSORS] = {
    33, 34, 35, 25, 26, 27};

// ================= MOTOR =================
#define LEFT_MOTOR_PWM 18
#define LEFT_MOTOR_DIR1 5
#define LEFT_MOTOR_DIR2 17

#define RIGHT_MOTOR_PWM 19
#define RIGHT_MOTOR_DIR1 16
#define RIGHT_MOTOR_DIR2 4

#define MOTOR_STBY 23

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RES 8
#define PWM_MAX 180

// ================= SPEED & PID =================
#define BASE_SPEED 90
#define PID_KP 35.0f
#define PID_KD 8.0f

#define RECOVERY_POS_THRESHOLD 4.0f
#define RECOVERY_FAST_SPEED 180
#define RECOVERY_SLOW_SPEED 20

// ================= ANALOG =================
#define SENSOR_THRESHOLD 2000
#define SENSOR_MAX 4095
#define SENSOR_ACTIVE_THRESHOLD 0.5f

// ================= HARDWARE =================
#define LED_PIN 2

// ================= MAP =================
#define DIR_LEFT -1
#define DIR_RIGHT 1
#define MAX_STEP 8

const int mapLeft[MAX_STEP] = {
    0, 0,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_RIGHT,
    0};

const int mapRight[MAX_STEP] = {
    0, 0,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_LEFT,
    0};

// ================= MAP STEPS =================
#define STEP_PASS_FIRST 1
#define STEP_DETECT_MAP 2
#define STEP_FOLLOW_MAP 3
#define STEP_BOTTLE_BRANCH 4
#define STEP_RETURN_1 5
#define STEP_RETURN_2 6
#define STEP_FINAL_STOP 7

// ================= INTERSECTION =================
#define DETECT_DELAY 200

#define INTERSECTION_MIN_ACTIVE 4
#define INTERSECTION_MIN_SUM 3.0f
#define INTERSECTION_STABLE_COUNT 3

#define OUT_INTERSECTION_MAX_ACTIVE 2
#define OUT_INTERSECTION_STABLE_COUNT 3

#define INTERSECTION_STOP_DELAY 10

// ================= FORWARD =================
#define STEP1_FORWARD_SPEED BASE_SPEED
#define STEP1_FORWARD_DELAY 10

#define FORWARD_CLEAR_SPEED BASE_SPEED
#define FORWARD_CLEAR_TIMEOUT_MS 1800U

// ================= TURN =================

// turn(): rẽ thông thường
// Tốc độ từng bánh độc lập

#define TURN_LEFT_SPEED 140
#define TURN_RIGHT_SPEED 110

// Tốc độ quay ngược
#define TURN_LEFT_PIVOT_SPEED -140
#define TURN_RIGHT_PIVOT_SPEED -110

#define TURN_LINE_STABLE_COUNT 3

// ================= TURN2 =================

// turn2(): rẽ bằng cách thoát line cũ
// rồi bắt line mới

#define TURN2_LEFT_SPEED 150
#define TURN2_RIGHT_SPEED 130

#define TURN2_LEFT_PIVOT_SPEED -150
#define TURN2_RIGHT_PIVOT_SPEED -130

#define TURN2_LINE_STABLE_COUNT 3

#define TURN_SENSOR_TIMEOUT_MS 2200U

// ================= BOTTLE ALIGN =================
#define BOTTLE_ALIGN_TIME 250U

// ================= BOTTLE APPROACH =================
#define BOTTLE_FORWARD_SPEED 75
#define BOTTLE_FORWARD_KP 10.0f
#define BOTTLE_MAX_CORRECTION 25

#define BOTTLE_NO_LINE_TIME 35U
#define BOTTLE_FORWARD_TIMEOUT 2500U

// ================= BOTTLE STOP =================
#define BOTTLE_STOP_TIME 120U

// ================= BOTTLE BACK =================
#define BOTTLE_BACK_SPEED 75
#define BOTTLE_BACK_LINE_STABLE 3

// Timeout chỉ dùng khi lùi
#define BOTTLE_BACK_FIND_TIMEOUT_MS 3500U

// ================= BOTTLE TURN =================

// Map trái: quay phải
// Map phải: quay trái

// Hai bánh có tốc độ riêng
#define BOTTLE_UTURN_LEFT_SPEED 140
#define BOTTLE_UTURN_RIGHT_SPEED 110

// Quay theo cảm biến, không dùng thời gian
#define BOTTLE_UTURN_CLEAR_STABLE 3
#define BOTTLE_UTURN_LINE_STABLE 3

// ================= RETURN & FINISH =================
#define RETURN_REARM_STABLE_COUNT 3

// Sau bước 7, chạy đến khi mất line
#define FINISH_NO_LINE_MS 50U

// ================= COMPATIBILITY =================

#define BOTTLE_ALIGN_MS BOTTLE_ALIGN_TIME
#define BOTTLE_ALIGN_TIMEOUT_MS 1200U

#define BOTTLE_APPROACH_SPEED BOTTLE_FORWARD_SPEED
#define BOTTLE_APPROACH_KP BOTTLE_FORWARD_KP
#define BOTTLE_APPROACH_MAX_CORR BOTTLE_MAX_CORRECTION
#define BOTTLE_APPROACH_TIMEOUT_MS BOTTLE_FORWARD_TIMEOUT

#define BOTTLE_NO_LINE_MS BOTTLE_NO_LINE_TIME
#define BOTTLE_STOP_MS BOTTLE_STOP_TIME

#define BOTTLE_BACK_TIMEOUT BOTTLE_BACK_FIND_TIMEOUT_MS
#define BOTTLE_BACK_TIMEOUT_MS BOTTLE_BACK_FIND_TIMEOUT_MS

#define BOTTLE_BACK_CENTER_TIME 0U
#define BOTTLE_BACK_CENTER_MS BOTTLE_BACK_CENTER_TIME

#define BOTTLE_BACK_IGNORE_MS 250U
#define BOTTLE_BACK_SETTLE_MS 80U

#define FINAL_BACK_STOP_DELAY INTERSECTION_STOP_DELAY
#define FINAL_FORWARD_DURATION BOTTLE_FORWARD_TIMEOUT
#define FINAL_BACK_SPEED (-BOTTLE_BACK_SPEED)
#define FINAL_BACK_DETECT_DELAY 100U

#endif
