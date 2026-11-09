#include <Arduino.h>
#include "config.h"
#include "map.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

// ================= MAP =================
static const int* currentMap = NULL;

// ================= STATE =================
enum RobotState
{
    FOLLOW_LINE,
    FINAL_FORWARD,
    FINAL_BACK,
    FINAL_TURN,
    FINAL_STOP
};

static RobotState state = FOLLOW_LINE;

enum MapSide
{
    UNKNOWN,
    LEFT_MAP,
    RIGHT_MAP
};

static MapSide mapSide = UNKNOWN;

// ================= VARIABLES =================
static unsigned long lastDetectTime = 0;
static uint8_t cnt = 0;
static int intersectionCount = 0;

// =====================================================
// DETECT INTERSECTION
// =====================================================
static bool detectIntersection()
{
    int active = 0;
    float sum = 0.0f;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorNormalized[i] > SENSOR_ACTIVE_THRESHOLD)
            active++;
        sum += sensorNormalized[i];
    }

    return (active >= INTERSECTION_MIN_ACTIVE && sum > INTERSECTION_MIN_SUM);
}

static bool detectStableIntersection()
{
    if (detectIntersection())
    {
        if (intersectionCount < INTERSECTION_STABLE_COUNT)
            intersectionCount++;
    }
    else if (intersectionCount > 0)
    {
        intersectionCount--;
    }

    if (intersectionCount >= INTERSECTION_STABLE_COUNT)
    {
        intersectionCount = 0;
        return true;
    }

    return false;
}

static bool isOutOfIntersection()
{
    int active = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorNormalized[i] > SENSOR_ACTIVE_THRESHOLD)
            active++;
    }

    // <= OUT_INTERSECTION_MAX_ACTIVE mắt sáng → đã ra khỏi ngã
    return (active <= OUT_INTERSECTION_MAX_ACTIVE);
}

static bool isNoLine()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorNormalized[i] > SENSOR_ACTIVE_THRESHOLD)
            return false;
    }
    return true;
}

static void forwardUntilNoLine()
{
    int stable = 0;

    while (true)
    {
        readLineSensors();

        setMotorRaw(FORWARD_CLEAR_SPEED, FORWARD_CLEAR_SPEED);

        if (isNoLine())
            stable++;
        else if (stable > 0)
            stable--;

        if (stable >= OUT_INTERSECTION_STABLE_COUNT)
            break;
    }

    stopMotors();
}

static void forwardUntilClear()
{
    int stable = 0;

    while (true)
    {
        readLineSensors();

        setMotorRaw(FORWARD_CLEAR_SPEED, FORWARD_CLEAR_SPEED); // đi thẳng nhẹ

        if (isOutOfIntersection())
            stable++;
        else if (stable > 0)
            stable--;

        if (stable >= OUT_INTERSECTION_STABLE_COUNT)
            break;
    }
    // // kick nhẹ để ổn định
    // setMotorRaw(160, 160);
    // delay(30);
}
// =====================================================
// READ TURN DIRECTION
// =====================================================
static int readTurnDirection()
{
    float s0 = sensorNormalized[0];
    float s1 = sensorNormalized[1];
    float s4 = sensorNormalized[4];
    float s5 = sensorNormalized[5];

    float left  = s4 + s5;
    float right = s0 + s1;

    return (left > right) ? DIR_LEFT : DIR_RIGHT;
}

// =====================================================
// TURN
// =====================================================

static void turn2(int dir)
{
    // ===== 1. BẮT ĐẦU QUAY =====
    if (dir == DIR_RIGHT)
        setMotorRaw(TURN2_SPEED, TURN2_PIVOT_SPEED);
    else
        setMotorRaw(TURN2_PIVOT_SPEED, TURN2_SPEED);

    // ===== 2. CHỜ THOÁT KHỎI LINE HIỆN TẠI (RA VÙNG TRẮNG) =====
    int clearCount = 0;

    while (true)
    {
        readLineSensors();

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (s2 <= SENSOR_ACTIVE_THRESHOLD && s3 <= SENSOR_ACTIVE_THRESHOLD)
            clearCount++;
        else if (clearCount > 0)
            clearCount--;

        if (clearCount >= TURN2_LINE_STABLE_COUNT)
            break;
    }

    // ===== 3. CHỜ BẮT VÀO ĐƯỜNG THẲNG MỚI =====
    int stableCount = 0;

    while (true)
    {
        readLineSensors();

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (s2 > SENSOR_ACTIVE_THRESHOLD && s3 > SENSOR_ACTIVE_THRESHOLD)
            stableCount++;
        else if (stableCount > 0)
            stableCount--;

        if (stableCount >= TURN2_LINE_STABLE_COUNT)
            break;
    }

    stopMotors();
}

static void turn(int dir)
{
    // ===== 1. QUAY =====
    if (dir == DIR_RIGHT)
        setMotorRaw(TURN_RIGHT_SPEED, TURN_PIVOT_SPEED);
    else
        setMotorRaw(TURN_PIVOT_SPEED, TURN_LEFT_SPEED);

    // ===== 2. BẮT LINE MỚI =====
    int stableCount = 0;

    while (true)
    {
        readLineSensors();

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (s2 > SENSOR_ACTIVE_THRESHOLD && s3 > SENSOR_ACTIVE_THRESHOLD)
            stableCount++;
        else if (stableCount > 0)
            stableCount--;

        if (stableCount >= TURN_LINE_STABLE_COUNT)
            break;
    }
}

// =====================================================
// INIT
// =====================================================
void mapInit()
{
    state = FOLLOW_LINE;
    mapSide = UNKNOWN;
    currentMap = NULL;
    lastDetectTime = 0;
    cnt = 0;
    intersectionCount = 0;

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

bool isMapFinished()
{
    return (state == FINAL_STOP);
}

// =====================================================
// MAIN UPDATE
// =====================================================
void mapUpdate()
{
    switch (state)
    {
        // =====================================================
        // FOLLOW LINE
        // =====================================================
        case FOLLOW_LINE:
        {
            if (millis() - lastDetectTime > DETECT_DELAY && detectStableIntersection())
            {
                cnt++;
                digitalWrite(LED_PIN, HIGH);

                // Dừng lại khoảng 1s khi nhận diện được ngã tư rồi mới đi tiếp
                stopMotors();
                delay(INTERSECTION_STOP_DELAY);

                // ===== CASE 1: đi thẳng =====
                if (cnt == STEP_PASS_FIRST)
                {
                    setMotorRaw(STEP1_FORWARD_SPEED, STEP1_FORWARD_SPEED);
                    delay(STEP1_FORWARD_DELAY);
                }

                // ===== CASE 2: xác định map =====
                else if (cnt == STEP_DETECT_MAP)
                {
                    int turnDir = readTurnDirection();

                    if (turnDir == DIR_LEFT)
                    {
                        mapSide = LEFT_MAP;
                        currentMap = mapLeft;
                    }
                    else
                    {
                        mapSide = RIGHT_MAP;
                        currentMap = mapRight;
                    }

                    forwardUntilNoLine();
                    turn(turnDir);
                }

                // ===== CASE 3: đi theo map =====
                else if (cnt == STEP_FOLLOW_MAP)
                {
                    forwardUntilNoLine();
                    if (currentMap != NULL)
                    {
                        int dir = currentMap[cnt];
                        turn(dir);
                    }
                }

                // ===== CASE 4: rẽ vào nhánh đâm tới chai =====
                else if (cnt == STEP_BOTTLE_BRANCH)
                {
                    if (currentMap != NULL)
                    {
                        int dir = currentMap[cnt];
                        turn2(dir);
                    }
                    lastDetectTime = millis();
                    state = FINAL_FORWARD;
                    return;
                }

                // ===== CASE 5 + 6: rẽ theo map về đích =====
                else if (cnt == STEP_RETURN_1 || cnt == STEP_RETURN_2)
                {
                    if (currentMap != NULL)
                    {
                        int dir = currentMap[cnt];
                        turn2(dir);
                    }
                }

                // ===== CASE 7: đến đích cuối cùng -> dừng hẳn =====
                else if (cnt >= STEP_FINAL_STOP)
                {
                    stopMotors();
                    digitalWrite(LED_PIN, LOW);
                    state = FINAL_STOP;
                    return;
                }

                lastDetectTime = millis();
                return;
            }
            else
            {
                digitalWrite(LED_PIN, LOW);
            }
            updateLineFollowing();
            break;
        }

        // =====================================================
        // ĐÂM TỚI CHAI
        // =====================================================
        case FINAL_FORWARD:
        {
            static unsigned long startTime = 0;

            if (startTime == 0)
                startTime = millis();

            updateLineFollowing();

            if (millis() - startTime > FINAL_FORWARD_DURATION)
            { 
                startTime = 0;
                intersectionCount = 0;
                state = FINAL_BACK;
            }
            break;
        }

        // =====================================================
        // LÙI VỀ NGÃ
        // =====================================================
        case FINAL_BACK:
        {
            static unsigned long lastDetect = 0;

            setMotorRaw(FINAL_BACK_SPEED, FINAL_BACK_SPEED);

            bool hasIntersection = detectStableIntersection();

            if (hasIntersection && millis() - lastDetect > FINAL_BACK_DETECT_DELAY)
            {
                lastDetect = millis();

                stopMotors();
                delay(FINAL_BACK_STOP_DELAY);

                state = FINAL_TURN;
            }

            break;
        }

        // =====================================================
        // QUAY NGƯỢC MAP
        // =====================================================
        case FINAL_TURN:
        {
            int dir;

            if (mapSide == LEFT_MAP)
                dir = DIR_RIGHT;
            else
                dir = DIR_LEFT;

            turn2(dir);

            lastDetectTime = millis();
            intersectionCount = 0;
            state = FOLLOW_LINE;
            break;
        }

        // =====================================================
        // ĐÍCH CUỐI CÙNG (DỪNG HẲN)
        // =====================================================
        case FINAL_STOP:
        {
            stopMotors();
            break;
        }

        default:
            break;
    }
}
