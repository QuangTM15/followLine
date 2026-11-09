
#include <Arduino.h>
#include "config.h"
#include "map.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

// ================= MAP =================
static const int *currentMap = NULL;

// ================= STATE =================
enum RobotState
{
    FOLLOW_LINE,
    BOTTLE_ALIGN,
    BOTTLE_APPROACH,
    BOTTLE_STOP,
    BOTTLE_BACK,
    BOTTLE_TURN_CLEAR,
    BOTTLE_TURN_FIND,
    FINISH_FOLLOW_LINE,
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

static unsigned long bottleStartTime = 0;
static unsigned long noLineStartTime = 0;
static unsigned long finishNoLineStartTime = 0;

static int bottleBackLineCount = 0;
static int bottleTurnClearCount = 0;
static int bottleTurnLineCount = 0;

static bool returnIntersectionArmed = true;
static int returnArmCount = 0;

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
        {
            active++;
        }

        sum += sensorNormalized[i];
    }

    return (
        active >= INTERSECTION_MIN_ACTIVE &&
        sum > INTERSECTION_MIN_SUM);
}

static bool detectStableIntersection()
{
    if (detectIntersection())
    {
        if (intersectionCount < INTERSECTION_STABLE_COUNT)
        {
            intersectionCount++;
        }
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

// =====================================================
// SENSOR HELPERS
// =====================================================
static bool isOutOfIntersection()
{
    int active = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorNormalized[i] > SENSOR_ACTIVE_THRESHOLD)
        {
            active++;
        }
    }

    return (active <= OUT_INTERSECTION_MAX_ACTIVE);
}

static bool isNoLine()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorNormalized[i] > SENSOR_ACTIVE_THRESHOLD)
        {
            return false;
        }
    }

    return true;
}

static bool isCenterOnLine()
{
    return (
        sensorNormalized[2] > SENSOR_ACTIVE_THRESHOLD &&
        sensorNormalized[3] > SENSOR_ACTIVE_THRESHOLD);
}

// =====================================================
// FORWARD UNTIL NO LINE
// =====================================================
static void forwardUntilNoLine()
{
    int stable = 0;

    while (true)
    {
        readLineSensors();

        setMotorRaw(
            FORWARD_CLEAR_SPEED,
            FORWARD_CLEAR_SPEED);

        if (isNoLine())
        {
            stable++;
        }
        else if (stable > 0)
        {
            stable--;
        }

        if (stable >= OUT_INTERSECTION_STABLE_COUNT)
        {
            break;
        }
    }

    stopMotors();
}

// =====================================================
// FORWARD UNTIL CLEAR
// =====================================================
static void forwardUntilClear()
{
    int stable = 0;

    while (true)
    {
        readLineSensors();

        setMotorRaw(
            FORWARD_CLEAR_SPEED,
            FORWARD_CLEAR_SPEED);

        if (isOutOfIntersection())
        {
            stable++;
        }
        else if (stable > 0)
        {
            stable--;
        }

        if (stable >= OUT_INTERSECTION_STABLE_COUNT)
        {
            break;
        }
    }
}

// =====================================================
// READ TURN DIRECTION
// =====================================================
static int readTurnDirection()
{
    float left =
        sensorNormalized[4] +
        sensorNormalized[5];

    float right =
        sensorNormalized[0] +
        sensorNormalized[1];

    return (left > right) ? DIR_LEFT : DIR_RIGHT;
}

// =====================================================
// TURN2 - SENSOR BASED TURN
// =====================================================
static void turn2(int dir)
{
    // Hai banh co toc do doc lap
    if (dir == DIR_RIGHT)
    {
        setMotorRaw(
            TURN2_LEFT_SPEED,
            TURN2_RIGHT_PIVOT_SPEED);
    }
    else
    {
        setMotorRaw(
            TURN2_LEFT_PIVOT_SPEED,
            TURN2_RIGHT_SPEED);
    }

    // Step 1: Leave old line
    int clearCount = 0;

    while (true)
    {
        readLineSensors();

        bool offLine =
            sensorNormalized[2] <= SENSOR_ACTIVE_THRESHOLD &&
            sensorNormalized[3] <= SENSOR_ACTIVE_THRESHOLD;

        if (offLine)
        {
            clearCount++;
        }
        else if (clearCount > 0)
        {
            clearCount--;
        }

        if (clearCount >= TURN2_LINE_STABLE_COUNT)
        {
            break;
        }
    }

    // Step 2: Find new line
    int stableCount = 0;

    while (true)
    {
        readLineSensors();

        bool onLine =
            sensorNormalized[2] > SENSOR_ACTIVE_THRESHOLD &&
            sensorNormalized[3] > SENSOR_ACTIVE_THRESHOLD;

        if (onLine)
        {
            stableCount++;
        }
        else if (stableCount > 0)
        {
            stableCount--;
        }

        if (stableCount >= TURN2_LINE_STABLE_COUNT)
        {
            break;
        }
    }

    stopMotors();
}

// =====================================================
// TURN - SENSOR BASED TURN
// =====================================================
static void turn(int dir)
{
    // Hai banh co toc do doc lap
    if (dir == DIR_RIGHT)
    {
        setMotorRaw(
            TURN_LEFT_SPEED,
            TURN_RIGHT_PIVOT_SPEED);
    }
    else
    {
        setMotorRaw(
            TURN_LEFT_PIVOT_SPEED,
            TURN_RIGHT_SPEED);
    }

    int stableCount = 0;

    while (true)
    {
        readLineSensors();

        bool onLine =
            sensorNormalized[2] > SENSOR_ACTIVE_THRESHOLD &&
            sensorNormalized[3] > SENSOR_ACTIVE_THRESHOLD;

        if (onLine)
        {
            stableCount++;
        }
        else if (stableCount > 0)
        {
            stableCount--;
        }

        if (stableCount >= TURN_LINE_STABLE_COUNT)
        {
            break;
        }
    }
}

// =====================================================
// TURN AFTER BOTTLE
// MAP LEFT -> RIGHT
// MAP RIGHT -> LEFT
// =====================================================
static void rotateTowardReturnLine()
{
    if (mapSide == LEFT_MAP)
    {
        // Map trai -> quay phai
        setMotorRaw(
            BOTTLE_UTURN_LEFT_SPEED,
            -BOTTLE_UTURN_RIGHT_SPEED);
    }
    else if (mapSide == RIGHT_MAP)
    {
        // Map phai -> quay trai
        setMotorRaw(
            -BOTTLE_UTURN_LEFT_SPEED,
            BOTTLE_UTURN_RIGHT_SPEED);
    }
    else
    {
        stopMotors();
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

    bottleStartTime = 0;
    noLineStartTime = 0;
    finishNoLineStartTime = 0;

    bottleBackLineCount = 0;
    bottleTurnClearCount = 0;
    bottleTurnLineCount = 0;

    returnIntersectionArmed = true;
    returnArmCount = 0;

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
    // FOLLOW LINE - FULL MAP
    // =====================================================
    case FOLLOW_LINE:
    {
        // Tranh dem lai giao diem cu khi quay ve
        if (!returnIntersectionArmed)
        {
            if (
                isOutOfIntersection() &&
                !isNoLine())
            {
                if (
                    returnArmCount <
                    RETURN_REARM_STABLE_COUNT)
                {
                    returnArmCount++;
                }
            }
            else
            {
                returnArmCount = 0;
            }

            if (
                returnArmCount >=
                RETURN_REARM_STABLE_COUNT)
            {
                returnIntersectionArmed = true;
                returnArmCount = 0;
                intersectionCount = 0;
                lastDetectTime = millis();
            }

            updateLineFollowing();
            break;
        }

        if (
            millis() - lastDetectTime > DETECT_DELAY &&
            detectStableIntersection())
        {
            cnt++;
            digitalWrite(LED_PIN, HIGH);

            stopMotors();
            delay(INTERSECTION_STOP_DELAY);

            // ===== STEP 1: GO STRAIGHT =====
            if (cnt == STEP_PASS_FIRST)
            {
                setMotorRaw(
                    STEP1_FORWARD_SPEED,
                    STEP1_FORWARD_SPEED);

                delay(STEP1_FORWARD_DELAY);
            }

            // ===== STEP 2: DETECT MAP =====
            else if (cnt == STEP_DETECT_MAP)
            {
                int dir = readTurnDirection();

                if (dir == DIR_LEFT)
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
                turn(dir);
            }

            // ===== STEP 3: FOLLOW MAP =====
            else if (cnt == STEP_FOLLOW_MAP)
            {
                forwardUntilNoLine();

                if (currentMap != NULL)
                {
                    turn(currentMap[cnt]);
                }
            }

            // ===== STEP 4: BOTTLE BRANCH =====
            else if (cnt == STEP_BOTTLE_BRANCH)
            {
                if (currentMap != NULL)
                {
                    turn2(currentMap[cnt]);
                }

                lastDetectTime = millis();
                intersectionCount = 0;

                bottleStartTime = millis();
                noLineStartTime = 0;

                state = BOTTLE_ALIGN;
                return;
            }

            // ===== STEP 5 + 6: RETURN =====
            else if (
                cnt == STEP_RETURN_1 ||
                cnt == STEP_RETURN_2)
            {
                if (currentMap != NULL)
                {
                    turn2(currentMap[cnt]);
                }
            }

            // ===== STEP 7: PASS CROSSROAD =====
            else if (cnt >= STEP_FINAL_STOP)
            {
                // Di thang qua nga tu
                forwardUntilClear();

                intersectionCount = 0;
                finishNoLineStartTime = 0;

                // Chua dung, tiep tuc bam line
                state = FINISH_FOLLOW_LINE;
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
    // ALIGN AFTER TURNING INTO BOTTLE BRANCH
    // =====================================================
    case BOTTLE_ALIGN:
    {
        updateLineFollowing();

        if (
            millis() - bottleStartTime >=
            BOTTLE_ALIGN_TIME)
        {
            stopMotors();

            bottleStartTime = millis();
            noLineStartTime = 0;

            state = BOTTLE_APPROACH;
        }

        break;
    }

    // =====================================================
    // SLOW APPROACH TO BOTTLE
    // =====================================================
    case BOTTLE_APPROACH:
    {
        // Safety timeout
        if (
            millis() - bottleStartTime >=
            BOTTLE_FORWARD_TIMEOUT)
        {
            stopMotors();
            state = FINAL_STOP;
            break;
        }

        // Het line o cuoi nhanh chai
        if (isNoLine())
        {
            stopMotors();

            if (noLineStartTime == 0)
            {
                noLineStartTime = millis();
            }

            if (
                millis() - noLineStartTime >=
                BOTTLE_NO_LINE_TIME)
            {
                bottleStartTime = millis();
                state = BOTTLE_STOP;
            }

            break;
        }

        noLineStartTime = 0;

        // Chinh huong nhe khi tien den chai
        float error = calculateLinePosition();

        float correction =
            BOTTLE_FORWARD_KP * error;

        correction = constrain(
            correction,
            -(float)BOTTLE_MAX_CORRECTION,
            (float)BOTTLE_MAX_CORRECTION);

        int left = constrain(
            (int)(BOTTLE_FORWARD_SPEED - correction),
            0,
            255);

        int right = constrain(
            (int)(BOTTLE_FORWARD_SPEED + correction),
            0,
            255);

        setMotorRaw(left, right);
        break;
    }

    // =====================================================
    // STOP AFTER BOTTLE
    // =====================================================
    case BOTTLE_STOP:
    {
        stopMotors();

        if (
            millis() - bottleStartTime >=
            BOTTLE_STOP_TIME)
        {
            bottleBackLineCount = 0;
            bottleStartTime = millis();

            state = BOTTLE_BACK;
        }

        break;
    }

    // =====================================================
    // REVERSE UNTIL LINE IS DETECTED
    // =====================================================
    case BOTTLE_BACK:
    {
        if (
            millis() - bottleStartTime >=
            BOTTLE_BACK_FIND_TIMEOUT_MS)
        {
            stopMotors();
            state = FINAL_STOP;
            break;
        }

        setMotorRaw(
            -BOTTLE_BACK_SPEED,
            -BOTTLE_BACK_SPEED);

        bool centerOnLine =
            sensorNormalized[2] > SENSOR_ACTIVE_THRESHOLD ||
            sensorNormalized[3] > SENSOR_ACTIVE_THRESHOLD;

        if (centerOnLine)
        {
            bottleBackLineCount++;
        }
        else
        {
            bottleBackLineCount = 0;
        }

        if (
            bottleBackLineCount >=
            BOTTLE_BACK_LINE_STABLE)
        {
            stopMotors();

            bottleTurnClearCount = 0;
            bottleTurnLineCount = 0;

            state = BOTTLE_TURN_CLEAR;
        }

        break;
    }

    // =====================================================
    // TURN: LEAVE OLD LINE
    // =====================================================
    case BOTTLE_TURN_CLEAR:
    {
        if (mapSide == UNKNOWN)
        {
            stopMotors();
            state = FINAL_STOP;
            break;
        }

        rotateTowardReturnLine();

        bool centerOffLine =
            sensorNormalized[2] <= SENSOR_ACTIVE_THRESHOLD &&
            sensorNormalized[3] <= SENSOR_ACTIVE_THRESHOLD;

        if (centerOffLine)
        {
            bottleTurnClearCount++;
        }
        else
        {
            bottleTurnClearCount = 0;
        }

        if (
            bottleTurnClearCount >=
            BOTTLE_UTURN_CLEAR_STABLE)
        {
            bottleTurnLineCount = 0;
            state = BOTTLE_TURN_FIND;
        }

        break;
    }

    // =====================================================
    // TURN: FIND NEW LINE
    // =====================================================
    case BOTTLE_TURN_FIND:
    {
        // Quay theo cam bien, khong timeout
        rotateTowardReturnLine();

        if (isCenterOnLine())
        {
            bottleTurnLineCount++;
        }
        else
        {
            bottleTurnLineCount = 0;
        }

        if (
            bottleTurnLineCount >=
            BOTTLE_UTURN_LINE_STABLE)
        {
            stopMotors();

            lastDetectTime = millis();
            intersectionCount = 0;

            // Tranh dem lai giao diem cu
            returnIntersectionArmed = false;
            returnArmCount = 0;

            // Chay ve theo map
            state = FOLLOW_LINE;
        }

        break;
    }

    // =====================================================
    // AFTER STEP 7: FOLLOW TO END OF LINE
    // =====================================================
    case FINISH_FOLLOW_LINE:
    {
        if (isNoLine())
        {
            stopMotors();

            if (finishNoLineStartTime == 0)
            {
                finishNoLineStartTime = millis();
            }

            if (
                millis() - finishNoLineStartTime >=
                FINISH_NO_LINE_MS)
            {
                digitalWrite(LED_PIN, LOW);
                state = FINAL_STOP;
            }
        }
        else
        {
            finishNoLineStartTime = 0;
            updateLineFollowing();
        }

        break;
    }

    // =====================================================
    // FINAL STOP
    // =====================================================
    case FINAL_STOP:
    {
        stopMotors();
        break;
    }

    default:
    {
        break;
    }
    }
}
