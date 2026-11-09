
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
// FORWARD STRAIGHT THROUGH INTERSECTION
// Dung o STEP 7: di thang qua nga tu, khong dung
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
    float s0 = sensorNormalized[0];
    float s1 = sensorNormalized[1];
    float s4 = sensorNormalized[4];
    float s5 = sensorNormalized[5];

    float left = s4 + s5;
    float right = s0 + s1;

    return (left > right) ? DIR_LEFT : DIR_RIGHT;
}

// =====================================================
// TURN2 - ORIGINAL SENSOR TURN
// =====================================================
static void turn2(int dir)
{
    if (dir == DIR_RIGHT)
    {
        setMotorRaw(
            TURN2_SPEED,
            TURN2_PIVOT_SPEED);
    }
    else
    {
        setMotorRaw(
            TURN2_PIVOT_SPEED,
            TURN2_SPEED);
    }

    // Step 1: Leave old line
    int clearCount = 0;

    while (true)
    {
        readLineSensors();

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (
            s2 <= SENSOR_ACTIVE_THRESHOLD &&
            s3 <= SENSOR_ACTIVE_THRESHOLD)
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

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (
            s2 > SENSOR_ACTIVE_THRESHOLD &&
            s3 > SENSOR_ACTIVE_THRESHOLD)
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
// TURN - ORIGINAL SENSOR TURN
// =====================================================
static void turn(int dir)
{
    if (dir == DIR_RIGHT)
    {
        setMotorRaw(
            TURN_RIGHT_SPEED,
            TURN_PIVOT_SPEED);
    }
    else
    {
        setMotorRaw(
            TURN_PIVOT_SPEED,
            TURN_LEFT_SPEED);
    }

    int stableCount = 0;

    while (true)
    {
        readLineSensors();

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (
            s2 > SENSOR_ACTIVE_THRESHOLD &&
            s3 > SENSOR_ACTIVE_THRESHOLD)
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
// ROTATE TOWARD RETURN LINE
// MAP LEFT -> RIGHT
// MAP RIGHT -> LEFT
// =====================================================
static void rotateTowardReturnLine()
{
    if (mapSide == LEFT_MAP)
    {
        setMotorRaw(
            BOTTLE_UTURN_SPEED,
            -BOTTLE_UTURN_SPEED);
    }
    else if (mapSide == RIGHT_MAP)
    {
        setMotorRaw(
            -BOTTLE_UTURN_SPEED,
            BOTTLE_UTURN_SPEED);
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
        // Sau khi quay ve P1, tranh dem lai giao diem cu.
        // Chi cho dem nga moi sau khi da thay line hep.
        if (!returnIntersectionArmed)
        {
            if (isOutOfIntersection() && !isNoLine())
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

            // =========================================
            // STEP 1: GO STRAIGHT
            // =========================================
            if (cnt == STEP_PASS_FIRST)
            {
                setMotorRaw(
                    STEP1_FORWARD_SPEED,
                    STEP1_FORWARD_SPEED);

                delay(STEP1_FORWARD_DELAY);
            }

            // =========================================
            // STEP 2: DETECT MAP
            // =========================================
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

            // =========================================
            // STEP 3: FOLLOW MAP
            // =========================================
            else if (cnt == STEP_FOLLOW_MAP)
            {
                forwardUntilNoLine();

                if (currentMap != NULL)
                {
                    turn(currentMap[cnt]);
                }
            }

            // =========================================
            // STEP 4: BOTTLE BRANCH
            // =========================================
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

            // =========================================
            // STEP 5 + 6: RETURN TURNS
            // =========================================
            else if (
                cnt == STEP_RETURN_1 ||
                cnt == STEP_RETURN_2)
            {
                if (currentMap != NULL)
                {
                    // Re theo map bang cam bien
                    turn2(currentMap[cnt]);
                }
            }

            // =========================================
            // STEP 7: PASS FINAL CROSSROAD
            // =========================================
            else if (cnt >= STEP_FINAL_STOP)
            {
                // KHONG DUNG TAI NGA TU NAY
                // Di thang qua nga tu
                forwardUntilClear();

                intersectionCount = 0;
                finishNoLineStartTime = 0;

                // Tiep tuc PID den khi het line
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
    // ALIGN AFTER ENTERING BOTTLE BRANCH
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
        if (
            millis() - bottleStartTime >=
            BOTTLE_FORWARD_TIMEOUT)
        {
            stopMotors();
            state = FINAL_STOP;
            break;
        }

        // Mat line tai cuoi nhanh chai
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

        // PID nhe khi tien den chai
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
    // STOP AFTER PUSHING BOTTLE
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
        // Chi la gioi han an toan khi lui,
        // khong phai timeout quay.
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
    // TURN: LEAVE THE OLD LINE
    // =====================================================
    case BOTTLE_TURN_CLEAR:
    {
        if (mapSide == UNKNOWN)
        {
            stopMotors();
            state = FINAL_STOP;
            break;
        }

        // Map trai -> quay phai
        // Map phai -> quay trai
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
    // TURN: FIND THE NEW LINE
    // =====================================================
    case BOTTLE_TURN_FIND:
    {
        // Khong co timeout quay
        // Tiep tuc quay den khi bat lai line
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

            // Khong dem lai line tai P1
            returnIntersectionArmed = false;
            returnArmCount = 0;

            // Bat dau chay ve theo map
            state = FOLLOW_LINE;
        }

        break;
    }

    // =====================================================
    // AFTER STEP 7: FOLLOW TO THE END OF LINE
    // =====================================================
    case FINISH_FOLLOW_LINE:
    {
        // Chi dung khi mat line sau nga tu cuoi
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
