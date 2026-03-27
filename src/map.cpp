#include <Arduino.h>
#include "map.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

// ================= CONFIG =================
#define DETECT_DELAY 300
#define LED_PIN 2

// ================= DIRECTION =================
#define DIR_LEFT  -1
#define DIR_RIGHT  1

#define MAX_STEP 8

// ================= MAP =================
const int mapLeft[MAX_STEP]  = {0, 0, DIR_LEFT,  DIR_RIGHT, DIR_RIGHT,DIR_LEFT,DIR_RIGHT,0};
const int mapRight[MAX_STEP] = {0, 0, DIR_RIGHT, DIR_LEFT,  DIR_LEFT,DIR_RIGHT,DIR_LEFT,0};
static const int* currentMap = NULL;

// ================= STATE =================
enum RobotState
{
    FOLLOW_LINE,
    FINAL_FORWARD,
    FINAL_BACK,
    FINAL_TURN
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

// =====================================================
// DETECT INTERSECTION
// =====================================================
static bool detectIntersection()
{
    float s0 = sensorNormalized[0];
    float s1 = sensorNormalized[1];
    float s2 = sensorNormalized[2];
    float s3 = sensorNormalized[3];
    float s4 = sensorNormalized[4];
    float s5 = sensorNormalized[5];

    int active = 0;

    if (s0 > 0.5) active++;
    if (s1 > 0.5) active++;
    if (s2 > 0.5) active++;
    if (s3 > 0.5) active++;
    if (s4 > 0.5) active++;
    if (s5 > 0.5) active++;

    float sum = s0 + s1 + s2 + s3 + s4 + s5;

    return (active >= 4 && sum > 3.0);
}

static bool detectStableIntersection()
{
    static int count = 0;

    if (detectIntersection())
        count++;
    else if (count > 0)
        count--;

    return (count >= 3);
}

static bool isOutOfIntersection()
{
    int active = 0;

    for (int i = 0; i < 6; i++)
    {
        if (sensorNormalized[i] > 0.5)
            active++;
    }

    // <=2 mắt sáng → đã ra khỏi ngã
    return (active <= 2);
}
static void forwardUntilClear()
{
    int stable = 0;

    while (true)
    {
        readLineSensors();

        setMotorRaw(150, 150); // đi thẳng nhẹ

        if (isOutOfIntersection())
            stable++;
        else if (stable > 0)
            stable--;

        if (stable >= 3)
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
    int speed = 200;

    if (dir == DIR_RIGHT)
    {
        setMotorRaw(speed, 1);
        delay(800);
    }
    else
    {
        setMotorRaw(1, speed);
        delay(1000);
    }
    setMotorRaw(180,180);
    delay(50);
}

static void turn(int dir)
{
    int speed = 180;

    // ===== 1. QUAY (kick mạnh để thoát tâm ngã) =====
    if (dir == DIR_RIGHT)
        setMotorRaw(speed-10, 1);
    else
        setMotorRaw(1, speed);

    // ===== 4. BẮT LINE MỚI =====
    int stableCount = 0;

    while (true)
    {
        readLineSensors();

        float s2 = sensorNormalized[2];
        float s3 = sensorNormalized[3];

        if (s2 > 0.5 && s3 > 0.5)
            stableCount++;
        else if (stableCount > 0)
            stableCount--;

        if (stableCount >= 3)
            break;
    }

    // // // ===== 5. KICK NHẸ → CHO PID =====
    setMotorRaw(200, 200);
    delay(50);
}

// =====================================================
// INIT
// =====================================================
void mapInit()
{
    lastDetectTime = 0;
    cnt = 0;

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

// =====================================================
// MAIN UPDATE
// =====================================================
void mapUpdate()
{
    switch (state)
    {
        // =====================================================
        // FOLLOW LINE (PHẦN ĐI - GIỮ NGUYÊN)
        // =====================================================
        case FOLLOW_LINE:
        {
            bool hasIntersection = detectStableIntersection();

            if (hasIntersection && millis() - lastDetectTime > DETECT_DELAY)
            {
                if (cnt !=0) {
                    stopMotors();
                    delay(200);
                }
                setMotorRaw(150,150);
                delay(30);

                lastDetectTime = millis();
                cnt++;

                digitalWrite(LED_PIN, HIGH);

                // ===== CASE 1: đi thẳng =====
                if (cnt == 1)
                {
                    setMotorRaw(140, 140);
                    delay(20);
                }

                // ===== CASE 2: xác định map =====
                else if (cnt == 2)
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

                    turn(turnDir);
                }

                // ===== CASE 3 + 4 =====
                else if (cnt == 3 || cnt == 4||cnt==5 || cnt ==6)
                {
                    if (cnt==3) {
                    forwardUntilClear();
                    }
                    if (currentMap != NULL && cnt!=4 && cnt!=5 && cnt!=6)
                    {
                        int dir = currentMap[cnt];
                        turn(dir);
                    }
                    if(cnt==5 && cnt==6){
                        int dir = currentMap[cnt];
                        turn2(dir);
                    }
                    if (cnt == 4)
                    {
                        int dir = currentMap[cnt];
                        turn2(dir);
                        state = FINAL_FORWARD;
                        return;
                    }
                }

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

            if (millis() - startTime > 1500)
            { 
                startTime = 0;
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

            setMotorRaw(-160, -160);

            bool hasIntersection = detectStableIntersection();

            if (hasIntersection && millis() - lastDetect > 300)
            {
                lastDetect = millis();

                stopMotors();
                delay(200);

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
            
            state = FOLLOW_LINE;
            break;
        }

        default:
            break;
    }
}
