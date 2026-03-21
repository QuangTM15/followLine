#include <Arduino.h>
#include "map.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

// ================= CONFIG =================
#define THRESHOLD 0.5
#define DETECT_DELAY 1500
#define LED_PIN 2



// ================= STATE =================    
static bool reachedT = false;
static unsigned long lastDetectTime = 0;
static uint8_t cnt=0;
static int turnDir = 0;

// ================= DIRECTION =================
#define DIR_LEFT  -1
#define DIR_RIGHT  1
//STATE
enum RobotState
{
    FOLLOW_LINE,
    TURNING
};

static RobotState state = FOLLOW_LINE;

enum MapSide
{
    UNKNOWN,
    LEFT_MAP,
    RIGHT_MAP
};

static MapSide mapSide = UNKNOWN;

static bool detectIntersection()
{
    float s0 = sensorNormalized[0];
    float s1 = sensorNormalized[1];
    float s2 = sensorNormalized[2];
    float s3 = sensorNormalized[3];
    float s4 = sensorNormalized[4];
    float s5 = sensorNormalized[5];

    // ===== ĐẾM SỐ MẮT ACTIVE =====
    int active = 0;

    if (s0 > 0.5) active++;
    if (s1 > 0.5) active++;
    if (s2 > 0.5) active++;
    if (s3 > 0.5) active++;
    if (s4 > 0.5) active++;
    if (s5 > 0.5) active++;

    // ===== TỔNG CƯỜNG ĐỘ =====
    float sum = s0 + s1 + s2 + s3 + s4 + s5;

    // ===== ĐIỀU KIỆN GIAO LỘ =====
    // nhiều mắt + tổng lớn
    if (active >= 4 && sum > 3.0)
        return true;

    return false;
}

static bool detectStableIntersection()
{
    static int count = 0;

    if (detectIntersection())
        count++;
    else if (count > 0)
        count--;

    if (count >= 3)
        return true;

    return false;
}

static int readTurnDirection()
{
    float s0 = sensorNormalized[0];
    float s1 = sensorNormalized[1];
    float s4 = sensorNormalized[4];
    float s5 = sensorNormalized[5];

    float left  = s4 + s5;
    float right = s0 + s1;

    if (left > right)
        return DIR_LEFT;
    else
        return DIR_RIGHT;
}

// =====================================================
// STOP
// =====================================================
static void stopRobot()
{
    stopMotors();
}

// =====================================================
// TURN 
// =====================================================
static void turn(int dir)
{
    int speed = 240;

    if (dir == DIR_RIGHT)
        setMotorRaw(speed, 1);  
    else
        setMotorRaw(1, speed);

    delay(500);

    stopMotors();
    delay(50);
}


// =====================================================
// INIT
// =====================================================
void mapInit()
{
    reachedT = false;
    lastDetectTime = 0;

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
        case FOLLOW_LINE:
        {
            bool hasIntersection = detectStableIntersection();

            if (hasIntersection && millis() - lastDetectTime > DETECT_DELAY)
            {
                lastDetectTime = millis();
                cnt++;

                digitalWrite(LED_PIN, HIGH);

                // ===== CASE 1: đi thẳng =====
                if (cnt == 1)
                {
                    // không làm gì → PID tự đi
                }

                // ===== CASE 2: xác định map =====
                else if (cnt == 2)
                {
                    turnDir = readTurnDirection();

                    if (turnDir == DIR_LEFT)
                        mapSide = LEFT_MAP;
                    else
                        mapSide = RIGHT_MAP;

                    turn(turnDir);
                }

                // ===== CASE 3: đi theo map =====
                else if (cnt == 3)
                {
                    if (mapSide == LEFT_MAP)
                        turn(DIR_RIGHT);
                    else
                        turn(DIR_LEFT);
                }

                // ===== CASE 4: về đích =====
                else if (cnt == 4)
                {
                    if (mapSide == LEFT_MAP)
                        turn(DIR_RIGHT);
                    else
                        turn(DIR_LEFT);
                    
                    stopMotors();   
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

        case TURNING:
        {
            state = FOLLOW_LINE;
            break;
        }
    }
}