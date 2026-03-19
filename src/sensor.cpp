#include <Arduino.h>
#include "sensor.h"

// ===== STORAGE =====
int sensorValues[NUM_SENSORS];
float sensorNormalized[NUM_SENSORS];

// ===== CONFIG =====
#define FILTER_SAMPLES 5

#define SENSOR_MIN 500
#define SENSOR_MAX 3500


static const int weights[NUM_SENSORS] = {-5, -3, -1, 1, 3, 5};

static float lastPosition = 0;

// =====================================================
// INIT
// =====================================================
void sensorsInit()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(sensorPins[i], INPUT);
    }
}

// =====================================================
// FILTER (giảm nhiễu analog)
// =====================================================
static int readAnalogFiltered(int pin)
{
    long sum = 0;

    for (int i = 0; i < FILTER_SAMPLES; i++)
    {
        sum += analogRead(pin);
        delayMicroseconds(150);
    }

    return sum / FILTER_SAMPLES;
}

// =====================================================
// READ + NORMALIZE
// =====================================================
void readLineSensors()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // đọc raw
        int raw = readAnalogFiltered(sensorPins[i]);
        sensorValues[i] = raw;

        // normalize: đen = 1, trắng = 0
        float value = (SENSOR_MAX - raw) / (float)(SENSOR_MAX - SENSOR_MIN);
        value = constrain(value, 0.0, 1.0);

        // deadzone chống nhiễu
        if (value < 0.1) value = 0;
        if (value > 0.9) value = 1;

        sensorNormalized[i] = value;
    }
}

// =====================================================
// CALCULATE POSITION
// =====================================================
float calculateLinePosition()
{
    float numerator = 0;
    float denominator = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        numerator += sensorNormalized[i] * weights[i];
        denominator += sensorNormalized[i];
    }

    // mất line
    if (denominator < 0.05)
    {
        return lastPosition;
    }

    float position = numerator / denominator;
    lastPosition = position;

    return position;
}

// =====================================================
// DEBUG
// =====================================================
void debugSensors()
{
    Serial.print("RAW: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print("\t");
    }

    Serial.print("| NORM: ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(sensorNormalized[i], 2);
        Serial.print(" ");
    }

    Serial.println();
}
