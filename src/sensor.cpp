#include <Arduino.h>
#include "sensor.h"

// ===== STORAGE =====
int sensorValues[NUM_SENSORS];
float sensorNormalized[NUM_SENSORS];

// ===== CONFIG =====
#define FILTER_SAMPLES 5

// min/max cho từng sensor
static int sensorMin[NUM_SENSORS];
static int sensorMax[NUM_SENSORS];
static bool isCalibrated = false;

// weights
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
// CALIBRATION
// =====================================================
void calibrateSensors(int durationMs)
{
    // init
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    unsigned long start = millis();

    while (millis() - start < durationMs)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            int val = analogRead(sensorPins[i]);

            if (val < sensorMin[i]) sensorMin[i] = val;
            if (val > sensorMax[i]) sensorMax[i] = val;
        }

        delay(5);
    }

    isCalibrated = true;
}

// =====================================================
// FILTER
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
        int raw = readAnalogFiltered(sensorPins[i]);
        sensorValues[i] = raw;

        float value;

        if (isCalibrated)
        {
            int minV = sensorMin[i];
            int maxV = sensorMax[i];

            if (maxV - minV > 0)
                value = (maxV - raw) / (float)(maxV - minV);
            else
                value = 0;
        }
        else
        {
            value = 1.0 - (raw / 4095.0);
        }

        value = constrain(value, 0.0, 1.0);

        // deadzone
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

    if (denominator < 0.03)
    {
        if (lastPosition > 0.2) return 6;
        if (lastPosition < -0.2) return -6;
        return 6;
    }

    float position = numerator / denominator;
    position = 0.7 * position + 0.3 * lastPosition;
    lastPosition = position;

    return position;
}

// =====================================================
// DEBUG (optional)
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