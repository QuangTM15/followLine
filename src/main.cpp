#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "sensor.h"

void setup()
{
    Serial.begin(115200);

    // 🔥 Tắt WiFi + Bluetooth (tránh lỗi ADC2)
    WiFi.mode(WIFI_OFF);
    btStop();

    // 🔧 ADC full range
    analogSetAttenuation(ADC_11db);

    sensorsInit();

    Serial.println("=== CALIBRATION MODE ===");
    Serial.println("Place sensor on BLACK and WHITE to get MIN/MAX");
}

void loop()
{
    readLineSensors();

    int minVal = 4095;
    int maxVal = 0;

    Serial.print("RAW: ");

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int v = sensorValues[i];

        Serial.print(v);
        Serial.print("\t");

        if (v < minVal) minVal = v;
        if (v > maxVal) maxVal = v;
    }

    Serial.print("| MIN: ");
    Serial.print(minVal);

    Serial.print(" | MAX: ");
    Serial.print(maxVal);

    Serial.println();

    delay(100);
