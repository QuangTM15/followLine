#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"

void setup()
{
    Serial.begin(9600);

    WiFi.mode(WIFI_OFF);
    btStop();

    analogSetAttenuation(ADC_11db);

    sensorsInit();
    motorInit();
    pidInit();

    delay(1000);

    // calibrate sensor
    calibrateSensors(3000);

    Serial.println("START");
}

void loop()
{
    readLineSensors();
    updateLineFollowing();
}