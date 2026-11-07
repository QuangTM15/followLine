#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"
#include "map.h"

void setup()
{
    WiFi.mode(WIFI_OFF);
    btStop();
    analogSetAttenuation(ADC_11db);
    sensorsInit();
    motorInit();
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);
    pidInit();
    mapInit();
    delay(1000);
    calibrateSensors(3000);
}

void loop()
{
    readLineSensors();
    mapUpdate();
    delay(5);
}
