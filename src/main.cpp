#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"
#include "map.h"

void setup()
{
    Serial.begin(9600);

    WiFi.mode(WIFI_OFF);
    btStop();

    analogSetAttenuation(ADC_11db);

    sensorsInit();
    motorInit();
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
