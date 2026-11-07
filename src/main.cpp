
#include <Arduino.h>
#include "config.h"
#include "motor.h"

#define MOTOR_STBY 23

void setup()
{
    Serial.begin(115200);

    // Initialize motor driver
    motorInit();

    // Enable TB6612FNG
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);

    delay(1000);

    Serial.println("Testing motor.cpp...");

    // Left motor reversed according to hardware test
    // Right motor forward
    setMotorRaw(-200, 200);

    Serial.println("Both motors should run forward!");
}

void loop()
{
    // Keep motors running
}
