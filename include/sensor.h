#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"

// ===== DATA =====
extern int sensorValues[NUM_SENSORS];
extern float sensorNormalized[NUM_SENSORS];

// ===== API =====
void sensorsInit();
void readLineSensors();
float calculateLinePosition();
void calibrateSensors(int durationMs);

// debug (optional)
void debugSensors();

#endif