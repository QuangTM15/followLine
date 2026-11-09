#include <Arduino.h>
#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"
#include "map.h"

void setup()
{
    Serial.begin(115200);

    btStop();

    // Cấu hình độ suy hao ADC 11dB (đọc toàn dải 0 -> 3.3V)
    analogSetAttenuation(ADC_11db);

    // Khởi tạo các module
    sensorsInit();
    motorInit();

    // Bật driver động cơ TB6612FNG
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);

    pidInit();
    mapInit();

    delay(1000);

    // Bật LED báo hiệu đang calibrate cảm biến (3 giây)
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Calibrating sensors...");
    calibrateSensors(3000);
    digitalWrite(LED_PIN, LOW);

    // Chờ đặt robot vào vị trí xuất phát trước khi chạy
    Serial.println("Calibration done. Starting in 2s...");
    delay(2000);
    Serial.println("START!");
}

void loop()
{
    // Nếu đã đến đích cuối cùng trên map thì dừng hẳn robot
    if (isMapFinished())
    {
        stopMotors();
        delay(50);
        return;
    }

    readLineSensors();
    mapUpdate();
    delay(1);
}
