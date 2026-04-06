#include "IMU.h"
#include "../config.h"
#include <Arduino.h>

void IMU::begin() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void IMU::calibrate() {
    Serial.println("Calibrating IMU...");
    float totalAccAngleX = 0;
    float totalAccAngleY = 0;

    for (int i = 0; i < 500; i++) {
        // Read Raw Data
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);

        float ax = (Wire.read() << 8 | Wire.read()) / 16384.0;
        float ay = (Wire.read() << 8 | Wire.read()) / 16384.0;
        float az = (Wire.read() << 8 | Wire.read()) / 16384.0;
        Wire.read(); Wire.read(); // Skip Temperature
        float gx = (Wire.read() << 8 | Wire.read()) / 131.0;
        float gy = (Wire.read() << 8 | Wire.read()) / 131.0;

        // Accumulate Gyro
        gyroX_offset += gx;
        gyroY_offset += gy;

        // Accumulate Accel Angles
        totalAccAngleX += atan(ay / sqrt(ax * ax + az * az)) * 180 / PI;
        totalAccAngleY += atan(-ax / sqrt(ay * ay + az * az)) * 180 / PI;

        delay(2);
    }

    gyroX_offset /= 500.0;
    gyroY_offset /= 500.0;
    accAngleX_offset = totalAccAngleX / 500.0; // Store the "tilt" error
    accAngleY_offset = totalAccAngleY / 500.0;
    
    Serial.println("IMU Calibrated.");
}

void IMU::update(float dt) {
    // Read accelerometer
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    float accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    float accAngleX = (atan(accY / sqrt(accX * accX + accZ * accZ)) * 180 / PI) - accAngleX_offset;
    float accAngleY = (atan(-accX / sqrt(accY * accY + accZ * accZ)) * 180 / PI) - accAngleY_offset;
    
    // Read gyro
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 4, true);

    float gyroX = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyroX_offset;
    float gyroY = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyroY_offset;

    // Complementary filter
    roll = ALPHA * (roll + gyroX * dt) + (1 - ALPHA) * accAngleX;
    pitch = ALPHA * (pitch + gyroY * dt) + (1 - ALPHA) * accAngleY;
}

float IMU::getRoll() { return roll; }
float IMU::getPitch() { return pitch; }