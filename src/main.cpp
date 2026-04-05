#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// --- MPU6050 Variables ---
const int MPU = 0x68;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float roll, pitch;
float elapsedTime, currentTime, previousTime;

// --- PID Variables ---
float pid_p_roll = 0, pid_i_roll = 0, pid_d_roll = 0, PID_roll = 0;
float pid_p_pitch = 0, pid_i_pitch = 0, pid_d_pitch = 0, PID_pitch = 0;
float error_roll, previous_error_roll;
float error_pitch, previous_error_pitch;

// TUNING CONSTANTS (You will need to change these!)
double kp = 3.55;
double ki = 0.005;
double kd = 2.05;

// --- Motor Variables ---
Servo esc1, esc2, esc3, esc4;
int base_throttle = 1200; // Low hover throttle

void setup()
{
    Serial.begin(115200); // Add this line
    Wire.begin(21, 22);
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    esc1.attach(13, 1000, 2000); // Front Left
    esc2.attach(12, 1000, 2000); // Front Right
    esc3.attach(14, 1000, 2000); // Back Left
    esc4.attach(27, 1000, 2000); // Back Right

    // Arming
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    delay(5000);
}

void loop()
{
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0; // Time in seconds

    // 1. Read MPU
    // Read Accelerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    // Calculate Roll and Pitch from Accelerometer
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // 0.58 is an example error calibration
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;

    // Read Gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Complementary Filter
    gyroAngleX = gyroAngleX + GyroX * elapsedTime;
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    // 2. Calculate PID for Roll
    error_roll = roll - 0; // 0 is desired angle
    pid_p_roll = kp * error_roll;
    pid_i_roll = pid_i_roll + (ki * error_roll);
    pid_d_roll = kd * ((error_roll - previous_error_roll) / elapsedTime);
    PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;
    previous_error_roll = error_roll;

    // Calculate PID for Pitch
    error_pitch = pitch - 0;
    pid_p_pitch = kp * error_pitch;
    pid_i_pitch = pid_i_pitch + (ki * error_pitch);
    pid_d_pitch = kd * ((error_pitch - previous_error_pitch) / elapsedTime);
    PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
    previous_error_pitch = error_pitch;

    // 3. Motor Mixing
    int pwm_1 = base_throttle + PID_roll - PID_pitch;
    int pwm_2 = base_throttle - PID_roll - PID_pitch;
    int pwm_3 = base_throttle + PID_roll + PID_pitch;
    int pwm_4 = base_throttle - PID_roll + PID_pitch;

    // 4. Constrain limits (Don't let motors stop in mid-air or max out)
    pwm_1 = constrain(pwm_1, 1100, 2000);
    pwm_2 = constrain(pwm_2, 1100, 2000);
    pwm_3 = constrain(pwm_3, 1100, 2000);
    pwm_4 = constrain(pwm_4, 1100, 2000);

    // 5. Write to ESCs
    esc1.writeMicroseconds(pwm_1);
    esc2.writeMicroseconds(pwm_2);
    esc3.writeMicroseconds(pwm_3);
    esc4.writeMicroseconds(pwm_4);

    // --- DEBUG PRINTING ---
    // 1. Print Angles
    Serial.print("Roll:");
    Serial.print(roll);
    Serial.print("\t");
    Serial.print("Pitch:");
    Serial.print(pitch);
    Serial.print("\t");

    // 2. Print Total PID Output
    Serial.print("PID_R:");
    Serial.print(PID_roll);
    Serial.print("\t");
    Serial.print("PID_P:");
    Serial.print(PID_pitch);
    Serial.print("\t");

    // 3. Print Final Motor PWM Values
    Serial.print("M1_FL:");
    Serial.print(pwm_1);
    Serial.print("\t");
    Serial.print("M2_FR:");
    Serial.print(pwm_2);
    Serial.print("\t");
    Serial.print("M3_BL:");
    Serial.print(pwm_3);
    Serial.print("\t");
    Serial.print("M4_BR:");
    Serial.println(pwm_4); // println for the last one

    // Optional: Add a tiny delay so you don't flood the serial port and crash it
    delay(10);
}
