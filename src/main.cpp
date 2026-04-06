#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensors/IMU.h"
#include "control/PID.h"
#include "actuators/MotorController.h"

IMU imu;
MotorController motors;

// Limits moved to PID_I_MAX in config.h for easier tuning
PID pidRoll(KP_ROLL, KI_ROLL, KD_ROLL, -PID_I_MAX, PID_I_MAX);
PID pidPitch(KP_PITCH, KI_PITCH, KD_PITCH, -PID_I_MAX, PID_I_MAX);

unsigned long loopTimer = 0;

void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    imu.begin();
    imu.calibrate();

    motors.begin();
    motors.arm();

    loopTimer = millis();
}

void loop() {
    // 1. Precise Timing Control: Wait until the exact target time has passed
    // This ensures a rock-solid LOOP_FREQ (e.g. 250Hz)
    while (micros() - loopTimer < TARGET_TIME_US);

    // 2. Calculate the actual elapsed time (dt) in seconds
    unsigned long now = micros();
    float dt = (now - loopTimer) / 1000000.0;
    loopTimer = now;

    // 3. Update Sensors
    imu.update(dt);

    // 4. Get current orientation
    float roll = imu.getRoll();
    float pitch = imu.getPitch();

    // 5. Compute PID Corrections
    // Setpoint is 0 (keep the drone level)
    float rollPID = pidRoll.compute(0, roll, dt);
    float pitchPID = pidPitch.compute(0, pitch, dt);

    // 6. Update Motors (Yaw is set to 0 for now)
    motors.update(BASE_THROTTLE, rollPID, pitchPID, 0);
}