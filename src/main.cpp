#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensors/IMU.h"
#include "control/PID.h"
#include "control/NRF.h"
#include "actuators/MotorController.h"

const byte nrfAddress[6] = NRF_ADDR;
NRFReceiver nrf(NRF_CE, NRF_CSN, nrfAddress);


IMU imu;
MotorController motors;

// Limits moved to PID_I_MAX in config.h for easier tuning
PID pidRoll(KP_ROLL, KI_ROLL, KD_ROLL, -PID_I_MAX, PID_I_MAX);
PID pidPitch(KP_PITCH, KI_PITCH, KD_PITCH, -PID_I_MAX, PID_I_MAX);

unsigned long loopTimer = 0, lastPrint = 0;

void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    nrf.begin();

    // Optional: center values check
    Serial.println("Reciever Ready");

    imu.begin();
    imu.calibrate();

    motors.begin();
    motors.arm();

    loopTimer = micros();
}

void loop() {
    // 1. Precise Timing Control: Wait until the exact target time has passed
    // This ensures a rock-solid LOOP_FREQ (e.g. 250Hz)
    while (micros() - loopTimer < TARGET_TIME_US);

    // 2. Calculate the actual elapsed time (dt) in seconds
    unsigned long now = micros();
    float dt = (now - loopTimer) / 1000000.0;
    loopTimer = now;

    // --- Read Radio ---
    if (nrf.available()) {
        nrf.read();
    }

    // --- FAILSAFE (signal lost) ---
    if (millis() - nrf.lastReceiveTime > 500) {  // 500ms timeout
        motors.update(PWM_MIN, 0, 0, 0);      // Kill motors
        pidRoll.reset();
        pidPitch.reset();
        return;
    }

    // --- Safety / Arming ---
    if (nrf.data.throttle < 1050) {
        motors.update(PWM_MIN, 0, 0, 0);
        pidRoll.reset();
        pidPitch.reset();
        return;
    }

    // --- Apply Deadband (remove small noise) ---
    if (abs(nrf.data.rollTarget) < 2) nrf.data.rollTarget = 0;
    if (abs(nrf.data.pitchTarget) < 2) nrf.data.pitchTarget = 0;
    if (abs(nrf.data.yawTarget) < 2) nrf.data.yawTarget = 0;

    // --- Flight Control ---
    // Update Sensors
    imu.update(dt);

    // Get current orientation
    float roll = imu.getRoll();
    float pitch = imu.getPitch();

    // Compute PID Corrections
    // Setpoint is 0 (keep the drone level)
    float rollPID = pidRoll.compute(nrf.data.rollTarget, roll, dt);
    float pitchPID = pidPitch.compute(nrf.data.pitchTarget, pitch, dt);

    // --- Update Motors ---
    motors.update(
        nrf.data.throttle,   // use real throttle from radio
        rollPID,
        pitchPID,
        nrf.data.yawTarget
    );
}
