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

// PID Tuning for Yaw Rate (This usually needs a higher P than angle)
PID pidRoll(KP_ROLL, KI_ROLL, KD_ROLL, -PID_I_MAX, PID_I_MAX);
PID pidPitch(KP_PITCH, KI_PITCH, KD_PITCH, -PID_I_MAX, PID_I_MAX);
PID pidYaw(KP_YAW, KI_YAW, KD_YAW, -PID_I_MAX, PID_I_MAX);

unsigned long loopTimer = 0;

void setup()
{
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    nrf.begin();
    Serial.println("Receiver Ready");

    imu.begin();
    imu.calibrate();

    motors.begin();
    motors.arm();

    loopTimer = micros();
}

void loop()
{
    // Timing Control
    while (micros() - loopTimer < TARGET_TIME_US);

    unsigned long now = micros();
    float dt = (now - loopTimer) / 1000000.0;
    loopTimer = now;

    // --- Read Radio ---
    if (nrf.available()) {
        nrf.read();
    }

    // --- FAILSAFE ---
    if (millis() - nrf.lastReceiveTime > 1000 || nrf.data.throttle < 1050) {
        motors.update(PWM_MIN, 0, 0, 0);
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        return;
    }

    // --- Deadband ---
    if (abs(nrf.data.rollTarget) < 2) nrf.data.rollTarget = 0;
    if (abs(nrf.data.pitchTarget) < 2) nrf.data.pitchTarget = 0;
    if (abs(nrf.data.yawTarget) < 5) nrf.data.yawTarget = 0;

    // --- Update Sensors ---
    imu.update(dt);

    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw(); 

    // --- PID Calculations ---
    
    // Roll/Pitch: Setpoint is the angle from the sticks, Measurement is current angle
    float rollPID = pidRoll.compute(nrf.data.rollTarget, roll, dt);
    float pitchPID = pidPitch.compute(nrf.data.pitchTarget, pitch, dt);
    
    // Yaw: Setpoint is desired rotation speed, Measurement is actual rotation speed
    // If nrf.data.yawTarget is 0, the PID will fight to keep yawRate at 0 (No spinning)
    float yawPID = pidYaw.compute(nrf.data.yawTarget, yaw, dt);

    // --- Update Motors ---
    motors.update(
        nrf.data.throttle,
        rollPID,
        pitchPID,
        yawPID);
}