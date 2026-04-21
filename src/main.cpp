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

// --- Task Handles ---
TaskHandle_t FlightTaskHandle;

// --- Shared Data (Thread-Safeish) ---
struct ControllData
{
    float throttle, rollTarget, pitchTarget, yawTarget;
    bool connected;
} sharedData;

// --- Task 1: Flight Control (Core 1 - High Priority) ---
void flightTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // 4ms period = 250Hz. Professional FCs usually run at 1kHz+,
    // but 250Hz is a great stable start for I2C.
    const TickType_t xFrequency = pdMS_TO_TICKS(4);
    const float dt = 0.004f; // Fixed dt for the PID math

    for (;;)
    {
        // Wait until exactly 4ms has passed since the last start
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Update the Sensors
        imu.update(dt);

        // Get local copies of targets (Minimize time spent accessing shared memory)
        float t = sharedData.throttle;
        float rt = sharedData.rollTarget;
        float pt = sharedData.pitchTarget;
        float yt = sharedData.yawTarget;
        bool isConnected = sharedData.connected;

        // Failsafe logic
        if (!isConnected || t < 1050)
        {
            motors.update(PWM_MIN, 0, 0, 0);
            pidRoll.reset();
            pidPitch.reset();
            pidYaw.reset();
            continue;
        }

        // PID Calculations
        float rPID = pidRoll.compute(rt, imu.getRoll(), dt);
        float pPID = pidPitch.compute(pt, imu.getPitch(), dt);
        float yPID = pidYaw.compute(yt, imu.getYaw(), dt);

        // Motor Output
        motors.update(t, rPID, pPID, yPID);
    }
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    nrf.begin();
    imu.begin();
    imu.calibrate();
    motors.begin();
    motors.arm();

    // Create the Flight Task on Core 1
    // Priority 24 is very high. Stack size 8192 is plenty for math.
    xTaskCreatePinnedToCore(
        flightTask,
        "FlightLoop",
        8192,
        NULL,
        24,
        &FlightTaskHandle,
        1);

    Serial.println("Flight Task Started on Core 1");
}

void loop()
{
    // --- Task 2: Radio & Telemetry (Core 0 - Default) ---
    // The loop() function on ESP32 runs on Core 1 by default, but we can 
    // use it for the Radio since the Flight Task will preempt it.

    // --- Read Radio ---
    if (nrf.available())
    {
        nrf.read();

        // Update the shared struct for Core 1 to see
        sharedData.throttle = nrf.data.throttle;
        sharedData.rollTarget = (abs(nrf.data.rollTarget) < 2) ? 0 : nrf.data.rollTarget;
        sharedData.pitchTarget = (abs(nrf.data.pitchTarget) < 2) ? 0 : nrf.data.pitchTarget;
        sharedData.yawTarget = (abs(nrf.data.yawTarget) < 5) ? 0 : nrf.data.yawTarget;
        sharedData.connected = true;
    }

    // Simple Failsafe timeout check
    if (millis() - nrf.lastReceiveTime > 1000) {
        sharedData.connected = false;
    }

    // Small delay to let the background Watchdog timer breathe
    delay(1);
}