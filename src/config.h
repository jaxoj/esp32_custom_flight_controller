#pragma once

// --- I2C & Sensors ---
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68
#define HMC5883L_ADDR 0x0D
#define NRF_ADDR "DRONE"

// --- ESC Pins --- 
#define ESC_FL 27
#define ESC_FR 12
#define ESC_BL 14
#define ESC_BR 13
// --- NRF Pins ---
#define NRF_CE 5
#define NRF_CSN 17

// --- Timing (CHANGED) ---
// Replaced LOOP_DELAY_MS with a fixed frequency target
#define LOOP_FREQ 250 // 250Hz = 4ms per loop
#define TARGET_TIME_US (1000000 / LOOP_FREQ) 

// --- Complementary Filter ---
#define ALPHA 0.96

// --- PID Tuning (CHANGED) ---
// Increased P gains so you can feel the motors fighting back
#define KP_ROLL 5.0   
#define KI_ROLL 0.01
#define KD_ROLL 0.05

#define KP_PITCH 5.0
#define KI_PITCH 0.01
#define KD_PITCH 0.05

#define KP_YAW 2.0
#define KI_YAW 0.05
#define KD_YAW 0.05

// --- Limits ---
#define PWM_MIN 1000
#define PWM_MAX 2000
#define BASE_THROTTLE 1350 // Increased to generate actual torque

// (NEW) Add an I-term limit to prevent windup on the ground
#define PID_I_MAX 200.0