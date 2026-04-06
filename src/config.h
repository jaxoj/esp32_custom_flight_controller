#pragma once

// --- I2C & Sensors ---
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68

// --- ESC Pins ---
#define ESC_FL 13
#define ESC_FR 12
#define ESC_BL 14
#define ESC_BR 27

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
#define KD_ROLL 1.5

#define KP_PITCH 5.0
#define KI_PITCH 0.01
#define KD_PITCH 1.5

#define KP_YAW 4.0
#define KI_YAW 0.02
#define KD_YAW 1.0

// --- Limits (CHANGED) ---
#define PWM_MIN 1100
#define PWM_MAX 2000
#define BASE_THROTTLE 1350 // Increased to generate actual torque

// (NEW) Add an I-term limit to prevent windup on the ground
#define PID_I_MAX 150.0