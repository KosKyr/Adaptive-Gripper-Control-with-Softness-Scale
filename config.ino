#ifndef CONFIG_H
#define CONFIG_H

// Feature toggles
#define ENABLE_COMMANDER 0
#define ENABLE_READ_ANGLE 0

// Motor setup
#define MOTOR_POLE_PAIRS 7
#define MOTOR_PHASE_RESISTANCE 0.24f
#define MOTOR_KV 360
#define MOTOR_PHASE_INDUCTANCE 0.000133f
#define MOTOR_VOLTAGE_ALIGN 3.0f
#define MOTOR_POWER_SUPPLY_VOLTAGE 12.0f
#define MOTOR_VOLTAGE_LIMIT 6.0f

// Motor driver pins (IFX007T)
#define PIN_U 11
#define PIN_V 10
#define PIN_W 9
#define PIN_EN_U 6
#define PIN_EN_V 5
#define PIN_EN_W 3

// SPI pins for TLE5012B
#define PIN_SPI1_SS0   94
#define PIN_SPI1_MOSI  69
#define PIN_SPI1_MISO  95
#define PIN_SPI1_SCK   68

// Button pins
#define BUTTON1 2  // Close gripper
#define BUTTON2 4  // Open gripper
#define MODE_SWITCH_PIN 7  // Optional toggle switch for SOFT/HARD mode

#endif
