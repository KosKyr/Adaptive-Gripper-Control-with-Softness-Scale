#ifndef CONFIG_H
#define CONFIG_H

// === Feature Toggles ===
#define ENABLE_COMMANDER 1
#define ENABLE_READ_ANGLE 0
#define ENABLE_MAGNETIC_SENSOR 1

// === Motor Configuration ===
#define MOTOR_POLE_PAIRS 7
#define MOTOR_PHASE_RESISTANCE 0.24f
#define MOTOR_KV 360
#define MOTOR_PHASE_INDUCTANCE 0.000133f
#define MOTOR_VOLTAGE_ALIGN 2.0f
#define MOTOR_POWER_SUPPLY_VOLTAGE 12.0f
#define MOTOR_VOLTAGE_LIMIT 6.0f

// === Motor Driver Pins ===
#define PIN_U 11
#define PIN_V 10
#define PIN_W 9
#define PIN_EN_U 6
#define PIN_EN_V 5
#define PIN_EN_W 3

// === TLE5012B SPI Pins ===
#define PIN_SPI1_SS0   94
#define PIN_SPI1_MOSI  69
#define PIN_SPI1_MISO  95
#define PIN_SPI1_SCK   68

// === Button Pins ===
#define BUTTON1 2
#define BUTTON2 4
#define MODE_SWITCH_PIN 7

// === Magnetic Sensor Settings (TLx493D) ===
#define CALIBRATION_SAMPLES 20
#define GRIP_FIELD_THRESHOLD 0.6     // mT, total field magnitude
#define GRIP_DELTA_THRESHOLD 0.2     // mT, rate of change
#define FIELD_SAMPLE_INTERVAL 50     // ms

#endif