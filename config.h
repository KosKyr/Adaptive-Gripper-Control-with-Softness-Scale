#ifndef CONFIG_H
#define CONFIG_H

// === Feature Toggles ===
#define ENABLE_COMMANDER         1
#define ENABLE_MAGNETIC_SENSOR   1

// === Motor Configuration ===
#define MOTOR_POLE_PAIRS         7
#define MOTOR_PHASE_RESISTANCE   0.24f      // Ohms
#define MOTOR_KV                 360        // RPM/V
#define MOTOR_PHASE_INDUCTANCE   0.000133f  // Henrys
#define MOTOR_VOLTAGE_ALIGN      4.0f       // Volts
#define MOTOR_POWER_SUPPLY_VOLTAGE 12.0f    // Volts
#define MOTOR_VOLTAGE_LIMIT      6.0f       // Max motor command voltage

// === Motor Driver Pins ===
#define PIN_U                    11
#define PIN_V                    10
#define PIN_W                    9
#define PIN_EN_U                 6
#define PIN_EN_V                 5
#define PIN_EN_W                 3

// === TLE5012B SPI Pins ===
#define PIN_SPI1_SS0             94
#define PIN_SPI1_MOSI            69
#define PIN_SPI1_MISO            95
#define PIN_SPI1_SCK             68

// === Magnetic Sensor Settings ===
#define CALIBRATION_SAMPLES      20
#define FIELD_SAMPLE_INTERVAL    50    // milliseconds

#endif // CONFIG_H
