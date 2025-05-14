#include "config.h"
#include "TLE5012Sensor.h"
#include <SimpleFOC.h>

// SPI interface
tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor angleSensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// Motor and driver
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN_U, PIN_EN_V, PIN_EN_W);

// Gripper state
enum GripperState { IDLE, CLOSING, OPENING, GRIPPED };
GripperState state = IDLE;

// Grip modes
enum GripperMode { SOFT, HARD };
GripperMode mode = SOFT;

// Control variables
float ramp_voltage = 0.0;
float ramp_step = 0.05;
float soft_limit = 1.0;
float hard_limit = 3.0;
float hold_voltage = -0.3;

// For stall detection
float last_angle = 0;
unsigned long last_check = 0;
bool object_gripped = false;

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Sensor
  angleSensor.init();
  motor.linkSensor(&angleSensor);

  // Driver
  driver.voltage_power_supply = MOTOR_POWER_SUPPLY_VOLTAGE;
  driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }

  motor.linkDriver(&driver);
  motor.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.init();
  motor.initFOC();
  Serial.println("Motor ready.");

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);  // HIGH = HARD, LOW = SOFT
}

void loop() {
  // Update mode
  mode = digitalRead(MODE_SWITCH_PIN) == HIGH ? HARD : SOFT;
  float max_voltage = (mode == HARD) ? -hard_limit : -soft_limit;

  // State machine transitions
  if (digitalRead(BUTTON1) == LOW) {
    state = CLOSING;
    object_gripped = false;
  } else if (digitalRead(BUTTON2) == LOW) {
    state = OPENING;
    object_gripped = false;
  } else if (state == CLOSING && object_gripped) {
    state = GRIPPED;
  } else if (state != GRIPPED) {
    state = IDLE;
  }

  // Stall detection for gripping
  float current_angle = angleSensor.getSensorAngle();
  if (millis() - last_check > 50) {
    float angle_diff = abs(current_angle - last_angle);
    if (state == CLOSING && angle_diff < 0.002) {
      object_gripped = true;
      Serial.println("Object gripped.");
    }
    last_angle = current_angle;
    last_check = millis();
  }

  // Voltage ramp logic
  switch (state) {
    case CLOSING:
      if (ramp_voltage > max_voltage)
        ramp_voltage -= ramp_step;
      break;
    case OPENING:
      if (ramp_voltage < -max_voltage)
        ramp_voltage += ramp_step;
      break;
    case GRIPPED:
      ramp_voltage = hold_voltage;
      break;
    case IDLE:
    default:
      ramp_voltage = 0;
      break;
  }

  // Run motor FOC loop
  motor.loopFOC();
  motor.move(ramp_voltage);

#if ENABLE_READ_ANGLE
  Serial.println(current_angle);
#endif
}
