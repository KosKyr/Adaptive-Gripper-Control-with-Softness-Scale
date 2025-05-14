#include <SimpleFOC.h>
#include "TLE5012Sensor.h"
#include "config.h"

// SPI pins for TLE5012B sensor
#define PIN_SPI1_SS0 94
#define PIN_SPI1_MOSI 69
#define PIN_SPI1_MISO 95
#define PIN_SPI1_SCK 68

tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// BLDC motor & driver
BLDCMotor motor = BLDCMotor(7, 0.24, 360, 0.000133);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 6, 5, 3);

// Control variables
float target_voltage = 0;
float last_angle = 0;
float velocity_threshold = 0.2; // Stricter object detection
int stable_count = 0;
const int stable_threshold = 40;
bool object_gripped = false;
unsigned long last_time = 0;

// Return-to-position
float initial_open_angle = 0;
bool angle_saved = false;
bool returning_to_open = false;

// Commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Sensor
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Motor configuration
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 6;

  // PID tuning
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.001;
  motor.LPF_velocity.Tf = 0.01f;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

  command.add('T', doTarget, "target voltage");

#if ENABLE_MAGNETIC_SENSOR
  Serial.println("3D magnetic sensor Calibration completed.");
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
#endif

  Serial.println(F("Motor ready."));
  Serial.println(F("Use buttons to open/close gripper."));
  delay(1000);
}

void loop() {
  motor.loopFOC();

  float current_angle = motor.sensor->getAngle();
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f;
  last_time = current_time;
  float velocity = (dt > 0) ? abs(current_angle - last_angle) / dt : 0.0;
  last_angle = current_angle;

#if ENABLE_MAGNETIC_SENSOR
  // Save the initial open angle once
  if (!angle_saved) {
    initial_open_angle = current_angle;
    angle_saved = true;
  }

  // --- Handle return to initial position ---
  if (returning_to_open) {
    motor.controller = MotionControlType::angle;
    motor.move(initial_open_angle);

    if (abs(current_angle - initial_open_angle) < 0.05) {  // ~3 degrees
      Serial.println("✅ Reached initial open position.");
      returning_to_open = false;

      motor.controller = MotionControlType::torque;
      target_voltage = 0;
    }

    // skip rest of loop while returning
    command.run();
    return;
  }

  // --- Button handling ---
  if (digitalRead(BUTTON1) == LOW && !object_gripped) {
    target_voltage = -3;  // close
  } else if (digitalRead(BUTTON2) == LOW && !returning_to_open) {
    Serial.println("🔁 Returning to initial open position...");
    object_gripped = false;
    stable_count = 0;
    returning_to_open = true;
    return;
  } else if (!object_gripped) {
    target_voltage = 0;  // idle hold
  }
#endif

  // --- Stiffness-based grip detection ---
  if (abs(target_voltage) > 0.2 && velocity < velocity_threshold) {
    stable_count++;
    if (stable_count > stable_threshold && !object_gripped) {
      object_gripped = true;
      target_voltage = -0.3;  // firm hold torque
      Serial.println("🟢 Object gripped. Holding firmly.");

      // Softer PID for holding
      motor.PID_velocity.P = 0.15;
      motor.PID_velocity.I = 3.0;
      motor.PID_velocity.D = 0.01;
    }
  } else if(digitalRead(BUTTON2) == LOW && returning_to_open){
    stable_count = 0;
    if (object_gripped && abs(target_voltage) > 0.2) {
      object_gripped = false;
      Serial.println("🔄 Grip released. Resuming movement.");

      // Restore default PID
      motor.PID_velocity.P = 0.3;
      motor.PID_velocity.I = 5.0;
      motor.PID_velocity.D = 0.001;
    }
  }

  // --- Motor torque output ---
  if (!object_gripped) {
    motor.move(target_voltage);
  }

  command.run();
}
