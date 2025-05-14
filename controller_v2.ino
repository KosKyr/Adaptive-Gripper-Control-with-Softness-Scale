#include <SimpleFOC.h>
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"

// --- TLE5012B Setup (angle sensor) ---
#define PIN_SPI1_SS0 94
#define PIN_SPI1_MOSI 69
#define PIN_SPI1_MISO 95
#define PIN_SPI1_SCK 68

tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// --- Motor & Driver ---
BLDCMotor motor = BLDCMotor(7, 0.24, 360, 0.000133);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 6, 5, 3);

// --- TLx493D 3D Magnetic Sensor (pressure sensor) ---
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
double xOffset = 0, yOffset = 0, zOffset = 0;
float magnetic_z = 0;

// --- Softness Classification Parameters ---
float theta_start = 0;
float softness_score = 0;
bool evaluating_softness = false;
const float GRIP_FORCE = -2.0;
const float MAGNETIC_TRIGGER = 1.0;     // mT to start classification
const float SOFTNESS_THRESHOLD = 5.0;   // score > this = soft

// --- Control ---
float target_voltage = 0;
float last_angle = 0;
float velocity_threshold = 0.2;
int stable_count = 0;
const int stable_threshold = 20;
bool object_gripped = false;
unsigned long last_time = 0;
float initial_open_angle = 0;
bool angle_saved = false;
bool returning_to_open = false;

// --- Commander ---
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 6;
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.001;
  motor.LPF_velocity.Tf = 0.01f;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

  dut.begin(); // no calibration needed
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  command.add('T', doTarget, "target voltage");

  Serial.println("✅ Gripper ready. Press button 1 to grip and classify.");
  delay(1000);
}

// --- Dynamic holding torque based on pressure ---
float adjustHoldingTorque(float z) {
  const float MIN_HOLD = -0.2;
  const float MAX_HOLD = -1.0;
  z = constrain(z, MAGNETIC_TRIGGER, 5.0);
  float norm = (z - MAGNETIC_TRIGGER) / (5.0 - MAGNETIC_TRIGGER);
  return MIN_HOLD + norm * (MAX_HOLD - MIN_HOLD);
}

void loop() {
  motor.loopFOC();

  float current_angle = motor.sensor->getAngle();
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f;
  last_time = current_time;
  float velocity = (dt > 0) ? abs(current_angle - last_angle) / dt : 0.0;
  last_angle = current_angle;

  // --- Read TLx493D Z-axis field ---
  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  magnetic_z = z - zOffset;

  // Save initial open angle
  if (!angle_saved) {
    initial_open_angle = current_angle;
    angle_saved = true;
  }

  // --- Return to initial open position ---
  if (returning_to_open) {
    motor.controller = MotionControlType::angle;
    motor.move(initial_open_angle);

    if (abs(current_angle - initial_open_angle) < 0.05) {
      Serial.println("🔓 Returned to open position.");
      returning_to_open = false;
      motor.controller = MotionControlType::torque;
      target_voltage = 0;
    }

    command.run();
    return;
  }

  // --- Button 1 (Grip + Classify) ---
  if (digitalRead(BUTTON1) == LOW && !object_gripped) {
    if (!evaluating_softness) {
      evaluating_softness = true;
      theta_start = current_angle;
      stable_count = 0;
      Serial.println("🟡 Evaluating softness...");
    }

    target_voltage = GRIP_FORCE;

    float angle_diff = abs(current_angle - theta_start);

    if (magnetic_z > MAGNETIC_TRIGGER) {
      softness_score = angle_diff / magnetic_z;
      stable_count++;

      if (stable_count > stable_threshold) {
        if (softness_score > SOFTNESS_THRESHOLD) {
          Serial.println("🟢 Object classified as: SOFT");
        } else {
          Serial.println("🔵 Object classified as: HARD");
        }

        object_gripped = true;
        evaluating_softness = false;

        // switch to dynamic hold
        target_voltage = adjustHoldingTorque(magnetic_z);

        // optional: softer PID
        motor.PID_velocity.P = 0.15;
        motor.PID_velocity.I = 3.0;
        motor.PID_velocity.D = 0.01;
      }
    }
  }

  // --- Button 2 (Open) ---
  else if (digitalRead(BUTTON2) == LOW && !returning_to_open) {
    Serial.println("🔁 Releasing object...");
    object_gripped = false;
    stable_count = 0;
    returning_to_open = true;
    evaluating_softness = false;

    // restore default PID
    motor.PID_velocity.P = 0.3;
    motor.PID_velocity.I = 5.0;
    motor.PID_velocity.D = 0.001;

    return;
  }

  // --- Default idle behavior ---
  else if (!object_gripped && !evaluating_softness) {
    target_voltage = 0;
  }

  motor.move(target_voltage);
  command.run();
}
