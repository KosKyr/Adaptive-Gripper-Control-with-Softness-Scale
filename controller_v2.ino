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

// --- TLx493D Magnetic Sensor (3D field sensor) ---
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
double xOffset = 0, yOffset = 0, zOffset = 0;
float magnetic_z = 0, magnetic_x = 0, magnetic_y = 0, magnetic_mag = 0;

// --- Softness Classification Parameters ---
float theta_start = 0;
bool evaluating_softness = false;
const float GRIP_FORCE = -3.5;
const float MAGNETIC_TRIGGER = 1.0;
const int stable_threshold = 5;

// --- Rate-of-Deformation Buffer ---
const int RATE_BUFFER_SIZE = 10;
float angle_history[RATE_BUFFER_SIZE];
float mag_history[RATE_BUFFER_SIZE];
float rate_history[RATE_BUFFER_SIZE];
int rate_index = 0;
bool rate_buffer_full = false;

// --- Control ---
float target_voltage = 0;
float last_angle = 0;
int stable_count = 0;
unsigned long last_time = 0;
float initial_open_angle = 0;
bool angle_saved = false;
bool returning_to_open = false;

// --- State Flags ---
bool object_gripped = false;
bool grip_requested = false;
bool button1_last_state = HIGH;

// --- Commander Interface Setup ---
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }
void doGrip(char* cmd) { grip_requested = true; }

// GUI/Open command resets control states and returns motor to initial open position
void doOpen(char* cmd) {
  Serial.println("🔁 Release requested from GUI...");
  object_gripped = false;
  evaluating_softness = false;
  stable_count = 0;
  grip_requested = false;
  returning_to_open = true;

  // Restore PID values for smoother return
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.001;
}

// Receive new PID parameters from GUI
void doPID(char* cmd) {
  float p = atof(strtok(cmd, " "));
  float i = atof(strtok(NULL, " "));
  float d = atof(strtok(NULL, " "));
  motor.PID_velocity.P = p;
  motor.PID_velocity.I = i;
  motor.PID_velocity.D = d;
  Serial.println("✅ PID updated from GUI");
}

// Average magnetic readings for offset calibration
void calibratePressureSensor(int samples = 20) {
  double sumX = 0, sumY = 0, sumZ = 0, x, y, z;
  Serial.print("📏 Calibrating TLx493D Z-offset...");
  for (int i = 0; i < samples; i++) {
    dut.setSensitivity(TLx493D_FULL_RANGE_e);
    dut.getMagneticField(&x, &y, &z);
    sumZ += z;
    sumX += x;
    sumY += y;
    delay(5);
  }
  zOffset = sumZ / samples;
  xOffset = sumX / samples;
  yOffset = sumY / samples;
  Serial.print(" Done. zOffset = ");
  Serial.println(zOffset, 4);
}

// Map Z-field magnitude to holding torque
float adjustHoldingTorque(float z) {
  const float MIN_HOLD = -0.2;
  const float MAX_HOLD = -3;
  z = constrain(z, MAGNETIC_TRIGGER, 5.0);
  float norm = (z - MAGNETIC_TRIGGER) / (5.0 - MAGNETIC_TRIGGER);
  return MIN_HOLD + norm * (MAX_HOLD - MIN_HOLD);
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Initialize sensor and motor
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

  dut.begin();
  calibratePressureSensor();

  // Setup buttons and commander commands
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  command.add('T', doTarget, "target voltage");
  command.add('G', doGrip, "GUI grip");
  command.add('O', doOpen, "GUI open");
  command.add('P', doPID, "Set PID via GUI");

  Serial.println("✅ Gripper ready. Press button 1 to grip and classify.");
  delay(1000);
}

void loop() {
  motor.loopFOC();

  float current_angle = motor.sensor->getAngle();
  unsigned long current_time = millis();
  last_time = current_time;

  // Read and normalize magnetic field
  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  magnetic_z = z - zOffset;
  magnetic_x = x - xOffset;
  magnetic_y = y - yOffset;
  magnetic_mag = sqrt(magnetic_z * magnetic_z + magnetic_y * magnetic_y + magnetic_x * magnetic_x);

  // Debug print
  Serial.print("A: "); Serial.print(current_angle, 4);
  Serial.print(" Z: "); Serial.print(magnetic_z, 3);
  Serial.print(" V: "); Serial.println(velocity, 4);

  // Save open angle once
  if (!angle_saved) {
    initial_open_angle = current_angle;
    angle_saved = true;
  }

  // Handle release to open state
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

  // Check for grip button press
  bool button1_state = digitalRead(BUTTON1);
  if (button1_last_state == HIGH && button1_state == LOW && !object_gripped && !evaluating_softness) {
    grip_requested = true;
  }
  button1_last_state = button1_state;

  // Grip + classify softness logic
  if (grip_requested && !object_gripped) {
    if (!evaluating_softness) {
      evaluating_softness = true;
      theta_start = current_angle;
      stable_count = 0;
      rate_index = 0;
      rate_buffer_full = false;
      Serial.println("🟡 Evaluating softness...");
    }

    target_voltage = GRIP_FORCE;
    float angle_diff = abs(current_angle - theta_start);

    if (magnetic_mag > MAGNETIC_TRIGGER) {
      int prev_index = (rate_index == 0) ? RATE_BUFFER_SIZE - 1 : rate_index - 1;
      float d_theta = angle_diff - angle_history[prev_index];
      float d_mag = magnetic_mag - mag_history[prev_index];
      float rate = (abs(d_mag) > 0.01) ? d_theta / d_mag : 0;

      angle_history[rate_index] = angle_diff;
      mag_history[rate_index] = magnetic_mag;
      rate_history[prev_index] = rate;

      rate_index++;
      if (rate_index >= RATE_BUFFER_SIZE) {
        rate_buffer_full = true;
        rate_index = 0;
      }

      stable_count++;

      // Perform classification after collecting enough stable samples
      if (stable_count > stable_threshold && rate_buffer_full) {
        int oldest = (rate_index + 1) % RATE_BUFFER_SIZE;
        int newest = (rate_index - 1 + RATE_BUFFER_SIZE) % RATE_BUFFER_SIZE;
        float trend = rate_history[newest] - rate_history[oldest];

        if (trend < -0.01) {
          Serial.println("Class: SOFT");
        } else {
          Serial.println("Class: HARD");
        }

        object_gripped = true;
        evaluating_softness = false;
        grip_requested = false;
        rate_index = 0;
        rate_buffer_full = false;

        target_voltage = adjustHoldingTorque(magnetic_mag);

        // Use gentler PID for holding
        motor.PID_velocity.P = 0.15;
        motor.PID_velocity.I = 3.0;
        motor.PID_velocity.D = 0.01;
      }
    }
  }

  // Handle release button
  if (digitalRead(BUTTON2) == LOW && !returning_to_open) {
    Serial.println("🔁 Releasing object...");
    object_gripped = false;
    evaluating_softness = false;
    stable_count = 0;
    grip_requested = false;
    returning_to_open = true;

    // Reset PID for return movement
    motor.PID_velocity.P = 0.3;
    motor.PID_velocity.I = 5.0;
    motor.PID_velocity.D = 0.001;
    return;
  }

  // Idle condition (no grip or release requested)
  if (!object_gripped && !evaluating_softness && !grip_requested) {
    target_voltage = 0;
  }

  motor.move(target_voltage);
  command.run();
}
