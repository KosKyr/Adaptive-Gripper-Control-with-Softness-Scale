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
float velocity_threshold = 0.5; // [rad/s] stop detection threshold
int stable_count = 0;
const int stable_threshold = 30;
bool object_gripped = false;
unsigned long last_time = 0;

// Commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Sensor setup
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver setup
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // FOC + torque control mode
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 6;

  // PID settings (used for velocity tracking internally)
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
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

  // --- Read button input for voltage control ---
#if ENABLE_MAGNETIC_SENSOR
  if (digitalRead(BUTTON1) == LOW && !object_gripped) {
    target_voltage = -3;  // close
  } else if (digitalRead(BUTTON2) == LOW && !object_gripped) {
    target_voltage = 3;   // open
  } else {
    target_voltage = 0;   // hold
  }
#endif

  // --- Velocity-based stiffness detection ---
  float current_angle = motor.sensor->getAngle();
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f;
  last_time = current_time;

  float velocity = (dt > 0) ? abs(current_angle - last_angle) / dt : 0.0;
  last_angle = current_angle;

  // If voltage applied but little movement = object gripped
  if (abs(target_voltage) > 0.1 && velocity < velocity_threshold) {
    stable_count++;
    if (stable_count > stable_threshold && !object_gripped) {
      object_gripped = true;
      target_voltage = 0;
      motor.move(0);  // Hold position
      Serial.println("🟢 Object gripped. Holding position.");

      // Softer PID for holding
      motor.PID_velocity.P = 0.1;
      motor.PID_velocity.I = 5;
      motor.PID_velocity.D = 0.01;
    }
  } else {
    stable_count = 0;
    if (object_gripped && abs(target_voltage) > 0.1) {
      object_gripped = false;
      Serial.println("🔄 Grip released. Resuming movement.");

      // Restore default PID
      motor.PID_velocity.P = 0.2;
      motor.PID_velocity.I = 10;
      motor.PID_velocity.D = 0;
    }
  }

  // Apply voltage if not locked
  if (!object_gripped) {
    motor.move(target_voltage);
  }

  command.run();
}
