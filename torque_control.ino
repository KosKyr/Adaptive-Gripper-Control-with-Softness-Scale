#include "config.h"
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include <SimpleFOC.h>

// SPI + Angle Sensor
tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor angleSensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// Motor + Driver
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN_U, PIN_EN_V, PIN_EN_W);

// State + Mode
enum GripperState { IDLE, CLOSING, OPENING, GRIPPED };
GripperState state = IDLE;
enum GripperMode { SOFT, HARD };
GripperMode mode = SOFT;

float ramp_voltage = 0.0;
float ramp_step = 0.05;
float soft_limit = 1.0;
float hard_limit = 3.0;
float hold_voltage = -0.3;

float last_angle = 0;
unsigned long last_check = 0;
bool object_gripped = false;

#if ENABLE_COMMANDER
Commander command = Commander(Serial);
void onTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void onTorquePID(char* cmd) { command.pid(&motor.PID_torque, cmd); }
#endif

#if ENABLE_MAGNETIC_SENSOR
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
double xOffset = 0, yOffset = 0, zOffset = 0;

// For delta thresholding
double lastFieldMag = 0;

void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0, temp;
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    double valX, valY, valZ;
    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX; sumY += valY; sumZ += valZ;
    delay(10);
  }
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
}
#endif

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  angleSensor.init();
  motor.linkSensor(&angleSensor);

  driver.voltage_power_supply = MOTOR_POWER_SUPPLY_VOLTAGE;
  driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.init();
  motor.initFOC();
  Serial.println("Motor ready.");

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);

#if ENABLE_COMMANDER
  command.add('T', onTarget, "target torque");
  command.add('L', onTorquePID, "torque PID");
#endif

#if ENABLE_MAGNETIC_SENSOR
  dut.begin();
  calibrateSensor();
  Serial.println("Magnetic sensor calibrated.");
#endif
}

void loop() {
  mode = digitalRead(MODE_SWITCH_PIN) == HIGH ? HARD : SOFT;
  float max_voltage = (mode == HARD) ? -hard_limit : -soft_limit;

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

  float current_angle = angleSensor.getSensorAngle();
  float field_mag = 0;

  if (millis() - last_check > FIELD_SAMPLE_INTERVAL) {
    float angle_diff = abs(current_angle - last_angle);
    bool stall = angle_diff < 0.002;
    bool pressure = false;

#if ENABLE_MAGNETIC_SENSOR
    double x, y, z;
    dut.setSensitivity(TLx493D_FULL_RANGE_e);
    dut.getMagneticField(&x, &y, &z);
    x -= xOffset; y -= yOffset; z -= zOffset;
    field_mag = sqrt(x*x + y*y + z*z);
    double delta_field = abs(field_mag - lastFieldMag);

    // Live feedback
    Serial.print("Mag: ");
    Serial.print(field_mag, 3);
    Serial.print(" mT | ΔMag: ");
    Serial.print(delta_field, 3);
    Serial.print(" | ");

    pressure = field_mag > GRIP_FIELD_THRESHOLD && delta_field > GRIP_DELTA_THRESHOLD;
    lastFieldMag = field_mag;
#endif

    if (state == CLOSING && (stall || pressure)) {
      object_gripped = true;
      Serial.print("Object gripped via ");
      if (stall) Serial.print("stall ");
      if (pressure) Serial.print("pressure ");
      Serial.println();
    }

    last_angle = current_angle;
    last_check = millis();
  }

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

#if ENABLE_READ_ANGLE
  Serial.print("Angle: ");
  Serial.println(current_angle);
#endif

  motor.loopFOC();
  motor.move(ramp_voltage);

#if ENABLE_COMMANDER
  command.run();
#endif
}