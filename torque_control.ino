#include "config.h"
#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include <SimpleFOC.h>

// --- SPI + Angle Sensor ---
tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor angleSensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

// --- Motor + Driver ---
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV, MOTOR_PHASE_INDUCTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN_U, PIN_EN_V, PIN_EN_W);

// --- PID for magnetic pressure ---
struct PID {
  float Kp, Ki, Kd;
  float prev_error;
  float integral;
  unsigned long last_time;

  PID(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), prev_error(0), integral(0), last_time(0) {}

  float compute(float setpoint, float measurement) {
    unsigned long now = millis();
    float dt = (last_time == 0) ? 0.01f : (now - last_time) / 1000.0f;
    last_time = now;

    float error = setpoint - measurement;
    integral += error * dt;
    integral = constrain(integral, -10.0f, 10.0f);

    float derivative = (dt > 0) ? (error - prev_error) / dt : 0;
    prev_error = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    return output;
  }
};

PID pressurePID(1.0, 0.01, 0.05);
float target_pressure = 2.0;  // mT, desired magnetic field

// --- State ---
enum GripperState { IDLE, CLOSING, APPLYING_FORCE, OPENING };
GripperState state = IDLE;

// --- Control variables ---
float ramp_voltage = 0.0;
float ramp_step    = 0.05;
float hard_limit   = 3.0;
float hold_voltage_soft = -0.1;
float hold_voltage_hard = -0.3;
float minimum_torque = -0.3;

bool contact_detected = false;
bool button1_pressed = false;
bool button2_pressed = false;

#if ENABLE_COMMANDER
Commander command = Commander(Serial);
void onTarget(char* cmd) { command.scalar(&target_pressure, cmd); }
#endif

#if ENABLE_MAGNETIC_SENSOR
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
double xOffset = 0, yOffset = 0, zOffset = 0;
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

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

#if ENABLE_COMMANDER
  command.add('T', onTarget, "target pressure");
#endif

#if ENABLE_MAGNETIC_SENSOR
  dut.begin();
  calibrateSensor();
  Serial.println("Magnetic sensor calibrated.");
#endif
}

void loop() {
  static bool last_button1 = HIGH;
  static bool last_button2 = HIGH;

  bool current_button1 = digitalRead(BUTTON1);
  bool current_button2 = digitalRead(BUTTON2);

  if (last_button1 == HIGH && current_button1 == LOW) {
    button1_pressed = true;
  }
  if (last_button2 == HIGH && current_button2 == LOW) {
    button2_pressed = true;
  }
  last_button1 = current_button1;
  last_button2 = current_button2;

  float field_mag = 0.0;
#if ENABLE_MAGNETIC_SENSOR
  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  x -= xOffset; y -= yOffset; z -= zOffset;
  field_mag = sqrt(x * x + y * y + z * z);
#endif

  switch (state) {
    case IDLE:
      if (button1_pressed) {
        state = CLOSING;
        button1_pressed = false;
        contact_detected = false;
        Serial.println("State: CLOSING");
      } else if (button2_pressed) {
        state = OPENING;
        button2_pressed = false;
        Serial.println("State: OPENING");
      }
      ramp_voltage = 0.0;
      break;

    case CLOSING:
      if (field_mag > target_pressure) {
        contact_detected = true;
        state = APPLYING_FORCE;
        Serial.println("Object detected, switching to PID control");
      } else {
        ramp_voltage = -minimum_torque;
      }
      break;

    case APPLYING_FORCE: {
      float pid_out = pressurePID.compute(target_pressure, field_mag);
      ramp_voltage = constrain(pid_out, -hard_limit, 0);
      break;
    }

    case OPENING:
      ramp_voltage = 2.0;
      if (!current_button2) {
        state = IDLE;
        ramp_voltage = 0.0;
        Serial.println("Returned to IDLE");
      }
      break;
  }

  motor.loopFOC();
  motor.move(ramp_voltage);

#if ENABLE_COMMANDER
  command.run();
#endif
}
