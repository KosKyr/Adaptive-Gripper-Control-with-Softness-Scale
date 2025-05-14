#ifndef CONFIG_H
#define CONFIG_H

#define ENABLE_MAGNETIC_SENSOR true

#define ENABLE_COMMANDER false

#define ENABLE_READ_ANGLE false

#endif // CONFIG_H



#include "TLE5012Sensor.h"
#include "TLx493D_inc.hpp"
#include "config.h"
#include <SimpleFOC.h>

#define PIN_SPI1_SS0 94
#define PIN_SPI1_MOSI 69
#define PIN_SPI1_MISO 95
#define PIN_SPI1_SCK 68

tle5012::SPIClass3W tle5012::SPI3W1(2);
TLE5012Sensor tle5012Sensor(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK);

BLDCMotor motor = BLDCMotor(7, 0.24, 360, 0.000133);
const int U = 11, V = 10, W = 9, EN_U = 6, EN_V = 5, EN_W = 3;
BLDCDriver3PWM driver = BLDCDriver3PWM(U, V, W, EN_U, EN_V, EN_W);

float target_voltage = -1;

#if ENABLE_MAGNETIC_SENSOR
using namespace ifx::tlx493d;
TLx493D_A2B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);
const int CALIBRATION_SAMPLES = 20;
double xOffset = 0, yOffset = 0, zOffset = 0;
#endif

#if ENABLE_COMMANDER
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_voltage, cmd); }
#endif

void calibrateSensor() {
  double sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    double temp, valX, valY, valZ;
    dut.getMagneticFieldAndTemperature(&valX, &valY, &valZ, &temp);
    sumX += valX; sumY += valY; sumZ += valZ;
    delay(10);
  }
  xOffset = sumX / CALIBRATION_SAMPLES;
  yOffset = sumY / CALIBRATION_SAMPLES;
  zOffset = sumZ / CALIBRATION_SAMPLES;
}

float readGripStrength() {
  double x, y, z;
  dut.getMagneticField(&x, &y, &z);
  x -= xOffset; y -= yOffset; z -= zOffset;
  return sqrt(x * x + y * y + z * z);
}

void autoTunePID() {
  float bestP = 0, bestI = 0, bestD = 0;
  float bestScore = 0;

  for (float p = 0.2; p <= 3.0; p += 0.5) {
    for (float i = 0.0; i <= 0.5; i += 0.1) {
      for (float d = 0.0; d <= 0.5; d += 0.1) {

        motor.PID_current_q.P = p;
        motor.PID_current_q.I = i;
        motor.PID_current_q.D = d;

        motor.move(-3);
        delay(300);

        float score = readGripStrength();

        Serial.print("P = "); Serial.print(p);
        Serial.print(" I = "); Serial.print(i);
        Serial.print(" D = "); Serial.print(d);
        Serial.print(" Score = "); Serial.println(score);

        if (score > bestScore) {
          bestScore = score;
          bestP = p;
          bestI = i;
          bestD = d;
        }

        motor.move(0);
        delay(300);
      }
    }
  }

  motor.PID_current_q.P = bestP;
  motor.PID_current_q.I = bestI;
  motor.PID_current_q.D = bestD;

  Serial.print("\n✅ Best PID values set:\nP = "); Serial.println(bestP);
  Serial.print("I = "); Serial.println(bestI);
  Serial.print("D = "); Serial.println(bestD);
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  tle5012Sensor.init();
  motor.linkSensor(&tle5012Sensor);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 2;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();
  Serial.println(F("Motor ready."));

#if ENABLE_MAGNETIC_SENSOR
  dut.begin();
  calibrateSensor();
  Serial.println("3D magnetic sensor Calibration completed.");
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
#endif

#if ENABLE_COMMANDER
  command.add('T', doTarget, "target voltage");
  Serial.println(F("Set the target voltage using serial terminal:"));
#endif
  delay(1000);
  autoTunePID();
  Serial.println("Setup done.\n");
}

void loop() {
#if ENABLE_MAGNETIC_SENSOR
  if (digitalRead(BUTTON1) == LOW) {
    target_voltage = -3;
  } else if (digitalRead(BUTTON2) == LOW) {
    target_voltage = 3;
  } else {
    target_voltage = 0;
  }

  double x, y, z;
  dut.setSensitivity(TLx493D_FULL_RANGE_e);
  dut.getMagneticField(&x, &y, &z);
  x -= xOffset; y -= yOffset; z -= zOffset;
  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.println(z);
#endif

  tle5012Sensor.update();
#if ENABLE_READ_ANGLE
  Serial.println(tle5012Sensor.getSensorAngle());
#endif

  motor.loopFOC();
  motor.move(target_voltage);

#if ENABLE_COMMANDER
  command.run();
#endif
}