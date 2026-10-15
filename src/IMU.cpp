#include "IMU.h"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

namespace {
constexpr float DEG2RAD = 0.017453292f;
constexpr float MAX_BIAS_RAD_S = 0.05f;  // more than this at rest = moving

BMI270 imus[2];
bool ok[2] = {false, false};
float gyroBias[2][3] = {};
const cfg::AxisMap *axes[2] = {&cfg::IMU1_AXES, &cfg::IMU2_AXES};

bool startImu(BMI270 &imu, uint8_t addr) {
  if (imu.beginI2C(addr) != BMI2_OK) return false;
  // 200 Hz so every 100 Hz loop pass sees a fresh sample.
  imu.setAccelODR(BMI2_ACC_ODR_200HZ);
  imu.setGyroODR(BMI2_GYR_ODR_200HZ);
  return true;
}

void remap(const cfg::AxisMap &m, const float in[3], float out[3]) {
  for (int k = 0; k < 3; k++) out[k] = m.sign[k] * in[m.axis[k]];
}

bool readRaw(int i, float gyro[3], float accel[3]) {
  if (!ok[i] || imus[i].getSensorData() != BMI2_OK) return false;
  const BMI270_SensorData &d = imus[i].data;
  // Library units: accel in g, gyro in deg/s.
  gyro[0] = d.gyroX * DEG2RAD;
  gyro[1] = d.gyroY * DEG2RAD;
  gyro[2] = d.gyroZ * DEG2RAD;
  accel[0] = d.accelX * cfg::GRAVITY_MS2;
  accel[1] = d.accelY * cfg::GRAVITY_MS2;
  accel[2] = d.accelZ * cfg::GRAVITY_MS2;
  return true;
}
}  // namespace

uint8_t initIMU() {
  ok[0] = startImu(imus[0], cfg::IMU1_I2C_ADDR);
  ok[1] = startImu(imus[1], cfg::IMU2_I2C_ADDR);
  return (ok[0] ? 1 : 0) | (ok[1] ? 2 : 0);
}

void getGyroBias(float bias[2][3]) {
  for (int i = 0; i < 2; i++)
    for (int k = 0; k < 3; k++) bias[i][k] = gyroBias[i][k];
}

bool calibrateGyroBias(int samples) {
  double sum[2][3] = {};
  int n[2] = {0, 0};
  for (int s = 0; s < samples; s++) {
    for (int i = 0; i < 2; i++) {
      float g[3], a[3];
      if (!readRaw(i, g, a)) continue;
      for (int k = 0; k < 3; k++) sum[i][k] += g[k];
      n[i]++;
    }
    delay(5);
  }
  for (int i = 0; i < 2; i++) {
    if (!n[i]) continue;
    for (int k = 0; k < 3; k++) {
      float b = sum[i][k] / n[i];
      if (fabsf(b) > MAX_BIAS_RAD_S) return false;
    }
  }
  for (int i = 0; i < 2; i++)
    for (int k = 0; k < 3; k++) gyroBias[i][k] = n[i] ? sum[i][k] / n[i] : 0;
  return true;
}

void readIMU(ImuReading out[2], ImuReading *raw) {
  for (int i = 0; i < 2; i++) {
    float g[3], a[3];
    out[i].valid = readRaw(i, g, a);
    if (raw) {
      raw[i].valid = out[i].valid;
      for (int k = 0; k < 3; k++) {
        raw[i].gyro[k] = out[i].valid ? g[k] : 0;
        raw[i].accel[k] = out[i].valid ? a[k] : 0;
      }
    }
    if (!out[i].valid) continue;
    for (int k = 0; k < 3; k++) g[k] -= gyroBias[i][k];
    remap(*axes[i], g, out[i].gyro);
    remap(*axes[i], a, out[i].accel);
  }
}
