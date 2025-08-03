#include "Scale.h"
#include "IMU.h"
#include "Kalman.h"
#include "PowerModel.h"
#include "Battery.h"
#include "BLEManager.h"

PowerModel powerModel(1.0f, 0.01f, 1.0f, 0.001f, 0.1f, 1.0f);

void setup() {
  Serial.begin(115200);
  initScale();
  initIMU();
  initBattery();
  initBLE();
}

void loop() {
  uint32_t now = millis();
  updateScale();
  updateIMU();
  powerModel.update(now);

  float instW = powerModel.getInstantaneousPower();
  float strokeW = powerModel.getStrokePower();
  float spm = powerModel.getStrokeRate();
  int batt = getBatteryLevel();

  sendBLEData(instW, spm, batt);
  delay(50);
}
