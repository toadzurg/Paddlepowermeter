#include "Scale.h"
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

namespace {
NAU7802 scale;
bool ok = false;
}

bool initScale(int32_t zeroOffset, float countsPerN) {
  ok = scale.begin(Wire);
  if (!ok) return false;
  scale.setGain(NAU7802_GAIN_128);
  scale.setSampleRate(NAU7802_SPS_80);
  scale.calibrateAFE();
  scale.setZeroOffset(zeroOffset);
  scale.setCalibrationFactor(countsPerN);
  return true;
}

bool readForce(float &forceN, int32_t *rawCounts) {
  if (!ok || !scale.available()) return false;
  float cal = scale.getCalibrationFactor();
  if (cal == 0) return false;
  int32_t counts = scale.getReading();
  if (rawCounts) *rawCounts = counts;
  forceN = (counts - scale.getZeroOffset()) / cal;
  return true;
}

int32_t tareScale() {
  if (!ok) return 0;
  scale.calculateZeroOffset(64, 2000);  // 64 samples @ 80 SPS = 0.8 s
  return scale.getZeroOffset();
}

float calibrateScale(float knownForceN) {
  if (!ok || knownForceN <= 0) return scale.getCalibrationFactor();
  scale.calculateCalibrationFactor(knownForceN, 64, 2000);
  return scale.getCalibrationFactor();
}
