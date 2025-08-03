#include "Scale.h"
#include <Wire.h>

NAU7802 myScale;

void initScale() {
  Wire.begin();
  myScale.begin();
  myScale.setGain(128);
  myScale.setSampleRate(NAU7802_SPS_40);
  myScale.calibrateAFE();
}

void updateScale() {
  // No-op or future filtering
}

float getForce() {
  return myScale.getReading();
}
