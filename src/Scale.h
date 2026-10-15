#ifndef SCALE_H
#define SCALE_H

#include <stdint.h>

// NAU7802 strain-gauge amplifier. Force is reported in newtons at the blade
// centre of pressure, using a zero offset and calibration factor that are
// kept in flash (see PaddlePowerMeter.ino serial commands).
bool initScale(int32_t zeroOffset, float countsPerN);

// Returns true and sets forceN (and rawCounts, if given) when a new
// conversion is ready. Otherwise leaves them unchanged (the ADC runs slower
// than the main loop).
bool readForce(float &forceN, int32_t *rawCounts = nullptr);

// Blocking calibration helpers (paddle must be held still).
int32_t tareScale();                    // returns new zero offset
float calibrateScale(float knownForceN); // returns new counts per newton

#endif  // SCALE_H
