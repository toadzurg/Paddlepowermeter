#ifndef IMU_H
#define IMU_H

#include "PowerModel.h"  // ImuReading

// Two BMI270s on the shaft (see Config.h for addresses, positions and axes).
// Returns a bitmask of the IMUs that started: bit 0 = IMU1, bit 1 = IMU2.
uint8_t initIMU();

// Average the gyros for `samples` readings to remove bias. Paddle must be
// still. Returns false (and keeps zero bias) if it was moving.
bool calibrateGyroBias(int samples = 200);

// Read both IMUs, converted to SI units and rotated into the paddle frame.
// If raw is given, it also receives the readings in each chip's own axes,
// before bias removal (what the SD log records).
void readIMU(ImuReading out[2], ImuReading *raw = nullptr);

// Current gyro bias, rad/s, in each chip's own axes.
void getGyroBias(float bias[2][3]);

#endif  // IMU_H
