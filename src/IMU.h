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
void readIMU(ImuReading out[2]);

#endif  // IMU_H
