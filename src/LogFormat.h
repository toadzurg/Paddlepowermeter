#ifndef LOGFORMAT_H
#define LOGFORMAT_H

#include <stdint.h>

// Binary sensor log written to the SD card (and read by tools/).
//
// File = one LogHeader, then fixed-size LogRecords at the sample rate.
// Records hold sensor readings as the drivers report them: SI units, but in
// each IMU's own axes and before gyro-bias removal. The header holds the
// bias, axis maps, geometry and calibration in force at the time, so a log
// can be re-processed later with corrected settings. All fields are
// little-endian (native on ESP32 and x86/ARM PCs).

namespace logfmt {

constexpr uint32_t MAGIC = 0x474C5050;  // "PPLG"
constexpr uint16_t VERSION = 1;

// LogRecord.flags
constexpr uint8_t F_IMU1_VALID  = 1 << 0;
constexpr uint8_t F_IMU2_VALID  = 1 << 1;
constexpr uint8_t F_FORCE_FRESH = 1 << 2;  // new NAU7802 conversion this sample
constexpr uint8_t F_BOAT_FRESH  = 1 << 3;  // boat node packet received recently

struct __attribute__((packed)) LogHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;   // sizeof(LogHeader), for forward compatibility
  uint16_t recordSize;   // sizeof(LogRecord)
  float sampleHz;
  // Geometry (Config.h), metres along the shaft from IMU1
  float imu2PosM, bladeCopPosM, pivotPriorPosM;
  // Axis maps: paddle[i] = sign[i] * sensor[axis[i]]
  uint8_t axis[2][3];
  int8_t sign[2][3];
  float gyroBias[2][3];  // rad/s, sensor axes, subtract before remapping
  // Calibration
  int32_t forceZeroOffset;  // ADC counts
  float forceCountsPerN;
  float pulsesPerM;         // impeller, 0 = uncalibrated
  uint32_t startMillis;     // millis() when the log started
  char firmware[16];        // build date
};

struct __attribute__((packed)) LogRecord {
  uint32_t tUs;             // micros(); wraps every ~71 min
  int32_t forceRaw;         // NAU7802 counts (last conversion)
  uint8_t flags;
  float gyro[2][3];         // rad/s, sensor axes, bias NOT removed
  float accel[2][3];        // m/s², sensor axes
  float boatPulseHz;
  uint32_t boatPulseCount;
  // A few firmware outputs, to compare against an offline replay
  float forceN, omega, pivotPosM, instPowerW;
};

static_assert(sizeof(LogHeader) == 94, "LogHeader layout changed");
static_assert(sizeof(LogRecord) == 81, "LogRecord layout changed");

}  // namespace logfmt

#endif  // LOGFORMAT_H
