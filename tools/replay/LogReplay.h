#ifndef LOG_REPLAY_H
#define LOG_REPLAY_H

// Reads SD-card logs (src/LogFormat.h) on a PC and rebuilds the exact
// PaddleSamples the firmware fed its PowerModel, so a recorded session can
// be re-run offline with the same or improved code.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "LogFormat.h"
#include "PowerModel.h"

class LogReader {
public:
  ~LogReader() { if (f_) std::fclose(f_); }

  bool open(const char *path, std::string &err) {
    f_ = std::fopen(path, "rb");
    if (!f_) { err = "cannot open file"; return false; }
    if (std::fread(&h_, sizeof h_, 1, f_) != 1) { err = "file too short"; return false; }
    if (h_.magic != logfmt::MAGIC) { err = "not a paddle log (bad magic)"; return false; }
    if (h_.version != logfmt::VERSION) { err = "unsupported log version"; return false; }
    if (h_.recordSize != sizeof(logfmt::LogRecord)) { err = "record size mismatch"; return false; }
    if (h_.headerSize < sizeof h_) { err = "header too short"; return false; }
    // A newer firmware may append header fields; skip what we don't know.
    std::fseek(f_, h_.headerSize, SEEK_SET);
    return true;
  }

  const logfmt::LogHeader &header() const { return h_; }

  // False at end of file. A partial record left by a power cut is ignored.
  bool next(logfmt::LogRecord &r) {
    return std::fread(&r, sizeof r, 1, f_) == 1;
  }

private:
  std::FILE *f_ = nullptr;
  logfmt::LogHeader h_ = {};
};

// Turns records into PaddleSamples. Calibration defaults to the header's
// but can be overridden, e.g. after a better force calibration.
class SampleBuilder {
public:
  explicit SampleBuilder(const logfmt::LogHeader &h)
    : h_(h), countsPerN_(h.forceCountsPerN), zero_(h.forceZeroOffset),
      pulsesPerM_(h.pulsesPerM) {}

  void setForceCalibration(int32_t zero, float countsPerN) {
    zero_ = zero;
    countsPerN_ = countsPerN;
  }
  void setPulsesPerM(float ppm) { pulsesPerM_ = ppm; }

  PaddleSample next(const logfmt::LogRecord &r) {
    PaddleSample s = {};
    if (first_) {
      s.dt = 0;
    } else {
      uint32_t dUs = r.tUs - prevUs_;  // unsigned: survives the µs wrap
      s.dt = dUs * 1e-6f;
      elapsedUs_ += dUs;
    }
    first_ = false;
    prevUs_ = r.tUs;
    s.timeMs = h_.startMillis + (uint32_t)(elapsedUs_ / 1000);

    // The firmware reports 0 N until the ADC's first conversion.
    if (r.flags & logfmt::F_FORCE_FRESH) haveForce_ = true;
    s.forceN = (haveForce_ && countsPerN_ != 0)
                   ? (r.forceRaw - zero_) / countsPerN_ : 0.0f;

    for (int i = 0; i < 2; i++) {
      ImuReading &m = s.imu[i];
      m.valid = r.flags & (i == 0 ? logfmt::F_IMU1_VALID : logfmt::F_IMU2_VALID);
      for (int k = 0; k < 3; k++) {
        uint8_t ax = h_.axis[i][k];
        float g = r.gyro[i][ax] - h_.gyroBias[i][ax];
        m.gyro[k] = h_.sign[i][k] * g;
        m.accel[k] = h_.sign[i][k] * r.accel[i][ax];
      }
    }

    bool boatFresh = r.flags & logfmt::F_BOAT_FRESH;
    s.boatSpeedValid = boatFresh && pulsesPerM_ > 0;
    s.boatSpeedMps = s.boatSpeedValid ? r.boatPulseHz / pulsesPerM_ : 0;
    return s;
  }

  double elapsedSeconds() const { return elapsedUs_ * 1e-6; }

private:
  logfmt::LogHeader h_;
  float countsPerN_;
  int32_t zero_;
  float pulsesPerM_;
  bool first_ = true, haveForce_ = false;
  uint32_t prevUs_ = 0;
  uint64_t elapsedUs_ = 0;
};

#endif  // LOG_REPLAY_H
