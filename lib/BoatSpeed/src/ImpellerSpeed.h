#ifndef IMPELLER_SPEED_H
#define IMPELLER_SPEED_H

#include <stdint.h>

// Turns impeller pulse timestamps into a pulse rate.
//
// Rate comes from the period of the last few pulses rather than from
// counting pulses in a fixed window: an impeller gives only tens of pulses
// per second, so counting would be too coarse to see the boat surge within
// a stroke. Timestamps are uint32 microseconds and may wrap (every ~71 min);
// only differences are used, so wrap is harmless.
class ImpellerSpeed {
public:
  // minIntervalUs: gaps shorter than this are electrical glitches.
  // timeoutUs: no pulse for this long means the boat is stopped.
  // window: number of most recent pulse intervals to average (1..8).
  ImpellerSpeed(uint32_t minIntervalUs, uint32_t timeoutUs, uint8_t window);

  // Feed one pulse. Returns false if it was rejected as a glitch.
  bool onPulse(uint32_t tUs);

  // Pulses per second at time nowUs (0 when stopped).
  float rateHz(uint32_t nowUs) const;

  uint32_t count() const { return count_; }        // accepted pulses (wraps)
  uint32_t rejected() const { return rejected_; }  // glitches dropped
  uint32_t usSinceLastPulse(uint32_t nowUs) const;

private:
  static constexpr uint8_t MAX_WINDOW = 8;

  uint32_t minIntervalUs_, timeoutUs_;
  uint8_t window_;
  uint32_t t_[MAX_WINDOW + 1];  // ring of the newest window_+1 timestamps
  uint8_t head_;                // index of the newest timestamp
  uint8_t n_;                   // timestamps stored
  uint32_t count_, rejected_;
};

#endif  // IMPELLER_SPEED_H
