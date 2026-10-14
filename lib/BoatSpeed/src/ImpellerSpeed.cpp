#include "ImpellerSpeed.h"

ImpellerSpeed::ImpellerSpeed(uint32_t minIntervalUs, uint32_t timeoutUs,
                             uint8_t window)
  : minIntervalUs_(minIntervalUs), timeoutUs_(timeoutUs),
    window_(window < 1 ? 1 : (window > MAX_WINDOW ? MAX_WINDOW : window)),
    t_{}, head_(0), n_(0), count_(0), rejected_(0) {}

bool ImpellerSpeed::onPulse(uint32_t tUs) {
  if (n_ > 0) {
    uint32_t gap = tUs - t_[head_];
    if (gap < minIntervalUs_) {
      rejected_++;
      return false;
    }
    // After a stop, the old timestamps would drag the first rates down.
    if (gap > timeoutUs_) n_ = 0;
  }
  head_ = (uint8_t)((head_ + 1) % (MAX_WINDOW + 1));
  t_[head_] = tUs;
  if (n_ < window_ + 1) n_++;
  count_++;
  return true;
}

float ImpellerSpeed::rateHz(uint32_t nowUs) const {
  if (n_ < 2) return 0;
  uint32_t last = t_[head_];
  uint32_t since = nowUs - last;
  if (since > timeoutUs_) return 0;

  uint8_t oldest = (uint8_t)((head_ + (MAX_WINDOW + 1) - (n_ - 1)) %
                             (MAX_WINDOW + 1));
  uint32_t span = last - t_[oldest];
  if (span == 0) return 0;
  float intervals = (float)(n_ - 1);
  float rate = intervals * 1e6f / (float)span;

  // If the next pulse is already overdue, the impeller has slowed down: the
  // rate can be at most one pulse per elapsed time. This makes the reading
  // decay smoothly to zero instead of freezing at the last value.
  if ((float)since * intervals > (float)span) {
    float bound = 1e6f / (float)since;
    if (bound < rate) rate = bound;
  }
  return rate;
}

uint32_t ImpellerSpeed::usSinceLastPulse(uint32_t nowUs) const {
  return n_ ? nowUs - t_[head_] : 0xFFFFFFFFu;
}
