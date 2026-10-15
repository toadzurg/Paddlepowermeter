// Host-side tests for ImpellerSpeed (boat-node pulse timing).

#include <cmath>
#include <cstdio>
#include <random>

#include "ImpellerSpeed.h"

static int failures = 0;

#define CHECK_NEAR(name, got, want, tol)                                      \
  do {                                                                        \
    double g_ = (got), w_ = (want), t_ = (tol);                               \
    bool ok_ = std::fabs(g_ - w_) <= t_;                                      \
    std::printf("  [%s] %-42s got %10.4f  want %10.4f ± %g\n",               \
                ok_ ? " ok " : "FAIL", name, g_, w_, t_);                     \
    if (!ok_) failures++;                                                     \
  } while (0)

static ImpellerSpeed make() { return ImpellerSpeed(500, 1500000, 2); }

int main() {
  std::printf("\nImpellerSpeed\n");

  {
    ImpellerSpeed imp = make();
    CHECK_NEAR("no pulses: rate 0", imp.rateHz(0), 0, 0);
    imp.onPulse(1000);
    CHECK_NEAR("one pulse: rate 0", imp.rateHz(1000), 0, 0);
  }

  {
    // 40 Hz with ±2% timing jitter
    std::mt19937 rng(1);
    std::normal_distribution<double> jitter(0, 500);
    ImpellerSpeed imp = make();
    double t = 0;
    for (int i = 0; i < 200; i++) {
      t += 25000 + jitter(rng);
      imp.onPulse((uint32_t)t);
    }
    CHECK_NEAR("steady 40 Hz", imp.rateHz((uint32_t)t + 1000), 40, 1.5);
    CHECK_NEAR("count", imp.count(), 200, 0);
  }

  {
    ImpellerSpeed imp = make();
    uint32_t t = 0;
    for (int i = 0; i < 20; i++) {
      t += 25000;
      imp.onPulse(t);
      if (i % 5 == 0) imp.onPulse(t + 100);  // 100 µs glitch after a pulse
    }
    CHECK_NEAR("glitches rejected", imp.rejected(), 4, 0);
    CHECK_NEAR("glitches don't change rate", imp.rateHz(t + 1000), 40, 0.01);
  }

  {
    ImpellerSpeed imp = make();
    uint32_t t = 0;
    for (int i = 0; i < 20; i++) imp.onPulse(t += 25000);
    // No more pulses: the rate decays as 1/elapsed, then hits 0 at timeout.
    CHECK_NEAR("overdue pulse: rate decays", imp.rateHz(t + 100000), 10, 0.01);
    CHECK_NEAR("after timeout: rate 0", imp.rateHz(t + 1600000), 0, 0);
  }

  {
    ImpellerSpeed imp = make();
    uint32_t t = 0;
    for (int i = 0; i < 20; i++) imp.onPulse(t += 25000);
    t += 5000000;  // stopped for 5 s, then restart at 20 Hz
    imp.onPulse(t);
    CHECK_NEAR("first pulse after stop: 0", imp.rateHz(t + 1000), 0, 0);
    imp.onPulse(t += 50000);
    CHECK_NEAR("restart not polluted by old data", imp.rateHz(t + 1000), 20,
               0.01);
  }

  {
    // Timestamps crossing the uint32 µs wrap (every ~71.6 minutes)
    ImpellerSpeed imp = make();
    uint32_t t = 0xFFFFFFFFu - 60000;
    for (int i = 0; i < 6; i++) imp.onPulse(t += 25000);
    CHECK_NEAR("rate across µs wrap", imp.rateHz(t + 1000), 40, 0.01);
  }

  {
    // Speed-up within a stroke must show up within a few pulses.
    ImpellerSpeed imp = make();
    uint32_t t = 0;
    for (int i = 0; i < 10; i++) imp.onPulse(t += 25000);  // 40 Hz
    for (int i = 0; i < 3; i++) imp.onPulse(t += 20000);   // 50 Hz
    CHECK_NEAR("tracks surge in 3 pulses", imp.rateHz(t + 1000), 50, 0.01);
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
