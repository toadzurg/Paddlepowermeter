// Host-side tests for the signal-processing code. No hardware needed:
//
//   ./test/host/run.sh
//
// A planar paddle stroke is simulated from first principles (rigid shaft
// rotating about a pivot fixed in the boat, gravity, boat surge, sensor
// noise and gyro bias) and the model's outputs are checked against the
// simulation's ground truth.

#include <cmath>
#include <cstdio>
#include <random>

#include "Config.h"
#include "Kalman.h"
#include "Orientation.h"
#include "PowerModel.h"
#include "StrokeSim.h"

static int failures = 0;

#define CHECK_NEAR(name, got, want, tol)                                      \
  do {                                                                        \
    double g_ = (got), w_ = (want), t_ = (tol);                               \
    bool ok_ = std::fabs(g_ - w_) <= t_;                                      \
    std::printf("  [%s] %-38s got %9.4f  want %9.4f ± %g\n",                 \
                ok_ ? " ok " : "FAIL", name, g_, w_, t_);                     \
    if (!ok_) failures++;                                                     \
  } while (0)

// ------------------------------------------------------------ simulation

struct SimResult {
  double truePower = 0;      // mean over full cycles while paddling
  double meanModelPower = 0; // mean of strokePower() after warm-up
  double strokeRate = 0;
  double pivot = 0;
  double finalPower = 0, finalRate = 0;
  int strokes = 0;
  double trueUseful = 0, trueEfficiency = 0;
  double meanUseful = 0, meanEfficiency = 0;
  double maxSplitMismatch = 0;  // |useful + slip − stroke power|
  bool splitValidAtEnd = false;
};

static SimResult simulate(const SimConfig &c, unsigned seed = 1) {
  PowerModel model;
  double sumP = 0, sumU = 0, sumE = 0, maxMismatch = 0;
  int nP = 0, nU = 0;
  bool first = true;

  SimTruth truth = generateStroke(c, seed, [&](const PaddleSample &s, double t) {
    if (first) model.begin(s);
    first = false;
    model.update(s);

    if (t > 10 && t < c.stopAfter) {  // after the filters have converged
      sumP += model.strokePower();
      nP++;
      if (model.hasPowerSplit()) {
        sumU += model.usefulPower();
        sumE += model.efficiency();
        nU++;
        maxMismatch = std::fmax(maxMismatch,
            std::fabs(model.usefulPower() + model.slipPower() -
                      model.strokePower()));
      }
    }
  });

  SimResult r;
  r.truePower = truth.paddleTime > 0 ? truth.energy / truth.paddleTime : 0;
  r.meanModelPower = nP ? sumP / nP : 0;
  r.strokeRate = model.strokeRate();
  r.pivot = model.pivotPosition();
  r.finalPower = model.strokePower();
  r.finalRate = model.strokeRate();
  r.strokes = model.strokeCount();
  r.trueUseful = truth.paddleTime > 0 ? truth.usefulEnergy / truth.paddleTime : 0;
  r.trueEfficiency = truth.energy > 0 ? truth.usefulEnergy / truth.energy : 0;
  r.meanUseful = nU ? sumU / nU : 0;
  r.meanEfficiency = nU ? sumE / nU : 0;
  r.maxSplitMismatch = maxMismatch;
  r.splitValidAtEnd = model.hasPowerSplit();
  return r;
}

static void reportSim(const char *title, const SimConfig &c, double powerTol) {
  std::printf("\n%s\n", title);
  SimResult r = simulate(c);
  CHECK_NEAR("stroke power (W)", r.meanModelPower, r.truePower,
             powerTol * r.truePower);
  CHECK_NEAR("stroke rate (spm)", r.strokeRate,
             60.0 / (c.driveS + c.recoveryS), 0.5);
  CHECK_NEAR("pivot position (m)", r.pivot, c.pivotPos, 0.05);
  int expected = (int)((c.seconds - 1.0) / (c.driveS + c.recoveryS));
  CHECK_NEAR("stroke count", r.strokes, expected, 1);
}

// ------------------------------------------------------------ unit tests

static void testKalman1D() {
  std::printf("\nKalman1D\n");
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0, 0.5);
  Kalman1D kf(0, 100, 0);
  for (int i = 0; i < 2000; i++) kf.update((float)(3.0 + n(rng)), 0.25f);
  CHECK_NEAR("converges to constant", kf.value(), 3.0, 0.05);
  CHECK_NEAR("variance ≈ r / n", kf.variance(), 0.25 / 2000, 1e-5);

  Kalman1D gated(0, 1e-4f, 0);
  bool used = gated.update(100, 1e-4f, 1, 3);
  CHECK_NEAR("outlier rejected by gate", used ? 1 : 0, 0, 0);
  CHECK_NEAR("state unchanged by outlier", gated.value(), 0, 0);

  Kalman1D zeroGain(5, 1, 0);
  zeroGain.update(-50, 1, 0.0f);  // h = 0: measurement has no information
  CHECK_NEAR("h = 0 leaves state alone", zeroGain.value(), 5, 0);
}

static void testRateKalman() {
  std::printf("\nRateKalman\n");
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0, 0.01);
  RateKalman kf(cfg::RATE_JERK_PSD);
  const double dt = 0.01, alpha = 4.0;
  double sumA = 0, sqErrRateOnly = 0;
  for (int i = 1; i <= 300; i++) {
    kf.predict((float)dt);
    kf.updateRate((float)(alpha * i * dt + n(rng)), 1e-4f);
    if (i > 200) {
      sumA += kf.accel();
      sqErrRateOnly += (kf.accel() - alpha) * (kf.accel() - alpha);
    }
  }
  CHECK_NEAR("tracks ramp rate", kf.rate(), alpha * 3.0, 0.05);
  // α from rate alone is noisy (that is why PowerModel also feeds it the
  // accelerometer difference), but it must be unbiased.
  CHECK_NEAR("mean α from rate only", sumA / 100, alpha, 0.5);

  RateKalman both(cfg::RATE_JERK_PSD);
  double sqErr = 0;
  for (int i = 1; i <= 300; i++) {
    both.predict((float)dt);
    both.updateRate((float)(alpha * i * dt + n(rng)), 1e-4f);
    both.updateAccel((float)(alpha + 2.0 * n(rng) / 0.01), 4.0f);
    if (i > 200) sqErr += (both.accel() - alpha) * (both.accel() - alpha);
  }
  double rmsRate = std::sqrt(sqErrRateOnly / 100), rmsBoth = std::sqrt(sqErr / 100);
  std::printf("         α rms error: gyro only %.2f, gyro + accel %.2f rad/s²\n",
              rmsRate, rmsBoth);
  CHECK_NEAR("accel measurement reduces α error", rmsBoth < rmsRate, 1, 0);
}

static void testOrientation() {
  std::printf("\nOrientation\n");
  const float a[3] = {0.3f, -0.5f, 0.81f};
  float an = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  Orientation o(0.1f);
  o.initFromAccel(a[0], a[1], a[2]);
  float ux, uy, uz;
  o.upVector(ux, uy, uz);
  CHECK_NEAR("init up.x matches accel", ux, a[0] / an, 1e-4);
  CHECK_NEAR("init up.y matches accel", uy, a[1] / an, 1e-4);
  CHECK_NEAR("init up.z matches accel", uz, a[2] / an, 1e-4);

  // Rotate 90° about Z at 1 rad/s; "up" along Z must not move.
  Orientation flat(0.0f);
  flat.initFromAccel(0, 0, 1);
  for (int i = 0; i < 157; i++) flat.update(0, 0, 1, 0, 0, 1, 0.01f);
  flat.upVector(ux, uy, uz);
  CHECK_NEAR("yaw rotation keeps up = +Z", uz, 1.0, 1e-3);

  // Pure gyro roll of 90° about X: up moves from +Z to −Y... in the sensor
  // frame, rotating the body +90° about X puts world-up along body +Y.
  Orientation roll(0.0f);
  roll.initFromAccel(0, 0, 1);
  for (int i = 0; i < 1000; i++)
    roll.update((float)(M_PI / 2), 0, 0, 0, 0, 0, 0.001f);
  roll.upVector(ux, uy, uz);
  CHECK_NEAR("gyro roll 90°: up.z", uz, 0.0, 2e-3);
  CHECK_NEAR("gyro roll 90°: |up.y|", std::fabs(uy), 1.0, 2e-3);
}

int main() {
  testKalman1D();
  testRateKalman();
  testOrientation();

  SimConfig nominal;
  reportSim("Simulated stroke: nominal", nominal, 0.05);

  SimConfig wrongPrior;
  wrongPrior.pivotPos = 0.25;  // prior is −0.10
  reportSim("Simulated stroke: pivot far from prior (0.25 m)", wrongPrior, 0.05);

  SimConfig surge;
  surge.surgeAmp = 0.8;
  reportSim("Simulated stroke: boat surge 0.8 m/s²", surge, 0.07);

  SimConfig fast;
  fast.driveS = 0.45; fast.recoveryS = 0.35; fast.peakForceN = 300;
  reportSim("Simulated stroke: fast race pace (75 spm)", fast, 0.05);

  SimConfig oneImu;
  oneImu.imu2Ok = false;
  reportSim("Simulated stroke: IMU2 failed", oneImu, 0.07);

  std::printf("\nShaft angle (for the app's stroke view)\n");
  {
    PowerModel model;
    double trueAngle = 0, sq = 0;
    int n = 0;
    bool first = true;
    generateStroke(SimConfig(), 1, [&](const PaddleSample &s, double t) {
      if (first) model.begin(s);
      first = false;
      model.update(s);
      if (t > 5) {
        double e = model.shaftAngle() - trueAngle;
        sq += e * e;
        n++;
      }
    }, &trueAngle);
    CHECK_NEAR("rms error (deg)", std::sqrt(sq / n) * 180 / M_PI, 0, 3.0);
  }

  std::printf("\nPower split: no impeller\n");
  {
    SimResult r = simulate(SimConfig());
    CHECK_NEAR("no split without boat speed", r.splitValidAtEnd, 0, 0);
  }

  struct SplitCase { const char *title; double speed, surge; };
  for (SplitCase sc : {SplitCase{"Power split: 2.5 m/s, 50 ms radio delay", 2.5, 0},
                       SplitCase{"Power split: 2.0 m/s with boat surge", 2.0, 0.8}}) {
    std::printf("\n%s\n", sc.title);
    SimConfig c;
    c.boatSpeed = sc.speed;
    c.surgeAmp = sc.surge;
    SimResult r = simulate(c);
    CHECK_NEAR("split reported", r.splitValidAtEnd, 1, 0);
    CHECK_NEAR("useful power (W)", r.meanUseful, r.trueUseful,
               0.05 * r.trueUseful);
    CHECK_NEAR("efficiency", r.meanEfficiency, r.trueEfficiency, 0.03);
    CHECK_NEAR("useful + slip = stroke power (W)", r.maxSplitMismatch, 0, 1e-3);
  }

  std::printf("\nPower split: impeller signal lost mid-session\n");
  {
    SimConfig c;
    c.boatSpeed = 2.5;
    c.boatLostAt = 20;
    SimResult r = simulate(c);
    CHECK_NEAR("split withdrawn, not stale", r.splitValidAtEnd, 0, 0);
    CHECK_NEAR("total power unaffected (W)", r.meanModelPower, r.truePower,
               0.05 * r.truePower);
  }

  std::printf("\nSimulated stroke: paddler stops\n");
  SimConfig stop;
  stop.seconds = 25;
  stop.stopAfter = 15;
  SimResult r = simulate(stop);
  CHECK_NEAR("power drops to 0 after stopping", r.finalPower, 0, 0);
  CHECK_NEAR("stroke rate drops to 0", r.finalRate, 0, 0);

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
