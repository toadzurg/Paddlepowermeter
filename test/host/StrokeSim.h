#ifndef STROKE_SIM_H
#define STROKE_SIM_H

// Planar paddle-stroke simulation from first principles: a rigid shaft
// rotating about a pivot fixed in the boat, with gravity, boat surge, sensor
// noise, gyro bias and a delayed, noisy impeller reading. Produces the
// PaddleSamples the firmware would see, plus ground truth.

#include <cmath>
#include <deque>
#include <random>

#include "Config.h"
#include "PowerModel.h"

struct SimConfig {
  double pivotPos = -0.15;   // true pivot position along shaft (m from IMU1)
  double amplitude = 0.6;    // shaft swing half-angle, rad
  double driveS = 0.6, recoveryS = 0.6;
  double peakForceN = 200;
  double surgeAmp = 0.0;     // boat surge acceleration amplitude, m/s²
  bool imu2Ok = true;
  double gyroNoise = 0.005, accelNoise = 0.05, forceNoise = 2.0;
  double gyroBias = 0.003;
  double seconds = 30;
  double stopAfter = 1e9;    // stop paddling after this time
  // Impeller: mean boat speed through the water (0 = no impeller). The
  // surge above modulates it consistently. The reading arrives late (radio)
  // and noisy, and drops out entirely from boatLostAt onward.
  double boatSpeed = 0;
  double boatLatencyS = 0.05, boatNoise = 0.02;
  double boatLostAt = 1e9;
};

struct SimTruth {
  double energy = 0;        // ∫ F·v_blade dt while paddling, J
  double usefulEnergy = 0;  // ∫ F·cosθ·v_boat dt while paddling, J
  double paddleTime = 0;    // s
};

// Calls onSample(const PaddleSample &s, double t) for every sample. If
// shaftAngle is given, it holds the true shaft angle during each call.
template <class OnSample>
SimTruth generateStroke(const SimConfig &c, unsigned seed, OnSample onSample,
                        double *shaftAngle = nullptr) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> N01(0, 1);

  const double dt = 1.0 / cfg::SAMPLE_HZ;
  const double cycle = c.driveS + c.recoveryS;
  const double L = cfg::BLADE_COP_POS_M - c.pivotPos;
  const double g = cfg::GRAVITY_MS2;
  const double imuPos[2] = {0.0, cfg::IMU2_POS_M};

  SimTruth truth;
  double &trueEnergy = truth.energy, &trueUsefulEnergy = truth.usefulEnergy,
         &truePaddleTime = truth.paddleTime;
  std::deque<double> boatPipe;  // models the radio latency

  for (int step = 0; step * dt < c.seconds; step++) {
    double t = step * dt;
    bool paddling = t >= 1.0 && t < c.stopAfter;  // 1 s at rest first

    // Shaft angle θ (0 = vertical); blade moves aft as θ increases.
    double th = -c.amplitude, thd = 0, thdd = 0, force = 0;
    if (paddling) {
      double tau = std::fmod(t - 1.0, cycle);
      if (tau < c.driveS) {
        double w = M_PI / c.driveS;
        th = -c.amplitude * std::cos(w * tau);
        thd = c.amplitude * w * std::sin(w * tau);
        thdd = c.amplitude * w * w * std::cos(w * tau);
        force = c.peakForceN * std::sin(w * tau);
      } else {
        double w = M_PI / c.recoveryS, r = tau - c.driveS;
        th = c.amplitude * std::cos(w * r);
        thd = -c.amplitude * w * std::sin(w * r);
        thdd = -c.amplitude * w * w * std::cos(w * r);
      }
      trueEnergy += force * thd * L * dt;
      truePaddleTime += dt;
    }

    // Boat speed consistent with the surge acceleration used below.
    double cycleW = 2 * M_PI / cycle;
    double vBoat = c.boatSpeed - c.surgeAmp / cycleW * std::cos(cycleW * t);
    // Only the horizontal part of the blade force drives the boat.
    if (paddling) trueUsefulEnergy += force * std::cos(th) * vBoat * dt;

    // World frame: X forward, Y left, Z up. Paddle axes in world:
    double ex[3] = {-std::sin(th), 0, -std::cos(th)};  // toward blade
    double ey[3] = {-std::cos(th), 0, std::sin(th)};   // power face motion
    double ez[3] = {0, 1, 0};                          // rotation axis
    double surge = c.surgeAmp * std::sin(2 * M_PI * t / cycle);

    PaddleSample s;
    s.timeMs = (uint32_t)std::lround(t * 1000);
    s.dt = step == 0 ? 0 : (float)dt;
    s.forceN = (float)(force + c.forceNoise * N01(rng));
    for (int i = 0; i < 2; i++) {
      double rr = imuPos[i] - c.pivotPos;
      double a[3];
      for (int k = 0; k < 3; k++)
        a[k] = rr * (thdd * ey[k] - thd * thd * ex[k]);
      a[0] += surge;
      a[2] += g;  // specific force = a − g_world
      ImuReading &m = s.imu[i];
      m.valid = (i == 0) || c.imu2Ok;
      const double *axes[3] = {ex, ey, ez};
      for (int k = 0; k < 3; k++) {
        double f = a[0] * axes[k][0] + a[1] * axes[k][1] + a[2] * axes[k][2];
        m.accel[k] = (float)(f + c.accelNoise * N01(rng));
        m.gyro[k] = (float)((k == 2 ? thd : 0) + c.gyroBias +
                            c.gyroNoise * N01(rng));
      }
    }

    boatPipe.push_back(vBoat + c.boatNoise * N01(rng));
    size_t lag = (size_t)std::lround(c.boatLatencyS / dt);
    s.boatSpeedValid = c.boatSpeed > 0 && t < c.boatLostAt &&
                       boatPipe.size() > lag;
    s.boatSpeedMps = s.boatSpeedValid ? (float)boatPipe.front() : 0;
    if (boatPipe.size() > lag) boatPipe.pop_front();

    if (shaftAngle) *shaftAngle = th;
    onSample(s, t);
  }
  return truth;
}

#endif  // STROKE_SIM_H
