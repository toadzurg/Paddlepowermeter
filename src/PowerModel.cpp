#include "PowerModel.h"
#include "Config.h"
#include <math.h>
#include <initializer_list>

namespace {
constexpr float INST_P_SMOOTH = 0.2f;   // EMA weight for displayed inst. power
constexpr float MAX_DT = 0.1f;          // larger gaps reset the filters
constexpr float MIN_LEVER_M = 0.1f;
constexpr float DEG2RAD = 0.017453292f;

// Unit normal of the power face in the paddle frame (X shaft, Y power face).
// A blade bent by β toward the bow runs along (cos β, −sin β); its power
// face normal is (sin β, cos β).
struct BladeNormal { float x, y; };
BladeNormal bladeNormal() {
  const float b = cfg::BLADE_BEND_DEG * DEG2RAD;
  return {sinf(b), cosf(b)};
}
}

float PowerModel::leverArmFor(float pivot) {
  const float b = cfg::BLADE_BEND_DEG * DEG2RAD;
  return cfg::BLADE_COP_FROM_BEND_M + (cfg::SHAFT_BEND_POS_M - pivot) * cosf(b);
}

PowerModel::PowerModel()
  : rateKf_(cfg::RATE_JERK_PSD),
    pivotKf_(cfg::PIVOT_PRIOR_POS_M,
             cfg::PIVOT_PRIOR_SIGMA_M * cfg::PIVOT_PRIOR_SIGMA_M,
             cfg::PIVOT_DRIFT_M2_S),
    ahrs_(cfg::AHRS_BETA),
    leverArm_(leverArmFor(cfg::PIVOT_PRIOR_POS_M)),
    smInstP_(0), prevForceN_(0),
    inDrive_(false), driveStartMs_(0), driveEnergy_(0),
    havePrev_(false), prevDriveStartMs_(0),
    strokePower_(0), strokeRate_(0), lastEnergy_(0),
    strokeCount_(0), lastStrokeMs_(0),
    driveUseful_(0), lastUseful_(0),
    driveImpulse_(0), lastImpulse_(0), lastDriveS_(0), dutyFactor_(0), meanForward_(0),
    driveUsefulValid_(false), lastUsefulValid_(false), splitValid_(false),
    usefulPower_(0), slipPower_(0), efficiency_(0) {}

void PowerModel::begin(const PaddleSample &atRest) {
  const ImuReading &r = atRest.imu[0].valid ? atRest.imu[0] : atRest.imu[1];
  if (r.valid) ahrs_.initFromAccel(r.accel[0], r.accel[1], r.accel[2]);
  rateKf_.reset();
}

void PowerModel::update(const PaddleSample &s) {
  const ImuReading &i1 = s.imu[0];
  const ImuReading &i2 = s.imu[1];
  const bool both = i1.valid && i2.valid;

  float dt = s.dt;
  if (!(dt > 0) || dt > MAX_DT) {
    // Stalled loop or first sample: the filters' motion model is no longer
    // valid, so restart the rate filter instead of integrating a huge step.
    rateKf_.reset();
    dt = 0;
  }

  // Raw angular velocity vector (mean of the gyros on a rigid shaft)
  float omega[3] = {0, 0, 0};
  int n = 0;
  for (const ImuReading *r : {&i1, &i2}) {
    if (!r->valid) continue;
    for (int k = 0; k < 3; k++) omega[k] += r->gyro[k];
    n++;
  }
  if (n == 0) return;  // no motion data at all
  for (int k = 0; k < 3; k++) omega[k] /= n;

  // --- ω_z / α_z Kalman filter ---------------------------------------------
  const float rGyro = cfg::GYRO_NOISE_RAD_S * cfg::GYRO_NOISE_RAD_S;
  rateKf_.predict(dt);
  if (i1.valid) rateKf_.updateRate(i1.gyro[2], rGyro);
  if (i2.valid) rateKf_.updateRate(i2.gyro[2], rGyro);
  if (both) {
    // For a rigid body, a2 − a1 = α × r + ω × (ω × r), r = d·x̂. The Y
    // component gives d·(α_z + ω_x·ω_y). Gravity is identical at both IMUs
    // and cancels, so this needs no orientation estimate.
    const float d = cfg::IMU2_POS_M;
    float alphaMeas = (i2.accel[1] - i1.accel[1]) / d - omega[0] * omega[1];
    float sigA = cfg::ACCEL_NOISE_MS2 * 1.41421356f / fabsf(d);
    rateKf_.updateAccel(alphaMeas, sigA * sigA);
  }

  // --- Orientation (needed to remove gravity for the pivot fit) ------------
  if (dt > 0) {
    // Feed the AHRS the specific force extrapolated to the pivot: the
    // kinematic terms are linear along the shaft and ~0 at the pivot, so
    // this is close to pure gravity even mid-stroke.
    float f[3];
    float t = both ? pivotKf_.value() / cfg::IMU2_POS_M : 0.0f;
    for (int k = 0; k < 3; k++) {
      f[k] = both ? i1.accel[k] + (i2.accel[k] - i1.accel[k]) * t
                  : (i1.valid ? i1.accel[k] : i2.accel[k]);
    }
    ahrs_.update(omega[0], omega[1], omega[2], f[0], f[1], f[2], dt);
  }

  // --- Pivot / lever arm ---------------------------------------------------
  pivotKf_.predict(dt);
  // Only fit the pivot while the blade is loaded: that is the only time the
  // lever arm affects power, and during recovery the paddle swings about a
  // different point entirely.
  if (s.forceN > cfg::STROKE_START_N) updatePivot(s, omega, rateKf_.accel());
  pivotKf_.clamp(cfg::PIVOT_MIN_POS_M, cfg::PIVOT_MAX_POS_M);

  leverArm_ = leverArmFor(pivotKf_.value());
  if (leverArm_ < MIN_LEVER_M) leverArm_ = MIN_LEVER_M;

  // --- Power ---------------------------------------------------------------
  float vBlade = rateKf_.rate() * leverArm_;  // m/s relative to the boat
  float instP = s.forceN * vBlade;            // W
  smInstP_ += INST_P_SMOOTH * (instP - smInstP_);

  // Useful (propulsive) power is the forward component of the blade force
  // times boat speed. The force acts along the power face normal n, so its
  // horizontal fraction is √(1 − (n·up)²): cos(θ − β) for a shaft at θ with
  // a blade bent by β. This assumes the blade moves straight aft; sweep or
  // steering strokes slightly overstate it.
  float ux, uy, uz;
  ahrs_.upVector(ux, uy, uz);
  const BladeNormal face = bladeNormal();
  float vertical = face.x * ux + face.y * uy;
  float horizontal = sqrtf(fmaxf(0.0f, 1.0f - vertical * vertical));
  float forwardN = s.forceN * horizontal;  // also gives the stroke's impulse
  float usefulP = s.boatSpeedValid ? forwardN * s.boatSpeedMps : 0;

  updateStrokes(s, instP, usefulP, forwardN, dt);
}

float PowerModel::shaftAngle() const {
  // Paddle-frame "up" is (−cosθ, sinθ, ·) for a shaft at θ from vertical
  // in the stroke plane (X toward blade, Y power face).
  float ux, uy, uz;
  ahrs_.upVector(ux, uy, uz);
  return atan2f(uy, -ux);
}

void PowerModel::updatePivot(const PaddleSample &s, const float omega[3],
                             float alphaZ) {
  float ux, uy, uz;
  ahrs_.upVector(ux, uy, uz);
  const float g[3] = {ux * cfg::GRAVITY_MS2, uy * cfg::GRAVITY_MS2,
                      uz * cfg::GRAVITY_MS2};

  // Point at position x, pivot at p, r = (x − p)·x̂. Its linear acceleration:
  //   a_y = (α_z + ω_x ω_y) (x − p)        (tangential)
  //   a_x = −(ω_y² + ω_z²) (x − p)          (centripetal)
  // i.e. (a − h·x) = −h·p, a linear measurement of p with gain −h.
  const float hT = alphaZ + omega[0] * omega[1];
  const float hR = -(omega[1] * omega[1] + omega[2] * omega[2]);
  const float r = cfg::LIN_ACCEL_NOISE_MS2 * cfg::LIN_ACCEL_NOISE_MS2;
  const float pos[2] = {0.0f, cfg::IMU2_POS_M};

  for (int i = 0; i < 2; i++) {
    const ImuReading &m = s.imu[i];
    if (!m.valid) continue;
    float aX = m.accel[0] - g[0];
    float aY = m.accel[1] - g[1];
    pivotKf_.update(aY - hT * pos[i], r, -hT, cfg::PIVOT_GATE_SIGMA);
    pivotKf_.update(aX - hR * pos[i], r, -hR, cfg::PIVOT_GATE_SIGMA);
  }
}

void PowerModel::updateStrokes(const PaddleSample &s, float instP,
                               float usefulP, float forwardN, float dt) {
  const uint32_t now = s.timeMs;

  if (!inDrive_ && s.forceN > cfg::STROKE_START_N) {
    inDrive_ = true;
    // Interpolate the threshold crossing between samples; otherwise stroke
    // timing is quantised to the sample period (±1 spm at race pace).
    uint32_t back = 0;
    if (dt > 0 && prevForceN_ < cfg::STROKE_START_N) {
      float frac = (s.forceN - cfg::STROKE_START_N) / (s.forceN - prevForceN_);
      back = (uint32_t)lroundf(frac * dt * 1000.0f);
    }
    driveStartMs_ = now - back;
    driveEnergy_ = 0;
    driveUseful_ = 0;
    driveImpulse_ = 0;
    driveUsefulValid_ = true;
  }
  prevForceN_ = s.forceN;

  if (inDrive_) {
    driveEnergy_ += instP * dt;
    driveImpulse_ += forwardN * dt;
    // One missing boat-speed sample spoils the stroke's split, not the
    // stroke's total power.
    if (s.boatSpeedValid) driveUseful_ += usefulP * dt;
    else driveUsefulValid_ = false;
  }

  if (inDrive_ && s.forceN < cfg::STROKE_END_N) {
    inDrive_ = false;
    if (now - driveStartMs_ >= cfg::MIN_DRIVE_MS) {
      uint32_t cycle = havePrev_ ? driveStartMs_ - prevDriveStartMs_ : 0;
      if (havePrev_ && cycle < cfg::MIN_CYCLE_MS) {
        // A second force peak inside the same stroke: fold it in.
        driveEnergy_ += lastEnergy_;
        driveUseful_ += lastUseful_;
        driveImpulse_ += lastImpulse_;
        driveUsefulValid_ = driveUsefulValid_ && lastUsefulValid_;
        driveStartMs_ = prevDriveStartMs_;
        strokeCount_--;
      } else if (havePrev_ && cycle <= cfg::MAX_CYCLE_MS) {
        // Average power over the full cycle (drive + recovery), which is
        // what a paddler's sustained output actually is.
        strokePower_ = fmaxf(0.0f, driveEnergy_ / (cycle * 0.001f));
        strokeRate_ = 60000.0f / cycle;
        meanForward_ = driveImpulse_ / (cycle * 0.001f);
        dutyFactor_ = fminf(1.0f, (now - driveStartMs_) / (float)cycle);
        splitValid_ = driveUsefulValid_ && driveEnergy_ > 0;
        if (splitValid_) {
          usefulPower_ = driveUseful_ / (cycle * 0.001f);
          slipPower_ = strokePower_ - usefulPower_;
          efficiency_ = driveUseful_ / driveEnergy_;
        }
      }
      lastEnergy_ = driveEnergy_;
      lastUseful_ = driveUseful_;
      lastImpulse_ = driveImpulse_;
      lastDriveS_ = (now - driveStartMs_) * 0.001f;
      lastUsefulValid_ = driveUsefulValid_;
      prevDriveStartMs_ = driveStartMs_;
      havePrev_ = true;
      strokeCount_++;
      lastStrokeMs_ = driveStartMs_;
    }
  }

  // Paddler stopped: don't keep reporting the last stroke forever.
  if (havePrev_ && !inDrive_ && now - prevDriveStartMs_ > cfg::MAX_CYCLE_MS) {
    strokePower_ = 0;
    strokeRate_ = 0;
    splitValid_ = false;
    usefulPower_ = slipPower_ = efficiency_ = 0;
    dutyFactor_ = meanForward_ = 0;
  }
}
