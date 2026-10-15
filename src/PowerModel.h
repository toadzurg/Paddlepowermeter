#ifndef POWERMODEL_H
#define POWERMODEL_H

#include <stdint.h>
#include "Kalman.h"
#include "Orientation.h"

// One IMU sample, already rotated into the paddle frame (see Config.h).
struct ImuReading {
  float gyro[3];   // rad/s
  float accel[3];  // m/s², specific force (reads +g upward at rest)
  bool valid;
};

// Everything the model needs for one time step. Pure data, so the model can
// be driven by real sensors or by a simulation on a PC.
struct PaddleSample {
  uint32_t timeMs;
  float dt;          // seconds since previous sample
  float forceN;      // force at blade centre of pressure
  ImuReading imu[2]; // imu[0] = IMU1 (position 0), imu[1] = IMU2
  float boatSpeedMps;   // boat speed through the water (impeller), optional
  bool boatSpeedValid;
};

// Estimates paddler power from blade force and paddle kinematics.
//
//   P = F · v_blade,  v_blade = ω · L,  L = x_blade − x_pivot
//
// ω and α come from a [rate, accel] Kalman filter fed by both gyros and by
// the difference of the two accelerometers (a rigid-body identity in which
// gravity cancels). The pivot is the point on the shaft that is not
// accelerating relative to the boat; its position is estimated with a
// scalar Kalman filter whose measurement noise scales with how hard the
// paddle is rotating, so the estimate only moves when the data is
// informative.
class PowerModel {
public:
  PowerModel();

  // Call once with the paddle at rest before the first update().
  void begin(const PaddleSample &atRest);
  void update(const PaddleSample &s);

  // Instantaneous values (low-pass filtered, for display/debug)
  float instantaneousPower() const { return smInstP_; }
  float angularVelocity() const { return rateKf_.rate(); }
  float angularAccel() const { return rateKf_.accel(); }
  float pivotPosition() const { return pivotKf_.value(); }
  float leverArm() const { return leverArm_; }

  // Per-stroke values
  float strokePower() const { return strokePower_; }  // W, energy / cycle time
  float strokeRate() const { return strokeRate_; }    // strokes per minute
  float strokeEnergy() const { return lastEnergy_; }  // J of last drive

  // Split of strokePower() into the part that pushes the boat and the part
  // lost to blade slip. Needs boat speed for the whole drive; check
  // hasPowerSplit() first.
  bool hasPowerSplit() const { return splitValid_; }
  float usefulPower() const { return usefulPower_; }  // W, F·cosθ·v_boat
  float slipPower() const { return slipPower_; }      // W, total − useful
  // useful / total. Not clamped: a value above 1 on the water means the
  // force, lever arm or impeller calibration is off.
  float efficiency() const { return efficiency_; }
  uint16_t strokeCount() const { return strokeCount_; }
  uint32_t lastStrokeTimeMs() const { return lastStrokeMs_; }
  bool inDrive() const { return inDrive_; }

private:
  void updatePivot(const PaddleSample &s, const float omega[3], float alphaZ);
  void updateStrokes(const PaddleSample &s, float instP, float usefulP,
                     float dt);

  RateKalman rateKf_;
  Kalman1D pivotKf_;
  Orientation ahrs_;

  float leverArm_;
  float smInstP_;
  float prevForceN_;

  bool inDrive_;
  uint32_t driveStartMs_;
  float driveEnergy_;

  bool havePrev_;
  uint32_t prevDriveStartMs_;

  float strokePower_, strokeRate_, lastEnergy_;
  uint16_t strokeCount_;
  uint32_t lastStrokeMs_;

  float driveUseful_, lastUseful_;
  bool driveUsefulValid_, lastUsefulValid_;
  bool splitValid_;
  float usefulPower_, slipPower_, efficiency_;
};

#endif  // POWERMODEL_H
