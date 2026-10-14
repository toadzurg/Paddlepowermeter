#ifndef KALMAN_H
#define KALMAN_H

// Scalar Kalman filter with a random-walk process model.
//
// Measurements follow z = h * x + v, v ~ N(0, r). Both h and r may change on
// every call, which lets a caller weight each measurement by how much
// information it actually carries about x.
class Kalman1D {
public:
  Kalman1D(float x0, float p0, float qPerSecond);

  void reset(float x0, float p0);

  // Grow the uncertainty by the process noise accumulated over dt seconds.
  void predict(float dt);

  // Fuse one measurement. If gateSigma > 0, measurements whose innovation is
  // larger than gateSigma standard deviations are rejected. Returns true if
  // the measurement was used.
  bool update(float z, float r, float h = 1.0f, float gateSigma = 0.0f);

  void clamp(float lo, float hi);

  float value() const { return x_; }
  float variance() const { return p_; }

private:
  float x_, p_, q_;
};

// Two-state [rate, acceleration] Kalman filter with a constant-acceleration
// (white jerk) process model. Used to get the paddle's angular velocity and
// angular acceleration without numerically differentiating a noisy gyro.
class RateKalman {
public:
  explicit RateKalman(float jerkPsd);

  void reset();
  void predict(float dt);
  void updateRate(float z, float r);   // direct rate measurement (gyro)
  void updateAccel(float z, float r);  // direct acceleration measurement

  float rate() const { return x0_; }
  float accel() const { return x1_; }

private:
  float q_;
  float x0_, x1_;
  float p00_, p01_, p11_;
};

#endif  // KALMAN_H
