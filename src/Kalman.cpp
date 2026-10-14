#include "Kalman.h"

// ---------------------------------------------------------------- Kalman1D

Kalman1D::Kalman1D(float x0, float p0, float qPerSecond)
  : x_(x0), p_(p0), q_(qPerSecond) {}

void Kalman1D::reset(float x0, float p0) {
  x_ = x0;
  p_ = p0;
}

void Kalman1D::predict(float dt) {
  if (dt > 0) p_ += q_ * dt;
}

bool Kalman1D::update(float z, float r, float h, float gateSigma) {
  float s = h * h * p_ + r;  // innovation variance
  if (!(s > 0)) return false;
  float y = z - h * x_;      // innovation
  if (gateSigma > 0 && y * y > gateSigma * gateSigma * s) return false;
  float k = p_ * h / s;
  x_ += k * y;
  p_ *= (1.0f - k * h);
  return true;
}

void Kalman1D::clamp(float lo, float hi) {
  if (x_ < lo) x_ = lo;
  if (x_ > hi) x_ = hi;
}

// -------------------------------------------------------------- RateKalman

RateKalman::RateKalman(float jerkPsd) : q_(jerkPsd) { reset(); }

void RateKalman::reset() {
  x0_ = x1_ = 0;
  p00_ = 1.0f;     // (rad/s)²
  p01_ = 0;
  p11_ = 100.0f;   // (rad/s²)²
}

void RateKalman::predict(float dt) {
  if (dt <= 0) return;
  // x = F x, F = [1 dt; 0 1]
  x0_ += x1_ * dt;
  // P = F P Fᵀ + Q, Q = q [dt³/3 dt²/2; dt²/2 dt]
  float dt2 = dt * dt;
  float n00 = p00_ + 2 * dt * p01_ + dt2 * p11_ + q_ * dt2 * dt / 3.0f;
  float n01 = p01_ + dt * p11_ + q_ * dt2 / 2.0f;
  float n11 = p11_ + q_ * dt;
  p00_ = n00; p01_ = n01; p11_ = n11;
}

void RateKalman::updateRate(float z, float r) {
  // H = [1 0]
  float s = p00_ + r;
  float k0 = p00_ / s, k1 = p01_ / s;
  float y = z - x0_;
  x0_ += k0 * y;
  x1_ += k1 * y;
  float n00 = p00_ - k0 * p00_;
  float n01 = p01_ - k0 * p01_;
  float n11 = p11_ - k1 * p01_;
  p00_ = n00; p01_ = n01; p11_ = n11;
}

void RateKalman::updateAccel(float z, float r) {
  // H = [0 1]
  float s = p11_ + r;
  float k0 = p01_ / s, k1 = p11_ / s;
  float y = z - x1_;
  x0_ += k0 * y;
  x1_ += k1 * y;
  float n00 = p00_ - k0 * p01_;
  float n01 = p01_ - k0 * p11_;
  float n11 = p11_ - k1 * p11_;
  p00_ = n00; p01_ = n01; p11_ = n11;
}
