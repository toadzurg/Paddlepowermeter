#include "Orientation.h"
#include <math.h>

Orientation::Orientation(float beta)
  : beta_(beta), q0_(1), q1_(0), q2_(0), q3_(0) {}

void Orientation::initFromAccel(float ax, float ay, float az) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n <= 0) return;
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll / 2), sr = sinf(roll / 2);
  float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
  q0_ = cr * cp;
  q1_ = sr * cp;
  q2_ = cr * sp;
  q3_ = -sr * sp;
}

void Orientation::update(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt) {
  float q0 = q0_, q1 = q1_, q2 = q2_, q3 = q3_;

  // Rate of change of quaternion from gyroscope
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an > 0) {
    ax /= an; ay /= an; az /= an;

    float _2q0 = 2 * q0, _2q1 = 2 * q1, _2q2 = 2 * q2, _2q3 = 2 * q3;
    float _4q0 = 4 * q0, _4q1 = 4 * q1, _4q2 = 4 * q2;
    float _8q1 = 8 * q1, _8q2 = 8 * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    // Gradient descent corrective step
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1
             + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
             + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;
    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 0) {
      qDot1 -= beta_ * s0 / sn;
      qDot2 -= beta_ * s1 / sn;
      qDot3 -= beta_ * s2 / sn;
      qDot4 -= beta_ * s3 / sn;
    }
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  float qn = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qn > 0) {
    q0_ = q0 / qn; q1_ = q1 / qn; q2_ = q2 / qn; q3_ = q3 / qn;
  }
}

void Orientation::upVector(float &ux, float &uy, float &uz) const {
  ux = 2 * (q1_ * q3_ - q0_ * q2_);
  uy = 2 * (q0_ * q1_ + q2_ * q3_);
  uz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;
}

void Orientation::quaternion(float &w, float &x, float &y, float &z) const {
  w = q0_; x = q1_; y = q2_; z = q3_;
}
