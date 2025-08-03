#include "PowerModel.h"

PowerModel::PowerModel(float I,
                       float forceCal,
                       float velGain,
                       float kf_Q,
                       float kf_R,
                       float kf_P)
 : I_(I),
   forceCal_(forceCal),
   velGain_(velGain),
   leverFilter_(kf_Q, kf_R, kf_P, 0),
   lastAngVel_(0),
   lastTime_(0),
   smInstP_(0),
   smStrokeP_(0),
   inStroke_(false),
   strokeStart_(0),
   accEnergy_(0),
   lastStrokeP_(0),
   leverArm_(0)
{}

void PowerModel::update(uint32_t now) {
  // 1) Compute Δt
  float dt = (lastTime_==0 ? 0 : (now - lastTime_)*0.001f);
  lastTime_ = now;

  // 2) Read sensors
  float rawCount = getForce();              
  float forceN   = rawCount * forceCal_;    // N
  float angVel   = getAngularVelocity() * velGain_; // rad/s

  // 3) Angular acceleration α
  float angAccel = computeAngularAcceleration(angVel, dt);

  // 4) Torque τ = I * α
  float torque = I_ * angAccel;            // N·m

  // 5) Dynamic lever arm ℓ = τ / F
  float rawArm = (forceN > 1.0f) ? (torque / forceN) : leverArm_;
  leverArm_ = leverFilter_.update(rawArm);

  // 6) Instantaneous power: P = F * v = F * (ω * ℓ)
  float linVel = angVel * leverArm_;
  float instP  = forceN * linVel;          // W
  smInstP_ = SMOOTH*instP + (1-SMOOTH)*smInstP_;

  // 7) Stroke detection / energy integration
  float dAngle = angVel * dt;              // rad
  if (!inStroke_ && forceN > 500*forceCal_) {
    inStroke_   = true;
    strokeStart_= now;
    accEnergy_  = 0;
  }
  else if (inStroke_ && forceN < 300*forceCal_) {
    inStroke_ = false;
    uint32_t dur = now - strokeStart_;      // ms
    if (dur > 300) {
      float strokeP = accEnergy_ / (dur*0.001f); // W
      smStrokeP_ = SMOOTH*strokeP + (1-SMOOTH)*smStrokeP_;
      lastStrokeP_ = smStrokeP_;
    }
  }

  if (inStroke_) {
    accEnergy_ += torque * dAngle;          // dE = τ·dθ
  }
}

float PowerModel::computeAngularAcceleration(float angVel, float dt) {
  if (dt <= 0) return 0;
  float alpha = (angVel - lastAngVel_) / dt;
  lastAngVel_ = angVel;
  return alpha;
}

float PowerModel::getLeverArm() const           { return leverArm_; }
float PowerModel::getInstantaneousPower() const { return smInstP_; }
float PowerModel::getStrokePower() const        { return smStrokeP_; }
