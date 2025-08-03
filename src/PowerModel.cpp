#include "PowerModel.h"
#include "Scale.h"
#include "IMU.h"
#include <Arduino.h>
#define SMOOTH 0.2f

Vec3 rotateVecByQuat(const Vec3 &v, float qw, float qx, float qy, float qz) {
  Vec3 qv(qx,qy,qz);
  Vec3 t = Vec3(
    2*(qv.y*v.z - qv.z*v.y),
    2*(qv.z*v.x - qv.x*v.z),
    2*(qv.x*v.y - qv.y*v.x)
  );
  Vec3 u = Vec3(
    qv.y*t.z - qv.z*t.y,
    qv.z*t.x - qv.x*t.z,
    qv.x*t.y - qv.y*t.x
  );
  return Vec3(
    v.x + qw*t.x + u.x,
    v.y + qw*t.y + u.y,
    v.z + qw*t.z + u.z
  );
}

PowerModel::PowerModel(float leverNominal, float forceCal, float velGain,
                       float q, float r, float p)
 : leverNominal_(leverNominal),
   forceCal_(forceCal),
   velGain_(velGain),
   leverArmFilter_(q,r,p,leverNominal),
   inStroke_(false),
   strokeStart_(0),
   accEnergy_(0),
   smInstP_(0),
   smStrokeP_(0),
   smSPM_(0),
   lastTime_(0),
   lastStrokeTime_(0)
{}

void PowerModel::update(uint32_t now) {
  float dt = lastTime_==0 ? 0 : (now - lastTime_)*0.001f;
  lastTime_ = now;

  float rawCount = getForce();
  float forceN = rawCount * forceCal_;
  float angVel = getAngularVelocity() * velGain_;
  float torque = forceN * computeDynamicLeverArm();

  // instantaneous power
  float instP = torque * angVel;
  smInstP_ = SMOOTH*instP + (1-SMOOTH)*smInstP_;
  instBuf_.push(smInstP_);

  // stroke detection & energy
  float dAngle = angVel * dt;
  detectStroke(forceN, dAngle, now);

  // stroke power
  smStrokeP_ = smStrokeP_; // updated in detectStroke end
  strokeBuf_.push(smStrokeP_);

  // stroke rate in SPM
  float spm = smSPM_;
  spmBuf_.push(spm);
}

float PowerModel::getInstantaneousPower() const { return smInstP_; }
float PowerModel::getStrokePower() const       { return smStrokeP_; }
float PowerModel::getStrokeRate() const        { return smSPM_; }

void PowerModel::detectStroke(float force, float dAngle, uint32_t now) {
  float startT = 1000 * forceCal_;
  float endT = 500 * forceCal_;
  if(!inStroke_ && force>startT) {
    inStroke_=true;
    accEnergy_ = 0;
    strokeStart_ = now;
  } else if(inStroke_ && force<endT) {
    inStroke_=false;
    uint32_t dur = now - strokeStart_;
    if(dur>300) {
      float strokeP = accEnergy_ / (dur*0.001f);
      smStrokeP_ = SMOOTH*strokeP + (1-SMOOTH)*smStrokeP_;
      smSPM_ = 60000.0f/dur;
    }
  }
  if(inStroke_) integrateEnergy(force*dAngle, dAngle);
}

void PowerModel::integrateEnergy(float torque, float dAngle) {
  accEnergy_ += torque * dAngle;
}

float PowerModel::computeDynamicLeverArm() {
  // quaternions
  float w1,x1,y1,z1, w2,x2,y2,z2;
  getIMU1Quat(w1,x1,y1,z1);
  getIMU2Quat(w2,x2,y2,z2);
  // local points
  Vec3 p1(0,0,0), p2(leverNominal_,0,0);
  Vec3 pw1 = rotateVecByQuat(p1,w1,x1,y1,z1);
  Vec3 pw2 = rotateVecByQuat(p2,w2,x2,y2,z2);
  Vec3 diff = pw2 - pw1;
  Vec3 axis = [&](){ float gx,gy,gz; getIMUGyro(gx,gy,gz); Vec3 v(gx,gy,gz); return v.length()?v*(1.0f/v.length()):Vec3(1,0,0); }();
  Vec3 crossP(diff.y*axis.z - diff.z*axis.y,
              diff.z*axis.x - diff.x*axis.z,
              diff.x*axis.y - diff.y*axis.x);
  float rawArm = crossP.length();
  return leverArmFilter_.update(rawArm);
}
