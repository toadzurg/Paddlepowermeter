#ifndef POWERMODEL_H
#define POWERMODEL_H

#include <Arduino.h>
#include "Scale.h"
#include "IMU.h"
#include "Kalman.h"

class PowerModel {
public:
  // I: moment of inertia (kg·m²)
  // forceCal: raw → N, velGain: raw → rad/s
  // q,r,p: Kalman filter params for lever arm
  PowerModel(float I,
             float forceCal   = 0.01f,
             float velGain    = 1.0f,
             float kf_Q       = 0.0001f,
             float kf_R       = 0.01f,
             float kf_P       = 1.0f);

  // call this each loop pass
  void update(uint32_t nowMillis);

  // results
  float getLeverArm() const;
  float getInstantaneousPower() const;
  float getStrokePower() const;

private:
  // helper
  float computeAngularAcceleration(float angVel, float dt);
  float computeDynamicLeverArm(float torque, float force);

  // params
  float I_;            // moment of inertia
  float forceCal_, velGain_;
  Kalman leverFilter_;

  // state
  float lastAngVel_;
  uint32_t lastTime_;

  // power/stroke
  float smInstP_, smStrokeP_;
  bool inStroke_;
  uint32_t strokeStart_;
  float accEnergy_;
  float lastStrokeP_;
  float leverArm_;

  // smoothing
  static constexpr float SMOOTH = 0.2f;
};

#endif  // POWERMODEL_H
