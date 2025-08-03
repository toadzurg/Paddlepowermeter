#ifndef POWERMODEL_H
#define POWERMODEL_H

#include <CircularBuffer.h>
#include "Kalman.h"

struct Vec3 {
  float x,y,z;
  Vec3():x(0),y(0),z(0){}
  Vec3(float _x,float _y,float _z):x(_x),y(_y),z(_z){}
  Vec3 operator*(float s) const { return Vec3(x*s,y*s,z*s); }
  Vec3 operator+(const Vec3 &o) const { return Vec3(x+o.x,y+o.y,z+o.z); }
  Vec3 operator-(const Vec3 &o) const { return Vec3(x-o.x,y-o.y,z-o.z); }
  float dot(const Vec3 &o) const { return x*o.x+ y*o.y+ z*o.z; }
  float length() const { return sqrt(dot(*this)); }
};

Vec3 rotateVecByQuat(const Vec3 &v, float qw, float qx, float qy, float qz);

class PowerModel {
public:
  PowerModel(float leverNominal, float forceCal, float velGain,
             float q, float r, float p);
  void update(uint32_t now);
  float getInstantaneousPower() const;
  float getStrokePower() const;
  float getStrokeRate() const;
private:
  float computeDynamicLeverArm();
  void detectStroke(float force, float angle, uint32_t now);
  void integrateEnergy(float torque, float dAngle);

  CircularBuffer<float, 10> instBuf_;
  CircularBuffer<float, 10> strokeBuf_;
  CircularBuffer<float, 10> spmBuf_;
  Kalman leverArmFilter_;

  float leverNominal_, forceCal_, velGain_;
  bool inStroke_;
  uint32_t strokeStart_;
  float accEnergy_;
  float smInstP_, smStrokeP_, smSPM_;
  uint32_t lastTime_, lastStrokeTime_;
};

#endif
