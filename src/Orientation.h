#ifndef ORIENTATION_H
#define ORIENTATION_H

// Madgwick 6-DOF orientation filter (gyro + accelerometer).
//
// Unlike the Arduino MadgwickAHRS library this takes gyro in rad/s and the
// real dt of every sample, so it stays correct when the loop rate jitters.
class Orientation {
public:
  explicit Orientation(float beta);

  // Snap roll/pitch to the measured gravity direction (call once at rest).
  void initFromAccel(float ax, float ay, float az);

  // gyro in rad/s, accel in any unit (only its direction is used).
  void update(float gx, float gy, float gz,
              float ax, float ay, float az, float dt);

  // Unit vector pointing "up" (against gravity) in the sensor frame. A
  // stationary accelerometer reads +1 g along this direction.
  void upVector(float &ux, float &uy, float &uz) const;

  void quaternion(float &w, float &x, float &y, float &z) const;

private:
  float beta_;
  float q0_, q1_, q2_, q3_;
};

#endif  // ORIENTATION_H
