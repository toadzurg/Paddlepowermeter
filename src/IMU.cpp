#include "IMU.h"
#include <Wire.h>

Madgwick filter1, filter2;
BMI270 imu1, imu2;
float lastGx = 0, lastGy = 0, lastGz = 0;

void initIMU() {
  Wire.begin();
  imu1.begin(0x68);
  imu2.begin(0x69);
  filter1.begin(100.0f);
  filter2.begin(100.0f);
}

void updateIMU() {
  sensors_event_t g1, a1, t1;
  sensors_event_t g2, a2, t2;
  imu1.getEvent(&a1, &g1, &t1);
  imu2.getEvent(&a2, &g2, &t2);
  lastGx = g1.gyro.x;
  lastGy = g1.gyro.y;
  lastGz = g1.gyro.z;
  filter1.updateIMU(g1.gyro.x, g1.gyro.y, g1.gyro.z,
                    a1.acceleration.x, a1.acceleration.y, a1.acceleration.z);
  filter2.updateIMU(g2.gyro.x, g2.gyro.y, g2.gyro.z,
                    a2.acceleration.x, a2.acceleration.y, a2.acceleration.z);
}

void getIMU1Quat(float &qw, float &qx, float &qy, float &qz) {
  filter1.getQuaternion(&qw, &qx, &qy, &qz);
}
void getIMU2Quat(float &qw, float &qx, float &qy, float &qz) {
  filter2.getQuaternion(&qw, &qx, &qy, &qz);
}
void getIMUGyro(float &gx, float &gy, float &gz) {
  gx = lastGx; gy = lastGy; gz = lastGz;
}
