#ifndef IMU_H
#define IMU_H

#include <MadgwickAHRS.h>
#include <SparkFun_BMI270_Arduino_Library.h>

extern Madgwick filter1;
extern Madgwick filter2;
extern float lastGx, lastGy, lastGz;

void initIMU();
void updateIMU();
void getIMU1Quat(float &qw, float &qx, float &qy, float &qz);
void getIMU2Quat(float &qw, float &qx, float &qy, float &qz);
void getIMUGyro(float &gx, float &gy, float &gz);

#endif
