#ifndef KALMAN_H
#define KALMAN_H

class Kalman {
public:
  Kalman(float q, float r, float p, float x0);
  float update(float measurement);
private:
  float Q, R, P, X, K;
};

#endif
