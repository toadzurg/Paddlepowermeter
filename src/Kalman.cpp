#include "Kalman.h"

Kalman::Kalman(float q, float r, float p, float x0)
  : Q(q), R(r), P(p), X(x0), K(0) {}

float Kalman::update(float measurement) {
  P += Q;
  K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;
  return X;
}
