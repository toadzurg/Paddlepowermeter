#include "PowerModel.h"
#include "Scale.h"
#include "IMU.h"
#include <Arduino.h>
#include <CircularBuffer.h>

#define FORCE_THRESHOLD_START 1000
#define FORCE_THRESHOLD_END   500
#define SMOOTHING_ALPHA       0.2  // Exponential moving average

static unsigned long lastStrokeTime = 0;
static bool inStroke = false;

static float smoothedPower = 0.0;
static float smoothedSPM = 0.0;

CircularBuffer<float, 10> powerBuffer;
CircularBuffer<float, 10> spmBuffer;

// Calibration constant (example: scale counts to Newtons)
static float calibrationScale = 0.01;  // counts to Newtons
static float velocityGain = 1.0;       // scale IMU velocity to m/s

float calculatePower() {
    float force = getForce() * calibrationScale;
    float velocity = getAngularVelocity() * velocityGain;

    float power = force * velocity;
    powerBuffer.push(power);

    // Exponential smoothing
    smoothedPower = SMOOTHING_ALPHA * power + (1 - SMOOTHING_ALPHA) * smoothedPower;
    return smoothedPower;
}

float calculateStrokeRate() {
    unsigned long now = millis();
    float force = getForce() * calibrationScale;
    float spm = smoothedSPM;

    if (!inStroke && force > FORCE_THRESHOLD_START) {
        inStroke = true;
        if (lastStrokeTime != 0) {
            unsigned long interval = now - lastStrokeTime;
            if (interval > 300) {  // ignore spurious strokes < 300ms apart
                spm = 60000.0 / interval;
                spmBuffer.push(spm);
                smoothedSPM = SMOOTHING_ALPHA * spm + (1 - SMOOTHING_ALPHA) * smoothedSPM;
            }
        }
        lastStrokeTime = now;
    } else if (inStroke && force < FORCE_THRESHOLD_END) {
        inStroke = false;
    }

    return smoothedSPM;
}