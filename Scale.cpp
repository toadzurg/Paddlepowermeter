#include "Scale.h"
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

NAU7802 myScale;

void initScale() {
    Wire.begin();
    myScale.begin();
    myScale.setGain(128);
    myScale.setSampleRate(NAU7802_SPS_40);
    myScale.calibrateAFE();
}

void updateScale() {
    // Optional: filter or buffer readings
}

float getForce() {
    return myScale.getReading();
}