#include "Scale.h"
#include "IMU.h"
#include "BLEManager.h"
#include "PowerModel.h"
#include "Battery.h"

void setup() {
    Serial.begin(115200);

    initScale();
    initIMU();
    initBattery();
    initBLE();
}

void loop() {
    updateScale();
    updateIMU();
    float strokeRate = calculateStrokeRate();
    float power = calculatePower();

    updateBattery();
    sendBLEData(power, strokeRate, getBatteryLevel());

    delay(100);
}