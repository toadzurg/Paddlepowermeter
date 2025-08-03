#include "Battery.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

SFE_MAX1704X lipo;

void initBattery() {
    lipo.begin();
}

void updateBattery() {
    lipo.quickStart();
}

int getBatteryLevel() {
    return (int)lipo.getSOC();
}