#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

bool initBattery();
uint8_t getBatteryLevel();  // 0–100 %

#endif  // BATTERY_H
