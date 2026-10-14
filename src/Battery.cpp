#include "Battery.h"
#include <Wire.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

namespace {
// The library defaults to the MAX17043, whose SOC register is scaled
// differently; the board uses a MAX17048.
SFE_MAX1704X lipo(MAX1704X_MAX17048);
bool ok = false;
}

bool initBattery() {
  ok = lipo.begin(Wire);
  return ok;
}

uint8_t getBatteryLevel() {
  if (!ok) return 0;
  float soc = lipo.getSOC();
  if (soc < 0) soc = 0;
  if (soc > 100) soc = 100;
  return (uint8_t)(soc + 0.5f);
}
