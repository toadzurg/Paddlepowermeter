#ifndef BLEMANAGER_H
#define BLEMANAGER_H

#include <stdint.h>

// Standard Bluetooth Cycling Power Service (0x1818) + Battery Service
// (0x180F), so bike computers and apps can pair with the paddle directly.
// Strokes are reported as crank revolutions, which makes "cadence" on the
// head unit read strokes per minute.
void initBLE();
bool bleConnected();

// watts: average power. strokes: cumulative stroke count (wraps).
// lastStrokeMs: millis() of the most recent stroke.
void sendPowerMeasurement(int16_t watts, uint16_t strokes, uint32_t lastStrokeMs);
void sendBattery(uint8_t percent);

#endif  // BLEMANAGER_H
