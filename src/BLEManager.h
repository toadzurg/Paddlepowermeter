#ifndef BLEMANAGER_H
#define BLEMANAGER_H

#include <Arduino.h>
#include <stdint.h>
#include "BleProtocol.h"

// Bluetooth: the standard Cycling Power Service (0x1818) and Battery
// Service (0x180F), so bike computers and apps can pair with the paddle
// directly, plus the custom app service in BleProtocol.h. Strokes are
// reported as crank revolutions, which makes "cadence" on the head unit read
// strokes per minute. Up to three devices can be connected at once (e.g. a
// bike computer, a phone and a laptop).

// pin: 6 digits that unlock app commands ("auth <pin>").
void initBLE(const char *pin);
void setBlePin(const char *pin);
bool bleConnected();

// watts: average power. strokes: cumulative stroke count (wraps).
// lastStrokeMs: millis() of the most recent stroke.
void sendPowerMeasurement(int16_t watts, uint16_t strokes, uint32_t lastStrokeMs);
void sendBattery(uint8_t percent);

void sendLive(const ble::LivePacket &p);
void sendStroke(const ble::StrokePacket &p);

// Call from loop(). Returns true with the next unlocked app command. Handles
// "auth" and rejects commands from locked connections itself.
bool takeBleCommand(String &cmd);

// Text sink for replies to app commands: lines are sent to the reply
// characteristic as they complete.
Print &bleReply();

#endif  // BLEMANAGER_H
