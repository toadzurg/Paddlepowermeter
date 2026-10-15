#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

#include <math.h>
#include <stdint.h>

// Custom Bluetooth service used by the web app (docs/app/index.html), next
// to the standard Cycling Power service that bike computers use.
//
//   live     notify, 20 Hz   LivePacket: force, rotation, power, boat speed
//   stroke   notify          StrokePacket after every completed stroke
//   command  write           text commands, same as the serial console
//   reply    notify          text replies, in chunks; lines end in '\n'
//
// Commands are locked until the connection sends "auth <pin>". Live and
// stroke data are readable without it.
//
// Packets are little-endian and at most 20 bytes, so they fit the smallest
// Bluetooth packet size every phone and laptop supports. Keep the decoders
// in docs/app/index.html in sync with these structs.

namespace ble {

constexpr const char *SERVICE_UUID = "7e1a0001-5c3b-4c2e-9a77-0f5d3e2b9c10";
constexpr const char *LIVE_UUID    = "7e1a0002-5c3b-4c2e-9a77-0f5d3e2b9c10";
constexpr const char *STROKE_UUID  = "7e1a0003-5c3b-4c2e-9a77-0f5d3e2b9c10";
constexpr const char *COMMAND_UUID = "7e1a0004-5c3b-4c2e-9a77-0f5d3e2b9c10";
constexpr const char *REPLY_UUID   = "7e1a0005-5c3b-4c2e-9a77-0f5d3e2b9c10";

constexpr uint32_t LIVE_INTERVAL_MS = 50;  // 20 Hz: ~12 points per drive

// LivePacket.flags
constexpr uint8_t LIVE_IN_DRIVE    = 1 << 0;
constexpr uint8_t LIVE_BOAT_FRESH  = 1 << 1;
constexpr uint8_t LIVE_LOGGING     = 1 << 2;

struct __attribute__((packed)) LivePacket {
  uint16_t seq;
  uint16_t tMs;          // device time, wraps every 65.5 s
  uint8_t flags;
  int16_t forceDeciN;    // 0.1 N
  int16_t omegaCrad;     // 0.01 rad/s
  int16_t instPowerW;
  uint16_t boatSpeedCms; // cm/s
  int16_t pivotMm;
  int16_t leverMm;
};

// StrokePacket.flags
constexpr uint8_t STROKE_HAS_SPLIT = 1 << 0;

struct __attribute__((packed)) StrokePacket {
  uint16_t count;
  int16_t powerW;
  uint16_t rateCentiSpm;   // 0.01 strokes/min
  uint16_t energyDeciJ;    // 0.1 J
  int16_t usefulW;
  int16_t slipW;
  uint16_t efficiencyPermille;  // not capped at 1000, like efficiency()
  uint8_t flags;
};

static_assert(sizeof(LivePacket) == 17, "LivePacket layout changed");
static_assert(sizeof(StrokePacket) == 15, "StrokePacket layout changed");

// Saturating conversions: an out-of-range value pins at the limit instead
// of wrapping to nonsense.
inline int16_t toI16(float v) {
  if (!(v == v)) return 0;  // NaN
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}
inline uint16_t toU16(float v) {
  if (!(v == v) || v < 0) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

}  // namespace ble

#endif  // BLE_PROTOCOL_H
