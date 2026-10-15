#ifndef BOAT_LINK_PACKET_H
#define BOAT_LINK_PACKET_H

#include <stdint.h>

// ESP-NOW packet broadcast by the boat node (impeller) to the paddle.
//
// The boat node sends raw pulse data; the paddle owns the calibration
// (pulses per metre), so every setting lives in one place.
namespace boatlink {

constexpr uint32_t MAGIC = 0x31425750;    // "PWB1" little-endian
constexpr uint8_t VERSION = 1;
constexpr uint8_t WIFI_CHANNEL = 1;       // both boards must match
constexpr uint32_t SEND_INTERVAL_MS = 50; // 20 Hz: resolves surge per stroke

struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint8_t version;
  uint32_t seq;           // increments every packet
  uint32_t pulseCount;    // cumulative accepted pulses (wraps)
  float pulseHz;          // current pulse rate, 0 when stopped
  uint32_t msSincePulse;  // time since the last pulse
};

}  // namespace boatlink

#endif  // BOAT_LINK_PACKET_H
