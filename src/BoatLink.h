#ifndef BOATLINK_H
#define BOATLINK_H

#include <stdint.h>

// Paddle-side receiver for the boat node's impeller broadcasts (ESP-NOW).
// Only packets from the paired boat node are used, so other paddlers'
// boat nodes at a race start are ignored.

struct BoatSpeed {
  bool fresh;          // a packet arrived recently
  float pulseHz;       // raw impeller rate
  uint32_t pulseCount; // cumulative pulses (wraps)
  float speedMps;      // boat speed through the water (0 if uncalibrated)
  uint32_t ageMs;      // time since the last packet
};

// pairedMac may be null (not paired yet). Call after initBLE().
bool initBoatLink(const uint8_t *pairedMac);

// Accept the next boat node heard as the paired one. Do this away from
// other boats.
void startBoatPairing();

// Returns true once when pairing completes, with the new MAC to store.
bool takeNewPairing(uint8_t mac[6]);

bool boatLinkPaired();
BoatSpeed readBoatSpeed(float pulsesPerMetre);

#endif  // BOATLINK_H
