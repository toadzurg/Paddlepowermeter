// Checks the app packet format (src/BleProtocol.h) and writes sample
// packets, as the firmware would send them, for the web app's decoder test:
//
//   test_ble_protocol OUT.json   (test/web/run.sh reads OUT.json)

#include <cmath>
#include <cstdio>
#include <cstring>

#include "BleProtocol.h"

static int failures = 0;

#define CHECK(name, cond)                                                     \
  do {                                                                        \
    bool ok_ = (cond);                                                        \
    std::printf("  [%s] %s\n", ok_ ? " ok " : "FAIL", name);                  \
    if (!ok_) failures++;                                                     \
  } while (0)

static void hex(std::FILE *f, const void *p, size_t n) {
  const uint8_t *b = static_cast<const uint8_t *>(p);
  std::fputc('"', f);
  for (size_t i = 0; i < n; i++) std::fprintf(f, "%02x", b[i]);
  std::fputc('"', f);
}

// Same conversions as sendAppPackets() in PaddlePowerMeter.ino.
static ble::LivePacket live(uint16_t seq, uint16_t tMs, uint8_t flags,
                            float force, float omega, float power,
                            float speed, float pivot, float lever,
                            float angle) {
  ble::LivePacket p;
  p.seq = seq;
  p.tMs = tMs;
  p.flags = flags;
  p.forceDeciN = ble::toI16(force * 10);
  p.omegaCrad = ble::toI16(omega * 100);
  p.instPowerW = ble::toI16(power);
  p.boatSpeedCms = ble::toU16(speed * 100);
  p.pivotMm = ble::toI16(pivot * 1000);
  p.leverMm = ble::toI16(lever * 1000);
  p.shaftAngleMrad = ble::toI16(angle * 1000);
  return p;
}

int main(int argc, char **argv) {
  std::printf("\nApp Bluetooth packets\n");
  CHECK("live packet fits a 20-byte notification", sizeof(ble::LivePacket) <= 20);
  CHECK("stroke packet fits a 20-byte notification", sizeof(ble::StrokePacket) <= 20);
  CHECK("force saturates instead of wrapping", ble::toI16(5000.0f * 10) == 32767);
  CHECK("negative saturates", ble::toI16(-1e9f) == -32768);
  CHECK("unsigned clamps negatives to 0", ble::toU16(-3.0f) == 0);
  CHECK("NaN encodes as 0", ble::toI16(NAN) == 0 && ble::toU16(NAN) == 0);
  CHECK("rounds to nearest", ble::toI16(12.6f) == 13 && ble::toI16(-12.6f) == -13);

  const char *path = argc > 1 ? argv[1] : "ble_samples.json";
  std::FILE *f = std::fopen(path, "w");
  if (!f) { std::perror(path); return 1; }

  // Expected values are what the app must show after decoding: the input
  // rounded to the packet's resolution, or the saturation limit.
  ble::LivePacket l1 = live(7, 65530, ble::LIVE_IN_DRIVE | ble::LIVE_BOAT_FRESH,
                            187.34f, 3.14159f, 612.4f, 2.456f, -0.137f, 1.087f,
                            -0.5236f);
  ble::LivePacket l2 = live(8, 15, ble::LIVE_LOGGING, -5000.0f, -2.5f, -40.0f,
                            -1.0f, 0.25f, 0.7f, 0.61f);
  ble::StrokePacket s;
  s.count = 42;
  s.powerW = ble::toI16(173.4f);
  s.rateCentiSpm = ble::toU16(50.04f * 100);
  s.energyDeciJ = ble::toU16(207.47f * 10);
  s.usefulW = ble::toI16(148.6f);
  s.slipW = ble::toI16(24.8f);
  s.efficiencyPermille = ble::toU16(1.0731f * 1000);  // >100% is allowed
  s.flags = ble::STROKE_HAS_SPLIT;
  s.impulseCentiNs = ble::toU16(78.456f * 100);
  s.driveMs = ble::toU16(0.5674f * 1000);

  std::fprintf(f, "{\n  \"live\": [\n    {\"hex\": ");
  hex(f, &l1, sizeof l1);
  std::fprintf(f, ", \"expect\": {\"seq\": 7, \"tMs\": 65530, \"inDrive\": true, "
                  "\"boatFresh\": true, \"logging\": false, \"force\": 187.3, "
                  "\"omega\": 3.14, \"instPower\": 612, \"boatSpeed\": 2.46, "
                  "\"pivot\": -0.137, \"lever\": 1.087, \"shaftAngle\": -0.524}},\n    {\"hex\": ");
  hex(f, &l2, sizeof l2);
  std::fprintf(f, ", \"expect\": {\"seq\": 8, \"tMs\": 15, \"inDrive\": false, "
                  "\"boatFresh\": false, \"logging\": true, \"force\": -3276.8, "
                  "\"omega\": -2.5, \"instPower\": -40, \"boatSpeed\": 0, "
                  "\"pivot\": 0.25, \"lever\": 0.7, \"shaftAngle\": 0.61}}\n  ],\n  \"stroke\": [\n    {\"hex\": ");
  hex(f, &s, sizeof s);
  std::fprintf(f, ", \"expect\": {\"count\": 42, \"power\": 173, \"rate\": 50.04, "
                  "\"energy\": 207.5, \"useful\": 149, \"slip\": 25, "
                  "\"efficiency\": 1.073, \"hasSplit\": true, \"impulse\": 78.46, "
                  "\"driveMs\": 567, \"duty\": 0.4729}}\n  ]\n}\n");
  std::fclose(f);

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
