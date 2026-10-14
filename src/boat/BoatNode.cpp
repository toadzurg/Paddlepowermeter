// Boat node firmware: a second ESP32 mounted in the hull that reads the
// impeller and broadcasts its pulse rate to the paddle over ESP-NOW.
//
//   pio run -e boat -t upload
//
// Wiring: a 3.3 V digital Hall switch (e.g. TI DRV5032) next to the impeller
// magnets. Output → IMPELLER_PIN, VCC → 3V3, GND → GND. Open-drain outputs
// work as-is thanks to the internal pull-up. A reed switch also works, but
// raise MIN_INTERVAL_US to ~2000 to swallow contact bounce.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <BoatLinkPacket.h>
#include <ImpellerSpeed.h>

namespace {
constexpr int IMPELLER_PIN = 4;
constexpr uint32_t MIN_INTERVAL_US = 500;    // > 2 kHz is noise, not water
constexpr uint32_t TIMEOUT_US = 1500000;     // no pulse for 1.5 s = stopped
constexpr uint8_t RATE_WINDOW = 2;           // pulse intervals averaged
constexpr uint32_t PRINT_MS = 500;
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

ImpellerSpeed impeller(MIN_INTERVAL_US, TIMEOUT_US, RATE_WINDOW);

// Single-producer (ISR) / single-consumer (loop) ring of pulse timestamps.
// Both run on the same core, so volatile indices are sufficient.
constexpr uint8_t RING = 64;
volatile uint32_t ring[RING];
volatile uint8_t ringWr = 0, ringRd = 0;
volatile uint32_t ringOverflow = 0;

void IRAM_ATTR onImpellerEdge() {
  uint32_t t = micros();
  uint8_t next = (uint8_t)((ringWr + 1) % RING);
  if (next == ringRd) {
    ringOverflow = ringOverflow + 1;
    return;
  }
  ring[ringWr] = t;
  ringWr = next;
}

void drainPulses() {
  while (ringRd != ringWr) {
    impeller.onPulse(ring[ringRd]);
    ringRd = (uint8_t)((ringRd + 1) % RING);
  }
}

boatlink::Packet packet = {};
uint32_t lastSendMs = 0, lastPrintMs = 0;
bool radioOk = false;
}  // namespace

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(boatlink::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  radioOk = esp_now_init() == ESP_OK;
  if (radioOk) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST, 6);
    peer.channel = boatlink::WIFI_CHANNEL;
    peer.encrypt = false;
    radioOk = esp_now_add_peer(&peer) == ESP_OK;
  }
  Serial.printf("boat node %s, ESP-NOW %s on channel %u\n",
                WiFi.macAddress().c_str(), radioOk ? "ok" : "FAILED",
                boatlink::WIFI_CHANNEL);

  packet.magic = boatlink::MAGIC;
  packet.version = boatlink::VERSION;

  pinMode(IMPELLER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IMPELLER_PIN), onImpellerEdge, FALLING);
}

void loop() {
  drainPulses();

  uint32_t nowMs = millis();
  if (nowMs - lastSendMs >= boatlink::SEND_INTERVAL_MS) {
    lastSendMs = nowMs;
    uint32_t nowUs = micros();
    packet.seq++;
    packet.pulseCount = impeller.count();
    packet.pulseHz = impeller.rateHz(nowUs);
    uint32_t sinceUs = impeller.usSinceLastPulse(nowUs);
    packet.msSincePulse = sinceUs == 0xFFFFFFFFu ? 0xFFFFFFFFu : sinceUs / 1000;
    if (radioOk)
      esp_now_send(BROADCAST, reinterpret_cast<const uint8_t *>(&packet),
                   sizeof packet);
  }

  // Bench check: spin the impeller by hand and watch the count climb.
  if (nowMs - lastPrintMs >= PRINT_MS) {
    lastPrintMs = nowMs;
    Serial.printf("pulses %lu  rate %.1f Hz  glitches %lu  overflow %lu\n",
                  (unsigned long)impeller.count(), packet.pulseHz,
                  (unsigned long)impeller.rejected(),
                  (unsigned long)ringOverflow);
  }
}
