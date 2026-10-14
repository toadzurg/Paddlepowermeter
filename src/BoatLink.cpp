#include "BoatLink.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <BoatLinkPacket.h>

namespace {
constexpr uint32_t FRESH_MS = 300;  // ~6 missed packets at 20 Hz

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t paired[6];
bool havePaired = false;
bool pairing = false;
bool newPairing = false;
boatlink::Packet latest = {};
uint32_t latestMs = 0;
bool haveLatest = false;

void handle(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != (int)sizeof(boatlink::Packet)) return;
  boatlink::Packet p;
  memcpy(&p, data, sizeof p);
  if (p.magic != boatlink::MAGIC || p.version != boatlink::VERSION) return;

  portENTER_CRITICAL(&mux);
  if (pairing) {
    memcpy(paired, mac, 6);
    havePaired = true;
    pairing = false;
    newPairing = true;
    haveLatest = false;
  }
  if (havePaired && memcmp(mac, paired, 6) == 0) {
    latest = p;
    latestMs = millis();
    haveLatest = true;
  }
  portEXIT_CRITICAL(&mux);
}

// The receive callback signature changed in arduino-esp32 3.x.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handle(info->src_addr, data, len);
}
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  handle(mac, data, len);
}
#endif
}  // namespace

bool initBoatLink(const uint8_t *pairedMac) {
  if (pairedMac) {
    memcpy(paired, pairedMac, 6);
    havePaired = true;
  }
  // Wi-Fi radio for ESP-NOW only (no access point); coexists with BLE.
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(boatlink::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  return esp_now_register_recv_cb(onReceive) == ESP_OK;
}

void startBoatPairing() {
  portENTER_CRITICAL(&mux);
  pairing = true;
  portEXIT_CRITICAL(&mux);
}

bool takeNewPairing(uint8_t mac[6]) {
  bool got = false;
  portENTER_CRITICAL(&mux);
  if (newPairing) {
    memcpy(mac, paired, 6);
    newPairing = false;
    got = true;
  }
  portEXIT_CRITICAL(&mux);
  return got;
}

bool boatLinkPaired() { return havePaired; }

BoatSpeed readBoatSpeed(float pulsesPerMetre) {
  boatlink::Packet p;
  uint32_t at;
  bool have;
  portENTER_CRITICAL(&mux);
  p = latest;
  at = latestMs;
  have = haveLatest;
  portEXIT_CRITICAL(&mux);

  BoatSpeed s = {};
  if (!have) {
    s.ageMs = 0xFFFFFFFFu;
    return s;
  }
  s.ageMs = millis() - at;
  s.fresh = s.ageMs <= FRESH_MS;
  s.pulseHz = p.pulseHz;
  s.pulseCount = p.pulseCount;
  s.speedMps = (s.fresh && pulsesPerMetre > 0) ? p.pulseHz / pulsesPerMetre : 0;
  return s;
}
