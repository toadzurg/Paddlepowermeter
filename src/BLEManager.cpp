#include "BLEManager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

namespace {
// Cycling Power Measurement flags (Bluetooth CPS spec)
constexpr uint16_t CPM_FLAG_CRANK_DATA = 1 << 5;
// Cycling Power Feature bits
constexpr uint32_t CPF_CRANK_DATA_SUPPORTED = 1 << 3;
constexpr uint8_t SENSOR_LOCATION_OTHER = 0;
constexpr uint16_t APPEARANCE_CYCLING_POWER = 0x0484;

BLECharacteristic *powerMeasurement = nullptr;
BLECharacteristic *batteryLevel = nullptr;
volatile bool connected = false;
uint8_t lastBattery = 0xFF;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; }
  void onDisconnect(BLEServer *server) override {
    connected = false;
    // The stack stops advertising on connect; without this the paddle can
    // never be reconnected after the first disconnect.
    server->startAdvertising();
  }
};
}  // namespace

void initBLE() {
  BLEDevice::init("PaddlePower");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *cps = server->createService(BLEUUID((uint16_t)0x1818));
  powerMeasurement = cps->createCharacteristic(
      BLEUUID((uint16_t)0x2A63), BLECharacteristic::PROPERTY_NOTIFY);
  powerMeasurement->addDescriptor(new BLE2902());

  // Cycling Power Feature and Sensor Location are mandatory for the service.
  BLECharacteristic *feature = cps->createCharacteristic(
      BLEUUID((uint16_t)0x2A65), BLECharacteristic::PROPERTY_READ);
  uint8_t featureBytes[4] = {
      (uint8_t)(CPF_CRANK_DATA_SUPPORTED & 0xFF), 0, 0, 0};
  feature->setValue(featureBytes, sizeof featureBytes);

  BLECharacteristic *location = cps->createCharacteristic(
      BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);
  uint8_t loc = SENSOR_LOCATION_OTHER;
  location->setValue(&loc, 1);

  BLEService *bas = server->createService(BLEUUID((uint16_t)0x180F));
  batteryLevel = bas->createCharacteristic(
      BLEUUID((uint16_t)0x2A19),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  batteryLevel->addDescriptor(new BLE2902());
  uint8_t zero = 0;
  batteryLevel->setValue(&zero, 1);

  cps->start();
  bas->start();

  // Head units scan for the service UUID; without it the sensor is invisible.
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID((uint16_t)0x1818));
  adv->addServiceUUID(BLEUUID((uint16_t)0x180F));
  adv->setAppearance(APPEARANCE_CYCLING_POWER);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

bool bleConnected() { return connected; }

void sendPowerMeasurement(int16_t watts, uint16_t strokes, uint32_t lastStrokeMs) {
  if (!powerMeasurement) return;
  // Event time is in 1/1024 s and wraps every 64 s, per the spec.
  uint16_t eventTime = (uint16_t)(((uint64_t)lastStrokeMs * 1024) / 1000);
  uint8_t buf[8];
  buf[0] = CPM_FLAG_CRANK_DATA & 0xFF;
  buf[1] = CPM_FLAG_CRANK_DATA >> 8;
  buf[2] = (uint16_t)watts & 0xFF;  // sint16, 1 W resolution
  buf[3] = (uint16_t)watts >> 8;
  buf[4] = strokes & 0xFF;
  buf[5] = strokes >> 8;
  buf[6] = eventTime & 0xFF;
  buf[7] = eventTime >> 8;
  powerMeasurement->setValue(buf, sizeof buf);
  if (connected) powerMeasurement->notify();
}

void sendBattery(uint8_t percent) {
  if (!batteryLevel) return;
  batteryLevel->setValue(&percent, 1);
  if (connected && percent != lastBattery) batteryLevel->notify();
  lastBattery = percent;
}
