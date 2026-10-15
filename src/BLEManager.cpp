#include "BLEManager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <set>

namespace {
// Cycling Power Measurement flags (Bluetooth CPS spec)
constexpr uint16_t CPM_FLAG_CRANK_DATA = 1 << 5;
// Cycling Power Feature bits
constexpr uint32_t CPF_CRANK_DATA_SUPPORTED = 1 << 3;
constexpr uint8_t SENSOR_LOCATION_OTHER = 0;
constexpr uint16_t APPEARANCE_CYCLING_POWER = 0x0484;
constexpr size_t MAX_CONNECTIONS = 3;  // CONFIG_BTDM_CTRL_BLE_MAX_CONN
constexpr size_t COMMAND_LEN = 64;
constexpr size_t MAX_CHUNK = 180;      // largest reply notification we send

BLEServer *server = nullptr;
BLECharacteristic *powerMeasurement = nullptr;
BLECharacteristic *batteryLevel = nullptr;
BLECharacteristic *liveChar = nullptr;
BLECharacteristic *strokeChar = nullptr;
BLECharacteristic *replyChar = nullptr;
uint8_t lastBattery = 0xFF;
char pin[7] = "";

// Events from the Bluetooth task, handled in loop(): command writes and
// disconnects. Doing the work in loop() keeps commands like "tare" (which
// read the sensors) off the Bluetooth task and away from the I²C bus.
enum EventType : uint8_t { EV_COMMAND, EV_DISCONNECT };
struct Event {
  EventType type;
  uint16_t conn;
  char text[COMMAND_LEN];
};
QueueHandle_t events = nullptr;
std::set<uint16_t> unlocked;  // loop() only

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    // Advertising stops on each connection; restart it so a second device
    // (phone + bike computer) can still connect.
    if (s->getConnectedCount() + 1 < MAX_CONNECTIONS) s->startAdvertising();
  }
  void onDisconnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    Event e = {EV_DISCONNECT, param->disconnect.conn_id, ""};
    xQueueSend(events, &e, 0);
    // Without this the paddle can never be reconnected after a disconnect.
    s->startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c, esp_ble_gatts_cb_param_t *param) override {
    Event e = {EV_COMMAND, param->write.conn_id, ""};
    std::string v = c->getValue();
    size_t n = v.size() < COMMAND_LEN - 1 ? v.size() : COMMAND_LEN - 1;
    memcpy(e.text, v.data(), n);
    e.text[n] = 0;
    xQueueSend(events, &e, 0);  // full queue: drop, the app will retry
  }
};

// Buffers reply text and notifies one line at a time, split into chunks the
// smallest connected device can receive.
class ReplyPrint : public Print {
public:
  size_t write(uint8_t c) override {
    if (len_ < sizeof buf_) buf_[len_++] = c;
    if (c == '\n' || len_ == sizeof buf_) flush();
    return 1;
  }
  void flush() override {
    if (!replyChar || !len_) return;
    size_t chunk = MAX_CHUNK;
    for (auto &peer : server->getPeerDevices(false)) {
      size_t room = peer.second.mtu > 3 ? peer.second.mtu - 3 : 20;
      if (room < chunk) chunk = room;
    }
    for (size_t off = 0; off < len_; off += chunk) {
      size_t n = len_ - off < chunk ? len_ - off : chunk;
      replyChar->setValue(buf_ + off, n);
      replyChar->notify();
      delay(4);  // let the stack drain; replies are rare and short
    }
    len_ = 0;
  }

private:
  uint8_t buf_[256];
  size_t len_ = 0;
};
ReplyPrint reply;

BLECharacteristic *makeNotify(BLEService *svc, const char *uuid) {
  BLECharacteristic *c = svc->createCharacteristic(
      BLEUUID(uuid), BLECharacteristic::PROPERTY_NOTIFY);
  c->addDescriptor(new BLE2902());
  return c;
}
}  // namespace

void setBlePin(const char *p) {
  strncpy(pin, p, sizeof pin - 1);
  pin[sizeof pin - 1] = 0;
  unlocked.clear();  // existing connections must authenticate again
}

void initBLE(const char *p) {
  setBlePin(p);
  events = xQueueCreate(8, sizeof(Event));

  BLEDevice::init("PaddlePower");
  BLEDevice::setMTU(185);  // phones negotiate up to this; fewer reply chunks
  server = BLEDevice::createServer();
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

  // App service: 4 characteristics, 3 with descriptors → 12 handles.
  BLEService *app = server->createService(BLEUUID(ble::SERVICE_UUID), 20);
  liveChar = makeNotify(app, ble::LIVE_UUID);
  strokeChar = makeNotify(app, ble::STROKE_UUID);
  replyChar = makeNotify(app, ble::REPLY_UUID);
  BLECharacteristic *command = app->createCharacteristic(
      BLEUUID(ble::COMMAND_UUID),
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  command->setCallbacks(new CommandCallbacks());

  cps->start();
  bas->start();
  app->start();

  // Head units scan for the service UUID; without it the sensor is invisible.
  // The app finds the paddle by name, so its 128-bit UUID (which wouldn't
  // fit in the advertisement) isn't advertised.
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID((uint16_t)0x1818));
  adv->addServiceUUID(BLEUUID((uint16_t)0x180F));
  adv->setAppearance(APPEARANCE_CYCLING_POWER);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

bool bleConnected() { return server && server->getConnectedCount() > 0; }

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
  if (bleConnected()) powerMeasurement->notify();
}

void sendBattery(uint8_t percent) {
  if (!batteryLevel) return;
  batteryLevel->setValue(&percent, 1);
  if (bleConnected() && percent != lastBattery) batteryLevel->notify();
  lastBattery = percent;
}

void sendLive(const ble::LivePacket &p) {
  if (!liveChar || !bleConnected()) return;
  liveChar->setValue((uint8_t *)&p, sizeof p);
  liveChar->notify();
}

void sendStroke(const ble::StrokePacket &p) {
  if (!strokeChar || !bleConnected()) return;
  strokeChar->setValue((uint8_t *)&p, sizeof p);
  strokeChar->notify();
}

bool takeBleCommand(String &cmd) {
  if (!events) return false;
  Event e;
  while (xQueueReceive(events, &e, 0) == pdTRUE) {
    if (e.type == EV_DISCONNECT) {
      unlocked.erase(e.conn);
      continue;
    }
    String text(e.text);
    text.trim();
    if (text.startsWith("auth ")) {
      String given = text.substring(5);
      given.trim();
      if (pin[0] && given == pin) {
        unlocked.insert(e.conn);
        reply.print("ok: unlocked\n");
      } else {
        reply.print("error: wrong PIN\n");
      }
      continue;
    }
    if (!unlocked.count(e.conn)) {
      reply.print("error: locked, send auth <pin>\n");
      continue;
    }
    cmd = text;
    return true;
  }
  return false;
}

Print &bleReply() { return reply; }
