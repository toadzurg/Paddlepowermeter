#include "BLEManager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *powerChar;
BLECharacteristic *batteryChar;

void initBLE() {
  BLEDevice::init("PaddlePower");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *powerService = pServer->createService(BLEUUID((uint16_t)0x1818));
  BLEService *batteryService = pServer->createService(BLEUUID((uint16_t)0x180F));

  powerChar = powerService->createCharacteristic(
    BLEUUID((uint16_t)0x2A63),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  powerChar->addDescriptor(new BLE2902());

  batteryChar = batteryService->createCharacteristic(
    BLEUUID((uint16_t)0x2A19),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  batteryChar->addDescriptor(new BLE2902());

  powerService->start();
  batteryService->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->start();
}

void sendBLEData(float power, float spm, int battery) {
  uint8_t buf[8];
  int16_t p = (int16_t)(power * 10);
  int16_t r = (int16_t)(spm * 10);
  buf[0] = p & 0xFF; buf[1] = p>>8;
  buf[2] = r & 0xFF; buf[3] = r>>8;
  buf[4] = battery;
  for(int i=5;i<8;i++) buf[i]=0;
  powerChar->setValue(buf,8);
  powerChar->notify();
  batteryChar->setValue(&buf[4],1);
  batteryChar->notify();
}
