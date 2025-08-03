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

    BLEService *powerService = pServer->createService(BLEUUID((uint16_t)0x1818));  // Cycling Power
    BLEService *batteryService = pServer->createService(BLEUUID((uint16_t)0x180F)); // Battery

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
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->start();
}

void sendBLEData(float power, float spm, int battery) {
    uint8_t buffer[8];
    int16_t p = (int16_t)(power * 10);  // 0.1 watt resolution
    int16_t r = (int16_t)(spm * 10);    // 0.1 spm resolution

    buffer[0] = p & 0xFF;
    buffer[1] = (p >> 8) & 0xFF;
    buffer[2] = r & 0xFF;
    buffer[3] = (r >> 8) & 0xFF;
    buffer[4] = battery;
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;

    powerChar->setValue(buffer, 8);
    powerChar->notify();

    batteryChar->setValue(&battery, 1);
    batteryChar->notify();
}