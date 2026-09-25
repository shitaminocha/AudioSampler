#ifndef LORA_H
#define LORA_H

// 1. MUST include Arduino.h before Heltec headers to avoid GPIO enum collisions
#include <Arduino.h>

// 2. Undefine standard SPI/I2C pin macros if heltec_unofficial conflicts with pins_arduino
#undef SS
#undef MOSI
#undef MISO
#undef SCK

#include <LoRaWAN_ESP32.h>

// Struct for passing keys safely into the task
struct LoRaKeys {
    uint8_t joinEui[8];
    uint8_t devEui[8];
    uint8_t appKey[16];
};

class LoRaManager {
private:
    static LoRaWANNode* node;
    static volatile bool IsReadyForTransmission;

    // FreeRTOS task worker
    static void loraTaskWorker(void* pvParameters);

public:
    static void begin(uint8_t* appEui, uint8_t* devEui, uint8_t* appKey);
    static void triggerTransmission(float* data, size_t count);
    static bool isReady();
};

#endif // LORA_H