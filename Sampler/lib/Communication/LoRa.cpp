#include "LoRa.h"
#include <heltec_unofficial.h>

// Retain RTC memory across deep sleep cycles for persistent LoRaWAN session states
RTC_DATA_ATTR static uint8_t lwSessionStore[712];

// Initialize static members
LoRaWANNode* LoRaManager::node = nullptr;
float data;
volatile bool LoRaManager::IsReadyForTransmission = false;

// Helper function for RadioLib low-power task delay
static void loRaSleepCallback(RadioLibTime_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void LoRaManager::begin(uint8_t* appEui, uint8_t* devEui, uint8_t* appKey) {
    heltec_setup();
    
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] Hardware initialization failed! Code: %d\n", state);
        return;
    }

    // Allocate keys dynamically on the heap to pass safely to the xTask
    LoRaKeys* savedKeys = new LoRaKeys();
    memcpy(savedKeys->joinEui, appEui, 8);
    memcpy(savedKeys->devEui, devEui, 8);
    memcpy(savedKeys->appKey, appKey, 16);

    LoRaManager::IsReadyForTransmission = false;

    // Allocate 8192 byte stack for RadioLib cryptographic operations
    xTaskCreatePinnedToCore(
        loraTaskWorker,     
        "LoRaWorker",       
        8192,               
        (void*)savedKeys,   
        2,                  
        NULL,               
        1                   
    );
}

void LoRaManager::triggerTransmission(float* aggregate, size_t count) {
    if (IsReadyForTransmission) return; // Prevent overwriting active buffer
    data = *aggregate;
    IsReadyForTransmission = true;
}

bool LoRaManager::isReady() {
    return !IsReadyForTransmission;
}

void LoRaManager::loraTaskWorker(void *pvParameters) {
    LoRaKeys* keys = (LoRaKeys*)pvParameters;

    // 'persist' is provided automatically as a global object by LoRaWAN_ESP32.h
    node = persist.manage(&radio, lwSessionStore);
    //node->setSleepFunction(loRaSleepCallback);

    Serial.printf("[LoRa] Node Activated State: %d\n", node->isActivated());
    
    if (!node->isActivated()) {
        Serial.println("[LoRa Task] Configuring TTN OTAA credentials...");
        
        node->beginOTAA(
            *((uint64_t*)keys->joinEui), 
            *((uint64_t*)keys->devEui), 
            keys->appKey,
            keys->appKey
        );

        Serial.println("[LoRa Task] Broadcasting Join Request...");
        
        // Blocks internally until RX windows close or join succeeds
        int16_t state = node->activateOTAA();
        
        if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_LORAWAN_NEW_SESSION) {
            Serial.println("[LoRa Task] Successfully Joined TTN!");
            persist.saveSession(node);
        } else {
            Serial.printf("[LoRa Task] OTAA Join Failed! Error Code: %d\n", state);
            delete keys;       
            vTaskDelete(NULL); 
            return;
        }
    }

    // Clean up heap allocation after successful configuration/join
    delete keys;

    // --- Main Transmission Loop ---
    while (true) {
        if (!IsReadyForTransmission) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int16_t encodedData = (int16_t)(data * 100.0f);

        uint8_t payload[2];
        payload[0] = (encodedData >> 8) & 0xFF;
        payload[1] = encodedData & 0xFF;

        Serial.printf("[LoRa Task] Uploading frame...\n");

        String strDownlinkResponse = "";
        int16_t state = node->sendReceive(payload, sizeof(payload), 1, strDownlinkResponse);

        if (state == RADIOLIB_ERR_NONE)
            Serial.println("[LoRa Task] Uplink broadcast complete!");
        else
            Serial.printf("[LoRa Task] Transmission skipped/dropped. Error: %d\n", state);

        IsReadyForTransmission = false;
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}