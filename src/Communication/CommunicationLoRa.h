#ifndef COMMUNICATION_LORA_H
#define COMMUNICATION_LORA_H

#include <RadioLib.h>

extern uint8_t     keyNWKS[];
extern uint8_t     appKey[];
extern uint8_t     keyEncNWKS[];
extern uint64_t    joinEUI;
extern uint64_t    devEUI;
extern SX1276      radio;
extern LoRaWANNode node;

void loraTask(void *pvParameters);
void loRaSend(float value);

#endif