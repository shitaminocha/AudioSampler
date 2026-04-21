#include <RadioLib.h>

// declare only — no memory allocated here
extern uint8_t keyNWKS[];
extern uint8_t appKey[];
extern uint64_t joinEUI;
extern uint64_t devEUI;
extern SX1276   radio;
extern LoRaWANNode node;

void loraTask(void *pvParameters);
void loRaSend(float value);