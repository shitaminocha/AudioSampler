#include "CommunicationLoRa.h"

uint8_t keyNWKS[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[]  = { 0xB3, 0x49, 0x0B, 0x9D, 0x2B, 0x5D, 0x96, 0x03,
                      0x8F, 0xD6, 0xC8, 0x27, 0xA1, 0x89, 0x3F, 0x96 };
uint64_t joinEUI  = 0x0000000000000000;
uint64_t devEUI   = 0x2C7107D07ED5B370;

SX1276     radio = new Module(18, 26, 14, 33);
LoRaWANNode node(&radio, &EU868);

void loRaSend(float value) {
  uint8_t payload[4];
  memcpy(payload, &value, sizeof(float));

  Serial.print("[LoRa] Sending value: ");
  Serial.println(value, 4);

  int state = node.sendReceive(payload, sizeof(payload));
  if (state == RADIOLIB_ERR_NONE) Serial.println("[LoRa] Sent OK");
  else if (state == RADIOLIB_ERR_RX_TIMEOUT) Serial.println("[LoRa] Sent OK (no downlink)");
  else Serial.printf("[LoRa] Send failed: %d", state);
}