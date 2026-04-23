#ifndef COMMUNICATION_MQTT_H
#define COMMUNICATION_MQTT_H

#include <WiFi.h>
#include <PubSubClient.h>

void MQTTSetup();
void MQTTloop();
void MQTTPublish(const char* topic, const char* msg);
const char* MQTTGetSendTopic();
const char* MQTTGetEchoTopic();

#endif