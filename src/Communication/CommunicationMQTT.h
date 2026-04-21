#include <WiFi.h>
#include <PubSubClient.h>

void MQTTSetup();
void MQTTloop ();
void MQTTPublish(const char* topic, const char* msg);
const char* MQTTGetSendTopic ();
const char* MQTTGetEchoTopic ();
