#ifndef MQTT_H
#define MQTT_H

#include <WiFi.h>
#include <PubSubClient.h>

class MqttClient {
private:
    const char* wifiSsid;
    const char* wifiPassword;
    const char* mqttServerIp;
    uint16_t mqttPort;
    
    WiFiClient wifiClient;
    PubSubClient pubSubClient;

public:
    MqttClient(const char* ssid, const char* password, const char* brokerIp, uint16_t port = 1883);
    
    void ConnectWifi();
    void ConnectMqtt();
    bool PublishAggregateValue(const char* topic, float aggregateValue);
    void Loop();
};

#endif