#include "MQTT.h"

MqttClient::MqttClient(const char* ssid, const char* password, const char* brokerIp, uint16_t port)
    : wifiSsid(ssid), wifiPassword(password), mqttServerIp(brokerIp), mqttPort(port), pubSubClient(wifiClient) {
    pubSubClient.setServer(mqttServerIp, mqttPort);
}

void MqttClient::ConnectWifi() {
    if (WiFi.status() == WL_CONNECTED) return;
    WiFi.begin(wifiSsid, wifiPassword);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void MqttClient::ConnectMqtt() {
    while (!pubSubClient.connected()) {
        if (WiFi.status() != WL_CONNECTED) ConnectWifi();
        
        String clientId = "HeltecV3Client-" + String(random(0xffff), HEX);
        if (pubSubClient.connect(clientId.c_str())) {
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

bool MqttClient::PublishAggregateValue(const char* topic, float aggregateValue) {
    if (!pubSubClient.connected()) ConnectMqtt();
    
    char payloadBuffer[32];
    snprintf(payloadBuffer, sizeof(payloadBuffer), "{\"avg\": %.4f}", aggregateValue);
    return pubSubClient.publish(topic, payloadBuffer);
}

void MqttClient::Loop() {
    if (!pubSubClient.connected()) ConnectMqtt();
    pubSubClient.loop();
}