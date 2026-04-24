#include "CommunicationMQTT.h"
#include "MQTTPasswords.h"

const char* mqtt_server = "192.168.142.183";
const char* topic_send = "dakshita/IoT/send";
const char* topic_echo = "dakshita/IoT/echo";

unsigned long last_sent_time = 0;
unsigned long rtts[100];
int rtt_count = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// Processes incoming messages and calculates Round-Trip Time (RTT)
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("CALLBACK TRIGGERED on topic: %s\n", topic);

  char messageArr[length + 1];
  memcpy(messageArr, payload, length);
  messageArr[length] = '\0';
  String message = String(messageArr);

  int commaIndex = message.indexOf(',');
  if (commaIndex == -1) return;

  unsigned long original_timestamp = message.substring(0, commaIndex).toInt();
  unsigned long pc_echo_time = message.substring(commaIndex + 1).toInt();
  unsigned long now = millis();

  unsigned long rtt = now - original_timestamp;
  unsigned long adjusted_rtt = rtt - pc_echo_time;

  Serial.print("RTT: "); Serial.print(rtt);
  Serial.print(" ms | PC echo: "); Serial.print(pc_echo_time);
  Serial.print(" ms | Adjusted RTT: "); Serial.print(adjusted_rtt);
  Serial.println(" ms");

  if (rtt_count < 100) {
    rtts[rtt_count++] = adjusted_rtt;
    Serial.printf(">rtt:%f", rtt);
  }

  if (rtt_count == 100) {
    unsigned long sum = 0;
    for (int i = 0; i < 100; i++) sum += rtts[i];
    float avg = (float)sum / 100.0;

    Serial.println();
    Serial.print("Average Adjusted RTT over 100 samples: ");
    Serial.print(avg);
    Serial.println(" ms");
    Serial.println("==========================");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print ("Attempting MQTT Connection");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Client Connected");
      client.subscribe(topic_echo);
    } else {
      Serial.printf("Failed, rc=%d\n", client.state());
      delay(1000);
    }
  }
}

const char* MQTTGetSendTopic () {
  return topic_send;
}

const char* MQTTGetEchoTopic () {
  return topic_echo;
}

void MQTTPublish(const char* topic, const char* msg) {
  uint64_t windowStart = esp_timer_get_time();
  if (client.connected()) client.publish(topic, msg);
  uint64_t windowEnd = esp_timer_get_time();
  uint64_t elapsed   = windowEnd - windowStart;
  Serial.printf("mqtt_publish_time_ms:%.2f\n", elapsed / 1000.0f);
  client.loop(); // Ensure we process incoming messages and maintain connection
}

void MQTTSetup() {
  uint64_t windowStart = esp_timer_get_time();
  Serial.println("Connecting to WiFi-------------------------------------");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);

    if (millis() - startAttempt > 50000) ESP.restart();
  }

  Serial.println("WiFi connected---------------------------------------------------");

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  uint64_t windowEnd = esp_timer_get_time();
  uint64_t elapsed   = windowEnd - windowStart;
  Serial.printf("mqtt_setup_window_time_ms:%.2f\n", elapsed / 1000.0f);
  reconnect();
}

void MQTTloop() {
  Serial.print ("Attempting MQTT Connection");
  client.loop();
  if (!client.connected()) reconnect();
  Serial.print ("MQTT Connected");

  static unsigned long last_send = 0;
  if (rtt_count < 100 && millis() - last_send > 1000) {
    last_send = millis();
    last_sent_time = millis();
    char msg[32];
    sprintf(msg, "%lu", last_sent_time);
    client.publish(topic_send, msg);
    Serial.printf("Sent:%f", last_sent_time);
  }
}