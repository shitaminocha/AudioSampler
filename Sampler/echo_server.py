import paho.mqtt.client as mqtt
import time

BROKER    = "localhost"
TOPIC_IN  = "dakshita/IoT/send"
TOPIC_OUT = "dakshita/IoT/echo"

def on_message(client, userdata, msg):
    receive_time = time.time()
    original_timestamp = msg.payload.decode()
    process_ms = int((time.time() - receive_time) * 1000)
    echo = f"{original_timestamp},{process_ms}"
    client.publish(TOPIC_OUT, echo)
    print(f"Echoed: {echo}")

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883)
client.subscribe(TOPIC_IN)
print("RTT echo server listening...")
client.loop_forever()