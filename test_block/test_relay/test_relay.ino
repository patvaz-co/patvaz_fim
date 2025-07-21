
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PCF8574.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define RELAY_PIN 2

PCF8574 pcf(0x27);

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "patvaz/fim/1/r2") {
    int cmd = msg.toInt();
    int relayIndex = (cmd / 10) - 1;
    int state = cmd % 10;
    if (relayIndex == 0) {
      pcf.write(RELAY_PIN, state == 1 ? LOW : HIGH);
      Serial.println("Relay state: " + String(state));
    }
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnect() {
  while (!client.connected()) {
    client.connect("test_relay");
    client.subscribe("patvaz/fim/1/r2");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
  delay(10);
}
