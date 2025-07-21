
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PCF8574.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MP1_PIN 6
#define MP2_PIN 7

PCF8574 pcf(0x27);

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";

WiFiClient espClient;
PubSubClient client(espClient);

int lastMP[2] = {-1};

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnect() {
  while (!client.connected()) {
    client.connect("test_mp");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin();
  for (int i = 0; i < 8; i++) pcf.write(i, HIGH);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  for (int j = 0; j < 2; j++) {
    int pin = (j == 0) ? MP1_PIN : MP2_PIN;
    int mpIndex = j + 1;
    int state = pcf.read(pin);
    int value = mpIndex * 10 + (state == LOW ? 1 : 0);
    if (lastMP[j] != value) {
      String topic = "patvaz/fim/1/il" + String(mpIndex);
      client.publish(topic.c_str(), String(value).c_str());
      Serial.println(topic + " = " + String(value));
      lastMP[j] = value;
    }
  }

  delay(100);
}
