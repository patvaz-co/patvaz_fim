
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PCF8574.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define WM_PIN 5

PCF8574 pcf(0x27);

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long wmCount = 0;
bool lastWMState = HIGH;
unsigned long lastSend = 0;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnect() {
  while (!client.connected()) {
    client.connect("test_wm");
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

  bool state = pcf.read(WM_PIN);
  if (lastWMState == HIGH && state == LOW) wmCount++;
  lastWMState = state;

  if (millis() - lastSend > 10000) {
    char msg[12];
    sprintf(msg, "%lu", wmCount);
    client.publish("patvaz/fim/1/wm1", msg);
    lastSend = millis();
  }
  delay(10);
}
