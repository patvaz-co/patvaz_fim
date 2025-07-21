
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PCF8574.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define SW_A 3
#define SW_B 4

PCF8574 pcf(0x27);

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";

WiFiClient espClient;
PubSubClient client(espClient);

String lastState = "";
unsigned long lastSend = 0;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnect() {
  while (!client.connected()) {
    client.connect("test_evs");
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

  int swA = pcf.read(SW_A);
  int swB = pcf.read(SW_B);
  String evVal = "00";
  if (swA == LOW && swB == HIGH) evVal = "10";
  else if (swA == HIGH && swB == LOW) evVal = "11";

  if (evVal != lastState && millis() - lastSend > 1000) {
    client.publish("patvaz/fim/1/evs1", evVal.c_str());
    Serial.println("evs1 = " + evVal);
    lastState = evVal;
    lastSend = millis();
  }

  delay(50);
}