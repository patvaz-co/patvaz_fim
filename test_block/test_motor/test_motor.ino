
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PCF8574.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MOTOR_L 0
#define MOTOR_R 1

PCF8574 pcf(0x27);

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";

WiFiClient espClient;
PubSubClient client(espClient);

bool motorRunning = false;
unsigned long motorStart = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "patvaz/fim/1/ev") {
    int id = (msg.toInt() / 10) - 1;
    int dir = msg.toInt() % 10;
    if (id == 0 && (dir == 0 || dir == 1)) {
      pcf.write(MOTOR_L, dir == 0 ? LOW : HIGH);
      pcf.write(MOTOR_R, dir == 1 ? LOW : HIGH);
      motorRunning = true;
      motorStart = millis();
      Serial.println("Motor ON direction: " + String(dir));
    }
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnect() {
  while (!client.connected()) {
    client.connect("test_motor");
    client.subscribe("patvaz/fim/1/ev");
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

  if (motorRunning && millis() - motorStart > 10000) {
    pcf.write(MOTOR_L, HIGH);
    pcf.write(MOTOR_R, HIGH);
    motorRunning = false;
    Serial.println("Motor OFF");
  }
  delay(10);
}
