#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <PCF8574.h>

#define SDA_PIN 21
#define SCL_PIN 22

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";
const char* mqtt_user = "P@vaz";
const char* mqtt_password = "2025";

WiFiClient espClient;
PubSubClient client(espClient);
PCF8574 pcf1(0x27);

// پایه‌های IC
#define MOTOR_L 0
#define MOTOR_R 1
#define RELAY1  2
#define RELAY2  3
#define SW_A    4
#define SW_B    5
#define WM_PIN  6
#define DI_PIN  7

int currentDir = -1; // -1: خاموش
unsigned long motorStart = 0;
bool motorRunning = false;
// حذف محدودیت تک‌بار اجرا:
bool newCommandAllowed = true;

unsigned long wmCount = 0;
bool lastWMState = HIGH;
unsigned long lastSendWM = 0;
unsigned long lastSendDI = 0;
int lastDIState = -1;
unsigned long lastSendEV = 0;
String lastEvSent = "";

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "patvaz/fim/1/ev") {
    if (!motorRunning && msg == "10") {
      pcf1.write(MOTOR_L, LOW);
      pcf1.write(MOTOR_R, HIGH);
      motorStart = millis();
      motorRunning = true;
      currentDir = 0;
    } else if (!motorRunning && msg == "11") {
      pcf1.write(MOTOR_L, HIGH);
      pcf1.write(MOTOR_R, LOW);
      motorStart = millis();
      motorRunning = true;
      currentDir = 1;
    }
  }

  if (String(topic) == "patvaz/fim/1/r2") {
    if (msg == "10") pcf1.write(RELAY1, HIGH);
    else if (msg == "11") pcf1.write(RELAY1, LOW);
    else if (msg == "20") pcf1.write(RELAY2, HIGH);
    else if (msg == "21") pcf1.write(RELAY2, LOW);
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("esp32_client", mqtt_user, mqtt_password)) {
      client.subscribe("patvaz/fim/1/ev");
      client.subscribe("patvaz/fim/1/r2");
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf1.begin();
  for (int i = 0; i < 8; i++) pcf1.write(i, HIGH); // همه خروجی‌ها خاموش

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) reconnect();
  client.loop();

  // توقف خودکار موتور بعد از 10 ثانیه
  if (motorRunning && millis() - motorStart >= 10000) {
    pcf1.write(MOTOR_L, HIGH);
    pcf1.write(MOTOR_R, HIGH);
    currentDir = -1;
    motorRunning = false;
  }

  // کلید سه‌حالته
  int swA = pcf1.read(SW_A);
  int swB = pcf1.read(SW_B);
  String evVal = "00";
  if (swA == LOW && swB == HIGH) evVal = "10";
  else if (swA == HIGH && swB == LOW) evVal = "11";

  if (evVal != lastEvSent && millis() - lastSendEV > 1000) {
    client.publish("patvaz/fim/1/evs", evVal.c_str());
    lastEvSent = evVal;
    lastSendEV = millis();
  }

  // شمارنده WM
  bool wmNow = pcf1.read(WM_PIN);
  static bool lastWM = HIGH;
  if (lastWM == HIGH && wmNow == LOW) wmCount++;
  lastWM = wmNow;

  if (millis() - lastSendWM > 10000) {
    char buff[12];
    sprintf(buff, "%lu", wmCount);
    client.publish("patvaz/fim/1/wm1", buff);
    lastSendWM = millis();
  }

  // ورودی DI
  int di = pcf1.read(DI_PIN);
  if (di != lastDIState || millis() - lastSendDI > 3000) {
    client.publish("patvaz/fim/1/di1", (di == LOW ? "10" : "11"));
    lastDIState = di;
    lastSendDI = millis();
  }

  delay(10);
}
