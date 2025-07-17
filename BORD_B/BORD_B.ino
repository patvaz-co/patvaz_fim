#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <PCF8574.h>
#include <EEPROM.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define RS485_TX_PIN 17
#define RS485_EN_PIN 4
HardwareSerial RS485(1);

const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "89.32.249.46";
const char* mqtt_user = "P@vaz";
const char* mqtt_password = "2025";

WiFiClient espClient;
PubSubClient client(espClient);

PCF8574 pcf[5] = {
  PCF8574(0x27),
  PCF8574(0x26),
  PCF8574(0x25),
  PCF8574(0x23),
  PCF8574(0x38)
};

#define MOTOR_L 0
#define MOTOR_R 1
#define RELAY1  2
#define RELAY2  3
#define SW_A    4
#define SW_B    5
#define WM_PIN  6
#define DI_PIN  7

int currentDir[5] = {-1, -1, -1, -1, -1};
bool motorRunning[5] = {false};
unsigned long motorStart[5] = {0};

unsigned long wmCount[5] = {0};
bool lastWMState[5] = {HIGH};
unsigned long lastSendWM[5] = {0};
int lastDIState[5] = {-1};
unsigned long lastSendDI[5] = {0};
unsigned long lastSendEV[5] = {0};
String lastEvSent[5] = {"", "", "", "", ""};

byte relayState[10] = {0};

// ===== RS485 send function =====
void sendRS485(String message) {
  Serial.println("RS485 → " + message); // ← نمایش در سریال
  digitalWrite(RS485_EN_PIN, HIGH);
  delayMicroseconds(100);
  RS485.println(message);
  RS485.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_EN_PIN, LOW);
}

// ===== EEPROM save/load =====
void saveRelayToEEPROM(int relayIndex, byte state) {
  int addr = 25 + relayIndex;
  EEPROM.write(addr, state);
  EEPROM.commit();
}

void saveWMToEEPROM() {
  for (int i = 0; i < 5; i++) {
    int base = 5 + i * 4;
    EEPROM.put(base, wmCount[i]);
  }
  EEPROM.commit();
}

void loadFromEEPROM() {
  for (int i = 0; i < 10; i++) relayState[i] = EEPROM.read(25 + i);
  for (int i = 0; i < 5; i++) EEPROM.get(5 + i * 4, wmCount[i]);
}

void applyRelayStates() {
  for (int i = 0; i < 10; i++) {
    int ic = i / 2;
    int pin = (i % 2 == 0) ? RELAY1 : RELAY2;
    pcf[ic].write(pin, relayState[i] == 1 ? LOW : HIGH);
  }
}

// ===== MQTT callback =====
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "patvaz/fim/1/ev") {
    int id = (msg.toInt() / 10) - 1;
    int dir = msg.toInt() % 10;
    if (id >= 0 && id < 5 && (dir == 0 || dir == 1)) {
      pcf[id].write(MOTOR_L, dir == 0 ? LOW : HIGH);
      pcf[id].write(MOTOR_R, dir == 1 ? LOW : HIGH);
      motorStart[id] = millis();
      motorRunning[id] = true;
      currentDir[id] = dir;
      sendRS485("ev=" + msg);
    }
  }

  if (String(topic) == "patvaz/fim/1/r2") {
    int cmd = msg.toInt();
    int relayIndex = (cmd / 10) - 1;
    int state = cmd % 10;
    if (relayIndex >= 0 && relayIndex < 10) {
      int ic = relayIndex / 2;
      int pin = (relayIndex % 2 == 0) ? RELAY1 : RELAY2;
      pcf[ic].write(pin, state == 1 ? LOW : HIGH);
      relayState[relayIndex] = state;
      saveRelayToEEPROM(relayIndex, state);
      sendRS485("r2=" + msg);
    }
  }
}

// ===== WiFi Connect =====
void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected ✔️");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ===== MQTT Connect =====
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect("esp32_client", mqtt_user, mqtt_password)) {
      Serial.println(" connected ✔️");
      client.subscribe("patvaz/fim/1/ev");
      client.subscribe("patvaz/fim/1/r2");
    } else {
      Serial.print(" failed ❌, rc=");
      Serial.print(client.state());
      Serial.println(" → retrying in 2s");
      delay(2000);
    }
  }
}

// ===== setup =====
void setup() {
  Serial.begin(115200);
  EEPROM.begin(64);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, LOW);
  RS485.begin(115200, SERIAL_8N1, -1, RS485_TX_PIN); // فقط TX

  for (int i = 0; i < 5; i++) {
    pcf[i].begin();
    for (int j = 0; j < 8; j++) pcf[i].write(j, HIGH);
  }

  loadFromEEPROM();
  applyRelayStates();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  ArduinoOTA.begin();
}

// ===== loop =====
void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) reconnect();
  client.loop();

  for (int i = 0; i < 5; i++) {
    if (motorRunning[i] && millis() - motorStart[i] >= 10000) {
      pcf[i].write(MOTOR_L, HIGH);
      pcf[i].write(MOTOR_R, HIGH);
      motorRunning[i] = false;
      currentDir[i] = -1;
    }

    int swA = pcf[i].read(SW_A);
    int swB = pcf[i].read(SW_B);
    String evVal = "00";
    if (swA == LOW && swB == HIGH) evVal = String((i + 1) * 10);
    else if (swA == HIGH && swB == LOW) evVal = String((i + 1) * 10 + 1);

    if (evVal != lastEvSent[i] && millis() - lastSendEV[i] > 1000) {
      String topic = "patvaz/fim/1/evs" + String(i + 1);
      client.publish(topic.c_str(), evVal.c_str());
      sendRS485("evs" + String(i + 1) + "=" + evVal);
      lastEvSent[i] = evVal;
      lastSendEV[i] = millis();
    }

    bool wmNow = pcf[i].read(WM_PIN);
    if (lastWMState[i] == HIGH && wmNow == LOW) {
      wmCount[i]++;
      saveWMToEEPROM();
    }
    lastWMState[i] = wmNow;

    if (millis() - lastSendWM[i] > 10000) {
      char buff[12];
      sprintf(buff, "%lu", wmCount[i]);
      String topic = "patvaz/fim/1/wm" + String(i + 1);
      client.publish(topic.c_str(), buff);
      sendRS485("wm" + String(i + 1) + "=" + String(buff));
      lastSendWM[i] = millis();
    }

    int di = pcf[i].read(DI_PIN);
    if (di != lastDIState[i] || millis() - lastSendDI[i] > 3000) {
      String topic = "patvaz/fim/1/di" + String(i + 1);
      String val = (di == LOW ? "10" : "11");
      client.publish(topic.c_str(), val.c_str());
      sendRS485("di" + String(i + 1) + "=" + val);
      lastDIState[i] = di;
      lastSendDI[i] = millis();
    }
  }

  delay(10);
}
