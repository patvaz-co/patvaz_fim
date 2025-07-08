#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PCF8574.h>
#include <ArduinoOTA.h>

#define SDA_PIN 21
#define SCL_PIN 22

// RS485 Config
#define RS485_TX_PIN 17
#define RS485_EN_PIN 4

// WiFi & MQTT
const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "37.148.56.88";
const char* hostname = "esp32_motor_ctrl";

WiFiClient espClient;
PubSubClient client(espClient);

// PCF8574 addresses
PCF8574 pcfMotor1(0x27);
PCF8574 pcfMotor2(0x26);
PCF8574 pcfSwitch1(0x25);
PCF8574 pcfSwitch2(0x23);
PCF8574 pcfRelay(0x38);

unsigned long motorTimers[6] = {0};
bool motorActive[6] = {false};
String lastSwitchState[6] = {"", "", "", "", "", ""};
int wmCounter[6] = {0};
bool lastWmState[6] = {false};
unsigned long lastWmPublish = 0;
bool lastDIState[5] = {false};

struct MotorControl {
  PCF8574* pcf;
  uint8_t pin1;
  uint8_t pin2;
};

MotorControl motors[6] = {
  {&pcfMotor1, 0, 1}, {&pcfMotor1, 2, 3},
  {&pcfMotor1, 4, 5}, {&pcfMotor1, 6, 7},
  {&pcfMotor2, 0, 1}, {&pcfMotor2, 2, 3},
};

void sendToRS485(const String& data) {
  digitalWrite(RS485_EN_PIN, HIGH);
  delayMicroseconds(100);
  Serial2.println(data);
  Serial2.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_EN_PIN, LOW);
}

void controlMotorTimed(int index, const String& cmd) {
  if (index < 0 || index >= 6) return;
  MotorControl m = motors[index];
  if (cmd.endsWith("1")) {
    m.pcf->write(m.pin1, HIGH);
    m.pcf->write(m.pin2, LOW);
  } else {
    m.pcf->write(m.pin1, LOW);
    m.pcf->write(m.pin2, HIGH);
  }
  motorTimers[index] = millis();
  motorActive[index] = true;
}

void stopMotor(int index) {
  MotorControl m = motors[index];
  m.pcf->write(m.pin1, LOW);
  m.pcf->write(m.pin2, LOW);
  motorActive[index] = false;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  if (String(topic) == "patvaz/fim/1/ev") {
    int motorId = msg.toInt() / 10;
    controlMotorTimed(motorId - 1, msg);
    Serial.printf("[MQTT] Motor %d -> %s\n", motorId, msg.c_str());
    sendToRS485("ev=" + msg);
  } else if (String(topic) == "patvaz/fim/1/r2") {
    int relayCmd = msg.toInt();
    int relayId = relayCmd / 10;
    bool state = relayCmd % 10 == 1;
    if (relayId == 1) pcfSwitch2.write(6, state);
    else if (relayId == 2) pcfSwitch2.write(7, state);
    else if (relayId >= 3 && relayId <= 5) pcfRelay.write(relayId - 3, state);
    Serial.printf("[MQTT] Relay %d -> %s\n", relayId, state ? "ON" : "OFF");
    sendToRS485("r2=" + String(relayCmd));
  }
}

void publishSwitchState(int index, const String& value) {
  String topic = "patvaz/fim/1/evs" + String(index + 1);
  client.publish(topic.c_str(), value.c_str());
  Serial.printf("Published %s to %s\n", value.c_str(), topic.c_str());
  sendToRS485("evs" + String(index + 1) + "=" + value);
}

String readSwitchState(int index) {
  PCF8574* chip = (index < 4) ? &pcfSwitch1 : &pcfSwitch2;
  int offset = (index < 4) ? index * 2 : (index - 4) * 2;
  bool left = chip->read(offset);
  bool right = chip->read(offset + 1);
  if (!left) return String((index + 1) * 10 + 1);
  if (!right) return String((index + 1) * 10 + 0);
  return "00";
}

void readWmCounters() {
  bool states[6] = {
    pcfMotor2.read(4) == LOW,
    pcfMotor2.read(5) == LOW,
    pcfSwitch2.read(4) == LOW,
    pcfSwitch2.read(5) == LOW,
    pcfRelay.read(0) == LOW,
    pcfRelay.read(1) == LOW
  };

  for (int i = 0; i < 6; i++) {
    if (states[i] && !lastWmState[i]) wmCounter[i]++;
    lastWmState[i] = states[i];
  }

  if (millis() - lastWmPublish >= 20000) {
    for (int i = 0; i < 6; i++) {
      String topic = "patvaz/fim/1/wm" + String(i + 1);
      client.publish(topic.c_str(), String(wmCounter[i]).c_str());
      sendToRS485("wm" + String(i + 1) + "=" + String(wmCounter[i]));
    }
    lastWmPublish = millis();
  }
}

void readDigitalInputs() {
  for (int i = 0; i < 5; i++) {
    bool state = pcfRelay.read(i + 3) == LOW;
    if (state != lastDIState[i]) {
      lastDIState[i] = state;
      int id = i + 21;
      int value = state ? id * 10 + 1 : id * 10;
      String topic = "patvaz/fim/1/di" + String(id);
      client.publish(topic.c_str(), String(value).c_str());
      sendToRS485("di" + String(id) + "=" + String(value));
    }
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("WiFi connected: " + WiFi.localIP().toString());
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      client.subscribe("patvaz/fim/1/ev");
      client.subscribe("patvaz/fim/1/r2");
      Serial.println("MQTT connected.");
    } else {
      delay(5000);
    }
  }
}

void setup_ota() {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, -1, RS485_TX_PIN);
  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  pcfMotor1.begin();
  pcfMotor2.begin();
  pcfSwitch1.begin();
  pcfSwitch2.begin();
  pcfRelay.begin();

  for (int i = 0; i < 6; i++) {
    motors[i].pcf->write(motors[i].pin1, LOW);
    motors[i].pcf->write(motors[i].pin2, LOW);
  }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  setup_ota();
}

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) reconnect();
  client.loop();

  for (int i = 0; i < 6; i++) {
    if (motorActive[i] && millis() - motorTimers[i] >= 10000) stopMotor(i);
  }

  for (int i = 0; i < 6; i++) {
    String state = readSwitchState(i);
    if (state != lastSwitchState[i]) {
      lastSwitchState[i] = state;
      publishSwitchState(i, state);
    }
  }

  readWmCounters();
  readDigitalInputs();
  delay(100);
}
