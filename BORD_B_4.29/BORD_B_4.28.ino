// نسخه جدید با 6 عدد IC شامل MP11 و MP12 و WM6 و EV6 و رله ششم
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

PCF8574 pcf[6] = {
  PCF8574(0x27), PCF8574(0x26), PCF8574(0x25),
  PCF8574(0x23), PCF8574(0x38), PCF8574(0x24) // ← IC جدید
};

#define MOTOR_L 0
#define MOTOR_R 1
#define RELAY   2
#define SW_A    3
#define SW_B    4
#define WM_PIN  5
#define MP1_PIN 6
#define MP2_PIN 7




#define P1 32
#define P2 33
#define P3 25
#define P4 26
#define P5 27
#define P6 15 



int pins[] = {P1, P2, P3, P4, P5, P6};
const int numPins = sizeof(pins) / sizeof(pins[0]);


int currentDir[6] = {-1, -1, -1, -1, -1, -1};
bool motorRunning[6] = {false};
unsigned long motorStart[6] = {0};

unsigned long wmCount[6] = {0};
bool lastWMState[6] = {HIGH};
unsigned long lastSendWM[6] = {0};

byte relayState[6] = {0};
String lastEvSent[6] = {"", "", "", "", "", ""};
unsigned long lastSendEV[6] = {0};

int lastMP[12] = {-1};




void setup() {
  Serial.begin(115200);
  EEPROM.begin(128);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, LOW);
  RS485.begin(115200, SERIAL_8N1, -1, RS485_TX_PIN);

pinMode(P1, OUTPUT);
pinMode(P2, OUTPUT);
pinMode(P3, OUTPUT);
pinMode(P4, OUTPUT);
pinMode(P5, OUTPUT);
pinMode(P6, OUTPUT);


  for (int i = 0; i < 6; i++) {
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

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) reconnect();
  client.loop();

  for (int i = 0; i < 6; i++) {
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

    for (int j = 0; j < 2; j++) {
      int mpIndex = i * 2 + j;
      int pin = (j == 0) ? MP1_PIN : MP2_PIN;
      int state = pcf[i].read(pin);
      int value = (mpIndex + 1) * 10 + (state == LOW ? 1 : 0);
      if (lastMP[mpIndex] != value) {
        String topic = "patvaz/fim/1/li" + String(mpIndex + 1);
        client.publish(topic.c_str(), String(value).c_str());
        sendRS485("mp" + String(mpIndex + 1) + "=" + String(value));
        lastMP[mpIndex] = value;
      }
    }
  }
    // for (int i = 0; i < numPins; i++) {
    digitalWrite(P1, HIGH);  // روشن
    digitalWrite(P2, HIGH);
    digitalWrite(P3, HIGH);
    digitalWrite(P4, HIGH);
    digitalWrite(P5, HIGH);
    digitalWrite(P6, HIGH);
    delay(500);
    digitalWrite(P1, LOW);
    digitalWrite(P2, LOW);
    digitalWrite(P3, LOW);
    digitalWrite(P4, LOW);
    digitalWrite(P5, LOW); 
    digitalWrite(P6, LOW);  // خاموش
    delay(250);
  // }
  delay(10);
}
