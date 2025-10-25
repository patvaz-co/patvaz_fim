/* Single MCP23017 controlling:
   - Port A pins 0..7 -> ra1..ra8
   - Port B pins 0..5 -> rb1..rb6
   Uses Wire + PubSubClient + FreeRTOS task for MQTT.
*/

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------- WiFi / MQTT ----------
const char* ssid = "patvaz";
const char* password = "qazvin402";

// ---------------- MQTT ----------------
const char* mqtt_server   = "217.144.107.162";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "Patvaz";
const char* mqtt_pass     = "P@vaz2025";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- MCP23017 ----------
#define MCP_ADDR 0x24

#define RESET_PIN 4   // پایه ریست MCP23017

#define IODIRA 0x00
#define IODIRB 0x01
#define OLATA  0x14
#define OLATB  0x15

uint8_t olatA = 0x00;
uint8_t olatB = 0x00;

void mcpWriteRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mcpInitOutputs() {
  // Set all A and B pins to outputs (0 = output)
  mcpWriteRegister(IODIRA, 0x00);
  mcpWriteRegister(IODIRB, 0x00);
  // Clear outputs
  olatA = 0x00;
  olatB = 0x00;
  mcpWriteRegister(OLATA, olatA);
  mcpWriteRegister(OLATB, olatB);
}

void setOlatBit(bool portA, uint8_t pin, bool state) {
  if (portA) {
    if (state) olatA |= (1 << pin);
    else       olatA &= ~(1 << pin);
    mcpWriteRegister(OLATA, olatA);
  } else {
    if (state) olatB |= (1 << pin);
    else       olatB &= ~(1 << pin);
    mcpWriteRegister(OLATB, olatB);
  }
}

// ---------- Relays mapping ----------
struct RelayDef {
  bool portA;        // true => Port A, false => Port B
  uint8_t pin;       // pin number within the port (0..7)
  const char* topic; // topic string
  int offCode;       // e.g. 10,20,...
  int onCode;        // e.g. 11,21,...
};

// ra1..ra8 => PortA 0..7
// rb1..rb6 => PortB 0..5
RelayDef relays[] = {
  { true, 0, "patvaz/fim/1/ra1",  10, 11},
  { true, 1, "patvaz/fim/1/ra2",  20, 21},
  { true, 2, "patvaz/fim/1/ra3",  30, 31},
  { true, 3, "patvaz/fim/1/ra4",  40, 41},
  { true, 4, "patvaz/fim/1/ra5",  50, 51},
  { true, 5, "patvaz/fim/1/ra6",  60, 61},
  { true, 6, "patvaz/fim/1/ra7",  70, 71},
  { true, 7, "patvaz/fim/1/ra8",  80, 81},

  { false, 0, "patvaz/fim/1/rb1", 10, 11},
  { false, 1, "patvaz/fim/1/rb2", 20, 21},
  { false, 2, "patvaz/fim/1/rb3", 30, 31},
  { false, 3, "patvaz/fim/1/rb4", 40, 41},
  { false, 4, "patvaz/fim/1/rb5", 50, 51},
  { false, 5, "patvaz/fim/1/rb6", 60, 61}
};

const int RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

// ---------- MQTT handling ----------
void callback(char* topic, byte* payload, unsigned int length) {
  // build msg string (robust against non-null-terminated payload)
  String msg;
  for (unsigned int i = 0; i < length; ++i) msg += (char)payload[i];
  int cmd = msg.toInt();

  // find which relay
  for (int i = 0; i < RELAY_COUNT; ++i) {
    if (String(topic) == String(relays[i].topic)) {
      if (cmd == relays[i].offCode) {
        setOlatBit(relays[i].portA, relays[i].pin, false);
      } else if (cmd == relays[i].onCode) {
        setOlatBit(relays[i].portA, relays[i].pin, true);
      }
      break;
    }
  }
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void subscribeAllTopics() {
  for (int i = 0; i < RELAY_COUNT; ++i) {
    client.subscribe(relays[i].topic);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_RelayOneMCP", mqtt_user, mqtt_pass)) {
      subscribeAllTopics();
    } else {
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  }
}

// ---------- FreeRTOS Task for MQTT ----------
void TaskMQTT(void *pvParameters) {
  for (;;) {
    if (!client.connected()) reconnect();
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ---------- setup / loop ----------
void setup() {
  Wire.begin();         // I2C default SDA/SCL pins

pinMode(RESET_PIN, OUTPUT);
digitalWrite(RESET_PIN, LOW);
delay(1);
digitalWrite(RESET_PIN, HIGH);
delay(5);


  mcpInitOutputs();

  client.setClient(espClient);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  setup_wifi();

  // create MQTT task pinned to core 0
  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // همه کارها داخل Task انجام می‌شود
}
