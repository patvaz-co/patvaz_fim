#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------------- WiFi ----------------
const char* ssid = "patvaz";
const char* password = "qazvin402";

// ---------------- MQTT ----------------
const char* mqtt_server   = "217.144.107.162";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "Patvaz";
const char* mqtt_pass     = "P@vaz2025";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- MCP23017 ----------------
#define MCP_ADDR 0x20

// رجیسترها
#define IODIRA 0x00
#define IODIRB 0x01
#define OLATA  0x14
#define OLATB  0x15



#define RESET_PIN 4   // پایه ریست MCP23017

// ---------------- شیرها ----------------
struct Valve {
  int in1;
  int in2;
  unsigned long stopMillis;
  bool active;
  bool portA; // true = Port A, false = Port B
};

#define RUN_TIME 10000

Valve valves[6] = {
  {0, 1, 0, false, true},   // ev1 (A0, A1)
  {2, 3, 0, false, true},   // ev2 (A2, A3)
  {4, 5, 0, false, true},   // ev3 (A4, A5)
  {6, 7, 0, false, true},   // ev4 (A6, A7)
  {0, 1, 0, false, false},  // ev5 (B0, B1)
  {2, 3, 0, false, false}   // ev11 (B2, B3)
};

// ---------------- توابع MCP ----------------
uint8_t portA = 0;
uint8_t portB = 0;





void mcpWriteRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mcpInit() {
  // همه پین‌ها خروجی
  mcpWriteRegister(IODIRA, 0x00);
  mcpWriteRegister(IODIRB, 0x00);
  // مقدار اولیه 0
  mcpWriteRegister(OLATA, 0x00);
  mcpWriteRegister(OLATB, 0x00);
  portA = 0;
  portB = 0;
}

void mcpDigitalWrite(bool portAFlag, int pin, bool state) {
  if (portAFlag) {
    if (state) portA |= (1 << pin);
    else portA &= ~(1 << pin);
    mcpWriteRegister(OLATA, portA);
  } else {
    if (state) portB |= (1 << pin);
    else portB &= ~(1 << pin);
    mcpWriteRegister(OLATB, portB);
  }
}

// ---------------- کنترل شیر ----------------
void controlValve(Valve &v, bool dir) {
  if (!dir) { // بسته
    mcpDigitalWrite(v.portA, v.in1, LOW);
    mcpDigitalWrite(v.portA, v.in2, HIGH);
  } else {    // باز
    mcpDigitalWrite(v.portA, v.in1, HIGH);
    mcpDigitalWrite(v.portA, v.in2, LOW);
  }
  v.stopMillis = millis() + RUN_TIME;
  v.active = true;
}

// ---------------- MQTT ----------------
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  String t = String(topic);

  for (int i = 0; i < 5; i++) {
    String tname = "patvaz/fim/1/ev" + String(i + 1);
    if (t == tname) {
      if (msg == String((i + 1) * 10)) controlValve(valves[i], false);
      else if (msg == String((i + 1) * 10 + 1)) controlValve(valves[i], true);
    }
  }

  if (t == "patvaz/fim/1/ev11") {
    if (msg == "110") controlValve(valves[5], false);
    else if (msg == "111") controlValve(valves[5], true);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_Client", mqtt_user, mqtt_pass)) {
      for (int i = 1; i <= 5; i++) {
        String tname = "patvaz/fim/1/ev" + String(i);
        client.subscribe(tname.c_str());
      }
      client.subscribe("patvaz/fim/1/ev11");
    } else {
      delay(2000);
    }
  }
}

// ---------------- FreeRTOS Tasks ----------------
void TaskMQTT(void *pvParameters) {
  for (;;) {
    if (!client.connected()) reconnect();
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TaskValves(void *pvParameters) {
  for (;;) {
    unsigned long now = millis();
    for (int i = 0; i < 6; i++) {
      if (valves[i].active && now > valves[i].stopMillis) {
        mcpDigitalWrite(valves[i].portA, valves[i].in1, LOW);
        mcpDigitalWrite(valves[i].portA, valves[i].in2, LOW);
        valves[i].active = false;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ---------------- setup ----------------
void setup() {
  Wire.begin();

pinMode(RESET_PIN, OUTPUT);
digitalWrite(RESET_PIN, LOW);
delay(1);
digitalWrite(RESET_PIN, HIGH);
delay(5);


  mcpInit();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskValves, "TaskValves", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // همه‌چی توی Task ها
}
