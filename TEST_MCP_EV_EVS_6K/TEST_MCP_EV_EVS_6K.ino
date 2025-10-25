#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== WiFi ==================
const char* ssid = "patvaz";
const char* password = "qazvin402";

// ================== MQTT ==================
const char* mqtt_server   = "217.144.107.162";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "Patvaz";
const char* mqtt_pass     = "P@vaz2025";

WiFiClient espClient;
PubSubClient client(espClient);

// ================== MCP23017 ==================
#define MCP_ADDR_CTRL 0x20   // IC اول → کنترل شیرها
#define MCP_ADDR_STAT 0x21   // IC دوم → وضعیت شیرها

#define RESET_PIN 4

// ---------------- شیرها (کنترل) ----------------
struct Valve {
  int in1;
  int in2;
  unsigned long stopMillis;
  bool active;
  bool portA;
};

#define RUN_TIME 10000

Valve valves[6] = {
  {0, 1, 0, false, true},   // ev1
  {2, 3, 0, false, true},   // ev2
  {4, 5, 0, false, true},   // ev3
  {6, 7, 0, false, true},   // ev4
  {0, 1, 0, false, false},  // ev5
  {2, 3, 0, false, false}   // ev11
};

uint8_t portA = 0;
uint8_t portB = 0;

// ---------------- MCP (کنترل) ----------------
void mcpWriteRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MCP_ADDR_CTRL);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mcpInit() {
  mcpWriteRegister(0x00, 0x00); // IODIRA خروجی
  mcpWriteRegister(0x01, 0x00); // IODIRB خروجی
  mcpWriteRegister(0x14, 0x00); // OLATA
  mcpWriteRegister(0x15, 0x00); // OLATB
  portA = 0;
  portB = 0;
}

void mcpDigitalWrite(bool portAFlag, int pin, bool state) {
  if (portAFlag) {
    if (state) portA |= (1 << pin);
    else portA &= ~(1 << pin);
    mcpWriteRegister(0x14, portA);
  } else {
    if (state) portB |= (1 << pin);
    else portB &= ~(1 << pin);
    mcpWriteRegister(0x15, portB);
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

// ---------------- MQTT Callback ----------------
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

// ---------------- MQTT Reconnect ----------------
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

// ---------------- Task MQTT ----------------
void TaskMQTT(void *pvParameters) {
  for (;;) {
    if (!client.connected()) reconnect();
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ---------------- Task کنترل شیر ----------------
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

// ---------------- MCP (وضعیت) ----------------
uint16_t lastState[6] = {0};

void writeRegister(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MCP_ADDR_STAT);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg){
  Wire.beginTransmission(MCP_ADDR_STAT);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP_ADDR_STAT, (uint8_t)1);
  if(Wire.available()){
    return Wire.read();
  }
  return 0;
}

// ---------------- Task وضعیت شیر ----------------
void readValvesTask(void * pvParameters){
  for(;;){
    uint8_t gpioA = readRegister(0x12);
    uint8_t gpioB = readRegister(0x13);

    for(int i=0;i<6;i++){
      int pinOpen, pinClose;

      if(i < 5){
        pinOpen  = i*2;
        pinClose = i*2 + 1;
      } else {
        pinOpen  = 10;
        pinClose = 11;
      }

      int valOpen, valClose;
      valOpen  = (pinOpen  < 8) ? ((gpioA & (1<<pinOpen))  ? 1 : 0) : ((gpioB & (1<<(pinOpen-8)))  ? 1 : 0);
      valClose = (pinClose < 8) ? ((gpioA & (1<<pinClose)) ? 1 : 0) : ((gpioB & (1<<(pinClose-8))) ? 1 : 0);

      uint16_t msgToSend = 0;

      if(i < 5){
        int base = (i+1)*10;
        if(valOpen == 0) msgToSend = base + 1;
        else if(valClose == 0) msgToSend = base;
        else msgToSend = 0;
      } else {
        if(valOpen == 0) msgToSend = 111;
        else if(valClose == 0) msgToSend = 110;
        else msgToSend = 0;
      }

      if(msgToSend != lastState[i]){
        lastState[i] = msgToSend;

        String topic = (i<5) ? "patvaz/fim/1/evs" + String(i+1) : "patvaz/fim/1/evs11";
        char buf[5];
        sprintf(buf,"%d",msgToSend);
        client.publish(topic.c_str(), buf);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ---------------- Setup ----------------
void setup() {
  Wire.begin();
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(1);
  digitalWrite(RESET_PIN, HIGH);
  delay(5);

  // IC کنترل
  mcpInit();

  // IC وضعیت
  writeRegister(0x00, 0xFF); // IODIRA ورودی
  writeRegister(0x01, 0xFF); // IODIRB ورودی
  writeRegister(0x0C, 0xFF); // GPPUA pull-up
  writeRegister(0x0D, 0xFF); // GPPUB pull-up

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Tasks
  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskValves, "TaskValves", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(readValvesTask,"ReadValves",4096,NULL,1,NULL,1);
}

void loop() {
  // همه چیز توی Task ها
}
