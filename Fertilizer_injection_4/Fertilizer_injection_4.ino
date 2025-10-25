#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <EEPROM.h>

// ============== EEPROM Configuration ==============
#define EEPROM_SIZE 512
#define EEPROM_RELAY_START 0
#define EEPROM_COUNTER_START 20
#define EEPROM_MAGIC_ADDR 50
#define EEPROM_MAGIC_VALUE 0xAB

// ---------------- WiFi / MQTT ----------------
const char* ssid = "IOT";
const char* password = "patvazIOT";
const char* mqtt_server   = "217.144.107.162";
const int   mqtt_port     = 1883;
const char* mqtt_user     = "Patvaz";
const char* mqtt_pass     = "P@vaz2025";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- MCP23017 addresses & registers ----------------
#define MCP_CTRL_ADDR  0x20
#define MCP_STAT_ADDR  0x21
#define MCP_RELAY_ADDR 0x24
#define MCP1_ADDR 0x22
#define MCP2_ADDR 0x23

#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA  0x12
#define GPIOB  0x13
#define OLATA  0x14
#define OLATB  0x15
#define IODIRA1 0x00
#define IODIRB1 0x01
#define GPIOA1 0x12
#define GPIOB1 0x13
#define GPPUA1 0x0C
#define GPPUB1 0x0D
#define GPPUA  0x0C
#define GPPUB  0x0D

#define LED_PING_NET 33
#define LED_PUBLISH 32
#define POWER_MODEM 27
#define buzer_PIN 26
#define RESET_PIN 18

uint16_t counters[6] = {0};
uint8_t prevCounterState[6];
uint8_t digitalState[8];
uint8_t prevDigitalState[8];

unsigned long lastHiMillis = 0;
#define HEARTBEAT_TOPIC_SEND "patvaz/fim/1/hw"
#define HEARTBEAT_TOPIC_RECV "patvaz/fim/1/sw"
#define HEARTBEAT_INTERVAL 20000
#define HEARTBEAT_TIMEOUT 200000

struct Sensor {
  uint8_t port;
  uint8_t pin;
  const char* topic;
  uint8_t onValue;
  uint8_t offValue;
  uint8_t lastState;
};

Adafruit_ADS1115 ads;

const float Vcc = 3.28;
float slope = 1666.67;
float offset = -700;
const char* topicEC  = "patvaz/fim/1/sec";
const char* topicTDS = "patvaz/fim/1/stds";
const char* topicSP  = "patvaz/fim/1/sp";
const float R_SENSE = 250.0;
const char* topicNTC = "patvaz/fim/1/stph";
const float R_FIXED_NTC = 10000.0;
const float B_NTC = 3950.0;
const float T0_NTC = 298.15;
const float R0_NTC = 10000.0;

unsigned long lastHeartbeatMillis = 0;

struct Valve {
  int in1;
  int in2;
  unsigned long stopMillis;
  bool active;
  bool portA;
};

Sensor sensors[12] = {
  {1,7,"patvaz/fim/1/li10",11,10,255},
  {1,6,"patvaz/fim/1/li11",11,10,255},
  {1,5,"patvaz/fim/1/li20",21,20,255},
  {1,4,"patvaz/fim/1/li21",21,20,255},
  {1,3,"patvaz/fim/1/li30",31,30,255},
  {1,2,"patvaz/fim/1/li31",31,30,255},
  {1,1,"patvaz/fim/1/li40",41,40,255},
  {1,0,"patvaz/fim/1/li41",41,40,255},
  {0,0,"patvaz/fim/1/li50",51,50,255},
  {0,1,"patvaz/fim/1/li51",51,50,255},
  {0,2,"patvaz/fim/1/li60",61,60,255},
  {0,3,"patvaz/fim/1/li61",61,60,255}
};

TaskHandle_t CounterTaskHandle;
TaskHandle_t DigitalTaskHandle;

uint8_t ctrl_portA = 0;
uint8_t ctrl_portB = 0;

#define RUN_TIME 10000

Valve valves[6] = {
  {0, 1, 0, false, false},
  {2, 3, 0, false, false},
  {4, 5, 0, false, false},
  {6, 7, 0, false, false},
  {0, 1, 0, false, true},
  {2, 3, 0, false, true}
};

uint16_t lastState[6] = {0};

uint8_t olatA_relay = 0x00;
uint8_t olatB_relay = 0x00;

struct RelayDef {
  bool portA;
  uint8_t pin;
  const char* topic;
  int offCode;
  int onCode;
};

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
int lastRA8Value = 80;
volatile bool hw_ok = false;

const uint8_t LM75_ADDR = 0x48;
bool sent50 = false;
bool sent51 = false;
const long tempInterval = 10000;
float lastSentTempMsg = NAN;
int lastRA6Value = 60;
int lastRA7Value = 70;

// ============== Forward declarations ==============
void mcpWrite(uint8_t addr, uint8_t reg, uint8_t val);
uint8_t mcpRead(uint8_t addr, uint8_t reg);
void setRelay(bool portA, uint8_t pin, bool state);

// ============== EEPROM Functions ==============

void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VALUE) {
    Serial.println("First boot - Initializing EEPROM...");
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    EEPROM.commit();
  } else {
    Serial.println("EEPROM already initialized");
  }
}

void saveRelayState(uint8_t relayIndex, uint8_t state) {
  if (relayIndex >= 14) return;
  EEPROM.write(EEPROM_RELAY_START + relayIndex, state);
  EEPROM.commit();
  Serial.printf("Saved relay %d state: %d\n", relayIndex, state);
}

uint8_t loadRelayState(uint8_t relayIndex) {
  if (relayIndex >= 14) return 0;
  return EEPROM.read(EEPROM_RELAY_START + relayIndex);
}

void saveCounterValue(uint8_t counterIndex, uint16_t value) {
  if (counterIndex >= 6) return;
  int addr = EEPROM_COUNTER_START + (counterIndex * 2);
  EEPROM.write(addr, value & 0xFF);
  EEPROM.write(addr + 1, (value >> 8) & 0xFF);
  EEPROM.commit();
  Serial.printf("Saved counter %d value: %d\n", counterIndex + 1, value);
}

uint16_t loadCounterValue(uint8_t counterIndex) {
  if (counterIndex >= 6) return 0;
  int addr = EEPROM_COUNTER_START + (counterIndex * 2);
  uint8_t low = EEPROM.read(addr);
  uint8_t high = EEPROM.read(addr + 1);
  return (high << 8) | low;
}

void clearEEPROM(String type) {
  if (type == "all") {
    Serial.println("Clearing ALL EEPROM...");
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  } 
  else if (type == "relay") {
    Serial.println("Clearing RELAY EEPROM...");
    for (int i = 0; i < 14; i++) {
      EEPROM.write(EEPROM_RELAY_START + i, 0);
    }
  } 
  else if (type == "wm") {
    Serial.println("Clearing COUNTER EEPROM...");
    for (int i = 0; i < 12; i++) {
      EEPROM.write(EEPROM_COUNTER_START + i, 0);
    }
  }
  EEPROM.commit();
  Serial.println("EEPROM cleared: " + type);
}

void restoreRelayStates() {
  Serial.println("Restoring relay states from EEPROM...");
  for (int i = 0; i < RELAY_COUNT; ++i) {
    uint8_t savedState = loadRelayState(i);
    if (savedState == 1) {
      setRelay(relays[i].portA, relays[i].pin, true);
      char msg[4];
      sprintf(msg, "%d", relays[i].onCode);
      client.publish(relays[i].topic, msg);
      Serial.printf("Restored relay %d to ON\n", i);
    } else {
      setRelay(relays[i].portA, relays[i].pin, false);
    }
  }
}

void restoreCounterValues() {
  Serial.println("Restoring counter values from EEPROM...");
  for (uint8_t i = 0; i < 6; i++) {
    counters[i] = loadCounterValue(i);
    Serial.printf("Restored counter WM%d: %d\n", i + 1, counters[i]);
  }
}

// ============== MCP Functions ==============

void mcp1Write(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MCP1_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mcp1Read(uint8_t reg){
  Wire.beginTransmission(MCP1_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP1_ADDR,(uint8_t)1);
  return Wire.read();
}

void mcp2Write(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MCP2_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mcp2Read(uint8_t reg){
  Wire.beginTransmission(MCP2_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP2_ADDR,(uint8_t)1);
  return Wire.read();
}

void setupMCP1(){
  mcp1Write(IODIRA1, 0xFF);  // پورت A تماماً ورودی برای DI
  mcp1Write(IODIRB1, 0xFF);  // پورت B تماماً ورودی برای WM
  mcp1Write(GPPUA1, 0xFF);   // پول آپ پورت A
  mcp1Write(GPPUB1, 0xFF);   // پول آپ پورت B

  // مقداردهی اولیه WM از پورت B
  uint8_t b = mcp1Read(GPIOB);
  uint8_t wm_pins[6] = {7, 6, 5, 4, 3, 2};
  for(uint8_t i=0;i<6;i++) 
    prevCounterState[i] = (b >> wm_pins[i]) & 0x01;

  // مقداردهی اولیه DI از پورت A  
  uint8_t a = mcp1Read(GPIOA);
  for(uint8_t i=0;i<8;i++) 
    prevDigitalState[i] = (a >> i) & 0x01;
}

void setupMCP2(){
  mcp2Write(IODIRA1,0xFF);
  mcp2Write(IODIRB1,0xFF);
  mcp2Write(GPPUA1,0xFF);
  mcp2Write(GPPUB1,0xFF);
}

uint8_t readMCP2Pin(uint8_t port, uint8_t pin){
  uint8_t val = (port==0)? mcp2Read(GPIOA) : mcp2Read(GPIOB);
  return (val & (1<<pin)) ? HIGH : LOW;
}

void ReadCountersTask(void * parameter){
  unsigned long lastPublish = millis();
  unsigned long lastSave = millis();
  
  for(;;){
    uint8_t gpioB = mcp1Read(GPIOB);
    // ترتیب پایه‌ها بر اساس اکسل: wm1=7, wm2=6, wm3=5, wm4=4, wm5=3, wm6=2
    uint8_t wm_pins[6] = {7, 6, 5, 4, 3, 2};
    
    for(uint8_t i=0;i<6;i++){
      uint8_t state = (gpioB >> wm_pins[i]) & 0x01;
      if(prevCounterState[i]==1 && state==0){
        counters[i]++;
      }
      prevCounterState[i]=state;
    }

    if(millis()-lastPublish >=10000){
      lastPublish = millis();
      for(uint8_t i=0;i<6;i++){
        char topic[20], msg[10];
        sprintf(topic,"patvaz/fim/1/wm%d",i+1);
        sprintf(msg,"%d",counters[i]);
        client.publish(topic,msg);
      }
    }

    if(millis()-lastSave >= 60000){
      lastSave = millis();
      for(uint8_t i=0;i<6;i++){
        saveCounterValue(i, counters[i]);
      }
    }

    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void ReadDigitalTask(void * parameter){
  for(;;){
    uint8_t gpioA = mcp1Read(GPIOA);
    for(uint8_t i=0;i<8;i++){
      uint8_t state = (gpioA >> i) & 0x01; // di1=پین0, di2=پین1, ..., di8=پین7
      if(state != prevDigitalState[i]){
        char topic[20], msg[10];
        sprintf(topic,"patvaz/fim/1/di%d",i+1);
        if(state==0) sprintf(msg,"%d",(i+1)*10+1);
        else sprintf(msg,"%d",(i+1)*10);
        client.publish(topic,msg);
        prevDigitalState[i]=state;
      }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

void SensorTask(void* pvParameters){
  for(;;){
    for(int i=0;i<12;i++){
      uint8_t val = readMCP2Pin(sensors[i].port, sensors[i].pin);
      uint8_t state = (val==LOW)? sensors[i].onValue : sensors[i].offValue;
      if(state != sensors[i].lastState){
        char msg[4];
        sprintf(msg,"%d",state);
        client.publish(sensors[i].topic,msg);
        sensors[i].lastState = state;
      }
    }
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

void mcpWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mcpRead(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((int)addr, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

void mcpWriteRegisterCtrl(uint8_t reg, uint8_t val) {
  mcpWrite(MCP_CTRL_ADDR, reg, val);
}

float readLM75() {
  Wire.beginTransmission(LM75_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);

  Wire.requestFrom(LM75_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return NAN;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  int16_t raw = (msb << 8) | lsb;
  int16_t temp9 = raw >> 7;
  return temp9 * 0.5;
}

void mcpInitCtrl() {
  mcpWriteRegisterCtrl(IODIRA, 0x00);
  mcpWriteRegisterCtrl(IODIRB, 0x00);
  mcpWriteRegisterCtrl(OLATA, 0x00);
  mcpWriteRegisterCtrl(OLATB, 0x00);
  ctrl_portA = 0;
  ctrl_portB = 0;
}

void mcpDigitalWriteCtrl(bool portAFlag, int pin, bool state) {
  if (portAFlag) {
    if (state) ctrl_portA |= (1 << pin);
    else ctrl_portA &= ~(1 << pin);
    mcpWriteRegisterCtrl(OLATA, ctrl_portA);
  } else {
    if (state) ctrl_portB |= (1 << pin);
    else ctrl_portB &= ~(1 << pin);
    mcpWriteRegisterCtrl(OLATB, ctrl_portB);
  }
}

void controlValve(Valve &v, bool dir) {
  if (!dir) {
    mcpDigitalWriteCtrl(v.portA, v.in1, LOW);
    mcpDigitalWriteCtrl(v.portA, v.in2, HIGH);
  } else {
    mcpDigitalWriteCtrl(v.portA, v.in1, HIGH);
    mcpDigitalWriteCtrl(v.portA, v.in2, LOW);
  }
  v.stopMillis = millis() + RUN_TIME;
  v.active = true;
}

void writeRegisterStat(uint8_t reg, uint8_t val){
  mcpWrite(MCP_STAT_ADDR, reg, val);
}

uint8_t readRegisterStat(uint8_t reg){
  return mcpRead(MCP_STAT_ADDR, reg);
}

void readValvesTask(void * pvParameters){
  for(;;){
    uint8_t gpioA = readRegisterStat(GPIOA);
    uint8_t gpioB = readRegisterStat(GPIOB);

    for(int i=0;i<6;i++){
      int pinOpen, pinClose;
      bool portA = false;

      // تعریف پایه‌ها بر اساس اکسل
      if(i == 0){ // evs1: پایه‌های 1,2 → پورت B پین 0,1
        pinOpen = 0; pinClose = 1; portA = false;
      }
      else if(i == 1){ // evs2: پایه‌های 3,4 → پورت B پین 2,3
        pinOpen = 2; pinClose = 3; portA = false;
      }
      else if(i == 2){ // evs3: پایه‌های 5,6 → پورت B پین 4,5
        pinOpen = 4; pinClose = 5; portA = false;
      }
      else if(i == 3){ // evs4: پایه‌های 7,8 → پورت B پین 6,7
        pinOpen = 6; pinClose = 7; portA = false;
      }
      else if(i == 4){ // evs5: پایه‌های 28,27 → پورت A پین 7,6
        pinOpen = 7; pinClose = 6; portA = true;
      }
      else if(i == 5){ // evs6: پایه‌های 26,25 → پورت A پین 5,4
        pinOpen = 5; pinClose = 4; portA = true;
      }

      int valOpen, valClose;

      if(portA){
        // پورت A
        valOpen = (gpioA & (1 << pinOpen)) ? 1 : 0;
        valClose = (gpioA & (1 << pinClose)) ? 1 : 0;
      } else {
        // پورت B  
        valOpen = (gpioB & (1 << pinOpen)) ? 1 : 0;
        valClose = (gpioB & (1 << pinClose)) ? 1 : 0;
      }

      uint16_t msgToSend = 0;

      // کدهای ارسالی بر اساس نام سنسور
      int base = (i + 1) * 10; // evs1=10, evs2=20, evs3=30, evs4=40, evs5=50, evs6=60
      if(valOpen == 0) msgToSend = base + 1;
      else if(valClose == 0) msgToSend = base;
      else msgToSend = 0;

      if(msgToSend != lastState[i]){
        lastState[i] = msgToSend;

        String topic = "patvaz/fim/1/evs" + String(i+1);

        char buf[6];
        sprintf(buf,"%d",msgToSend);
        client.publish(topic.c_str(), buf);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void mcpInitRelays() {
  mcpWrite(MCP_RELAY_ADDR, IODIRA, 0x00);
  mcpWrite(MCP_RELAY_ADDR, IODIRB, 0x00);
  olatA_relay = 0x00;
  olatB_relay = 0x00;
  mcpWrite(MCP_RELAY_ADDR, OLATA, olatA_relay);
  mcpWrite(MCP_RELAY_ADDR, OLATB, olatB_relay);
}

void setRelay(bool portA, uint8_t pin, bool state) {
  if (portA) {
    if (state) olatA_relay |= (1 << pin);
    else       olatA_relay &= ~(1 << pin);
    mcpWrite(MCP_RELAY_ADDR, OLATA, olatA_relay);
  } else {
    if (state) olatB_relay |= (1 << pin);
    else       olatB_relay &= ~(1 << pin);
    mcpWrite(MCP_RELAY_ADDR, OLATB, olatB_relay);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; ++i) msg += (char)payload[i];
  String topicStr = String(topic);

  digitalWrite(LED_PING_NET, HIGH);
  delay(50);
  digitalWrite(LED_PING_NET, LOW);

  if (topicStr == "patvaz/fim/1/reset") {
    if (msg == "all") {
      clearEEPROM("all");
      client.publish("patvaz/fim/1/reset_status", "All EEPROM cleared");
      delay(1000);
      ESP.restart();
    } else if (msg == "relay") {
      clearEEPROM("relay");
      client.publish("patvaz/fim/1/reset_status", "Relay EEPROM cleared");
    } else if (msg == "wm") {
      clearEEPROM("wm");
      for (uint8_t i = 0; i < 6; i++) {
        counters[i] = 0;
      }
      client.publish("patvaz/fim/1/reset_status", "Counter EEPROM cleared");
    }
    return;
  }

  if (String(topic) == "patvaz/fim/1/sw") {
    if (msg == "hi") hw_ok = true;
  }

  for (int i = 0; i < 5; i++) {
    String tname = "patvaz/fim/1/ev" + String(i + 1);
    if (String(topic) == tname) {
      if (msg == String((i + 1) * 10)) controlValve(valves[i], false);
      else if (msg == String((i + 1) * 10 + 1)) controlValve(valves[i], true);
      return;
    }
  }
  if (String(topic) == "patvaz/fim/1/ev11") {
    if (msg == "110") controlValve(valves[5], false);
    else if (msg == "111") controlValve(valves[5], true);
    return;
  }

  int cmd = msg.toInt();
  for (int i = 0; i < RELAY_COUNT; ++i) {
    if (String(topic) == String(relays[i].topic)) {
      if (cmd == relays[i].offCode) {
        setRelay(relays[i].portA, relays[i].pin, false);
        saveRelayState(i, 0);
      }
      else if (cmd == relays[i].onCode) {
        setRelay(relays[i].portA, relays[i].pin, true);
        saveRelayState(i, 1);
      }
      return;
    }
  }

  if (String(topic) == "patvaz/fim/1/porogram") {
    int correctValue = lastRA8Value;
    if (msg == "run") correctValue = 81;
    else if (msg == "stop") correctValue = 80;

    if (correctValue != lastRA8Value) {
      client.publish("patvaz/fim/1/ra8", String(correctValue).c_str());
      lastRA8Value = correctValue;
      Serial.print("Program status RA8 sent: ");
      Serial.println(correctValue);
    }
  }

  if (topicStr == HEARTBEAT_TOPIC_RECV && msg == "hi") {
    lastHiMillis = millis();
  }
}

void subscribeAllTopics() {
  for (int i = 1; i <= 5; i++) {
    String tname = "patvaz/fim/1/ev" + String(i);
    client.subscribe(tname.c_str());
  }
  client.subscribe("patvaz/fim/1/ev11");

  for (int i = 0; i < RELAY_COUNT; ++i) {
    client.subscribe(relays[i].topic);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_3MCP", mqtt_user, mqtt_pass)) {
      subscribeAllTopics();
      client.subscribe("patvaz/fim/1/porogram");
      client.subscribe(HEARTBEAT_TOPIC_RECV);
      client.subscribe("patvaz/fim/1/reset");
    } else {
      delay(2000);
    }
  }
}

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
        mcpDigitalWriteCtrl(valves[i].portA, valves[i].in1, LOW);
        mcpDigitalWriteCtrl(valves[i].portA, valves[i].in2, LOW);
        valves[i].active = false;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskLM75(void *pvParameters) {
  unsigned long lastMsg = 0;
  const long checkInterval = 1000;

  for (;;) {
    unsigned long now = millis();
    if (now - lastMsg > tempInterval) {
      lastMsg = now;
      float tempC = readLM75();
      if (!isnan(tempC)) {
        char buf[16];
        dtostrf(tempC, 4, 2, buf);
        Serial.print("Publish temperature: ");
        Serial.println(buf);
        client.publish("patvaz/fim/1/tempb", buf);

        if (tempC >= 32.0) {
          client.publish("patvaz/fim/1/ra5", "51");
          lastSentTempMsg = 51;
          sent51 = true;
          sent50 = false;
        } else if (tempC <= 28.0) {
          client.publish("patvaz/fim/1/ra5", "50");
          lastSentTempMsg = 50;
          sent50 = true;
          sent51 = false;
        }
      }
    }

    static unsigned long lastCheck = 0;
    if (now - lastCheck > checkInterval) {
      lastCheck = now;

      float tempC = readLM75();
      int correctMsg = 0;
      if (tempC >= 30.0) correctMsg = 51;
      else if (tempC <= 25.0) correctMsg = 50;

      if (correctMsg != 0 && lastSentTempMsg != correctMsg) {
        char buf[4];
        sprintf(buf, "%d", correctMsg);
        client.publish("patvaz/fim/1/ra5", buf);
        Serial.print("Corrected ra5 to: ");
        Serial.println(buf);
        lastSentTempMsg = correctMsg;
        sent50 = (correctMsg == 50);
        sent51 = (correctMsg == 51);
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void taskHeartbeat(void *param) {
  for (;;) {
    if (millis() - lastHeartbeatMillis >= HEARTBEAT_INTERVAL) {
      lastHeartbeatMillis = millis();
      client.publish(HEARTBEAT_TOPIC_SEND, "?");
      digitalWrite(LED_PUBLISH, HIGH);
      delay(50);
      digitalWrite(LED_PUBLISH, LOW);
    }
    if (millis() - lastHiMillis >= HEARTBEAT_TIMEOUT) {
      client.publish("patvaz/fim/1/ra4", "41");
      delay(1000);
      client.publish("patvaz/fim/1/ra4", "40");
      lastHiMillis = millis();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskWiFiStatus(void *pvParameters) {
  const long interval = 5000;
  for (;;) {
    int correctValue = WiFi.status() == WL_CONNECTED ? 61 : 60;

    if (lastRA6Value != correctValue) {
      client.publish("patvaz/fim/1/ra6", String(correctValue).c_str());
      lastRA6Value = correctValue;
      Serial.print("WiFi status RA6 sent: ");
      Serial.println(correctValue);
    }

    client.publish("patvaz/fim/1/ra6", String(lastRA6Value).c_str());

    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

void TaskMQTTStatus(void *pvParameters) {
  const long interval = 5000;
  for (;;) {
    int correctValue = client.connected() ? 71 : 70;

    if (lastRA7Value != correctValue) {
      client.publish("patvaz/fim/1/ra7", String(correctValue).c_str());
      lastRA7Value = correctValue;
      Serial.print("MQTT status RA7 sent: ");
      Serial.println(correctValue);
    }

    client.publish("patvaz/fim/1/ra7", String(lastRA7Value).c_str());

    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

void TaskReadSensors(void *pvParameters) {
  for (;;) {
    int16_t adcEC = ads.readADC_SingleEnded(0);
    float voltageEC = adcEC * 0.000125;
    float ec = slope * voltageEC + offset;
    if (ec < 0) ec = 0;
    float tds = ec * 0.5;
    if (tds < 0) tds = 0;
    char msgEC[16], msgTDS[16];
    dtostrf(ec, 5, 2, msgEC);
    dtostrf(tds, 5, 2, msgTDS);
    client.publish(topicEC, msgEC);
    client.publish(topicTDS, msgTDS);

    int16_t adcP = ads.readADC_SingleEnded(1);
    float voltageP = adcP * 0.000125;
    float current_mA = voltageP / R_SENSE;
    float pressure_bar = (current_mA - 4.0) * 10.0 / 16.0;
    if (pressure_bar < 0) pressure_bar = 0;
    char msgP[16];
    dtostrf(pressure_bar, 5, 2, msgP);
    client.publish(topicSP, msgP);

    int16_t adcNTC = ads.readADC_SingleEnded(2);
    float voltageNTC = adcNTC * 0.000125 * 1000;
    float R_ntc = (voltageNTC * R_FIXED_NTC) / (Vcc - voltageNTC);
    float tempK = 1.0 / ((1.0 / T0_NTC) + (1.0 / B_NTC) * log(R_ntc / R0_NTC));
    float tempC = tempK - 273.15;
    char msgNTC[16];
    dtostrf(tempC, 5, 2, msgNTC);
    client.publish(topicNTC, msgNTC);

    Serial.print("EC: "); Serial.print(msgEC);
    Serial.print(" | TDS: "); Serial.print(msgTDS);
    Serial.print(" | Pressure: "); Serial.print(msgP);
    Serial.print(" | NTC: "); Serial.println(msgNTC);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void taskWiFiConnectionCheck(void *param) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi Disconnected → Reconnecting...");
      digitalWrite(LED_PING_NET, LOW);

      WiFi.disconnect();
      WiFi.begin(ssid, password);

      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ WiFi Connected again!");
        digitalWrite(LED_PING_NET, HIGH);
      } else {
        Serial.println("❌ WiFi reconnect failed, restarting...");
        digitalWrite(POWER_MODEM, HIGH);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        digitalWrite(POWER_MODEM, LOW);
        ESP.restart();
      }
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initEEPROM();

  pinMode(LED_PING_NET, OUTPUT);
  digitalWrite(LED_PING_NET, HIGH);

  pinMode(LED_PUBLISH, OUTPUT);
  digitalWrite(LED_PUBLISH, HIGH);

  pinMode(POWER_MODEM, OUTPUT);
  digitalWrite(POWER_MODEM, LOW);

  pinMode(buzer_PIN, OUTPUT);
  digitalWrite(buzer_PIN, HIGH);
  delay(500);
  digitalWrite(buzer_PIN, LOW);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(1);
  digitalWrite(RESET_PIN, HIGH);
  delay(5);

  mcpInitCtrl();

  writeRegisterStat(IODIRA, 0xFF);
  writeRegisterStat(IODIRB, 0xFF);
  writeRegisterStat(GPPUA, 0xFF);
  writeRegisterStat(GPPUB, 0xFF);

  mcpInitRelays();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  setupMCP1();
  setupMCP2();

  WiFi.begin(ssid, password);

  restoreCounterValues();

  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskValves, "TaskValves", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(readValvesTask,"ReadValves",4096,NULL,1, NULL,1);
  xTaskCreatePinnedToCore(TaskLM75, "TaskLM75", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskHeartbeat, "Heartbeat", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskWiFiStatus, "TaskWiFiStatus", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMQTTStatus, "TaskMQTTStatus", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ReadCountersTask,"Counters",4096,NULL,1,&CounterTaskHandle,1);
  xTaskCreatePinnedToCore(ReadDigitalTask,"DigitalInputs",4096,NULL,1,&DigitalTaskHandle,1);
  xTaskCreatePinnedToCore(taskWiFiConnectionCheck, "WiFiFail", 2048, NULL, 1, NULL, 1);
  xTaskCreate(SensorTask,"SensorTask",4096,NULL,1,NULL);

  if (!ads.begin(0x49)) {
    Serial.println("Failed to initialize ADS1115");
    while (1);
  }

  xTaskCreate(TaskReadSensors, "Read Sensors", 4096, NULL, 1, NULL);

  delay(3000);
  restoreRelayStates();
}

void loop() {
}
