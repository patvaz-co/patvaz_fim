#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

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
#define MCP_CTRL_ADDR  0x20  // کنترل شیرها (کد اول)
#define MCP_STAT_ADDR  0x21  // خواندن وضعیت شیرها (کد دوم)
#define MCP_RELAY_ADDR 0x24  // رله‌ها (کد سوم)
#define MCP1_ADDR 0x22// MCP1: کانترها و ورودی دیجیتال
#define MCP2_ADDR 0x23// MCP2: سنسورهای سطح آب

#define RESET_PIN 4

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



#define LED_PING_NET 33   //
#define LED_PUBLISH 32
#define POWER_MODEM 27
#define buzer_PIN 26


uint16_t counters[6] = {0};
uint8_t prevCounterState[6];
uint8_t digitalState[8];
uint8_t prevDigitalState[8];

struct Sensor {
  uint8_t port;        // 0 = A, 1 = B
  uint8_t pin;         // 0-7
  const char* topic;
  uint8_t onValue;
  uint8_t offValue;
  uint8_t lastState;
};

// =================== ADS1115 ===================
Adafruit_ADS1115 ads; // آدرس پیش‌فرض 0x48

// =================== سنسورها ===================
const float Vcc = 3.28;  // برای EC
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



// =================== بخش 1: کنترل شیرها (از کد اول شما) ===================
struct Valve {
  int in1;
  int in2;
  unsigned long stopMillis;
  bool active;
  bool portA; // true = Port A, false = Port B
};

Sensor sensors[12] = {
  {0,0,"patvaz/fim/1/li10",11,10,255},
  {0,1,"patvaz/fim/1/li11",11,10,255},
  {0,2,"patvaz/fim/1/li20",21,20,255},
  {0,3,"patvaz/fim/1/li21",21,20,255},
  {0,4,"patvaz/fim/1/li30",31,30,255},
  {0,5,"patvaz/fim/1/li31",31,30,255},
  {1,0,"patvaz/fim/1/li40",41,40,255},
  {1,1,"patvaz/fim/1/li41",41,40,255},
  {1,2,"patvaz/fim/1/li50",51,50,255},
  {1,3,"patvaz/fim/1/li51",51,50,255},
  {1,4,"patvaz/fim/1/li60",61,60,255},
  {1,5,"patvaz/fim/1/li61",61,60,255}
};

TaskHandle_t CounterTaskHandle;
TaskHandle_t DigitalTaskHandle;







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

// =================== توابع I2C MCP2 ===================
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

// =================== راه اندازی MCP1 ===================
void setupMCP1(){
  mcp1Write(IODIRA1, 0xFF);
  mcp1Write(IODIRB1, 0xFF);
  mcp1Write(GPPUA1, 0xFF);
  mcp1Write(GPPUB1, 0xFF);

  uint8_t a = mcp1Read(GPIOA);
  for(uint8_t i=0;i<6;i++) prevCounterState[i] = (a>>i)&0x01;

  uint8_t b = mcp1Read(GPIOB);
  for(uint8_t i=0;i<8;i++) prevDigitalState[i] = (b>>i)&0x01;
}

// =================== راه اندازی MCP2 ===================
void setupMCP2(){
  mcp2Write(IODIRA1,0xFF);
  mcp2Write(IODIRB1,0xFF);
  mcp2Write(GPPUA1,0xFF);
  mcp2Write(GPPUB1,0xFF);
}



// =================== توابع عمومی ===================
uint8_t readMCP2Pin(uint8_t port, uint8_t pin){
  uint8_t val = (port==0)? mcp2Read(GPIOA) : mcp2Read(GPIOB);
  return (val & (1<<pin)) ? HIGH : LOW;
}




void ReadCountersTask(void * parameter){
  unsigned long lastPublish = millis();
  for(;;){
    uint8_t gpioA = mcp1Read(GPIOA);
    for(uint8_t i=0;i<6;i++){
      uint8_t state = (gpioA>>i)&0x01;
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
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

// =================== تسک ورودی دیجیتال ===================
void ReadDigitalTask(void * parameter){
  for(;;){
    uint8_t gpioB = mcp1Read(GPIOB);
    for(uint8_t i=0;i<8;i++){
      uint8_t state = (gpioB>>i)&0x01;
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

// =================== تسک سنسورها ===================
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




// Generic MCP read/write helpers
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



#define RUN_TIME 10000

Valve valves[6] = {
  {0, 1, 0, false, true},   // ev1 (A0, A1)
  {2, 3, 0, false, true},   // ev2 (A2, A3)
  {4, 5, 0, false, true},   // ev3 (A4, A5)
  {6, 7, 0, false, true},   // ev4 (A6, A7)
  {0, 1, 0, false, false},  // ev5 (B0, B1)
  {2, 3, 0, false, false}   // ev11 (B2, B3)
};

uint8_t ctrl_portA = 0;
uint8_t ctrl_portB = 0;

void mcpWriteRegisterCtrl(uint8_t reg, uint8_t val) {
  mcpWrite(MCP_CTRL_ADDR, reg, val);
}




// ---------------- LM75 ----------------
const uint8_t LM75_ADDR = 0x48; // آدرس پیش‌فرض

float readLM75() {
  Wire.beginTransmission(LM75_ADDR);
  Wire.write(0x00); // رجیستر دما
  Wire.endTransmission(false);

  Wire.requestFrom(LM75_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return NAN;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  int16_t raw = (msb << 8) | lsb;
  int16_t temp9 = raw >> 7;  // 9 بیت signed
  return temp9 * 0.5;        // هر واحد = 0.5 درجه
}






void mcpInitCtrl() {
  // همه پین‌ها خروجی
  mcpWriteRegisterCtrl(IODIRA, 0x00);
  mcpWriteRegisterCtrl(IODIRB, 0x00);
  // مقدار اولیه 0
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
  if (!dir) { // بسته
    mcpDigitalWriteCtrl(v.portA, v.in1, LOW);
    mcpDigitalWriteCtrl(v.portA, v.in2, HIGH);
  } else {    // باز
    mcpDigitalWriteCtrl(v.portA, v.in1, HIGH);
    mcpDigitalWriteCtrl(v.portA, v.in2, LOW);
  }
  v.stopMillis = millis() + RUN_TIME;
  v.active = true;
}

// =================== بخش 2: خواندن وضعیت شیرها (از کد دوم شما) ===================
uint16_t lastState[6] = {0};

void writeRegisterStat(uint8_t reg, uint8_t val){
  mcpWrite(MCP_STAT_ADDR, reg, val);
}

uint8_t readRegisterStat(uint8_t reg){
  return mcpRead(MCP_STAT_ADDR, reg);
}

void readValvesTask(void * pvParameters){
  for(;;){
    uint8_t gpioA = readRegisterStat(GPIOA); // پایه 0-7
    uint8_t gpioB = readRegisterStat(GPIOB); // پایه 8-15

    // بررسی هر شیر
    for(int i=0;i<6;i++){
      int pinOpen, pinClose;

      if(i < 5){ // شیر 1 تا 5
        pinOpen  = i*2;     // پایه اول
        pinClose = i*2 + 1; // پایه دوم
      } else { // شیر 6
        pinOpen  = 10;
        pinClose = 11;
      }

      int valOpen, valClose;

      // خواندن وضعیت پایه‌ها
      valOpen  = (pinOpen  < 8) ? ((gpioA & (1<<pinOpen))  ? 1 : 0) : ((gpioB & (1<<(pinOpen-8)))  ? 1 : 0);
      valClose = (pinClose < 8) ? ((gpioA & (1<<pinClose)) ? 1 : 0) : ((gpioB & (1<<(pinClose-8))) ? 1 : 0);

      uint16_t msgToSend = 0;

      if(i < 5){ // شیر 1 تا 5
        int base = (i+1)*10;
        if(valOpen == 0) msgToSend = base + 1;   // باز
        else if(valClose == 0) msgToSend = base; // بسته
        else msgToSend = 0;                       // وسط / None
      } else { // شیر 6
        if(valOpen == 0) msgToSend = 111;   // باز
        else if(valClose == 0) msgToSend = 110; // بسته
        else msgToSend = 0;                       // وسط
      }

      // فقط وقتی تغییر کرد پیام ارسال شود
      if(msgToSend != lastState[i]){
        lastState[i] = msgToSend;

        String topic = (i<5) ? "patvaz/fim/1/evs" + String(i+1) : "patvaz/fim/1/evs11";

        char buf[6];
        sprintf(buf,"%d",msgToSend);
        client.publish(topic.c_str(), buf);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// =================== بخش 3: رله‌ها (از کد سوم شما) ===================
uint8_t olatA_relay = 0x00;
uint8_t olatB_relay = 0x00;

void mcpInitRelays() {
  mcpWrite(MCP_RELAY_ADDR, IODIRA, 0x00); // PortA خروجی
  mcpWrite(MCP_RELAY_ADDR, IODIRB, 0x00); // PortB خروجی
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

struct RelayDef {
  bool portA;        // true => Port A, false => Port B
  uint8_t pin;       // pin number within the port (0..7)
  const char* topic; // topic string
  int offCode;       // e.g. 10,20,... (string payload تبدیل می شود به عدد)
  int onCode;        // e.g. 11,21,...
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
int lastRA8Value = 80; // مقدار پیش‌فرض: stop = 80
volatile bool hw_ok = false; 
// =================== MQTT callback (مرکزی) ===================
// این callback هر دو حالت (شیرها و رله‌ها) را هندل می‌کند.
// من منطق هر کد شما را بدون تغییر نگه‌داشتم.
void callback(char* topic, byte* payload, unsigned int length) {
  // ساختن رشته پیام
  String msg;
  for (unsigned int i = 0; i < length; ++i) msg += (char)payload[i];



  digitalWrite(LED_PING_NET, HIGH);
  delay(50);
  digitalWrite(LED_PING_NET, LOW);



    // --------- Heartbeat ----------
    if (String(topic) == "patvaz/fim/1/sw") {
        if (msg == "hi") hw_ok = true;
    }

  // --- هندل شیرها (همان منطق کد اول) ---
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

  // --- هندل رله‌ها (همان منطق کد سوم) ---
  int cmd = msg.toInt();
  for (int i = 0; i < RELAY_COUNT; ++i) {
    if (String(topic) == String(relays[i].topic)) {
      if (cmd == relays[i].offCode) setRelay(relays[i].portA, relays[i].pin, false);
      else if (cmd == relays[i].onCode) setRelay(relays[i].portA, relays[i].pin, true);
      return;
    }
  }


    // --- برنامه (Program) ---
    if (String(topic) == "patvaz/fim/1/porogram") {
        int correctValue = lastRA8Value; // مقدار پیش‌فرض نگه‌داری شود
        if (msg == "run") correctValue = 81;
        else if (msg == "stop") correctValue = 80;

        if (correctValue != lastRA8Value) {
            client.publish("patvaz/fim/1/ra8", String(correctValue).c_str());
            lastRA8Value = correctValue;
            Serial.print("Program status RA8 sent: ");
            Serial.println(correctValue);
        }
    }




}

// =================== MQTT / اتصال / subscribe ===================
void subscribeAllTopics() {
  // شیرها (کد اول)
  for (int i = 1; i <= 5; i++) {
    String tname = "patvaz/fim/1/ev" + String(i);
    client.subscribe(tname.c_str());
  }
  client.subscribe("patvaz/fim/1/ev11");

  // رله‌ها (کد سوم)
  for (int i = 0; i < RELAY_COUNT; ++i) {
    client.subscribe(relays[i].topic);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_3MCP", mqtt_user, mqtt_pass)) {
      subscribeAllTopics();
       client.subscribe("patvaz/fim/1/porogram");
    } else {
      delay(2000);
    }
  }
}

// =================== Tasks ===================
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




// ---------- Task جدید برای LM75 ----------
bool sent50 = false;
bool sent51 = false;
const long tempInterval = 10000; // هر 20 ثانیه

float lastSentTempMsg = NAN; // مقدار آخرین پیام روی تاپیک ra5

void TaskLM75(void *pvParameters) {
  unsigned long lastMsg = 0;
  const long checkInterval = 1000; // بررسی هر 3 ثانیه

  for (;;) {
    unsigned long now = millis();
    if (now - lastMsg > tempInterval) {  // هر 20 ثانیه دما خوانده و ارسال اولیه
      lastMsg = now;
      float tempC = readLM75();
      if (!isnan(tempC)) {
        char buf[16];
        dtostrf(tempC, 4, 2, buf);
        Serial.print("Publish temperature: ");
        Serial.println(buf);
        client.publish("patvaz/fim/1/tempb", buf);

        // تصمیم‌گیری براساس دما برای ra5
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

    // ---------- بررسی وضعیت تاپیک ra5 هر 3 ثانیه ----------
    static unsigned long lastCheck = 0;
    if (now - lastCheck > checkInterval) {
      lastCheck = now;

      // بررسی وضعیت داخلی (متغیر lastSentTempMsg)
      // اگر پیام روی تاپیک تغییر کرده باشه و با دما مطابقت نداشته باشه، اصلاح شود
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




int lastRA4Value = 0;           // آخرین مقدار صحیح روی تاپیک RA4

void TaskHeartbeat(void *pvParameters) {
    const long interval = 10000; // هر 10 ثانیه
    unsigned long lastSend = 0;

    for (;;) {
        unsigned long now = millis();
        if (now - lastSend > interval) {
            lastSend = now;
            hw_ok = false; // ریست برای هر چرخه

            // ارسال علامت سوال
            client.publish("patvaz/fim/1/hw", "?");
                 digitalWrite(LED_PUBLISH, HIGH);
                 delay(50);
                 digitalWrite(LED_PUBLISH, LOW);

            // 10 ثانیه صبر برای دریافت hi
            vTaskDelay(pdMS_TO_TICKS(30000));

            if (!hw_ok) {
                // پیام 41 به مدت 3 ثانیه
                client.publish("patvaz/fim/1/ra4", "41");
                lastRA4Value = 41;
                vTaskDelay(pdMS_TO_TICKS(3000));

                // سپس 40
                client.publish("patvaz/fim/1/ra4", "40");
                lastRA4Value = 40;
            }

    
            client.publish("patvaz/fim/1/ra4", String(lastRA4Value).c_str());
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // کاهش مصرف CPU
    }
}



int lastRA6Value = 60; // مقدار پیش‌فرض، 60 = قطع WiFi

void TaskWiFiStatus(void *pvParameters) {
    const long interval = 5000; // هر 5 ثانیه بررسی شود
    for (;;) {
        int correctValue = WiFi.status() == WL_CONNECTED ? 61 : 60;

        // فقط اگر تغییر کرد یا اشتباه نشست، منتشر شود
        if (lastRA6Value != correctValue) {
            client.publish("patvaz/fim/1/ra6", String(correctValue).c_str());
            lastRA6Value = correctValue;
            Serial.print("WiFi status RA6 sent: ");
            Serial.println(correctValue);
        }

        // هر چند ثانیه یک بار بررسی کنیم که پیام RA6 درست است یا نه
        client.publish("patvaz/fim/1/ra6", String(lastRA6Value).c_str());

        vTaskDelay(pdMS_TO_TICKS(interval));
    }
}



int lastRA7Value = 70; // مقدار پیش‌فرض، 70 = MQTT قطع

void TaskMQTTStatus(void *pvParameters) {
    const long interval = 5000; // هر 5 ثانیه بررسی
    for (;;) {
        int correctValue = client.connected() ? 71 : 70;

        // فقط اگر تغییر کرد یا اشتباه نشست، منتشر شود
        if (lastRA7Value != correctValue) {
            client.publish("patvaz/fim/1/ra7", String(correctValue).c_str());
            lastRA7Value = correctValue;
            Serial.print("MQTT status RA7 sent: ");
            Serial.println(correctValue);
        }

        // هر چند ثانیه یک بار بررسی کنیم که پیام RA7 درست است یا نه
        client.publish("patvaz/fim/1/ra7", String(lastRA7Value).c_str());

        vTaskDelay(pdMS_TO_TICKS(interval));
    }
}




// =================== Task مشترک برای خواندن همه سنسورها ===================
void TaskReadSensors(void *pvParameters) {
  for (;;) {
    // --------- EC/TDS (A0) ---------
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

    // --------- فشار (A1) ---------
    int16_t adcP = ads.readADC_SingleEnded(1);
    float voltageP = adcP * 0.000125;
    float current_mA = voltageP / R_SENSE;
    float pressure_bar = (current_mA - 4.0) * 10.0 / 16.0;
    if (pressure_bar < 0) pressure_bar = 0;
    char msgP[16];
    dtostrf(pressure_bar, 5, 2, msgP);
    client.publish(topicSP, msgP);

    // --------- NTC (A2) ---------
    int16_t adcNTC = ads.readADC_SingleEnded(2);
    float voltageNTC = adcNTC * 0.000125 * 1000; // تبدیل به ولتاژ واقعی (در صورت نیاز)
    float R_ntc = (voltageNTC * R_FIXED_NTC) / (Vcc - voltageNTC);
    float tempK = 1.0 / ((1.0 / T0_NTC) + (1.0 / B_NTC) * log(R_ntc / R0_NTC));
    float tempC = tempK - 273.15;
    char msgNTC[16];
    dtostrf(tempC, 5, 2, msgNTC);
    client.publish(topicNTC, msgNTC);

    // نمایش روی سریال
    Serial.print("EC: "); Serial.print(msgEC);
    Serial.print(" | TDS: "); Serial.print(msgTDS);
    Serial.print(" | Pressure: "); Serial.print(msgP);
    Serial.print(" | NTC: "); Serial.println(msgNTC);

    vTaskDelay(10000 / portTICK_PERIOD_MS); // هر 10 ثانیه
  }
}



void taskWiFiConnectionCheck(void *param) {
  unsigned long startMillis1 = millis();
  for (;;) {
    if (millis() - startMillis1 > 60000 && WiFi.status() != WL_CONNECTED) {  // WIFI CONNECTION TIME OUT 60S
      Serial.println("❌ WiFi connection failed → Restarting");
      digitalWrite(POWER_MODEM, HIGH);
      delay(5000);
      digitalWrite(POWER_MODEM, LOW);
      ESP.restart();
    }

    if (WiFi.status() == WL_CONNECTED) {
      vTaskDelete(NULL);
      digitalWrite(LED_PING_NET, HIGH);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}












// =================== setup / loop ===================
void setup() {
  Wire.begin();


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

  // Reset MCPs (یکبار برای همه)
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(1);
  digitalWrite(RESET_PIN, HIGH);
  delay(5);

  // Init هر سه IC با همان منطق اصلی شما
  // IC کنترل شیرها (خروجی)
  mcpInitCtrl();

  // IC وضعیت شیرها (همه ورودی + pull-up)
  writeRegisterStat(IODIRA, 0xFF); // IODIRA همه پایه‌ها ورودی
  writeRegisterStat(IODIRB, 0xFF); // IODIRB همه پایه‌ها ورودی
  writeRegisterStat(GPPUA, 0xFF);  // GPPUA pull-up فعال
  writeRegisterStat(GPPUB, 0xFF);  // GPPUB pull-up فعال

  // IC رله‌ها
  mcpInitRelays();

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
 
   setupMCP1();
  setupMCP2();

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  // Tasks
  xTaskCreatePinnedToCore(TaskMQTT, "TaskMQTT", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskValves, "TaskValves", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(readValvesTask,"ReadValves",4096,NULL,1, NULL,1);
  xTaskCreatePinnedToCore(TaskLM75, "TaskLM75", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskHeartbeat, "TaskHeartbeat", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskWiFiStatus, "TaskWiFiStatus", 4096, NULL, 1, NULL, 1);
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


}

void loop() {
  // همه‌چی توی Task ها اجرا میشه
}
