#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// =================== WiFi & MQTT ===================
const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "217.144.107.162";
const char* mqtt_user = "Patvaz";
const char* mqtt_pass = "P@vaz2025";

WiFiClient espClient;
PubSubClient client(espClient);

// =================== MCP23017 ===================
#define MCP_ADDR 0x21  // آدرس I2C
#define RESET_PIN 4   // پایه ریست MCP23017
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA 0x12
#define GPIOB 0x13
#define GPPUA 0x0C
#define GPPUB 0x0D

uint16_t counters[6] = {0};
uint8_t prevCounterState[6];
uint8_t digitalState[8];
uint8_t prevDigitalState[8];

// =================== FreeRTOS ===================
TaskHandle_t CounterTaskHandle;
TaskHandle_t DigitalTaskHandle;

// =================== توابع کمکی ===================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// =================== راه‌اندازی MCP ===================
void setupMCP() {
  // کانال A (0-5) و B (8-15) همه ورودی
  writeRegister(IODIRA, 0xFF); // A همه ورودی
  writeRegister(IODIRB, 0xFF); // B همه ورودی

  // pull-up داخلی فعال
  writeRegister(GPPUA, 0xFF);
  writeRegister(GPPUB, 0xFF);

  // وضعیت اولیه
  uint8_t a = readRegister(GPIOA);
  for(uint8_t i=0;i<6;i++) prevCounterState[i] = (a>>i)&0x01;

  uint8_t b = readRegister(GPIOB);
  for(uint8_t i=0;i<8;i++) prevDigitalState[i] = (b>>i)&0x01;
}

// =================== WiFi و MQTT ===================
void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void reconnectMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      // متصل شد
    } else {
      delay(2000);
    }
  }
}

// =================== تسک کانترها ===================
void ReadCountersTask(void * parameter) {
  unsigned long lastPublish = millis();
  for(;;){
    uint8_t gpioA = readRegister(GPIOA);
    for(uint8_t i=0;i<6;i++){
      uint8_t state = (gpioA>>i)&0x01;
      // لبه پایین => ثبت پالس
      if(prevCounterState[i]==1 && state==0){
        counters[i]++;  // هر پالس شمارش می‌شود
      }
      prevCounterState[i]=state;
    }

    // ارسال MQTT هر 10 ثانیه
    if(millis() - lastPublish >= 10000){
      lastPublish = millis();
      for(uint8_t i=0;i<6;i++){
        char topic[20], msg[10];
        sprintf(topic,"patvaz/fim/1/wm%d",i+1);
        sprintf(msg,"%d",counters[i]);
        client.publish(topic,msg);
      }
    }

    vTaskDelay(10/portTICK_PERIOD_MS); // بررسی هر 10ms
  }
}


// =================== تسک ورودی دیجیتال ===================
void ReadDigitalTask(void * parameter) {
  for(;;){
    uint8_t gpioB = readRegister(GPIOB);
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

// =================== setup ===================
void setup() {
  Wire.begin();

  pinMode(RESET_PIN, OUTPUT);
digitalWrite(RESET_PIN, LOW);
delay(1);
digitalWrite(RESET_PIN, HIGH);
delay(5);

  setupWiFi();
  client.setServer(mqtt_server,1883);
  setupMCP();

  xTaskCreatePinnedToCore(ReadCountersTask,"Counters",4096,NULL,1,&CounterTaskHandle,1);
  xTaskCreatePinnedToCore(ReadDigitalTask,"DigitalInputs",4096,NULL,1,&DigitalTaskHandle,1);
}

// =================== loop ===================
void loop() {
  if(!client.connected()) reconnectMQTT();
  client.loop();
}
