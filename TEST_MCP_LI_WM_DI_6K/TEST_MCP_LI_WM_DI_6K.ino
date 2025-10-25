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
// MCP1: کانترها و ورودی دیجیتال
#define MCP1_ADDR 0x22
// MCP2: سنسورهای سطح آب
#define MCP2_ADDR 0x23
#define RESET_PIN 4   // پایه ریست مشترک

#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA 0x12
#define GPIOB 0x13
#define GPPUA 0x0C
#define GPPUB 0x0D

// =================== داده ها ===================
// کانترها
uint16_t counters[6] = {0};
uint8_t prevCounterState[6];
uint8_t digitalState[8];
uint8_t prevDigitalState[8];

// سنسورها
struct Sensor {
  uint8_t port;        // 0 = A, 1 = B
  uint8_t pin;         // 0-7
  const char* topic;
  uint8_t onValue;
  uint8_t offValue;
  uint8_t lastState;
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

// =================== FreeRTOS ===================
TaskHandle_t CounterTaskHandle;
TaskHandle_t DigitalTaskHandle;

// =================== توابع I2C MCP1 ===================
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
  mcp1Write(IODIRA, 0xFF);
  mcp1Write(IODIRB, 0xFF);
  mcp1Write(GPPUA, 0xFF);
  mcp1Write(GPPUB, 0xFF);

  uint8_t a = mcp1Read(GPIOA);
  for(uint8_t i=0;i<6;i++) prevCounterState[i] = (a>>i)&0x01;

  uint8_t b = mcp1Read(GPIOB);
  for(uint8_t i=0;i<8;i++) prevDigitalState[i] = (b>>i)&0x01;
}

// =================== راه اندازی MCP2 ===================
void setupMCP2(){
  mcp2Write(IODIRA,0xFF);
  mcp2Write(IODIRB,0xFF);
  mcp2Write(GPPUA,0xFF);
  mcp2Write(GPPUB,0xFF);
}

// =================== توابع عمومی ===================
uint8_t readMCP2Pin(uint8_t port, uint8_t pin){
  uint8_t val = (port==0)? mcp2Read(GPIOA) : mcp2Read(GPIOB);
  return (val & (1<<pin)) ? HIGH : LOW;
}

// =================== WiFi & MQTT ===================
void setupWiFi(){
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED) delay(500);
}

void reconnectMQTT(){
  while(!client.connected()){
    if(client.connect("ESP32Client",mqtt_user,mqtt_pass)){}
    else delay(2000);
  }
}

// =================== تسک کانترها ===================
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

// =================== setup ===================
void setup(){
  Wire.begin();

  pinMode(RESET_PIN,OUTPUT);
  digitalWrite(RESET_PIN,LOW);
  delay(1);
  digitalWrite(RESET_PIN,HIGH);
  delay(5);

  setupWiFi();
  client.setServer(mqtt_server,1883);
  setupMCP1();
  setupMCP2();

  xTaskCreatePinnedToCore(ReadCountersTask,"Counters",4096,NULL,1,&CounterTaskHandle,1);
  xTaskCreatePinnedToCore(ReadDigitalTask,"DigitalInputs",4096,NULL,1,&DigitalTaskHandle,1);
  xTaskCreate(SensorTask,"SensorTask",4096,NULL,1,NULL);
}

// =================== loop ===================
void loop(){
  if(!client.connected()) reconnectMQTT();
  client.loop();
}
