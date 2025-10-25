#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------- WiFi ----------
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
#define MCP23017_ADDR 0x21
#define RESET_PIN 4   // پایه ریست MCP23017
// ثبات‌ها
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA  0x12
#define GPIOB  0x13
#define GPPUA  0x0C
#define GPPUB  0x0D

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

// ---------- I2C Functions ----------
void mcpWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mcpRead(uint8_t reg){
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP23017_ADDR,(uint8_t)1);
  return Wire.read();
}

void mcpPinModeAllInput(){
  mcpWrite(IODIRA, 0xFF);
  mcpWrite(IODIRB, 0xFF);
  mcpWrite(GPPUA, 0xFF);
  mcpWrite(GPPUB, 0xFF);
}

uint8_t readPin(uint8_t port, uint8_t pin){
  uint8_t val = (port==0)? mcpRead(GPIOA) : mcpRead(GPIOB);
  return (val & (1<<pin)) ? HIGH : LOW;
}

// ---------- Sensor Task ----------
void SensorTask(void* pvParameters){
  for(;;){
    for(int i=0;i<12;i++){
      uint8_t val = readPin(sensors[i].port, sensors[i].pin);
      uint8_t state = (val == LOW) ? sensors[i].onValue : sensors[i].offValue;
      if(state != sensors[i].lastState){
        char msg[4];
        sprintf(msg,"%d",state);
        client.publish(sensors[i].topic,msg);
        sensors[i].lastState = state;
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ---------- MQTT Reconnect ----------
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_Sensors", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(RESET_PIN, OUTPUT);
digitalWrite(RESET_PIN, LOW);
delay(1);
digitalWrite(RESET_PIN, HIGH);
delay(5);

  mcpPinModeAllInput();

  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  client.setServer(mqtt_server,1883);
  reconnect();

  xTaskCreate(SensorTask,"SensorTask",4096,NULL,1,NULL);
}

// ---------- Loop ----------
void loop() {
  if(!client.connected()) reconnect();
  client.loop();
}
