#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== تنظیمات WiFi و MQTT ==================
const char* ssid = "patvaz";
const char* password = "qazvin402";

const char* mqtt_server = "217.144.107.162";
const int mqtt_port = 1883;
const char* mqtt_user = "Patvaz";
const char* mqtt_pass = "P@vaz2025";

// ================== آدرس MCP23017 ==================
#define MCP_ADDR 0x21  // آدرس I2C پیش‌فرض

#define RESET_PIN 4   // پایه ریست MCP23017
// ================== متغیرهای MQTT ==================
WiFiClient espClient;
PubSubClient client(espClient);

// ================== نگه داشتن وضعیت قبلی ==================
uint16_t lastState[6] = {0}; // 5 شیر اول + شیر 6

// ================== توابع I2C ==================
void writeRegister(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg){
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP_ADDR, (uint8_t)1);
  if(Wire.available()){
    return Wire.read();
  }
  return 0;
}

// ================== خواندن وضعیت شیرها ==================
void readValvesTask(void * pvParameters){
  for(;;){
    uint8_t gpioA = readRegister(0x12); // پایه 0-7
    uint8_t gpioB = readRegister(0x13); // پایه 8-15

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

        char buf[5];
        sprintf(buf,"%d",msgToSend);
        client.publish(topic.c_str(), buf);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ================== اتصال MQTT ==================
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32ValveReader", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);

  // راه‌اندازی I2C
  Wire.begin();
pinMode(RESET_PIN, OUTPUT);
digitalWrite(RESET_PIN, LOW);
delay(1);
digitalWrite(RESET_PIN, HIGH);
delay(5);

  // پیکربندی MCP23017
  writeRegister(0x00, 0xFF); // IODIRA همه پایه‌ها ورودی
  writeRegister(0x01, 0xFF); // IODIRB همه پایه‌ها ورودی
  writeRegister(0x0C, 0xFF); // GPPUA pull-up فعال
  writeRegister(0x0D, 0xFF); // GPPUB pull-up فعال

  // راه‌اندازی WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // راه‌اندازی MQTT
  client.setServer(mqtt_server, mqtt_port);
  reconnect();

  // ایجاد Task
  xTaskCreatePinnedToCore(readValvesTask,"ReadValves",4096,NULL,1, NULL,1);
}

// ================== Loop ==================
void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}