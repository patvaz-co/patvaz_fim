#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <math.h>

// =================== WiFi ===================
const char* ssid = "Patvaz_Host";
const char* password = "P@vaz2025";
const char* hostname = "esp32_monitor";

// =================== MQTT ===================
const char* mqtt_server = "192.168.43.1";
WiFiClient espClient;
PubSubClient client(espClient);

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

// =================== توابع WiFi و MQTT ===================
void setup_wifi() {
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
}

void setup_ota() {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
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

// =================== Setup ===================
void setup() {
  Serial.begin(115200);

  setup_wifi();
  setup_ota();
  client.setServer(mqtt_server, 1883);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115");
    while (1);
  }

  xTaskCreate(TaskReadSensors, "Read Sensors", 4096, NULL, 1, NULL);
}

// =================== Loop ===================
void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) reconnect();
  client.loop();
}
