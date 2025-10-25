#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// ---------------- WiFi ----------------
const char* ssid = "patvaz";
const char* password = "qazvin402";

// ---------------- MQTT ----------------
const char* mqtt_server = "217.144.107.162"; // آی‌پی یا آدرس MQTT بروکر
const int   mqtt_port   = 1883;
const char* mqtt_user   = "Patvaz";  // اگر نیاز نداری خالی بذار
const char* mqtt_pass   = "P@vaz2025";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- LM75 ----------------
const uint8_t LM75_ADDR = 0x48; // آدرس پیش‌فرض سنسور

// ---------------- توابع ----------------
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // تا وقتی وصل نشه تلاش می‌کنه
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

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

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA=21 , SCL=22 (پیش‌فرض ESP32)

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

// ---------------- Loop ----------------
unsigned long lastMsg = 0;
const long interval = 20000; // هر 20 ثانیه

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;

    float tempC = readLM75();
    if (!isnan(tempC)) {
      char msg[16];
      dtostrf(tempC, 4, 2, msg);  // تبدیل float به string
      Serial.print("Publish: ");
      Serial.println(msg);

      client.publish("patvaz/fim/1/tempb", msg);
    } else {
      Serial.println("خطا در خواندن سنسور LM75");
    }
  }
}
