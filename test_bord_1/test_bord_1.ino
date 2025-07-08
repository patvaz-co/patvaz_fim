
// این کد توسط afshar نوشته و آپلود شده 


#include <Wire.h>
#include <PCF8574.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// WiFi و MQTT
const char* ssid = "patvaz";
const char* password = "qazvin402";
const char* mqtt_server = "37.148.56.88";
const char* hostname = "relay_ctrl_esp32";

WiFiClient espClient;
PubSubClient client(espClient);

// PCF8574
PCF8574 pcfRelay(0x27); // رله‌ها
PCF8574 pcfKeys(0x26);  // کلیدها

int lastKeyStates[8];  // ذخیره آخرین وضعیت کلیدها

// اتصال WiFi
void setup_wifi() {
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
  Serial.print("🔌 Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected!");
  Serial.print("📡 IP: ");
  Serial.println(WiFi.localIP());
}

// تنظیم OTA
void setup_ota() {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    Serial.println("🔁 OTA Update Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ OTA Update Finished");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("🔄 OTA Progress: %u%%\r", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("❌ OTA Error[%u]: ", error);
  });
  ArduinoOTA.begin();
  Serial.println("✅ OTA Ready");
}

// اتصال MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("🔁 Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("✅ MQTT connected");
      client.subscribe("patvaz/fim/1/r");
    } else {
      Serial.print("❌ MQTT failed, rc=");
      Serial.print(client.state());
      Serial.println(" → retrying...");
      delay(2000);
    }
  }
}

// دریافت پیام MQTT
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("📨 MQTT [%s] => %s\n", topic, msg.c_str());

  int val = msg.toInt();
  if (val >= 10 && val <= 81 && val % 10 <= 1) {
    int relay = val / 10 - 1;
    int state = val % 10;
    pcfRelay.write(relay, state == 1 ? LOW : HIGH); // اکتیو-لو

    Serial.printf("🔧 Relay %d set to: %s\n", relay + 1, (state == 1 ? "ON" : "OFF"));
  }
}

// ارسال وضعیت کلید
void publishKeyState(int keyIndex, int state) {
  String topic = "patvaz/fim/1/di" + String(keyIndex + 1);
  int value = (keyIndex + 1) * 10 + (state == LOW ? 1 : 0);
  client.publish(topic.c_str(), String(value).c_str());

  Serial.printf("📤 Key %d changed → %d → Sent to %s\n",
                keyIndex + 1, value, topic.c_str());
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Serial.println("🚀 Starting relay + key system...");

  pcfRelay.begin();
  pcfKeys.begin();

  for (int i = 0; i < 8; i++) {
    pcfRelay.write(i, HIGH); // خاموش (اکتیو-لو)
    lastKeyStates[i] = pcfKeys.read(i);
  }

  setup_wifi();
  setup_ota();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);
  reconnect();
}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) reconnect();
  client.loop();

  for (int i = 0; i < 8; i++) {
    int current = pcfKeys.read(i);
    if (current != lastKeyStates[i]) {
      lastKeyStates[i] = current;
      publishKeyState(i, current);
    }
  }

  delay(100);
}
