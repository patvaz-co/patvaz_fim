#define ADC_PIN_TEMP 34
#define ADC_PIN_EC   35

const float Vref = 1.41;         // برای تقویت‌کننده دما
const float Vcc = 3.28;          // ولتاژ تغذیه واقعی
const float Gain = 2.0;          // گین مدار دما
const float R_fixed = 1000.0;    // مقاومت سری با PT1000
const float R_pull = 1000.0;     // مقاومت pull-down برای EC

// ضرایب کالیبراسیون EC (مثال: محلول 500 و 1500 µS/cm)
float slope = 1666.67;
float offset = -700;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // دقت 12 بیت برای ESP32
}

void loop() {
  // ========= بخش دما =========
  int adcValTemp = analogRead(ADC_PIN_TEMP);
  float VoutTemp = adcValTemp * Vcc / 4095.0;
  float V_pt1000 = (VoutTemp / Gain) + Vref;
  float R_pt1000 = (V_pt1000 * R_fixed) / (Vcc - V_pt1000);
  float temperature = (R_pt1000 - 1000.0) / 3.85;

  // ========= بخش EC =========
  int adcValEC = analogRead(ADC_PIN_EC);
  float voltageEC = adcValEC * Vcc / 4095.0;
  float ec = slope * voltageEC + offset;
  float tds = ec * 0.5;

  // ====== جلوگیری از منفی بودن ======
  if (ec < 0) ec = 0;
  if (tds < 0) tds = 0;

  // ========= نمایش =========
  Serial.println("=============");
  Serial.print("TEMP_ADC: "); Serial.print(adcValTemp);
  Serial.print("\tVout: "); Serial.print(VoutTemp, 3);
  Serial.print(" V\tVin: "); Serial.print(V_pt1000, 3);
  Serial.print(" V\tR_pt1000: "); Serial.print(R_pt1000, 1);
  Serial.print(" Ω\tTemp: "); Serial.print(temperature, 1);
  Serial.println(" °C");

  Serial.print("EC_ADC: "); Serial.print(adcValEC);
  Serial.print("\tVoltage: "); Serial.print(voltageEC, 3); Serial.print(" V");
  Serial.print("\tEC: "); Serial.print(ec, 2); Serial.print(" µS/cm");
  Serial.print("\tTDS: "); Serial.print(tds, 2); Serial.println(" ppm");

  delay(1000);
}
