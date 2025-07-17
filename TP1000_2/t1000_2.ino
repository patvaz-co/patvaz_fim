#define ADC_PIN 34

void setup() {
  Serial.begin(115200);
}

void loop() {
  const float Vref = 1.40;         // تقسیم ولتاژ R1 و R2
  const float Vcc = 3.29;           // تغذیه
  const float Gain = 2.0;          // گین آپ‌امپ
  const float R_fixed = 1000.0;    // R3 = 1kΩ

  // خواندن ADC
  int adcVal = analogRead(ADC_PIN);
  float Vout = adcVal * Vcc / 4095.0;

  // ولتاژ ورودی از روی Vout و گین
  float V_pt1000 = (Vout / Gain) + Vref;

  // تقسیم ولتاژ بین R3 و PT1000 → Vpt = (R_pt / (R_pt + R_fixed)) * Vcc
  // حل شده:
  float R_pt1000 = (V_pt1000 * R_fixed) / (Vcc - V_pt1000);

  // تخمین دما:
  float temperature = (R_pt1000 - 1000.0) / 3.85;

  Serial.print("ADC: ");
  Serial.print(adcVal);
  Serial.print("\t Vout: ");
  Serial.print(Vout, 3);
  Serial.print(" V\t Vin: ");
  Serial.print(V_pt1000, 3);
  Serial.print(" V\t R_pt1000: ");
  Serial.print(R_pt1000, 1);
  Serial.print(" Ω\t Temp: ");
  Serial.print(temperature, 1);
  Serial.println(" °C");

  delay(1000);
}
