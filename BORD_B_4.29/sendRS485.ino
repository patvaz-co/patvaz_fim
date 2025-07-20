void sendRS485(String message) {
  Serial.println("RS485 → " + message);
  digitalWrite(RS485_EN_PIN, HIGH);
  delayMicroseconds(100);
  RS485.println(message);
  RS485.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_EN_PIN, LOW);
}