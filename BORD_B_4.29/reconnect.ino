void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect("esp32_client", mqtt_user, mqtt_password)) {
      Serial.println(" connected ✔️");
      client.subscribe("patvaz/fim/1/ev");
      client.subscribe("patvaz/fim/1/r2");
    } else {
      Serial.print(" failed ❌, rc=");
      Serial.print(client.state());
      Serial.println(" → retrying in 2s");
      delay(2000);
    }
  }
}