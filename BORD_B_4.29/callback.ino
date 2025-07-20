void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "patvaz/fim/1/ev") {
    int id = (msg.toInt() / 10) - 1;
    int dir = msg.toInt() % 10;
    if (id >= 0 && id < 6 && (dir == 0 || dir == 1)) {
      pcf[id].write(MOTOR_L, dir == 0 ? LOW : HIGH);
      pcf[id].write(MOTOR_R, dir == 1 ? LOW : HIGH);
      motorStart[id] = millis();
      motorRunning[id] = true;
      currentDir[id] = dir;
      sendRS485("ev=" + msg);
    }
  }

  if (String(topic) == "patvaz/fim/1/r2") {
    int cmd = msg.toInt();
    int relayIndex = (cmd / 10) - 1;
    int state = cmd % 10;
    if (relayIndex >= 0 && relayIndex < 6) {
      pcf[relayIndex].write(RELAY, state == 1 ? LOW : HIGH);
      relayState[relayIndex] = state;
      saveRelayToEEPROM(relayIndex, state);
      sendRS485("r2=" + msg);
    }
  }
}