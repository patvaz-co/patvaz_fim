
void loadFromEEPROM() {
  for (int i = 0; i < 6; i++) relayState[i] = EEPROM.read(25 + i);
  for (int i = 0; i < 6; i++) EEPROM.get(5 + i * 4, wmCount[i]);
}