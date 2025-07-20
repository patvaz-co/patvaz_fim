void saveWMToEEPROM() {
  for (int i = 0; i < 6; i++) {
    int base = 5 + i * 4;
    EEPROM.put(base, wmCount[i]);
  }
  EEPROM.commit();
}