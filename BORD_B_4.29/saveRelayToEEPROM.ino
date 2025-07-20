
void saveRelayToEEPROM(int relayIndex, byte state) {
  int addr = 25 + relayIndex;
  EEPROM.write(addr, state);
  EEPROM.commit();
}