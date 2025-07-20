void applyRelayStates() {
  for (int i = 0; i < 6; i++) {
    pcf[i].write(RELAY, relayState[i] == 1 ? LOW : HIGH);
  }
}