// Two boards, same sketch. Build one with ROLE_SENDER 1 and the other with 0.
// The sender sends "PING n" every two seconds, the other board answers
// "PONG n". Both print what they receive and the RSSI.
//
// Messages are text ending in '\n'. The module does not keep packet
// boundaries on the serial side, so we collect bytes until the newline.

#include <E22.h>

#define ROLE_SENDER 1

constexpr int8_t PIN_M0 = 32;
constexpr int8_t PIN_M1 = 33;
constexpr int8_t PIN_AUX = 34;
constexpr int8_t PIN_RESET = 27;
constexpr int8_t PIN_RX = 16;
constexpr int8_t PIN_TX = 17;

E22 radio(Serial2, PIN_M0, PIN_M1, PIN_AUX, PIN_RESET);

uint32_t counter = 0;
uint32_t lastSend = 0;

char line[128];
size_t lineLen = 0;
bool awaitingRssi = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(ROLE_SENDER ? "\nE22 PingPong: sender" : "\nE22 PingPong: responder");

  if (!radio.begin(9600, PIN_RX, PIN_TX)) Serial.println("begin() failed");

  // Same channel and address on both ends, RSSI byte on.
  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    Serial.println("readConfig() failed");
    return;
  }
  cfg.channel = 23;          // 433.125 MHz
  cfg.address = 0x0000;
  cfg.netId = 0;
  cfg.airRate = E22AirRate::Rate2k4;
  cfg.rssiByte = true;
  if (!radio.writeConfig(cfg)) Serial.println("writeConfig() failed");
}

void handleLine(const char* text, int rssiDbm) {
  Serial.printf("rx  \"%s\"  rssi %d dBm\n", text, rssiDbm);
#if !ROLE_SENDER
  if (strncmp(text, "PING ", 5) == 0) {
    char reply[32];
    snprintf(reply, sizeof reply, "PONG %s\n", text + 5);
    radio.send(reply);
    Serial.printf("tx  \"PONG %s\"\n", text + 5);
  }
#endif
}

void pumpReceive() {
  int b;
  while ((b = radio.readByte()) >= 0) {
    if (awaitingRssi) {
      awaitingRssi = false;
      line[lineLen] = '\0';
      handleLine(line, E22::rssiByteToDbm((uint8_t)b));
      lineLen = 0;
      continue;
    }
    if (b == '\n') {
      awaitingRssi = true;  // the RSSI byte comes after the packet
      continue;
    }
    if (lineLen + 1 < sizeof line) line[lineLen++] = (char)b;
  }
}

void loop() {
  pumpReceive();
#if ROLE_SENDER
  if (millis() - lastSend >= 2000) {
    lastSend = millis();
    char msg[32];
    snprintf(msg, sizeof msg, "PING %lu\n", (unsigned long)counter++);
    radio.send(msg);
    Serial.printf("tx  \"PING %lu\"\n", (unsigned long)(counter - 1));
  }
#endif
}
