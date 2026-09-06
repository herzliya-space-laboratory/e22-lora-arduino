// Two boards, same sketch. Build one with ROLE_SENDER 1 and the other with 0.
// The sender sends "PING n" every two seconds, the other board answers
// "PONG n". Both print what they receive and the RSSI.
//
// Messages are text ending in '\n'. The module does not keep packet
// boundaries on the serial side, so we collect bytes until the newline.

#include <E22.h>

#define ROLE_SENDER 1

#if defined(ESP32)
#define RADIO_SERIAL Serial2
constexpr int8_t PIN_M0 = 32, PIN_M1 = 33, PIN_AUX = 34, PIN_RESET = 27;
constexpr int8_t PIN_RX = 16, PIN_TX = 17;
#elif defined(HAVE_HWSERIAL1)
#define RADIO_SERIAL Serial1  // ATmega328PB USART1: PB4 = RXD1, PB3 = TXD1
constexpr int8_t PIN_M0 = 4, PIN_M1 = 5, PIN_AUX = 6, PIN_RESET = -1;
constexpr int8_t PIN_RX = -1, PIN_TX = -1;
#else
#define RADIO_SERIAL Serial
constexpr int8_t PIN_M0 = 4, PIN_M1 = 5, PIN_AUX = 6, PIN_RESET = -1;
constexpr int8_t PIN_RX = -1, PIN_TX = -1;
#endif
#define DEBUG Serial

E22 radio(RADIO_SERIAL, PIN_M0, PIN_M1, PIN_AUX, PIN_RESET);

uint32_t counter = 0;
uint32_t lastSend = 0;

char line[64];
uint8_t lineLen = 0;
bool awaitingRssi = false;

void setup() {
  DEBUG.begin(115200);
  delay(500);
  DEBUG.println(ROLE_SENDER ? F("\nE22 PingPong: sender") : F("\nE22 PingPong: responder"));

  if (!radio.begin(9600, PIN_RX, PIN_TX)) DEBUG.println(F("begin() failed"));

  // Same channel and address on both ends, RSSI byte on.
  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    DEBUG.println(F("readConfig() failed"));
    return;
  }
  cfg.channel = 23;          // 433.125 MHz, away from the 436.4 MHz main radio
  cfg.address = 0x0000;
  cfg.netId = 0;
  cfg.airRate = E22AirRate::Rate2k4;
  cfg.rssiByte = true;
  if (!radio.writeConfig(cfg)) DEBUG.println(F("writeConfig() failed"));
}

void handleLine(const char* text, int rssiDbm) {
  DEBUG.print(F("rx  \"")); DEBUG.print(text);
  DEBUG.print(F("\"  rssi ")); DEBUG.print(rssiDbm); DEBUG.println(F(" dBm"));
#if !ROLE_SENDER
  if (strncmp(text, "PING ", 5) == 0) {
    char reply[24];
    snprintf(reply, sizeof reply, "PONG %s\n", text + 5);
    radio.send(reply);
    DEBUG.print(F("tx  \"PONG ")); DEBUG.print(text + 5); DEBUG.println('"');
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
    char msg[24];
    snprintf(msg, sizeof msg, "PING %lu\n", (unsigned long)counter++);
    radio.send(msg);
    DEBUG.print(F("tx  \"PING ")); DEBUG.print(counter - 1); DEBUG.println('"');
  }
#endif
}
