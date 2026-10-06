// Two boards, same sketch. Build one with ROLE_SENDER 1 and the other with 0
// (edit the line below, or pass -DROLE_SENDER=0 from the command line).
// The sender sends "PING n" every two seconds, the other board answers
// "PONG n". Both print what they receive and the RSSI.
//
// Messages are text ending in '\n'. The module does not keep packet
// boundaries on the serial side, so we collect bytes until the newline.

#include <E22.h>

#ifndef ROLE_SENDER
#define ROLE_SENDER 1
#endif

#if defined(ESP32)
#define RADIO_SERIAL Serial2
constexpr int8_t PIN_M0 = 32, PIN_M1 = 33, PIN_AUX = 34, PIN_RESET = 27;
constexpr int8_t PIN_RX = 16, PIN_TX = 17;
#elif defined(__AVR_ATmega328PB__)
// Flight board (ISC), module on J3: PD2 <- TXD, PD3 -> RXD through a divider,
// PB0 -> M0, PE3 -> M1. PD2/PD3 are not a hardware UART, so SoftwareSerial.
// AUX is not wired to the MCU, so the driver uses fixed delays.
#include <SoftwareSerial.h>
SoftwareSerial radioSerial(2, 3);  // RX, TX
#define RADIO_SERIAL radioSerial
constexpr int8_t PIN_M0 = 8, PIN_M1 = 26, PIN_AUX = -1, PIN_RESET = -1;  // 26 = PE3
constexpr int8_t PIN_RX = -1, PIN_TX = -1;
#else
// Boards with one UART (Arduino Uno). Serial stays free for the debug prints
// and the radio goes on SoftwareSerial: D10 <- TXD, D11 -> RXD.
#include <SoftwareSerial.h>
SoftwareSerial radioSerial(10, 11);  // RX, TX
#define RADIO_SERIAL radioSerial
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
  // Lowest power. Two modules a metre apart at 30 dBm overload each other's
  // receiver, and 30 dBm draws more current than a USB-powered Uno can give.
  cfg.txPower = E22TxPower::Level3;
  // Temporary (C2): the module's flash keeps its own settings.
  if (!radio.writeConfig(cfg, false)) DEBUG.println(F("writeConfig() failed"));
}

void handleLine(const char* text, int rssiDbm) {
  DEBUG.print(F("rx  \"")); DEBUG.print(text);
  DEBUG.print(F("\"  rssi "));
  if (rssiDbm == 0) {
    DEBUG.println(F("n/a"));  // no valid reading, see E22::rssiByteToDbm
  } else {
    DEBUG.print(rssiDbm); DEBUG.println(F(" dBm"));
  }
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
    if ((size_t)lineLen + 1 < sizeof line) line[lineLen++] = (char)b;
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
