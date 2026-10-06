// Reads the module registers and the firmware version and prints them.
// Run this first on a new board. If the values are correct, the wiring and
// the baud rate are correct.
//
// The pins for each target are in the block below: the flight board
// (ATmega328PB, module on J3, pins from the schematic), an Arduino Uno and an
// ESP32 on the bench. On the ESP32: GPIO17 -> RXD, GPIO16 <- TXD,
// GPIO32 -> M0, GPIO33 -> M1, GPIO34 <- AUX, GPIO27 -> RESET if the module
// has one (the DIP module does not).

#include <E22.h>

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

static const char* airRateName(E22AirRate r) {
  switch (r) {
    case E22AirRate::Rate4k8:  return "4.8 kbps";
    case E22AirRate::Rate9k6:  return "9.6 kbps";
    case E22AirRate::Rate19k2: return "19.2 kbps";
    case E22AirRate::Rate38k4: return "38.4 kbps";
    case E22AirRate::Rate62k5: return "62.5 kbps";
    default:                   return "2.4 kbps";
  }
}

static void printRow(const char* label, const char* value) {
  DEBUG.print(label);
  DEBUG.println(value);
}

static void printRow(const char* label, unsigned long value) {
  DEBUG.print(label);
  DEBUG.println(value);
}

void setup() {
  DEBUG.begin(115200);
  delay(500);
  DEBUG.println(F("\nE22 ReadConfig"));

  if (!radio.begin(9600, PIN_RX, PIN_TX)) {
    DEBUG.println(F("begin() failed: AUX never went idle. Check wiring and power."));
  }

  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    DEBUG.println(F("readConfig() failed. Check M0/M1 wiring and that the UART is 9600."));
    return;
  }

  DEBUG.print(F("address      0x")); DEBUG.println(cfg.address, HEX);
  printRow("netId        ", (unsigned long)cfg.netId);
  printRow("uart baud    ", cfg.uartBaudValue());
  printRow("air rate     ", airRateName(cfg.airRate));
  printRow("packet size  ", (unsigned long)cfg.packetSizeBytes());
  DEBUG.print(F("tx power     level ")); DEBUG.print((uint8_t)cfg.txPower);
  DEBUG.println(F("  (0 = max: 30 dBm on T30, 22 dBm on T22)"));
  DEBUG.print(F("channel      ")); DEBUG.print(cfg.channel);
  DEBUG.print(F("  (")); DEBUG.print(cfg.frequencyKHz() / 1000);
  DEBUG.print('.'); DEBUG.print(cfg.frequencyKHz() % 1000); DEBUG.println(F(" MHz)"));
  printRow("rssi byte    ", cfg.rssiByte ? "on" : "off");
  printRow("ambient rssi ", cfg.ambientRssi ? "on" : "off");
  printRow("fixed point  ", cfg.fixedPoint ? "on" : "off");
  printRow("relay        ", cfg.relay ? "on" : "off");
  printRow("lbt          ", cfg.lbt ? "on" : "off");

  char fw[32];
  if (radio.readFirmwareVersion(fw, sizeof fw)) {
    printRow("firmware     ", fw);
  } else {
    DEBUG.println(F("firmware     (AT+FWCODE not answered; old firmware or wiring)"));
  }
}

void loop() {}
