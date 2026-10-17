// One station of the satellite network: a satellite, or the ground station.
// Same sketch for both; build the ground station with -DROLE_GROUND=1.
//
// A satellite sends a beacon every 30 s, relays every message it hears for
// the first time (hop limit, see Node.h), and answers messages addressed to
// it with an ACK. The ground station does not beacon or relay. It sends what
// you type and prints what it hears. Frame format: Frame.h.
//
// Debug console on Serial at 115200, lines ending in newline:
//   msg ALL hello       message to every station
//   msg 4X0SAT hello    message to one station, it answers with an ACK
//   call 4X1ABC         set this station's callsign (kept in EEPROM)
//   status
//
// Nothing here waits forever: if the radio does not answer, the sketch tries
// again every 5 s, and on AVR the watchdog resets the MCU if loop() stops.

#include <E22.h>
#include <EEPROM.h>

#include "Frame.h"
#include "Node.h"

#if defined(__AVR__)
#include <avr/wdt.h>
#endif

#ifndef ROLE_GROUND
#define ROLE_GROUND 0
#endif
// 21 dBm by default (bench, modules close together, USB power). 1 = 30 dBm.
#ifndef FULL_POWER
#define FULL_POWER 0
#endif

#if defined(ESP32)
#define RADIO_SERIAL Serial2
constexpr int8_t PIN_M0 = 32, PIN_M1 = 33, PIN_AUX = 34, PIN_RESET = 27;
constexpr int8_t PIN_RX = 16, PIN_TX = 17;
#elif defined(__AVR_ATmega328PB__)
// Flight board (ISC), module on J3: PD2 <- TXD, PD3 -> RXD through a divider,
// PB0 -> M0, PE3 -> M1. AUX is not wired to the MCU.
#include <SoftwareSerial.h>
SoftwareSerial radioSerial(2, 3);  // RX, TX
#define RADIO_SERIAL radioSerial
constexpr int8_t PIN_M0 = 8, PIN_M1 = 26, PIN_AUX = -1, PIN_RESET = -1;  // 26 = PE3
constexpr int8_t PIN_RX = -1, PIN_TX = -1;
#else
// Arduino Uno bench wiring: D10 <- TXD, D11 -> RXD, M0 = D4, M1 = D5, AUX = D6.
#include <SoftwareSerial.h>
SoftwareSerial radioSerial(10, 11);  // RX, TX
#define RADIO_SERIAL radioSerial
constexpr int8_t PIN_M0 = 4, PIN_M1 = 5, PIN_AUX = 6, PIN_RESET = -1;
constexpr int8_t PIN_RX = -1, PIN_TX = -1;
#endif
#define DEBUG Serial

// Radio settings, the same on every station.
constexpr uint8_t kChannel = 23;  // 433.125 MHz. 25..28 would disturb the main radio on 436.4
constexpr E22AirRate kAirRate = E22AirRate::Rate2k4;  // slowest = longest range
constexpr E22TxPower kTxPower = FULL_POWER ? E22TxPower::dBm30 : E22TxPower::dBm21;

constexpr uint32_t kRadioRetryMs = 5000;
constexpr uint32_t kConfigCheckMs = 600000;  // re-check the module's settings every 10 min
constexpr uint32_t kQuietMs = 30;           // no transmitting until the input has been quiet this long

// EEPROM: magic, boot count, callsign.
constexpr int kEeMagic = 0, kEeBoot = 1, kEeCall = 3;
constexpr uint8_t kMagic = 0x5A;

E22 radio(RADIO_SERIAL, PIN_M0, PIN_M1, PIN_AUX, PIN_RESET);
sat::Parser parser;
sat::Node node;

bool radioUp = false;
uint32_t lastRadioTry = 0;
uint32_t lastConfigCheck = 0;
uint32_t lastRxByte = 0;

char line[64];
uint8_t lineLen = 0;

