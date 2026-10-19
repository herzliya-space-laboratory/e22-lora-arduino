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

// ---------------------------------------------------------------------------
// Printing

void printCall(const char call[sat::kCallLen]) {
  char s[sat::kCallLen + 1];
  sat::callToString(call, s);
  DEBUG.print(s);
}

void printType(sat::FrameType t) {
  switch (t) {
    case sat::FrameType::Beacon: DEBUG.print(F("BEACON")); break;
    case sat::FrameType::Message: DEBUG.print(F("MSG")); break;
    case sat::FrameType::Ack: DEBUG.print(F("ACK")); break;
  }
}

// "MSG 4X0GND>ALL #3.12 via 4X0SAT hops 2"
void printHeader(const sat::Frame& f) {
  printType(f.type);
  DEBUG.print(' ');
  printCall(f.origin);
  DEBUG.print('>');
  printCall(f.dest);
  DEBUG.print(F(" #"));
  DEBUG.print(f.boot);
  DEBUG.print('.');
  DEBUG.print(f.seq);
  DEBUG.print(F(" via "));
  printCall(f.sender);
  DEBUG.print(F(" hops "));
  DEBUG.print(f.hops);
}

void printPayload(const sat::Frame& f) {
  DEBUG.print(F(": "));
  if (f.type == sat::FrameType::Beacon) {
    sat::BeaconInfo b;
    if (!sat::decodeBeacon(f, b)) return;
    DEBUG.print(F("n=")); DEBUG.print(b.number);
    DEBUG.print(F(" up=")); DEBUG.print(b.uptimeS);
    DEBUG.print(F("s boot=")); DEBUG.print(b.boot);
    DEBUG.print(F(" tx=")); DEBUG.print(b.txCount);
    DEBUG.print(F(" rx=")); DEBUG.print(b.rxCount);
    DEBUG.print(F(" relayed=")); DEBUG.print(b.relayed);
    DEBUG.print(F(" stored=")); DEBUG.print(b.stored);
    DEBUG.print(F(" rssi=")); DEBUG.print(b.lastRssi);
  } else if (f.type == sat::FrameType::Ack && f.len >= 4) {
    DEBUG.print(F("got #"));
    DEBUG.print(f.payload[0] | (f.payload[1] << 8));
    DEBUG.print('.');
    DEBUG.print(f.payload[2] | (f.payload[3] << 8));
  } else {
    DEBUG.print('"');
    for (uint8_t i = 0; i < f.len; i++) {
      char c = (char)f.payload[i];
      DEBUG.print(c >= 32 && c < 127 ? c : '.');
    }
    DEBUG.print('"');
  }
}

void printRx(sat::Rx rx) {
  switch (rx) {
    case sat::Rx::Invalid: DEBUG.print(F("drop (invalid)")); break;
    case sat::Rx::NotIsraeli: DEBUG.print(F("drop (not 4X/4Z)")); break;
    case sat::Rx::Own: DEBUG.print(F("drop (own)")); break;
    case sat::Rx::Duplicate: DEBUG.print(F("drop (seen)")); break;
    case sat::Rx::Beacon: DEBUG.print(F("rx")); break;
    case sat::Rx::ForUs: DEBUG.print(F("rx")); break;
    case sat::Rx::ForOthers: DEBUG.print(F("pass")); break;
    case sat::Rx::AckForUs: DEBUG.print(F("rx")); break;
    case sat::Rx::AckForOthers: DEBUG.print(F("pass")); break;
  }
}

// ---------------------------------------------------------------------------
// Radio

// Our settings, written as a temporary (C2) config: the module's flash keeps
// its own values, and we write ours again after every boot.
void applySettings(E22Config& cfg) {
  cfg.address = 0xFFFF;  // broadcast, and hear every address
  cfg.netId = 0;
  cfg.uartBaud = E22UartBaud::B9600;
  cfg.parity = E22Parity::N8;
  cfg.airRate = kAirRate;
  cfg.packetSize = E22PacketSize::B240;
  cfg.ambientRssi = false;
  cfg.txPower = kTxPower;
  cfg.channel = kChannel;
  cfg.rssiByte = true;
  cfg.fixedPoint = false;
  cfg.relay = false;  // the module's own relay mode; relaying is done here
  cfg.lbt = false;
  cfg.worTransmitter = false;
}

bool settingsMatch(const E22Config& a, const E22Config& b) {
  return a.address == b.address && a.netId == b.netId && a.airRate == b.airRate &&
         a.txPower == b.txPower && a.channel == b.channel && a.rssiByte == b.rssiByte &&
         a.fixedPoint == b.fixedPoint && a.uartBaud == b.uartBaud;
}

bool startRadio() {
  if (!radio.begin(9600, PIN_RX, PIN_TX)) {
    DEBUG.println(F("radio: begin() failed"));
    return false;
  }
  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    DEBUG.println(F("radio: readConfig() failed"));
    return false;
  }
  applySettings(cfg);
  if (!radio.writeConfig(cfg, false)) {
    DEBUG.println(F("radio: writeConfig() failed"));
    return false;
  }
  parser.reset();
  parser.setExpectRssi(cfg.rssiByte);
  DEBUG.print(F("radio: ok, "));
  DEBUG.print(cfg.frequencyKHz());
  DEBUG.println(F(" kHz"));
  return true;
}

// The module could have been reset (brown-out on a 30 dBm transmission) and
// gone back to the settings in its flash.
void checkRadio() {
  E22Config want, have;
  if (radio.readConfig(have)) {
    want = have;
    applySettings(want);
    if (settingsMatch(want, have)) return;
    DEBUG.println(F("radio: settings changed, writing them again"));
  } else {
    DEBUG.println(F("radio: no answer"));
  }
  radioUp = startRadio();
  lastRadioTry = millis();
}

void transmit(const sat::Frame& f) {
  uint8_t buf[sat::kMaxFrameLen];
  size_t n = sat::encode(f, buf, sizeof buf);
  if (n == 0) return;
  size_t sent = radio.send(buf, n);
  DEBUG.print(sent == n ? F("tx ") : F("tx FAILED "));
  printHeader(f);
  printPayload(f);
  DEBUG.println();
}

void handleFrame() {
  const sat::Frame& f = parser.frame();
  int rssi = E22::rssiByteToDbm(parser.rssiByte());
  sat::Rx rx = node.onReceive(f, (int8_t)(rssi < -128 ? -128 : rssi), millis());
  printRx(rx);
  DEBUG.print(' ');
  printHeader(f);
  DEBUG.print(F(" rssi "));
  if (rssi == 0) DEBUG.print(F("n/a"));
  else DEBUG.print(rssi);
  if (rx != sat::Rx::Duplicate && rx != sat::Rx::Own) printPayload(f);
  if (rx == sat::Rx::AckForUs) DEBUG.print(F(" (delivered)"));
  if (node.queued()) DEBUG.print(F(" (relaying)"));
  DEBUG.println();
}

// ---------------------------------------------------------------------------
// EEPROM and console

uint16_t loadIdentity(char call[sat::kCallLen]) {
  uint16_t boot = 0;
  bool ok = EEPROM.read(kEeMagic) == kMagic;
  if (ok) {
    EEPROM.get(kEeBoot, boot);
    for (uint8_t i = 0; i < sat::kCallLen; i++) call[i] = (char)EEPROM.read(kEeCall + i);
    ok = sat::isStation(call);
  }
  if (!ok) {
    // Placeholder callsigns. Set the real ones with the "call" command.
    sat::setCall(call, ROLE_GROUND ? "4X0GND" : "4X0SAT");
    for (uint8_t i = 0; i < sat::kCallLen; i++) EEPROM.write(kEeCall + i, (uint8_t)call[i]);
    EEPROM.write(kEeMagic, kMagic);
    boot = 0;
  }
  boot++;
  EEPROM.put(kEeBoot, boot);
#if defined(ESP32)
  EEPROM.commit();
#endif
  return boot;
}

void saveCall(const char call[sat::kCallLen]) {
  for (uint8_t i = 0; i < sat::kCallLen; i++) EEPROM.write(kEeCall + i, (uint8_t)call[i]);
#if defined(ESP32)
  EEPROM.commit();
#endif
}

void printStatus() {
  DEBUG.print(F("station "));
  printCall(node.call());
  DEBUG.print(ROLE_GROUND ? F(" (ground)") : F(" (satellite)"));
  DEBUG.print(F(" boot ")); DEBUG.print(node.boot());
  DEBUG.print(F(" radio ")); DEBUG.print(radioUp ? F("ok") : F("DOWN"));
  DEBUG.print(F(" tx ")); DEBUG.print(node.txCount());
  DEBUG.print(F(" rx ")); DEBUG.print(node.rxCount());
  DEBUG.print(F(" relayed ")); DEBUG.print(node.relayed());
  DEBUG.print(F(" seen-again ")); DEBUG.print(node.duplicates());
  DEBUG.print(F(" bad-frames ")); DEBUG.print(parser.dropped());
  DEBUG.print(F(" stored ")); DEBUG.println(node.stored());
}

