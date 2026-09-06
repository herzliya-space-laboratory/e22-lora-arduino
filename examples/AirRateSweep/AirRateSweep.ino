// The "change the SF every 10 seconds" experiment from the project document.
// The module does not allow setting the SF directly, so we change the air
// rate instead. The effect is the same.
//
// Both boards run this sketch and go through the six rates in the same
// order, DWELL_MS each, counting from boot. Reset both boards at the same
// time so they stay in sync. The sender sends a beacon once per second, the
// receiver prints each beacon with its RSSI.
//
// Rate changes use temporary writes (C2). This does not wear the module's
// flash, and a power cycle returns the module to the saved configuration.

#include <E22.h>

#define ROLE_SENDER 1

constexpr uint32_t DWELL_MS = 10000;
constexpr uint32_t BEACON_MS = 1000;

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

const E22AirRate RATES[] = {
  E22AirRate::Rate2k4, E22AirRate::Rate4k8, E22AirRate::Rate9k6,
  E22AirRate::Rate19k2, E22AirRate::Rate38k4, E22AirRate::Rate62k5,
};
const char* const RATE_NAMES[] = {"2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"};
constexpr uint8_t RATE_COUNT = sizeof RATES / sizeof RATES[0];

uint8_t currentRate = RATE_COUNT;  // forces a rate change in the first loop()
uint32_t t0 = 0;
uint32_t lastBeacon = 0;
uint32_t seq = 0;

char line[64];
uint8_t lineLen = 0;
bool awaitingRssi = false;

void setup() {
  DEBUG.begin(115200);
  delay(500);
  DEBUG.println(ROLE_SENDER ? F("\nE22 AirRateSweep: sender") : F("\nE22 AirRateSweep: receiver"));

  if (!radio.begin(9600, PIN_RX, PIN_TX)) DEBUG.println(F("begin() failed"));

  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    DEBUG.println(F("readConfig() failed"));
    return;
  }
  cfg.channel = 23;
  cfg.address = 0x0000;
  cfg.netId = 0;
  cfg.rssiByte = true;
  cfg.airRate = RATES[0];
  if (!radio.writeConfig(cfg)) DEBUG.println(F("writeConfig() failed"));
  t0 = millis();
}

void pumpReceive() {
  int b;
  while ((b = radio.readByte()) >= 0) {
    if (awaitingRssi) {
      awaitingRssi = false;
      line[lineLen] = '\0';
      DEBUG.print('['); DEBUG.print(RATE_NAMES[currentRate]); DEBUG.print(F("] rx \""));
      DEBUG.print(line); DEBUG.print(F("\" rssi "));
      DEBUG.print(E22::rssiByteToDbm((uint8_t)b)); DEBUG.println(F(" dBm"));
      lineLen = 0;
      continue;
    }
    if (b == '\n') { awaitingRssi = true; continue; }
    if (lineLen + 1 < sizeof line) line[lineLen++] = (char)b;
  }
}

void loop() {
  uint8_t slot = (uint8_t)(((millis() - t0) / DWELL_MS) % RATE_COUNT);
  if (slot != currentRate) {
    currentRate = slot;
    lineLen = 0;
    awaitingRssi = false;
    bool ok = radio.setAirRate(RATES[slot], /*persist=*/false);
    DEBUG.print(F("== air rate ")); DEBUG.print(RATE_NAMES[slot]);
    DEBUG.println(ok ? F("") : F(" (set failed)"));
  }

  pumpReceive();

#if ROLE_SENDER
  if (millis() - lastBeacon >= BEACON_MS) {
    lastBeacon = millis();
    char msg[40];
    snprintf(msg, sizeof msg, "BCN seq=%lu rate=%s\n", (unsigned long)seq++, RATE_NAMES[currentRate]);
    radio.send(msg);
  }
#endif
}
