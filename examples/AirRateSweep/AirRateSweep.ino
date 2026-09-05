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

constexpr int8_t PIN_M0 = 32;
constexpr int8_t PIN_M1 = 33;
constexpr int8_t PIN_AUX = 34;
constexpr int8_t PIN_RESET = 27;
constexpr int8_t PIN_RX = 16;
constexpr int8_t PIN_TX = 17;

E22 radio(Serial2, PIN_M0, PIN_M1, PIN_AUX, PIN_RESET);

const E22AirRate RATES[] = {
  E22AirRate::Rate2k4, E22AirRate::Rate4k8, E22AirRate::Rate9k6,
  E22AirRate::Rate19k2, E22AirRate::Rate38k4, E22AirRate::Rate62k5,
};
const char* RATE_NAMES[] = {"2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"};
constexpr size_t RATE_COUNT = sizeof RATES / sizeof RATES[0];

size_t currentRate = RATE_COUNT;  // forces a rate change in the first loop()
uint32_t t0 = 0;
uint32_t lastBeacon = 0;
uint32_t seq = 0;

char line[128];
size_t lineLen = 0;
bool awaitingRssi = false;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(ROLE_SENDER ? "\nE22 AirRateSweep: sender" : "\nE22 AirRateSweep: receiver");

  if (!radio.begin(9600, PIN_RX, PIN_TX)) Serial.println("begin() failed");

  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    Serial.println("readConfig() failed");
    return;
  }
  cfg.channel = 23;
  cfg.address = 0x0000;
  cfg.netId = 0;
  cfg.rssiByte = true;
  cfg.airRate = RATES[0];
  if (!radio.writeConfig(cfg)) Serial.println("writeConfig() failed");
  t0 = millis();
}

void pumpReceive() {
  int b;
  while ((b = radio.readByte()) >= 0) {
    if (awaitingRssi) {
      awaitingRssi = false;
      line[lineLen] = '\0';
      Serial.printf("[%s] rx \"%s\" rssi %d dBm\n", RATE_NAMES[currentRate], line,
                    E22::rssiByteToDbm((uint8_t)b));
      lineLen = 0;
      continue;
    }
    if (b == '\n') { awaitingRssi = true; continue; }
    if (lineLen + 1 < sizeof line) line[lineLen++] = (char)b;
  }
}

void loop() {
  size_t slot = ((millis() - t0) / DWELL_MS) % RATE_COUNT;
  if (slot != currentRate) {
    currentRate = slot;
    lineLen = 0;
    awaitingRssi = false;
    bool ok = radio.setAirRate(RATES[slot], /*persist=*/false);
    Serial.printf("== air rate %s %s\n", RATE_NAMES[slot], ok ? "" : "(set failed)");
  }

  pumpReceive();

#if ROLE_SENDER
  if (millis() - lastBeacon >= BEACON_MS) {
    lastBeacon = millis();
    char msg[48];
    snprintf(msg, sizeof msg, "BCN seq=%lu rate=%s\n", (unsigned long)seq++, RATE_NAMES[currentRate]);
    radio.send(msg);
  }
#endif
}
