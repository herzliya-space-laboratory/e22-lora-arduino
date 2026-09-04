// Reads the module registers and the firmware version and prints them.
// Run this first on a new board. If the values are correct, the wiring and
// the baud rate are correct.
//
// The pins below are for an ESP32: GPIO17 -> RXD, GPIO16 <- TXD,
// GPIO32 -> M0, GPIO33 -> M1, GPIO34 <- AUX (an input-only pin is fine
// here), GPIO27 -> RESET (optional, -1 if not connected). Change them to
// match your board.

#include <E22.h>

constexpr int8_t PIN_M0 = 32;
constexpr int8_t PIN_M1 = 33;
constexpr int8_t PIN_AUX = 34;
constexpr int8_t PIN_RESET = 27;
constexpr int8_t PIN_RX = 16;  // ESP32 pin connected to E22 TXD
constexpr int8_t PIN_TX = 17;  // ESP32 pin connected to E22 RXD

E22 radio(Serial2, PIN_M0, PIN_M1, PIN_AUX, PIN_RESET);

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

static const char* txPowerName(E22TxPower p) {
  switch (p) {
    case E22TxPower::dBm17: return "17 dBm";
    case E22TxPower::dBm14: return "14 dBm";
    case E22TxPower::dBm10: return "10 dBm";
    default:                return "22 dBm";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nE22 ReadConfig");

  if (!radio.begin(9600, PIN_RX, PIN_TX)) {
    Serial.println("begin() failed: AUX never went idle. Check wiring and power.");
  }

  E22Config cfg;
  if (!radio.readConfig(cfg)) {
    Serial.println("readConfig() failed. Check M0/M1 wiring and that the UART is 9600.");
    return;
  }

  Serial.printf("address      0x%04X\n", cfg.address);
  Serial.printf("netId        %u\n", cfg.netId);
  Serial.printf("uart         %lu baud, parity %u\n", (unsigned long)cfg.uartBaudValue(), (unsigned)cfg.parity);
  Serial.printf("air rate     %s\n", airRateName(cfg.airRate));
  Serial.printf("packet size  %u bytes\n", cfg.packetSizeBytes());
  Serial.printf("tx power     %s\n", txPowerName(cfg.txPower));
  Serial.printf("channel      %u  (%lu.%03lu MHz)\n", cfg.channel,
                (unsigned long)(cfg.frequencyKHz() / 1000), (unsigned long)(cfg.frequencyKHz() % 1000));
  Serial.printf("rssi byte    %s\n", cfg.rssiByte ? "on" : "off");
  Serial.printf("ambient rssi %s\n", cfg.ambientRssi ? "on" : "off");
  Serial.printf("fixed point  %s\n", cfg.fixedPoint ? "on" : "off");
  Serial.printf("relay        %s\n", cfg.relay ? "on" : "off");
  Serial.printf("lbt          %s\n", cfg.lbt ? "on" : "off");

  char fw[32];
  if (radio.readFirmwareVersion(fw, sizeof fw)) {
    Serial.printf("firmware     %s\n", fw);
  } else {
    Serial.println("firmware     (AT+FWCODE not answered; old firmware or wiring)");
  }
}

void loop() {}
