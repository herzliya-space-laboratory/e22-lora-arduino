#include "E22.h"

namespace {

constexpr uint32_t kModeSwitchMs = 12;    // manual: 9..11 ms per mode switch
constexpr uint32_t kStartupMs = 30;       // manual: about 16 ms from power-on

}  // namespace

// ---------------------------------------------------------------------------
// E22Config

void E22Config::toBytes(uint8_t out[kBytes]) const {
  out[0] = (uint8_t)(address >> 8);
  out[1] = (uint8_t)(address & 0xFF);
  out[2] = netId;
  out[3] = (uint8_t)(((uint8_t)uartBaud & 0x07) << 5) |
           (uint8_t)(((uint8_t)parity & 0x03) << 3) |
           (uint8_t)((uint8_t)airRate & 0x07);
  out[4] = (uint8_t)(((uint8_t)packetSize & 0x03) << 6) |
           (uint8_t)(ambientRssi ? 0x20 : 0) |
           (uint8_t)(softwareModeSwitch ? 0x04 : 0) |
           (uint8_t)((uint8_t)txPower & 0x03);
  out[5] = channel;
  out[6] = (uint8_t)(rssiByte ? 0x80 : 0) |
           (uint8_t)(fixedPoint ? 0x40 : 0) |
           (uint8_t)(relay ? 0x20 : 0) |
           (uint8_t)(lbt ? 0x10 : 0) |
           (uint8_t)(worTransmitter ? 0x08 : 0) |
           (uint8_t)(worCycle & 0x07);
  out[7] = (uint8_t)(cryptKey >> 8);
  out[8] = (uint8_t)(cryptKey & 0xFF);
}

E22Config E22Config::fromBytes(const uint8_t in[kBytes]) {
  E22Config c;
  c.address = (uint16_t)((in[0] << 8) | in[1]);
  c.netId = in[2];
  c.uartBaud = (E22UartBaud)((in[3] >> 5) & 0x07);
  uint8_t par = (in[3] >> 3) & 0x03;
  c.parity = (par == 3) ? E22Parity::N8 : (E22Parity)par;  // 0b11 == 8N1
  c.airRate = (E22AirRate)(in[3] & 0x07);
  c.packetSize = (E22PacketSize)((in[4] >> 6) & 0x03);
  c.ambientRssi = in[4] & 0x20;
  c.softwareModeSwitch = in[4] & 0x04;
  c.txPower = (E22TxPower)(in[4] & 0x03);
  c.channel = in[5];
  c.rssiByte = in[6] & 0x80;
  c.fixedPoint = in[6] & 0x40;
  c.relay = in[6] & 0x20;
  c.lbt = in[6] & 0x10;
  c.worTransmitter = in[6] & 0x08;
  c.worCycle = in[6] & 0x07;
  c.cryptKey = 0;  // write-only register, always reads as 0
  return c;
}

uint16_t E22Config::packetSizeBytes() const {
  switch (packetSize) {
    case E22PacketSize::B128: return 128;
    case E22PacketSize::B64:  return 64;
    case E22PacketSize::B32:  return 32;
    default:                  return 240;
  }
}

uint32_t E22Config::uartBaudValue() const {
  static const uint32_t table[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  return table[(uint8_t)uartBaud & 0x07];
}

// ---------------------------------------------------------------------------
// E22

E22::E22(HardwareSerial& uart, int8_t pinM0, int8_t pinM1, int8_t pinAux,
         int8_t pinReset)
    : uart_(uart), pinM0_(pinM0), pinM1_(pinM1), pinAux_(pinAux), pinReset_(pinReset) {}

bool E22::begin(uint32_t uartBaud, int8_t rxPin, int8_t txPin) {
  uartBaud_ = uartBaud;
  rxPin_ = rxPin;
  txPin_ = txPin;

  if (pinM0_ >= 0) pinMode(pinM0_, OUTPUT);
  if (pinM1_ >= 0) pinMode(pinM1_, OUTPUT);
  if (pinAux_ >= 0) pinMode(pinAux_, INPUT);
  if (pinReset_ >= 0) {
    pinMode(pinReset_, OUTPUT);
    digitalWrite(pinReset_, HIGH);
  }

  driveModePins(E22Mode::Transmission);
  mode_ = E22Mode::Transmission;
  openSerial(uartBaud_);

  delay(kStartupMs);
  return waitIdle();
}

void E22::openSerial(uint32_t baud) {
  uart_.end();
#if defined(ESP32)
  uart_.begin(baud, SERIAL_8N1, rxPin_, txPin_);
#else
  uart_.begin(baud);
#endif
  uart_.setTimeout(20);
}

void E22::driveModePins(E22Mode mode) {
  uint8_t m = (uint8_t)mode;
  if (pinM0_ >= 0) digitalWrite(pinM0_, (m & 0x01) ? HIGH : LOW);
  if (pinM1_ >= 0) digitalWrite(pinM1_, (m & 0x02) ? HIGH : LOW);
}

bool E22::isIdle() const {
  if (pinAux_ < 0) return true;
  return digitalRead(pinAux_) == HIGH;
}

bool E22::waitIdle(uint32_t timeoutMs) {
  if (pinAux_ < 0) {
    delay(kModeSwitchMs);
    return true;
  }
  uint32_t start = millis();
  while (!isIdle()) {
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

