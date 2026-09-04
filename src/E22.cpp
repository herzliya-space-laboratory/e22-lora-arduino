#include "E22.h"

namespace {

constexpr uint8_t kCmdWrite      = 0xC0;  // write registers, saved in flash
constexpr uint8_t kCmdRead       = 0xC1;  // read registers (replies start with this too)
constexpr uint8_t kCmdWriteTemp  = 0xC2;  // write registers, lost after power cycle
constexpr uint8_t kErrByte       = 0xFF;  // FF FF FF = the module rejected the command

constexpr uint32_t kModeSwitchMs = 12;    // manual: 9..11 ms per mode switch
constexpr uint32_t kStartupMs = 30;       // manual: about 16 ms from power-on
constexpr uint32_t kAtIdleGapMs = 50;     // some firmware versions send AT replies without \n

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

bool E22::setMode(E22Mode mode) {
  // The module ignores M0/M1 while AUX is low (manual 5.2.5).
  if (!waitIdle()) return false;
  if (mode == mode_) return true;

  E22Mode previous = mode_;
  driveModePins(mode);
  mode_ = mode;

  // Wait a little for AUX to go low, then wait for it to go high again.
  delay(2);
  bool ok = waitIdle();
  delay(kModeSwitchMs);

  // Configuration mode forces 9600 8N1 on the UART (manual 6.1 / 7.1.12).
  bool wasConfig = previous == E22Mode::Configuration;
  bool isConfig = mode == E22Mode::Configuration;
  if (isConfig && uartBaud_ != kConfigBaud) openSerial(kConfigBaud);
  if (wasConfig && uartBaud_ != kConfigBaud) openSerial(uartBaud_);

  flushInput();
  return ok;
}

bool E22::hardReset() {
  if (pinReset_ < 0) return false;
  digitalWrite(pinReset_, LOW);
  delay(2);  // manual: hold low > 100 us
  digitalWrite(pinReset_, HIGH);
  delay(kStartupMs);
  // The module samples M0/M1 at boot, so mode_ stays valid.
  if (mode_ == E22Mode::Configuration && uartBaud_ != kConfigBaud) openSerial(kConfigBaud);
  flushInput();
  return waitIdle();
}

// --- low-level serial helpers ------------------------------------------------

void E22::flushInput() {
  while (uart_.available() > 0) uart_.read();
}

bool E22::readBytes(uint8_t* buf, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len) {
    int b = uart_.read();
    if (b >= 0) {
      buf[got++] = (uint8_t)b;
      continue;
    }
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

// --- register protocol (mode 2) ----------------------------------------------

bool E22::enterConfig(E22Mode& previous) {
  previous = mode_;
  return setMode(E22Mode::Configuration);
}

bool E22::leaveConfig(E22Mode previous) {
  if (previous == E22Mode::Configuration) return true;
  return setMode(previous);
}

bool E22::readRegisters(uint8_t addr, uint8_t len, uint8_t* out) {
  flushInput();
  const uint8_t cmd[3] = {kCmdRead, addr, len};
  uart_.write(cmd, sizeof cmd);
  uart_.flush();

  uint8_t head[3];
  if (!readBytes(head, 3, kDefaultTimeoutMs)) return false;
  if (head[0] == kErrByte && head[1] == kErrByte && head[2] == kErrByte) return false;
  if (head[0] != kCmdRead || head[1] != addr || head[2] != len) return false;
  return readBytes(out, len, kDefaultTimeoutMs);
}

bool E22::writeRegisters(uint8_t cmd, uint8_t addr, uint8_t len, const uint8_t* data) {
  flushInput();
  const uint8_t head[3] = {cmd, addr, len};
  uart_.write(head, sizeof head);
  uart_.write(data, len);
  uart_.flush();

  // The reply is C1 addr len followed by the same values. Only the header
  // is checked here. writeConfig reads the registers back afterwards.
  uint8_t reply[3];
  if (!readBytes(reply, 3, kDefaultTimeoutMs)) return false;
  if (reply[0] == kErrByte && reply[1] == kErrByte && reply[2] == kErrByte) return false;
  if (reply[0] != kCmdRead || reply[1] != addr || reply[2] != len) return false;
  uint8_t echo[E22Config::kBytes];
  return readBytes(echo, len, kDefaultTimeoutMs);
}

bool E22::readConfig(E22Config& out) {
  E22Mode prev;
  if (!enterConfig(prev)) return false;
  uint8_t raw[E22Config::kBytes];
  bool ok = readRegisters(0x00, E22Config::kBytes, raw);
  leaveConfig(prev);
  if (!ok) return false;
  out = E22Config::fromBytes(raw);
  cfg_ = out;
  cfgKnown_ = true;
  return true;
}

bool E22::writeConfig(const E22Config& cfg, bool persist) {
  uint8_t want[E22Config::kBytes];
  cfg.toBytes(want);

  E22Mode prev;
  if (!enterConfig(prev)) return false;

  bool ok = writeRegisters(persist ? kCmdWrite : kCmdWriteTemp, 0x00,
                           E22Config::kBytes, want);
  if (ok) {
    uint8_t got[E22Config::kBytes];
    ok = readRegisters(0x00, E22Config::kBytes, got);
    // Bytes 7 and 8 are the key, which always reads back as 0.
    if (ok) ok = memcmp(want, got, 7) == 0;
  }

  if (ok) {
    cfg_ = cfg;
    cfgKnown_ = true;
    uartBaud_ = cfg.uartBaudValue();  // leaveConfig reopens the port at this rate
  }
  leaveConfig(prev);
  return ok;
}

bool E22::updateConfig(bool persist, void (*mutate)(E22Config&, uint32_t), uint32_t arg) {
  E22Config c;
  if (cfgKnown_) {
    c = cfg_;
  } else if (!readConfig(c)) {
    return false;
  }
  mutate(c, arg);
  return writeConfig(c, persist);
}

bool E22::setChannel(uint8_t channel, bool persist) {
  if (channel > 83) return false;
  return updateConfig(persist, [](E22Config& c, uint32_t v) { c.channel = (uint8_t)v; }, channel);
}

bool E22::setAirRate(E22AirRate rate, bool persist) {
  return updateConfig(persist, [](E22Config& c, uint32_t v) { c.airRate = (E22AirRate)v; },
                      (uint32_t)rate);
}

bool E22::setTxPower(E22TxPower power, bool persist) {
  return updateConfig(persist, [](E22Config& c, uint32_t v) { c.txPower = (E22TxPower)v; },
                      (uint32_t)power);
}

bool E22::setAddress(uint16_t address, uint8_t netId, bool persist) {
  uint32_t packed = ((uint32_t)address << 8) | netId;
  return updateConfig(persist, [](E22Config& c, uint32_t v) {
    c.address = (uint16_t)(v >> 8);
    c.netId = (uint8_t)(v & 0xFF);
  }, packed);
}

bool E22::setRssiByte(bool enable, bool persist) {
  return updateConfig(persist, [](E22Config& c, uint32_t v) { c.rssiByte = v != 0; }, enable);
}

bool E22::setAmbientRssi(bool enable, bool persist) {
  return updateConfig(persist, [](E22Config& c, uint32_t v) { c.ambientRssi = v != 0; }, enable);
}

// --- AT commands (mode 2) ----------------------------------------------------

bool E22::atCommand(const char* cmd, char* reply, size_t replyLen, uint32_t timeoutMs) {
  if (replyLen == 0) return false;
  E22Mode prev;
  if (!enterConfig(prev)) return false;

  flushInput();
  // No CR/LF. Firmware 7453-0-20 and older does not accept it, and newer
  // firmware works with or without it.
  uart_.print(cmd);
  uart_.flush();

  size_t n = 0;
  bool gotAny = false, done = false;
  uint32_t start = millis(), last = millis();
  while (!done) {
    int b = uart_.read();
    if (b >= 0) {
      gotAny = true;
      last = millis();
      if (b == '\n') { done = true; continue; }
      if (b == '\r') continue;
      if (n + 1 < replyLen) reply[n++] = (char)b;
      continue;
    }
    uint32_t now = millis();
    if (gotAny && now - last > kAtIdleGapMs) done = true;
    else if (now - start > timeoutMs) done = true;
    else delay(1);
  }
  reply[n] = '\0';

  leaveConfig(prev);
  return gotAny && strstr(reply, "ERR") == nullptr;
}

bool E22::readFirmwareVersion(char* buf, size_t len) {
  return atCommand("AT+FWCODE=?", buf, len);
}

