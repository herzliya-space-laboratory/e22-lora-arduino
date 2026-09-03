// Driver for the Ebyte E22 UART LoRa modules (E22-400T22S; the rest of the
// T series uses the same protocol).
//
// The module works like a serial modem. You talk to it over UART, select the
// mode with M0/M1, and read AUX to know when it is busy. This class does the
// mode switching, the register read/write protocol (C0/C1/C2 commands,
// manual section 6) and simple send/receive. It does not do packet framing.
// That is done in the sketch.

#pragma once

#include <Arduino.h>

// The enums below follow the register bit fields in manual section 6.3.

enum class E22Mode : uint8_t {
  Transmission  = 0,  // M1=0 M0=0  normal send/receive
  WOR           = 1,  // M1=0 M0=1  wake-on-radio, unused here
  Configuration = 2,  // M1=1 M0=0  register access, serial forced to 9600 8N1
  Sleep         = 3,  // M1=1 M0=1
};

// REG0 bits 7:5
enum class E22UartBaud : uint8_t {
  B1200 = 0, B2400 = 1, B4800 = 2, B9600 = 3,
  B19200 = 4, B38400 = 5, B57600 = 6, B115200 = 7,
};

// REG0 bits 4:3
enum class E22Parity : uint8_t { N8 = 0, O8 = 1, E8 = 2 };

// REG0 bits 2:0. Codes 0..2 all mean 2.4 kbps on the 400 MHz modules.
// There is no direct SF setting. This is the only rate control.
enum class E22AirRate : uint8_t {
  Rate2k4  = 0b010,  // factory default
  Rate4k8  = 0b011,
  Rate9k6  = 0b100,
  Rate19k2 = 0b101,
  Rate38k4 = 0b110,
  Rate62k5 = 0b111,
};

// REG1 bits 7:6
enum class E22PacketSize : uint8_t { B240 = 0, B128 = 1, B64 = 2, B32 = 3 };

// REG1 bits 1:0. These are the dBm values for the 22 dBm modules.
enum class E22TxPower : uint8_t { dBm22 = 0, dBm17 = 1, dBm14 = 2, dBm10 = 3 };

// The nine config registers, 0x00..0x08, unpacked.
struct E22Config {
  uint16_t address = 0x0000;      // 00h,01h. 0xFFFF = broadcast / listen to all
  uint8_t netId = 0;              // 02h
  // 03h
  E22UartBaud uartBaud = E22UartBaud::B9600;
  E22Parity parity = E22Parity::N8;
  E22AirRate airRate = E22AirRate::Rate2k4;
  // 04h
  E22PacketSize packetSize = E22PacketSize::B240;
  bool ambientRssi = false;       // enables the noise/RSSI register reads in mode 0
  bool softwareModeSwitch = false;
  E22TxPower txPower = E22TxPower::dBm22;
  // 05h. Frequency = 410.125 MHz + channel MHz. Factory 23 = 433.125.
  uint8_t channel = 23;
  // 06h
  bool rssiByte = false;          // module adds an RSSI byte after received data
  bool fixedPoint = false;        // first 3 bytes sent = destination address + channel
  bool relay = false;
  bool lbt = false;               // listen before talk
  bool worTransmitter = false;
  uint8_t worCycle = 3;           // (n+1) * 500 ms
  uint16_t cryptKey = 0;          // 07h,08h. Write only, reads back as 0.

  static constexpr size_t kBytes = 9;
  void toBytes(uint8_t out[kBytes]) const;
  static E22Config fromBytes(const uint8_t in[kBytes]);

  uint16_t packetSizeBytes() const;
  uint32_t uartBaudValue() const;
  uint32_t frequencyKHz() const { return 410125UL + 1000UL * channel; }  // 400 MHz band
};

class E22 {
 public:
  // Pass -1 for a pin that is not connected. M0 and M1 are needed to
  // configure the module. AUX is optional; without it the driver uses fixed
  // delays.
  E22(HardwareSerial& uart, int8_t pinM0, int8_t pinM1, int8_t pinAux,
      int8_t pinReset = -1);

  // Sets up the pins, opens the port at uartBaud (must match the baud rate
  // stored in the module, factory 9600) and enters transmission mode.
  // rxPin/txPin are only used on ESP32. Returns false if AUX stays low.
  bool begin(uint32_t uartBaud = 9600, int8_t rxPin = -1, int8_t txPin = -1);

  // Mode
  bool setMode(E22Mode mode);
  E22Mode mode() const { return mode_; }
  bool waitIdle(uint32_t timeoutMs = kDefaultTimeoutMs);  // AUX high, or timeout
  bool isIdle() const;
  bool hardReset();  // pulses RESET, if a RESET pin was given

  void flushInput();

  static constexpr uint32_t kDefaultTimeoutMs = 1000;
  static constexpr uint32_t kConfigBaud = 9600;

 private:
  bool enterConfig(E22Mode& previous);
  bool leaveConfig(E22Mode previous);
  bool readRegisters(uint8_t addr, uint8_t len, uint8_t* out);
  bool writeRegisters(uint8_t cmd, uint8_t addr, uint8_t len, const uint8_t* data);
  bool readBytes(uint8_t* buf, size_t len, uint32_t timeoutMs);
  void openSerial(uint32_t baud);
  void driveModePins(E22Mode mode);

  HardwareSerial& uart_;
  int8_t pinM0_, pinM1_, pinAux_, pinReset_;
  int8_t rxPin_ = -1, txPin_ = -1;
  uint32_t uartBaud_ = 9600;
  E22Mode mode_ = E22Mode::Transmission;
  E22Config cfg_;
  bool cfgKnown_ = false;
};
