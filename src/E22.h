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

