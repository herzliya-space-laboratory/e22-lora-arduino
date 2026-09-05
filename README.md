# E22 LoRa UART

Arduino driver for the Ebyte E22 UART LoRa modules, written for the satellite communication
card (E22-400T22S on an ESP32 for now). This is only the module driver. The beacon and
transponder logic that runs on the satellite is a separate sketch that uses this library.

## About the module

The E22 is not a LoRa chip with an SPI interface. It is a small board with its own
microcontroller in front of an SX1268 radio. You talk to that microcontroller over a serial port
at 9600 baud. Three more pins are used: M0 and M1 select the operating mode, and AUX shows if
the module is busy (low) or idle (high). There is no NSS, no DIO0, and no direct access to the
radio registers.

| Pin   | Notes                                                        |
|-------|--------------------------------------------------------------|
| RXD   | serial input, 3.3 V logic                                    |
| TXD   | serial output                                                |
| M0    | mode bit 0. Must not be left floating.                       |
| M1    | mode bit 1. Must not be left floating.                       |
| AUX   | busy/idle output. Do not drive it.                           |
| RESET | active low. Optional, but the manual recommends connecting it |

Operating modes, selected with M1 and M0:

- `0 0` transmission. Normal send and receive.
- `0 1` wake-on-radio. Not used here.
- `1 0` configuration. The serial port is always 9600 8N1 in this mode.
- `1 1` sleep.

The configuration is nine register bytes: address, network ID, baud rate and air rate, packet
size and power, channel, option bits, and an encryption key. Read them with `C1 addr len`. Write
them with `C0 addr len data` (saved in flash) or `C2 addr len data` (temporary). Section 6 of the
manual in `docs/` has the full register map.

Some facts about the module that affect the design:

- The spreading factor cannot be set directly. The module has an "air data rate" from 2.4 to
  62.5 kbps, and each rate is a fixed SF/BW pair chosen by Ebyte. `setAirRate()` is the only
  control we have.
- The other side of the link has to be another E22. Ebyte adds its own framing and FEC on top
  of LoRa, and we did not find anyone who decoded it with a normal SX127x.
- The frequency is a channel number: 410.125 MHz + channel × 1 MHz, channels 0 to 83. The
  factory default is 23, which is 433.125 MHz.
- The module does not keep packet boundaries on the serial side. Two packets sent one after the
  other can come out of the UART as one stream. Our packet format needs its own sync bytes, a
  length field and a checksum.

## Usage

```cpp
#include <E22.h>

E22 radio(Serial2, 32, 33, 34, 27);   // serial port, M0, M1, AUX, RESET

void setup() {
  Serial.begin(115200);
  radio.begin(9600, /*rx=*/16, /*tx=*/17);   // the pin arguments are only used on ESP32

  E22Config cfg;
  radio.readConfig(cfg);
  cfg.channel = 23;            // 433.125 MHz
  cfg.rssiByte = true;         // the module adds an RSSI byte to each received packet
  radio.writeConfig(cfg);      // writes, reads back, compares

  radio.send("hello\n");
}

void loop() {
  int b = radio.readByte();
  if (b >= 0) Serial.write(b);
}
```

Every configuration call switches the module to configuration mode, does the operation, and
switches back. Every wait on AUX has a timeout, and the call returns `false` if the timeout
expires. Nothing in the library blocks forever. This matters because the only thing the OBC can
do to this card is turn it off and on.

The library only moves bytes. Framing, checksums and counters belong in the sketch that uses it.

### Examples

Run them in this order on a new board:

1. `ReadConfig` prints all the registers and the firmware version. If this works, the wiring
   is correct.
2. `PingPong` needs two boards. One sends PING, the other answers PONG. Both print the RSSI.
3. `AirRateSweep` is the "change the SF every 10 seconds" experiment from the project document,
   done with the air rate. Both boards go through the six rates in sync, starting from boot.

### API

| Call | Description |
|------|-------------|
| `begin(baud, rx, tx)` | opens the port, enters transmission mode, waits for AUX |
| `setMode(E22Mode)` | sets M0/M1, waits for AUX, and switches the serial port to 9600 in configuration mode |
| `waitIdle(timeoutMs)` | waits until AUX is high |
| `hardReset()` | pulses RESET, if a RESET pin was given |
| `readConfig(cfg)` / `writeConfig(cfg, persist)` | all nine registers. `persist=false` uses `C2`, so the change is lost after a power cycle. |
| `setChannel`, `setAirRate`, `setTxPower`, `setAddress`, `setRssiByte`, `setAmbientRssi` | change one field |
| `atCommand(cmd, reply, len)`, `readFirmwareVersion(buf, len)` | AT commands, in configuration mode |
| `send(buf, len)`, `send("text")` | sends in chunks of the packet size, waits for AUX between chunks |
| `sendTo(addr, ch, buf, len)` | adds the fixed-point header. Needs `cfg.fixedPoint`. |
| `available()`, `read()`, `readByte()`, `flushInput()` | raw bytes from the module |
| `readAmbientRssi(dbm)`, `readLastPacketRssi(dbm)` | need `cfg.ambientRssi`, only in transmission mode |
| `E22::rssiByteToDbm(b)` | converts the RSSI byte to dBm |

## Notes and limitations

- The module uses 3.3 V logic. A 5 V Arduino needs level shifting on RXD, M0 and M1.
- Configuration mode is always 9600 8N1. If the UART runs at another speed, the driver reopens
  the port before and after every configuration call.
- If AUX is held low while the module powers up, the module enters firmware upgrade mode and
  stops responding. Never put a pull-down on AUX.
- The module draws 100 to 140 mA while transmitting at 22 dBm. If the 3.3 V supply drops, the
  module resets in the middle of a packet.
- On firmware 7453-0-21 and newer, AUX does not go low during the radio transmission unless
  `AT+UAUX` is enabled. So `send()` returns when the serial buffer is empty, not when the packet
  has been transmitted.

## Status

Compiles. Not tested on real hardware yet. Written from the manual, and the byte layouts were
compared with Renzo Mischianti's
[E22 library](https://github.com/xreef/EByte_LoRa_E22_Series_Library). The first test on a real
board: run `ReadConfig` and check that the factory values `00 00 00 62 00 17 03 00 00` come
back.

MIT license.
