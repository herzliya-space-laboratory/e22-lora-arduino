# E22 LoRa UART

Arduino driver for the Ebyte E22 UART LoRa modules. We use it on the satellite communication
card, which has an ATmega328PB and an E22-400T30D. It also works on an ESP32, which is easier
to use on the bench.

This is only the module driver. The beacon and transponder logic that runs on the satellite is a
separate sketch that uses this library.

## About the module

The E22 is not a LoRa chip with an SPI interface. It is a small board with its own
microcontroller in front of an SX1268 radio. You talk to that microcontroller over a serial port
at 9600 baud. Three more pins are used: M0 and M1 select the operating mode, and AUX shows if
the module is busy (low) or idle (high). There is no NSS, no DIO0, and no direct access to the
radio registers.

The DIP module has seven pins:

| Pin | Name | Notes                                                   |
|-----|------|---------------------------------------------------------|
| 1   | M0   | mode bit 0. Must not be left floating.                  |
| 2   | M1   | mode bit 1. Must not be left floating.                  |
| 3   | RXD  | serial input, 3.3 V logic                               |
| 4   | TXD  | serial output                                           |
| 5   | AUX  | busy/idle output. Do not drive it.                      |
| 6   | VCC  | 5 V for full power. TX draws up to 620 mA on the T30D.  |
| 7   | GND  |                                                         |

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
- The other side of the link probably has to be another E22. Ebyte adds its own framing and FEC
  on top of LoRa, and we did not find anyone who decoded it with a normal SX127x. David says he
  received it once with different hardware. We will test this instead of assuming.
- The frequency is a channel number: 410.125 MHz + channel × 1 MHz, channels 0 to 83. The
  satellite's main transceiver is on 436.4 MHz, so channels 25 to 28 must not be used. The
  channel is a runtime setting, not a constant.
- The module does not keep packet boundaries on the serial side. Two packets sent one after the
  other can come out of the UART as one stream. Our packet format needs its own sync bytes, a
  length field and a checksum.

## Usage

```cpp
#include <E22.h>

E22 radio(Serial1, 4, 5, 6);   // serial port, M0, M1, AUX. Optional 5th argument: RESET pin.

void setup() {
  Serial.begin(115200);
  radio.begin(9600);

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

Each example has a block at the top that selects the serial port and the pins for each target.
The ATmega pin numbers there are placeholders until we have the schematic.

### API

| Call | Description |
|------|-------------|
| `begin(baud, rx, tx)` | opens the port, enters transmission mode, waits for AUX. rx/tx are only used on ESP32. |
| `setMode(E22Mode)` | sets M0/M1, waits for AUX, and switches the serial port to 9600 in configuration mode |
| `waitIdle(timeoutMs)` | waits until AUX is high |
| `hardReset()` | pulses RESET, if a RESET pin was given. The DIP module has no RESET pin. |
| `readConfig(cfg)` / `writeConfig(cfg, persist)` | all nine registers. `persist=false` uses `C2`, so the change is lost after a power cycle. |
| `setChannel`, `setAirRate`, `setTxPower`, `setAddress`, `setRssiByte`, `setAmbientRssi` | change one field |
| `atCommand(cmd, reply, len)`, `readFirmwareVersion(buf, len)` | AT commands, in configuration mode |
| `send(buf, len)`, `send("text")` | sends in chunks of the packet size, waits for AUX between chunks |
| `sendTo(addr, ch, buf, len)` | adds the fixed-point header. Needs `cfg.fixedPoint`. |
| `available()`, `read()`, `readByte()`, `flushInput()` | raw bytes from the module |
| `readAmbientRssi(dbm)`, `readLastPacketRssi(dbm)` | need `cfg.ambientRssi`, only in transmission mode |
| `E22::rssiByteToDbm(b)` | converts the RSSI byte to dBm |

Power levels: `E22TxPower::Level0` is the maximum on every module. `dBm30`..`dBm21` and
`dBm22`..`dBm10` are names for the same four codes on the 30 dBm and 22 dBm modules.

## Building

For the flight board you need MiniCore, which adds the ATmega328PB:

```
arduino-cli config add board_manager.additional_urls https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
arduino-cli core install MiniCore:avr
arduino-cli compile --fqbn MiniCore:avr:328:variant=modelPB,clock=16MHz_external --library . examples/ReadConfig
```

For an ESP32 on the bench:

```
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 --library . examples/ReadConfig
```

CI compiles the examples for both targets on every push.

## Notes and limitations

- The ATmega has 2 KB of RAM. Use small buffers, put string literals in `F()`, do not use
  `printf`.
- The ATmega runs at 5 V and the module inputs are 3.3 V. The board must level-shift RXD, M0
  and M1. The schematic seems to have series resistors on these lines. Not confirmed yet.
- Configuration mode is always 9600 8N1. If the UART runs at another speed, the driver reopens
  the port before and after every configuration call.
- If AUX is held low while the module powers up, the module enters firmware upgrade mode and
  stops responding. Never put a pull-down on AUX.
- At 30 dBm the module draws up to 620 mA while transmitting. If the 5 V supply drops, the module
  resets in the middle of a packet.
- On firmware 7453-0-21 and newer, AUX does not go low during the radio transmission unless
  `AT+UAUX` is enabled. So `send()` returns when the serial buffer is empty, not when the packet
  has been transmitted.

## Status

Compiles for both targets. Not tested on real hardware yet. Written from the manual, and the
byte layouts were compared with Renzo Mischianti's
[E22 library](https://github.com/xreef/EByte_LoRa_E22_Series_Library). The first test on a real
board: run `ReadConfig` and check that the factory values `00 00 00 62 00 17 03 00 00` come
back, then record the firmware version.

MIT license.
