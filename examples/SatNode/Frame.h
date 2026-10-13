// The frame format of the satellite network, and a parser that finds frames
// in the byte stream coming from the module. Plain C++ without Arduino calls,
// so the host tests in extras/test build the same file.
//
// One frame is one radio packet, 77 bytes at most (the module's packet size
// is 240, so a frame is never split on air):
//
//   offset  size  field
//   0       2     sync, A5 5A
//   2       1     version (high 4 bits, = 1) and type (low 4 bits)
//   3       1     hops left. A station that relays the frame subtracts 1;
//                 at 0 nobody relays it.
//   4       6     origin: callsign of the station that wrote the message
//   10      6     sender: callsign of the station that transmitted this copy
//   16      6     destination callsign, or ALL
//   22      2     origin's boot count, little endian
//   24      2     origin's sequence number, little endian
//   26      1     payload length, 0..48
//   27      n     payload
//   27+n    2     CRC-16/CCITT-FALSE over bytes 2..26+n, little endian
//
// Callsigns are ASCII, upper case, padded with spaces to 6 bytes. Every frame
// carries the callsign of the station that transmitted it (sender).
// (origin, boot, seq) names a message. Every copy of it, relayed or repeated,
// keeps the same three values. The boot count comes from EEPROM, so a station
// that was power-cycled and starts seq at 0 again still makes new names.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace sat {

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kVersion = 1;
constexpr size_t kCallLen = 6;
constexpr size_t kMaxPayload = 48;  // open: maximum message size (ask Maayan)
constexpr size_t kHeaderLen = 27;
constexpr size_t kCrcLen = 2;
constexpr size_t kMaxFrameLen = kHeaderLen + kMaxPayload + kCrcLen;
constexpr uint8_t kMaxHops = 7;     // frames claiming more are dropped

enum class FrameType : uint8_t {
  Beacon = 1,   // status of the sender, see Node.h for the payload
  Message = 2,  // text from a station, to one station or to ALL
  Ack = 3,      // the destination got a message. Payload: its boot, seq
};

struct Frame {
  FrameType type;
  uint8_t hops;
  char origin[kCallLen];
  char sender[kCallLen];
  char dest[kCallLen];
  uint16_t boot;
  uint16_t seq;
  uint8_t len;
  uint8_t payload[kMaxPayload];
};

uint16_t crc16(const uint8_t* data, size_t len);

// Writes the frame to out. Returns the number of bytes, or 0 if out is too
// small or the payload too long.
size_t encode(const Frame& f, uint8_t* out, size_t outLen);
// in must hold exactly one frame. Checks sync, version, type, length and CRC.
bool decode(const uint8_t* in, size_t len, Frame& f);

// Callsigns. "text" is a C string, the 6-byte form is padded with spaces.
extern const char kBroadcast[kCallLen];  // "ALL"
bool setCall(char out[kCallLen], const char* text);  // false if not 1..6 letters/digits
void callToString(const char call[kCallLen], char out[kCallLen + 1]);
bool callEquals(const char a[kCallLen], const char b[kCallLen]);
bool isBroadcast(const char call[kCallLen]);
bool isStation(const char call[kCallLen]);  // 3..6 letters and digits, at least one digit
bool isIsraeli(const char call[kCallLen]);  // starts with 4X or 4Z

