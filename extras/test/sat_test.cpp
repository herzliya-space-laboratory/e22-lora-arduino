// Host tests for examples/SatNode: the frame format, the parser and the relay
// logic. The last part runs many Nodes in a simulated network (airtime,
// half duplex, collisions, lost packets, the module's RSSI byte, other
// people's traffic on the channel, power cycles) and checks that every
// message stops, that no station transmits a message more often than
// allowed, and that messages arrive.
//
// Build and run: extras/test/run.sh

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "Frame.h"
#include "Node.h"

using namespace sat;

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                         \
  do {                                                                      \
    checks++;                                                               \
    if (!(cond)) {                                                          \
      failures++;                                                           \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                       \
  } while (0)

#define CHECK_MSG(cond, ...)                                                \
  do {                                                                      \
    checks++;                                                               \
    if (!(cond)) {                                                          \
      failures++;                                                           \
      std::printf("FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond);           \
      std::printf(__VA_ARGS__);                                             \
      std::printf("\n");                                                    \
    }                                                                       \
  } while (0)

static std::string str(const char call[kCallLen]) {
  char s[kCallLen + 1];
  callToString(call, s);
  return s;
}

static Frame makeMessage(const char* origin, const char* sender, const char* dest, uint8_t hops,
                         uint16_t boot, uint16_t seq, const char* text) {
  Frame f{};
  f.type = FrameType::Message;
  f.hops = hops;
  setCall(f.origin, origin);
  setCall(f.sender, sender);
  setCall(f.dest, dest);
  f.boot = boot;
  f.seq = seq;
  f.len = (uint8_t)std::strlen(text);
  std::memcpy(f.payload, text, f.len);
  return f;
}

static bool sameFrame(const Frame& a, const Frame& b) {
  return a.type == b.type && a.hops == b.hops && callEquals(a.origin, b.origin) &&
         callEquals(a.sender, b.sender) && callEquals(a.dest, b.dest) && a.boot == b.boot &&
         a.seq == b.seq && a.len == b.len && std::memcmp(a.payload, b.payload, a.len) == 0;
}

// ---------------------------------------------------------------------------
// Frame

static void testCrc() {
  const char* v = "123456789";
  CHECK(crc16((const uint8_t*)v, 9) == 0x29B1);  // CRC-16/CCITT-FALSE check value
}

static void testEncodeDecode() {
  Frame f = makeMessage("4X1ABC", "4Z9XY", "ALL", 3, 7, 1234, "hello");
  uint8_t buf[kMaxFrameLen];
  size_t n = encode(f, buf, sizeof buf);
  CHECK(n == kHeaderLen + 5 + kCrcLen);
  CHECK(buf[0] == 0xA5 && buf[1] == 0x5A && buf[2] == 0x12 && buf[3] == 3);
  CHECK(std::memcmp(buf + 4, "4X1ABC", 6) == 0);
  CHECK(std::memcmp(buf + 10, "4Z9XY ", 6) == 0);
  CHECK(std::memcmp(buf + 16, "ALL   ", 6) == 0);
  CHECK(buf[22] == 7 && buf[23] == 0 && buf[24] == (1234 & 0xFF) && buf[25] == (1234 >> 8));
  Frame g{};
  CHECK(decode(buf, n, g));
  CHECK(sameFrame(f, g));

  // Every single-bit error is caught.
  bool allCaught = true;
  for (size_t i = 0; i < n; i++) {
    for (int bit = 0; bit < 8; bit++) {
      buf[i] ^= (uint8_t)(1 << bit);
      if (decode(buf, n, g)) allCaught = false;
      buf[i] ^= (uint8_t)(1 << bit);
    }
  }
  CHECK(allCaught);
  CHECK(!decode(buf, n - 1, g));
  CHECK(!decode(buf, n + 1, g));

  // Full payload fits, one more byte does not.
  f.len = kMaxPayload;
  CHECK(encode(f, buf, sizeof buf) == kMaxFrameLen);
  CHECK(encode(f, buf, sizeof buf - 1) == 0);
  f.len = kMaxPayload + 1;
  CHECK(encode(f, buf, sizeof buf) == 0);
  CHECK(kMaxFrameLen <= 240);  // one radio packet
}

static void testCallsigns() {
  char c[kCallLen];
  CHECK(setCall(c, "4x1abc") && std::memcmp(c, "4X1ABC", 6) == 0);
  CHECK(setCall(c, "4Z5") && std::memcmp(c, "4Z5   ", 6) == 0);
  CHECK(!setCall(c, ""));
  CHECK(!setCall(c, "4X1ABCD"));
  CHECK(!setCall(c, "4X-1"));
  setCall(c, "4X1ABC");
  CHECK(isStation(c) && isIsraeli(c) && !isBroadcast(c));
  setCall(c, "W1AW");
  CHECK(isStation(c) && !isIsraeli(c));
  setCall(c, "ALL");
  CHECK(isBroadcast(c) && !isStation(c));
  setCall(c, "AB");
  CHECK(!isStation(c));
  std::memcpy(c, "4X1 AB", 6);  // space in the middle
  CHECK(!isStation(c));
  std::memcpy(c, "4X1\0AB", 6);
  CHECK(!isStation(c));
  char s[kCallLen + 1];
  setCall(c, "4Z5");
  callToString(c, s);
  CHECK(std::strcmp(s, "4Z5") == 0);
}

