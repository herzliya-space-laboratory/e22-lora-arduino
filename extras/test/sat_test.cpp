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

// ---------------------------------------------------------------------------
// Parser

static std::vector<uint8_t> bytesOf(const Frame& f) {
  uint8_t buf[kMaxFrameLen];
  size_t n = encode(f, buf, sizeof buf);
  return std::vector<uint8_t>(buf, buf + n);
}

struct Got {
  Frame f;
  uint8_t rssi;
};

static std::vector<Got> feedAll(Parser& p, const std::vector<uint8_t>& bytes, uint32_t t = 0) {
  std::vector<Got> out;
  for (uint8_t b : bytes)
    if (p.feed(b, t)) out.push_back({p.frame(), p.rssiByte()});
  return out;
}

static void testParser() {
  Frame a = makeMessage("4X1ABC", "4X1ABC", "ALL", 3, 1, 1, "first");
  Frame b = makeMessage("4X2DEF", "4X2DEF", "4X1ABC", 2, 9, 77, "second");
  std::vector<uint8_t> fa = bytesOf(a), fb = bytesOf(b);

  {  // Without RSSI byte, garbage before and between.
    Parser p;
    std::vector<uint8_t> s = {0x00, 0xA5, 0x13, 0xA5};
    s.insert(s.end(), fa.begin(), fa.end());
    s.insert(s.end(), {'h', 'i', '\n'});
    s.insert(s.end(), fb.begin(), fb.end());
    auto got = feedAll(p, s);
    CHECK(got.size() == 2);
    CHECK(got.size() == 2 && sameFrame(got[0].f, a) && sameFrame(got[1].f, b));
  }
  {  // With RSSI byte after each packet, including RSSI bytes that look like sync.
    Parser p;
    p.setExpectRssi(true);
    std::vector<uint8_t> s = fa;
    s.push_back(0xA5);  // RSSI -91 dBm
    s.insert(s.end(), fb.begin(), fb.end());
    s.push_back(0x5A);
    auto got = feedAll(p, s);
    CHECK(got.size() == 2);
    CHECK(got.size() == 2 && got[0].rssi == 0xA5 && got[1].rssi == 0x5A);
  }
  {  // RSSI byte never comes: the frame is handed over after the gap.
    Parser p;
    p.setExpectRssi(true);
    auto got = feedAll(p, fa, 1000);
    CHECK(got.empty() && p.busy());
    CHECK(!p.expire(1000 + Parser::kGapMs - 1));
    CHECK(p.expire(1000 + Parser::kGapMs));
    CHECK(sameFrame(p.frame(), a) && p.rssiByte() == 0);
    CHECK(!p.busy());
  }
  {  // A frame cut off in the middle is dropped after the gap, the next one is fine.
    Parser p;
    std::vector<uint8_t> half(fa.begin(), fa.begin() + 20);
    CHECK(feedAll(p, half, 0).empty());
    CHECK(!p.expire(Parser::kGapMs) || false);
    CHECK(!p.busy());
    CHECK(p.dropped() == 1);
    auto got = feedAll(p, fb, 500);
    CHECK(got.size() == 1 && sameFrame(got[0].f, b));
  }
  {  // Corrupted frame, then a good one: the good one survives.
    Parser p;
    std::vector<uint8_t> bad = fa;
    bad[30] ^= 0x40;
    std::vector<uint8_t> s = bad;
    s.insert(s.end(), fb.begin(), fb.end());
    auto got = feedAll(p, s);
    CHECK(got.size() == 1 && sameFrame(got[0].f, b));
    CHECK(p.dropped() == 1);
  }
  {  // False sync in foreign data right before a frame, no gap between them.
    Parser p;
    // The false header claims a long frame, so the real one is found only
    // after the gap, but it is found.
    std::vector<uint8_t> s = {'4', 'X', 0xA5, 0x5A, 0x12, 'x', 'y'};
    s.insert(s.end(), fa.begin(), fa.end());
    auto got = feedAll(p, s, 0);
    CHECK(got.empty());
    CHECK(p.expire(Parser::kGapMs) && sameFrame(p.frame(), a));
    CHECK(!p.busy());
    // Same with the RSSI byte after the frame.
    Parser q;
    q.setExpectRssi(true);
    s.push_back(0xC8);
    CHECK(feedAll(q, s, 0).empty());
    CHECK(q.expire(Parser::kGapMs) && sameFrame(q.frame(), a) && q.rssiByte() == 0xC8);
  }
  {  // False sync whose length byte says 255.
    Parser p;
    std::vector<uint8_t> s(30, 0x00);
    s[0] = 0xA5; s[1] = 0x5A; s[2] = 0x12; s[26] = 255;
    s.insert(s.end(), fb.begin(), fb.end());
    auto got = feedAll(p, s);
    CHECK(got.size() == 1 && sameFrame(got[0].f, b));
  }
  {  // Random noise never crashes the parser and never yields a frame.
    std::mt19937 rng(1);
    Parser p;
    p.setExpectRssi(true);
    int frames = 0;
    for (int i = 0; i < 200000; i++)
      if (p.feed((uint8_t)rng(), (uint32_t)i)) frames++;
    CHECK(frames == 0);
  }
}

