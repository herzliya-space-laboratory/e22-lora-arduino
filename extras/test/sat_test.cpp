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

// ---------------------------------------------------------------------------
// Node, one at a time

static Node makeNode(const char* call, uint16_t boot = 1, uint32_t seed = 1) {
  Node n;
  char c[kCallLen];
  setCall(c, call);
  n.begin(c, boot, seed, 0);
  n.config().beacon = false;
  return n;
}

static int drain(Node& n, uint32_t from, uint32_t to, std::vector<Frame>* out = nullptr) {
  int count = 0;
  Frame f;
  for (uint32_t t = from; t <= to; t += 10) {
    if (n.poll(t, f)) {
      count++;
      if (out) out->push_back(f);
    }
  }
  return count;
}

static void testNodeRules() {
  char all[kCallLen], sat1[kCallLen];
  setCall(all, "ALL");
  setCall(sat1, "4X1SAT");

  {  // Relays a new broadcast with hops - 1, 1 + repeats times, then stops.
    Node n = makeNode("4X1SAT");
    Frame m = makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, 5, "hi");
    CHECK(n.onReceive(m, -50, 0) == Rx::ForUs);
    CHECK(n.queued());
    std::vector<Frame> sent;
    int c = drain(n, 0, 200000, &sent);
    CHECK(c == 1 + n.config().repeats);
    for (const Frame& f : sent) {
      CHECK(f.hops == 2 && str(f.sender) == "4X1SAT" && str(f.origin) == "4X0GND" && f.seq == 5);
    }
    CHECK(n.stored() == 0);
    // The first copy waits for the random backoff, not longer.
    Node n2 = makeNode("4X1SAT");
    n2.onReceive(m, -50, 1000);
    Frame f;
    CHECK(!n2.poll(1000 + n2.config().backoffMinMs - 1, f));
    CHECK(n2.poll(1000 + n2.config().backoffMaxMs, f));
  }
  {  // Second copy is a duplicate, own messages are ignored.
    Node n = makeNode("4X1SAT");
    Frame m = makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, 5, "hi");
    CHECK(n.onReceive(m, -50, 0) == Rx::ForUs);
    Frame again = m;
    setCall(again.sender, "4X2SAT");
    again.hops = 1;
    CHECK(n.onReceive(again, -50, 10) == Rx::Duplicate);
    CHECK(!n.queued());
    Frame own = makeMessage("4X1SAT", "4X2SAT", "ALL", 2, 1, 0, "mine");
    CHECK(n.onReceive(own, -50, 20) == Rx::Own);
    CHECK(!n.queued());
  }
  {  // hops 0: delivered, not relayed.
    Node n = makeNode("4X1SAT");
    CHECK(n.onReceive(makeMessage("4X0GND", "4X2SAT", "ALL", 0, 1, 1, "x"), -50, 0) == Rx::ForUs);
    CHECK(!n.queued() && n.stored() == 0);
  }
  {  // Bad frames.
    Node n = makeNode("4X1SAT");
    CHECK(n.onReceive(makeMessage("4X0GND", "4X0GND", "ALL", kMaxHops + 1, 1, 1, "x"), 0, 0) ==
          Rx::Invalid);
    CHECK(n.onReceive(makeMessage("W1AW", "W1AW", "ALL", 3, 1, 1, "x"), 0, 0) == Rx::NotIsraeli);
    CHECK(n.onReceive(makeMessage("4X0GND", "K1ABC", "ALL", 3, 1, 1, "x"), 0, 0) == Rx::NotIsraeli);
    Frame bad = makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, 1, "x");
    std::memcpy(bad.origin, "4X\x01GND", 6);
    CHECK(n.onReceive(bad, 0, 0) == Rx::Invalid);
    CHECK(n.stored() == 0);
    n.config().israeliOnly = false;
    CHECK(n.onReceive(makeMessage("W1AW", "W1AW", "ALL", 3, 1, 1, "x"), 0, 0) == Rx::ForUs);
  }
  {  // A message to us: ACK queued, the message itself not relayed.
    Node n = makeNode("4X1SAT");
    CHECK(n.onReceive(makeMessage("4X0GND", "4X2SAT", "4X1SAT", 2, 3, 9, "x"), 0, 0) == Rx::ForUs);
    CHECK(!n.queued());
    std::vector<Frame> sent;
    drain(n, 0, 1000, &sent);
    CHECK(sent.size() == 1);
    if (sent.size() == 1) {
      const Frame& a = sent[0];
      CHECK(a.type == FrameType::Ack && str(a.dest) == "4X0GND" && str(a.origin) == "4X1SAT");
      CHECK(a.len == 4 && a.payload[0] == 3 && a.payload[2] == 9);
      CHECK(a.hops == n.config().hops);
    }
  }
  {  // A message to someone else is relayed.
    Node n = makeNode("4X1SAT");
    CHECK(n.onReceive(makeMessage("4X0GND", "4X0GND", "4X3SAT", 2, 1, 1, "x"), 0, 0) == Rx::ForOthers);
    CHECK(n.queued());
  }
  {  // ACK for our message stops our repeats; an ACK passing by stops a relay's repeats.
    Node g = makeNode("4X0GND");
    uint16_t seq = 0;
    CHECK(g.send(sat1, (const uint8_t*)"hi", 2, 0, &seq));
    Frame f;
    CHECK(g.poll(0, f) && f.type == FrameType::Message && f.seq == seq);
    CHECK(g.stored() == 1);
    Frame ack = makeMessage("4X1SAT", "4X1SAT", "4X0GND", 3, 4, 0, "");
    ack.type = FrameType::Ack;
    ack.len = 4;
    ack.payload[0] = 1; ack.payload[1] = 0; ack.payload[2] = (uint8_t)seq; ack.payload[3] = 0;
    CHECK(g.onReceive(ack, 0, 500) == Rx::AckForUs);
    CHECK(g.ackedSeq() == seq);
    CHECK(g.stored() == 0);
    CHECK(drain(g, 500, 200000) == 0);

    Node r = makeNode("4X2SAT");
    Frame m = makeMessage("4X0GND", "4X0GND", "4X1SAT", 3, 1, 0, "hi");
    r.onReceive(m, 0, 0);
    CHECK(drain(r, 0, 1000) == 1);  // relayed once, repeats pending
    CHECK(r.stored() == 1);
    Frame ack2 = ack;
    ack2.seq = 1;
    ack2.payload[2] = 0;
    CHECK(r.onReceive(ack2, 0, 2000) == Rx::AckForOthers);
    CHECK(r.stored() == 1);  // only the relayed ACK is left
    std::vector<Frame> sent;
    drain(r, 2000, 200000, &sent);
    bool onlyAcks = true;
    for (const Frame& s : sent)
      if (s.type != FrameType::Ack) onlyAcks = false;
    CHECK(onlyAcks && sent.size() == 1u + r.config().repeats);
  }
  {  // Wrong send() arguments.
    Node n = makeNode("4X0GND");
    uint8_t big[kMaxPayload + 1] = {};
    CHECK(!n.send(all, big, kMaxPayload + 1, 0));
    char bad[kCallLen];
    setCall(bad, "AB");
    CHECK(!n.send(bad, big, 1, 0));
  }
  {  // Minimum gap between our transmissions.
    Node n = makeNode("4X0GND");
    n.send(all, (const uint8_t*)"a", 1, 0);
    n.send(all, (const uint8_t*)"b", 1, 0);
    Frame f;
    CHECK(n.poll(0, f));
    CHECK(!n.poll(n.config().minGapMs - 1, f));
    CHECK(n.poll(n.config().minGapMs, f));
  }
  {  // Beacons: on time, counted, payload decodes, not stored, not relayed.
    Node n = makeNode("4X1SAT");
    n.config().beacon = true;
    std::vector<Frame> sent;
    drain(n, 0, 95000, &sent);
    CHECK(sent.size() == 3 || sent.size() == 4);
    BeaconInfo b{};
    CHECK(!sent.empty() && decodeBeacon(sent.back(), b));
    CHECK(b.number == sent.size() && b.boot == 1 && b.txCount == sent.size());
    CHECK(sent.back().hops == 0 && isBroadcast(sent.back().dest));
    CHECK(n.stored() == 0);
    Node other = makeNode("4X2SAT");
    CHECK(other.onReceive(sent.back(), -60, 0) == Rx::Beacon);
    CHECK(!other.queued());
  }
  {  // Store full: the copy transmitted most often makes room, nothing breaks.
    Node n = makeNode("4X1SAT");
    for (uint16_t i = 0; i < 10; i++)
      n.onReceive(makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, i, "x"), 0, 0);
    CHECK(n.stored() == Node::kStoreSize);
    CHECK(drain(n, 0, 300000) <= (int)(Node::kStoreSize * (1 + n.config().repeats)));
    CHECK(n.stored() == 0);
  }
  {  // A power-cycled station starts seq at 0 again; the new boot count keeps it new.
    Node n = makeNode("4X1SAT");
    CHECK(n.onReceive(makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, 0, "old"), 0, 0) == Rx::ForUs);
    CHECK(n.onReceive(makeMessage("4X0GND", "4X0GND", "ALL", 3, 2, 0, "new"), 0, 0) == Rx::ForUs);
  }
  {  // A better copy upgrades the hops of the stored one.
    Node n = makeNode("4X1SAT");
    n.onReceive(makeMessage("4X0GND", "4X3SAT", "ALL", 1, 1, 0, "x"), 0, 0);
    CHECK(drain(n, 0, 1000) == 1);
    n.onReceive(makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, 0, "x"), 0, 1000);
    std::vector<Frame> sent;
    drain(n, 1000, 200000, &sent);
    CHECK(!sent.empty() && sent[0].hops == 2);
  }
  {  // millis() wrapping around.
    Node n;
    char c[kCallLen];
    setCall(c, "4X1SAT");
    uint32_t t0 = 0xFFFFF000UL;
    n.begin(c, 1, 7, t0);
    n.config().beacon = false;
    n.onReceive(makeMessage("4X0GND", "4X0GND", "ALL", 3, 1, 0, "x"), 0, t0);
    int c2 = 0;
    Frame f;
    for (uint32_t t = t0; t != t0 + 200000; t += 10)
      if (n.poll(t, f)) c2++;
    CHECK(c2 == 1 + n.config().repeats);
  }
}

// ---------------------------------------------------------------------------
// Simulated network

struct SimConfig {
  bool collisions = true;
  double loss = 0.0;           // chance a packet is lost at one receiver
  double foreign = 0.0;        // chance per second of someone else's packet at a station
  bool forgetSeen = false;     // wipe every seen list every 500 ms
  int stallMaxMs = 15;         // loop() sometimes busy for up to this long
};

struct Air {
  int from;
  uint32_t start, end;
  std::vector<uint8_t> bytes;
};

struct MsgKey {
  std::string origin;
  uint16_t boot, seq;
  bool operator<(const MsgKey& o) const {
    if (origin != o.origin) return origin < o.origin;
    if (boot != o.boot) return boot < o.boot;
    return seq < o.seq;
  }
};

static MsgKey keyOf(const Frame& f) { return {str(f.origin), f.boot, f.seq}; }

struct Station {
  std::string call;
  Node node;
  Parser parser;
  std::vector<std::pair<uint32_t, uint8_t>> uart;  // byte, time it is readable
  size_t uartPos = 0;
  uint32_t lastRxByte = 0;
  uint32_t txEnd = 0;
  uint32_t stallUntil = 0;
  std::map<MsgKey, int> txCount;        // transmissions per message
  std::map<MsgKey, int> delivered;      // Rx::ForUs per message
  std::set<uint16_t> acked;             // our seqs confirmed by AckForUs
  uint32_t lastTx = 0;
};

class Sim {
 public:
  Sim(int n, uint32_t seed, SimConfig cfg) : st_(n), adj_(n, std::vector<bool>(n)), rng_(seed), cfg_(cfg) {}

  void link(int a, int b) { adj_[a][b] = adj_[b][a] = true; }
  bool linked(int a, int b) const { return adj_[a][b]; }
  int size() const { return (int)st_.size(); }
  Station& at(int i) { return st_[i]; }

