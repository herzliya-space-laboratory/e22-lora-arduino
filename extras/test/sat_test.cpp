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

  void setup(int i, const std::string& call, bool ground, uint16_t boot = 1) {
    Station& s = st_[i];
    s.call = call;
    s.node = Node();
    char c[kCallLen];
    setCall(c, call.c_str());
    s.node.config().beacon = !ground;
    s.node.config().relay = !ground;
    s.node.begin(c, boot, (uint32_t)rng_() | 1, now_);
    s.parser = Parser();
    s.parser.setExpectRssi(true);
    s.uart.clear();
    s.uartPos = 0;
  }

  // Power cycle: RAM gone, boot count + 1.
  void powerCycle(int i) {
    Station& s = st_[i];
    bool ground = !s.node.config().relay;
    std::map<MsgKey, int> tx = s.txCount, del = s.delivered;
    setup(i, s.call, ground, (uint16_t)(s.node.boot() + 1));
    s.txCount = tx;
    s.delivered = del;
  }

  MsgKey send(int i, const std::string& dest, const std::string& text) {
    char d[kCallLen];
    setCall(d, dest.c_str());
    uint16_t seq = 0;
    bool ok = st_[i].node.send(d, (const uint8_t*)text.data(), (uint8_t)text.size(), now_, &seq);
    CHECK(ok);
    return {st_[i].call, st_[i].node.boot(), seq};
  }

  void run(uint32_t ms) {
    uint32_t until = now_ + ms;
    for (; now_ < until; now_++) step();
  }

  uint32_t now() const { return now_; }
  uint32_t lastTxAnywhere() const { return lastTxAnywhere_; }
  int collisions() const { return collisions_; }
  uint64_t transmissions() const { return transmissions_; }

 private:
  void step() {
    // Packets that finish now come out of the receivers' modules.
    for (const Air& a : air_) {
      if (a.end != now_) continue;
      for (int r = 0; r < size(); r++) {
        if (r == a.from || !adj_[a.from][r]) continue;
        if (heardBy(r, a)) deliver(r, a.bytes, a.end);
      }
    }
    air_.erase(std::remove_if(air_.begin(), air_.end(),
                              [&](const Air& a) { return a.end + 2000 < now_; }),
               air_.end());

    std::uniform_real_distribution<double> u(0, 1);
    for (int i = 0; i < size(); i++) {
      Station& s = st_[i];
      if (cfg_.foreign > 0 && u(rng_) < cfg_.foreign / 1000.0) {
        std::string junk = "4X1ZZZ hello from a PingPong " + std::to_string(rng_() % 1000) + "\n";
        if (rng_() % 4 == 0) junk[5] = (char)0xA5, junk[6] = 0x5A;  // false sync
        deliver(i, std::vector<uint8_t>(junk.begin(), junk.end()), now_);
      }
      if (now_ < s.stallUntil) continue;  // loop() busy elsewhere
      if (cfg_.stallMaxMs > 0 && rng_() % 200 == 0) s.stallUntil = now_ + rng_() % cfg_.stallMaxMs;

      // Same order as SatNode.ino loop().
      while (s.uartPos < s.uart.size() && s.uart[s.uartPos].first <= now_) {
        s.lastRxByte = now_;
        if (s.parser.feed(s.uart[s.uartPos].second, now_)) handle(i);
        s.uartPos++;
      }
      if (s.uartPos == s.uart.size()) {
        s.uart.clear();
        s.uartPos = 0;
      }
      if (s.parser.expire(now_)) handle(i);
      if (now_ < s.txEnd) continue;  // module still transmitting
      bool quiet = !s.parser.busy() && now_ - s.lastRxByte >= 30 && s.uartPos == s.uart.size();
      Frame f;
      if (quiet && s.node.poll(now_, f)) transmit(i, f);
    }
  }

  bool heardBy(int r, const Air& a) {
    std::uniform_real_distribution<double> u(0, 1);
    if (cfg_.loss > 0 && u(rng_) < cfg_.loss) return false;
    if (!cfg_.collisions) return true;
    for (const Air& o : air_) {
      if (&o == &a || o.end < a.start || o.start > a.end) continue;
      if (o.from == r || adj_[o.from][r]) {  // r was transmitting, or heard two at once
        collisions_++;
        return false;
      }
    }
    return true;
  }

  // Module output at 9600 baud: about one byte per ms, then the RSSI byte.
  void deliver(int r, const std::vector<uint8_t>& bytes, uint32_t t) {
    Station& s = st_[r];
    uint32_t start = t;
    if (!s.uart.empty()) start = std::max(start, s.uart.back().first + 1);
    for (size_t k = 0; k < bytes.size(); k++) s.uart.push_back({start + (uint32_t)k, bytes[k]});
    s.uart.push_back({start + (uint32_t)bytes.size(), (uint8_t)(256 - 40 - rng_() % 60)});
  }

  void transmit(int i, const Frame& f) {
    Station& s = st_[i];
    uint8_t buf[kMaxFrameLen];
    size_t n = encode(f, buf, sizeof buf);
    // UART into the module, then about 3.3 ms per byte at 2.4 kbps plus preamble.
    uint32_t airMs = (uint32_t)(n + n * 8 * 1000 / 2400 + 40);
    air_.push_back({i, now_ + (uint32_t)n, now_ + airMs, std::vector<uint8_t>(buf, buf + n)});
    s.txEnd = now_ + airMs;
    if (f.type != FrameType::Beacon) s.txCount[keyOf(f)]++;
    lastTxAnywhere_ = now_;
    if (f.type != FrameType::Beacon) lastMsgTx_ = now_;
    transmissions_++;
    if (cfg_.forgetSeen && now_ - lastForget_ >= 500) {
      for (Station& o : st_) o.node.forgetSeen();
      lastForget_ = now_;
    }
  }

  void handle(int i) {
    Station& s = st_[i];
    const Frame& f = s.parser.frame();
    Rx rx = s.node.onReceive(f, (int8_t)E22rssi(s.parser.rssiByte()), now_);
    if (rx == Rx::ForUs) s.delivered[keyOf(f)]++;
    if (rx == Rx::AckForUs) s.acked.insert(s.node.ackedSeq());
  }

  static int E22rssi(uint8_t b) { return b == 0 ? 0 : -(256 - (int)b); }

 public:
  uint32_t lastMsgTx() const { return lastMsgTx_; }

 private:
  std::vector<Station> st_;
  std::vector<std::vector<bool>> adj_;
  std::vector<Air> air_;
  std::mt19937 rng_;
  SimConfig cfg_;
  uint32_t now_ = 0;
  uint32_t lastTxAnywhere_ = 0;
  uint32_t lastMsgTx_ = 0;
  uint32_t lastForget_ = 0;
  int collisions_ = 0;
  uint64_t transmissions_ = 0;
};

// Hops from a station, passing only through satellites (station 0, the
// ground, does not relay).
static std::vector<int> distancesFrom(Sim& sim, int from) {
  std::vector<int> d(sim.size(), -1);
  std::vector<int> q = {from};
  d[from] = 0;
  for (size_t k = 0; k < q.size(); k++)
    for (int j = 0; j < sim.size(); j++)
      if (sim.linked(q[k], j) && d[j] < 0 && (q[k] == from || q[k] != 0)) {
        d[j] = d[q[k]] + 1;
        q.push_back(j);
      }
  return d;
}

static std::string satName(int i) {
  // 4X1SA, 4X1SB, ... 4X1SZ, 4X2SA ...
  return "4X" + std::to_string(1 + i / 26) + "S" + std::string(1, (char)('A' + i % 26));
}

// Every station transmits each message at most 1 + repeats times, and once
// everything has settled no station still holds anything to send.
static void checkBounded(Sim& sim, const char* name) {
  int maxCopies = 1 + sim.at(1 % sim.size()).node.config().repeats;
  for (int i = 0; i < sim.size(); i++) {
    for (auto& kv : sim.at(i).txCount)
      CHECK_MSG(kv.second <= maxCopies, "%s: %s sent %s#%u.%u %d times", name,
                sim.at(i).call.c_str(), kv.first.origin.c_str(), kv.first.boot, kv.first.seq,
                kv.second);
    CHECK_MSG(sim.at(i).node.stored() == 0, "%s: %s still holds %u", name,
              sim.at(i).call.c_str(), sim.at(i).node.stored());
    for (auto& kv : sim.at(i).delivered)
      CHECK_MSG(kv.second == 1, "%s: %s got %s#%u.%u %d times", name, sim.at(i).call.c_str(),
                kv.first.origin.c_str(), kv.first.boot, kv.first.seq, kv.second);
  }
}

// Ground station 0, satellites 1..4 in a line: 0 - 1 - 2 - 3 - 4.
static void testLine() {
  SimConfig cfg;
  cfg.collisions = false;
  Sim sim(5, 11, cfg);
  sim.setup(0, "4X0GND", true);
  for (int i = 1; i < 5; i++) sim.setup(i, satName(i), false);
  for (int i = 0; i < 4; i++) sim.link(i, i + 1);
  sim.run(5000);
  MsgKey m = sim.send(0, "ALL", "hello chain");
  sim.run(180000);
  for (int i = 1; i < 5; i++)
    CHECK_MSG(sim.at(i).delivered.count(m) == 1, "line: %s did not get the broadcast",
              sim.at(i).call.c_str());
  checkBounded(sim, "line broadcast");

  // To the far end, 4 hops away with hops = 3: ACK comes back, ground stops repeating.
  MsgKey d = sim.send(0, satName(4), "to the end");
  sim.run(180000);
  CHECK(sim.at(4).delivered.count(d) == 1);
  CHECK(sim.at(0).acked.count(d.seq) == 1);
  CHECK_MSG(sim.at(0).txCount[d] == 1, "ground sent the addressed message %d times",
            sim.at(0).txCount[d]);
  for (int i = 1; i < 4; i++) CHECK(sim.at(i).delivered.count(d) == 0);
  checkBounded(sim, "line addressed");

  // 5 hops away is out of reach with hops = 3: the message dies at station 4.
  Sim far(6, 12, cfg);
  far.setup(0, "4X0GND", true);
  for (int i = 1; i < 6; i++) far.setup(i, satName(i), false);
  for (int i = 0; i < 5; i++) far.link(i, i + 1);
  MsgKey f = far.send(0, "ALL", "too far");
  far.run(180000);
  CHECK(far.at(4).delivered.count(f) == 1);
  CHECK(far.at(5).delivered.count(f) == 0);
  checkBounded(far, "line too far");
}

// Satellites in a ring, so every message comes back around: it must still stop.
static void testRingAndMesh() {
  for (int collisions = 0; collisions < 2; collisions++) {
    SimConfig cfg;
    cfg.collisions = collisions;
    Sim ring(7, 21 + collisions, cfg);
    ring.setup(0, "4X0GND", true);
    for (int i = 1; i < 7; i++) ring.setup(i, satName(i), false);
    ring.link(0, 1);
    for (int i = 1; i < 6; i++) ring.link(i, i + 1);
    ring.link(6, 1);
    MsgKey m = ring.send(0, "ALL", "around");
    ring.run(240000);
    for (int i = 1; i < 7; i++)
      CHECK_MSG(ring.at(i).delivered.count(m) == 1, "ring (collisions %d): %s missed it",
                collisions, ring.at(i).call.c_str());
    checkBounded(ring, "ring");

    Sim mesh(8, 31 + collisions, cfg);
    mesh.setup(0, "4X0GND", true);
    for (int i = 1; i < 8; i++) mesh.setup(i, satName(i), false);
    for (int a = 0; a < 8; a++)
      for (int b = a + 1; b < 8; b++) mesh.link(a, b);
    std::vector<MsgKey> msgs;
    for (int k = 0; k < 3; k++) {
      msgs.push_back(mesh.send(0, "ALL", "mesh " + std::to_string(k)));
      mesh.run(2000);
    }
    MsgKey to5 = mesh.send(0, satName(5), "just you");
    mesh.run(240000);
    for (const MsgKey& m2 : msgs)
      for (int i = 1; i < 8; i++)
        CHECK_MSG(mesh.at(i).delivered.count(m2) == 1, "mesh (collisions %d): %s missed #%u",
                  collisions, mesh.at(i).call.c_str(), m2.seq);
    CHECK(mesh.at(5).delivered.count(to5) == 1);
    CHECK(mesh.at(0).acked.count(to5.seq) == 1);
    checkBounded(mesh, "mesh");
  }
}

// Every station forgets what it has seen twice a second: only the hop limit
// is left to stop the messages, and it has to.
static void testForgetful() {
  SimConfig cfg;
  cfg.collisions = false;
  cfg.forgetSeen = true;
  Sim sim(6, 41, cfg);
  sim.setup(0, "4X0GND", true);
  for (int i = 1; i < 6; i++) sim.setup(i, satName(i), false);
  for (int a = 0; a < 6; a++)
    for (int b = a + 1; b < 6; b++) sim.link(a, b);
  sim.send(0, "ALL", "forgotten");
  sim.run(600000);
  uint32_t quietFor = sim.now() - sim.lastMsgTx();
  CHECK_MSG(quietFor > 300000, "messages still going %u ms before the end", sim.now() - sim.lastMsgTx());
  for (int i = 0; i < 6; i++) CHECK(sim.at(i).node.stored() == 0);
}

// A satellite is power-cycled in the middle; its new messages (seq from 0
// again) still get through.
static void testPowerCycle() {
  SimConfig cfg;
  cfg.collisions = false;
  Sim sim(4, 51, cfg);
  sim.setup(0, "4X0GND", true);
  for (int i = 1; i < 4; i++) sim.setup(i, satName(i), false);
  for (int i = 0; i < 3; i++) sim.link(i, i + 1);
  MsgKey before = sim.send(3, "4X0GND", "before");
  sim.run(60000);
  CHECK(sim.at(0).delivered.count(before) == 1);
  sim.powerCycle(3);
  MsgKey after = sim.send(3, "4X0GND", "after");
  CHECK(after.seq == before.seq && after.boot == before.boot + 1);
  sim.run(60000);
  CHECK(sim.at(0).delivered.count(after) == 1);
  CHECK(sim.at(3).acked.count(after.seq) == 1);
}

// Random networks, random traffic, collisions, lost packets, foreign traffic.
// Always: every message stops and nobody sends one too often. Without
// collisions and loss: every station in reach gets every broadcast, and every
// addressed message arrives and is confirmed.
static void testRandom() {
  int runs = 0;
  int cleanReachable = 0, cleanReached = 0, noisyReachable = 0, noisyReached = 0;
  for (uint32_t seed = 1; seed <= 300; seed++) {
    std::mt19937 rng(seed * 7919);
    int n = 3 + (int)(rng() % 8);
    SimConfig cfg;
    cfg.collisions = rng() % 3 != 0;
    cfg.loss = (rng() % 3) * 0.1;
    cfg.foreign = (rng() % 2) * 0.2;
    bool clean = !cfg.collisions && cfg.loss == 0;
    Sim sim(n, seed, cfg);
    sim.setup(0, "4X0GND", true);
    for (int i = 1; i < n; i++) sim.setup(i, satName(i), false);
    for (int a = 0; a < n; a++)
      for (int b = a + 1; b < n; b++)
        if (rng() % 100 < 35) sim.link(a, b);

    struct Sent {
      int from, to;  // to = -1: ALL
      MsgKey key;
    };
    std::vector<Sent> sent;
    int count = 1 + (int)(rng() % 4);
    for (int k = 0; k < count; k++) {
      int from = (int)(rng() % n);
      int to = -1;
      if (rng() % 2) to = from == 0 ? 1 + (int)(rng() % (n - 1)) : 0;
      std::string dest = to < 0 ? "ALL" : sim.at(to).call;
      sent.push_back({from, to, sim.send(from, dest, "random " + std::to_string(k))});
      sim.run(rng() % 20000);
    }
    sim.run(300000);
    char name[32];
    std::snprintf(name, sizeof name, "random seed %u", seed);
    checkBounded(sim, name);

    for (const Sent& s : sent) {
      std::vector<int> d = distancesFrom(sim, s.from);
      int hops = sim.at(s.from).node.config().hops;
      for (int i = 0; i < n; i++) {
        if (i == s.from || d[i] < 0 || d[i] > hops + 1) continue;
        if (s.to >= 0 ? i != s.to : i == 0) continue;  // the ground only takes messages to it
        bool got = sim.at(i).delivered.count(s.key) == 1;
        (clean ? cleanReachable : noisyReachable)++;
        (clean ? cleanReached : noisyReached) += got;
        if (clean) {
          CHECK_MSG(got, "%s: %s (%d hops away) missed %s#%u.%u", name, sim.at(i).call.c_str(),
                    d[i], s.key.origin.c_str(), s.key.boot, s.key.seq);
          if (s.to >= 0) CHECK_MSG(sim.at(s.from).acked.count(s.key.seq) == 1, "%s: no ACK", name);
        }
      }
    }
    runs++;
  }
  std::printf("random: %d networks. Delivered in reach: %d/%d clean, %d/%d with collisions/loss\n",
              runs, cleanReached, cleanReachable, noisyReached, noisyReachable);
}

int main() {
  testCrc();
  testEncodeDecode();
  testCallsigns();
  testParser();
  testNodeRules();
  testLine();
  testRingAndMesh();
  testForgetful();
  testPowerCycle();
  testRandom();
  std::printf("%d checks, %d failed\n", checks, failures);
  return failures ? 1 : 0;
}
