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

