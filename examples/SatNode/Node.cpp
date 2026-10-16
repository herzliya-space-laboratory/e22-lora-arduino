#include "Node.h"

#include <string.h>

namespace sat {

namespace {

void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

void put32(uint8_t* p, uint32_t v) {
  put16(p, (uint16_t)v);
  put16(p + 2, (uint16_t)(v >> 16));
}

uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

uint32_t get32(const uint8_t* p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

// Time comparison that survives millis() wrapping around.
bool reached(uint32_t nowMs, uint32_t dueMs) { return (int32_t)(nowMs - dueMs) >= 0; }

}  // namespace

bool decodeBeacon(const Frame& f, BeaconInfo& out) {
  if (f.type != FrameType::Beacon || f.len < kBeaconLen) return false;
  const uint8_t* p = f.payload;
  out.number = get32(p);
  out.uptimeS = get32(p + 4);
  out.boot = get16(p + 8);
  out.txCount = get16(p + 10);
  out.rxCount = get16(p + 12);
  out.relayed = get16(p + 14);
  out.stored = p[16];
  out.lastRssi = (int8_t)p[17];
  return true;
}

