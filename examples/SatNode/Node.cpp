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

void Node::begin(const char call[kCallLen], uint16_t boot, uint32_t seed, uint32_t nowMs) {
  setCall(call);
  boot_ = boot;
  rng_ = seed ? seed : 1;
  startMs_ = nowMs;
  // First beacon a few seconds after power-up, at a random moment so that
  // stations switched on together do not beacon together.
  nextBeaconMs_ = nowMs + 1000 + random(3000);
}

void Node::setCall(const char call[kCallLen]) { memcpy(call_, call, kCallLen); }

Rx Node::onReceive(const Frame& f, int8_t rssiDbm, uint32_t nowMs) {
  queued_ = false;
  rxCount_++;
  if (f.hops > kMaxHops || !isStation(f.origin) || !isStation(f.sender) ||
      !(isBroadcast(f.dest) || isStation(f.dest)))
    return Rx::Invalid;
  if (cfg_.israeliOnly && !(isIsraeli(f.origin) && isIsraeli(f.sender))) return Rx::NotIsraeli;
  lastRssi_ = rssiDbm;
  if (callEquals(f.origin, call_)) return Rx::Own;

  // A beacon that nobody relays reaches us once, so it does not need a
  // place in the seen list.
  if (f.type != FrameType::Beacon || f.hops > 0) {
    uint32_t id = messageId(f.origin, f.boot, f.seq);
    if (seen(id)) {
      duplicates_++;
      upgrade(f);
      return Rx::Duplicate;
    }
    remember(id);
  }

  bool toUs = callEquals(f.dest, call_);
  Rx result;
  switch (f.type) {
    case FrameType::Beacon:
      result = Rx::Beacon;
      break;
    case FrameType::Message:
      result = toUs || isBroadcast(f.dest) ? Rx::ForUs : Rx::ForOthers;
      if (toUs) {
        Frame ack;
        makeFrame(ack, FrameType::Ack, f.origin, cfg_.hops);
        put16(ack.payload, f.boot);
        put16(ack.payload + 2, f.seq);
        ack.len = 4;
        queue(ack, nowMs + backoff());
      }
      break;
    case FrameType::Ack:
    default:
      if (f.len < 4) return Rx::Invalid;
      // The message got there: stop repeating it, here and at every station
      // the ACK passes.
      cancel(f.dest, get16(f.payload), get16(f.payload + 2));
      if (toUs) {
        ackedSeq_ = get16(f.payload + 2);
        return Rx::AckForUs;
      }
      result = Rx::AckForOthers;
      break;
  }

  if (cfg_.relay && f.hops > 0 && !toUs) {
    Frame copy = f;
    copy.hops--;
    queue(copy, nowMs + backoff());
    relayed_++;
    queued_ = true;
  }
  return result;
}

bool Node::send(const char dest[kCallLen], const uint8_t* data, uint8_t len, uint32_t nowMs,
                uint16_t* seqOut) {
  if (len > kMaxPayload || !(isBroadcast(dest) || isStation(dest))) return false;
  Frame f;
  makeFrame(f, FrameType::Message, dest, cfg_.hops);
  memcpy(f.payload, data, len);
  f.len = len;
  queue(f, nowMs);
  if (seqOut) *seqOut = f.seq;
  return true;
}

bool Node::poll(uint32_t nowMs, Frame& out) {
  if (sentAny_ && nowMs - lastTxMs_ < cfg_.minGapMs) return false;

  Slot* next = nullptr;
  for (Slot& s : store_) {
    if (s.left == 0 || !reached(nowMs, s.dueMs)) continue;
    if (!next || (int32_t)(s.dueMs - next->dueMs) < 0) next = &s;
  }
  if (next) {
    out = next->frame;
    next->left--;
    next->dueMs = nowMs + cfg_.repeatIntervalMs + random(cfg_.repeatIntervalMs / 4 + 1);
  } else if (cfg_.beacon && reached(nowMs, nextBeaconMs_)) {
    makeFrame(out, FrameType::Beacon, kBroadcast, cfg_.beaconHops);
    uint8_t* p = out.payload;
    put32(p, ++beaconNumber_);
    put32(p + 4, (nowMs - startMs_) / 1000);
    put16(p + 8, boot_);
    put16(p + 10, (uint16_t)(txCount_ + 1));  // this beacon included
    put16(p + 12, rxCount_);
    put16(p + 14, relayed_);
    p[16] = stored();
    p[17] = (uint8_t)lastRssi_;
    out.len = kBeaconLen;
    nextBeaconMs_ = nowMs + cfg_.beaconIntervalMs + random(cfg_.beaconIntervalMs / 10 + 1);
  } else {
    return false;
  }

  memcpy(out.sender, call_, kCallLen);
  txCount_++;
  lastTxMs_ = nowMs;
  sentAny_ = true;
  return true;
}

uint8_t Node::stored() const {
  uint8_t n = 0;
  for (const Slot& s : store_)
    if (s.left) n++;
  return n;
}

void Node::forgetSeen() {
  memset(seen_, 0, sizeof seen_);
  seenNext_ = 0;
}

// FNV-1a. 0 marks an empty entry in seen_, so it is never returned.
uint32_t Node::messageId(const char origin[kCallLen], uint16_t boot, uint16_t seq) {
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < kCallLen; i++) h = (h ^ (uint8_t)origin[i]) * 16777619UL;
  uint8_t tail[4] = {(uint8_t)boot, (uint8_t)(boot >> 8), (uint8_t)seq, (uint8_t)(seq >> 8)};
  for (uint8_t b : tail) h = (h ^ b) * 16777619UL;
  return h ? h : 1;
}

bool Node::seen(uint32_t id) const {
  for (uint32_t s : seen_)
    if (s == id) return true;
  return false;
}

void Node::remember(uint32_t id) {
  seen_[seenNext_] = id;
  seenNext_ = (uint8_t)((seenNext_ + 1) % kSeenSize);
}

// Puts a frame in the store for 1 + repeats transmissions. When the store is
// full, the copy that has been transmitted most often makes room.
void Node::queue(const Frame& f, uint32_t dueMs) {
  Slot* slot = &store_[0];
  for (Slot& s : store_) {
    if (s.left < slot->left) slot = &s;
    if (s.left == 0) break;
  }
  slot->frame = f;
  slot->dueMs = dueMs;
  slot->left = (uint8_t)(1 + cfg_.repeats);
}

void Node::cancel(const char origin[kCallLen], uint16_t boot, uint16_t seq) {
  for (Slot& s : store_) {
    if (s.left && s.frame.type == FrameType::Message && s.frame.boot == boot &&
        s.frame.seq == seq && callEquals(s.frame.origin, origin))
      s.left = 0;
  }
}

// We heard a message first along a longer path, and now a copy with more
// hops left. Our next transmissions of it carry the larger number, so the
// message still gets as far as the shortest path allows.
void Node::upgrade(const Frame& f) {
  if (f.hops == 0) return;
  for (Slot& s : store_) {
    if (s.left && s.frame.type == f.type && s.frame.boot == f.boot && s.frame.seq == f.seq &&
        callEquals(s.frame.origin, f.origin) && s.frame.hops + 1 < f.hops)
      s.frame.hops = (uint8_t)(f.hops - 1);
  }
}

void Node::makeFrame(Frame& f, FrameType type, const char dest[kCallLen], uint8_t hops) {
  f.type = type;
  f.hops = hops;
  memcpy(f.origin, call_, kCallLen);
  memcpy(f.sender, call_, kCallLen);
  memcpy(f.dest, dest, kCallLen);
  f.boot = boot_;
  f.seq = seq_++;
  f.len = 0;
}

// xorshift32
uint32_t Node::random(uint32_t range) {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return range ? rng_ % range : 0;
}

uint32_t Node::backoff() {
  return cfg_.backoffMinMs + random((uint32_t)(cfg_.backoffMaxMs - cfg_.backoffMinMs) + 1);
}

}  // namespace sat
