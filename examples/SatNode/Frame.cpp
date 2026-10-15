#include "Frame.h"

#include <string.h>

namespace sat {

const char kBroadcast[kCallLen] = {'A', 'L', 'L', ' ', ' ', ' '};

namespace {

void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

bool validType(uint8_t t) {
  return t >= (uint8_t)FrameType::Beacon && t <= (uint8_t)FrameType::Ack;
}

}  // namespace

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

size_t encode(const Frame& f, uint8_t* out, size_t outLen) {
  if (f.len > kMaxPayload) return 0;
  size_t n = kHeaderLen + f.len + kCrcLen;
  if (outLen < n) return 0;
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = (uint8_t)((kVersion << 4) | ((uint8_t)f.type & 0x0F));
  out[3] = f.hops;
  memcpy(out + 4, f.origin, kCallLen);
  memcpy(out + 10, f.sender, kCallLen);
  memcpy(out + 16, f.dest, kCallLen);
  put16(out + 22, f.boot);
  put16(out + 24, f.seq);
  out[26] = f.len;
  memcpy(out + kHeaderLen, f.payload, f.len);
  put16(out + kHeaderLen + f.len, crc16(out + 2, kHeaderLen - 2 + f.len));
  return n;
}

bool decode(const uint8_t* in, size_t len, Frame& f) {
  if (len < kHeaderLen + kCrcLen) return false;
  if (in[0] != kSync0 || in[1] != kSync1) return false;
  if ((in[2] >> 4) != kVersion || !validType(in[2] & 0x0F)) return false;
  uint8_t payloadLen = in[26];
  if (payloadLen > kMaxPayload || len != kHeaderLen + payloadLen + kCrcLen) return false;
  if (crc16(in + 2, kHeaderLen - 2 + payloadLen) != get16(in + kHeaderLen + payloadLen)) return false;

  f.type = (FrameType)(in[2] & 0x0F);
  f.hops = in[3];
  memcpy(f.origin, in + 4, kCallLen);
  memcpy(f.sender, in + 10, kCallLen);
  memcpy(f.dest, in + 16, kCallLen);
  f.boot = get16(in + 22);
  f.seq = get16(in + 24);
  f.len = payloadLen;
  memcpy(f.payload, in + kHeaderLen, payloadLen);
  return true;
}

bool setCall(char out[kCallLen], const char* text) {
  char tmp[kCallLen];
  size_t n = 0;
  for (; text[n]; n++) {
    if (n == kCallLen) return false;
    char c = text[n];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
    tmp[n] = c;
  }
  if (n == 0) return false;
  for (size_t i = n; i < kCallLen; i++) tmp[i] = ' ';
  memcpy(out, tmp, kCallLen);
  return true;
}

void callToString(const char call[kCallLen], char out[kCallLen + 1]) {
  size_t n = 0;
  while (n < kCallLen && call[n] != ' ') {
    out[n] = call[n];
    n++;
  }
  out[n] = '\0';
}

bool callEquals(const char a[kCallLen], const char b[kCallLen]) {
  return memcmp(a, b, kCallLen) == 0;
}

bool isBroadcast(const char call[kCallLen]) { return callEquals(call, kBroadcast); }

bool isStation(const char call[kCallLen]) {
  size_t n = 0;
  bool digit = false, letter = false;
  for (; n < kCallLen && call[n] != ' '; n++) {
    char c = call[n];
    if (c >= '0' && c <= '9') digit = true;
    else if (c >= 'A' && c <= 'Z') letter = true;
    else return false;
  }
  for (size_t i = n; i < kCallLen; i++)
    if (call[i] != ' ') return false;
  return n >= 3 && digit && letter;
}

bool isIsraeli(const char call[kCallLen]) {
  return call[0] == '4' && (call[1] == 'X' || call[1] == 'Z');
}

void Parser::reset() {
  state_ = State::Sync0;
  n_ = 0;
}

bool Parser::feed(uint8_t b, uint32_t nowMs) {
  lastMs_ = nowMs;
  switch (state_) {
    case State::Sync0:
      if (b == kSync0) {
        buf_[0] = b;
        n_ = 1;
        state_ = State::Sync1;
      }
      return false;
    case State::Sync1:
      if (b == kSync1) {
        buf_[1] = b;
        n_ = 2;
        state_ = State::Body;
      } else if (b != kSync0) {
        state_ = State::Sync0;
      }
      return false;
    case State::Body:
      buf_[n_++] = b;
      return check();
    case State::Rssi:
      rssi_ = b;
      state_ = State::Sync0;
      return true;
  }
  return false;
}

bool Parser::expire(uint32_t nowMs) {
  if (state_ == State::Sync0 || nowMs - lastMs_ < kGapMs) return false;
  if (state_ == State::Rssi) {  // good frame, only the RSSI byte is missing
    reset();
    return true;
  }
  // Nothing more is coming, so what we have is not one frame. After a false
  // sync a real frame can still be inside it.
  if (state_ == State::Body) dropped_++;
  while (state_ == State::Body) {
    resync();
    if (check()) return true;
    if (state_ == State::Rssi) {
      reset();
      return true;
    }
  }
  reset();
  return false;
}

// Looks at what has been collected. Returns true when a whole frame is in and
// no RSSI byte is expected; with an RSSI byte, feed() returns true one byte later.
bool Parser::check() {
  while (state_ == State::Body) {
    if (n_ > 2 && ((buf_[2] >> 4) != kVersion || !validType(buf_[2] & 0x0F))) {
      resync();
      continue;
    }
    if (n_ < kHeaderLen) return false;
    if (buf_[26] > kMaxPayload) {
      resync();
      continue;
    }
    size_t need = kHeaderLen + buf_[26] + kCrcLen;
    if (n_ < need) return false;
    if (!decode(buf_, need, frame_)) {
      dropped_++;
      resync();
      continue;
    }
    rssi_ = 0;
    if (!expectRssi_) {
      reset();
      return true;
    }
    if (n_ > need) {  // only after a resync: the RSSI byte is already here
      rssi_ = buf_[need];
      reset();
      return true;
    }
    state_ = State::Rssi;
    return false;
  }
  return false;
}

// The bytes collected so far are not a frame. A frame may still start inside
// them (a false sync in noise or in someone else's data), so start again from
// the next A5 5A instead of throwing everything away.
void Parser::resync() {
  for (uint8_t i = 1; i + 1 < n_; i++) {
    if (buf_[i] == kSync0 && buf_[i + 1] == kSync1) {
      memmove(buf_, buf_ + i, n_ - i);
      n_ -= i;
      return;  // still State::Body
    }
  }
  if (n_ > 0 && buf_[n_ - 1] == kSync0) {
    buf_[0] = kSync0;
    n_ = 1;
    state_ = State::Sync1;
  } else {
    reset();
  }
}

}  // namespace sat
