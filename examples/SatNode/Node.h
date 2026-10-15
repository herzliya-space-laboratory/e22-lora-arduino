// What one station (satellite or ground) does with frames: beacons, sending
// messages, relaying other stations' messages, ACKs. Plain C++ without
// Arduino calls; the sketch passes in the time and moves the bytes, and the
// host tests in extras/test run many Nodes in a simulated network.
//
// Relaying (flooding with a hop limit):
//   - A station relays every frame it hears for the first time if hops > 0,
//     with hops - 1, after a random wait so that neighbours do not transmit
//     at the same moment. It keeps the copy and transmits it `repeats` more
//     times, `repeatIntervalMs` apart, for stations that come into range later.
//   - It remembers the names (origin, boot, seq) of the last kSeenSize
//     messages and ignores them when they come back, and it ignores its own
//     messages coming back.
//   - If a copy with more hops left arrives while ours is still waiting to be
//     repeated, our repeats carry the larger number minus one.
//   - The hop limit ends every message even if a station has forgotten it:
//     each station that takes a message in transmits it with fewer hops than
//     it got, so the chain of stations passing it on is at most hops + 1 long,
//     and each of them transmits it at most 1 + repeats times.
//   - A message to one station is answered with an ACK, which floods back the
//     same way. An ACK stops the repeats of that message at every station
//     that hears it.

#pragma once

#include "Frame.h"

namespace sat {

struct NodeConfig {
  bool beacon = true;
  uint32_t beaconIntervalMs = 30000;
  uint8_t beaconHops = 0;            // 0: beacons are not relayed
  bool relay = true;
  uint8_t hops = 3;                  // hops left on messages and ACKs we write
  uint8_t repeats = 2;               // copies after the first one
  uint32_t repeatIntervalMs = 20000;
  uint16_t backoffMinMs = 50;        // random wait before a relay or an ACK
  uint16_t backoffMaxMs = 600;
  uint16_t minGapMs = 300;           // between two of our transmissions
  bool israeliOnly = true;           // origin and sender must be 4X / 4Z
};

// What onReceive() did with a frame.
enum class Rx : uint8_t {
  Invalid,       // bad callsign or too many hops
  NotIsraeli,    // origin or sender is not 4X / 4Z, and israeliOnly is on
  Own,           // one of our own messages, relayed back to us
  Duplicate,     // already seen
  Beacon,        // another station's beacon
  ForUs,         // a message to us (ACK queued) or to ALL
  ForOthers,     // a message to another station
  AckForUs,      // the destination got one of our messages, see ackedSeq()
  AckForOthers,
};

// Beacon payload, little endian, 18 bytes.
struct BeaconInfo {
  uint32_t number;   // beacons sent since boot
  uint32_t uptimeS;
  uint16_t boot;
  uint16_t txCount;  // frames sent since boot, all types
  uint16_t rxCount;  // frames received since boot, all types
  uint16_t relayed;  // copies queued for relay since boot
  uint8_t stored;    // messages waiting in the store
  int8_t lastRssi;   // dBm of the last frame received, 0 = none
};
constexpr uint8_t kBeaconLen = 18;
bool decodeBeacon(const Frame& f, BeaconInfo& out);

