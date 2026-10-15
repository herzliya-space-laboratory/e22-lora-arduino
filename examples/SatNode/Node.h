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

