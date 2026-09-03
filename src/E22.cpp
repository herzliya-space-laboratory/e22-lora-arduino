#include "E22.h"

// ---------------------------------------------------------------------------
// E22Config

void E22Config::toBytes(uint8_t out[kBytes]) const {
  out[0] = (uint8_t)(address >> 8);
  out[1] = (uint8_t)(address & 0xFF);
  out[2] = netId;
  out[3] = (uint8_t)(((uint8_t)uartBaud & 0x07) << 5) |
           (uint8_t)(((uint8_t)parity & 0x03) << 3) |
           (uint8_t)((uint8_t)airRate & 0x07);
  out[4] = (uint8_t)(((uint8_t)packetSize & 0x03) << 6) |
           (uint8_t)(ambientRssi ? 0x20 : 0) |
           (uint8_t)(softwareModeSwitch ? 0x04 : 0) |
           (uint8_t)((uint8_t)txPower & 0x03);
  out[5] = channel;
  out[6] = (uint8_t)(rssiByte ? 0x80 : 0) |
           (uint8_t)(fixedPoint ? 0x40 : 0) |
           (uint8_t)(relay ? 0x20 : 0) |
           (uint8_t)(lbt ? 0x10 : 0) |
           (uint8_t)(worTransmitter ? 0x08 : 0) |
           (uint8_t)(worCycle & 0x07);
  out[7] = (uint8_t)(cryptKey >> 8);
  out[8] = (uint8_t)(cryptKey & 0xFF);
}

E22Config E22Config::fromBytes(const uint8_t in[kBytes]) {
  E22Config c;
  c.address = (uint16_t)((in[0] << 8) | in[1]);
  c.netId = in[2];
  c.uartBaud = (E22UartBaud)((in[3] >> 5) & 0x07);
  uint8_t par = (in[3] >> 3) & 0x03;
  c.parity = (par == 3) ? E22Parity::N8 : (E22Parity)par;  // 0b11 == 8N1
  c.airRate = (E22AirRate)(in[3] & 0x07);
  c.packetSize = (E22PacketSize)((in[4] >> 6) & 0x03);
  c.ambientRssi = in[4] & 0x20;
  c.softwareModeSwitch = in[4] & 0x04;
  c.txPower = (E22TxPower)(in[4] & 0x03);
  c.channel = in[5];
  c.rssiByte = in[6] & 0x80;
  c.fixedPoint = in[6] & 0x40;
  c.relay = in[6] & 0x20;
  c.lbt = in[6] & 0x10;
  c.worTransmitter = in[6] & 0x08;
  c.worCycle = in[6] & 0x07;
  c.cryptKey = 0;  // write-only register, always reads as 0
  return c;
}

