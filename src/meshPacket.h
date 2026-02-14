#pragma once

#include <Arduino.h>

constexpr uint8_t MESH_BROADCAST = 0xFF;

enum MeshPacketType : uint8_t {
  PKT_GPS = 1,
  PKT_HELLO = 2,
};

struct MeshPacket {
  uint8_t src;
  uint8_t lastHop;
  uint8_t dst;
  uint8_t ttl;
  uint16_t seq;
  uint8_t type;
  uint8_t payload[32];
};
