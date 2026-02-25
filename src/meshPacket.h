#pragma once
#include <stdint.h>

#define PKT_GPS  2
#define PKT_HELLO 1
#define MESH_BROADCAST 0xFF

struct MeshPacket {
  uint8_t src;
  uint8_t lastHop;
  uint8_t dst;
  uint8_t ttl;
  uint16_t seq;
  uint8_t type;
  uint8_t payload[32];
};

// Encrypted packet structure for over-the-air transmission (51 bytes)
struct EncryptedMeshPacket {
  // Plaintext header (11 bytes) - routing info, not encrypted
  uint8_t src;
  uint8_t lastHop;
  uint8_t dst;
  uint8_t ttl;
  uint16_t seq;
  uint8_t type;
  uint8_t nonce[7];

  // Encrypted section (40 bytes)
  uint8_t encrypted_payload[32];
  uint8_t tag[4];
};

