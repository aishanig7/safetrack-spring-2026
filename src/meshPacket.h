#pragma once
#include <stdint.h>

#define PKT_GPS  2
#define PKT_HELLO 1
#define MESH_BROADCAST 0xFF
#define MAX_ADVERT_ROUTES 5
#define PKT_ACK 0x04 


struct MeshPacket {
  uint8_t src;
  uint8_t nextHop; 
  uint8_t lastHop;
  uint8_t dst;
  uint8_t ttl;
  uint16_t seq;
  uint8_t type;
  uint8_t payload[32];
};

struct RouteAdvertisement {
  uint8_t dest;
  uint8_t hops;
};

struct HelloPayload {
  uint8_t routeCount;
  RouteAdvertisement routes[MAX_ADVERT_ROUTES];
};

struct ACKPayload {
    uint8_t originalSrc;  // who sent the original packet
    uint16_t originalSeq; // which packet we're acknowledging
};

// Routing headers stay plaintext for forwarding; only payload is encrypted
struct EncryptedMeshPacket {
  uint8_t src;
  uint8_t nextHop;
  uint8_t lastHop;
  uint8_t dst;
  uint8_t ttl;
  uint16_t seq;
  uint8_t type;

  uint8_t nonce[7];               // [NodeID(1)] + [Counter(6)]
  uint8_t encrypted_payload[32];  // AES-CCM ciphertext
  uint8_t tag[4];                 // AES-CCM authentication tag
};