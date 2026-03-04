#pragma once
#include <stdint.h>

#define PKT_GPS  2
#define PKT_HELLO 1
#define MESH_BROADCAST 0xFF
#define MAX_ADVERT_ROUTES 5

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