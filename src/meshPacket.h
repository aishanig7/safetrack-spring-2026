#pragma once
#include <stdint.h>

#define PKT_GPS  1
#define PKT_PING 2
#define PKT_PONG 3

#define MAX_PATH 8 // max possible path packet can travel (ex. total number of nodes)
#define MAX_PONGS 10 // max number of pongs node checks for

#define MESH_BROADCAST 0xFF



struct MeshPacket {
  uint8_t src;
  uint8_t dst;
  uint8_t nextHop; 

  uint8_t ttl;
  uint16_t seq;

  uint8_t type;

  uint8_t pathLen;
  uint8_t path[MAX_PATH];
  uint8_t payload[32];
};



struct PongPacket{
    uint8_t nodeID;
    uint16_t rssi;

};

