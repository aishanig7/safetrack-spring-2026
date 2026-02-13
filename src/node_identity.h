#pragma once

#include <Arduino.h>
#include <nrf.h>
#include <stdio.h>

#ifndef IS_HEAD_NODE
#define IS_HEAD_NODE 0
#endif

namespace NodeIdentity {

static bool g_initialized = false;
static uint8_t g_mac[6] = {0};
static uint8_t g_nodeId = 0;
static char g_nodeName[10] = "SAFE-0000";

inline void readMacFromFicr(uint8_t mac[6]) {
  const uint32_t low = NRF_FICR->DEVICEADDR[0];
  const uint32_t high = NRF_FICR->DEVICEADDR[1] & 0x0000FFFFUL;

  mac[0] = (uint8_t)(low & 0xFF);
  mac[1] = (uint8_t)((low >> 8) & 0xFF);
  mac[2] = (uint8_t)((low >> 16) & 0xFF);
  mac[3] = (uint8_t)((low >> 24) & 0xFF);
  mac[4] = (uint8_t)(high & 0xFF);
  mac[5] = (uint8_t)((high >> 8) & 0xFF);
}

inline uint8_t foldMacToNodeId(const uint8_t mac[6]) {
  uint8_t id = 0xA5;
  for (uint8_t i = 0; i < 6; i++) {
    id ^= mac[i];
    id = (uint8_t)((id << 1) | (id >> 7));
  }
  if (id == 0x00) {
    id = 0x01;
  }
  return id;
}

inline void initNodeIdentity() {
  if (g_initialized) {
    return;
  }

  readMacFromFicr(g_mac);

#ifdef NODE_ID_OVERRIDE
  g_nodeId = (uint8_t)(NODE_ID_OVERRIDE);
#else
  g_nodeId = foldMacToNodeId(g_mac);
#endif

  const uint16_t suffix = (uint16_t)(((uint16_t)g_mac[1] << 8) | g_mac[0]);
  snprintf(g_nodeName, sizeof(g_nodeName), "SAFE-%04X", suffix);

  g_initialized = true;
}

inline uint8_t getNodeId() {
  if (!g_initialized) {
    initNodeIdentity();
  }
  return g_nodeId;
}

inline const char* getNodeName() {
  if (!g_initialized) {
    initNodeIdentity();
  }
  return g_nodeName;
}

inline const uint8_t* getNodeMac() {
  if (!g_initialized) {
    initNodeIdentity();
  }
  return g_mac;
}

inline bool isHeadNode() {
  return (IS_HEAD_NODE != 0);
}

}
