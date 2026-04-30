#ifndef CRYPTO_CONFIG_H
#define CRYPTO_CONFIG_H

#include <stdint.h>

// WARNING: For development only. Use UICR storage in production.
// This is a test key - replace with your own secure key for production
const uint8_t NETWORK_KEY[16] = {
  0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
  0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

#endif
