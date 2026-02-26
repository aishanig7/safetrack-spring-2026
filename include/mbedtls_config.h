#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

// Minimal mbedtls configuration for AES-CCM on nRF52840
// This config enables ONLY AES-CCM and disables all other crypto

// System support
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_PLATFORM_NO_STD_FUNCTIONS

// AES support (required for CCM)
#define MBEDTLS_AES_C
#define MBEDTLS_AES_ROM_TABLES

// CCM mode support (our actual requirement)
#define MBEDTLS_CCM_C

// Cipher support (required for CCM)
#define MBEDTLS_CIPHER_C

// Explicitly disable ALL elliptic curve / X25519 / Everest features
#undef MBEDTLS_ECDH_C
#undef MBEDTLS_ECDSA_C
#undef MBEDTLS_ECJPAKE_C
#undef MBEDTLS_ECP_C
#undef MBEDTLS_ECDH_LEGACY_CONTEXT
#undef MBEDTLS_ECP_DP_SECP192R1_ENABLED
#undef MBEDTLS_ECP_DP_SECP224R1_ENABLED
#undef MBEDTLS_ECP_DP_SECP256R1_ENABLED
#undef MBEDTLS_ECP_DP_SECP384R1_ENABLED
#undef MBEDTLS_ECP_DP_SECP521R1_ENABLED
#undef MBEDTLS_ECP_DP_SECP192K1_ENABLED
#undef MBEDTLS_ECP_DP_SECP224K1_ENABLED
#undef MBEDTLS_ECP_DP_SECP256K1_ENABLED
#undef MBEDTLS_ECP_DP_BP256R1_ENABLED
#undef MBEDTLS_ECP_DP_BP384R1_ENABLED
#undef MBEDTLS_ECP_DP_BP512R1_ENABLED
#undef MBEDTLS_ECP_DP_CURVE25519_ENABLED
#undef MBEDTLS_ECP_DP_CURVE448_ENABLED

// Disable other unwanted features
#undef MBEDTLS_FS_IO
#undef MBEDTLS_NET_C
#undef MBEDTLS_TIMING_C
#undef MBEDTLS_RSA_C
#undef MBEDTLS_DHM_C
#undef MBEDTLS_GCM_C
#undef MBEDTLS_CHACHA20_C
#undef MBEDTLS_POLY1305_C
#undef MBEDTLS_CHACHAPOLY_C
#undef MBEDTLS_SSL_TLS_C
#undef MBEDTLS_X509_USE_C
#undef MBEDTLS_PK_C

// Memory allocation functions
#include <stdlib.h>
#define MBEDTLS_PLATFORM_STD_CALLOC   calloc
#define MBEDTLS_PLATFORM_STD_FREE     free

// Check config
#include "mbedtls/check_config.h"

#endif /* MBEDTLS_CONFIG_H */
