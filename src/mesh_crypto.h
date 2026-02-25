#ifndef MESH_CRYPTO_H
#define MESH_CRYPTO_H

#include <Arduino.h>
#include <mbedtls/ccm.h>
#include "meshPacket.h"
#include "crypto_config.h"

// Global packet counter for nonce generation
static uint64_t g_packetCounter = 0;

// CCM context for encryption/decryption
static mbedtls_ccm_context g_ccm_ctx;
static bool g_crypto_initialized = false;

/**
 * Initialize the crypto system
 * Must be called once during setup
 */
bool initCrypto() {
  if (g_crypto_initialized) {
    return true;
  }

  mbedtls_ccm_init(&g_ccm_ctx);

  int ret = mbedtls_ccm_setkey(&g_ccm_ctx, MBEDTLS_CIPHER_ID_AES, NETWORK_KEY, 128);
  if (ret != 0) {
    Serial.print("[CRYPTO] Failed to set key: ");
    Serial.println(ret);
    return false;
  }

  g_crypto_initialized = true;
  Serial.println("[CRYPTO] Initialized successfully");
  return true;
}

/**
 * Build a unique 7-byte nonce from node ID and packet counter
 * Structure: [Node ID (1 byte)] [48-bit Counter (6 bytes)]
 */
void buildNonce(uint8_t nodeId, uint8_t nonce[7]) {
  nonce[0] = nodeId;
  uint64_t counter = g_packetCounter++;

  // Pack counter into bytes 1-6 (little-endian)
  for (int i = 0; i < 6; i++) {
    nonce[i + 1] = (counter >> (i * 8)) & 0xFF;
  }
}

/**
 * Encrypt a mesh packet using AES-CCM
 *
 * @param plain Plaintext packet to encrypt
 * @param encrypted Output encrypted packet
 * @return true if encryption succeeded, false otherwise
 */
bool encryptMeshPacket(const MeshPacket* plain, EncryptedMeshPacket* encrypted) {
  if (!g_crypto_initialized) {
    Serial.println("[CRYPTO] Not initialized");
    return false;
  }

  // Copy plaintext header
  encrypted->src = plain->src;
  encrypted->lastHop = plain->lastHop;
  encrypted->dst = plain->dst;
  encrypted->ttl = plain->ttl;
  encrypted->seq = plain->seq;
  encrypted->type = plain->type;

  // Generate nonce
  buildNonce(plain->src, encrypted->nonce);

  // Build AAD (Additional Authenticated Data) from header fields
  uint8_t aad[7];
  aad[0] = plain->src;
  aad[1] = plain->lastHop;
  aad[2] = plain->dst;
  aad[3] = plain->ttl;
  aad[4] = (plain->seq >> 8) & 0xFF;  // seq high byte
  aad[5] = plain->seq & 0xFF;         // seq low byte
  aad[6] = plain->type;

  // Encrypt payload and generate authentication tag
  int ret = mbedtls_ccm_encrypt_and_tag(
    &g_ccm_ctx,
    32,                      // Payload length
    encrypted->nonce, 7,     // Nonce
    aad, 7,                  // AAD
    plain->payload,          // Plaintext input
    encrypted->encrypted_payload,  // Ciphertext output
    encrypted->tag, 4        // Authentication tag
  );

  if (ret != 0) {
    Serial.print("[CRYPTO] Encryption failed: ");
    Serial.println(ret);
    return false;
  }

  return true;
}

/**
 * Decrypt an encrypted mesh packet using AES-CCM
 *
 * @param encrypted Encrypted packet to decrypt
 * @param plain Output plaintext packet
 * @return true if decryption and authentication succeeded, false otherwise
 */
bool decryptMeshPacket(const EncryptedMeshPacket* encrypted, MeshPacket* plain) {
  if (!g_crypto_initialized) {
    Serial.println("[CRYPTO] Not initialized");
    return false;
  }

  // Copy plaintext header
  plain->src = encrypted->src;
  plain->lastHop = encrypted->lastHop;
  plain->dst = encrypted->dst;
  plain->ttl = encrypted->ttl;
  plain->seq = encrypted->seq;
  plain->type = encrypted->type;

  // Build AAD from header fields
  uint8_t aad[7];
  aad[0] = encrypted->src;
  aad[1] = encrypted->lastHop;
  aad[2] = encrypted->dst;
  aad[3] = encrypted->ttl;
  aad[4] = (encrypted->seq >> 8) & 0xFF;
  aad[5] = encrypted->seq & 0xFF;
  aad[6] = encrypted->type;

  // Decrypt payload and verify authentication tag
  int ret = mbedtls_ccm_auth_decrypt(
    &g_ccm_ctx,
    32,                      // Payload length
    encrypted->nonce, 7,     // Nonce
    aad, 7,                  // AAD
    encrypted->encrypted_payload,  // Ciphertext input
    plain->payload,          // Plaintext output
    encrypted->tag, 4        // Authentication tag
  );

  if (ret != 0) {
    Serial.print("[CRYPTO] Decryption failed: ");
    Serial.println(ret);
    return false;
  }

  return true;
}

/**
 * Measure crypto latency for performance testing
 */
void measureCryptoLatency() {
  MeshPacket testPkt = {};
  testPkt.src = 1;
  testPkt.lastHop = 1;
  testPkt.dst = 2;
  testPkt.ttl = 5;
  testPkt.seq = 100;
  testPkt.type = PKT_GPS;

  EncryptedMeshPacket encrypted;
  MeshPacket decrypted;

  uint32_t start = micros();
  encryptMeshPacket(&testPkt, &encrypted);
  uint32_t encryptTime = micros() - start;

  start = micros();
  decryptMeshPacket(&encrypted, &decrypted);
  uint32_t decryptTime = micros() - start;

  Serial.print("[CRYPTO] Encrypt latency: ");
  Serial.print(encryptTime);
  Serial.println(" us");
  Serial.print("[CRYPTO] Decrypt latency: ");
  Serial.print(decryptTime);
  Serial.println(" us");
}

/**
 * Run crypto self-tests
 *
 * @return true if all tests pass, false otherwise
 */
bool testCrypto() {
  Serial.println("[CRYPTO] Running self-tests...");

  // Test 1: Basic encrypt/decrypt
  MeshPacket original = {};
  original.src = 1;
  original.lastHop = 1;
  original.dst = 2;
  original.ttl = 5;
  original.seq = 100;
  original.type = PKT_GPS;
  original.payload[0] = 0x01;
  original.payload[1] = 0x02;
  original.payload[2] = 0x03;
  original.payload[3] = 0x04;

  EncryptedMeshPacket encrypted;
  MeshPacket decrypted;

  if (!encryptMeshPacket(&original, &encrypted)) {
    Serial.println("[CRYPTO] Test 1 FAILED: Encryption failed");
    return false;
  }

  if (!decryptMeshPacket(&encrypted, &decrypted)) {
    Serial.println("[CRYPTO] Test 1 FAILED: Decryption failed");
    return false;
  }

  if (memcmp(&original, &decrypted, sizeof(MeshPacket)) != 0) {
    Serial.println("[CRYPTO] Test 1 FAILED: Data mismatch");
    return false;
  }
  Serial.println("[CRYPTO] Test 1 PASSED: Basic encrypt/decrypt");

  // Test 2: Nonce uniqueness
  uint8_t nonce1[7], nonce2[7];
  buildNonce(1, nonce1);
  buildNonce(1, nonce2);

  if (memcmp(nonce1, nonce2, 7) == 0) {
    Serial.println("[CRYPTO] Test 2 FAILED: Nonce not unique");
    return false;
  }
  Serial.println("[CRYPTO] Test 2 PASSED: Nonce uniqueness");

  // Test 3: Tampering detection
  EncryptedMeshPacket tampered;
  if (!encryptMeshPacket(&original, &tampered)) {
    Serial.println("[CRYPTO] Test 3 FAILED: Encryption failed");
    return false;
  }

  tampered.encrypted_payload[0] ^= 0xFF;  // Corrupt data

  if (decryptMeshPacket(&tampered, &decrypted)) {
    Serial.println("[CRYPTO] Test 3 FAILED: Tampering not detected");
    return false;
  }
  Serial.println("[CRYPTO] Test 3 PASSED: Tampering detection");

  Serial.println("[CRYPTO] All self-tests PASSED");
  return true;
}

/**
 * Cleanup crypto resources
 * Call before system shutdown (optional)
 */
void cleanupCrypto() {
  if (g_crypto_initialized) {
    mbedtls_ccm_free(&g_ccm_ctx);
    g_crypto_initialized = false;
  }
}

#endif // MESH_CRYPTO_H
