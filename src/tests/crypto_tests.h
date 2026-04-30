#ifndef CRYPTO_TESTS_H
#define CRYPTO_TESTS_H

#include <Arduino.h>
#include "../meshPacket.h"
#include "../mesh_crypto.h"
#include "../gpsPacket.h"
#include <math.h>

// ============================================================
//  BOOT SELF-TESTS (always run on startup)
// ============================================================

/**
 * Run crypto self-tests at boot.
 * Tests: basic encrypt/decrypt, nonce uniqueness, tampering detection.
 * @return true if all tests pass
 */
bool testCrypto() {
  Serial.println("[CRYPTO] Running self-tests...");

  // Test 1: Basic encrypt/decrypt round-trip
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

  tampered.encrypted_payload[0] ^= 0xFF;  // Corrupt one byte

  if (decryptMeshPacket(&tampered, &decrypted)) {
    Serial.println("[CRYPTO] Test 3 FAILED: Tampering not detected");
    return false;
  }
  Serial.println("[CRYPTO] Test 3 PASSED: Tampering detection");

  Serial.println("[CRYPTO] All self-tests PASSED");
  return true;
}

// ============================================================
//  LATENCY BENCHMARK
// ============================================================

#define CRYPTO_BENCH_RUNS 100

/**
 * Measure encrypt/decrypt latency over multiple runs.
 * Reports min/avg/max in microseconds.
 */
void measureCryptoLatency() {
  MeshPacket testPkt = {};
  testPkt.src = 1;
  testPkt.lastHop = 1;
  testPkt.dst = 2;
  testPkt.ttl = 5;
  testPkt.seq = 100;
  testPkt.type = PKT_GPS;

  EncryptedMeshPacket enc;
  MeshPacket dec;

  uint32_t encMin = UINT32_MAX, encMax = 0;
  uint64_t encSum = 0;
  uint32_t decMin = UINT32_MAX, decMax = 0;
  uint64_t decSum = 0;

  // Iteration 0 is warm-up and gets discarded
  for (int i = 0; i <= CRYPTO_BENCH_RUNS; i++) {
    uint32_t start = micros();
    encryptMeshPacket(&testPkt, &enc);
    uint32_t encTime = micros() - start;

    start = micros();
    decryptMeshPacket(&enc, &dec);
    uint32_t decTime = micros() - start;

    if (i == 0) continue; // discard warm-up

    encSum += encTime;
    if (encTime < encMin) encMin = encTime;
    if (encTime > encMax) encMax = encTime;

    decSum += decTime;
    if (decTime < decMin) decMin = decTime;
    if (decTime > decMax) decMax = decTime;
  }

  uint32_t encAvg = (uint32_t)(encSum / CRYPTO_BENCH_RUNS);
  uint32_t decAvg = (uint32_t)(decSum / CRYPTO_BENCH_RUNS);

  Serial.println("[CRYPTO] Latency benchmark (100 runs, warm-up discarded):");
  Serial.print("  Encrypt  min="); Serial.print(encMin);
  Serial.print(" avg=");           Serial.print(encAvg);
  Serial.print(" max=");           Serial.print(encMax);
  Serial.println(" us");
  Serial.print("  Decrypt  min="); Serial.print(decMin);
  Serial.print(" avg=");           Serial.print(decAvg);
  Serial.print(" max=");           Serial.print(decMax);
  Serial.println(" us");
}

// ============================================================
//  ADVANCED TESTS (enabled with -DCRYPTO_ADVANCED_TESTS=1)
// ============================================================

#ifdef CRYPTO_ADVANCED_TESTS

/**
 * Create a dummy test packet with known GPS coordinates.
 * Location: ASU Tempe Campus (33.4152, -111.9283)
 */
MeshPacket createDummyPacket() {
  MeshPacket pkt;
  pkt.src = 1;
  pkt.lastHop = 1;
  pkt.dst = 2;
  pkt.ttl = 5;
  pkt.seq = 1000;
  pkt.type = PKT_GPS;
  buildPacket(33.4152, -111.9283, pkt.payload);
  return pkt;
}

/**
 * Print detailed breakdown of plaintext packet structure.
 */
void printPacketDetails(const MeshPacket& pkt) {
  Serial.println("\n========================================");
  Serial.println("      PLAINTEXT PACKET STRUCTURE");
  Serial.println("========================================");

  Serial.println("\nHEADER FIELDS (7 bytes):");
  Serial.print("  Byte 0:     src      = 0x");
  if (pkt.src < 16) Serial.print("0");
  Serial.print(pkt.src, HEX);
  Serial.print(" ("); Serial.print(pkt.src); Serial.println(")");

  Serial.print("  Byte 1:     lastHop  = 0x");
  if (pkt.lastHop < 16) Serial.print("0");
  Serial.print(pkt.lastHop, HEX);
  Serial.print(" ("); Serial.print(pkt.lastHop); Serial.println(")");

  Serial.print("  Byte 2:     dst      = 0x");
  if (pkt.dst < 16) Serial.print("0");
  Serial.print(pkt.dst, HEX);
  Serial.print(" ("); Serial.print(pkt.dst); Serial.println(")");

  Serial.print("  Byte 3:     ttl      = 0x");
  if (pkt.ttl < 16) Serial.print("0");
  Serial.print(pkt.ttl, HEX);
  Serial.print(" ("); Serial.print(pkt.ttl); Serial.println(")");

  Serial.print("  Bytes 4-5:  seq      = 0x");
  if ((pkt.seq >> 8) < 16) Serial.print("0");
  Serial.print(pkt.seq >> 8, HEX);
  if ((pkt.seq & 0xFF) < 16) Serial.print("0");
  Serial.print(pkt.seq & 0xFF, HEX);
  Serial.print(" ("); Serial.print(pkt.seq); Serial.println(")");

  Serial.print("  Byte 6:     type     = 0x");
  if (pkt.type < 16) Serial.print("0");
  Serial.print(pkt.type, HEX);
  Serial.print(" (PKT_GPS="); Serial.print(pkt.type); Serial.println(")");

  Serial.println("\nGPS PAYLOAD (9 bytes within 32-byte payload):");

  float lat, lon;
  uint8_t nodeId;
  parsePacket((uint8_t*)pkt.payload, lat, lon, nodeId);

  int32_t latInt = (int32_t)(lat * 1e6);
  int32_t lonInt = (int32_t)(lon * 1e6);

  Serial.print("  Byte 7:     node_id  = 0x");
  if (pkt.payload[0] < 16) Serial.print("0");
  Serial.print(pkt.payload[0], HEX);
  Serial.print(" ("); Serial.print(pkt.payload[0]); Serial.println(")");

  Serial.print("  Bytes 8-11: latitude = 0x");
  for (int i = 1; i <= 4; i++) {
    if (pkt.payload[i] < 16) Serial.print("0");
    Serial.print(pkt.payload[i], HEX);
  }
  Serial.print(" ("); Serial.print(latInt);
  Serial.print(" = "); Serial.print(lat, 4); Serial.println(" deg)");

  Serial.print("  Bytes 12-15: longitude = 0x");
  for (int i = 5; i <= 8; i++) {
    if (pkt.payload[i] < 16) Serial.print("0");
    Serial.print(pkt.payload[i], HEX);
  }
  Serial.print(" ("); Serial.print(lonInt);
  Serial.print(" = "); Serial.print(lon, 4); Serial.println(" deg)");

  Serial.println("\nSEARCH BYTES (for leakage test):");
  Serial.print("  Latitude bytes:  ");
  for (int i = 1; i <= 4; i++) {
    if (pkt.payload[i] < 16) Serial.print("0");
    Serial.print(pkt.payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("  Longitude bytes: ");
  for (int i = 5; i <= 8; i++) {
    if (pkt.payload[i] < 16) Serial.print("0");
    Serial.print(pkt.payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("\nFULL PACKET HEX (39 bytes total):");
  Serial.print("  ");
  const uint8_t* data = (uint8_t*)&pkt;
  for (size_t i = 0; i < sizeof(MeshPacket); i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 16 == 0 && i < sizeof(MeshPacket) - 1) {
      Serial.println();
      Serial.print("  ");
    }
  }
  Serial.println("\n");
}

/**
 * Print detailed breakdown of encrypted packet structure.
 */
void printEncryptedPacketDetails(const EncryptedMeshPacket& enc) {
  Serial.println("\n========================================");
  Serial.println("    ENCRYPTED PACKET STRUCTURE");
  Serial.println("========================================");

  Serial.println("\nPLAINTEXT HEADERS (7 bytes - for routing):");
  Serial.print("  Bytes 0-6:  ");
  Serial.print(enc.src); Serial.print(", ");
  Serial.print(enc.lastHop); Serial.print(", ");
  Serial.print(enc.dst); Serial.print(", ");
  Serial.print(enc.ttl); Serial.print(", seq=");
  Serial.print(enc.seq); Serial.print(", type=");
  Serial.println(enc.type);

  Serial.println("\nNONCE (7 bytes):");
  Serial.print("  Bytes 7-13:  ");
  for (int i = 0; i < 7; i++) {
    if (enc.nonce[i] < 16) Serial.print("0");
    Serial.print(enc.nonce[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("\nENCRYPTED PAYLOAD (32 bytes - AES-CCM):");
  Serial.println("  Bytes 14-45:");
  Serial.print("    ");
  for (int i = 0; i < 32; i++) {
    if (enc.encrypted_payload[i] < 16) Serial.print("0");
    Serial.print(enc.encrypted_payload[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 16 == 0 && i < 31) {
      Serial.println();
      Serial.print("    ");
    }
  }
  Serial.println();

  Serial.println("\nAUTHENTICATION TAG (4 bytes):");
  Serial.print("  Bytes 46-49: ");
  for (int i = 0; i < 4; i++) {
    if (enc.tag[i] < 16) Serial.print("0");
    Serial.print(enc.tag[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("\nFULL ENCRYPTED PACKET HEX (51 bytes total):");
  Serial.print("  ");
  const uint8_t* data = (uint8_t*)&enc;
  for (size_t i = 0; i < sizeof(EncryptedMeshPacket); i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 16 == 0 && i < sizeof(EncryptedMeshPacket) - 1) {
      Serial.println();
      Serial.print("  ");
    }
  }
  Serial.println("\n");
}

/**
 * Calculate Shannon entropy for a byte array.
 * Returns bits per byte (0-8, where 8 is maximum entropy).
 */
float calculateEntropy(const uint8_t* data, size_t length) {
  int freq[256] = {0};
  for (size_t i = 0; i < length; i++) {
    freq[data[i]]++;
  }

  float entropy = 0.0;
  for (int i = 0; i < 256; i++) {
    if (freq[i] > 0) {
      float p = (float)freq[i] / length;
      entropy -= p * log2(p);
    }
  }

  return entropy;
}

/**
 * Dump plaintext and encrypted packet bytes to Serial for eyeball comparison.
 * Enable file capture with: monitor_filters = log2file in platformio.ini
 * Output saved to: .pio/build/nicenano/monitor.log
 */
void dumpEncryptedPacketToSerial() {
  MeshPacket plainPacket = createDummyPacket();

  EncryptedMeshPacket encPacket;
  if (!encryptMeshPacket(&plainPacket, &encPacket)) {
    Serial.println("[CRYPTO] ERROR: Encryption failed in dump!");
    return;
  }

  // ---- PLAINTEXT ----
  Serial.println();
  Serial.println("============================================================");
  Serial.println(" PLAINTEXT PACKET (39 bytes)");
  Serial.println("============================================================");

  Serial.print(" src=");      Serial.print(plainPacket.src);
  Serial.print("  lastHop="); Serial.print(plainPacket.lastHop);
  Serial.print("  dst=");     Serial.print(plainPacket.dst);
  Serial.print("  ttl=");     Serial.print(plainPacket.ttl);
  Serial.print("  seq=");     Serial.print(plainPacket.seq);
  Serial.print("  type=");    Serial.println(plainPacket.type);

  Serial.println();
  Serial.println(" HEADER (bytes 0-6):");
  Serial.print("   ");
  const uint8_t* ph = (const uint8_t*)&plainPacket;
  for (int i = 0; i < 7; i++) {
    if (ph[i] < 16) Serial.print("0");
    Serial.print(ph[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println();
  Serial.println(" PAYLOAD (bytes 7-38):");
  for (int i = 7; i < 39; i++) {
    if ((i - 7) % 16 == 0) Serial.print("   ");
    if (ph[i] < 16) Serial.print("0");
    Serial.print(ph[i], HEX);
    Serial.print(" ");
    if ((i - 7) % 16 == 15) Serial.println();
  }
  Serial.println();

  // ---- ENCRYPTED ----
  Serial.println("============================================================");
  Serial.println(" ENCRYPTED PACKET (51 bytes)");
  Serial.println("============================================================");

  Serial.print(" src=");      Serial.print(encPacket.src);
  Serial.print("  lastHop="); Serial.print(encPacket.lastHop);
  Serial.print("  dst=");     Serial.print(encPacket.dst);
  Serial.print("  ttl=");     Serial.print(encPacket.ttl);
  Serial.print("  seq=");     Serial.print(encPacket.seq);
  Serial.print("  type=");    Serial.print(encPacket.type);
  Serial.println("  [routing header UNCHANGED]");

  Serial.println();
  Serial.println(" NONCE (bytes 6-12):");
  Serial.print("   ");
  for (int i = 0; i < 7; i++) {
    if (encPacket.nonce[i] < 16) Serial.print("0");
    Serial.print(encPacket.nonce[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println();
  Serial.println(" CIPHERTEXT (bytes 13-44):  <-- compare this to PAYLOAD above");
  for (int i = 0; i < 32; i++) {
    if (i % 16 == 0) Serial.print("   ");
    if (encPacket.encrypted_payload[i] < 16) Serial.print("0");
    Serial.print(encPacket.encrypted_payload[i], HEX);
    Serial.print(" ");
    if (i % 16 == 15) Serial.println();
  }

  Serial.println();
  Serial.println(" AUTH TAG (bytes 45-48):");
  Serial.print("   ");
  for (int i = 0; i < 4; i++) {
    if (encPacket.tag[i] < 16) Serial.print("0");
    Serial.print(encPacket.tag[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println();
  Serial.println("============================================================");
  Serial.println(" EYEBALL CHECK: compare PAYLOAD rows vs CIPHERTEXT rows");
  Serial.println(" -> They should look COMPLETELY DIFFERENT");
  Serial.println(" -> Routing header (src/dst/ttl) should be IDENTICAL");
  Serial.println("============================================================");

  Serial.println("\n[CRYPTO] Pausing 60s. Ctrl-C to stop monitor, then check .log file.");
  delay(60000);
}

/**
 * Entry point for advanced crypto tests.
 */
void runCryptoFileTests() {
  dumpEncryptedPacketToSerial();
}

#endif // CRYPTO_ADVANCED_TESTS

#endif // CRYPTO_TESTS_H
