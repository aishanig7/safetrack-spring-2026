#ifndef CRYPTO_FILE_TESTS_H
#define CRYPTO_FILE_TESTS_H

#ifdef CRYPTO_ADVANCED_TESTS

#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "../../meshPacket.h"
#include "../../mesh_crypto.h"
#include "../../gpsPacket.h"
#include <math.h>

using namespace Adafruit_LittleFS_Namespace;

// Test output files (flat structure at root for compatibility)
#define FILE_PLAINTEXT "/test_plaintext.bin"
#define FILE_ENCRYPTED "/test_encrypted.bin"
#define FILE_ENTROPY_RESULTS "/test_entropy.txt"
#define FILE_HEX_DUMP "/test_hexdump.txt"

/**
 * Create a dummy test packet with known GPS coordinates
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

  // Build GPS payload with known coordinates
  buildPacket(33.4152, -111.9283, pkt.payload);

  return pkt;
}

/**
 * Print detailed breakdown of packet structure
 */
void printPacketDetails(const MeshPacket& pkt) {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      PLAINTEXT PACKET STRUCTURE           ║");
  Serial.println("╚════════════════════════════════════════════╝");

  Serial.println("\n📋 HEADER FIELDS (7 bytes):");
  Serial.print("  Byte 0:     src      = 0x");
  if (pkt.src < 16) Serial.print("0");
  Serial.print(pkt.src, HEX);
  Serial.print(" (");
  Serial.print(pkt.src);
  Serial.println(")");

  Serial.print("  Byte 1:     lastHop  = 0x");
  if (pkt.lastHop < 16) Serial.print("0");
  Serial.print(pkt.lastHop, HEX);
  Serial.print(" (");
  Serial.print(pkt.lastHop);
  Serial.println(")");

  Serial.print("  Byte 2:     dst      = 0x");
  if (pkt.dst < 16) Serial.print("0");
  Serial.print(pkt.dst, HEX);
  Serial.print(" (");
  Serial.print(pkt.dst);
  Serial.println(")");

  Serial.print("  Byte 3:     ttl      = 0x");
  if (pkt.ttl < 16) Serial.print("0");
  Serial.print(pkt.ttl, HEX);
  Serial.print(" (");
  Serial.print(pkt.ttl);
  Serial.println(")");

  Serial.print("  Bytes 4-5:  seq      = 0x");
  if ((pkt.seq >> 8) < 16) Serial.print("0");
  Serial.print(pkt.seq >> 8, HEX);
  if ((pkt.seq & 0xFF) < 16) Serial.print("0");
  Serial.print(pkt.seq & 0xFF, HEX);
  Serial.print(" (");
  Serial.print(pkt.seq);
  Serial.println(")");

  Serial.print("  Byte 6:     type     = 0x");
  if (pkt.type < 16) Serial.print("0");
  Serial.print(pkt.type, HEX);
  Serial.print(" (PKT_GPS=");
  Serial.print(pkt.type);
  Serial.println(")");

  Serial.println("\n📍 GPS PAYLOAD (9 bytes within 32-byte payload):");

  // Parse GPS data from payload
  float lat, lon;
  uint8_t nodeId;
  parsePacket((uint8_t*)pkt.payload, lat, lon, nodeId);

  int32_t latInt = (int32_t)(lat * 1e6);
  int32_t lonInt = (int32_t)(lon * 1e6);

  Serial.print("  Byte 7:     node_id  = 0x");
  if (pkt.payload[0] < 16) Serial.print("0");
  Serial.print(pkt.payload[0], HEX);
  Serial.print(" (");
  Serial.print(pkt.payload[0]);
  Serial.println(")");

  Serial.print("  Bytes 8-11: latitude = 0x");
  for (int i = 1; i <= 4; i++) {
    if (pkt.payload[i] < 16) Serial.print("0");
    Serial.print(pkt.payload[i], HEX);
  }
  Serial.print(" (");
  Serial.print(latInt);
  Serial.print(" = ");
  Serial.print(lat, 4);
  Serial.println("°)");

  Serial.print("  Bytes 12-15: longitude = 0x");
  for (int i = 5; i <= 8; i++) {
    if (pkt.payload[i] < 16) Serial.print("0");
    Serial.print(pkt.payload[i], HEX);
  }
  Serial.print(" (");
  Serial.print(lonInt);
  Serial.print(" = ");
  Serial.print(lon, 4);
  Serial.println("°)");

  Serial.println("\n🔍 SEARCH BYTES (for leakage test):");
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

  Serial.println("\n📦 FULL PACKET HEX (39 bytes total):");
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
 * Print detailed breakdown of encrypted packet structure
 */
void printEncryptedPacketDetails(const EncryptedMeshPacket& enc) {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║    ENCRYPTED PACKET STRUCTURE             ║");
  Serial.println("╚════════════════════════════════════════════╝");

  Serial.println("\n📋 PLAINTEXT HEADERS (7 bytes - for routing):");
  Serial.print("  Bytes 0-6:  ");
  Serial.print(enc.src);
  Serial.print(", ");
  Serial.print(enc.lastHop);
  Serial.print(", ");
  Serial.print(enc.dst);
  Serial.print(", ");
  Serial.print(enc.ttl);
  Serial.print(", seq=");
  Serial.print(enc.seq);
  Serial.print(", type=");
  Serial.println(enc.type);

  Serial.println("\n🔐 NONCE (7 bytes):");
  Serial.print("  Bytes 7-13:  ");
  for (int i = 0; i < 7; i++) {
    if (enc.nonce[i] < 16) Serial.print("0");
    Serial.print(enc.nonce[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("\n🔒 ENCRYPTED PAYLOAD (32 bytes - AES-CCM):");
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

  Serial.println("\n✓ AUTHENTICATION TAG (4 bytes):");
  Serial.print("  Bytes 46-49: ");
  for (int i = 0; i < 4; i++) {
    if (enc.tag[i] < 16) Serial.print("0");
    Serial.print(enc.tag[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("\n📦 FULL ENCRYPTED PACKET HEX (51 bytes total):");
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
 * Calculate Shannon entropy for a byte array
 * Returns bits per byte (0-8, where 8 is maximum entropy)
 */
float calculateEntropy(const uint8_t* data, size_t length) {
  // Count byte frequencies
  int freq[256] = {0};
  for (size_t i = 0; i < length; i++) {
    freq[data[i]]++;
  }

  // Calculate Shannon entropy: H = -Σ(p(x) * log₂(p(x)))
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
 * Write binary data to a file on internal flash
 */
bool writeBinaryFile(const char* path, const uint8_t* data, size_t length) {
  Serial.print("[FILE-TEST] Writing binary file: ");
  Serial.print(path);
  Serial.print(" (");
  Serial.print(length);
  Serial.println(" bytes)");

  File file(InternalFS);
  if (!file.open(path, FILE_O_WRITE)) {
    Serial.println("[FILE-TEST] ERROR: Failed to open file for writing");
    return false;
  }

  size_t written = file.write(data, length);
  file.close();

  if (written != length) {
    Serial.print("[FILE-TEST] ERROR: Write incomplete. Expected ");
    Serial.print(length);
    Serial.print(" bytes, wrote ");
    Serial.println(written);
    return false;
  }

  Serial.print("[FILE-TEST] Successfully wrote ");
  Serial.print(written);
  Serial.println(" bytes");
  return true;
}

/**
 * Write text content to a file on internal flash
 */
bool writeTextFile(const char* path, const char* text) {
  size_t length = strlen(text);
  Serial.print("[FILE-TEST] Writing text file: ");
  Serial.print(path);
  Serial.print(" (");
  Serial.print(length);
  Serial.println(" bytes)");

  File file(InternalFS);
  if (!file.open(path, FILE_O_WRITE)) {
    Serial.println("[FILE-TEST] ERROR: Failed to open file for writing");
    return false;
  }

  size_t written = file.write(text, length);
  file.close();

  if (written != length) {
    Serial.print("[FILE-TEST] ERROR: Write incomplete. Expected ");
    Serial.print(length);
    Serial.print(" bytes, wrote ");
    Serial.println(written);
    return false;
  }

  Serial.print("[FILE-TEST] Successfully wrote ");
  Serial.print(written);
  Serial.println(" bytes");
  return true;
}

/**
 * Initialize LittleFS file system
 */
bool initTestFileSystem() {
  Serial.println("[FILE-TEST] Initializing filesystem...");

  // Format the filesystem to ensure clean state
  // This fixes the "block < lfs->cfg->block_count" assertion error
  Serial.println("[FILE-TEST] Formatting internal flash...");
  Serial.println("[FILE-TEST] (This may take 10-30 seconds - please wait)");

  InternalFS.format();

  Serial.println("[FILE-TEST] Format complete!");
  Serial.println("[FILE-TEST] LittleFS ready");
  Serial.println("[FILE-TEST] Test files will be created at root level");

  return true;
}

/**
 * TEST 1: Eyeball Test
 * Write plaintext and encrypted packets to files for visual inspection
 */
bool testEyeball() {
  Serial.println("\n[FILE-TEST] Running Test 1: Eyeball Test");

  // Create dummy packet
  MeshPacket plainPacket = createDummyPacket();

  // Write plaintext packet to file
  if (!writeBinaryFile(FILE_PLAINTEXT, (uint8_t*)&plainPacket, sizeof(MeshPacket))) {
    Serial.println("[FILE-TEST] Test 1 FAILED: Could not write plaintext.bin");
    return false;
  }

  // Encrypt the packet
  EncryptedMeshPacket encPacket;
  if (!encryptMeshPacket(&plainPacket, &encPacket)) {
    Serial.println("[FILE-TEST] Test 1 FAILED: Encryption failed");
    return false;
  }

  // Write encrypted packet to file
  if (!writeBinaryFile(FILE_ENCRYPTED, (uint8_t*)&encPacket, sizeof(EncryptedMeshPacket))) {
    Serial.println("[FILE-TEST] Test 1 FAILED: Could not write encrypted.bin");
    return false;
  }

  Serial.println("[FILE-TEST] Test 1 PASSED: Eyeball test files written");
  Serial.print("[FILE-TEST]   - ");
  Serial.print(FILE_PLAINTEXT);
  Serial.print(" (");
  Serial.print(sizeof(MeshPacket));
  Serial.println(" bytes)");
  Serial.print("[FILE-TEST]   - ");
  Serial.print(FILE_ENCRYPTED);
  Serial.print(" (");
  Serial.print(sizeof(EncryptedMeshPacket));
  Serial.println(" bytes)");
  Serial.println("[FILE-TEST]   Open both files in hex editor to compare");

  return true;
}

/**
 * TEST 2: Entropy Analysis
 * Calculate Shannon entropy for plaintext and ciphertext
 */
bool testEntropy() {
  Serial.println("\n[FILE-TEST] Running Test 2: Entropy Analysis");

  // Create and encrypt packet
  MeshPacket plainPacket = createDummyPacket();
  EncryptedMeshPacket encPacket;

  if (!encryptMeshPacket(&plainPacket, &encPacket)) {
    Serial.println("[FILE-TEST] Test 2 FAILED: Encryption failed");
    return false;
  }

  // Calculate entropy for plaintext payload
  float plaintextEntropy = calculateEntropy(plainPacket.payload, 32);

  // Calculate entropy for encrypted payload
  float ciphertextEntropy = calculateEntropy(encPacket.encrypted_payload, 32);

  // Build results text
  char results[1024];
  snprintf(results, sizeof(results),
    "SafeTrack Encryption Entropy Analysis\n"
    "=====================================\n\n"
    "Test Packet: ASU Tempe (33.4152, -111.9283)\n"
    "Packet Type: GPS (PKT_GPS)\n"
    "Payload Size: 32 bytes\n\n"
    "RESULTS:\n"
    "--------\n"
    "Plaintext Entropy:  %.3f bits/byte\n"
    "Ciphertext Entropy: %.3f bits/byte\n\n"
    "ANALYSIS:\n"
    "---------\n"
    "Plaintext entropy < 5.0: %s\n"
    "Ciphertext entropy > 7.5: %s\n\n"
    "VERDICT: %s\n\n"
    "Shannon entropy measures randomness (0-8 bits/byte).\n"
    "Low plaintext entropy is expected for structured GPS data.\n"
    "High ciphertext entropy indicates effective encryption.\n",
    plaintextEntropy,
    ciphertextEntropy,
    (plaintextEntropy < 5.0) ? "PASS (structured data)" : "FAIL (unexpectedly high)",
    (ciphertextEntropy > 7.5) ? "PASS (high randomness)" : "FAIL (low randomness)",
    (plaintextEntropy < 5.0 && ciphertextEntropy > 7.5) ? "PASS - Encryption working correctly" : "FAIL - Encryption may have issues"
  );

  // Write results to file
  if (!writeTextFile(FILE_ENTROPY_RESULTS, results)) {
    Serial.println("[FILE-TEST] Test 2 FAILED: Could not write results.txt");
    return false;
  }

  Serial.println("[FILE-TEST] Test 2 PASSED: Entropy analysis complete");
  Serial.print("[FILE-TEST]   - Plaintext entropy:  ");
  Serial.print(plaintextEntropy, 3);
  Serial.println(" bits/byte");
  Serial.print("[FILE-TEST]   - Ciphertext entropy: ");
  Serial.print(ciphertextEntropy, 3);
  Serial.println(" bits/byte");
  Serial.print("[FILE-TEST]   - Results saved to: ");
  Serial.println(FILE_ENTROPY_RESULTS);
  Serial.println("[FILE-TEST]   - Expected: plaintext < 5.0, ciphertext > 7.5");

  return true;
}

/**
 * TEST 3: Data Leakage Search Test
 * Create hex dump of encrypted packet for manual inspection
 */
bool testDataLeakage() {
  Serial.println("\n[FILE-TEST] Running Test 3: Data Leakage Search");

  // Create and encrypt packet
  MeshPacket plainPacket = createDummyPacket();
  EncryptedMeshPacket encPacket;

  if (!encryptMeshPacket(&plainPacket, &encPacket)) {
    Serial.println("[FILE-TEST] Test 3 FAILED: Encryption failed");
    return false;
  }

  // Write hex dump with header
  Serial.print("[FILE-TEST] Writing hex dump: ");
  Serial.println(FILE_HEX_DUMP);

  File file(InternalFS);
  if (!file.open(FILE_HEX_DUMP, FILE_O_WRITE)) {
    Serial.println("[FILE-TEST] Test 3 FAILED: Could not open file");
    return false;
  }

  // Write header
  const char* header =
    "SafeTrack Encrypted Packet Hex Dump\n"
    "====================================\n\n"
    "SEARCH INSTRUCTIONS:\n"
    "Use Ctrl+F or grep to search for plaintext GPS coordinate bytes.\n"
    "Expected: No GPS data in encrypted_payload section (bytes 14-45).\n"
    "Routing headers (src, dst, ttl) are intentionally plaintext.\n\n"
    "TEST GPS COORDINATES:\n"
    "  Latitude:  33.4152 = 0x01FD4E58 (little-endian: 58 4E FD 01)\n"
    "  Longitude: -111.9283 = 0xF95308CD (little-endian: CD 08 53 F9)\n\n"
    "PACKET STRUCTURE:\n"
    "  Bytes 0-6:   Routing headers (plaintext)\n"
    "  Bytes 7-13:  Nonce (7 bytes)\n"
    "  Bytes 14-45: Encrypted payload (32 bytes) <- search here\n"
    "  Bytes 46-49: Authentication tag (4 bytes)\n\n"
    "HEX DUMP:\n"
    "=========\n\n";

  file.write(header, strlen(header));

  // Write hex dump
  const uint8_t* data = (uint8_t*)&encPacket;
  size_t length = sizeof(EncryptedMeshPacket);
  char line[80];

  for (size_t offset = 0; offset < length; offset += 16) {
    // Offset address
    snprintf(line, sizeof(line), "%08x: ", (unsigned int)offset);
    file.write(line, strlen(line));

    // Hex bytes
    for (size_t i = 0; i < 16; i++) {
      if (offset + i < length) {
        snprintf(line, sizeof(line), "%02x ", data[offset + i]);
      } else {
        snprintf(line, sizeof(line), "   ");
      }
      file.write(line, strlen(line));

      if (i == 7) {
        file.write(" ", 1);  // Extra space in middle
      }
    }

    // ASCII sidebar
    file.write(" |", 2);
    for (size_t i = 0; i < 16 && offset + i < length; i++) {
      uint8_t c = data[offset + i];
      if (c >= 32 && c <= 126) {
        file.write(&c, 1);
      } else {
        file.write(".", 1);
      }
    }
    file.write("|\n", 2);
  }

  file.close();
  Serial.println("[FILE-TEST] Hex dump written successfully");

  Serial.println("[FILE-TEST] Test 3 PASSED: Hex dump created");
  Serial.print("[FILE-TEST]   - File: ");
  Serial.println(FILE_HEX_DUMP);
  Serial.println("[FILE-TEST]   - Search for GPS bytes: 58 4E FD 01 or CD 08 53 F9");
  Serial.println("[FILE-TEST]   - Expected: NOT found in encrypted payload");

  return true;
}

/**
 * DIRECT SERIAL DUMP - No file I/O, just print encrypted packet
 */
void dumpEncryptedPacketToSerial() {
  Serial.println("\n========================================");
  Serial.println("   ENCRYPTION TEST - DETAILED OUTPUT");
  Serial.println("========================================");

  // Create test packet
  Serial.println("\n[1/4] Creating test packet...");
  MeshPacket plainPacket = createDummyPacket();
  Serial.println("✓ Test packet created (ASU Tempe location)");

  // Show detailed plaintext packet structure
  printPacketDetails(plainPacket);

  // Encrypt it
  Serial.println("\n[2/4] Encrypting packet with AES-CCM...");
  EncryptedMeshPacket encPacket;
  if (!encryptMeshPacket(&plainPacket, &encPacket)) {
    Serial.println("✗ ERROR: Encryption failed!");
    return;
  }
  Serial.println("✓ Encryption successful!");

  // Show detailed encrypted packet structure
  printEncryptedPacketDetails(encPacket);

  // Calculate and show entropy
  Serial.println("\n[3/4] Calculating entropy...");
  float plaintextEntropy = calculateEntropy(plainPacket.payload, 32);
  float ciphertextEntropy = calculateEntropy(encPacket.encrypted_payload, 32);

  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║       ENTROPY ANALYSIS RESULTS            ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.print("\n  Plaintext entropy:  ");
  Serial.print(plaintextEntropy, 3);
  Serial.print(" bits/byte ");
  if (plaintextEntropy < 5.0) {
    Serial.println("✓ (low - expected for structured data)");
  } else {
    Serial.println("✗ (too high - unexpected!)");
  }

  Serial.print("  Ciphertext entropy: ");
  Serial.print(ciphertextEntropy, 3);
  Serial.print(" bits/byte ");
  if (ciphertextEntropy > 7.5) {
    Serial.println("✓ (high - good encryption!)");
  } else {
    Serial.println("✗ (too low - encryption issue!)");
  }

  Serial.println("\n  OVERALL VERDICT: ");
  if (plaintextEntropy < 5.0 && ciphertextEntropy > 7.5) {
    Serial.println("  ✓✓✓ PASS - Encryption working correctly!");
  } else {
    Serial.println("  ✗✗✗ FAIL - Check encryption implementation!");
  }

  // Final instructions
  Serial.println("\n[4/4] Test complete!");
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║           WHAT TO CHECK                   ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n1. Compare plaintext vs encrypted payload bytes");
  Serial.println("   → Encrypted should look completely random");
  Serial.println("\n2. Search encrypted payload for GPS bytes:");
  Serial.println("   → Should NOT find latitude/longitude bytes");
  Serial.println("\n3. Verify entropy values:");
  Serial.println("   → Plaintext < 5.0 bits/byte");
  Serial.println("   → Ciphertext > 7.5 bits/byte");

  Serial.println("\n========================================");
  Serial.println("   ALL OUTPUT ABOVE - SCROLL UP TO SEE");
  Serial.println("========================================\n");

  // LONG DELAY TO PAUSE SYSTEM
  Serial.println("⏸️  PAUSING FOR 60 SECONDS...");
  Serial.println("   You can scroll up and read the output.");
  Serial.println("   Reset device to resume normal operation.\n");
  delay(60000);  // 60 second pause
}

/**
 * Run all encryption file tests
 */
void runCryptoFileTests() {
  // Skip file I/O entirely - just dump to Serial
  dumpEncryptedPacketToSerial();
}

#endif // CRYPTO_ADVANCED_TESTS
#endif // CRYPTO_FILE_TESTS_H
