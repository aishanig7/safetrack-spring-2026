#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstdio>
#include <nrf_wdt.h>
#include "gpsPacket.h"
#include "meshPacket.h"
#include "node_identity.h"
#include "mesh_crypto.h"
#include "tests/encryption/crypto_file_tests.h"

//Hardware and Pins
#define LED (0 + 15)
#define OLED_RESET -1
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define MAX_NEIGHBORS 10
#define MAX_ROUTES 10

//#define NODE_ID 1
//#define NODE_ID 2
#define NODE_ID 3

#define DUP_CACHE_SIZE 8 

struct SeenPacket {
  uint8_t src;
  uint16_t seq;
};

struct Neighbor {
  uint8_t nodeID;
  unsigned long lastSeen;  // millis() timestamp
};

struct Route {
	uint8_t dest;
    uint8_t nextHop;
    uint8_t hops;
    unsigned long lastUpdated; 
};

Route routes[MAX_ROUTES];

Neighbor neighbors[MAX_NEIGHBORS];
SeenPacket seenCache[DUP_CACHE_SIZE];
uint8_t seenIndex = 0;

// Declarations Functions
void setFlag(void);
void setup();
void loop();
String bytesToAscii(const uint8_t *, size_t);

// Declarations Objects/prims
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];
TinyGPSPlus gps;
HardwareSerial &gpsSerial = Serial1;
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
char latBuffer[30]; // Buffer for latitude display
char lngBuffer[30]; // Buffer for longitude display
char dateBuffer[30]; // Buffer for date display
char recvBuffer[64]; // Buffer for LoRa recv display
char rssiBuffer[30]; // Buffer for rssi

double lastLat = 0;
double lastLng = 0;
TinyGPSDate lastDate;

// Modem Parameters (Meshtastic LongFast)
const float FREQ_MHZ = 906.875;
const uint8_t SF = 11;
const unsigned long BW = 250000;
const uint8_t CR = 5;
const uint16_t PREAMBLE = 16;
const uint8_t SYNCWORD = 0x2B;
volatile bool receivedFlag = false; // flag to indicate that a packet was received (IRQ)

//button state variables
int buttonState = 0;
int lastButtonState = 0; 

//display variables
int showRX = 0; 

//Role Definitions
//#define ROLE_TX
#define ROLE_RX

void setFlag(void) {
  receivedFlag = true;
}

/**
 * Initialize watchdog timer for tamper detection
 * Watchdog must be pet regularly or system will reset
 */
void initWatchdog() {
  nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP);
  nrf_wdt_reload_value_set(NRF_WDT, 10 * 32768);  // 10 seconds
  nrf_wdt_reload_request_enable(NRF_WDT, NRF_WDT_RR0);
  nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);
  Serial.println("[SECURITY] Watchdog timer initialized");
}

/**
 * Initialize brown-out detection
 * System will reset if voltage drops below threshold
 */
void initBOD() {
  NRF_POWER->POFCON = (POWER_POFCON_POF_Enabled << POWER_POFCON_POF_Pos) |
                      (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos);
  Serial.println("[SECURITY] Brown-out detection enabled");
}

void readGPS() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        double lat = gps.location.lat();
        double lng = gps.location.lng();

        if (lat != lastLat || lng != lastLng) {
            lastLat = lat;
            lastLng = lng;

            display.setCursor(0, 0);
            display.print("Lat: ");
            display.print(lat, 6);

            display.setCursor(0, 10);
            display.print("Lng: ");
            display.print(lng, 6);

            display.display();
        }
    }
}

bool seenBefore(uint8_t src, uint16_t seq) {
  for (int i = 0; i < DUP_CACHE_SIZE; i++) {
    if (seenCache[i].src == src && seenCache[i].seq == seq) {
      return true;
    }
  }

  seenCache[seenIndex].src = src;
  seenCache[seenIndex].seq = seq;
  seenIndex = (seenIndex + 1) % DUP_CACHE_SIZE;

  return false;
}

void sendHello() {
	const unsigned long HELLO_INTERVAL = 15000; // 15s
	static unsigned long lastHello = 0;

	if (millis() - lastHello > HELLO_INTERVAL) {
		lastHello = millis();

		static uint16_t seqCounter = 0;

		MeshPacket pkt = {};
		pkt.src = NODE_ID;
		pkt.lastHop = NODE_ID;
		pkt.dst = MESH_BROADCAST;
		pkt.ttl = 1;
		pkt.seq = seqCounter++;
		pkt.type = PKT_HELLO;

		// optional: fill payload[0..3] with millis() timestamp
		// memcpy(pkt.payload, &lastHello, sizeof(lastHello));

		// Encrypt packet before transmission
		EncryptedMeshPacket encPkt;
		if (!encryptMeshPacket(&pkt, &encPkt)) {
			Serial.println("[CRYPTO] Encryption failed in sendHello!");
			return;
		}

		radio.standby();
		radio.transmit((uint8_t*)&encPkt, sizeof(EncryptedMeshPacket));
		radio.startReceive();

		Serial.println("HELLO sent (encrypted)");
	}
}

void updateNeighbors(MeshPacket* pkt) {
	bool found = false;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].nodeID == pkt->src) {
            neighbors[i].lastSeen = millis();
            found = true;
            break;
        }
    }

    if (!found) {
        for (int i = 0; i < MAX_NEIGHBORS; i++) {
            if (neighbors[i].nodeID == 0) {
                neighbors[i].nodeID = pkt->src;
                neighbors[i].lastSeen = millis();
                Serial.print("New neighbor added: ");
                Serial.println(pkt->src);
                break;
            }
        }
    }

	for (int i = 0; i < MAX_ROUTES; i++) {
		if (routes[i].dest == 0 || routes[i].dest == pkt->src) {
			routes[i].dest = pkt->src;
			routes[i].nextHop = pkt->src;
			routes[i].hops = 1;
			routes[i].lastUpdated = millis();
			break;
    	}
	}
}

void handleTX() {
	buttonState = digitalRead(BUTTON_PIN);
	if (buttonState != lastButtonState) {
		if (buttonState == LOW) {
			Serial.println("Button Pressed Once!");
			display.clearDisplay();  
			display.setCursor(0,0); 
			display.print("TX: Node Ready"); 
			display.display(); 

			static uint16_t seqCounter = 0;

			MeshPacket pkt = {};
			pkt.src = NODE_ID;
			pkt.lastHop = NODE_ID;
			
			uint8_t targetNode = 1;   // change per test
			uint8_t nextHop = targetNode;

			// If target not neighbor, lookup route
			bool isNeighbor = false;
			for (int i = 0; i < MAX_NEIGHBORS; i++) {
				if (neighbors[i].nodeID == targetNode) {
					isNeighbor = true;
					break;
				}
			}

			if (!isNeighbor) {
				for (int i = 0; i < MAX_ROUTES; i++) {
					if (routes[i].dest == targetNode) {
						nextHop = routes[i].nextHop;
						break;
					}
				}
			}

			pkt.dst = nextHop;

			pkt.ttl = 5;
			pkt.seq = seqCounter++;
			pkt.type = PKT_GPS;

			buildPacket(33.4152, 111.9283, pkt.payload);

			// Encrypt packet before transmission
			EncryptedMeshPacket encPkt;
			if (!encryptMeshPacket(&pkt, &encPkt)) {
				Serial.println("[CRYPTO] Encryption failed in handleTX!");
				return;
			}

			radio.standby();
			radio.transmit((uint8_t*)&encPkt, sizeof(EncryptedMeshPacket));
			radio.startReceive();
		}
	lastButtonState = buttonState;
	}
}

void handleRX() {
	if (receivedFlag) {
		receivedFlag = false;

		uint8_t inBuf[64];
		int len = radio.getPacketLength();

		int state = radio.readData(inBuf, len);
		if (state != RADIOLIB_ERR_NONE || len != sizeof(EncryptedMeshPacket)) {
			radio.startReceive();
			return;
		}

		// Cast to encrypted packet and decrypt
		EncryptedMeshPacket *encPkt = (EncryptedMeshPacket*)inBuf;
		MeshPacket pkt;

		if (!decryptMeshPacket(encPkt, &pkt)) {
			Serial.println("[CRYPTO] Decryption failed");
			radio.startReceive();
			return;
		}

		// Check if packet is from self
		if (pkt.src == NODE_ID) {
			radio.startReceive();
			return;
		}

		// duplicate check
		if (pkt.type != PKT_HELLO && seenBefore(pkt.src, pkt.seq)) {
			Serial.println("Duplicate packet dropped");
			radio.startReceive();
			return;
		}

		// HELLO packet handling
		if (pkt.type == PKT_HELLO) {
			updateNeighbors(&pkt);
		}

		//GPS packet handling
		if (pkt.type == PKT_GPS && pkt.dst == NODE_ID) {
			float rxLat, rxLng;
			uint8_t rxNode;
			parsePacket(pkt.payload, rxLat, rxLng, rxNode);

			Serial.print("RX from node ");
			Serial.print(pkt.src);
			//Serial.print(" Lat=");
			Serial.print(NODE_ID);
			Serial.print(" ");
			Serial.println(rxLng, 6);
			Serial.print(" ");
			Serial.print(rxLat, 6);
			//Serial.print(" Lng=");

			display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
			display.setCursor(0, 40);
			display.print("RX ");
			display.print(pkt.src);
			display.print(": ");
			display.print(rxLat, 4);
			display.print(",");
			display.print(rxLng, 4);
			display.display();

			Serial.print("RSSI: "); Serial.println(radio.getRSSI());
			Serial.print("SNR: "); Serial.println(radio.getSNR());
		}

		//  forward packet
		if (pkt.ttl > 0 && pkt.dst != NODE_ID) {
			uint8_t nextHop = MESH_BROADCAST;

			for (int i = 0; i < MAX_ROUTES; i++) {
				if (routes[i].dest == pkt.dst) {
					nextHop = routes[i].nextHop;
					break;
				}
			}
			display.setCursor(0, 50);
			display.print("FWD: ");
			display.print(pkt.dst);
			display.display();

			// Update routing fields
			pkt.ttl--;
			pkt.lastHop = NODE_ID;

			if (nextHop != MESH_BROADCAST) {
				pkt.dst = nextHop;
			}

			// Re-encrypt with updated headers
			EncryptedMeshPacket fwdPkt;
			if (!encryptMeshPacket(&pkt, &fwdPkt)) {
				Serial.println("[CRYPTO] Re-encryption failed");
				radio.startReceive();
				return;
			}

			delay(random(40, 200));
			radio.transmit((uint8_t*)&fwdPkt, sizeof(EncryptedMeshPacket));
		}

		// go back to receive mode
		radio.startReceive();
	}
}

void setup() {
  Serial.begin(115200);
  const unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 3000)) {
    delay(10);
  }
  delay(50);

  NodeIdentity::initNodeIdentity();
  const uint8_t *mac = NodeIdentity::getNodeMac();
  Serial.printf(
      "[Node] name=%s id=0x%02X role=%s mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
      NodeIdentity::getNodeName(), NodeIdentity::getNodeId(),
      NodeIdentity::isHeadNode() ? "HEAD" : "FIELD", mac[5], mac[4], mac[3],
      mac[2], mac[1], mac[0]);

  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(LED, LOW);

  // Initialize crypto system
  if (!initCrypto()) {
    Serial.println("[CRYPTO] Initialization failed!");
    digitalWrite(LED, HIGH);
    while (true) { delay(1000); }
  }

  // Run crypto self-tests
  if (!testCrypto()) {
    Serial.println("[CRYPTO] Self-tests failed!");
    digitalWrite(LED, HIGH);
    while (true) { delay(1000); }
  }

  // Measure crypto performance
  measureCryptoLatency();

  // Pause so crypto test results are visible in serial monitor
  Serial.println("\n========================================");
  Serial.println("   CRYPTO TESTS COMPLETE - SUCCESS!");
  Serial.println("========================================");

#ifdef CRYPTO_ADVANCED_TESTS
  // Dump encrypted packet directly to Serial (no file I/O)
  runCryptoFileTests();
#endif

  Serial.println("Waiting 5 seconds before starting main loop...\n");
  delay(5000);  // 5 second pause to read crypto results

  // Initialize security features
  initBOD();
  initWatchdog();

	for(int i = 0; i < MAX_NEIGHBORS; i++){
        neighbors[i].nodeID = 0;
        neighbors[i].lastSeen = 0;
    }
	
	gpsSerial.begin(9600);
	//SPI.begin(); // uses variant default SPI pins

	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { //if error then builtin led light up
	  digitalWrite(LED, HIGH); //Set the LED to high
	  while (true);
	}
	
	// display boot
	memset(buffer, 0, sizeof(buffer)); // set loop display buffer with zeros
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);
	display.println("Booting or smth...");
	display.display();

  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) {
      delay(10);
    }
  }

	int st = radio.begin();
	Serial.print("radio.begin() -> ");
	Serial.println(st);
	if (st != RADIOLIB_ERR_NONE) {
	  Serial.println("radio.begin() failed. Check wiring, power, constructor ordering.");
	  while (true) { delay(1000); }
	}
	
	// set modem parameters
	radio.setFrequency(FREQ_MHZ);
	radio.setSpreadingFactor(SF);
	radio.setBandwidth(BW);
	radio.setCodingRate(CR);
	radio.setPreambleLength(PREAMBLE);
	radio.setSyncWord(SYNCWORD);
	//radio.setCRC(2);
	radio.setPacketReceivedAction(setFlag); // IRQ func
	
	// start listening for LoRa packets
	Serial.print(F("[SX1262] Starting to listen ... "));
	state = radio.startReceive();
	if (state == RADIOLIB_ERR_NONE) {
	  Serial.println(F("success!"));
	} else {
	  Serial.print(F("failed, code "));
	  Serial.println(state);
	  while (true) { delay(10); }
	}
	Serial.println("boot successful...");
	
	delay(1000);
	display.clearDisplay();
	display.display();
	
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // White text on black background (for overwr)
	display.setCursor(0, 0);
	display.print("Lat: NONE");
	display.setCursor(0, 10);
	display.print("Lng: NONE");
	display.setCursor(0, 20);
	display.print("Date: NONE");
	display.setCursor(0, 30);
	display.print(NodeIdentity::getNodeName());
	display.setCursor(0, 40);
	display.print("no message recv");
	display.display();
}

void loop() {
  static unsigned long lastStatusPrint = 0;

  // Pet watchdog to prevent reset
  nrf_wdt_reload_request_set(NRF_WDT, NRF_WDT_RR0);

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

    bool locationUpdated = gps.location.isUpdated();
    bool dateUpdated = gps.date.isUpdated();
 
    // Update latitude and longitude if changed
    if (locationUpdated) {
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        bool redraw = false;

        // Only update display if lat or lng has changed
        if (lat != lastLat) {
            lastLat = lat;
            snprintf(latBuffer, sizeof(latBuffer), "Lat: %3.6f", lat);
            //snprintf(latBuffer, sizeof(latBuffer), "Lat: 30.974632");
            redraw = true;
        }

        if (lng != lastLng) {
            lastLng = lng;
            snprintf(lngBuffer, sizeof(lngBuffer), "Lng: %3.6f", lng);
            //snprintf(lngBuffer, sizeof(lngBuffer), "Lng: -97.777298");
            redraw = true;
        }

        // if loc data changed, redraw
        if (redraw) {
            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // White text on black background (for overwr)
            display.setCursor(0, 0); // cursor pos for lat
            display.print(latBuffer); // Print new latitude
            
            display.setCursor(0, 10); // cursor pos for lng
            display.print(lngBuffer); // Print new longitude

            display.display();
        }
    }

  if (dateUpdated) {
    TinyGPSDate date = gps.date;

    if (date.isValid() && date.value() != lastDate.value()) {
      lastDate = date;
      snprintf(dateBuffer, sizeof(dateBuffer), "Date: %02d-%02d-%02d",
               date.year(), date.month(), date.day());

      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      display.setCursor(0, 20);
      display.print(dateBuffer);
      display.display();
    }
  }

  if (millis() - lastStatusPrint >= 1000) {
    lastStatusPrint = millis();

    Serial.print("Node: ");
    Serial.println(NodeIdentity::getNodeName());

    if (gps.location.isValid()) {
      Serial.print("Lat: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Lng: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Lat: NONE");
      Serial.println("Lng: NONE");
    }

    if (gps.date.isValid()) {
      Serial.print("Date: ");
      Serial.print(gps.date.year());
      Serial.print("-");
      Serial.print(gps.date.month());
      Serial.print("-");
      Serial.println(gps.date.day());
    } else {
      Serial.println("Date: NONE");
    }

    Serial.println("no message recv");

    // Keep node label visible on OLED alongside the status fields.
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 30);
    display.print(NodeIdentity::getNodeName());
    display.display();
  }

	const unsigned long HELLO_INTERVAL = 15000; // 15s
	static unsigned long lastHello = 0;

	if (millis() - lastHello > HELLO_INTERVAL) {
		lastHello = millis();

		static uint16_t seqCounter = 0;

		MeshPacket pkt = {};
		pkt.src = NODE_ID;
		pkt.lastHop = NODE_ID;
		pkt.dst = MESH_BROADCAST;
		pkt.ttl = 1;
		pkt.seq = seqCounter++;
		pkt.type = PKT_HELLO;

		// optional: fill payload[0..3] with millis() timestamp
		// memcpy(pkt.payload, &lastHello, sizeof(lastHello));

		// Encrypt packet before transmission
		EncryptedMeshPacket encPkt;
		if (!encryptMeshPacket(&pkt, &encPkt)) {
			Serial.println("[CRYPTO] Encryption failed in loop HELLO!");
			return;
		}

		radio.standby();
		radio.transmit((uint8_t*)&encPkt, sizeof(EncryptedMeshPacket));
		radio.startReceive();

		Serial.println("HELLO sent (encrypted)");
	}

	Serial.println("Current neighbors:");
	for(int i = 0; i < MAX_NEIGHBORS; i++){
		if(neighbors[i].nodeID != 0){
			Serial.print("  Node ");
			Serial.print(neighbors[i].nodeID);
			Serial.print(" lastSeen=");
			Serial.println(neighbors[i].lastSeen);
		}
	}
	Serial.println("---");
	

	#ifdef ROLE_TX 
		buttonState = digitalRead(BUTTON_PIN);
		if (buttonState != lastButtonState) {
			if (buttonState == LOW) {
				Serial.println("Button Pressed Once!");
				display.clearDisplay();  
				display.setCursor(0,0); 
				display.print("TX: Node Ready"); 
				display.display(); 

				static uint16_t seqCounter = 0;

				MeshPacket pkt = {};
				pkt.src = NODE_ID;
				pkt.lastHop = NODE_ID;
				pkt.dst = MESH_BROADCAST;
				pkt.ttl = 5;
				pkt.seq = seqCounter++;
				pkt.type = PKT_GPS;

				buildPacket(45.8239, -112.5641, pkt.payload);

				// Encrypt packet before transmission
				EncryptedMeshPacket encPkt;
				if (!encryptMeshPacket(&pkt, &encPkt)) {
					Serial.println("[CRYPTO] Encryption failed in loop TX!");
					return;
				}

				radio.standby();
				radio.transmit((uint8_t*)&encPkt, sizeof(EncryptedMeshPacket));
				radio.startReceive();
			}
		lastButtonState = buttonState;
		}
		
	#endif


	#ifdef ROLE_RX
		if (receivedFlag) {
			receivedFlag = false;

			uint8_t inBuf[64];
			int len = radio.getPacketLength();

			int state = radio.readData(inBuf, len);
			if (state != RADIOLIB_ERR_NONE || len != sizeof(EncryptedMeshPacket)) {
				radio.startReceive();
				return;
			}

			// Cast to encrypted packet and decrypt
			EncryptedMeshPacket *encPkt = (EncryptedMeshPacket*)inBuf;
			MeshPacket pkt;

			if (!decryptMeshPacket(encPkt, &pkt)) {
				Serial.println("[CRYPTO] Decryption failed");
				radio.startReceive();
				return;
			}

			// duplicate check
			if (pkt.type != PKT_HELLO && seenBefore(pkt.src, pkt.seq)) {
				Serial.println("Duplicate packet dropped");
				radio.startReceive();
				return;
			}

			// HELLO packet handling
			if (pkt.type == PKT_HELLO) {
				bool found = false;
				for (int i = 0; i < MAX_NEIGHBORS; i++) {
					if (neighbors[i].nodeID == pkt.src) {
						neighbors[i].lastSeen = millis();
						found = true;
						break;
					}
				}
				if (!found) {
					for (int i = 0; i < MAX_NEIGHBORS; i++) {
						if (neighbors[i].nodeID == 0) {
							neighbors[i].nodeID = pkt.src;
							neighbors[i].lastSeen = millis();
							break;
						}
					}
				}
				Serial.print("Neighbor updated: ");
				Serial.println(pkt.src);
			}

			//GPS packet handling
			if (pkt.type == PKT_GPS && (pkt.dst == NODE_ID || pkt.dst == MESH_BROADCAST)) {
				float rxLat, rxLng;
				uint8_t rxNode;
				parsePacket(pkt.payload, rxLat, rxLng, rxNode);

				Serial.print("RX from node ");
				Serial.print(pkt.src);
				Serial.print(" Lat=");
				Serial.print(rxLat, 6);
				Serial.print(" Lng=");
				Serial.println(rxLng, 6);

				display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
				display.setCursor(0, 40);
				display.print("RX ");
				display.print(pkt.src);
				display.print(": ");
				display.print(rxLat, 4);
				display.print(",");
				display.print(rxLng, 4);
				display.display();

				Serial.print("RSSI: "); Serial.println(radio.getRSSI());
				Serial.print("SNR: "); Serial.println(radio.getSNR());
			}

			//  forward packet
			if (pkt.ttl > 0 && pkt.dst != NODE_ID) {
				pkt.ttl--;
				pkt.lastHop = NODE_ID;

				// Re-encrypt with updated headers
				EncryptedMeshPacket fwdPkt;
				if (!encryptMeshPacket(&pkt, &fwdPkt)) {
					Serial.println("[CRYPTO] Re-encryption failed in loop RX");
					radio.startReceive();
					return;
				}

				delay(random(40, 200)); // small random delay to avoid collisions
				radio.transmit((uint8_t*)&fwdPkt, sizeof(EncryptedMeshPacket));
			}

			// go back to receive mode
			radio.startReceive();
		}
	#endif

}

// Convert bytes to ASCII-safe String
String bytesToAscii(const uint8_t *buf, size_t len) {
  String s;
  s.reserve(len);
  for (size_t i = 0; i < len; i++) {
    char c = (char)buf[i];
    if (c >= 32 && c <= 126) {
      s += c;
    } else {
      s += '.';
    }
  }
  return s;
}
