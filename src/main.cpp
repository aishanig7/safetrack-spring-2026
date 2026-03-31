#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstdio>
#include "gpsPacket.h"
#include "meshPacket.h"

//Hardware and Pins
#define LED (0 + 15)
#define BUTTON_PIN (0+32)
#define OLED_RESET -1
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

int32_t counter = 0; 

// Node and Mesh
#define NODE_ID 1
//#define NODE_ID 2
//#define NODE_ID 3

#define DUP_CACHE_SIZE 8 
#define MAX_NEIGHBORS 10
#define MAX_ROUTES 10
//#define MESH_BROADCAST 0xFF

//other definations
#define gpsSerial Serial1

//LoRa Modem Parametrers
const float FREQ_MHZ = 906.875;
const uint8_t SF = 11;
const unsigned long BW = 250000;
const uint8_t CR = 5;
const uint16_t PREAMBLE = 16;
const uint8_t SYNCWORD = 0x2B;

//Timing and State
volatile bool receivedFlag = false; // flag to indicate that a packet was received (IRQ)
int buttonState = 0;
int lastButtonState = 0; 
int showRX = 0; 

//Objects
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
TinyGPSPlus gps;

uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];
char latBuffer[30]; // Buffer for latitude display
char lngBuffer[30]; // Buffer for longitude display
char dateBuffer[30]; // Buffer for date display
char recvBuffer[64]; // Buffer for LoRa recv display
char rssiBuffer[30]; // Buffer for rssi

double lastLat = 0;
double lastLng = 0;
TinyGPSDate lastDate;

//Data Structures
struct SeenPacket {
  uint8_t src;
  uint16_t seq;
};

struct Neighbor {
    uint8_t nodeID;
    unsigned long lastSeen; // millis() timestamp
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

//Role Definitions
//#define ROLE_TX
#define ROLE_RX

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void readGPS() {

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

bool seenBefore(uint8_t src, uint16_t seq) {
  // check cache
  for (int i = 0; i < DUP_CACHE_SIZE; i++) {
    if (seenCache[i].src == src &&
        seenCache[i].seq == seq) {
      return true;  // duplicate found
    }
  }

  // not seen then store it (circular buffer)
  seenCache[seenIndex].src = src;
  seenCache[seenIndex].seq = seq;
  seenIndex = (seenIndex + 1) % DUP_CACHE_SIZE;

  return false;
}

void sendHello() {
    const unsigned long HELLO_INTERVAL = 15000;
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

        // Build route advertisement
        HelloPayload advert = {};
        for (int i = 0; i < MAX_ROUTES; i++) {
            if (routes[i].dest != 0 && 
                advert.routeCount < MAX_ADVERT_ROUTES) {
                advert.routes[advert.routeCount].dest = routes[i].dest;
                advert.routes[advert.routeCount].hops = routes[i].hops;
                advert.routeCount++;
            }
        }
        memcpy(pkt.payload, &advert, sizeof(HelloPayload));

        radio.standby();
        radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));
        radio.startReceive();
        Serial.println("HELLO sent");
    }
}
void updateRoute(uint8_t dest, uint8_t nextHop, uint8_t hops) {
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == dest) {
            if (hops < routes[i].hops) {
                routes[i].nextHop = nextHop;
                routes[i].hops = hops;
                Serial.print("Route updated: dest=");
                Serial.print(dest);
                Serial.print(" via=");
                Serial.print(nextHop);
                Serial.print(" hops=");
                Serial.println(hops);
            }
            routes[i].lastUpdated = millis();
            return;
        }
    }
    // New route
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == 0) {
            routes[i].dest = dest;
            routes[i].nextHop = nextHop;
            routes[i].hops = hops;
            routes[i].lastUpdated = millis();
            Serial.print("Route added: dest=");
            Serial.print(dest);
            Serial.print(" via=");
            Serial.print(nextHop);
            Serial.print(" hops=");
            Serial.println(hops);
            return;
        }
    }
}

void updateNeighbors(MeshPacket* pkt) {
    // Update neighbor table
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
                Serial.print("New neighbor: ");
                Serial.println(pkt->src);
                break;
            }
        }
    }

    // Always add direct 1-hop route to sender
    updateRoute(pkt->src, pkt->src, 1);

    // Parse and merge their advertised routes
    HelloPayload advert;
    memcpy(&advert, pkt->payload, sizeof(HelloPayload));

    for (int i = 0; i < advert.routeCount; i++) {
        uint8_t dest = advert.routes[i].dest;
        uint8_t hops = advert.routes[i].hops + 1;

        if (dest == NODE_ID) continue; // don't route to yourself

        updateRoute(dest, pkt->src, hops);
    }
}

void cleanStaleRoutes() {
    const unsigned long ROUTE_EXPIRY = 60000;
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != 0 && 
            (millis() - routes[i].lastUpdated) > ROUTE_EXPIRY) {
            Serial.print("Route expired: dest=");
            Serial.println(routes[i].dest);
            routes[i].dest = 0;
            routes[i].nextHop = 0;
            routes[i].hops = 0;
            routes[i].lastUpdated = 0;
        }
    }
}

void printRoutingTable() {
    Serial.println("=== Routing Table ===");
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != 0) {
            Serial.print("dest=");
            Serial.print(routes[i].dest);
            Serial.print(" nextHop=");
            Serial.print(routes[i].nextHop);
            Serial.print(" hops=");
            Serial.print(routes[i].hops);
            Serial.print(" age=");
            Serial.print((millis() - routes[i].lastUpdated) / 1000);
            Serial.println("s");
        }
    }
    Serial.println("=====================");
}

uint8_t selectNextHop(uint8_t finalDest, uint8_t srcNode) {
    const unsigned long ROUTE_EXPIRY = 45000;

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == finalDest && 
            routes[i].nextHop != 0 &&
            (millis() - routes[i].lastUpdated) < ROUTE_EXPIRY) {
            return routes[i].nextHop;
        }
    }

    // fallback: pick any neighbor
    /*
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].nodeID != 0 && 
            neighbors[i].nodeID != NODE_ID && 
            neighbors[i].nodeID != srcNode) {
            return neighbors[i].nodeID;
        }
    }
    */

    return 0;
}


void handleTX() {
	buttonState = digitalRead(BUTTON_PIN);
    if (buttonState != lastButtonState) {
        if (buttonState == LOW) {
            Serial.println("Button Pressed Once!");
            display.setCursor(0,40); 
            display.print("TX: Packet sent"); 
            display.display(); 

            static uint16_t seqCounter = 0;

            MeshPacket pkt = {};
            pkt.src = NODE_ID;
            pkt.lastHop = NODE_ID;
            pkt.dst = 3;            
            pkt.ttl = 5;
            pkt.seq = seqCounter++;
            pkt.type = PKT_GPS;

            buildPacket(counter, 1, pkt.payload);
            counter++;

            /*
            // HARD CODE: send first hop to Node 2
            pkt.nextHop = 2;
            */

           pkt.nextHop = selectNextHop(pkt.dst, NODE_ID);

            if (pkt.nextHop == 0) {
                Serial.println("No route yet, waiting for HELLO convergence...");
                lastButtonState = buttonState;
                return;
            }

            display.setCursor(0, 50);
            display.print("FWD: ");
            display.print(pkt.nextHop);
            display.display();

            delay(random(40, 200));
            radio.standby();
            radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));
            radio.startReceive();
        }
        lastButtonState = buttonState;
    }
}

void handleRX() {
    if (!receivedFlag) return;
    receivedFlag = false;

    uint8_t inBuf[sizeof(MeshPacket)];
    int len = radio.getPacketLength();
    int state = radio.readData(inBuf, len);

    float rssi = radio.getRSSI();
    float snr = radio.getSNR();

    if (state != RADIOLIB_ERR_NONE) {
        radio.startReceive();
        return;
    }


    MeshPacket *pkt = (MeshPacket*)inBuf;

    // Ignore own packets
    if (pkt->src == NODE_ID) {
        radio.startReceive();
        return;
    }

    /*
    // remove for outdoor testing
    #if NODE_ID == 1
        if (pkt->lastHop == 3) {
            radio.startReceive();
            return;
        }
    #endif
    #if NODE_ID == 3
        if (pkt->lastHop == 1) {
            radio.startReceive();
            return;
        }
    #endif
    */


    //stop if the packet already been seen before
    if (pkt->type != PKT_HELLO && seenBefore(pkt->src, pkt->seq)) {
        radio.startReceive();
        return;
    }

    // HELLO packets update neighbors immediately
    if (pkt->type == PKT_HELLO) {
        updateNeighbors(pkt);
        radio.startReceive(); // ADD
        return;
    }

    // DEBUG: show what this node received
    if (pkt->type == PKT_GPS) {
        Serial.print("RX from node ");
        Serial.print(pkt->src);
        Serial.print(" lastHop=");
        Serial.println(pkt->lastHop);

        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0,10); 
        display.print("RSSI: ");
        display.print((int)rssi);
        display.print(" ");
        display.print("SNR: "); 
        display.print(snr, 1); 
        display.setCursor(0,20);
        display.print("dest: ");
        display.print(pkt->dst);
        display.print(" "); 
        display.print("pkt id: ");
        display.print(pkt->seq); 
        display.setCursor(0,30);
        display.print("next hop: "); 
        display.print(pkt->nextHop); 
        display.display(); 
    }

    // Packet reached final destination
    if (pkt->type == PKT_GPS && pkt->dst == NODE_ID) {
        float rxLat, rxLng;
        uint8_t rxNode;
        parsePacket(pkt->payload, rxLat, rxLng, rxNode);

        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0, 40);
        display.print("RX ");
        display.print(pkt->src);
        display.print(": ");
        display.print(rxLat, 4);
        display.print(",");
        display.print(rxLng, 4);

        display.setCursor(50, 50);
        display.print("Last Hop: ");
        display.print(pkt->lastHop); 
        display.display();

        radio.startReceive();
        return;
    }

    // Forward if not final destination
    if (pkt->type == PKT_GPS && pkt->dst != NODE_ID && pkt->ttl > 0) {

        // Only forward if this node is the intended nextHop
        if (pkt->nextHop == NODE_ID || pkt->nextHop == MESH_BROADCAST) {

            /*
            // HARDCODE FOR PROOF OF CONCEPT:
            // Node 1 → 2 → 3
            if (NODE_ID == 2) {
                pkt->nextHop = 3;
            }
            */

           pkt->nextHop = selectNextHop(pkt->dst, pkt->src);
            if (pkt->nextHop == 0) {
                radio.startReceive();
                return; // no route, drop it
            }

            // Update lastHop BEFORE forwarding
            pkt->lastHop = NODE_ID;
            pkt->ttl--;

            Serial.print("Forwarding packet to node: ");
            Serial.println(pkt->nextHop);

            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            display.setCursor(0, 50);
            display.print("FWD: ");
            display.print(pkt->nextHop);
            display.display();

            delay(random(40, 200));
            radio.transmit((uint8_t*)pkt, len);  // transmit ONCE
            radio.startReceive();
        }
    }

    radio.startReceive();
}

void setup(){
    Serial.begin(115200); 
	delay(1000); 
	pinMode(LED, OUTPUT); //set output mode
	pinMode(BUTTON_PIN, INPUT);
	digitalWrite(LED, LOW);

	for(int i = 0; i < MAX_NEIGHBORS; i++){
        neighbors[i].nodeID = 0;
        neighbors[i].lastSeen = 0;
    }

	for(int i = 0; i < MAX_ROUTES; i++){
    	routes[i].dest = 0;
        routes[i].nextHop = 0;  // ADD
        routes[i].hops = 0;     // ADD
        routes[i].lastUpdated = 0; // ADD
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

	// modem init
	// initialize SX1262 with default settings
	Serial.print(F("[SX1262] Initializing ... "));
	int state = radio.begin();
	if (state == RADIOLIB_ERR_NONE) {
	  Serial.println(F("success!"));
	} else {
	  Serial.print(F("failed, code "));
	  Serial.println(state);
	  while (true) { delay(10); }
	}

	Serial.print("radio.begin() -> ");
	Serial.println(state);
	if (state != RADIOLIB_ERR_NONE) {
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
	
    /*
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // White text on black background (for overwr)
	display.setCursor(0, 0);
	display.print("Lat: NONE");
	display.setCursor(0, 10);
	display.print("Lng: NONE");
	display.setCursor(0, 20);
	display.print("Date: NONE");
	display.setCursor(0, 30);
	display.print("Node ID: ");
	display.print(NODE_ID); 
	display.display();
    */

    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 0);
    display.print("Node ID: ");
	display.print(NODE_ID);
    display.display(); 

}

void loop() {
	const size_t BUF_SZ = 256;

    // Read and decode GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    //bool locationUpdated = gps.location.isUpdated();
    bool dateUpdated = gps.date.isUpdated();

	sendHello(); 
	//readGPS(); 

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 20000) {
        lastPrint = millis();
        printRoutingTable();
    }

    static unsigned long lastCleanup = 0;
    if (millis() - lastCleanup > 15000) {
        lastCleanup = millis();
        cleanStaleRoutes();
    }
 

    if (dateUpdated) {
        TinyGPSDate date = gps.date;

        if (date.isValid() && date.value() != lastDate.value()) {
            lastDate = date;
            snprintf(dateBuffer, sizeof(dateBuffer), "Date: %02d-%02d-%02d", date.year(), date.month(), date.day());

            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // Set color for date (also for overwr)
            display.setCursor(0, 20); // curosr pos for date
            display.print(dateBuffer);

            display.display();
        }
    }



	/*
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
    */

	
	
	handleRX(); 
	handleTX(); 

}


// Convert bytes to ASCII-safe String
String bytesToAscii(const uint8_t *buf, size_t len) {
  String s;
  s.reserve(len);
  for (size_t i = 0; i < len; i++) {
    char c = (char)buf[i];
    if (c >= 32 && c <= 126) {
      s += c;            // printable ASCII
    } else {
      s += '.';          // non-printable → dot
    }
  }
  return s;
}
