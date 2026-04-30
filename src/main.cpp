#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstdio>
#include "gpsPacket.h"
#include "meshPacket.h"

//Hardware and Pins
#define LED (0 + 15)
#define BUTTON_PIN (0+32)

int32_t counter = 0; 
// Node and Mesh
//#define NODE_ID 1
//#define NODE_ID 2
#define NODE_ID 3

#define DUP_CACHE_SIZE 8 
#define MAX_NEIGHBORS 10
#define MAX_ROUTES 10
#define MESH_BROADCAST 0xFF


#define PRI_ACK   3
#define PRI_GPS   2
#define PRI_HELLO 1

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

unsigned long lastRXActivity = 0;

volatile uint16_t pendingAckSeq = 0;
volatile bool ackReceivedFlag = false;
unsigned long ackWaitStart = 0;
bool pendingACK = false;
MeshPacket pendingPkt;
uint16_t pendingSeq = 0;
unsigned long pendingSentAt = 0;
uint8_t pendingRetries = 0;
#define MAX_RETRIES 3
#define ACK_TIMEOUT 5000

//Objects
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
TinyGPSPlus gps;

double lastLat = 0;
double lastLng = 0;
bool gpsValid = false;
uint8_t fwdDots = 0;
double rxDisplayLat = 0;
double rxDisplayLng = 0;
uint8_t rxDisplaySrc = 0;
TinyGPSDate lastDate;

//Data Structures
struct SeenPacket {
  uint8_t src;
  uint16_t seq;
};

struct Neighbor {
    uint8_t nodeID;
    int16_t rssi; 
    unsigned long lastSeen; // millis() timestamp
};

struct Route {
	uint8_t dest;
    uint8_t nextHop;
    uint8_t hops;
    int16_t linkRSSI;
    unsigned long lastUpdated; 
};

Route routes[MAX_ROUTES];

Neighbor neighbors[MAX_NEIGHBORS];

SeenPacket seenCache[DUP_CACHE_SIZE];
uint8_t seenIndex = 0;

// Display state machine
enum DisplayState { DS_HOME, DS_SOS_FLASH, DS_SOS_SENT, DS_FORWARDING, DS_CONFIRMED, DS_SOS_RECEIVED, DS_SEND_FAILED };
DisplayState dispState = DS_HOME;
unsigned long flashFrameAt = 0;
int flashFrame = 0;
bool flashNoRoute = false;
unsigned long sendFailedAt = 0;

// Battery state
uint8_t battPct = 100;
bool battUSB = false;
unsigned long lastBattSample = 0;

// Declarations Functions
void setFlag(void);
void setup();
void loop();
void updateDisplay();
void readBattery();
String bytesToAscii(const uint8_t *, size_t);



bool channelBusy() {
    return (millis() - lastRXActivity) < 600;
}

unsigned long getBackoff(uint8_t priority) {
    switch(priority) {
        case PRI_ACK:   return random(10, 50);
        case PRI_GPS:   return random(50, 150);
        case PRI_HELLO: return random(200, 600);
    }
    return 100;
}

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

// ── UI helpers ────────────────────────────────────────────────────────────────

static void drawBattery(int x, int y, uint8_t pct, bool usb) {
    u8g2.drawFrame(x, y, 18, 8);
    u8g2.drawBox(x + 18, y + 2, 2, 4);
    int fillW = usb ? 13 : (pct * 13) / 100;
    if (fillW > 0) u8g2.drawBox(x + 2, y + 2, fillW, 4);
}

static void drawWarningTriangle(int cx, int by) {
    u8g2.drawLine(cx, by - 12, cx - 7, by);
    u8g2.drawLine(cx - 7, by, cx + 7, by);
    u8g2.drawLine(cx + 7, by, cx, by - 12);
    u8g2.drawVLine(cx, by - 9, 5);
    u8g2.drawPixel(cx, by - 2);
}

static void drawTopHalf() {
    char nodeName[12];
    snprintf(nodeName, sizeof(nodeName), "SAFE-%04X", NODE_ID);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 8, nodeName);
    drawBattery(108, 1, battPct, battUSB);
}

static void drawTXHome() {
    drawWarningTriangle(26, 46);
    drawWarningTriangle(102, 46);
    u8g2.setFont(u8g2_font_logisoso24_tr);
    const char* s = "SOS";
    int w = u8g2.getStrWidth(s);
    u8g2.drawStr((128 - w) / 2, 52, s);
}

static void drawRXHome() {
    u8g2.setFont(u8g2_font_6x10_tr);
    if (rxDisplaySrc != 0) {
        char buf[22];
        snprintf(buf, sizeof(buf), "Lat: %.6f", rxDisplayLat);
        u8g2.drawStr(0, 26, buf);
        snprintf(buf, sizeof(buf), "Lng: %.6f", rxDisplayLng);
        u8g2.drawStr(0, 38, buf);
    }
}

static void drawSOS() {
    u8g2.setFont(u8g2_font_logisoso16_tr);
    const char* s = "SOS SENT";
    int w = u8g2.getStrWidth(s);
    u8g2.drawStr((128 - w) / 2, 52, s);
    drawWarningTriangle(10, 50);
    drawWarningTriangle(118, 50);
}

static void drawConfirmed() {
    u8g2.drawLine(4, 55, 8, 60);
    u8g2.drawLine(8, 60, 18, 48);
    u8g2.drawLine(5, 55, 9, 60);
    u8g2.drawLine(9, 60, 19, 48);
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.drawStr(24, 61, "Confirmed!");
}

// ──────────────────────────────────────────────────────────────────────────────

void readGPS() {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    if (lat != lastLat || lng != lastLng) {
        lastLat = lat;
        lastLng = lng;
        gpsValid = true;

        u8g2.clearBuffer();
        drawTopHalf();
        drawTXHome();
        u8g2.sendBuffer();
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

bool hasSeen(uint8_t src, uint16_t seq) {
    for (int i = 0; i < DUP_CACHE_SIZE; i++) {
        if (seenCache[i].src == src && 
            seenCache[i].seq == seq) {
            return true;
        }
    }
    return false;
}

void markSeen(uint8_t src, uint16_t seq) {
    seenCache[seenIndex].src = src;
    seenCache[seenIndex].seq = seq;
    seenIndex = (seenIndex + 1) % DUP_CACHE_SIZE;
}

void sendHello() {
    //const unsigned long HELLO_INTERVAL = 15000;
    static unsigned long lastHello = 0;

    const unsigned long HELLO_BASE = 20000;
    const unsigned long HELLO_JITTER = 5000;

    static unsigned long nextHelloIn = HELLO_BASE + random(0, HELLO_JITTER);

    if (millis() - lastHello > nextHelloIn) {
        if (channelBusy()) {
            Serial.println("Channel busy, backing off HELLO");

            delay(getBackoff(PRI_HELLO));
            return;
        }

        Serial.print(millis() / 1000);
        Serial.println("s: Sending HELLO");

        lastHello = millis();
        nextHelloIn = HELLO_BASE + random(0, HELLO_JITTER) + NODE_ID * 37;

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

        //radio.standby();
        delay(random(40, 200));

        if (channelBusy()) {
            delay(getBackoff(PRI_HELLO));
            return;
        }

        radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));
        lastRXActivity = millis(); 
        delay(10);
        radio.setPacketReceivedAction(setFlag);
        int rxState = radio.startReceive();
        Serial.print("startReceive after HELLO: ");
        Serial.println(rxState);
        Serial.println("HELLO sent");
    }
}

void updateRoute(uint8_t dest, uint8_t nextHop, uint8_t hops, int16_t rssi) {
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == dest) {
            bool betterHopCount = hops < routes[i].hops;
            bool samePath = (nextHop == routes[i].nextHop);
            bool betterRSSI = rssi > routes[i].linkRSSI;

            if (betterHopCount || (samePath && betterRSSI)) {
                routes[i].nextHop = nextHop;
                routes[i].hops = hops;
                routes[i].linkRSSI = rssi;

                Serial.print(millis() / 1000);
                Serial.print("s Route added: dest=");
                Serial.print(dest);
                Serial.print(" via=");
                Serial.print(nextHop);
                Serial.print(" hops=");
                Serial.println(hops);
            }

            if (samePath || betterHopCount) {
                routes[i].lastUpdated = millis(); // refresh if route confirmed
            }
            return;
        }
    }
    // New route
    int worstIndex = -1;
    unsigned long oldest = ULONG_MAX;

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest == 0) {
            worstIndex = i;
            break;
        }
    }

    if (worstIndex == -1) {
        for (int i = 0; i < MAX_ROUTES; i++) {
            if (routes[i].lastUpdated < oldest) {
                oldest = routes[i].lastUpdated;
                worstIndex = i;
            }
        }
    }

    if (worstIndex != -1) {
        routes[worstIndex].dest = dest;
        routes[worstIndex].nextHop = nextHop;
        routes[worstIndex].hops = hops;
        routes[worstIndex].lastUpdated = millis();
        routes[worstIndex].linkRSSI = rssi;

        Serial.print("Route added/replaced: dest=");
        Serial.print(dest);
        Serial.print(" via=");
        Serial.print(nextHop);
        Serial.print(" hops=");
        Serial.println(hops);
    }
}

void updateNeighbors(MeshPacket* pkt, int16_t rssi) {
    // Update neighbor table
    Serial.print("HELLO received from node ");
    Serial.println(pkt->src);
    bool found = false;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].nodeID == pkt->src) {
            neighbors[i].rssi = rssi;
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
                neighbors[i].rssi = (int16_t)rssi;
                Serial.print("New neighbor: ");
                Serial.println(pkt->src);
                break;
            }
        }
    }

    // Always add direct 1-hop route to sender
    updateRoute(pkt->src, pkt->src, 1, (int16_t)rssi);

    // Parse and merge their advertised routes
    HelloPayload advert;
    memcpy(&advert, pkt->payload, sizeof(HelloPayload));

    for (int i = 0; i < advert.routeCount; i++) {
        uint8_t dest = advert.routes[i].dest;
        uint8_t hops = advert.routes[i].hops + 1;

        if (dest == NODE_ID) continue; // don't route to yourself

        updateRoute(dest, pkt->src, hops, (int16_t)rssi);
    }
}

void cleanStaleNeighbors() {
    const unsigned long NEIGHBOR_TIMEOUT = 45000;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].nodeID != 0 &&
            millis() - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT) {

            Serial.print("Neighbor expired: ");
            Serial.println(neighbors[i].nodeID);

            neighbors[i].nodeID = 0;
            neighbors[i].rssi = -200;
            neighbors[i].lastSeen = 0;
        }
    }
}

void cleanStaleRoutes() {
    const unsigned long ROUTE_EXPIRY = 45000;
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != 0 && 
            (millis() - routes[i].lastUpdated) > ROUTE_EXPIRY) {
            Serial.print(millis() / 1000);
            Serial.print("s Route expired: dest=");
            Serial.println(routes[i].dest);
            routes[i].dest = 0;
            routes[i].nextHop = 0;
            routes[i].hops = 0;
            routes[i].lastUpdated = 0;
            routes[i].linkRSSI = -128; 
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
    const int16_t MIN_RSSI = -100;

    uint8_t bestNext = 0;
    int16_t bestRSSI = -999;
    uint8_t bestHops = 255;

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (routes[i].dest != finalDest) continue;

        uint8_t nextHop = routes[i].nextHop;
        if (nextHop == srcNode) continue;

        bool neighborFound = false;
        int16_t linkRSSI = routes[i].linkRSSI;

        for (int j = 0; j < MAX_NEIGHBORS; j++) {
            if (neighbors[j].nodeID == nextHop) {
                if (millis() - neighbors[j].lastSeen > 30000) {
                    Serial.println("Neighbor stale, using stored linkRSSI");
                    neighborFound = true;
                    break;
                }
                linkRSSI = neighbors[j].rssi;
                neighborFound = true;
                break;
            }
        }

        if (!neighborFound) continue;
        if (linkRSSI < MIN_RSSI) continue;

        if (linkRSSI > bestRSSI) {
            bestRSSI = linkRSSI;
            bestHops = routes[i].hops;
            bestNext = nextHop;
        } else if (abs(linkRSSI - bestRSSI) <= 2 && routes[i].hops < bestHops) {
            bestHops = routes[i].hops;
            bestNext = nextHop;
        }
    }

    return bestNext;
}

bool reliableTransmit(MeshPacket &pkt, uint8_t maxRetries) {
    for (int attempt = 0; attempt < maxRetries; attempt++) {

        if (channelBusy()) {
            delay(getBackoff(PRI_ACK));
        }

        int state = radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));

        if (state == RADIOLIB_ERR_NONE) {
            delay(10);
            radio.setPacketReceivedAction(setFlag);
            return true; // success
        }

        Serial.print("TX failed, retry ");
        Serial.println(attempt + 1);

        delay(50 + random(0, 50));
    }

    return false; // failed after retries
}

void handleTX() {
	buttonState = digitalRead(BUTTON_PIN);
    if (buttonState != lastButtonState) {
        if (buttonState == LOW) {

            if (pendingACK) {
                Serial.println("Waiting for ACK, ignoring button press");
                lastButtonState = buttonState;
                return;
            }

            Serial.println("Button Pressed Once!");

            static uint16_t seqCounter = 0;

            MeshPacket pkt = {};
            pkt.src = NODE_ID;
            pkt.lastHop = NODE_ID;
            pkt.dst = 3;            
            pkt.ttl = 5;
            pkt.seq = seqCounter++;

            pendingAckSeq = pkt.seq;
            ackReceivedFlag = false;
            ackWaitStart = millis();

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
                flashNoRoute = true;
            } else {
                flashNoRoute = false;
                delay(random(40, 200));
                radio.standby();
                if (channelBusy()) {
                    Serial.println("GPS delayed due to busy channel");
                    delay(getBackoff(PRI_GPS));
                }
                radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));
                lastRXActivity = millis();
                delay(10);
                radio.setPacketReceivedAction(setFlag);
                radio.startReceive();

                pendingACK = true;
                pendingPkt = pkt;
                pendingSeq = pkt.seq;
                pendingSentAt = millis();
                pendingRetries = 0;
            }

            // Start non-blocking flash — loop() drives it via updateDisplay()
            dispState = DS_SOS_FLASH;
            flashFrame = 0;
            flashFrameAt = millis();
        }
        lastButtonState = buttonState;
    }
}

void handleRX() {
    if (!receivedFlag) return;
    Serial.println("handleRX triggered");
    receivedFlag = false;
    lastRXActivity = millis();

    uint8_t inBuf[sizeof(MeshPacket)];
    int len = radio.getPacketLength();

    if (len > sizeof(MeshPacket)) {
        radio.startReceive();
        return;
    }

    int state = radio.readData(inBuf, len);
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();

    MeshPacket *rawPkt = (MeshPacket*)inBuf;
    Serial.print("RAW RX: type=");
    Serial.print(rawPkt->type);
    Serial.print(" src=");
    Serial.print(rawPkt->src);
    Serial.print(" dst=");
    Serial.println(rawPkt->dst);

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
    /*
    bool isDuplicate = seenBefore(pkt->src, pkt->seq);

    if (pkt->type != PKT_HELLO && isDuplicate) {
        radio.startReceive();
        return;
    }
    */


    /*
    if (pkt->type != PKT_HELLO && hasSeen(pkt->src, pkt->seq)) {
        Serial.print("Dropped duplicate src=");
        Serial.print(pkt->src);
        Serial.print(" seq=");
        Serial.println(pkt->seq);
        radio.startReceive();
        return;
    }
    */

    bool isDuplicate = hasSeen(pkt->src, pkt->seq);

    if (pkt->type != PKT_HELLO && isDuplicate) {

        if (pkt->dst == NODE_ID) {
            Serial.println("Duplicate received at destination → re-sending ACK");

        } else {
            Serial.print("Dropped duplicate src=");
            Serial.print(pkt->src);
            Serial.print(" seq=");
            Serial.println(pkt->seq);
            radio.startReceive();
            return;
        }
    }

    // HELLO packets update neighbors immediately
    if (pkt->type == PKT_HELLO) {
        Serial.println("Hello Received"); 
        updateNeighbors(pkt, (int16_t)rssi);
        radio.startReceive();
        return;
    }

    if (pkt->type == PKT_ACK && pkt->dst == NODE_ID) {
        ACKPayload ackPayload;
        memcpy(&ackPayload, pkt->payload, sizeof(ACKPayload));
        Serial.print("ACK received for seq=");
        Serial.println(ackPayload.originalSeq);

        if (pendingACK && ackPayload.originalSeq == pendingSeq) {
            pendingACK = false;
            Serial.println("Pending ACK cleared");
        }

        u8g2.clearBuffer();
        drawTopHalf();
        drawConfirmed();
        u8g2.sendBuffer();
        dispState = DS_CONFIRMED;
        radio.startReceive();
        return;
    }

    if (pkt->type == PKT_GPS) {
        Serial.print("RX from node ");
        Serial.print(pkt->src);
        Serial.print(" lastHop=");
        Serial.println(pkt->lastHop);

    }

    // Packet reached final destination
    if (pkt->type == PKT_GPS && pkt->dst == NODE_ID) {
        float rxLat, rxLng;
        uint8_t rxNode;
        parsePacket(pkt->payload, rxLat, rxLng, rxNode);

        rxDisplayLat = rxLat;
        rxDisplayLng = rxLng;
        rxDisplaySrc = pkt->src;

        u8g2.clearBuffer();
        drawTopHalf();
        u8g2.setFont(u8g2_font_logisoso16_tr);
        const char* sosRx = "SOS RECEIVED";
        u8g2.drawStr((128 - u8g2.getStrWidth(sosRx)) / 2, 52, sosRx);
        u8g2.sendBuffer();
        dispState = DS_SOS_RECEIVED;

        MeshPacket ack = {};
        ack.src = NODE_ID;
        ack.lastHop = NODE_ID;
        ack.dst = pkt->src;
        ack.ttl = 5;
        ack.type = PKT_ACK;

        static uint16_t ackSeq = 0;
        ack.seq = ackSeq++;

        ACKPayload ackPayload = {};
        ackPayload.originalSrc = pkt->src;
        ackPayload.originalSeq = pkt->seq;
        memcpy(ack.payload, &ackPayload, sizeof(ACKPayload));

        ack.nextHop = selectNextHop(ack.dst, NODE_ID);
        Serial.print("ACK nextHop resolved: ");
        Serial.println(ack.nextHop); 

        if (ack.nextHop == 0) {
            Serial.println("No route for ACK, using lastHop fallback");
            ack.nextHop = pkt->lastHop;
        }

        if (ack.nextHop != 0) {
            Serial.print("Sending ACK to node ");
            Serial.print(ack.dst);
            Serial.print(" via nextHop=");
            Serial.println(ack.nextHop);
            delay(random(20, 80));
            radio.standby();

            bool success = reliableTransmit(ack, 3);
            Serial.print("ACK transmit success: ");
            Serial.println(success ? "YES" : "NO");

            if (!success) {
                Serial.println("ACK failed after retries");
            }

            lastRXActivity = millis();
            radio.setPacketReceivedAction(setFlag);
        }
        markSeen(pkt->src, pkt->seq);
        radio.startReceive();
        return;
    }

    // Forward if not final destination
    if ((pkt->type == PKT_GPS || pkt->type == PKT_ACK) && pkt->dst != NODE_ID && pkt->ttl > 0) {

        // Only forward if this node is the intended nextHop
        if (pkt->nextHop == NODE_ID || pkt->nextHop == MESH_BROADCAST) {

            if (pkt->lastHop == NODE_ID) return;

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

            Serial.print("Forwarding ");
            Serial.print(pkt->type == PKT_ACK ? "ACK" : "packet");
            Serial.print(" to node: ");
            Serial.println(pkt->nextHop);

            fwdDots = (fwdDots % 3) + 1;
            const char* fwdSuffixes[] = {".", "..", "..."};
            char fwdStr[16];
            snprintf(fwdStr, sizeof(fwdStr), "Forwarding%s", fwdSuffixes[fwdDots - 1]);
            u8g2.clearBuffer();
            drawTopHalf();
            u8g2.setFont(u8g2_font_6x10_tr);
            u8g2.drawStr(0, 52, fwdStr);
            u8g2.sendBuffer();

            delay(random(40, 200));
            //radio.standby(); 

            int attempts = 0;
            while (channelBusy() && attempts < 3) {
                if (pkt->type == PKT_ACK) {
                    delay(getBackoff(PRI_ACK));
                } else {
                    delay(getBackoff(PRI_GPS));
                }
                attempts++;
            }

            //default send 
            radio.transmit((uint8_t*)pkt, len);
            markSeen(pkt->src, pkt->seq); 
            lastRXActivity = millis();
            delay(10);
            radio.setPacketReceivedAction(setFlag);
            radio.startReceive();
            u8g2.clearBuffer();
            drawTopHalf();
            drawTXHome();
            u8g2.sendBuffer();
        }
    }

    radio.startReceive();
}

void readBattery() {
    battUSB = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
    if (battUSB) {
        Serial.println("[BATT] Voltage: --V | Level: --% | USB: YES");
        return;
    }

    volatile uint32_t raw = 0;
    NRF_SAADC->ENABLE = 1;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
    NRF_SAADC->CH[0].CONFIG =
        (SAADC_CH_CONFIG_GAIN_Gain1_4   << SAADC_CH_CONFIG_GAIN_Pos)   |
        (SAADC_CH_CONFIG_MODE_SE        << SAADC_CH_CONFIG_MODE_Pos)   |
        (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos);
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->RESULT.PTR   = (uint32_t)&raw;
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->TASKS_START = 1;
    while (!NRF_SAADC->EVENTS_STARTED);
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (!NRF_SAADC->EVENTS_END);
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_STOP = 1;
    while (!NRF_SAADC->EVENTS_STOPPED);
    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->ENABLE = 0;

    float voltage = ((float)raw * 2.4f / 4095.0f) * 5.0f;
    int pct = (int)(((voltage - 3.0f) / (4.2f - 3.0f)) * 100.0f);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    battPct = (uint8_t)pct;

    Serial.print("[BATT] Voltage: ");
    Serial.print(voltage, 2);
    Serial.print("V | Level: ");
    Serial.print(battPct);
    Serial.println("% | USB: NO");
}

void bootScreen() {
    u8g2.begin();
    const char* text = "SAFETRACK";
    u8g2.setFont(u8g2_font_logisoso20_tr);
    int fullW = u8g2.getStrWidth(text);
    int startX = (128 - fullW) / 2;
    int y = (64 + 20) / 2;

    char partial[11];
    for (int i = 1; i <= 9; i++) {
        strncpy(partial, text, i);
        partial[i] = '\0';
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_logisoso20_tr);
        u8g2.drawStr(startX, y, partial);
        u8g2.sendBuffer();
        delay(100);
    }
    delay(2850);
}

void setup(){
    bootScreen();
    Serial.begin(115200);
	delay(1000); 
	pinMode(LED, OUTPUT); //set output mode
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	digitalWrite(LED, LOW);

	for(int i = 0; i < MAX_NEIGHBORS; i++){
        neighbors[i].nodeID = 0;
        neighbors[i].lastSeen = 0;
        neighbors[i].rssi = -200;
    }

	for(int i = 0; i < MAX_ROUTES; i++){
    	routes[i].dest = 0;
        routes[i].nextHop = 0;
        routes[i].hops = 0;    
        routes[i].lastUpdated = 0; 
        routes[i].linkRSSI = -128;
	}
	
	gpsSerial.begin(9600);
	//SPI.begin(); // uses variant default SPI pins

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
    u8g2.clearBuffer();
    drawTopHalf();
    drawTXHome();
    u8g2.sendBuffer();

    readBattery();
    lastBattSample = millis();

    delay(random(1000, 4000));

}

void updateDisplay() {
    unsigned long now = millis();

    if (dispState == DS_SOS_FLASH) {
        if (now - flashFrameAt >= 500) {
            flashFrameAt = now;
            flashFrame++;
            bool showOn = (flashFrame % 2 == 1);
            u8g2.clearBuffer();
            drawTopHalf();
            if (showOn) drawSOS();
            u8g2.sendBuffer();

            if (flashFrame >= 10) {
                if (flashNoRoute) {
                    u8g2.clearBuffer();
                    drawTopHalf();
                    u8g2.setFont(u8g2_font_logisoso16_tr);
                    const char* sf = "SEND FAILED";
                    u8g2.drawStr((128 - u8g2.getStrWidth(sf)) / 2, 52, sf);
                    u8g2.sendBuffer();
                    dispState = DS_SEND_FAILED;
                    sendFailedAt = now;
                } else {
                    u8g2.clearBuffer();
                    drawTopHalf();
                    drawSOS();
                    u8g2.sendBuffer();
                    dispState = DS_SOS_SENT;
                }
            }
        }
    }

    if (dispState == DS_SEND_FAILED) {
        if (now - sendFailedAt >= 5000) {
            u8g2.clearBuffer();
            drawTopHalf();
            drawTXHome();
            u8g2.sendBuffer();
            dispState = DS_HOME;
        }
    }
}

void loop() {
	const size_t BUF_SZ = 256;

    if (millis() - lastBattSample >= 30000) {
        readBattery();
        lastBattSample = millis();
    }

    // Read and decode GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    //Checking that packets are actually being received
    if (millis() - lastRXActivity > 55000) {
        Serial.println("WATCHDOG: restarting RX mode");
        radio.setPacketReceivedAction(setFlag); 
        radio.startReceive();
        lastRXActivity = millis();
    }

    if (pendingACK && millis() - pendingSentAt > ACK_TIMEOUT) {
        if (pendingRetries < MAX_RETRIES) {
            pendingRetries++;
            Serial.print("ACK timeout - retry ");
            Serial.print(pendingRetries);
            Serial.print("/");
            Serial.println(MAX_RETRIES);

            pendingPkt.nextHop = selectNextHop(pendingPkt.dst, NODE_ID);
            pendingPkt.lastHop = NODE_ID;

            if (pendingPkt.nextHop == 0) {
                Serial.println("No route yet, will retry again");
                pendingSentAt = millis();
                pendingRetries--;
            } else {
                char retryStr[20];
                snprintf(retryStr, sizeof(retryStr), "Retry %d/%d...", pendingRetries, MAX_RETRIES);
                u8g2.clearBuffer();
                drawTopHalf();
                u8g2.setFont(u8g2_font_6x10_tr);
                u8g2.drawStr(0, 52, retryStr);
                u8g2.sendBuffer();

                if (channelBusy()) delay(getBackoff(PRI_GPS));
                radio.standby();
                radio.transmit((uint8_t*)&pendingPkt, sizeof(MeshPacket));
                lastRXActivity = millis();
                delay(10);
                radio.setPacketReceivedAction(setFlag);
                radio.startReceive();
                pendingSentAt = millis();
                u8g2.clearBuffer();
                drawTopHalf();
                drawSOS();
                u8g2.sendBuffer();
                dispState = DS_SOS_SENT;
            }
        } else {
            Serial.println("Max retries reached - SEND FAILED");
            pendingACK = false;
            u8g2.clearBuffer();
            drawTopHalf();
            u8g2.setFont(u8g2_font_logisoso16_tr);
            const char* sf = "SEND FAILED";
            u8g2.drawStr((128 - u8g2.getStrWidth(sf)) / 2, 52, sf);
            u8g2.sendBuffer();
            dispState = DS_SEND_FAILED;
            sendFailedAt = millis();
        }
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

    static unsigned long lastNeighborCleanup = 0;
    if (millis() - lastNeighborCleanup > 15000) {
        lastNeighborCleanup = millis();
        cleanStaleNeighbors();
    }
 

    /*
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
    */



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
    updateDisplay();

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
