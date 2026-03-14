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
//#define NODE_ID 1
#define ROOT_NODE 3

#define DUP_CACHE_SIZE 8 

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
int buttonState;
int lastButtonState = 0; 

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

struct SeenPacket {
  uint8_t src;
  uint16_t seq;
};

SeenPacket seenCache[DUP_CACHE_SIZE];
uint8_t seenIndex = 0;

PongPacket pongList[MAX_PONGS];
uint8_t pongCount = 0;
bool collectingPongs = false;
unsigned long collectPongStartTime = 0;

uint16_t seqCounter = 0;

MeshPacket currentPkt = {};

// Declarations Functions
void setFlag(void);
void readGPS();
bool seenBefore(uint8_t src, uint16_t seq);
void sendPing();
uint8_t nextNeighbor();
void handleTX();
void handleRX();
void setup();
void loop();

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void readGPS() {

    double lng = gps.location.lng();
    double lat = gps.location.lat();
    

    if (lat != lastLat || lng != lastLng) {
        lastLat = lat;
        lastLng = lng;

        display.setCursor(0, 0);
        display.print("Lng: ");
        display.print(lng, 6);

        display.setCursor(0, 10);
        display.print("Lat: ");
        display.print(lat, 6);

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

void sendPing() {
    MeshPacket pingPKT = {};
    pingPKT.src = NODE_ID;
    pingPKT.dst = MESH_BROADCAST;
    pingPKT.type = PKT_PING;

    radio.standby();
    radio.transmit((uint8_t *)&pingPKT, sizeof(pingPKT));
    radio.startReceive();

    pongCount = 0;
    collectingPongs = true;
    collectPongStartTime = millis();
    Serial.println("Pinging nearby nodes...");
}

uint8_t nextNeighbor() {
    int bestRSSI = -100;
    uint8_t nextNeighbor = 0;

    for(int i = 0; i < pongCount; i++) {
        bool visited = false;

        for(int j = 0; j < currentPkt.pathLen; j++) {
            if(currentPkt.path[j] == pongList[i].nodeID) {
                visited = true;
                break;
            }
        }

        if(!visited && pongList[i].rssi > bestRSSI) {
                bestRSSI = pongList[i].rssi;
                nextNeighbor = pongList[i].nodeID;
        }
    }
    return nextNeighbor;
}

void handleTX() {
    buttonState = digitalRead(BUTTON_PIN);
    if( buttonState != lastButtonState) {
        if (buttonState == LOW) {
            Serial.println("Button Pressed Once!");

            currentPkt.src = NODE_ID;
            currentPkt.dst = ROOT_NODE;

            currentPkt.ttl = 10;
            currentPkt.seq = seqCounter++;
            currentPkt.pathLen = 0;

            buildPacket(lastLat, lastLng, currentPkt.payload);

            sendPing();  
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

    if(pkt->type == PKT_PING && pkt->src != NODE_ID) {
        MeshPacket pongPKT = {};
        pongPKT.src = NODE_ID;
        pongPKT.dst = pkt->src;

        int16_t rssi = (int16_t)radio.getRSSI();
        memcpy(pongPKT.payload, &rssi, sizeof(rssi));

        delay(random(20,120));

        radio.transmit((uint8_t*)&pongPKT, sizeof(pongPKT));
        radio.startReceive();
        return;
    }

    if(pkt->type == PKT_PONG && collectingPongs) {
        int16_t rssi;
        memcpy(&rssi, pkt->payload, sizeof(rssi));

        if(pongCount < MAX_PONGS) {
            pongList[pongCount].nodeID = pkt->src;
            pongList[pongCount].rssi = rssi;
            pongCount++;
        } else {
            collectingPongs = false;
        }
        radio.startReceive();
        return;
    }

    if(pkt->type == PKT_GPS) {

        if(seenBefore(pkt->src, pkt->seq)) {
            radio.startReceive();
            return;
        }

        if(pkt->dst == NODE_ID) {
            float rxLat, rxLng;
            uint8_t rxNode;
            parsePacket(pkt->payload, rxLat, rxLng, rxNode);
            
            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
            display.setCursor(0, 40);
            display.print("RX ");
            display.print(pkt->src);
            display.print(": ");
            display.print(rxLng, 4);
            display.print(",");
            display.print(rxLat, 4);
            radio.startReceive();
            return;
            
        }

        memcpy(&currentPkt, pkt, sizeof(MeshPacket));

        if(currentPkt.ttl > 0) {
            currentPkt.ttl--;
            sendPing();
            radio.startReceive();
            return;
        }
    }  
    radio.startReceive();
}

void setup(){
    Serial.begin(9600); 
	delay(1000); 
	pinMode(LED, OUTPUT); //set output mode
	pinMode(BUTTON_PIN, INPUT);
	digitalWrite(LED, LOW);

	
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
}

void loop() {
	//const size_t BUF_SZ = 256;

    // Read and decode GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    //bool locationUpdated = gps.location.isUpdated();
    bool dateUpdated = gps.date.isUpdated();

	readGPS(); 

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

	
	
	handleRX();
    handleTX();

    if(collectingPongs && millis() - collectPongStartTime > 3000){
        collectingPongs = false;
        uint8_t next = nextNeighbor();

        if(next != 0) {
            currentPkt.nextHop = next;
            currentPkt.path[currentPkt.pathLen++] = NODE_ID;

            radio.standby();
            radio.transmit((uint8_t*)&currentPkt, sizeof(currentPkt));
            radio.startReceive();
        } else {
            Serial.println("NO NEIGHBOR FOUND");
            return;
        }
    } 

}