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

// Definitions
#define LED (0 + 15)
#define OLED_RESET -1
#define gpsSerial Serial1
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define BUTTON_PIN (0+32)
#define MAX_NEIGHBORS 10

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
    unsigned long lastSeen; // millis() timestamp
};

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
  // we got a packet, set the flag
  receivedFlag = true;
}

bool seenBefore(uint8_t src, uint16_t seq) {
  // check cache
  for (int i = 0; i < DUP_CACHE_SIZE; i++) {
    if (seenCache[i].src == src &&
        seenCache[i].seq == seq) {
      return true;  // duplicate found
    }
  }

  // not seen → store it (circular buffer)
  seenCache[seenIndex].src = src;
  seenCache[seenIndex].seq = seq;
  seenIndex = (seenIndex + 1) % DUP_CACHE_SIZE;

  return false;
}

void setup(){
	pinMode(LED, OUTPUT); //set output mode
	pinMode(BUTTON_PIN, INPUT);
	digitalWrite(LED, LOW);

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
	display.print("no message recv");
	display.display();
}

void loop() {
	const size_t BUF_SZ = 256;

    // Read and decode GPS data
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
            snprintf(dateBuffer, sizeof(dateBuffer), "Date: %02d-%02d-%02d", date.year(), date.month(), date.day());

            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // Set color for date (also for overwr)
            display.setCursor(0, 20); // curosr pos for date
            display.print(dateBuffer);

            display.display();
        }
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

		radio.standby();
		radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));
		radio.startReceive();

		Serial.println("HELLO sent");
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

				radio.standby();
				radio.transmit((uint8_t*)&pkt, sizeof(MeshPacket));
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
			if (state != RADIOLIB_ERR_NONE) {
				radio.startReceive();
				return;
			}

			// interpret as mesh packet
			MeshPacket *pkt = (MeshPacket*)inBuf;

			// duplicate check
			if (pkt->type != PKT_HELLO && seenBefore(pkt->src, pkt->seq)) {
				Serial.println("Duplicate packet dropped");
				radio.startReceive();
				return;
			}

			// HELLO packet handling 
			if (pkt->type == PKT_HELLO) {
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
							break;
						}
					}
				}
				Serial.print("Neighbor updated: ");
				Serial.println(pkt->src);
			}

			//GPS packet handling
			if (pkt->type == PKT_GPS && (pkt->dst == NODE_ID || pkt->dst == MESH_BROADCAST)) {
				float rxLat, rxLng;
				uint8_t rxNode;
				parsePacket(pkt->payload, rxLat, rxLng, rxNode);

				Serial.print("RX from node ");
				Serial.print(pkt->src);
				Serial.print(" Lat=");
				Serial.print(rxLat, 6);
				Serial.print(" Lng=");
				Serial.println(rxLng, 6);

				display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
				display.setCursor(0, 40);
				display.print("RX ");
				display.print(pkt->src);
				display.print(": ");
				display.print(rxLat, 4);
				display.print(",");
				display.print(rxLng, 4);
				display.display();

				Serial.print("RSSI: "); Serial.println(radio.getRSSI());
				Serial.print("SNR: "); Serial.println(radio.getSNR());
			}

			//  forward packet
			if (pkt->ttl > 0 && pkt->dst != NODE_ID) {
				pkt->ttl--;
				pkt->lastHop = NODE_ID;

				delay(random(40, 200)); // small random delay to avoid collisions
				radio.transmit((uint8_t*)pkt, len);
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
      s += c;            // printable ASCII
    } else {
      s += '.';          // non-printable → dot
    }
  }
  return s;
}
