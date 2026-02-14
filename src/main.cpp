#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <SPI.h>
#include "gpsPacket.h"
#include "meshPacket.h"
#include "nodeIdentity.h"
#include "battery.h"

// ============= PIN DEFINITIONS =============
#define LED            (0 + 15)
#define OLED_RESET     -1
#define GPS_SERIAL     Serial1
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 64

#define SX126X_CS      (32 + 13)
#define SX126X_DIO1    (0 + 10)
#define SX126X_BUSY    (0 + 29)
#define SX126X_RESET   (0 + 9)
#define SX126X_RXEN    (0 + 17)
#define SX126X_TXEN    RADIOLIB_NC

#define BUTTON_PIN (0 + 32)

// ============= ROLE SELECTION =============
enum NodeState { RX_MODE, TX_MODE };
NodeState currentMode = RX_MODE;

// #define ROLE_TX
// #define ROLE_RX

// ============= BATTERY CONFIGURATION =============
#define BATTERY_PIN (0 + 1) //define the battery pin!! here
#define BATTERY_UPDATE_INTERVAL 1000

// ============= LORA SETTINGS =============
const float FREQ_MHZ = 906.875;
const uint8_t SF = 11;
const unsigned long BW = 250000;
const uint8_t CR = 5;
const uint16_t PREAMBLE = 16;
const uint8_t SYNCWORD = 0x2B;

// ============= GLOBAL OBJECTS =============
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
SX1262 radio = new Module(SX126X_CS, SX126X_DIO1, SX126X_RESET, SX126X_BUSY);
Battery battery(BATTERY_PIN, BATTERY_UPDATE_INTERVAL);

// ============= GLOBAL VARIABLES =============
char latBuffer[30], lngBuffer[30], dateBuffer[30];
double lastLat = 0, lastLng = 0;
TinyGPSDate lastDate;
volatile bool receivedFlag = false;
volatile bool shouldTransmit = false;

// button state variables
int buttonState = 0;
int lastButtonState = 0;

// node id
uint8_t selfId = 0;

// ============= FUNCTION DECLARATIONS =============
void setFlag(void);
void feedGPSParser();
void handleBatteryUpdate();
void updateBatteryDisplay();
// void drawBatteryIcon(int x, int y, float percent);
void buttonPushed();
void handleGPSLocation();
void handleDateUpdate();
void displayRadioReceived();
String bytesToAscii(const uint8_t *buf, size_t len);

// ============= ISR =============
void setFlag() {
    receivedFlag = true;
}

// ============= SETUP =============
void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    digitalWrite(LED, LOW);

    // GPS init
    GPS_SERIAL.begin(9600);

    // Display init
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        digitalWrite(LED, HIGH);
        while (true);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();

    // Battery init
    battery.begin();

    // Radio init
    Serial.print(F("[SX1262] Initializing ... "));
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) { delay(10); }
    }

    radio.setFrequency(FREQ_MHZ);
    radio.setSpreadingFactor(SF);
    radio.setBandwidth(BW);
    radio.setCodingRate(CR);
    radio.setPreambleLength(PREAMBLE);
    radio.setSyncWord(SYNCWORD);
    radio.setDio1Action(setFlag);

    Serial.print(F("[SX1262] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true) { delay(10); }
    }
    
    Serial.println("Boot successful...");
    delay(1000);
    display.clearDisplay();
    display.display();

    // nodeID
    selfId = NodeIdentity::getNodeId();
}

// ============= MAIN LOOP =============
void loop() {
    battery.update();
    if (battery.hasChanged(1.0)) {
        updateBatteryDisplay();
    }

    if (currentMode == RX_MODE) {
        feedGPSParser();
        if (gps.location.isUpdated()) {
            handleGPSLocation();
        }
        if (gps.date.isUpdated()) {
            handleDateUpdate();
        }
        if (receivedFlag) {
            receivedFlag = false;
            displayRadioReceived(); // maybe call this later? transmit first vs display firsy
            shouldTransmit = true; // check : 1. is this a head node? 2. should this be forwarded?
        }
        if (shouldTransmit) {
            currentMode = TX_MODE;
        }

    } else if (currentMode == TX_MODE) {
        buttonState = digitalRead(BUTTON_PIN);
        if (buttonState != lastButtonState) {
            buttonPushed();
        }
        currentMode = RX_MODE; // end of TX, back to RX
    }
}

// ============= COMMON FUNCTIONS =============
void updateBatteryDisplay() {
    float batteryPercent = battery.getPercentage();
    
    // Screen
    // CLEAR TOP RIGHT
    display.fillRect(100, 0, 28, 8, SSD1306_BLACK);
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(100, 0);
    display.print((int)batteryPercent);
    display.print("%");
    
    display.display();
    
    // Terminal
    Serial.print(battery.getVoltage());
    Serial.print("Battery: ");
    Serial.print(batteryPercent, 1);
    Serial.println("%");
}

// void drawBatteryIcon(int x, int y, float percent) {
//     display.drawRect(x, y, 20, 10, SSD1306_WHITE);
//     display.fillRect(x + 20, y + 3, 2, 4, SSD1306_WHITE);
//     int fillWidth = (int)((percent / 100.0) * 18);
//     display.fillRect(x + 1, y + 1, fillWidth, 8, SSD1306_WHITE);
// }

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

// ============= TX FUNCTIONS =============
void buttonPushed() {
    if (buttonState == 0) {
    Serial.println("Button Pressed Once!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("TX: Node Ready");
    display.display();

    static uint16_t seqCounter = 0;

    MeshPacket pkt = {};
    pkt.src = selfId;
    pkt.lastHop = selfId;
    pkt.dst = MESH_BROADCAST;
    pkt.ttl = 5;
    pkt.seq = seqCounter++;
    pkt.type = PKT_GPS;

    buildPacket(45.8239, -112.5641, pkt.payload);

    radio.standby();
    radio.transmit((uint8_t *)&pkt, sizeof(MeshPacket));
    radio.startReceive();
    }
    lastButtonState = buttonState;
}

// ============= RX FUNCTIONS =============
void feedGPSParser() {
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }
}

void handleGPSLocation() {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    bool redraw = false;

    if (lat != lastLat) {
        lastLat = lat;
        snprintf(latBuffer, sizeof(latBuffer), "Lat: %3.6f", lat);
        redraw = true;
    }
    
    if (lng != lastLng) {
        lastLng = lng;
        snprintf(lngBuffer, sizeof(lngBuffer), "Lng: %3.6f", lng);
        redraw = true;
    }

    if (redraw) {
        // Update OLED
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.setCursor(0, 0);
        display.print(latBuffer);
        display.setCursor(0, 10);
        display.print(lngBuffer);
        display.display();

        // Build and transmit GPS packet
        uint8_t outBuf[sizeof(GPSPacket)];
        buildPacket(lastLat, lastLng, outBuf);

        radio.standby();
        int st = radio.transmit(outBuf, sizeof(GPSPacket));
        
        if (st == RADIOLIB_ERR_NONE) {
            Serial.println("TX: GPS packet sent");
        } else {
            Serial.print("TX error: ");
            Serial.println(st);
        }

        radio.startReceive();
    }
}

void handleDateUpdate() {
    if (!gps.date.isUpdated()) {
        return;
    }

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

void displayRadioReceived() {
    uint8_t inBuf[sizeof(GPSPacket)];
    int state = radio.readData(inBuf, sizeof(GPSPacket));

    if (state == RADIOLIB_ERR_NONE) {
        float rxLat, rxLng;
        uint8_t rxNode;
        parsePacket(inBuf, rxLat, rxLng, rxNode);

        Serial.print("Node ");
        Serial.print(rxNode);
        Serial.print(" Lat=");
        Serial.print(rxLat, 6);
        Serial.print(" Lng=");
        Serial.println(rxLng, 6);

        // Update OLED
        display.setCursor(0, 0);
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.print("RX Node ");
        display.println(rxNode);
        
        display.setCursor(0, 10);
        display.print("Lat: ");
        display.println(rxLat, 4);
        
        display.setCursor(0, 20);
        display.print("Lng: ");
        display.println(rxLng, 4);
        
        display.setCursor(0, 30);
        display.print("RSSI: ");
        display.println(radio.getRSSI());
        
        display.setCursor(0, 40);
        display.print("SNR: ");
        display.println(radio.getSNR());
        
        display.display();

        // Debug info
        Serial.print("RSSI: ");
        Serial.println(radio.getRSSI());
        Serial.print("SNR: ");
        Serial.println(radio.getSNR());
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("CRC error!");
    }
    else {
        Serial.print("RX error: ");
        Serial.println(state);
    }

    // Re-enable RX mode
    radio.startReceive();
}