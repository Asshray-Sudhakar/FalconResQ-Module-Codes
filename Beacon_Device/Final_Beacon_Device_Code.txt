/*
 * FalconResQ Beacon - SIMPLE MVP VERSION
 * Heltec Wireless Tracker (ESP32-S3 + SX1262 + GPS)
 * 
 * Simple Operation:
 * - Button activated transmission
 * - Transmits every 5 seconds on single channel (866.1 MHz)
 * - JSON packet with CRC validation
 * 
 * Device ID: 001
 */

#include "Arduino.h"
#include "HT_TinyGPS++.h"
#include "LoRaWan_APP.h"

/* ================== TFT ADDITIONS ================== */
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

#define TFT_BL 21
#define VEXT_CTRL 3
#define STATUS_DOT_X 150
#define STATUS_DOT_Y 10
#define STATUS_DOT_R 4

bool blinkState = false;
unsigned long lastBlink = 0;
bool packetJustSent = false;
/* ================================================== */

// ================= UI STATE =================
enum UiState {
    UI_IDLE,
    UI_TRANSMITTING,
    UI_TX_SUCCESS,
    UI_TX_TIMEOUT
};

UiState uiState = UI_IDLE;
// ============================================

// ============ CONFIGURATION ============
#define DEVICE_ID 1
#define VGNSS_CTRL 3
#define BUTTON_PIN 0

#define RF_FREQUENCY        866100000
#define TX_OUTPUT_POWER     17
#define LORA_BANDWIDTH      0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE     1
#define LORA_PREAMBLE_LENGTH 12
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define TRANSMIT_INTERVAL_MS 5000

// ============ GLOBAL VARIABLES ============
TinyGPSPlus GPS;
static RadioEvents_t RadioEvents;

char txpacket[100];
bool lora_idle = true;
bool transmission_active = false;
bool button_pressed = false;

unsigned long last_tx_time = 0;
unsigned long last_button_check = 0;
int packet_count = 0;

// ============ FUNCTION DECLARATIONS ============
void OnTxDone(void);
void OnTxTimeout(void);
void GPS_Init(void);
void LoRa_Init(void);
void check_button(void);
void transmit_packet(void);
void build_packet(char* buffer);
uint16_t calculate_crc16(const char* data, size_t length);

/* ================== TFT UI ================== */
void drawStatusDot() {

    // -------- IDLE MODE --------
    // Red dot only on first page (before SOS)
    if (!transmission_active && uiState == UI_IDLE) {
        tft.fillCircle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, TFT_RED);
        return;
    }

    // -------- SOS MODE --------
    if (transmission_active) {

        // During actual RF transmission → NO DOT
        if (!lora_idle) {
            tft.fillCircle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, TFT_BLACK);
            return;
        }

        // Between transmissions → Solid GREEN dot
        tft.fillCircle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, TFT_GREEN);
        return;
    }

    // Default: clear
    tft.fillCircle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, TFT_BLACK);
}

void drawMainUI() {
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_CYAN);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.println("FalconResQ Beacon");

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(5, 20);
    tft.printf("Device ID: %03d", DEVICE_ID);

    tft.setCursor(5, 35);
    switch (uiState) {
        case UI_IDLE:
            tft.setTextColor(TFT_YELLOW);
            tft.println("Status: IDLE");
            tft.setCursor(5, 50);
            tft.setTextColor(TFT_WHITE);
            tft.println("Press button for SOS");
            break;

        case UI_TRANSMITTING:
            tft.setTextColor(TFT_GREEN);
            tft.println("TRANSMITTING...");
            tft.setCursor(5, 50);
            tft.setTextColor(TFT_WHITE);
            tft.printf("Packet #%d sent", packet_count);
            break;

        case UI_TX_SUCCESS:
            tft.setTextColor(TFT_GREEN);
            tft.println("TX SUCCESS");
            tft.setCursor(5, 50);
            tft.setTextColor(TFT_WHITE);
            tft.printf("Packet #%d sent", packet_count);
            break;

        case UI_TX_TIMEOUT:
            tft.setTextColor(TFT_RED);
            tft.println("TX TIMEOUT");
            tft.setCursor(5, 50);
            tft.setTextColor(TFT_WHITE);
            tft.printf("Packet #%d sent", packet_count);
            break;
    }
}

int tftCursorY = 20;

void tftPrintLine(const char* msg, uint16_t color) {
    tft.setTextColor(color, TFT_BLACK);
    tft.setCursor(5, tftCursorY);
    tft.println(msg);
    tftCursorY += 12;

    if (tftCursorY > 70) {
        tft.fillRect(0, 20, 160, 60, TFT_BLACK);
        tftCursorY = 20;
    }
}

/* ================================================= */

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, HIGH);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    tft.init();
    tft.setRotation(1);

    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   FalconResQ Beacon                   ║");
    Serial.println("║   Device ID: 001                      ║");
    Serial.println("╚═══════════════════════════════════════╝\n");

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    GPS_Init();
    LoRa_Init();

    Serial.println("✓ System Ready!\n");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("Be Safe & Alert.");
    Serial.println("Incase of rescue press the button");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_CYAN);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.println("FalconResQ Beacon");
    tftPrintLine("System Booting...", TFT_WHITE);
    tftPrintLine("GPS Initialized", TFT_GREEN);
    tftPrintLine("LoRa Initialized", TFT_GREEN);
    tftPrintLine("System Ready", TFT_CYAN);
    delay(800);   // small pause so user can read


    uiState = UI_IDLE;
    drawMainUI();
}

// ============ MAIN LOOP ============
void loop() {
    unsigned long current_time = millis();

    drawStatusDot();

    if (current_time - last_button_check > 100) {
        check_button();
        last_button_check = current_time;
    }

    while (Serial1.available() > 0) {
        GPS.encode(Serial1.read());
    }

    if (!transmission_active) {
        delay(10);
        return;
    }

    if (lora_idle && (current_time - last_tx_time >= TRANSMIT_INTERVAL_MS)) {
        transmit_packet();
        last_tx_time = current_time;
    }

    Radio.IrqProcess();
}

// ============ GPS INITIALIZATION ============
void GPS_Init(void) {
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, HIGH);
    Serial1.begin(115200, SERIAL_8N1, 33, 34);
    Serial.println("✓ GPS Module Initialized");
    delay(100);
}

// ============ LoRa INITIALIZATION ============
void LoRa_Init(void) {
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Serial.println("✓ LoRa Module Initialized");
    Serial.printf("  Channel: %.1f MHz\n", RF_FREQUENCY / 1000000.0);
    Serial.printf("  Power: %d dBm\n", TX_OUTPUT_POWER);
    Serial.printf("  SF: %d, BW: 125kHz, CR: 4/5\n", LORA_SPREADING_FACTOR);
    Serial.printf("  Interval: %d seconds\n\n", TRANSMIT_INTERVAL_MS / 1000);
}

// ============ BUTTON CHECK ============
void check_button(void) {
    if (digitalRead(BUTTON_PIN) == LOW && !button_pressed) {
        button_pressed = true;
        delay(50);

        if (digitalRead(BUTTON_PIN) == LOW) {
            transmission_active = true;
            last_tx_time = 0;
            packet_count = 0;

            Serial.println("\n╔═══════════════════════════════════════╗");
            Serial.println("║      TRANSMISSION ACTIVATED           ║");
            Serial.println("╚═══════════════════════════════════════╝");
            Serial.println("SOS Mode: Broadcasting location every 5s\n");

            uiState = UI_TRANSMITTING;
            drawMainUI();
        }
    } else if (digitalRead(BUTTON_PIN) == HIGH) {
        button_pressed = false;
    }
}

// ============ CRC ============
uint16_t calculate_crc16(const char* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// ============ BUILD PACKET ============
void build_packet(char* buffer) {
    float lat = GPS.location.isValid() ? GPS.location.lat() : 0.0;
    float lon = GPS.location.isValid() ? GPS.location.lng() : 0.0;

    char timeStr[6] = "00:00";
    if (GPS.time.isValid()) {
        int hour = GPS.time.hour();
        int minute = GPS.time.minute();
        minute += 30;
        if (minute >= 60) { minute -= 60; hour++; }
        hour += 5;
        if (hour >= 24) hour -= 24;
        sprintf(timeStr, "%02d:%02d", hour, minute);
    }

    char temp_packet[80];
    sprintf(temp_packet, "{\"ID\":%d,\"LAT\":%.6f,\"LON\":%.6f,\"TIME\":\"%s\"}",
            DEVICE_ID, lat, lon, timeStr);

    uint16_t crc = calculate_crc16(temp_packet, strlen(temp_packet));

    sprintf(buffer,
            "{\"ID\":%d,\"LAT\":%.6f,\"LON\":%.6f,\"TIME\":\"%s\",\"CRC\":\"%04X\"}",
            DEVICE_ID, lat, lon, timeStr, crc);
}

// ============ TRANSMIT PACKET ============
void transmit_packet(void) {
    packet_count++;
    build_packet(txpacket);

    Serial.println("┌─────────────────────────────────────┐");
    Serial.printf("│   TRANSMITTING [#%d]                 │\n", packet_count);
    Serial.println("└─────────────────────────────────────┘");
    Serial.printf(" %s\n", txpacket);
    Serial.println("┌─────────────────────────────────────┐");
    Serial.printf("│ Size: %d bytes                      │\n", strlen(txpacket));
    Serial.printf("│ Channel: %.1f MHz                  │\n", RF_FREQUENCY / 1000000.0);
    Serial.println("└─────────────────────────────────────┘\n");

    uiState = UI_TRANSMITTING;
    packetJustSent = true;
    drawMainUI();

    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    lora_idle = false;
}

// ============ RADIO CALLBACKS ============
void OnTxDone(void) {
    Serial.println("  ✓ TX Success");
    Serial.printf("  → Total Packets Sent: %d\n\n", packet_count);

    lora_idle = true;
    packetJustSent = false;

    uiState = UI_TX_SUCCESS;
    drawMainUI();
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("  ✗ TX Timeout");
    Serial.printf("  → Total Packets Sent: %d\n\n", packet_count);

    lora_idle = true;
    packetJustSent = false;

    uiState = UI_TX_TIMEOUT;
    drawMainUI();
}

