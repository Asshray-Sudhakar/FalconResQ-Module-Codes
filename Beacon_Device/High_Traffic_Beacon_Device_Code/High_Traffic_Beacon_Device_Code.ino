/*
 * To be used only if the there are high number of packets/beacon devices which can cause
    loss of data packets, therfore we randomly send the data packets at 4 different channels
 * FalconResQ Beacon Module
 * Heltec Wireless Tracker (ESP32-S3 + SX1262 + GPS)
 * 
 * Features:
 * - GPS location tracking
 * - Button-activated SOS transmission
 * - Slotted p-persistent CSMA with CAD
 * - Multi-channel operation
 * 
 * Device ID: 001
 */

#include "Arduino.h"
#include "HT_TinyGPS++.h"
#include "LoRaWan_APP.h"

// ============ CONFIGURATION ============
#define DEVICE_ID 1                     // Beacon ID
#define VGNSS_CTRL 3                    // GPS power control pin
#define BUTTON_PIN 0                    // PRG/USER button (GPIO 0)

// LoRa Configuration for India SRD Band
#define RF_FREQUENCY_CH0    866100000   // 866.1 MHz - Primary channel
#define RF_FREQUENCY_CH1    866300000   // 866.3 MHz
#define RF_FREQUENCY_CH2    866500000   // 866.5 MHz  
#define RF_FREQUENCY_CH3    866700000   // 866.7 MHz

#define TX_OUTPUT_POWER     17          // dBm
#define LORA_BANDWIDTH      0           // 125 kHz
#define LORA_SPREADING_FACTOR 7         // SF7
#define LORA_CODINGRATE     1           // 4/5
#define LORA_PREAMBLE_LENGTH 12         // Symbols
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// MAC Protocol Parameters
#define FRAME_LENGTH_MS     60000       // 60 seconds
#define UPLINK_WINDOW_MS    55000       // 55 seconds for uplink
#define SLOT_LENGTH_MS      120         // 120ms per slot
#define SLOTS_PER_FRAME     (UPLINK_WINDOW_MS / SLOT_LENGTH_MS)
#define TX_PROBABILITY      0.1         // p = 0.1 for normal mode
#define SOS_TX_PROBABILITY  0.8         // p = 0.8 for SOS mode

// ============ GLOBAL VARIABLES ============
TinyGPSPlus GPS;
static RadioEvents_t RadioEvents;

char txpacket[100];
bool lora_idle = true;
bool transmission_active = false;
bool button_pressed = false;
unsigned long last_button_check = 0;
unsigned long frame_start_time = 0;
int selected_slot = 0;
uint32_t selected_channel = RF_FREQUENCY_CH0;

// ============ FUNCTION DECLARATIONS ============
void OnTxDone(void);
void OnTxTimeout(void);
void OnCadDone(bool channelActivityDetected);
void GPS_Init(void);
void LoRa_Init(void);
void check_button(void);
void select_random_slot_and_channel(void);
bool perform_CAD(void);
void transmit_packet(void);
void build_packet(char* buffer);

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== FalconResQ Beacon Module ===");
    Serial.println("Device ID: 001");
    Serial.println("================================\n");
    
    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize GPS
    GPS_Init();
    
    // Initialize LoRa
    LoRa_Init();
    
    Serial.println("System Ready!");
    Serial.println("Be Safe & Alert.");
    Serial.println("Incase of rescue press the button\n");
}

// ============ MAIN LOOP ============
void loop() {
    // Check button every 100ms
    if (millis() - last_button_check > 100) {
        check_button();
        last_button_check = millis();
    }
    
    // Update GPS data
    while (Serial1.available() > 0) {
        GPS.encode(Serial1.read());
    }
    
    // If transmission not active, just wait
    if (!transmission_active) {
        delay(10);
        return;
    }
    
    // ===== Slotted Transmission Logic =====
    unsigned long current_time = millis();
    unsigned long time_in_frame = current_time - frame_start_time;
    
    // Check if we're in a new frame
    if (time_in_frame >= FRAME_LENGTH_MS) {
        // New frame starts
        frame_start_time = current_time;
        time_in_frame = 0;
        select_random_slot_and_channel();
    }
    
    // Check if we're in the uplink window
    if (time_in_frame < UPLINK_WINDOW_MS) {
        // Calculate current slot
        int current_slot = time_in_frame / SLOT_LENGTH_MS;
        
        // Is it our slot?
        if (current_slot == selected_slot && lora_idle) {
            // Perform CAD (Channel Activity Detection)
            if (perform_CAD()) {
                // Channel is clear, decide to transmit with probability p
                float p = SOS_TX_PROBABILITY; // High priority for SOS
                float random_val = random(0, 100) / 100.0;
                
                if (random_val < p) {
                    transmit_packet();
                }
            }
            // If CAD detected activity or we skipped, we'll try next frame
            lora_idle = false; // Prevent multiple attempts in same slot
        }
    }
    
    Radio.IrqProcess();
}

// ============ GPS INITIALIZATION ============
void GPS_Init(void) {
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, HIGH);  // Power on GPS
    Serial1.begin(115200, SERIAL_8N1, 33, 34);
    Serial.println("GPS Module Initialized");
    delay(100);
}

// ============ LoRa INITIALIZATION ============
void LoRa_Init(void) {
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.CadDone = OnCadDone;
    
    Radio.Init(&RadioEvents);
    Radio.SetChannel(selected_channel);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    
    Serial.println("LoRa Module Initialized");
    Serial.printf("Frequency: %.1f MHz\n", selected_channel / 1000000.0);
    Serial.printf("SF: %d, BW: 125kHz, CR: 4/5\n", LORA_SPREADING_FACTOR);
}

// ============ BUTTON CHECK ============
void check_button(void) {
    if (digitalRead(BUTTON_PIN) == LOW && !button_pressed) {
        button_pressed = true;
        delay(50); // Debounce
        
        if (digitalRead(BUTTON_PIN) == LOW) {
            // Button confirmed pressed
            transmission_active = true;
            frame_start_time = millis();
            select_random_slot_and_channel();
            
            Serial.println("\n*** TRANSMISSION STARTED ***");
            Serial.println("SOS Mode Activated!");
            Serial.println("Beacon will transmit location data...\n");
        }
    } else if (digitalRead(BUTTON_PIN) == HIGH) {
        button_pressed = false;
    }
}

// ============ SLOT & CHANNEL SELECTION ============
void select_random_slot_and_channel(void) {
    // Select random slot in uplink window
    selected_slot = random(0, SLOTS_PER_FRAME);
    
    // Select random channel (CH0, CH1, CH2, or CH3)
    int channel_index = random(0, 4);
    switch (channel_index) {
        case 0: selected_channel = RF_FREQUENCY_CH0; break;
        case 1: selected_channel = RF_FREQUENCY_CH1; break;
        case 2: selected_channel = RF_FREQUENCY_CH2; break;
        case 3: selected_channel = RF_FREQUENCY_CH3; break;
    }
    
    // Update radio channel
    Radio.SetChannel(selected_channel);
    lora_idle = true; // Ready for next transmission
    
    Serial.printf("Next TX: Slot %d, CH: %.1f MHz\n", 
                  selected_slot, selected_channel / 1000000.0);
}

// ============ CAD (Channel Activity Detection) ============
bool perform_CAD(void) {
    // For simplicity, we'll use a basic RSSI check
    // In production, use Radio.StartCad() with proper callback
    Radio.Rx(0);
    delay(10); // Brief listen
    int16_t rssi = Radio.Rssi(MODEM_LORA);
    Radio.Standby();
    
    // If RSSI > -100 dBm, consider channel busy
    if (rssi > -100) {
        Serial.println("CAD: Channel BUSY");
        return false;
    }
    
    Serial.println("CAD: Channel CLEAR");
    return true;
}

// ============ CRC CALCULATION ============
uint16_t calculate_crc16(const char* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

// ============ BUILD PACKET ============
void build_packet(char* buffer) {
    // Get current GPS data
    float lat = GPS.location.isValid() ? GPS.location.lat() : 0.0;
    float lon = GPS.location.isValid() ? GPS.location.lng() : 0.0;
    
    // Get time
    char timeStr[6] = "00:00";
    if (GPS.time.isValid()) {
        sprintf(timeStr, "%02d:%02d", GPS.time.hour(), GPS.time.minute());
    }
    
    // Build JSON packet without CRC first
    char temp_packet[80];
    sprintf(temp_packet, "{\"ID\":%d,\"LAT\":%.6f,\"LON\":%.6f,\"TIME\":\"%s\"}",
            DEVICE_ID, lat, lon, timeStr);
    
    // Calculate CRC16 for the packet
    uint16_t crc = calculate_crc16(temp_packet, strlen(temp_packet));
    
    // Add CRC to final packet
    sprintf(buffer, "{\"ID\":%d,\"LAT\":%.6f,\"LON\":%.6f,\"TIME\":\"%s\",\"CRC\":\"%04X\"}",
            DEVICE_ID, lat, lon, timeStr, crc);
}

// ============ TRANSMIT PACKET ============
void transmit_packet(void) {
    build_packet(txpacket);
    
    Serial.println("\n----- Transmitting -----");
    Serial.println(txpacket);
    Serial.printf("Length: %d bytes\n", strlen(txpacket));
    Serial.printf("Channel: %.1f MHz\n", selected_channel / 1000000.0);
    Serial.println("------------------------\n");
    
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    lora_idle = false;
}

// ============ RADIO CALLBACKS ============
void OnTxDone(void) {
    Serial.println("✓ TX Success");
    lora_idle = true;
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("✗ TX Timeout");
    lora_idle = true;
}

void OnCadDone(bool channelActivityDetected) {
    if (channelActivityDetected) {
        Serial.println("CAD: Activity detected");
    } else {
        Serial.println("CAD: Channel clear");
    }
}