/*
 * FalconResQ Ground Station Receiver
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * 
 * Purpose:
 * - Receives aggregated packets from drone on 866.9 MHz
 * - Parses JSON arrays with multiple beacon locations
 * - Outputs data to Serial for Python/PC processing
 * - Supports high-traffic multi-beacon scenario
 * 
 * Output Format: Clean JSON for easy Python parsing
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"

// ============ CONFIGURATION ============
#define RF_FREQUENCY_GROUND 866900000   // 866.9 MHz - Drone relay channel

#define RX_OUTPUT_POWER     17          // dBm
#define LORA_BANDWIDTH      0           // 125 kHz
#define LORA_SPREADING_FACTOR 7         // SF7
#define LORA_CODINGRATE     1           // 4/5
#define LORA_PREAMBLE_LENGTH 12         // Symbols
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_BUFFER_SIZE      2500        // Large buffer for aggregated packets

// ============ GLOBAL VARIABLES ============
static RadioEvents_t RadioEvents;
char rxpacket[RX_BUFFER_SIZE];

int packets_received = 0;
int total_beacons_tracked = 0;
unsigned long last_packet_time = 0;

// ============ FUNCTION DECLARATIONS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void LoRa_Init(void);
void parse_and_display_aggregated(const char* packet, int16_t rssi, int8_t snr);
int count_beacons_in_packet(const char* packet);

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║  FalconResQ Ground Station           ║");
    Serial.println("║  Multi-Beacon High-Traffic Mode      ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    // Initialize board
    Serial.println("Initializing board...");
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    Serial.println("✓ Board initialized");
    
    // Setup radio callbacks
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    
    // Initialize radio
    Serial.println("Initializing radio...");
    Radio.Init(&RadioEvents);
    Serial.println("✓ Radio initialized");
    
    // Set RX channel
    Serial.printf("Setting frequency: %.1f MHz\n", RF_FREQUENCY_GROUND / 1000000.0);
    Radio.SetChannel(RF_FREQUENCY_GROUND);
    Serial.println("✓ Channel set");
    
    // Configure RX
    Serial.println("Configuring RX parameters...");
    Radio.SetRxConfig(MODEM_LORA, 
                      LORA_BANDWIDTH, 
                      LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 
                      0,
                      LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, 
                      LORA_FIX_LENGTH_PAYLOAD_ON,
                      0,
                      true,
                      0,
                      0,
                      LORA_IQ_INVERSION_ON, 
                      true);
    Serial.println("✓ RX configured");
    
    Serial.println("\nConfiguration:");
    Serial.printf("  Frequency:   %.1f MHz\n", RF_FREQUENCY_GROUND / 1000000.0);
    Serial.printf("  Bandwidth:   125 kHz\n");
    Serial.printf("  SF:          %d\n", LORA_SPREADING_FACTOR);
    Serial.printf("  CR:          4/%d\n", LORA_CODINGRATE + 4);
    Serial.printf("  Preamble:    %d symbols\n", LORA_PREAMBLE_LENGTH);
    Serial.println();
    
    // Start receiving
    Serial.println("Starting continuous RX mode...");
    Radio.Rx(0);
    Serial.println("✓ RX mode active - Listening for drone relay");
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("Waiting for aggregated beacon data...");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

// ============ MAIN LOOP ============
void loop() {
    // Heartbeat every 20 seconds
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 20000) {
        unsigned long time_since_last = (millis() - last_packet_time) / 1000;
        Serial.printf("[%lu s] 💚 Packets RX:%d | Total Beacons:%d | Last packet: %lu s ago\n", 
                      millis()/1000, packets_received, total_beacons_tracked, time_since_last);
        last_heartbeat = millis();
    }
    
    Radio.IrqProcess();
    delay(10);
}

// ============ COUNT BEACONS IN AGGREGATED PACKET ============
int count_beacons_in_packet(const char* packet) {
    int count = 0;
    const char* pos = packet;
    
    // Count occurrences of "ID": field
    while ((pos = strstr(pos, "\"ID\":")) != NULL) {
        count++;
        pos += 5;
    }
    
    return count;
}

// ============ PARSE AND DISPLAY AGGREGATED PACKET ============
void parse_and_display_aggregated(const char* packet, int16_t rssi, int8_t snr) {
    int beacon_count = count_beacons_in_packet(packet);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.printf("║  AGGREGATED PACKET [%d beacons]       ║\n", beacon_count);
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.printf("Relay RSSI: %d dBm | SNR: %d dB\n", rssi, snr);
    Serial.println("─────────────────────────────────────────");
    
    // Check if it's a JSON array
    if (packet[0] == '[') {
        Serial.println("\n📦 Individual Beacon Data:");
        Serial.println(packet);  // Print raw JSON array
        Serial.println();
        
        // Parse individual beacons (simplified display)
        const char* start = packet + 1;  // Skip opening '['
        int beacon_num = 1;
        
        while (*start != '\0' && *start != ']') {
            // Find next complete JSON object
            const char* obj_start = strchr(start, '{');
            if (!obj_start) break;
            
            const char* obj_end = strchr(obj_start, '}');
            if (!obj_end) break;
            
            // Extract and print individual beacon
            int len = obj_end - obj_start + 1;
            char beacon_json[300];
            strncpy(beacon_json, obj_start, len);
            beacon_json[len] = '\0';
            
            Serial.printf("  Beacon #%d: %s\n", beacon_num, beacon_json);
            beacon_num++;
            
            // Move to next
            start = obj_end + 1;
            if (*start == ',') start++;
        }
        
        total_beacons_tracked += beacon_count;
        
    } else {
        // Single beacon (fallback)
        Serial.println("\n📍 Single Beacon:");
        Serial.println(packet);
        total_beacons_tracked++;
    }
    
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("💾 Data ready for Python/CSV export");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

// ============ RADIO CALLBACKS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memset(rxpacket, 0, sizeof(rxpacket));
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    
    packets_received++;
    last_packet_time = millis();
    
    Serial.println("\n┌─────────────────────────────────────┐");
    Serial.printf("│ 📡 RELAY FROM DRONE [#%d]           │\n", packets_received);
    Serial.println("├─────────────────────────────────────┤");
    Serial.printf("│ Size: %d bytes                      │\n", size);
    Serial.printf("│ RSSI: %d dBm  SNR: %d dB           │\n", rssi, snr);
    Serial.println("└─────────────────────────────────────┘");
    
    // Parse and display the aggregated data
    parse_and_display_aggregated(rxpacket, rssi, snr);
    
    // Continue receiving
    Radio.Rx(0);
}

void OnRxTimeout(void) {
    static unsigned long timeout_count = 0;
    timeout_count++;
    
    // Show timeout every 100 times to reduce spam
    if (timeout_count % 100 == 0) {
        Serial.printf("⏱ Timeout #%lu (normal - waiting for drone)\n", timeout_count);
    }
    
    Radio.Rx(0);
}

void OnRxError(void) {
    Serial.println("❌ RX Error - Restarting");
    Radio.Rx(0);
}