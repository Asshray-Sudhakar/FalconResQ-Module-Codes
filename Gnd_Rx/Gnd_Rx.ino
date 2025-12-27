/*
 * FalconResQ Ground Station Receiver - FINAL VERSION
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * 
 * Operation:
 * - Receives packets from drone on 866.9 MHz
 * - Validates CRC
 * - Forwards to PC via Serial (USB) to Streamlit web app
 * - Clean JSON format for web interface
 * 
 * Connected to PC via USB (COM20)
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"

// ============ CONFIGURATION ============
#define RF_FREQUENCY_RX     866900000   // 866.9 MHz - Drone relay channel
#define TX_OUTPUT_POWER     17          // dBm (not used, RX only)
#define LORA_BANDWIDTH      0           // 125 kHz
#define LORA_SPREADING_FACTOR 7         // SF7
#define LORA_CODINGRATE     1           // 4/5
#define LORA_PREAMBLE_LENGTH 12         // Symbols
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_BUFFER_SIZE      200

// ============ GLOBAL VARIABLES ============
static RadioEvents_t RadioEvents;
char rxpacket[RX_BUFFER_SIZE];

int packets_received = 0;
int crc_failures = 0;
int packets_forwarded = 0;

// ============ FUNCTION DECLARATIONS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
uint16_t calculate_crc16(const char* data, size_t length);
bool verify_packet_crc(const char* packet);
void forward_to_web_app(const char* packet);
void extract_and_send_json(const char* packet);

// ============ SETUP ============
void setup() {
    // Initialize Serial (USB) at 115200 for web app communication
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║  FalconResQ Ground Station              ║");
    Serial.println("║  Receiver Module                        ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    // Initialize board
    Serial.println("Initializing hardware...");
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
    
    // Set RX channel (drone relay frequency)
    Serial.printf("Setting RX frequency: %.1f MHz\n", RF_FREQUENCY_RX / 1000000.0);
    Radio.SetChannel(RF_FREQUENCY_RX);
    Serial.println("✓ Channel set");
    
    // Configure RX parameters
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
    
    // Display configuration
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("Configuration:");
    Serial.printf("  RX Frequency:   %.1f MHz\n", RF_FREQUENCY_RX / 1000000.0);
    Serial.printf("  Bandwidth:      125 kHz\n");
    Serial.printf("  SF:             %d\n", LORA_SPREADING_FACTOR);
    Serial.printf("  CR:             4/%d\n", LORA_CODINGRATE + 4);
    Serial.printf("  Preamble:       %d symbols\n", LORA_PREAMBLE_LENGTH);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    // Start receiving
    Serial.println("Starting continuous RX mode...");
    Radio.Rx(0);
    Serial.println("✓ Ground Station ACTIVE");
    Serial.println("✓ Listening for drone relays...");
    Serial.println("✓ Serial port ready for web app\n");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("System Ready - Waiting for packets...");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

// ============ MAIN LOOP ============
void loop() {
    // Status heartbeat every 30 seconds
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 30000) {
        Serial.printf("\n[STATUS] Uptime: %lu s | RX: %d | Forwarded: %d | CRC Fail: %d\n\n", 
                      millis()/1000, packets_received, packets_forwarded, crc_failures);
        last_heartbeat = millis();
    }
    
    Radio.IrqProcess();
    delay(10);
}

// ============ CRC FUNCTIONS ============
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

bool verify_packet_crc(const char* packet) {
    // Find CRC field
    const char* crc_field = strstr(packet, ",\"CRC\":\"");
    if (!crc_field) return false;
    
    // Extract received CRC
    const char* crc_val = crc_field + 8;
    char recv_crc_str[5];
    strncpy(recv_crc_str, crc_val, 4);
    recv_crc_str[4] = '\0';
    uint16_t recv_crc = (uint16_t)strtol(recv_crc_str, NULL, 16);
    
    // Reconstruct original packet (without CRC field)
    size_t orig_len = crc_field - packet;
    char temp[RX_BUFFER_SIZE];
    strncpy(temp, packet, orig_len);
    temp[orig_len] = '}';
    temp[orig_len + 1] = '\0';
    
    // Calculate CRC
    uint16_t calc_crc = calculate_crc16(temp, strlen(temp));
    
    return (calc_crc == recv_crc);
}

// ============ EXTRACT AND SEND JSON ============
void extract_and_send_json(const char* packet) {
    // Extract clean JSON for web app
    // Input format: {"ID":1,"LAT":13.022,"LON":77.587,"TIME":"15:42","CRC":"XXXX","RSSI":-65,"SNR":8}
    // Output format: {"ID":1,"LAT":13.022,"LON":77.587,"TIME":"15:42","RSSI":-65}
    
    char clean_json[150];
    memset(clean_json, 0, sizeof(clean_json));
    
    // Parse the packet
    int id = 0;
    float lat = 0.0, lon = 0.0;
    char time_str[10] = "00:00";
    int rssi = -999;
    
    // Extract values using sscanf
    sscanf(packet, "{\"ID\":%d,\"LAT\":%f,\"LON\":%f,\"TIME\":\"%[^\"]\"", 
           &id, &lat, &lon, time_str);
    
    // Extract RSSI
    const char* rssi_field = strstr(packet, "\"RSSI\":");
    if (rssi_field) {
        sscanf(rssi_field, "\"RSSI\":%d", &rssi);
    }
    
    // Build clean JSON packet for web app
    sprintf(clean_json, "{\"ID\":%d,\"LAT\":%.6f,\"LON\":%.6f,\"TIME\":\"%s\",\"RSSI\":%d}",
            id, lat, lon, time_str, rssi);
    
    // Send to web app via Serial
    Serial.println(clean_json);
    
    packets_forwarded++;
}

// ============ CALLBACK: RX DONE ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Clear buffer and copy payload
    memset(rxpacket, 0, sizeof(rxpacket));
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    
    packets_received++;
    
    // Display received packet (for debugging)
    Serial.println("\n┌─────────────────────────────────────┐");
    Serial.printf(" PACKET RECEIVED [#%d]               \n", packets_received);
    Serial.println("└─────────────────────────────────────┘");
    Serial.printf(" Raw: %s\n", rxpacket);
    Serial.println("┌─────────────────────────────────────┐");
    Serial.printf("│ Ground RSSI: %d dBm                │\n", rssi);
    Serial.printf("│ Ground SNR:  %d dB                  │\n", snr);
    Serial.printf("│ Size: %d bytes                      │\n", size);
    Serial.println("└─────────────────────────────────────┘");
    
    // Verify CRC
    if (verify_packet_crc(rxpacket)) {
        Serial.println("  ✓ CRC Valid");
        
        // Extract and send clean JSON to web app
        Serial.println("\n▶ Forwarding to Web App:");
        extract_and_send_json(rxpacket);
        Serial.println("  ✓ Sent to Serial (Web App)\n");
        
    } else {
        crc_failures++;
        Serial.println("  ✗ CRC Failed - Packet Discarded\n");
    }
    
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    // Continue receiving
    Radio.Rx(0);
}

// ============ CALLBACK: RX TIMEOUT ============
void OnRxTimeout(void) {
    // Silent timeout - normal operation
    Radio.Rx(0);
}

// ============ CALLBACK: RX ERROR ============
void OnRxError(void) {
    Serial.println("❌ RX Error - Restarting RX");
    Radio.Rx(0);
}