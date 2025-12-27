/*
 * FalconResQ Drone Receiver - SIMPLE MVP VERSION
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * 
 * Simple Operation:
 * - Continuous RX on beacon channel (866.1 MHz)
 * - Immediate relay to ground station (866.9 MHz) when packet received
 * - Returns to RX mode automatically
 * - CRC validation
 * 
 * Mounted on Drone
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"

// ============ CONFIGURATION ============
#define RF_FREQUENCY_RX     866100000   // 866.1 MHz - Beacon channel
#define RF_FREQUENCY_TX     866900000   // 866.9 MHz - Ground station

#define TX_OUTPUT_POWER     17          // dBm
#define LORA_BANDWIDTH      0           // 125 kHz
#define LORA_SPREADING_FACTOR 7         // SF7
#define LORA_CODINGRATE     1           // 4/5
#define LORA_PREAMBLE_LENGTH 12         // Symbols
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_BUFFER_SIZE      150
#define TX_BUFFER_SIZE      200

// ============ GLOBAL VARIABLES ============
static RadioEvents_t RadioEvents;
char rxpacket[RX_BUFFER_SIZE];
char txpacket[TX_BUFFER_SIZE];

int packets_received = 0;
int packets_relayed = 0;
int crc_failures = 0;

bool relay_mode = false;
bool lora_idle = true;

int16_t last_rssi = 0;
int8_t last_snr = 0;

// ============ FUNCTION DECLARATIONS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void OnTxDone(void);
void OnTxTimeout(void);
uint16_t calculate_crc16(const char* data, size_t length);
bool verify_packet_crc(const char* packet);
void relay_to_ground(const char* original, int16_t rssi, int8_t snr);

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║  FalconResQ Drone Receiver           ║");
    Serial.println("║                                      ║");
    Serial.println("╚═══════════════════════════════════════╝\n");
    
    // Initialize board
    Serial.println("Initializing board...");
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    Serial.println("✓ Board initialized");
    
    // Setup radio callbacks
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    // Initialize radio
    Serial.println("Initializing radio...");
    Radio.Init(&RadioEvents);
    Serial.println("✓ Radio initialized");
    
    // Set RX channel
    Serial.printf("Setting RX frequency: %.1f MHz\n", RF_FREQUENCY_RX / 1000000.0);
    Radio.SetChannel(RF_FREQUENCY_RX);
    Serial.println("✓ Channel set");
    
    // Configure RX - EXACT as working debug
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
    
    // Configure TX
    Radio.SetTxConfig(MODEM_LORA, 
                      TX_OUTPUT_POWER, 
                      0,
                      LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, 
                      LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, 
                      LORA_FIX_LENGTH_PAYLOAD_ON,
                      true,
                      0,
                      0,
                      LORA_IQ_INVERSION_ON, 
                      3000);
    Serial.println("✓ TX configured");
    
    Serial.println("\nConfiguration:");
    Serial.printf("  RX Frequency:   %.1f MHz\n", RF_FREQUENCY_RX / 1000000.0);
    Serial.printf("  TX Frequency:   %.1f MHz\n", RF_FREQUENCY_TX / 1000000.0);
    Serial.printf("  Bandwidth:      125 kHz\n");
    Serial.printf("  SF:             %d\n", LORA_SPREADING_FACTOR);
    Serial.printf("  CR:             4/%d\n", LORA_CODINGRATE + 4);
    Serial.printf("  Preamble:       %d symbols\n", LORA_PREAMBLE_LENGTH);
    Serial.println();
    
    // Start receiving
    Serial.println("Starting continuous RX mode...");
    Radio.Rx(0);
    Serial.println("✓ RX mode active - Listening for beacons");
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("Waiting for packets...");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

// ============ MAIN LOOP ============
void loop() {
    // Heartbeat
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 15000) {
        Serial.printf("[%lu s] Last Data-Packet: RX:%d | Relayed:%d | CRC Fail:%d\n", 
                      millis()/1000, packets_received, packets_relayed, crc_failures);
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
    const char* crc_field = strstr(packet, ",\"CRC\":\"");
    if (!crc_field) return false;
    
    const char* crc_val = crc_field + 8;
    char recv_crc_str[5];
    strncpy(recv_crc_str, crc_val, 4);
    recv_crc_str[4] = '\0';
    uint16_t recv_crc = (uint16_t)strtol(recv_crc_str, NULL, 16);
    
    size_t orig_len = crc_field - packet;
    char temp[RX_BUFFER_SIZE];
    strncpy(temp, packet, orig_len);
    temp[orig_len] = '}';
    temp[orig_len + 1] = '\0';
    
    uint16_t calc_crc = calculate_crc16(temp, strlen(temp));
    
    return (calc_crc == recv_crc);
}

// ============ RELAY FUNCTION ============
void relay_to_ground(const char* original, int16_t rssi, int8_t snr) {
    Serial.println("\n▶ Switching to TX mode for relay...");
    
    // Switch to TX channel
    Radio.Standby();
    delay(50);
    Radio.SetChannel(RF_FREQUENCY_TX);
    delay(50);
    
    // Build relay packet (add RSSI and SNR)
    strcpy(txpacket, original);
    char* closing = strrchr(txpacket, '}');
    if (closing) *closing = '\0';
    
    char addon[50];
    sprintf(addon, ",\"RSSI\":%d,\"SNR\":%d}", rssi, snr);
    strcat(txpacket, addon);
    
    Serial.println("┌────────────────────────────────────┐");
    Serial.println("│     RELAYING TO GROUND             │");
    Serial.println("└────────────────────────────────────┘");
    Serial.printf(" %s\n", txpacket);
    Serial.println("┌────────────────────────────────────┐");
    Serial.printf("│ Size: %d bytes                     │\n", strlen(txpacket));
    Serial.printf("│ Channel: %.1f MHz                 │\n", RF_FREQUENCY_TX / 1000000.0);
    Serial.println("└────────────────────────────────────┘");
    
    // Transmit
    lora_idle = false;
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    
    // Wait for TX
    unsigned long start = millis();
    while (!lora_idle && (millis() - start < 3000)) {
        Radio.IrqProcess();
        delay(10);
    }
    
    packets_relayed++;
    
    // Return to RX
    Serial.println("\n▶ Returning to RX mode...");
    Radio.Standby();
    delay(50);
    Radio.SetChannel(RF_FREQUENCY_RX);
    delay(50);
    Radio.Rx(0);
    Serial.println("✓ Back in RX mode\n");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

// ============ CALLBACKS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memset(rxpacket, 0, sizeof(rxpacket));
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    
    packets_received++;
    last_rssi = rssi;
    last_snr = snr;
    
    Serial.println("\n┌─────────────────────────────────────┐");
    Serial.printf(" ✓  PACKET RECEIVED [#%d]            \n", packets_received);
    Serial.println("└─────────────────────────────────────┘");
    Serial.printf(" %s\n", rxpacket);
    Serial.println("┌─────────────────────────────────────┐");
    Serial.printf("│ RSSI: %d dBm  │  SNR: %d dB        │\n", rssi, snr);
    Serial.printf("│ Size: %d bytes                      │\n", size);
    Serial.println("└─────────────────────────────────────┘");
    
    // Verify CRC
    if (verify_packet_crc(rxpacket)) {
        Serial.println("  ✓ CRC Valid");
        
        // Relay to ground station
        relay_to_ground(rxpacket, rssi, snr);
        
    } else {
        crc_failures++;
        Serial.println("  ✗ CRC Failed - Discarding");
        Serial.println();
        Radio.Rx(0);
    }
}

void OnRxTimeout(void) {
    static unsigned long count = 0;
    count++;
    if (count % 50 == 0) {
        Serial.printf(" Timeout #%lu (normal)\n", count);
    }
    Radio.Rx(0);
}

void OnRxError(void) {
    Serial.println("❌ RX Error");
    Radio.Rx(0);
}

void OnTxDone(void) {
    Serial.println("  ✓ Relay TX Success");
    lora_idle = true;
}

void OnTxTimeout(void) {
    Serial.println("  ✗ Relay TX Timeout");
    lora_idle = true;
}