/*
 * To be used only if the there are high number of packets/beacon devices which can cause
    loss of data packets, therfore we randomly receive the data packets at 4 different channels.
 * Receiver Scans each frequency channel at the rate of 200ms per channel
 * FalconResQ Drone Receiver Module
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * 
 * Features:
 * - Multi-channel RX scanning (4 beacon channels)
 * - Packet buffering and aggregation
 * - Periodic relay to ground station
 * - RSSI/SNR recording
 * 
 * Mounted on Drone - Acts as airborne collector & relay
 */

#include "Arduino.h"
#include "LoRaWan_APP.h"

// ============ CONFIGURATION ============
// Beacon channels (uplink - drone receives)
#define RF_FREQUENCY_CH0    866100000   // 866.1 MHz
#define RF_FREQUENCY_CH1    866300000   // 866.3 MHz
#define RF_FREQUENCY_CH2    866500000   // 866.5 MHz  
#define RF_FREQUENCY_CH3    866700000   // 866.7 MHz

// Ground station channel (downlink - drone transmits)
#define RF_FREQUENCY_GROUND 866900000   // 866.9 MHz

#define RX_OUTPUT_POWER     17          // dBm for relay
#define LORA_BANDWIDTH      0           // 125 kHz
#define LORA_SPREADING_FACTOR 7         // SF7
#define LORA_CODINGRATE     1           // 4/5
#define LORA_PREAMBLE_LENGTH 12         // Symbols
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// Timing Configuration
#define FRAME_LENGTH_MS     60000       // 60 seconds
#define UPLINK_WINDOW_MS    55000       // 55 seconds for RX
#define RELAY_WINDOW_MS     5000        // 5 seconds for TX relay
#define CHANNEL_SCAN_TIME_MS 200        // Time to listen on each channel

// Buffer Configuration
#define MAX_BUFFERED_PACKETS 50         // Maximum packets to buffer
#define RX_BUFFER_SIZE      100         // Individual packet buffer size
#define TX_BUFFER_SIZE      2000        // Aggregated packet buffer size

// ============ STRUCTURES ============
struct BeaconPacket {
    char data[RX_BUFFER_SIZE];
    int16_t rssi;
    int8_t snr;
    uint32_t timestamp;
    bool valid;
};

// ============ GLOBAL VARIABLES ============
static RadioEvents_t RadioEvents;

BeaconPacket packet_buffer[MAX_BUFFERED_PACKETS];
int buffer_index = 0;
bool lora_idle = true;

uint32_t current_channel = RF_FREQUENCY_CH0;
int current_channel_index = 0;
uint32_t channels[4] = {RF_FREQUENCY_CH0, RF_FREQUENCY_CH1, 
                        RF_FREQUENCY_CH2, RF_FREQUENCY_CH3};

unsigned long frame_start_time = 0;
bool in_relay_window = false;
bool relay_completed = false;

char rxpacket[RX_BUFFER_SIZE];
char aggregated_packet[TX_BUFFER_SIZE];

int16_t last_rssi = 0;
int8_t last_snr = 0;

// ============ FUNCTION DECLARATIONS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnTxDone(void);
void OnTxTimeout(void);
void LoRa_Init(void);
void switch_to_next_channel(void);
void buffer_packet(char* data, int16_t rssi, int8_t snr);
void relay_to_ground_station(void);
void build_aggregated_packet(char* buffer);
uint16_t calculate_crc16(const char* data, size_t length);
bool verify_packet_crc(const char* packet);
void enter_rx_mode(void);
void enter_relay_mode(void);

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== FalconResQ Drone Receiver ===");
    Serial.println("Role: Airborne Collector & Relay");
    Serial.println("================================\n");
    
    // Initialize buffer
    for (int i = 0; i < MAX_BUFFERED_PACKETS; i++) {
        packet_buffer[i].valid = false;
    }
    
    // Initialize LoRa
    LoRa_Init();
    
    // Start in RX mode
    frame_start_time = millis();
    enter_rx_mode();
    
    Serial.println("Drone Receiver Ready!");
    Serial.println("Scanning for beacon transmissions...\n");
}

// ============ MAIN LOOP ============
void loop() {
    unsigned long current_time = millis();
    unsigned long time_in_frame = current_time - frame_start_time;
    
    // Check if we need to start a new frame
    if (time_in_frame >= FRAME_LENGTH_MS) {
        frame_start_time = current_time;
        time_in_frame = 0;
        buffer_index = 0;  // Reset buffer for new frame
        relay_completed = false;
        Serial.println("\n--- New Frame Started ---\n");
        enter_rx_mode();
    }
    
    // ===== UPLINK WINDOW (RX Mode) =====
    if (time_in_frame < UPLINK_WINDOW_MS) {
        if (in_relay_window) {
            // Switch back to RX mode
            enter_rx_mode();
        }
        
        // Fast channel scanning
        static unsigned long last_channel_switch = 0;
        if (current_time - last_channel_switch > CHANNEL_SCAN_TIME_MS) {
            switch_to_next_channel();
            last_channel_switch = current_time;
        }
    }
    // ===== RELAY WINDOW (TX Mode) =====
    else if (time_in_frame >= UPLINK_WINDOW_MS && !relay_completed) {
        enter_relay_mode();
        relay_to_ground_station();
        relay_completed = true;
    }
    
    Radio.IrqProcess();
}

// ============ LoRa INITIALIZATION ============
void LoRa_Init(void) {
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init(&RadioEvents);
    Radio.SetChannel(current_channel);
    
    // Configure for RX initially
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    
    // Also configure TX for relay
    Radio.SetTxConfig(MODEM_LORA, RX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    
    Serial.println("LoRa Module Initialized");
    Serial.printf("RX Channels: 866.1, 866.3, 866.5, 866.7 MHz\n");
    Serial.printf("TX Channel: 866.9 MHz (Ground relay)\n");
    Serial.printf("SF: %d, BW: 125kHz, CR: 4/5\n\n", LORA_SPREADING_FACTOR);
}

// ============ ENTER RX MODE ============
void enter_rx_mode(void) {
    if (in_relay_window) {
        in_relay_window = false;
        Radio.SetChannel(current_channel);
        Radio.Rx(0);  // Continuous RX
        Serial.println(">>> Switched to RX Mode (Uplink Window)");
    }
}

// ============ ENTER RELAY MODE ============
void enter_relay_mode(void) {
    if (!in_relay_window) {
        in_relay_window = true;
        Radio.Standby();
        Radio.SetChannel(RF_FREQUENCY_GROUND);
        Serial.println("\n>>> Switched to TX Mode (Relay Window)");
    }
}

// ============ CHANNEL SWITCHING ============
void switch_to_next_channel(void) {
    current_channel_index = (current_channel_index + 1) % 4;
    current_channel = channels[current_channel_index];
    
    Radio.Standby();
    Radio.SetChannel(current_channel);
    Radio.Rx(0);  // Restart RX on new channel
    
    Serial.printf("Scanning CH%d: %.1f MHz\n", 
                  current_channel_index, current_channel / 1000000.0);
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

// ============ VERIFY PACKET CRC ============
bool verify_packet_crc(const char* packet) {
    // Find the CRC field in JSON
    const char* crc_start = strstr(packet, "\"CRC\":\"");
    if (crc_start == NULL) {
        Serial.println("  ⚠ No CRC field found");
        return false;
    }
    
    crc_start += 7;  // Move past "CRC":"
    char received_crc_str[5];
    strncpy(received_crc_str, crc_start, 4);
    received_crc_str[4] = '\0';
    uint16_t received_crc = (uint16_t)strtol(received_crc_str, NULL, 16);
    
    // Calculate CRC on packet excluding CRC field
    size_t packet_length = (crc_start - 7) - packet;  // Length before "CRC"
    char temp[RX_BUFFER_SIZE];
    strncpy(temp, packet, packet_length - 1);  // -1 to remove trailing comma
    temp[packet_length - 1] = '}';
    temp[packet_length] = '\0';
    
    uint16_t calculated_crc = calculate_crc16(temp, strlen(temp));
    
    if (calculated_crc == received_crc) {
        Serial.println("  ✓ CRC Valid");
        return true;
    } else {
        Serial.printf("  ✗ CRC Mismatch! Calc: %04X, Recv: %04X\n", 
                      calculated_crc, received_crc);
        return false;
    }
}

// ============ BUFFER PACKET ============
void buffer_packet(char* data, int16_t rssi, int8_t snr) {
    if (buffer_index < MAX_BUFFERED_PACKETS) {
        strcpy(packet_buffer[buffer_index].data, data);
        packet_buffer[buffer_index].rssi = rssi;
        packet_buffer[buffer_index].snr = snr;
        packet_buffer[buffer_index].timestamp = millis();
        packet_buffer[buffer_index].valid = true;
        
        Serial.printf("  → Buffered [%d/%d]\n", buffer_index + 1, MAX_BUFFERED_PACKETS);
        buffer_index++;
    } else {
        Serial.println("  ⚠ Buffer full, packet dropped!");
    }
}

// ============ BUILD AGGREGATED PACKET ============
void build_aggregated_packet(char* buffer) {
    strcpy(buffer, "[");
    
    for (int i = 0; i < buffer_index; i++) {
        if (packet_buffer[i].valid) {
            char temp[150];
            
            // Remove closing brace from original packet
            char* closing_brace = strrchr(packet_buffer[i].data, '}');
            if (closing_brace) *closing_brace = '\0';
            
            // Add RSSI and SNR to packet
            sprintf(temp, "%s,\"RSSI\":%d,\"SNR\":%d}", 
                    packet_buffer[i].data,
                    packet_buffer[i].rssi,
                    packet_buffer[i].snr);
            
            strcat(buffer, temp);
            
            if (i < buffer_index - 1) {
                strcat(buffer, ",");
            }
        }
    }
    
    strcat(buffer, "]");
}

// ============ RELAY TO GROUND STATION ============
void relay_to_ground_station(void) {
    if (buffer_index == 0) {
        Serial.println("No packets to relay.\n");
        return;
    }
    
    Serial.println("\n===== RELAYING TO GROUND =====");
    Serial.printf("Packets to relay: %d\n", buffer_index);
    
    build_aggregated_packet(aggregated_packet);
    
    Serial.println("Aggregated Packet:");
    Serial.println(aggregated_packet);
    Serial.printf("Total Size: %d bytes\n", strlen(aggregated_packet));
    Serial.printf("Channel: %.1f MHz\n", RF_FREQUENCY_GROUND / 1000000.0);
    Serial.println("------------------------------");
    
    Radio.Send((uint8_t *)aggregated_packet, strlen(aggregated_packet));
    lora_idle = false;
    
    // Wait for TX to complete
    while (!lora_idle) {
        Radio.IrqProcess();
        delay(10);
    }
    
    Serial.println("===== RELAY COMPLETE =====\n");
}

// ============ RADIO CALLBACKS ============
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memset(rxpacket, 0, sizeof(rxpacket));
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    
    Serial.println("\n----- Packet Received -----");
    Serial.println(rxpacket);
    Serial.printf("RSSI: %d dBm, SNR: %d dB\n", rssi, snr);
    Serial.printf("Size: %d bytes\n", size);
    
    // Verify CRC
    if (verify_packet_crc(rxpacket)) {
        buffer_packet(rxpacket, rssi, snr);
    } else {
        Serial.println("  ✗ Packet discarded due to CRC error");
    }
    
    Serial.println("---------------------------\n");
    
    // Continue receiving
    Radio.Rx(0);
}

void OnRxTimeout(void) {
    // Timeout is normal in continuous RX mode
    Radio.Rx(0);
}

void OnTxDone(void) {
    Serial.println("✓ Relay TX Success");
    lora_idle = true;
    Radio.Sleep();
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("✗ Relay TX Timeout");
    lora_idle = true;
}