/*
 * Heltec Wireless Tracker – GNSS TTFF Acquisition Test
 * Serial-only | No OLED | No LoRa | Stops after first fix
 */

#include <Arduino.h>
#include "HT_TinyGPS++.h"

// ---------- POWER & UART PINS (CONFIRMED) ----------
#define VEXT_CTRL   21     // Peripheral power rail
#define VGNSS_CTRL  3      // GNSS power enable
#define GNSS_RX_PIN 33
#define GNSS_TX_PIN 34

TinyGPSPlus GPS;

unsigned long startMillis;
bool fixRecorded = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---------- POWER SEQUENCE (CRITICAL) ----------
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, HIGH);

  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);

  delay(200);  // allow GNSS RF to stabilize

  // ---------- GNSS UART ----------
  Serial1.begin(115200, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

  Serial.println("\n==============================");
  Serial.println(" GNSS TTFF ACQUISITION TEST ");
  Serial.println("==============================");
  Serial.println("GNSS Powered ON");
  Serial.println("Waiting for GPS fix...\n");

  startMillis = millis();
}

void loop() {

  // ---------- READ GNSS DATA ----------
  while (Serial1.available()) {
    GPS.encode(Serial1.read());
  }

  // ---------- LIVE STATUS PRINT (2 Hz) ----------
  static unsigned long lastPrint = 0;
  if (!fixRecorded && millis() - lastPrint > 500) {
    lastPrint = millis();

    Serial.print("Sats: ");
    Serial.print(GPS.satellites.value());

    Serial.print(" | Fix: ");
    Serial.print(GPS.location.isValid() ? "YES" : "NO");

    Serial.print(" | Lat: ");
    Serial.print(GPS.location.lat(), 6);

    Serial.print(" | Lon: ");
    Serial.println(GPS.location.lng(), 6);
  }

  // ---------- FIRST FIX DETECTION ----------
  if (!fixRecorded &&
      GPS.location.isValid() &&
      GPS.location.lat() != 0.0 &&
      GPS.location.lng() != 0.0 &&
      GPS.satellites.value() >= 4) {

    fixRecorded = true;
    float ttff = (millis() - startMillis) / 1000.0;

    Serial.println("\n******** GPS LOCK ACQUIRED ********");
    Serial.print("TTFF = ");
    Serial.print(ttff, 2);
    Serial.println(" seconds");
    Serial.println("**********************************\n");

    // ---------- STOP AFTER FIRST FIX ----------
    while (true) {
      delay(1000);
    }
  }
}
