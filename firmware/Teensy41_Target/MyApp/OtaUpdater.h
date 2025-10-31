#pragma once
#include <Arduino.h>

/*
 * OtaUpdater.h
 * ------------
 * In-app OTA loader wrapper for Teensy 4.1 that speaks the same protocol
 * as your FlasherX stub (HELLO / BEGIN HEX / L <line> / END).
 *
 * Usage (in your MyApp.ino):
 *   #include "OtaUpdater.h"
 *   #include "OtaConsole.h"
 *
 *   void setup() {
 *     Serial.begin(115200);
 *     Serial2.begin(115200);            // Teensy RX2/TX2 on pins 7/8
 *     OtaUpdater::begin(Serial2);       // arm OTA
 *     OtaConsole::begin(Serial2);       // optional logs to ESP32
 *   }
 *
 *   void loop() {
 *     OtaUpdater::tick();               // cheap when idle; handles OTA when active
 *     if (!OtaUpdater::inProgress()) {
 *       OLOG("heartbeat %lu", millis()/1000);
 *     }
 *   }
 */

namespace OtaUpdater {

  // Initialize OTA on the given UART (usually Serial2). Baud defaults to 115200.
  void begin(HardwareSerial& otaPort, uint32_t baud = 115200);

  // Call this frequently from loop(); it parses incoming lines and performs OTA.
  void tick();

  // True while an OTA session is active (useful to mute app chatter on Serial2).
  bool inProgress();

  // Optional: string to identify the embedded loader (for VERSION requests).
  const char* loaderId();   // returns e.g. "FlasherX v2.4 (in-app)"
}
