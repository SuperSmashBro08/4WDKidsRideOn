#pragma once
#include <Arduino.h>

/*
 * OtaUpdater.h
 * ------------
 * Teensy in-app OTA loader (Serial2) compatible with your ESP32 uploader:
 *   HELLO <token>        -> READY
 *   BEGIN HEX            -> HEX BEGIN
 *   L <:ii..IntelHex..>  -> OK <line#> | BAD <line#>
 *   END                  -> HEX OK <lines> | HEX ERR <stats>
 *                           APPLIED (after flash/reboot prep)
 *
 * Non-OTA lines on Serial2 can be consumed by the app via OtaUserHandleLine().
 */

namespace OtaUpdater {

  // Initialize OTA on the given UART (usually Serial2). Baud defaults to 115200.
  void begin(HardwareSerial& otaPort, uint32_t baud = 115200);

  // Provide a human-readable app identifier/version for VERSION replies.
  void setAppVersion(const char* name);

  // Call this frequently from loop(); it parses incoming lines and performs OTA.
  void tick();

  // True while an OTA session is active.
  bool inProgress();

  // Optional: string to identify the embedded loader (for VERSION requests).
  const char* loaderId();   // returns e.g. "FlasherX v2.4 (in-app)"
}

// -------- App hook (implemented in your sketch) --------
// Return true if you handled the line (so OTA won't respond with ERR).
extern "C" bool OtaUserHandleLine(const char* line);
