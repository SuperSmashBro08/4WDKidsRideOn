#pragma once
#include <Arduino.h>

// ===============================
// ESP32 AS5600 feed → Teensy
// ===============================
//
// Usage (in your .ino):
//   #include "Esp32AsFeed.h"
//
//   // Choose the UART connected to the ESP32 (example: Serial1)
//   #define ESP32_PORT Serial1
//
//   void setup(){
//     ESP32_PORT.begin(230400);         // already true in your project
//     Esp32AsFeed::begin(&ESP32_PORT);  // start parser
//   }
//
//   void loop(){
//     Esp32AsFeed::pump();  // call often (non-blocking)
//     // Use data:
//     if (Esp32AsFeed::isFresh()) {
//        int16_t deg_centi = Esp32AsFeed::degCenti();
//        uint16_t raw      = Esp32AsFeed::raw12();
//        // ... steering logic here ...
//     }
//   }
//
// Optionally print to your "S " console stream:
//   Esp32AsFeed::printS();  // prints: S AS raw=#### deg=###.## (only if fresh)
//
// Notes:
// - Freshness window defaults to 400ms (configurable).
// - Parser ignores non-ASCII and unrelated lines.
// - Does not interfere with your FlasherX protocol; it only consumes normal
//   runtime "AS ..." lines. Your uploader handshake (HELLO/BEGIN HEX) is unaffected.

namespace Esp32AsFeed {

// ===== Config =====
static constexpr uint16_t kLineMax      = 120;   // max incoming line length
static constexpr uint32_t kFreshMs      = 400;   // data freshness window
static constexpr uint32_t kMinPeriodMs  = 40;    // ignore spam faster than 25 Hz

// ===== State =====
static HardwareSerial* gPort = nullptr;

static char     gLine[kLineMax+1];
static uint16_t gPos = 0;

static uint16_t g_raw12 = 0;
static int16_t  g_deg_centi = INT16_MIN;   // 0.01 deg units
static uint32_t g_lastMs = 0;
static uint32_t g_lastAcceptMs = 0;

// ===== Helpers =====
static inline bool asciiPrintable(uint8_t b){
  return (b == '\r') || (b == '\n') || (b >= 32 && b < 127);
}

static void resetLine(){ gPos = 0; gLine[0] = 0; }

static void commitLine(){
  if (!gPos) return;
  gLine[gPos] = 0;

  // Expect: "AS raw=#### deg=###.##"
  // Be tolerant: allow either order; allow extra spaces.
  if (gLine[0]=='A' && gLine[1]=='S' && (gLine[2]==' ' || gLine[2]=='=' || gLine[2]==':')){
    // quick rate-limit accept
    uint32_t now = millis();
    if (now - g_lastAcceptMs < kMinPeriodMs) { resetLine(); return; }

    // Parse two numbers
    // We'll scan tokens separated by space.
    long raw = -1;
    float deg = NAN;

    const char* s = gLine + 2;
    while (*s==' ' || *s=='=' || *s==':') ++s;

    // simple token walk
    while (*s){
      // raw=####
      if (s[0]=='r' && s[1]=='a' && s[2]=='w'){
        const char* eq = strchr(s, '=');
        if (eq){ raw = strtol(eq+1, nullptr, 10); }
      }
      // deg=###.##
      if (s[0]=='d' && s[1]=='e' && s[2]=='g'){
        const char* eq = strchr(s, '=');
        if (eq){ deg = strtof(eq+1, nullptr); }
      }
      // advance to next space
      const char* sp = strchr(s, ' ');
      if (!sp) break;
      s = sp + 1;
    }

    if (raw >= 0 && raw <= 4095 && isfinite(deg)){
      g_raw12     = (uint16_t)raw;
      g_deg_centi = (int16_t) lroundf(deg * 100.0f);
      g_lastMs    = now;
      g_lastAcceptMs = now;
    }
  }

  resetLine();
}

// ===== API =====
static void begin(HardwareSerial* port){
  gPort = port;
  resetLine();
  g_raw12 = 0;
  g_deg_centi = INT16_MIN;
  g_lastMs = 0;
  g_lastAcceptMs = 0;
}

static void pump(){
  if (!gPort) return;
  while (gPort->available()){
    int r = gPort->read();
    if (r < 0) break;
    uint8_t b = (uint8_t)r;
    if (!asciiPrintable(b)) continue;

    char c = (char)b;
    if (c == '\r') continue;
    if (c == '\n'){
      commitLine();
      continue;
    }

    if (gPos < kLineMax){
      gLine[gPos++] = c;
    } else {
      // overflow → drop line
      resetLine();
    }
  }
}

static bool isFresh(uint32_t windowMs = kFreshMs){
  if (!g_lastMs) return false;
  return (millis() - g_lastMs) <= windowMs;
}

static uint16_t raw12(){ return g_raw12; }
static int16_t  degCenti(){ return g_deg_centi; }

// Optional: print mirrored S-line when fresh
static void printS(Stream& s = Serial){
  if (!isFresh()) return;
  // Convert centi-deg back to float with two decimals
  float deg = ((float)g_deg_centi) / 100.0f;
  s.print("S AS raw="); s.print(g_raw12);
  s.print(" deg="); s.print(deg, 2);
  s.print("\r\n");
}

} // namespace Esp32AsFeed
