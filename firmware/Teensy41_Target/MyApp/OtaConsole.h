#pragma once
#include <Arduino.h>

/*
 * OtaConsole.h
 * ------------
 * Tiny helper to stream your app logs to the ESP32 over the same UART
 * used for OTA, *without* interfering with the OTA line protocol.
 *
 * Rules it follows:
 * - Every log line is prefixed with "S " so the ESP32 can ignore it
 *   (your uploader only reacts to READY/OK/BAD/HEX... etc.).
 * - Adds "\r\n" line endings to keep things consistent with your ESP32 code.
 * - You can enable/disable at runtime.
 *
 * Usage (in MyApp.ino):
 *   #include "OtaConsole.h"
 *   ...
 *   void setup() {
 *     Serial2.begin(115200);
 *     OtaConsole::begin(Serial2);
 *     OLOG("MyApp started");
 *   }
 *   void loop() {
 *     if (!otaBusy) OLOG("heartbeat %lu", millis()/1000);
 *   }
 */

namespace OtaConsole {
  // Where to write (default: Serial2). Use begin() to change.
  static Stream* s = &Serial2;

  // On/off switch (quick mute during OTA if you want)
  static bool enabled = true;

  // Initialize the console output target
  inline void begin(Stream& out = Serial2) {
    s = &out;
    enabled = true;
  }

  inline void setEnabled(bool on) { enabled = on; }

  // Print a full line with automatic "S " prefix and CRLF
  inline void println(const char* msg) {
    if (!enabled || !s) return;
    s->print("S ");
    s->print(msg);
    s->print("\r\n");
  }

  // Print formatted line (like printf) with prefix and CRLF
  inline void printf(const char* fmt, ...) {
    if (!enabled || !s) return;
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    s->print("S ");
    s->print(buf);
    s->print("\r\n");
  }

  // Print partial text (no CRLF); still prefixed once
  inline void print(const char* msg) {
    if (!enabled || !s) return;
    s->print("S ");
    s->print(msg);
  }

  // Optional: mirror any USB serial input to ESP32 console as lines.
  // Handy when you type into the Teensy's USB serial and want it visible on /console.
  inline void mirrorUsbStreamOnce() {
    if (!enabled || !s) return;
    static char line[192];
    static size_t n = 0;

    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line[n] = 0;
        if (n) {
          s->print("S ");
          s->print(line);
          s->print("\r\n");
          n = 0;
        }
      } else if (n + 1 < sizeof(line)) {
        line[n++] = c;
      }
    }
  }
}

// Shorthand macros
#define OLOG(...)  OtaConsole::printf(__VA_ARGS__)
#define OLINE(msg) OtaConsole::println(msg)
