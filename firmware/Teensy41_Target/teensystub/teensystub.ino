/*
 * Teensy 4.1 OTA target + interactive console
 * -------------------------------------------
 * Wiring: Serial2 RX2 (pin 7) ← ESP32 GPIO43, Serial2 TX2 (pin 8) → ESP32 GPIO44, share GND.
 * Telnet: connect through the ESP32 bridge with `telnet <esp32-ip> 2323`.
 * Quick test: type `VERSION`, then `STREAM ON` (or `POT ON`) and move the potentiometer on A0 to see
 *             `POT raw=... pct=...` lines; use `STREAM OFF`/`POT OFF` to silence the stream.
 * OTA: upload a Teensy `.hex` via the ESP32 web page. During HELLO/HEX streaming this console
 *      pauses and only the OTA replies (OK/BAD/HEX OK/APPLIED) are emitted.
 */

#include <Arduino.h>
#include <stdio.h>

// ------------------------- Config -------------------------
static constexpr uint8_t LED_PIN    = 13;
static constexpr uint32_t BAUD_USB  = 115200;   // Serial (USB) to PC
static constexpr uint32_t BAUD_UART = 115200;   // Serial2 (pins 7/8) to ESP32
static constexpr char OTA_TOKEN[]   = "8d81ab8762c545dabe699044766a0b72";
static constexpr char FW_VERSION[]  = "fw-raw-bridge-demo";
static constexpr uint8_t POT_PIN    = A0;

// ------------------------- IO helpers -------------------------
#define PC  Serial
#define NET Serial2

inline void logBoth(const String &s) {
  PC.println(s);
  NET.println(s);
}

inline void netln(const String &s) {
  NET.println(s);
}

// ------------------------- State -------------------------
static bool     in_hex_session = false;
static bool     echo_enabled   = false;
static bool     stream_enabled = false;
static uint32_t blink_ms       = 1000;
static uint32_t last_stream_ms = 0;
static uint32_t hex_lines = 0, hex_ok = 0, hex_bad = 0, hex_bytes = 0;
static bool     echo_resume_after_hex = false;
static bool     stream_resume_after_hex = false;
static bool     ota_suspend_armed = false;

// ------------------------- Utils -------------------------
static inline void fastBlink(int times = 6, int ms = 120) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(ms);
  }
}

static inline int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static inline int byteAt(const String& s, int i) {
  int hi = hexNibble(s.charAt(i));
  int lo = hexNibble(s.charAt(i + 1));
  if (hi < 0 || lo < 0) return -1;
  return (hi << 4) | lo;
}

// Intel-HEX checksum + basic shape verification (no flashing here).
static bool verifyIntelHexLine(const String& rec) {
  if (rec.length() < 11) return false;
  if (rec.charAt(0) != ':') return false;

  int len_i   = byteAt(rec, 1);  if (len_i < 0) return false;
  int addr_hi = byteAt(rec, 3);  if (addr_hi < 0) return false;
  int addr_lo = byteAt(rec, 5);  if (addr_lo < 0) return false;
  int rectype = byteAt(rec, 7);  if (rectype < 0) return false;

  const size_t data_start = 9u;
  const size_t data_end   = data_start + (size_t)len_i * 2u;
  if (rec.length() < data_end + 2u) return false;

  int sum = len_i + addr_hi + addr_lo + rectype;
  for (size_t i = data_start; i < data_end; i += 2) {
    int b = byteAt(rec, (int)i); if (b < 0) return false;
    sum += b;
  }
  sum = ((~sum + 1) & 0xFF);
  int chk = byteAt(rec, (int)data_end); if (chk < 0) return false;

  bool ok = (sum == chk);
  if (ok) hex_bytes += (uint32_t)len_i;
  return ok;
}

static void printStatus() {
  logBoth(String("FW ") + FW_VERSION);
  logBoth(String("BLINK ") + String(blink_ms) + "ms");
  logBoth(String("ECHO ") + (echo_enabled ? "ON" : "OFF"));
  logBoth(String("STREAM ") + (stream_enabled ? "ON" : "OFF"));
  logBoth(String("HEX session ") + (in_hex_session ? "ACTIVE" : "IDLE"));

  char buf[96];
  snprintf(buf, sizeof(buf), "HEX lines=%lu ok=%lu bad=%lu bytes=%lu", hex_lines, hex_ok, hex_bad, hex_bytes);
  logBoth(String(buf));

  snprintf(buf, sizeof(buf), "UPTIME %lu ms", millis());
  logBoth(String(buf));
}

static void printHelp() {
  netln("HELLO <token>");
  netln("BEGIN HEX  /  L <hex>  /  END");
  netln("PING | VERSION | STATUS");
  netln("ECHO ON|OFF");
  netln("STREAM ON|OFF (pot demo)");
}

// ------------------------- Setup -------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);

  Serial.begin(BAUD_USB);
  Serial2.begin(BAUD_UART);

  delay(300);
  Serial.println("\n[Teensy] OTA HEX receiver ready on Serial2 (7=RX,8=TX).");
  Serial.print  ("[Teensy] FW: "); Serial.println(FW_VERSION);
  Serial.println("[Teensy] Console commands: PING | VERSION | STATUS | ECHO ON|OFF | STREAM ON|OFF | HELP");
}

// ------------------------- Loop -------------------------
void loop() {
  // LED heartbeat (visual only)
  static uint32_t t_led = 0;
  if (millis() - t_led >= blink_ms) {
    t_led = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Optional potentiometer streaming (~25 Hz) only when requested and not during OTA
  if (stream_enabled && !in_hex_session) {
    uint32_t now = millis();
    if (now - last_stream_ms >= 40) {
      last_stream_ms = now;
      int raw = analogRead(POT_PIN);
      float pct = (raw / 4095.0f) * 100.0f;
      NET.print(F("POT raw="));
      NET.print(raw);
      NET.print(F(" pct="));
      NET.println(pct, 1);
    }
  }

  // -------- Command / HEX receiver on Serial2 --------
  static String line;
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    if (echo_enabled && !in_hex_session) NET.write((uint8_t)c);

    if (c == '\r') continue;
    if (c != '\n') {
      if (line.length() < 520) line += c;
      continue;
    }
    line.trim();
    if (!line.length()) { line = ""; continue; }

    String cmd = line;
    cmd.toUpperCase();

    // ================= OTA handshake =================
    if (cmd.startsWith("HELLO ")) {
      String tok = line.substring(6);
      if (tok == OTA_TOKEN) {
        Serial.println("[Teensy] HELLO OK");
        echo_resume_after_hex = echo_enabled;
        stream_resume_after_hex = stream_enabled;
        ota_suspend_armed = true;
        echo_enabled = false;
        stream_enabled = false;
        netln("READY");
        fastBlink();
        netln("DONE");
      } else {
        Serial.println("[Teensy] HELLO token mismatch");
        netln("NACK");
      }
    }
    else if (cmd == "BEGIN HEX") {
      if (in_hex_session) {
        netln("HEX BUSY");
      } else {
        if (!ota_suspend_armed) {
          echo_resume_after_hex = echo_enabled;
          stream_resume_after_hex = stream_enabled;
          ota_suspend_armed = true;
        }
        echo_enabled = false;
        stream_enabled = false;
        in_hex_session = true;
        hex_lines = hex_ok = hex_bad = hex_bytes = 0;
        Serial.println("[Teensy] HEX session begin");
        netln("HEX BEGIN");
      }
    }
    else if (line.startsWith("L ") && in_hex_session) {
      String rec = line.substring(2);
      hex_lines++;
      bool ok = verifyIntelHexLine(rec);
      if (ok) {
        hex_ok++;
        NET.print(F("OK "));
        NET.println(hex_lines);
      } else {
        hex_bad++;
        NET.print(F("BAD "));
        NET.println(hex_lines);
      }
    }
    else if (cmd == "END") {
      if (!in_hex_session) {
        netln("HEX IDLE");
      } else {
        in_hex_session = false;
        echo_enabled = echo_resume_after_hex;
        stream_enabled = stream_resume_after_hex;
        echo_resume_after_hex = false;
        stream_resume_after_hex = false;
        ota_suspend_armed = false;

        Serial.printf("[Teensy] HEX end: lines=%lu ok=%lu bad=%lu dataBytes=%lu\n",
                      hex_lines, hex_ok, hex_bad, hex_bytes);

        if (hex_bad == 0 && hex_ok > 0) {
          NET.printf("HEX OK lines=%lu bytes=%lu\n", hex_ok, hex_bytes);
          blink_ms = 87;
          netln("APPLIED");
          Serial.println("[Teensy] GOODBYE — OTA transfer verified ✔");
        } else {
          NET.printf("HEX ERR lines=%lu bad=%lu\n", hex_lines, hex_bad);
        }
      }
    }

    // ================= Utility commands =================
    else if (cmd == "PING") {
      netln("PONG");
    }
    else if (cmd == "VERSION") {
      netln(String("FW ") + FW_VERSION);
    }
    else if (cmd == "STATUS") {
      printStatus();
    }
    else if (cmd == "ECHO ON") {
      if (!in_hex_session) { echo_enabled = true; netln("OK"); }
      else { netln("BUSY"); }
    }
    else if (cmd == "ECHO OFF") {
      if (!in_hex_session) { echo_enabled = false; netln("OK"); }
      else { netln("BUSY"); }
    }
    else if (cmd == "STREAM ON" || cmd == "POT ON") {
      if (!in_hex_session) {
        stream_enabled = true;
        last_stream_ms = 0;
        netln("OK");
      } else {
        netln("BUSY");
      }
    }
    else if (cmd == "STREAM OFF" || cmd == "POT OFF") {
      stream_enabled = false;
      netln("OK");
    }
    else if (cmd == "HELP") {
      printHelp();
    }
    else {
      netln("ERR");
    }

    line = "";
  }
}
