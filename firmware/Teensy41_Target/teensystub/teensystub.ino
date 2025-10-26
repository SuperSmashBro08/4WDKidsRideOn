/*
 * Teensy 4.1 OTA target + interactive console
 * -------------------------------------------
 * Wiring: Serial2 RX2 (pin 7) ← ESP32 GPIO43, Serial2 TX2 (pin 8) → ESP32 GPIO44, share GND.
 * Telnet: connect through the ESP32 bridge with `telnet <esp32-ip> 2323`.
 * Quick test: type `VERSION`, then `POT ON` and move the potentiometer on A0 to see `POT:` lines;
 *             use `POT OFF` to silence the stream.
 * OTA: upload a Teensy `.hex` via the ESP32 web page. During HELLO/HEX streaming this console
 *      pauses and only the OTA replies (OK/BAD/HEX OK/APPLIED) are emitted.
 */

#include <Arduino.h>

// ------------------------- Config -------------------------
static constexpr uint8_t LED_PIN    = 13;
static constexpr uint32_t BAUD_USB  = 115200;   // Serial (USB) to PC
static constexpr uint32_t BAUD_UART = 115200;   // Serial2 (pins 7/8) to ESP32
static constexpr char OTA_TOKEN[]   = "8d81ab8762c545dabe699044766a0b72";
static constexpr char FW_VERSION[]  = "fw-raw-bridge-demo";
static constexpr uint8_t POT_PIN    = A0;

// ------------------------- State -------------------------
static bool     in_hex_session = false;
static bool     echo_enabled   = true;
static bool     pot_streaming  = false;
static uint32_t blink_ms       = 1000;
static uint32_t last_pot_ms    = 0;
static uint32_t hex_lines = 0, hex_ok = 0, hex_bad = 0, hex_bytes = 0;

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
  Serial2.print("FW "); Serial2.println(FW_VERSION);
  Serial2.print("BLINK "); Serial2.print(blink_ms); Serial2.println("ms");
  Serial2.print("ECHO "); Serial2.println(echo_enabled ? "ON" : "OFF");
  Serial2.print("POT "); Serial2.println(pot_streaming ? "ON" : "OFF");
  Serial2.print("HEX session "); Serial2.println(in_hex_session ? "ACTIVE" : "IDLE");
  Serial2.printf("HEX lines=%lu ok=%lu bad=%lu bytes=%lu\n", hex_lines, hex_ok, hex_bad, hex_bytes);
  Serial2.printf("UPTIME %lu ms\n", millis());
}

static void printHelp() {
  Serial2.println("HELLO <token>");
  Serial2.println("BEGIN HEX  /  L <hex>  /  END");
  Serial2.println("PING | VERSION | STATUS");
  Serial2.println("ECHO ON|OFF");
  Serial2.println("POT ON|OFF");
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
  Serial.println("[Teensy] Console commands: PING | VERSION | STATUS | ECHO ON|OFF | POT ON|OFF | HELP");
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
  if (pot_streaming && !in_hex_session) {
    uint32_t now = millis();
    if (now - last_pot_ms >= 40) {
      last_pot_ms = now;
      int raw = analogRead(POT_PIN);
      float pct = (raw / 4095.0f) * 100.0f;
      Serial2.print("POT: ");
      Serial2.print(raw);
      Serial2.print(' ');
      Serial2.println(pct, 1);
    }
  }

  // -------- Command / HEX receiver on Serial2 --------
  static String line;
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    if (echo_enabled && !in_hex_session) Serial2.write((uint8_t)c);

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
        Serial2.println("READY");
        fastBlink();
        Serial2.println("DONE");
      } else {
        Serial.println("[Teensy] HELLO token mismatch");
        Serial2.println("NACK");
      }
    }
    else if (cmd == "BEGIN HEX") {
      if (in_hex_session) {
        Serial2.println("HEX BUSY");
      } else {
        in_hex_session = true;
        echo_enabled = false;
        pot_streaming = false;
        hex_lines = hex_ok = hex_bad = hex_bytes = 0;
        Serial.println("[Teensy] HEX session begin");
        Serial2.println("HEX BEGIN");
      }
    }
    else if (line.startsWith("L ") && in_hex_session) {
      String rec = line.substring(2);
      hex_lines++;
      bool ok = verifyIntelHexLine(rec);
      if (ok) {
        hex_ok++;
        Serial2.print("OK "); Serial2.println(hex_lines);
      } else {
        hex_bad++;
        Serial2.print("BAD "); Serial2.println(hex_lines);
      }
    }
    else if (cmd == "END") {
      if (!in_hex_session) {
        Serial2.println("HEX IDLE");
      } else {
        in_hex_session = false;
        echo_enabled = true;

        Serial.printf("[Teensy] HEX end: lines=%lu ok=%lu bad=%lu dataBytes=%lu\n",
                      hex_lines, hex_ok, hex_bad, hex_bytes);

        if (hex_bad == 0 && hex_ok > 0) {
          Serial2.printf("HEX OK lines=%lu bytes=%lu\n", hex_ok, hex_bytes);
          blink_ms = 87;
          Serial2.println("APPLIED");
          Serial.println("[Teensy] GOODBYE — OTA transfer verified ✔");
        } else {
          Serial2.printf("HEX ERR lines=%lu bad=%lu\n", hex_lines, hex_bad);
        }
      }
    }

    // ================= Utility commands =================
    else if (cmd == "PING") {
      Serial2.println("PONG");
    }
    else if (cmd == "VERSION") {
      Serial2.print("FW "); Serial2.println(FW_VERSION);
    }
    else if (cmd == "STATUS") {
      printStatus();
    }
    else if (cmd == "ECHO ON") {
      if (!in_hex_session) { echo_enabled = true; Serial2.println("OK"); }
      else { Serial2.println("BUSY"); }
    }
    else if (cmd == "ECHO OFF") {
      if (!in_hex_session) { echo_enabled = false; Serial2.println("OK"); }
      else { Serial2.println("BUSY"); }
    }
    else if (cmd == "POT ON") {
      if (!in_hex_session) {
        pot_streaming = true;
        last_pot_ms = 0;
        Serial2.println("OK");
      } else {
        Serial2.println("BUSY");
      }
    }
    else if (cmd == "POT OFF") {
      pot_streaming = false;
      Serial2.println("OK");
    }
    else if (cmd == "HELP") {
      printHelp();
    }
    else {
      Serial2.println("ERR");
    }

    line = "";
  }
}
