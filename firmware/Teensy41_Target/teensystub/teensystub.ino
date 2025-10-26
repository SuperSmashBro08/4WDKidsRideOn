/*
 * Teensy 4.1 OTA receiver stub (simplified)
 * -----------------------------------------
 * Wiring: Serial2 RX2 (pin 7) <- ESP32 GPIO43, Serial2 TX2 (pin 8) -> ESP32 GPIO44, share GND.
 * Use Arduino Serial Monitor on USB to check logs and firmware version.
 * OTA protocol: HELLO <token> / BEGIN HEX / L <record> / END.
 */

#include <Arduino.h>

static constexpr uint8_t  LED_PIN    = 13;
static constexpr uint32_t BAUD_USB  = 115200;   // Serial (USB) to PC
static constexpr uint32_t BAUD_UART = 115200;   // Serial2 (pins 7/8) to ESP32
static constexpr char     OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";
static constexpr char     FW_VERSION[] = "fw-simple-ota-v2";

static bool handshake_ready = false;
static bool hex_in_progress = false;
static uint32_t hex_lines = 0;
static uint32_t hex_ok = 0;
static uint32_t hex_bad = 0;
static uint32_t hex_bytes = 0;

static inline void resetHexCounters() {
  hex_lines = hex_ok = hex_bad = hex_bytes = 0;
}

static int nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int byteFromHex(const String& rec, size_t index) {
  if (index + 1 >= rec.length()) return -1;
  int hi = nibble(rec.charAt(index));
  int lo = nibble(rec.charAt(index + 1));
  if (hi < 0 || lo < 0) return -1;
  return (hi << 4) | lo;
}

static int verifyIntelHex(const String& rec) {
  if (rec.length() < 11) return -1;
  if (rec.charAt(0) != ':') return -1;

  int len     = byteFromHex(rec, 1);
  int addr_hi = byteFromHex(rec, 3);
  int addr_lo = byteFromHex(rec, 5);
  int rectype = byteFromHex(rec, 7);
  if (len < 0 || addr_hi < 0 || addr_lo < 0 || rectype < 0) return -1;

  size_t data_start = 9;
  size_t data_end = data_start + (size_t)len * 2;
  if (rec.length() < data_end + 2) return -1;

  int sum = len + addr_hi + addr_lo + rectype;
  for (size_t i = data_start; i < data_end; i += 2) {
    int b = byteFromHex(rec, i);
    if (b < 0) return -1;
    sum += b;
  }
  sum = ((~sum + 1) & 0xFF);
  int chk = byteFromHex(rec, data_end);
  if (chk < 0) return -1;

  if (sum != chk) return -1;
  return len;
}

static void handleUsbConsole() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (line.length() < 80) line += c;
      continue;
    }
    line.trim();
    if (!line.length()) { line = ""; continue; }

    String cmd = line;
    cmd.toLowerCase();
    if (cmd == "version") {
      Serial.print("FW version: "); Serial.println(FW_VERSION);
    } else if (cmd == "status") {
      Serial.printf("HEX lines=%lu ok=%lu bad=%lu bytes=%lu\n", hex_lines, hex_ok, hex_bad, hex_bytes);
    } else if (cmd == "help") {
      Serial.println("Commands: version, status, help");
    } else {
      Serial.println("Unknown command. Try: version");
    }
    line = "";
  }
}

static void handleSerial2Line(const String& line) {
  String cmd = line;
  cmd.toUpperCase();

  if (cmd.startsWith("HELLO")) {
    if (hex_in_progress) {
      Serial2.println("BUSY");
      return;
    }
    int spacePos = line.indexOf(' ');
    String token = (spacePos >= 0) ? line.substring(spacePos + 1) : String();
    token.trim();
    if (token == OTA_TOKEN) {
      handshake_ready = true;
      Serial.println("[Teensy] HELLO accepted");
      Serial2.println("READY");
    } else {
      Serial.println("[Teensy] HELLO rejected (token mismatch)");
      Serial2.println("NACK");
    }
  }
  else if (cmd == "BEGIN HEX") {
    if (!handshake_ready || hex_in_progress) {
      Serial2.println("HEX IDLE");
      return;
    }
    resetHexCounters();
    hex_in_progress = true;
    Serial.println("[Teensy] HEX session begin");
    Serial2.println("HEX BEGIN");
  }
  else if (cmd.startsWith("L ") && hex_in_progress) {
    String rec = line.substring(2);
    int len = verifyIntelHex(rec);
    hex_lines++;
    if (len >= 0) {
      hex_ok++;
      hex_bytes += (uint32_t)len;
      Serial2.print("OK ");
      Serial2.println(hex_lines);
    } else {
      hex_bad++;
      Serial2.print("BAD ");
      Serial2.println(hex_lines);
    }
  }
  else if (cmd == "END") {
    if (!hex_in_progress) {
      Serial2.println("HEX IDLE");
      return;
    }
    hex_in_progress = false;
    handshake_ready = false;

    if (hex_bad == 0 && hex_ok > 0) {
      Serial2.printf("HEX OK lines=%lu bytes=%lu\n", hex_ok, hex_bytes);
      Serial2.println("APPLIED");
      Serial.println("[Teensy] OTA verified (checksum only)");
    } else {
      Serial2.printf("HEX ERR lines=%lu bad=%lu\n", hex_lines, hex_bad);
      Serial.println("[Teensy] OTA errors detected");
    }
  }
  else if (cmd == "PING") {
    Serial2.println("PONG");
  }
  else if (cmd == "VERSION") {
    Serial2.print("FW ");
    Serial2.println(FW_VERSION);
  }
  else {
    Serial2.println("ERR");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(BAUD_USB);
  Serial2.begin(BAUD_UART);

  delay(200);
  Serial.println("\n[Teensy] OTA HEX receiver ready on Serial2 (pins 7/8)");
  Serial.print("[Teensy] Firmware version: ");
  Serial.println(FW_VERSION);
  Serial.println("[Teensy] Type 'version' or 'status' in Serial Monitor for info.");
}

void loop() {
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  handleUsbConsole();

  static String line;
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (line.length() < 520) line += c;
      continue;
    }
    line.trim();
    if (!line.length()) { line = ""; continue; }
    handleSerial2Line(line);
    line = "";
  }
}
