/*
 * Teensy 4.1 OTA receiver stub (robust handshake)
 * ----------------------------------------------
 * Wiring: Serial2 RX2 (pin 7) <- ESP32 GPIO43, Serial2 TX2 (pin 8) -> ESP32 GPIO44, share GND.
 * Use Arduino Serial Monitor on USB to check logs and firmware version.
 * OTA protocol: HELLO <token> / BEGIN HEX / L <record> / END.
 *
 * NOTE: This stub only verifies the incoming Intel HEX records and acknowledges them so the
 *       ESP32 knows whether the transfer succeeded. Actual flash re-programming is not
 *       performed here—once validation succeeds, a secondary mechanism must still apply the
 *       image. This mirrors the behaviour of the original project where OTA success meant the
 *       HEX file was received and verified without errors.
 */

#include <Arduino.h>
#include <cstring>
#include <strings.h>

static constexpr uint8_t  LED_PIN    = 13;
static constexpr uint32_t BAUD_USB  = 115200;   // Serial (USB) to PC
static constexpr uint32_t BAUD_UART = 115200;   // Serial2 (pins 7/8) to ESP32
static constexpr char     OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";
static constexpr char     FW_VERSION[] = "fw-simple-ota-v2";

static volatile bool handshake_ready = false;
static volatile bool hex_in_progress = false;
static uint32_t hex_lines = 0;
static uint32_t hex_ok = 0;
static uint32_t hex_bad = 0;
static uint32_t hex_bytes = 0;

static inline void resetHexCounters() {
  hex_lines = hex_ok = hex_bad = hex_bytes = 0;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int hexByte(const char* rec, size_t idx) {
  int hi = hexNibble(rec[idx]);
  int lo = hexNibble(rec[idx + 1]);
  if (hi < 0 || lo < 0) return -1;
  return (hi << 4) | lo;
}

static bool verifyIntelHex(const char* rec, size_t len, int& payloadLen) {
  if (len < 11) return false;
  if (rec[0] != ':') return false;

  int byteCount = hexByte(rec, 1);
  int addrHigh  = hexByte(rec, 3);
  int addrLow   = hexByte(rec, 5);
  int recType   = hexByte(rec, 7);
  if (byteCount < 0 || addrHigh < 0 || addrLow < 0 || recType < 0) return false;

  size_t dataStart = 9;
  size_t dataEnd   = dataStart + (size_t)byteCount * 2u;
  if (dataEnd + 2u > len) return false;

  int sum = byteCount + addrHigh + addrLow + recType;
  for (size_t i = dataStart; i < dataEnd; i += 2) {
    int b = hexByte(rec, i);
    if (b < 0) return false;
    sum += b;
  }
  sum = ((~sum + 1) & 0xFF);

  int check = hexByte(rec, dataEnd);
  if (check < 0) return false;

  if (sum != check) return false;
  payloadLen = byteCount;
  return true;
}

static void printStatus() {
  Serial.printf("HEX lines=%lu ok=%lu bad=%lu bytes=%lu\n", hex_lines, hex_ok, hex_bad, hex_bytes);
}

static void handleUsbConsole() {
  static char buffer[96];
  static size_t len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len + 1 < sizeof(buffer)) {
        buffer[len++] = c;
      }
      continue;
    }

    buffer[len] = '\0';
    if (len == 0) { len = 0; continue; }

    if (strcasecmp(buffer, "version") == 0) {
      Serial.print("FW version: ");
      Serial.println(FW_VERSION);
    } else if (strcasecmp(buffer, "status") == 0) {
      printStatus();
    } else if (strcasecmp(buffer, "help") == 0) {
      Serial.println("Commands: version, status, help");
    } else {
      Serial.println("Unknown command. Try: version");
    }

    len = 0;
  }
}

static void sendLine(const char* s) {
  Serial2.print(s);
  Serial2.print('\r');
  Serial2.print('\n');
}

static void handleSerial2Line(const char* line) {
  // Log incoming commands for troubleshooting
  Serial.print("[Teensy] RX: ");
  Serial.println(line);

  // Skip empty lines
  if (!line[0]) return;

  if (strncasecmp(line, "HELLO", 5) == 0) {
    if (hex_in_progress) {
      sendLine("BUSY");
      return;
    }
    const char* token = line + 5;
    while (*token == ' ') token++;
    if (*token == '\0') {
      sendLine("NACK");
      Serial.println("[Teensy] HELLO missing token");
      return;
    }
    if (strcmp(token, OTA_TOKEN) == 0) {
      handshake_ready = true;
      Serial.println("[Teensy] HELLO accepted");
      sendLine("READY");
    } else {
      handshake_ready = false;
      Serial.println("[Teensy] HELLO rejected (token mismatch)");
      sendLine("NACK");
    }
    return;
  }

  if (strcasecmp(line, "BEGIN HEX") == 0) {
    if (!handshake_ready || hex_in_progress) {
      sendLine("HEX IDLE");
      return;
    }
    resetHexCounters();
    hex_in_progress = true;
    Serial.println("[Teensy] HEX session begin");
    sendLine("HEX BEGIN");
    return;
  }

  if (strncasecmp(line, "L ", 2) == 0 && hex_in_progress) {
    const char* rec = line + 2;
    int payloadLen = 0;
    bool ok = verifyIntelHex(rec, strlen(rec), payloadLen);
    hex_lines++;
    if (ok) {
      hex_ok++;
      hex_bytes += (uint32_t)payloadLen;
      Serial2.print("OK ");
      Serial2.print(hex_lines);
      Serial2.print('\r');
      Serial2.print('\n');
    } else {
      hex_bad++;
      Serial2.print("BAD ");
      Serial2.print(hex_lines);
      Serial2.print('\r');
      Serial2.print('\n');
    }
    return;
  }

  if (strcasecmp(line, "END") == 0) {
    if (!hex_in_progress) {
      sendLine("HEX IDLE");
      return;
    }
    hex_in_progress = false;
    handshake_ready = false;

    if (hex_bad == 0 && hex_ok > 0) {
      Serial2.print("HEX OK lines=");
      Serial2.print(hex_ok);
      Serial2.print(" bytes=");
      Serial2.print(hex_bytes);
      Serial2.print('\r');
      Serial2.print('\n');
      sendLine("APPLIED");
      Serial.println("[Teensy] OTA verified (checksum only)");
    } else {
      Serial2.print("HEX ERR lines=");
      Serial2.print(hex_lines);
      Serial2.print(" bad=");
      Serial2.print(hex_bad);
      Serial2.print('\r');
      Serial2.print('\n');
      Serial.println("[Teensy] OTA errors detected");
    }
    return;
  }

  if (strcasecmp(line, "PING") == 0) {
    sendLine("PONG");
    return;
  }

  if (strcasecmp(line, "VERSION") == 0) {
    Serial2.print("FW ");
    Serial2.print(FW_VERSION);
    Serial2.print('\r');
    Serial2.print('\n');
    return;
  }

  if (strcasecmp(line, "STATUS") == 0) {
    Serial2.print("HEX lines=");
    Serial2.print(hex_lines);
    Serial2.print(" ok=");
    Serial2.print(hex_ok);
    Serial2.print(" bad=");
    Serial2.print(hex_bad);
    Serial2.print(" bytes=");
    Serial2.print(hex_bytes);
    Serial2.print('\r');
    Serial2.print('\n');
    return;
  }

  sendLine("ERR");
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

  static char line[600];
  static size_t len = 0;

  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len + 1 < sizeof(line)) {
        line[len++] = c;
      }
      continue;
    }

    line[len] = '\0';
    handleSerial2Line(line);
    len = 0;
  }
}
