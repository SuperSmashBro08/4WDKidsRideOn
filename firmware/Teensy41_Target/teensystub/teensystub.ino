// teensy41_ota_stage3_recvhex_echo.ino
// Teensy 4.1 — OTA HEX verify + optional Telnet-friendly echo/heartbeat/status

// ------------------------- Config -------------------------
#define LED_PIN     13
#define BAUD_USB    115200   // Serial (USB) to PC
#define BAUD_UART   115200   // Serial2 (pins 7=RX2, 8=TX2) to ESP32
#define OTA_TOKEN   "8d81ab8762c545dabe699044766a0b72"
#define FW_VERSION  "v0.3-stage3-hex-verify+echo+status"

// Heartbeat LED period (ms). Will change to 87 ms after a good OTA apply.
static uint32_t blink_ms = 1000;

// Optional text heartbeat to Serial2 (Telnet). Default OFF to keep noise down.
static bool print_hb = false;

// Local echo (characters you type via Telnet show back instantly). Auto OFF during OTA.
static bool echo_enabled = true;

// ------------------------- State -------------------------
static bool     in_hex_session = false;
static uint32_t hex_lines = 0, hex_ok = 0, hex_bad = 0, hex_bytes = 0;

// ------------------------- Utils -------------------------
static inline void fastBlink(int times = 6, int ms = 120) {
  for (int i = 0; i < times; i++) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(ms); }
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

// Intel-HEX checksum + basic shape verification.
// This does NOT write flash; it only verifies structure & checksum and counts payload bytes.
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

// ------------------------- Setup -------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(BAUD_USB);
  Serial2.begin(BAUD_UART);  // Teensy 4.1: Serial2 is pins 7=RX2, 8=TX2

  delay(300);
  Serial.println("\n[Teensy] OTA HEX receiver ready on Serial2 (7=RX,8=TX).");
  Serial.print  ("[Teensy] FW: "); Serial.println(FW_VERSION);
  Serial.println("[Teensy] Commands over Serial2: HELLO <token> | BEGIN HEX | L <hex> | END |");
  Serial.println("          PING | VERSION | SET BLINK <ms> | HB ON|OFF | ECHO ON|OFF | STATUS | HELP");
}

// ------------------------- Helpers -------------------------
static void printHelp() {
  Serial2.println("OK HELP");
  Serial2.println("HELLO <token>");
  Serial2.println("BEGIN HEX  /  L <hexLine>  /  END");
  Serial2.println("PING  |  VERSION");
  Serial2.println("SET BLINK <ms>   (50..5000)");
  Serial2.println("HB ON|OFF        (text heartbeat every 1s)");
  Serial2.println("ECHO ON|OFF      (local char echo)");
  Serial2.println("STATUS");
}

// ------------------------- Loop -------------------------
void loop() {
  // LED heartbeat
  static uint32_t t_led = 0;
  if (millis() - t_led >= blink_ms) {
    t_led = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Optional text heartbeat to Serial2 (for Telnet bring-up). Default OFF.
  static uint32_t t_hb = 0;
  if (print_hb && (millis() - t_hb >= 1000)) {
    t_hb = millis();
    Serial2.print("HB "); Serial2.print(blink_ms); Serial2.println("ms");
  }

  // -------- Command / HEX receiver on Serial2 --------
  static String line;
  while (Serial2.available()) {
    char c = (char)Serial2.read();

    // If echo is enabled and we're NOT in an OTA session, echo raw chars back for a "console" feel
    if (echo_enabled && !in_hex_session) Serial2.write((uint8_t)c);

    if (c == '\r') continue;
    if (c != '\n') {
      // keep lines bounded to avoid runaway RAM if sender forgets newline
      if (line.length() < 520) line += c;
      continue;
    }
    line.trim();
    if (!line.length()) { line = ""; continue; }

    // Parse a copy case-insensitively for commands
    String cmd = line; cmd.toUpperCase();

    // ================= OTA handshake =================
    if (cmd.startsWith("HELLO ")) {
      String tok = line.substring(6); // preserve case
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
        echo_enabled = false;     // silence console echo during OTA
        hex_lines = hex_ok = hex_bad = hex_bytes = 0;
        Serial.println("[Teensy] HEX session begin");
        Serial2.println("HEX BEGIN");
      }
    }
    else if (line.startsWith("L ") && in_hex_session) {
      // exact check for data lines inside session
      String rec = line.substring(2);
      hex_lines++;
      bool ok = verifyIntelHexLine(rec);
      if (ok) {
        hex_ok++;
        Serial2.print("OK "); Serial2.println(hex_lines);
        if ((hex_lines % 64) == 0) Serial2.println("ACK");  // lightweight progress ping
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
        echo_enabled = true;      // resume console echo after OTA

        Serial.printf("[Teensy] HEX end: lines=%lu ok=%lu bad=%lu dataBytes=%lu\n",
                      hex_lines, hex_ok, hex_bad, hex_bytes);

        if (hex_bad == 0 && hex_ok > 0) {
          Serial2.printf("HEX OK lines=%lu bytes=%lu\n", hex_ok, hex_bytes);
          blink_ms = 87;                          // visible proof-of-apply
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
    else if (cmd.startsWith("SET BLINK ")) {
      long v = line.substring(10).toInt();
      if (v >= 50 && v <= 5000) { blink_ms = (uint32_t)v; Serial2.println("OK"); }
      else { Serial2.println("ERR"); }
    }
    else if (cmd == "HB ON") {
      print_hb = true; Serial2.println("OK");
    }
    else if (cmd == "HB OFF") {
      print_hb = false; Serial2.println("OK");
    }
    else if (cmd == "ECHO ON") {
      if (!in_hex_session) { echo_enabled = true; Serial2.println("OK"); }
      else { Serial2.println("BUSY"); }
    }
    else if (cmd == "ECHO OFF") {
      if (!in_hex_session) { echo_enabled = false; Serial2.println("OK"); }
      else { Serial2.println("BUSY"); }
    }
    else if (cmd == "STATUS") {
      Serial2.print("FW "); Serial2.println(FW_VERSION);
      Serial2.print("BLINK "); Serial2.print(blink_ms); Serial2.println("ms");
      Serial2.print("HB "); Serial2.println(print_hb ? "ON" : "OFF");
      Serial2.print("ECHO "); Serial2.println(echo_enabled ? "ON" : "OFF");
      Serial2.print("HEX session "); Serial2.println(in_hex_session ? "ACTIVE" : "IDLE");
      Serial2.printf("HEX lines=%lu ok=%lu bad=%lu bytes=%lu\n", hex_lines, hex_ok, hex_bad, hex_bytes);
      Serial2.printf("UPTIME %lu ms\n", millis());
    }
    else if (cmd == "HELP") {
      printHelp();
    }
    else {
      Serial2.println("ERR UNKNOWN (try HELP)");
    }

    line = ""; // reset for next line
  }
}
