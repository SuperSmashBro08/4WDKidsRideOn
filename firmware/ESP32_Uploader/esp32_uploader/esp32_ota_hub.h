#pragma once

#ifndef ESP32_UPLOADER_ESP32_OTA_HUB_H
#define ESP32_UPLOADER_ESP32_OTA_HUB_H
#define ESP32_OTA_HUB_H_INCLUDED
// ===== Mad Labs ESP32 OTA Hub (header-only) =====
// Web UI, Teensy .hex OTA over UART, ESP32 self-OTA .bin,
// live consoles, persisted last-result, reboot-friendly upload UX,
// plus diagnostics: UART1 stats, I2C scan, AS5600 dump.

#ifndef APP_NAME
  #define APP_NAME "ESP32_Uploader"
#endif
#ifndef APP_VER
  #define APP_VER  "v1.2.3"
#endif
#ifndef TEENSY_TX
  #define TEENSY_TX 43
#endif
#ifndef TEENSY_RX
  #define TEENSY_RX 44
#endif

// Teensy live console filter:
// 0 = store ALL non-empty lines; 1 = only lines starting with "S "
#ifndef TEENSY_CON_REQUIRE_S
  #define TEENSY_CON_REQUIRE_S 1
#endif

// I2C pins (default to SDA=8, SCL=9 for your board; override if needed)
#ifndef I2C_SDA_PIN
  #define I2C_SDA_PIN 8
#endif
#ifndef I2C_SCL_PIN
  #define I2C_SCL_PIN 9
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Wire.h>      // NEW: for I2C diagnostics/AS5600
#include <stdarg.h>    // for ELOGF
#include <string.h>

// ---- Optional ESP-IDF rollback marking ----
#ifndef __has_include
  #define __has_include(x) 0
#endif
#if __has_include(<esp_idf_version.h>)
  #include <esp_idf_version.h>
#endif
#if __has_include(<esp_ota_ops.h>)
  #include <esp_ota_ops.h>
  #define HAVE_ESP_OTA_OPS 1
#endif

namespace OtaHub {

// ---------- Public API ----------
void begin(const char* wifiSsid, const char* wifiPass, const char* otaToken,
           const char* mdnsHost = "esp32-teensy");
void loop();

// ---------- Private (implementation) ----------
namespace {
  // Build timestamp
  static const char* APP_BUILD_DATE = __DATE__ " " __TIME__;

  // Secrets (provided by begin()) — renamed to avoid macro collisions
  static String HUB_WIFI_SSID, HUB_WIFI_PASS, HUB_OTA_TOKEN;

  // Peripherals
  static HardwareSerial SerialTeensy(1);
  static WebServer server(80);

  // Teensy OTA state
  static String lastOtaLog;
  static bool   lastOtaSuccess = false;
  static unsigned long lastOtaMillis = 0;

  // ESP32 self-OTA state
  static String esp32LastLog;
  static bool   esp32LastSuccess = false;
  static unsigned long esp32LastMillis = 0;

  // Teensy live console buffer
  static const uint16_t CON_CAP = 500;
  static String         conBuf[CON_CAP];
  static uint32_t       conId = 0;
  static String         conLine;

  // ESP32 live console (our own log ring)
  static const uint16_t E_CON_CAP = 800;
  static String         eConBuf[E_CON_CAP];
  static uint32_t       eConId = 0;

  // UART1 diagnostics (NEW)
  static volatile uint32_t uart1_rx_bytes = 0;
  static volatile uint32_t uart1_lines    = 0;
  static uint32_t          uart1_last_ms  = 0;

  // Teensy binary telemetry
  static constexpr uint8_t  TELEM_MAGIC0 = 0xA5;
  static constexpr uint8_t  TELEM_MAGIC1 = 0x5A;
  static constexpr uint8_t  TELEM_VERSION = 1;
  static constexpr size_t   TELEM_MAX_PAYLOAD = 64;

  struct __attribute__((packed)) TelemetryPayload {
    uint16_t seq;
    uint32_t millis;
    uint16_t flags;
    uint16_t s1;
    uint16_t t1;
    int16_t  s2_us;
    int16_t  t2_us;
    int16_t  s2_map;
    int16_t  t2_map;
    uint16_t as_raw;
    int16_t  as_deg_centi;
    uint8_t  gear;
    uint8_t  steer_target;
    uint8_t  steer_state;
    uint8_t  reserved;
    int16_t  steer_err_centi;
  };

  struct __attribute__((packed)) TelemetryPacket {
    uint8_t          magic0;
    uint8_t          magic1;
    uint8_t          version;
    uint8_t          length;
    TelemetryPayload payload;
  };

  static TelemetryPacket latestTelem{};
  static bool            telemetryValid    = false;
  static bool            telemetryHasSeq   = false;
  static uint16_t        telemetryLastSeq  = 0;
  static uint32_t        telemetryDropCount = 0;
  static uint32_t        telemetryUpdatedMs = 0;

  enum class TelemetryState : uint8_t { WaitMagic0, WaitMagic1, WaitVersion, WaitLength, ReadPayload, SkipPayload };

  struct TelemetryParser {
    TelemetryState state = TelemetryState::WaitMagic0;
    uint8_t        curVersion = 0;
    uint8_t        curLength  = 0;
    uint8_t        payload[TELEM_MAX_PAYLOAD] = {};
    uint8_t        pos = 0;
    uint8_t        skip = 0;
  };

  static TelemetryParser telemParser;

  static inline void telemetryResetParser(){ telemParser = TelemetryParser{}; }

  static bool telemetryConsumeByte(uint8_t b){
    TelemetryParser& p = telemParser;

    switch (p.state){
      case TelemetryState::WaitMagic0:
        if (b == TELEM_MAGIC0){
          p.state = TelemetryState::WaitMagic1;
          return true;
        }
        return false;

      case TelemetryState::WaitMagic1:
        if (b == TELEM_MAGIC1){
          p.state = TelemetryState::WaitVersion;
          return true;
        }
        p.state = TelemetryState::WaitMagic0;
        return false;

      case TelemetryState::WaitVersion:
        p.curVersion = b;
        p.state = TelemetryState::WaitLength;
        return true;

      case TelemetryState::WaitLength:
        p.curLength = b;
        if (p.curLength == 0){
          p.state = TelemetryState::WaitMagic0;
          return true;
        }
        if (p.curLength > TELEM_MAX_PAYLOAD){
          p.skip = p.curLength;
          p.state = TelemetryState::SkipPayload;
          return true;
        }
        p.pos = 0;
        p.state = TelemetryState::ReadPayload;
        return true;

      case TelemetryState::ReadPayload:
        p.payload[p.pos++] = b;
        if (p.pos >= p.curLength){
          if (p.curVersion == TELEM_VERSION && p.curLength == sizeof(TelemetryPayload)){
            memcpy(&latestTelem.payload, p.payload, sizeof(TelemetryPayload));
            latestTelem.magic0 = TELEM_MAGIC0;
            latestTelem.magic1 = TELEM_MAGIC1;
            latestTelem.version = p.curVersion;
            latestTelem.length  = p.curLength;
            telemetryValid      = true;
            telemetryUpdatedMs  = millis();
            uint16_t seq        = latestTelem.payload.seq;
            if (telemetryHasSeq){
              uint16_t delta = (uint16_t)(seq - telemetryLastSeq);
              if (delta > 1) telemetryDropCount += (uint32_t)(delta - 1);
            }
            telemetryLastSeq = seq;
            telemetryHasSeq  = true;
          }
          p.state = TelemetryState::WaitMagic0;
        }
        return true;

      case TelemetryState::SkipPayload:
        if (p.skip){
          --p.skip;
          if (!p.skip) p.state = TelemetryState::WaitMagic0;
        }
        return true;
    }

    p.state = TelemetryState::WaitMagic0;
    return false;
  }

  static void drainTeensyInput(){
    while (SerialTeensy.available()){
      int raw = SerialTeensy.read();
      if (raw < 0) break;
      telemetryConsumeByte((uint8_t)raw);
    }
  }

  // ----- Utilities -----
  static inline void conStore(const String& s){ ++conId; conBuf[conId % CON_CAP] = s; }
  static inline void eConStore(const String& s){ ++eConId; eConBuf[eConId % E_CON_CAP] = s; }
  static void ELOG(const String& s){ eConStore(s); Serial.println(s); }
  static void ELOGF(const char* fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    eConStore(String(buf)); Serial.println(buf);
  }

  static inline void markAppValidIfPossible() {
    #if defined(HAVE_ESP_OTA_OPS) && defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
      #if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 4)
        esp_ota_mark_app_valid_cancel_rollback();
      #else
        #ifdef esp_ota_mark_app_valid
          esp_ota_mark_app_valid();
        #endif
      #endif
    #endif
  }

  static void ensureLittleFS(){
    if(!LittleFS.begin(true)) Serial.println("[FS] LittleFS mount failed");
    else                      Serial.println("[FS] LittleFS mounted");
  }

  static void connectWifi(const char* mdnsHost) {
    Serial.println("\n[WiFi] Connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(mdnsHost);
    WiFi.begin(HUB_WIFI_SSID.c_str(), HUB_WIFI_PASS.c_str());
    unsigned long t0=millis();
    while(WiFi.status()!=WL_CONNECTED){
      delay(250); Serial.print('.');
      if(millis()-t0>25000){ Serial.println("\n[WiFi] Failed. Rebooting."); delay(500); ESP.restart(); }
    }
    Serial.printf("\n[WiFi] Connected. IP: %s\n",WiFi.localIP().toString().c_str());
    if(MDNS.begin(mdnsHost)){ MDNS.addService("http","tcp",80); Serial.printf("[mDNS] %s.local ready\n", mdnsHost); }
    else Serial.println("[mDNS] start failed");
    ELOGF("Boot %s %s | IP %s", APP_NAME, APP_VER, WiFi.localIP().toString().c_str());
  }

  static String htmlEscape(const String& in) {
    String out; out.reserve(in.length() + 16);
    for (size_t i=0;i<in.length();++i) {
      char c = in[i];
      if (c=='&') out += F("&amp;");
      else if (c=='<') out += F("&lt;");
      else if (c=='>') out += F("&gt;");
      else out += c;
    }
    return out;
  }

  static bool readLineFrom(Stream& s, String& out, uint32_t to_ms=3000){
    out=""; unsigned long t0=millis();
    while(millis()-t0<to_ms){
      while(s.available()){
        int raw = s.read();
        if (raw < 0) break;
        uint8_t byte = (uint8_t)raw;
        if (telemetryConsumeByte(byte)) { t0 = millis(); continue; }
        if ((byte < 0x20 && byte != '\n' && byte != '\r') || byte >= 0x7F) { t0 = millis(); continue; }
        char c = (char)byte;
        if(c=='\r') continue;
        if(c=='\n'){ out.trim(); return true; }
        if(out.length()<400) out+=c;
      }
      delay(1); yield();
    }
    out.trim(); return out.length()>0;
  }

  enum class TelemetryState : uint8_t { WaitMagic0, WaitMagic1, WaitVersion, WaitLength, ReadPayload, SkipPayload };

  static bool telemetryConsumeByte(uint8_t b){
    static TelemetryState state = TelemetryState::WaitMagic0;
    static uint8_t curVersion = 0;
    static uint8_t curLength  = 0;
    static uint8_t payload[TELEM_MAX_PAYLOAD];
    static uint8_t pos = 0;
    static uint8_t skip = 0;

    switch (state){
      case TelemetryState::WaitMagic0:
        if (b == TELEM_MAGIC0){
          state = TelemetryState::WaitMagic1;
          return true;
        }
        return false;

      case TelemetryState::WaitMagic1:
        if (b == TELEM_MAGIC1){
          state = TelemetryState::WaitVersion;
          return true;
        }
        state = TelemetryState::WaitMagic0;
        return false;

      case TelemetryState::WaitVersion:
        curVersion = b;
        state = TelemetryState::WaitLength;
        return true;

      case TelemetryState::WaitLength:
        curLength = b;
        if (curLength == 0){
          state = TelemetryState::WaitMagic0;
          return true;
        }
        if (curLength > TELEM_MAX_PAYLOAD){
          skip = curLength;
          state = TelemetryState::SkipPayload;
          return true;
        }
        pos = 0;
        state = TelemetryState::ReadPayload;
        return true;

      case TelemetryState::ReadPayload:
        payload[pos++] = b;
        if (pos >= curLength){
          if (curVersion == TELEM_VERSION && curLength == sizeof(TelemetryPayload)){
            memcpy(&latestTelem.payload, payload, sizeof(TelemetryPayload));
            latestTelem.magic0 = TELEM_MAGIC0;
            latestTelem.magic1 = TELEM_MAGIC1;
            latestTelem.version = curVersion;
            latestTelem.length  = curLength;
            telemetryValid      = true;
            telemetryUpdatedMs  = millis();
            uint16_t seq        = latestTelem.payload.seq;
            if (telemetryHasSeq){
              uint16_t delta = (uint16_t)(seq - telemetryLastSeq);
              if (delta > 1) telemetryDropCount += (uint32_t)(delta - 1);
            }
            telemetryLastSeq = seq;
            telemetryHasSeq  = true;
          }
          state = TelemetryState::WaitMagic0;
        }
        return true;

      case TelemetryState::SkipPayload:
        if (skip){
          --skip;
          if (!skip) state = TelemetryState::WaitMagic0;
        }
        return true;
    }

    state = TelemetryState::WaitMagic0;
    return false;
  }

  // === Teensy console pump (robust CR/LF + idle flush) ===
  static void pumpTeensyConsole(){
    static bool lastWasCR = false;
    static uint32_t lastByteMs = 0;
    static uint32_t lastTextByteMs = 0;

    while (SerialTeensy.available()){
      int raw = SerialTeensy.read();
      if (raw < 0) break;
      uint8_t byte = (uint8_t)raw;
      uart1_rx_bytes++;
      uint32_t nowMs = millis();
      lastByteMs = uart1_last_ms = nowMs;

      if (telemetryConsumeByte(byte)){
        continue;
      }

      char c = (char)byte;
      if (c == '\r' || c == '\n'){
        if (c == '\n' && lastWasCR) { lastWasCR = false; continue; }
        lastWasCR = (c == '\r');
        lastTextByteMs = nowMs;

        conLine.trim();
        if (conLine.length()){
          bool stored=false;
          #if TEENSY_CON_REQUIRE_S
            if (conLine.startsWith("S ")){
              String trimmed = conLine;
              trimmed.remove(0,2);
              conStore(trimmed);
              stored=true;
            }
          #else
            String trimmed = conLine;
            if (trimmed.startsWith("S ")) trimmed.remove(0,2);
            conStore(trimmed);
            stored=true;
          #endif
          if (stored) uart1_lines++;
        }
        conLine = "";
      } else {
        lastWasCR = false;
        lastTextByteMs = nowMs;
        if (conLine.length() < 512) conLine += c;
      }
    }

    // idle flush (if newline never arrives)
    if (conLine.length() && (millis() - lastTextByteMs) > 120){
      conLine.trim();
      bool stored=false;
      #if TEENSY_CON_REQUIRE_S
        if (conLine.startsWith("S ")){
          String trimmed = conLine;
          trimmed.remove(0,2);
          conStore(trimmed);
          stored=true;
        }
      #else
        String trimmed = conLine;
        if (trimmed.startsWith("S ")) trimmed.remove(0,2);
        conStore(trimmed);
        stored=true;
      #endif
      conLine = "";
      if (stored) uart1_lines++;
    }
  }

  // ---------- Persist ESP32 last-OTA result across reboot ----------
  static void esp32WriteLastFile(bool ok, const String& log) {
    File f = LittleFS.open("/esp32_last.txt", FILE_WRITE);
    if (f) { f.print(ok ? "OK\n" : "ERR\n"); f.print(log); f.close(); }
  }
  static bool esp32ReadLastFile(String& out) {
    File f = LittleFS.open("/esp32_last.txt", FILE_READ);
    if (!f) return false;
    out = f.readString(); f.close(); return true;
  }

  // ---------- Teensy HEX helpers ----------
  static bool validateHexFile(File& f, String& err) {
    if (!f) { err="open failed"; return false; }
    if (!f.size()) { err="empty file"; return false; }
    size_t pos = f.position();
    String first = f.readStringUntil('\n'); first.trim();
    while (first.length()==0 && f.available()) { first = f.readStringUntil('\n'); first.trim(); }
    if (first.length()==0 || first[0] != ':') { err="not Intel HEX (no leading colon)"; f.seek(pos); return false; }
    f.seek(0);
    String line, lastNonEmpty;
    while (f.available()) { line = f.readStringUntil('\n'); line.trim(); if (line.length()) lastNonEmpty = line; }
    f.seek(0);
    if (lastNonEmpty != ":00000001FF") { err = "missing EOF (:00000001FF)"; return false; }
    return true;
  }

  static bool expectStartsWith(Stream& s, const char* prefix, String* captured=nullptr, uint32_t to_ms=3000) {
    String line; if (!readLineFrom(s, line, to_ms)) return false;
    Serial.print("<- "); Serial.println(line);
    if (captured) *captured = line;
    return line.startsWith(prefix);
  }

  static bool flashTeensyFromFS(const char* path, String& logOut) {
    File f = LittleFS.open(path, FILE_READ);
    String vErr;
    if (!validateHexFile(f, vErr)) { logOut += "HEX validate error: " + vErr + "\n"; f.close(); return false; }

    const size_t totalBytes = f.size();
    size_t sentBytes = 0;
    auto logProgress = [&](size_t extra) {
      sentBytes += extra;
      if (totalBytes) {
        int pct = (int)((100ULL * sentBytes) / totalBytes);
        logOut += "PROGRESS " + String(pct) + "% (" + String(sentBytes) + "/" + String(totalBytes) + ")\n";
      }
    };

    drainTeensyInput();

    SerialTeensy.printf("HELLO %s\r\n", HUB_OTA_TOKEN.c_str());
    logOut += "Sent HELLO\n";

    String resp;
    bool helloOk = false;
    uint8_t attempts = 0;
    unsigned long t_deadline = millis() + 4000;

    while (millis() < t_deadline && attempts < 3) {
      if (!readLineFrom(SerialTeensy, resp, 800)) {
        attempts++;
        SerialTeensy.printf("HELLO %s\r\n", HUB_OTA_TOKEN.c_str());
        logOut += "HELLO timeout, retrying...\n";
        continue;
      }
      logOut += "<- " + resp + "\n";
      if (resp == "READY") { helloOk = true; break; }
      if (resp == "BUSY")  { delay(250); continue; }
      if (resp == "NACK")  { logOut += "Token mismatch (NACK)\n"; break; }
      if (resp.startsWith("S ") || resp.startsWith("FW ") || resp.startsWith("FLASHERX ")) continue;
      if (resp == "ERR") { 
        attempts++; 
        SerialTeensy.printf("HELLO %s\r\n", HUB_OTA_TOKEN.c_str());
        logOut += "HELLO got ERR, retrying...\n"; 
        continue; 
      }
      logOut += "Unexpected during HELLO: " + resp + "\n";
    }
    if (!helloOk) { f.close(); return false; }

    SerialTeensy.print("BEGIN HEX\r\n");
    if (!expectStartsWith(SerialTeensy, "HEX BEGIN", &resp, 1500)) {
      logOut += "No HEX BEGIN (got: " + resp + ")\n"; f.close(); return false;
    }
    logOut += "<- " + resp + "\n";

    f.seek(0);
    size_t lineNumber = 0, ok=0, bad=0;
    const uint16_t lineDelayUs = 200;
    while (f.available()) {
      String rec = f.readStringUntil('\n'); rec.trim();
      if (!rec.length()) continue;
      lineNumber++;

      SerialTeensy.print("L "); SerialTeensy.print(rec); SerialTeensy.print("\r\n");

      String ack;
      if (!readLineFrom(SerialTeensy, ack, 1500)) { logOut += "Line " + String(lineNumber) + " timeout\n"; bad++; break; }
      logOut += "<- " + ack + "\n";

      if (ack.startsWith("OK ")) {
        uint32_t n = ack.substring(3).toInt();
        if (n != lineNumber) logOut += "WARN: OK index mismatch (got " + String(n) + ")\n";
        ok++;
      } else if (ack.startsWith("BAD ")) {
        bad++; logOut += "Teensy reported BAD at line " + String(lineNumber) + "\n"; break;
      } else {
        bad++; logOut += "Unexpected reply at line " + String(lineNumber) + ": " + ack + "\n"; 
        break;
      }

      logProgress(rec.length()+3);
      delayMicroseconds(lineDelayUs);
      yield();
    }
    f.close();

    SerialTeensy.print("END\r\n");
    logOut += "Sent lines: " + String(lineNumber) + " (OK=" + String(ok) + " BAD=" + String(bad) + ")\n";

    bool success = false;
    unsigned long t0 = millis();
    while (millis()-t0 < 6000) {
      String summary;
      if (!readLineFrom(SerialTeensy, summary, 500)) { yield(); continue; }
      logOut += summary + "\n";
      if (summary.startsWith("HEX OK")) success = true;
      if (summary=="APPLIED" || summary.startsWith("HEX ERR")) break;
    }
    return success && (bad==0);
  }

  // ---------- Diagnostics: UART1 + I2C + AS5600 ----------
  static void handleUart1Stats(){
    String out; out.reserve(256);
    out += "rx_bytes="; out += String(uart1_rx_bytes);
    out += "  lines=";  out += String(uart1_lines);
    out += "  last_ms="; out += String(uart1_last_ms);
    out += "\n";
    uint32_t cur = conId;
    if (cur){
      out += "tail: ";
      const String& line = conBuf[cur % CON_CAP];
      if (line.length()){
        if (line.length() > 120) out += line.substring(line.length()-120);
        else out += line;
      } else out += "<empty>";
      out += "\n";
    }
    out += "telemetry=";
    if (telemetryValid){
      out += "seq="; out += String(latestTelem.payload.seq);
      out += " drops="; out += String(telemetryDropCount);
      out += " age_ms="; out += String(millis() - telemetryUpdatedMs);
    } else {
      out += "none";
    }
    out += "\n";
    server.send(200, "text/plain", out);
  }

  static void handleTelemetryBin(){
    if (!telemetryValid || latestTelem.length == 0){
      server.send(204, "text/plain", "");
      return;
    }
    size_t payloadLen = 4u + latestTelem.length;
    server.setContentLength(payloadLen);
    server.send(200, "application/octet-stream", "");
    WiFiClient client = server.client();
    client.write(reinterpret_cast<const uint8_t*>(&latestTelem), payloadLen);
  }

  static void handleTelemetryJson(){
    String out;
    out.reserve(320);
    if (!telemetryValid){
      out = F("{\"valid\":false}");
    } else {
      const TelemetryPayload& p = latestTelem.payload;
      uint32_t age = millis() - telemetryUpdatedMs;
      out += F("{\"valid\":true");
      out += F(",\"seq\":"); out += p.seq;
      out += F(",\"millis\":"); out += p.millis;
      out += F(",\"flags\":"); out += p.flags;
      out += F(",\"s1\":"); out += p.s1;
      out += F(",\"t1\":"); out += p.t1;
      out += F(",\"s2_us\":"); out += (int)p.s2_us;
      out += F(",\"t2_us\":"); out += (int)p.t2_us;
      out += F(",\"s2_map\":"); out += (int)p.s2_map;
      out += F(",\"t2_map\":"); out += (int)p.t2_map;
      out += F(",\"as_raw\":"); out += p.as_raw;
      out += F(",\"as_deg_centi\":"); out += (int)p.as_deg_centi;
      out += F(",\"gear\":"); out += (unsigned)p.gear;
      out += F(",\"steer_target\":"); out += (unsigned)p.steer_target;
      out += F(",\"steer_state\":"); out += (unsigned)p.steer_state;
      out += F(",\"steer_err_centi\":"); out += (int)p.steer_err_centi;
      out += F(",\"drops\":"); out += telemetryDropCount;
      out += F(",\"age_ms\":"); out += age;
      out += '}';
    }
    server.send(200, "application/json", out);
  }

  static bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n){
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    int got = Wire.readBytes(buf, n);
    return got == n;
  }

  static void handleI2CScan(){
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    String out; out.reserve(512);
    out += "Scanning I2C on SDA="; out += String(I2C_SDA_PIN);
    out += " SCL="; out += String(I2C_SCL_PIN); out += "\n";
    uint8_t found = 0;
    for (uint8_t a=0x03; a<=0x77; ++a){
      Wire.beginTransmission(a);
      uint8_t e = Wire.endTransmission();
      if (e == 0){
        out += "  - 0x"; out += String(a, HEX); out += "\n";
        found++;
      }
    }
    if (!found) out += "  (no devices)\n";
    server.send(200, "text/plain", out);
  }

  static void handleAS5600Dump(){
    const uint8_t ADDR = 0x36;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

    String out; out.reserve(256);
    out += "AS5600 @ 0x36\n";

    uint8_t st=0;
    if (i2cReadN(ADDR, 0x0B, &st, 1)){
      out += "STATUS 0x0B = 0x"; out += String(st, HEX); out += "  (MD=";
      out += (st & 0x20) ? "1" : "0"; out += ", ML="; out += (st & 0x10) ? "1" : "0";
      out += ", MH="; out += (st & 0x08) ? "1" : "0"; out += ")\n";
    } else {
      out += "STATUS read failed\n";
    }

    uint8_t b[2];
    if (i2cReadN(ADDR, 0x0C, b, 2)){
      uint16_t raw = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
      float deg = raw * (360.0f/4096.0f);
      out += "ANGLE  raw="; out += String(raw);
      out += "  deg="; out += String(deg, 2); out += "\n";
    } else {
      out += "ANGLE read failed\n";
    }
    server.send(200, "text/plain", out);
  }

  // ---------- Pages (Teensy & ESP32 consoles + Index) ----------
  static String buildEsp32ConsolePage() {
    String page;
    page.reserve(2800);
    page += F(
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name=viewport content='width=device-width,initial-scale=1'>"
      "<title>ESP32 Console</title>"
      "<style>"
        "body{font-family:system-ui;margin:12px}"
        "#log{background:#0b1220;color:#d7e0f2;padding:12px;border-radius:8px;height:70vh;overflow:auto;font:13px ui-monospace,Consolas,monospace}"
      "</style>"
      "</head><body>"
      "<h3>ESP32 Console</h3><p><a href='/'>⬅ Back</a></p>"
      "<div id='log'></div>"
      "<script>"
        "let since=0;const log=document.getElementById('log');"
        "function addLine(t){if(!t)return;const d=document.createElement('div');d.textContent=t;log.appendChild(d);if(log.childNodes.length>1000)log.removeChild(log.firstChild);log.scrollTop=log.scrollHeight;}"
        "function pull(){fetch('/esp32-tail?since='+since).then(r=>r.text()).then(t=>{const n=t.indexOf('\\n');if(n>0){const hdr=t.substring(0,n);if(hdr.startsWith('NEXT '))since=parseInt(hdr.substring(5))||since;const body=t.substring(n+1);body.split('\\n').filter(Boolean).forEach(addLine);}}).catch(()=>{});}"
        "setInterval(pull,180);"
      "</script>"
      "</body></html>"
    );
    return page;
  }

  static String buildTeensyConsolePage() {
    String page;
    page.reserve(3200);
    page += F(
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name=viewport content='width=device-width,initial-scale=1'>"
      "<title>Teensy Console</title>"
      "<style>"
        "body{font-family:system-ui;margin:12px}"
        "#log{background:#0b1220;color:#d7e0f2;padding:12px;border-radius:8px;height:70vh;overflow:auto;font:13px ui-monospace,Consolas,monospace}"
        ".toolbar{margin:8px 0}"
        ".toolbar button{padding:6px 12px;border:0;border-radius:6px;background:#1f2937;color:#fff;cursor:pointer}"
        ".toolbar button:active{transform:translateY(1px)}"
      "</style>"
      "</head><body>"
      "<h3>Teensy Console</h3><p><a href='/'>⬅ Back</a></p>"
      "<div class='toolbar'><button id='clear-log' type='button'>Clear</button></div>"
      "<div id='log'></div>"
      "<script>"
        "let since=0;const log=document.getElementById('log');"
        "const clearBtn=document.getElementById('clear-log');"
        "if(clearBtn){clearBtn.addEventListener('click',()=>{log.innerHTML='';log.scrollTop=0;});}"
        "function addLine(t){if(!t)return;const d=document.createElement('div');d.textContent=t;log.appendChild(d);if(log.childNodes.length>1000)log.removeChild(log.firstChild);log.scrollTop=log.scrollHeight;}"
        "function pull(){fetch('/tail?since='+since).then(r=>r.text()).then(t=>{const n=t.indexOf('\\n');if(n>0){const hdr=t.substring(0,n);if(hdr.startsWith('NEXT '))since=parseInt(hdr.substring(5))||since;const body=t.substring(n+1);body.split('\\n').filter(Boolean).forEach(addLine);}}).catch(()=>{});}"
        "setInterval(pull,180);"
      "</script>"
      "</body></html>"
    );
    return page;
  }

  static String buildIndexPage() {
    String page;
    page.reserve(12000);
    page += F(
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name=viewport content='width=device-width,initial-scale=1'>"
      "<title>Mad Labs OTA Hub</title>"
      "<style>"
        "body{font-family:system-ui;margin:24px;max-width:900px}"
        "section{margin:18px 0;padding:16px;border:1px solid #e5e7eb;border-radius:12px}"
        "h2,h3{margin:0 0 8px 0}"
        "pre{background:#f6f8fa;padding:12px;border-radius:8px;overflow:auto}"
        "code{background:#f6f8fa;padding:2px 6px;border-radius:4px}"
        ".row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}"
        "button{padding:8px 14px;border:0;border-radius:8px;background:#111;color:#fff;cursor:pointer}"
        "button[disabled]{opacity:.6;cursor:not-allowed}"
        "a.btn{display:inline-block;padding:8px 12px;border-radius:8px;background:#0b5;color:#fff;text-decoration:none}"
        ".bar{width:100%;height:12px;background:#eee;border-radius:6px;overflow:hidden;margin:8px 0 4px 0;display:none}"
        ".fill{height:100%;width:0%}"
        ".ok{background:#2ecc71}.bad{background:#e74c3c}.info{background:#3498db}"
        ".muted{color:#64748b}"
      "</style></head><body>"
      "<h2>Mad Labs OTA Hub</h2>"
      "<p class='muted'>Upload new firmware for <b>Teensy (.hex)</b> and <b>ESP32 (.bin)</b>, and view live consoles.</p>"
    );

    // Teensy section...
    page += F(
      "<section><h3>Update Teensy (.hex)</h3>"
      "<form id='f-teensy' method='POST' action='/upload' enctype='multipart/form-data'>"
      "<div class='row'>"
      "<input id='file-teensy' type='file' name='firmware' accept='.hex' required>"
      "<button id='go-teensy' type='submit'>Upload & Flash</button>"
      "<a class='btn' href='/console'>Teensy Live Console</a>"
      "<button type='button' onclick=\"fetch('/ping').then(r=>r.text()).then(t=>alert(t)).catch(()=>alert('ping failed'))\">Ping Teensy</button>"
      "<button type='button' onclick=\"fetch('/version').then(r=>r.text()).then(t=>alert(t)).catch(()=>alert('version failed'))\">Get Teensy Version</button>"
      "</div>"
      "<div id='bar-teensy' class='bar'><div id='fill-teensy' class='fill info'></div></div>"
      "<div id='status-teensy'></div></form></section>"
    );

    // ESP32 section...
    page += F(
      "<section><h3>Update ESP32 (.bin)</h3>"
      "<div class='row'>"
      "<input id='file-esp' type='file' accept='.bin' required>"
      "<button id='go-esp' type='button' onclick=\"(function(){"
        "try{"
          "var fE=document.getElementById('file-esp');"
          "var barE=document.getElementById('bar-esp');"
          "var fillE=document.getElementById('fill-esp');"
          "var statE=document.getElementById('status-esp');"
          "var btnE=document.getElementById('go-esp');"
          "if(!fE||!fE.files||!fE.files.length){alert('Choose a .bin first');return;}"
          "btnE.disabled=true;"
          "statE.textContent='Uploading…';"
          "barE.style.display='block';"
          "fillE.style.width='0%';"
          "fillE.className='fill info';"

          "var fd=new FormData();"
          "fd.append('firmware', fE.files[0]);"
          "var xhr=new XMLHttpRequest();"
          "xhr.open('POST','/esp32-ota');"
          "xhr.timeout=240000;"

          "var sawProgress=false;"
          "xhr.upload.onprogress=function(ev){"
            "if(ev.lengthComputable){sawProgress=true;fillE.style.width=Math.round((ev.loaded/ev.total)*100)+'%';}"
            "else{fillE.style.width='100%';}"
          "};"
          "xhr.onreadystatechange=function(){if(xhr.readyState===2){fillE.style.width='100%';}};"

          "function pollBackOnline(){"
            "var tries=0;"
            "(function tick(){"
              "fetch('/esp32-version?x='+(Date.now()),{cache:'no-store'})"
              ".then(function(r){return r.text();})"
              ".then(function(txt){"
                "fillE.className='fill ok';"
                "var first=(txt||'').split('\\n')[0];"
                "statE.textContent='ESP32 back online ✔'+(first?(' — '+first):'');"
                "btnE.disabled=false;"
              "})"
              ".catch(function(){"
                "if(++tries<=60){ setTimeout(tick,1000); }"
                "else { statE.textContent='ESP32 flashed, but not responding yet.'; btnE.disabled=false; }"
              "});"
            "})();"
          "}"

          "function finish(){"
            "var body=(xhr.responseText||'').trim();"
            "var ok=(xhr.status===200 && body==='OK');"
            "if(ok){"
              "fillE.className='fill ok';"
              "statE.textContent='ESP32 flashed. Rebooting…';"
              "pollBackOnline();"
            "}else{"
              "if(xhr.status===0 && sawProgress){"
                "statE.textContent='Rebooting… (connection will drop briefly)';"
                "fillE.className='fill info';"
                "pollBackOnline();"
              "}else{"
                "fillE.className='fill bad';"
                "statE.textContent='ESP32 update failed. HTTP '+xhr.status+' '+xhr.statusText+' | Body: '+body;"
                "btnE.disabled=false;"
              "}"
            "}"
          "}"

          "xhr.onload=finish;"
          "xhr.onerror=finish;"
          "xhr.ontimeout=finish;"
          "xhr.send(fd);"
        "}catch(e){alert('ESP32 JS error: '+e);}"
      "})()\">Upload & Flash</button>"
      "<a class='btn' href='/esp32-console'>ESP32 Live Console</a>"
      "<button type='button' onclick=\"fetch('/esp32-ping').then(r=>r.text()).then(t=>alert(t)).catch(()=>alert('ping failed'))\">Ping ESP32</button>"
      "<button type='button' onclick=\"fetch('/esp32-version').then(r=>r.text()).then(t=>alert(t)).catch(()=>alert('version failed'))\">Get ESP32 Version</button>"
      "</div>"
      "<div id='bar-esp' class='bar'><div id='fill-esp' class='fill info'></div></div>"
      "<div id='status-esp'></div>"
      "</section>"
    );

    // Last results (Teensy + ESP32)
    if (lastOtaLog.length()) {
      page += lastOtaSuccess ? F("<h3 style='color:#0a0'>Last Teensy OTA: success</h3>") : F("<h3 style='color:#b00'>Last Teensy OTA: failed</h3>");
      if (lastOtaMillis) {
        page += F("<p class='muted'><small>Completed ");
        page += String((millis() - lastOtaMillis) / 1000);
        page += F(" s ago</small></p>");
      }
      page += F("<pre>"); page += htmlEscape(lastOtaLog); page += F("</pre>");
    }
    if (esp32LastLog.length()) {
      page += esp32LastSuccess ? F("<h3 style='color:#0a0'>Last ESP32 OTA: success</h3>") : F("<h3 style='color:#b00'>Last ESP32 OTA: failed</h3>");
      if (esp32LastMillis) {
        page += F("<p class='muted'><small>Completed ");
        page += String((millis() - esp32LastMillis) / 1000);
        page += F(" s ago</small></p>");
      }
      page += F("<pre>"); page += htmlEscape(esp32LastLog); page += F("</pre>");
    }

    // Teensy JS uploader
    page += F(
      "<script>"
      "const ft=document.getElementById('f-teensy');"
      "const bt=document.getElementById('go-teensy');"
      "const fT=document.getElementById('file-teensy');"
      "const barT=document.getElementById('bar-teensy');"
      "const fillT=document.getElementById('fill-teensy');"
      "const statT=document.getElementById('status-teensy');"
      "function finalizeTeensyUI(ok){fillT.className=ok?'fill ok':'fill bad';statT.textContent=ok?'Flashing complete.':'Upload failed.';setTimeout(function(){window.location='/'},900);}"
      "async function checkTeensyLast(){try{const r=await fetch('/last',{cache:'no-store'});const t=await r.text();finalizeTeensyUI(t.trim()==='OK');}catch(e){finalizeTeensyLast(false);}}"
      "ft.addEventListener('submit',function(e){"
        "e.preventDefault();"
        "if(!fT.files.length){alert('Choose a .hex first');return;}"
        "bt.disabled=true;"
        "statT.textContent='Uploading…';"
        "barT.style.display='block';"
        "fillT.style.width='0%';"
        "fillT.className='fill info';"
        "const xhr=new XMLHttpRequest();"
        "xhr.open('POST','/upload');"
        "xhr.timeout=180000;"
        "xhr.upload.onprogress=function(ev){"
          "if(ev.lengthComputable){fillT.style.width=Math.round((ev.loaded/ev.total)*100)+'%';}"
          "else{fillT.style.width='100%';}"
        "};"
        "xhr.onload=function(){checkTeensyLast();};"
        "xhr.onerror=function(){checkTeensyLast();};"
        "xhr.ontimeout=function(){checkTeensyLast();};"
        "xhr.send(new FormData(ft));"
      "});"
      "</script>"
      "</body></html>"
    );
    return page;
  }

  // ---------- HTTP Handlers ----------
  static void handleRoot(){ server.send(200,"text/html",buildIndexPage()); }

  // Teensy quick commands
  static void handlePing(){
    drainTeensyInput();
    SerialTeensy.print("PING\r\n");
    String r; if (readLineFrom(SerialTeensy, r, 800)) server.send(200,"text/plain",r);
    else server.send(504,"text/plain","timeout");
  }
  static void handleVersion(){
    drainTeensyInput();
    SerialTeensy.print("VERSION\r\n");
    String a,b,o;
    if (readLineFrom(SerialTeensy,a,800)) o+=a+"\n";
    if (readLineFrom(SerialTeensy,b,200)) o+=b+"\n";
    server.send(o.length()?200:504,"text/plain",o.length()?o:"timeout");
  }
  static void handleLast(){ server.send(200,"text/plain",lastOtaSuccess?"OK":"ERR"); }

  // ESP32 quick commands
  static void handleEspPing(){ server.send(200,"text/plain","ESP32 PONG "+String(millis())); }
  static void handleEspVersion(){
    String out; out.reserve(128);
    out += APP_NAME; out += " "; out += APP_VER; out += "\n";
    out += "Build: "; out += APP_BUILD_DATE; out += "\n";
    out += "IP: "; out += WiFi.localIP().toString(); out += "\n";
    server.send(200,"text/plain",out);
  }
  static void handleEsp32Last(){
    String buf;
    if (esp32ReadLastFile(buf)) {
      int nl = buf.indexOf('\n');
      String head = (nl >= 0) ? buf.substring(0, nl) : buf;
      server.send(200, "text/plain", head);
      return;
    }
    server.send(200,"text/plain",esp32LastSuccess?"OK":"ERR");
  }

  // Console pages
  static void handleConsolePage(){ server.send(200,"text/html",buildTeensyConsolePage()); }
  static void handleEsp32ConsolePage(){ server.send(200,"text/html",buildEsp32ConsolePage()); }

  // Tail endpoints
  static void handleTail(){ // Teensy
    uint32_t since=0;
    if(server.hasArg("since")) since=(uint32_t)strtoul(server.arg("since").c_str(),nullptr,10);
    const uint16_t MAX_LINES=200;
    uint32_t curId=conId;
    if(since>=curId){ server.send(200,"text/plain","NEXT "+String(curId)+"\n"); return; }
    uint32_t start=since+1;
    if(start+MAX_LINES<curId) start=curId-MAX_LINES;
    String out; out.reserve(4096);
    out += "NEXT "; out += String(curId); out += "\n";
    for(uint32_t id=start; id<=curId; ++id){
      if((curId-id)>=CON_CAP) continue;
      const String& line = conBuf[id % CON_CAP];
      if(line.length()){ out += line; out += "\n"; if(out.length()>60000) break; }
    }
    server.send(200,"text/plain",out);
  }
  static void handleEsp32Tail(){ // ESP32
    uint32_t since=0;
    if(server.hasArg("since")) since=(uint32_t)strtoul(server.arg("since").c_str(),nullptr,10);
    const uint16_t MAX_LINES=300;
    uint32_t curId=eConId;
    if(since>=curId){ server.send(200,"text/plain","NEXT "+String(curId)+"\n"); return; }
    uint32_t start=since+1;
    if(start+MAX_LINES<curId) start=curId-MAX_LINES;
    String out; out.reserve(4096);
    out += "NEXT "; out += String(curId); out += "\n";
    for(uint32_t id=start; id<=curId; ++id){
      if((curId-id)>=E_CON_CAP) continue;
      const String& line = eConBuf[id % E_CON_CAP];
      if(line.length()){ out += line; out += "\n"; if(out.length()>60000) break; }
    }
    server.send(200,"text/plain",out);
  }

  // ---------- Teensy upload (.hex) ----------
  static File uploadFile;
  static bool uploadedWasHex = false;
  static String uploadName;

  static void handleUpload() {
    HTTPUpload& up = server.upload();

    if (up.status == UPLOAD_FILE_START) {
      uploadName = up.filename;
      uploadedWasHex = uploadName.endsWith(".hex") || uploadName.endsWith(".HEX");
      Serial.printf("[HTTP] Upload start: %s\n", uploadName.c_str());
      uploadFile = LittleFS.open("/teensy41.hex", FILE_WRITE);
      if (!uploadFile) { Serial.println("[HTTP] open for write failed"); }
    }
    else if (up.status == UPLOAD_FILE_WRITE) {
      if (uploadFile) uploadFile.write(up.buf, up.currentSize);
    }
    else if (up.status == UPLOAD_FILE_END) {
      if (!uploadFile) { server.send(500, "text/plain", "Upload failed to open file"); return; }
      uploadFile.close();
      Serial.printf("[HTTP] Upload complete: %s bytes=%u\n", uploadName.c_str(), up.totalSize);

      if (!uploadedWasHex) {
        LittleFS.remove("/teensy41.hex");
        server.send(415, "text/plain", "Please upload the Teensy Intel HEX (.hex) file.");
        return;
      }

      String log;
      bool ok = flashTeensyFromFS("/teensy41.hex", log);

      lastOtaSuccess = ok;
      lastOtaLog = log;
      lastOtaMillis = millis();

      Serial.println("[TEENSY OTA RESULT]\n" + log);
      server.send(ok ? 200 : 500, "text/plain", ok ? "OK\n" : "ERR\n");
    }
  }

  // ---------- ESP32 self-OTA (.bin) ----------
  static void handleEsp32Ota() {
    HTTPUpload& up = server.upload();

    if (up.status == UPLOAD_FILE_START) {
      String fname = up.filename;
      bool isBin = fname.endsWith(".bin") || fname.endsWith(".BIN");
      esp32LastLog = "";
      esp32LastSuccess = false;
      Serial.printf("[ESP32 OTA] Upload start: %s\n", fname.c_str());
      ELOGF("ESP32 OTA start: %s", fname.c_str());

      if (!isBin) {
        esp32LastLog += "Reject: not a .bin\n";
        return; // handled in END stage
      }

      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        esp32LastLog += "Update.begin failed: ";
        esp32LastLog += String(Update.errorString()) + "\n";
      } else {
        esp32LastLog += "Update.begin OK\n";
      }
    }
    else if (up.status == UPLOAD_FILE_WRITE) {
      if (!Update.hasError()) {
        size_t n = Update.write(up.buf, up.currentSize);
        if (n != up.currentSize) {
          esp32LastLog += "Write short: ";
          esp32LastLog += String(n) + " of " + String(up.currentSize) + "\n";
        }
      }
    }
    else if (up.status == UPLOAD_FILE_END) {
      bool ok = false;
      if (!Update.hasError()) {
        ok = Update.end(true); // set as valid app
        if (!ok) {
          esp32LastLog += "Update.end failed: ";
          esp32LastLog += String(Update.errorString()) + "\n";
        } else {
          esp32LastLog += "Update.end OK\n";
          markAppValidIfPossible();
        }
      } else {
        esp32LastLog += "Update error before end: ";
        esp32LastLog += String(Update.errorString()) + "\n";
      }

      esp32LastSuccess = ok;
      esp32LastMillis = millis();
      Serial.println("[ESP32 OTA RESULT]\n" + esp32LastLog);
      ELOG(ok ? "ESP32 OTA: SUCCESS" : "ESP32 OTA: FAIL");

      // Persist the outcome so UI can read it even after reboot
      esp32WriteLastFile(ok, esp32LastLog);

      if (ok) {
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", "OK\n");
        delay(800);            // let TCP flush
        ESP.restart();         // reboot into new app
      } else {
        server.send(500, "text/plain", "ERR\n");
      }
    }
    else if (up.status == UPLOAD_FILE_ABORTED) {
      Update.abort();
      esp32LastLog += "Upload aborted\n";
      esp32LastSuccess = false;
      esp32WriteLastFile(false, esp32LastLog);
      server.send(500, "text/plain", "ERR\n");
    }
  }

  // ---------- Route setup ----------
  static void setupRoutes() {
    server.on("/",              HTTP_GET,  handleRoot);

    // Teensy
    server.on("/upload",        HTTP_POST, [](){}, handleUpload);
    server.on("/ping",          HTTP_GET,  handlePing);
    server.on("/version",       HTTP_GET,  handleVersion);
    server.on("/last",          HTTP_GET,  handleLast);
    server.on("/console",       HTTP_GET,  handleConsolePage);
    server.on("/tail",          HTTP_GET,  handleTail);
    server.on("/telemetry.bin", HTTP_GET,  handleTelemetryBin);
    server.on("/telemetry.json",HTTP_GET,  handleTelemetryJson);

    // ESP32
    server.on("/esp32-ota",     HTTP_POST, [](){}, handleEsp32Ota);
    server.on("/esp32-last",    HTTP_GET,  handleEsp32Last);
    server.on("/esp32-ping",    HTTP_GET,  handleEspPing);
    server.on("/esp32-version", HTTP_GET,  handleEspVersion);
    server.on("/esp32-console", HTTP_GET,  handleEsp32ConsolePage);
    server.on("/esp32-tail",    HTTP_GET,  handleEsp32Tail);

    // Diagnostics (NEW)
    server.on("/uart1-stats",   HTTP_GET,  handleUart1Stats);
    server.on("/i2c-scan",      HTTP_GET,  handleI2CScan);
    server.on("/as5600-dump",   HTTP_GET,  handleAS5600Dump);
  }

} // namespace (private)

// ---------- Public API impl ----------
void begin(const char* wifiSsid, const char* wifiPass, const char* otaToken,
           const char* mdnsHost) {
  HUB_WIFI_SSID = wifiSsid;
  HUB_WIFI_PASS = wifiPass;
  HUB_OTA_TOKEN = otaToken;

  Serial.begin(115200);
  delay(200);
  Serial.println("\n== ESP32 ⇄ Teensy OTA Hub + Console ==");
  ELOGF("Starting %s %s | Build %s", APP_NAME, APP_VER, APP_BUILD_DATE);

  ensureLittleFS();
  connectWifi(mdnsHost);

  // Start I2C early so diagnostics are ready
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  SerialTeensy.setRxBufferSize(4096);
  SerialTeensy.setTxBufferSize(1024);
  SerialTeensy.begin(230400, SERIAL_8N1, TEENSY_RX, TEENSY_TX);
  telemetryResetParser();
  Serial.printf("[UART] SerialTeensy on RX=%d TX=%d @230400\n", TEENSY_RX, TEENSY_TX);
  ELOGF("UART to Teensy: RX=%d TX=%d @230400", TEENSY_RX, TEENSY_TX);

  setupRoutes();
  server.begin();
  Serial.println("[HTTP] Server ready.");
  ELOG("HTTP server ready.");
}

void loop() {
  pumpTeensyConsole();
  server.handleClient();

  static unsigned long lastCheck = 0;
  if (millis()-lastCheck > 3000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Lost, rebooting");
      ELOG("WiFi lost — rebooting");
      delay(250); ESP.restart();
    }
  }
}

} // namespace OtaHub

#endif // ESP32_UPLOADER_ESP32_OTA_HUB_H
