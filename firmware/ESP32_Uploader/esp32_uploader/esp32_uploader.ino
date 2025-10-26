/*
 * ESP32-S3 OTA uploader + transparent Telnet ↔ Teensy Serial2 bridge
 * ------------------------------------------------------------------
 * Wiring: ESP32 GPIO43 (TX) → Teensy RX2 pin 7, GPIO44 (RX) ← Teensy TX2 pin 8, common GND.
 * Telnet:  `telnet <esp32-ip> 2323` (up to 3 clients share the same raw UART stream).
 * Quick test: connect via Telnet, type `VERSION` and expect the Teensy reply; run `POT ON`
 *            then move the potentiometer to see live `POT:` lines; send `POT OFF` to stop.
 * OTA: open the ESP32 web page, upload a Teensy `.hex`, wait for `HEX OK`/`APPLIED`; the
 *      Telnet bridge pauses automatically during OTA and resumes afterwards.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, OTA_TOKEN

// ===== Teensy UART on ESP32-S3 DevKitC-1 =====
// TX -> GPIO43 (to Teensy RX2 pin 7)
// RX -> GPIO44 (from Teensy TX2 pin 8)
#define TEENSY_TX 43
#define TEENSY_RX 44
HardwareSerial SerialTeensy(1);

// ===== Web + Telnet =====
WebServer server(80);
#define TELNET_PORT 2323
#define MAX_TELNET_CLIENTS 3
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClients[MAX_TELNET_CLIENTS];

// Pauses Telnet bridge while OTA owns the UART
volatile bool otaActive = false;

// ---------------- HTML ----------------
static const char* PAGE_INDEX =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name=viewport content='width=device-width,initial-scale=1'/>"
"<title>ESP32 → Teensy OTA</title>"
"<style>body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:24px;max-width:780px}"
"code{background:#f6f8fa;padding:.2em .4em;border-radius:4px}"
"button{padding:.6em 1em;border-radius:8px;border:1px solid #888;background:#fafafa;cursor:pointer}"
"a{color:#06c;text-decoration:none}a:hover{text-decoration:underline}"
".card{border:1px solid #ddd;border-radius:12px;padding:16px;margin-bottom:16px}"
"</style></head><body>"
"<h2>ESP32 → Teensy OTA</h2>"
"<div class=card>"
"<form method='POST' action='/upload' enctype='multipart/form-data'>"
"<p><b>Upload Teensy firmware</b> (<code>.hex</code> preferred)</p>"
"<input type='file' name='firmware' accept='.hex,.bin' required>"
"<button type='submit'>Upload & Flash</button></form>"
"</div>"
"<div class=card>"
"<p>Telnet monitor: <b>telnet</b> <code>ESP32-IP</code> <b>2323</b></p>"
"<p>mDNS (if supported): <code>telnet esp32-teensy.local 2323</code></p>"
"</div>"
"<div class=card>"
"<p><a href='/ls'>List LittleFS</a> · <a href='/health'>Health</a> · <a href='/reboot'>Reboot ESP32</a></p>"
"</div>"
"</body></html>";

// ---------------- FS ----------------
static void ensureLittleFS() {
  if (!LittleFS.begin(true)) Serial.println("[FS] LittleFS mount failed.");
  else                       Serial.println("[FS] LittleFS mounted.");
}

static void handleRoot() { server.send(200, "text/html", PAGE_INDEX); }

static void handleListFS() {
  String html = "<h3>LittleFS contents</h3><pre>";
  File root = LittleFS.open("/");
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    html += String(f.name()) + "  " + String((unsigned)f.size()) + " bytes\n";
  }
  html += "</pre><p><a href='/'>Back</a></p>";
  server.send(200, "text/html", html);
}

static void handleHealth() {
  String s;
  s.reserve(256);
  s += "WiFi: " + WiFi.SSID() + "\n";
  s += "IP: " + WiFi.localIP().toString() + "\n";
  s += "Telnet: port " + String(TELNET_PORT) + " (clients ";
  int n=0; for (int i=0;i<MAX_TELNET_CLIENTS;i++) if (telnetClients[i] && telnetClients[i].connected()) n++;
  s += String(n) + ")\n";
  s += "OTA active: " + String(otaActive ? "yes" : "no") + "\n";
  server.send(200, "text/plain", s);
}

static void handleReboot() {
  server.send(200, "text/plain", "ESP32 rebooting in 0.5s...");
  delay(500);
  ESP.restart();
}

// ---------------- Wi-Fi ----------------
static void connectWifi() {
  Serial.println("\n[WiFi] Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-teensy");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250); Serial.print('.');
    if (millis() - t0 > 25000) {
      Serial.println("\n[WiFi] Failed. Rebooting.");
      delay(500);
      ESP.restart();
    }
  }
  Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin("esp32-teensy")) {
    MDNS.addService("telnet", "tcp", TELNET_PORT);
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] esp32-teensy.local started");
  } else {
    Serial.println("[mDNS] start failed (continuing)");
  }
}

// ---------------- Teensy HELLO handshake ----------------
static String readLineFromTeensy(uint32_t wait_ms) {
  unsigned long t0 = millis();
  String ln; ln.reserve(96);
  while (millis() - t0 < wait_ms) {
    while (SerialTeensy.available()) {
      char c = (char)SerialTeensy.read();
      if (c == '\r') continue;
      if (c != '\n') { ln += c; continue; }
      ln.trim();
      if (ln.length()) return ln;
      ln = "";
    }
    delay(1);
    yield();
  }
  return String();
}

static String sendHelloAndWait(uint32_t timeout_ms = 2000) {
  while (SerialTeensy.available()) SerialTeensy.read();
  SerialTeensy.print("HELLO "); SerialTeensy.print(OTA_TOKEN); SerialTeensy.print("\n");
  Serial.println("[UART] Sent: HELLO <token>");

  String log, line; unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    line = readLineFromTeensy(50);
    if (line.length()) {
      Serial.printf("[UART] <- %s\n", line.c_str());
      log += line + "\n";
      if (line == "READY" || line == "NACK" || line == "DONE") break;
    }
  }
  return log.length() ? log : String("No response within timeout\n");
}

// ---------------- HEX streaming with per-line OK/BAD + gentle pacing ----------------
static String streamHexFromFS(const char* path = "/teensy41.hex") {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return String("ERR: ") + path + " not found\n";
  const size_t sz = f.size();
  if (sz < 1000) { f.close(); return String("ERR: File too small (") + sz + " bytes)\n"; }

  otaActive = true;  // pause Telnet bridge

  // Clear UART rx
  while (SerialTeensy.available()) SerialTeensy.read();

  // Begin session
  SerialTeensy.print("BEGIN HEX\n");
  Serial.println("[OTA] BEGIN HEX sent");

  size_t lines = 0, chars = 0;
  uint32_t lastProgressMs = millis();
  String respLog;

  // We’ll read by lines, trimming CR/LF
  String hex; hex.reserve(128);

  while (f.available()) {
    hex = f.readStringUntil('\n'); hex.trim();
    if (!hex.length()) continue;

    lines++; chars += hex.length();

    // Send one record line
    SerialTeensy.print("L "); SerialTeensy.print(hex); SerialTeensy.print("\n");

    // Wait for OK/BAD (fast path first)
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      String r = readLineFromTeensy(25);  // Teensy should ack quickly
      if      (r.startsWith("OK "))  { ok = true; }
      else if (r.startsWith("BAD ")) {
        Serial.printf("[OTA] %s — resend line %u (attempt %d)\n", r.c_str(), (unsigned)lines, attempt+1);
        SerialTeensy.print("L "); SerialTeensy.print(hex); SerialTeensy.print("\n");
      } else if (r.length() == 0) {
        // Timeout—resend the line (rare on USB-serial bridges)
        Serial.println("[OTA] No per-line reply; retrying");
        SerialTeensy.print("L "); SerialTeensy.print(hex); SerialTeensy.print("\n");
      } else {
        // Could be progress/ACK; peek one more small window
        String r2 = readLineFromTeensy(15);
        if      (r2.startsWith("OK "))  ok = true;
        else if (r2.startsWith("BAD ")) {
          Serial.printf("[OTA] %s — late BAD, resend line %u\n", r2.c_str(), (unsigned)lines);
          SerialTeensy.print("L "); SerialTeensy.print(hex); SerialTeensy.print("\n");
        }
      }
      // Keep Wi-Fi stack happy
      delay(0); yield();
    }

    if (!ok) {
      f.close();
      SerialTeensy.print("END\n");
      otaActive = false;
      return String("Aborted: line ") + lines + " did not get OK after retries\n";
    }

    // Light progress every ~250 ms to avoid chatty logs
    if (millis() - lastProgressMs > 250) {
      lastProgressMs = millis();
      String ack = readLineFromTeensy(5);
      if (ack.length()) { respLog += ack + "\n"; Serial.printf("[UART] <- %s\n", ack.c_str()); }
    }
  }
  f.close();

  SerialTeensy.print("END\n");

  // Gather summary
  unsigned long t0 = millis(); String resp, part;
  while (millis() - t0 < 1200) {
    while (SerialTeensy.available()) {
      char c = (char)SerialTeensy.read();
      if (c == '\r') continue;
      if (c != '\n') { part += c; continue; }
      part.trim(); if (part.length()) { resp += part + "\n"; }
      part = "";
    }
    delay(5); yield();
  }
  if (!resp.length()) resp = "No reply\n";
  resp += respLog;
  resp += "Sent lines=" + String(lines) + " approxChars=" + String(chars) + "\n";

  otaActive = false;
  return resp;
}

// ---------------- Upload handler: save, then flash immediately ----------------
File uploadFile;
String lastUploadName;
bool uploadedWasHex = false;

static void handleUpload() {
  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    lastUploadName = up.filename;
    uploadedWasHex = lastUploadName.endsWith(".hex") || lastUploadName.endsWith(".HEX");
    const char* saveAs = uploadedWasHex ? "/teensy41.hex" : "/teensy41.bin";
    Serial.printf("[HTTP] Upload start: %s -> %s\n", up.filename.c_str(), saveAs);
    uploadFile = LittleFS.open(saveAs, FILE_WRITE);
    if (!uploadFile) Serial.println("[HTTP] Cannot open target file for write!");
  }
  else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(up.buf, up.currentSize);
  }
  else if (up.status == UPLOAD_FILE_END) {
    if (!uploadFile) { server.send(500, "text/plain", "Upload failed to open file."); return; }

    uploadFile.close();
    Serial.printf("[HTTP] Upload complete: %s bytes=%u\n", lastUploadName.c_str(), up.totalSize);

    otaActive = true;  // pause Telnet bridge for HELLO + flashing

    // 1) HELLO
    String hello = sendHelloAndWait(2500);

    // 2) Stream HEX now (.bin just stored)
    String result;
    if (uploadedWasHex) result = streamHexFromFS("/teensy41.hex");
    else                result = "BIN saved as /teensy41.bin — Teensy stub expects .hex.\n";

    if (!uploadedWasHex) otaActive = false;  // normally cleared by streamHex

    Serial.println("[RESULT]\n" + hello + result);

    // 3) Redirect back to "/"
    server.sendHeader("Location", "/");
    server.send(303);
  }
}

// ---------------- TELNET BRIDGE (transparent) ----------------
static void telnetAcceptClients() {
  if (!telnetServer.hasClient()) return;
  WiFiClient newClient = telnetServer.available();
  if (!newClient) return;

  int slot = -1;
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (!telnetClients[i] || !telnetClients[i].connected()) { slot = i; break; }
  }
  if (slot >= 0) {
    telnetClients[slot].stop();
    telnetClients[slot] = newClient;
  } else {
    newClient.println("[ESP32] Too many telnet clients.");
    newClient.stop();
  }
}

static void telnetBroadcastFromTeensy() {
  if (otaActive) return;               // OTA owns UART
  int avail = SerialTeensy.available();
  if (!avail) return;

  uint8_t buf[512];
  int n = SerialTeensy.readBytes(buf, min(avail, (int)sizeof(buf)));
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (telnetClients[i] && telnetClients[i].connected()) {
      telnetClients[i].write(buf, n);  // raw bytes out
    }
  }
}

static void telnetPipeToTeensy() {
  if (otaActive) {
    // Drain (discard) so OTA isn't interfered by stray keystrokes
    for (int i = 0; i < MAX_TELNET_CLIENTS; i++)
      while (telnetClients[i] && telnetClients[i].connected() && telnetClients[i].available())
        telnetClients[i].read();
    return;
  }
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (!telnetClients[i] || !telnetClients[i].connected()) continue;
    while (telnetClients[i].available()) {
      int c = telnetClients[i].read();
      if (c == '\r') continue;        // normalize CRLF
      SerialTeensy.write((uint8_t)c); // raw to Teensy
    }
  }
}

static void telnetCleanup() {
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (telnetClients[i] && !telnetClients[i].connected()) {
      telnetClients[i].stop();
    }
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200); delay(150);
  Serial.println("\n== ESP32 OTA Uploader + Telnet ==");
  ensureLittleFS();
  connectWifi();

  // Bigger UART buffers help during bursts
  SerialTeensy.setRxBufferSize(4096);
  SerialTeensy.setTxBufferSize(1024);
  SerialTeensy.begin(115200, SERIAL_8N1, TEENSY_RX, TEENSY_TX);
  Serial.printf("[UART] SerialTeensy on RX=%d TX=%d @115200\n", TEENSY_RX, TEENSY_TX);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/ls", HTTP_GET, handleListFS);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/reboot", HTTP_GET, handleReboot);
  server.on("/upload", HTTP_POST, [](){}, handleUpload);
  server.begin();
  Serial.println("[HTTP] Server started.");

  telnetServer.begin(TELNET_PORT);
  telnetServer.setNoDelay(true);
  Serial.printf("[TELNET] Listening on %d\n", TELNET_PORT);
}

void loop() {
  // Keep services responsive even during heavy UART traffic
  server.handleClient();
  telnetAcceptClients();
  telnetBroadcastFromTeensy();
  telnetPipeToTeensy();
  telnetCleanup();

  // Lightweight Wi-Fi watchdog (auto-reconnect)
  static uint32_t lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 3000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Lost connection; rebooting for clean recover.");
      delay(250);
      ESP.restart();
    }
  }
}
