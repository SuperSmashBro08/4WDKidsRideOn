/*
 * ESP32-S3 Teensy OTA uploader (simplified)
 * -----------------------------------------
 * Wiring: ESP32 GPIO43 (TX) -> Teensy RX2 pin 7, GPIO44 (RX) <- Teensy TX2 pin 8, shared GND.
 * Usage: connect ESP32 to Wi-Fi, browse to http://<esp32-ip>/ and upload the Teensy-exported
 *        Intel HEX (.hex). The ESP32 performs HELLO / BEGIN HEX / L <line> / END handshakes
 *        with the Teensy and reports the summary on the web page and Serial monitor.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, OTA_TOKEN

#define TEENSY_TX 43
#define TEENSY_RX 44
HardwareSerial SerialTeensy(1);

WebServer server(80);

static String lastOtaLog;
static bool   lastOtaSuccess = false;
static unsigned long lastOtaMillis = 0;

static String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in.charAt(i);
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;");  break;
      case '>': out += F("&gt;");  break;
      default:  out += c;         break;
    }
  }
  return out;
}

static String buildIndexPage() {
  String page;
  page.reserve(1024);
  page += F("<!doctype html><html><head><meta charset='utf-8'>");
  page += F("<meta name=viewport content='width=device-width,initial-scale=1'>");
  page += F("<title>ESP32 → Teensy OTA</title>");
  page += F("<style>body{font-family:system-ui;margin:24px;max-width:720px}pre{background:#f6f8fa;padding:12px;border-radius:8px;overflow-x:auto}</style>");
  page += F("</head><body><h2>ESP32 → Teensy OTA</h2>");
  page += F("<form method='POST' action='/upload' enctype='multipart/form-data'>");
  page += F("<p><b>Upload Teensy firmware</b> (<code>.hex</code> from <i>Export compiled Binary</i>)</p>");
  page += F("<input type='file' name='firmware' accept='.hex' required>");
  page += F("<p><button type='submit'>Upload & Flash</button></p></form>");
  if (lastOtaLog.length()) {
    page += lastOtaSuccess ? F("<h3 style='color:#0a0'>Last OTA: success</h3>")
                           : F("<h3 style='color:#b00'>Last OTA: failed</h3>");
    if (lastOtaMillis) {
      page += F("<p><small>Completed ");
      page += String((millis() - lastOtaMillis) / 1000);
      page += F(" s ago</small></p>");
    }
    page += F("<pre>");
    page += htmlEscape(lastOtaLog);
    page += F("</pre>");
  }
  page += F("</body></html>");
  return page;
}

static String buildResultPage(bool success, const String& log) {
  String page;
  page.reserve(1024 + log.length());
  page += F("<!doctype html><html><head><meta charset='utf-8'>");
  page += F("<meta name=viewport content='width=device-width,initial-scale=1'>");
  page += F("<title>ESP32 → Teensy OTA result</title></head><body>");
  page += success ? F("<h2 style='color:#0a0'>OTA upload complete</h2>")
                  : F("<h2 style='color:#b00'>OTA upload failed</h2>");
  page += F("<p><a href='/'>Back</a></p><pre>");
  page += htmlEscape(log);
  page += F("</pre></body></html>");
  return page;
}

static void ensureLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount failed");
  } else {
    Serial.println("[FS] LittleFS mounted");
  }
}

static void connectWifi() {
  Serial.println("\n[WiFi] Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-teensy");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    if (millis() - t0 > 25000) {
      Serial.println("\n[WiFi] Failed. Rebooting.");
      delay(500);
      ESP.restart();
    }
  }

  Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin("esp32-teensy")) {
    Serial.println("[mDNS] esp32-teensy.local ready");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("[mDNS] start failed");
  }
}

static String readLineFromTeensy(uint32_t wait_ms) {
  String line;
  line.reserve(96);
  unsigned long start = millis();
  while (millis() - start < wait_ms) {
    while (SerialTeensy.available()) {
      char c = (char)SerialTeensy.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line.trim();
        return line;
      }
      if (line.length() < 200) line += c;
    }
    delay(1);
    yield();
  }
  line.trim();
  return line;
}

static bool flashTeensyFromFS(const char* path, String& logOut) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) {
    logOut += String("ERR: cannot open ") + path + "\n";
    return false;
  }
  if (!f.size()) {
    logOut += String("ERR: ") + path + " is empty\n";
    f.close();
    return false;
  }

  while (SerialTeensy.available()) SerialTeensy.read();

  SerialTeensy.printf("HELLO %s\r\n", OTA_TOKEN);
  logOut += "Sent HELLO\n";

  String resp;
  const uint8_t maxHelloAttempts = 2;
  bool helloOk = false;
  for (uint8_t attempt = 0; attempt < maxHelloAttempts; ++attempt) {
    resp = readLineFromTeensy(2000);
    if (resp == "READY") {
      helloOk = true;
      break;
    }
    if (resp == "BUSY") {
      logOut += "Teensy reported BUSY, waiting...\n";
      delay(250);
      continue;
    }
    if (resp == "NACK") {
      logOut += "HELLO rejected (token mismatch).\n";
      break;
    }
    if (resp.length() == 0) {
      logOut += "HELLO timeout, retrying...\n";
      continue;
    }
    logOut += String("HELLO unexpected: ") + resp + "\n";
    break;
  }

  if (!helloOk) {
    f.close();
    return false;
  }
  logOut += "READY\n";

  SerialTeensy.print("BEGIN HEX\r\n");
  resp = readLineFromTeensy(1000);
  if (resp.length()) {
    logOut += resp + "\n";
  }

  size_t lineNumber = 0;
  while (f.available()) {
    String rec = f.readStringUntil('\n');
    rec.trim();
    if (!rec.length()) continue;

    SerialTeensy.print("L ");
    SerialTeensy.print(rec);
    SerialTeensy.print("\r\n");
    lineNumber++;

    String ack = readLineFromTeensy(600);
    if (ack.startsWith("OK ")) {
      continue;
    }
    if (ack.startsWith("BAD ")) {
      logOut += String("Line ") + lineNumber + " reported BAD\n";
    } else {
      logOut += String("Timeout/Unexpected reply at line ") + lineNumber + ": " + (ack.length() ? ack : String("<timeout>")) + "\n";
    }
    f.close();
    SerialTeensy.print("END\r\n");
    return false;
  }
  f.close();

  SerialTeensy.print("END\r\n");
  logOut += String("Sent lines: ") + lineNumber + "\n";

  bool success = false;
  unsigned long start = millis();
  while (millis() - start < 2500) {
    String summary = readLineFromTeensy(300);
    if (!summary.length()) continue;
    logOut += summary + "\n";
    if (summary.indexOf("HEX OK") >= 0) success = true;
    if (summary == "APPLIED" || summary.startsWith("HEX ERR")) break;
  }

  return success;
}

static void handleRoot() { server.send(200, "text/html", buildIndexPage()); }

static File uploadFile;
static bool uploadedWasHex = false;
static String uploadName;

static void handleUpload() {
  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    uploadName = up.filename;
    uploadedWasHex = uploadName.endsWith(".hex") || uploadName.endsWith(".HEX");
    const char* saveName = "/teensy41.hex";
    Serial.printf("[HTTP] Upload start: %s\n", up.filename.c_str());
    uploadFile = LittleFS.open(saveName, FILE_WRITE);
    if (!uploadFile) {
      Serial.println("[HTTP] Failed to open file for writing");
    }
  }
  else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(up.buf, up.currentSize);
  }
  else if (up.status == UPLOAD_FILE_END) {
    if (!uploadFile) {
      server.send(500, "text/plain", "Upload failed to open file");
      return;
    }

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

    Serial.println("[OTA RESULT]\n" + log);

    server.send(ok ? 200 : 500, "text/html", buildResultPage(ok, log));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== ESP32 → Teensy OTA (simplified) ==");

  ensureLittleFS();
  connectWifi();

  SerialTeensy.setRxBufferSize(4096);
  SerialTeensy.setTxBufferSize(1024);
  SerialTeensy.begin(115200, SERIAL_8N1, TEENSY_RX, TEENSY_TX);
  Serial.printf("[UART] SerialTeensy on RX=%d TX=%d @115200\n", TEENSY_RX, TEENSY_TX);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/upload", HTTP_POST, [](){}, handleUpload);
  server.begin();
  Serial.println("[HTTP] Server ready.");
}

void loop() {
  server.handleClient();

  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 3000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connection lost, rebooting");
      delay(250);
      ESP.restart();
    }
  }
}
