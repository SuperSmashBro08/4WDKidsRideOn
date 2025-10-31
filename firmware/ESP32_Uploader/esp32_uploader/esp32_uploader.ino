/*
 * ESP32-S3 → Teensy 4.1 OTA uploader (robust)
 * -------------------------------------------
 * - Wiring: ESP32 GPIO43 (TX) -> Teensy RX2 pin 7
 *           ESP32 GPIO44 (RX) <- Teensy TX2 pin 8
 *           GND ↔ GND
 * - Browse to http://<esp32-ip>/ to upload the Teensy-exported Intel HEX (.hex).
 * - Speaks HELLO / BEGIN HEX / L <line> / END; verifies OK <n>; checks EOF.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include "secrets.h"   // #define WIFI_SSID "...", WIFI_PASS "...", OTA_TOKEN "..."

#define TEENSY_TX 43
#define TEENSY_RX 44
HardwareSerial SerialTeensy(1);

WebServer server(80);

// ---------- State for UI ----------
static String lastOtaLog;
static bool   lastOtaSuccess = false;
static unsigned long lastOtaMillis = 0;

// ---------- Small helpers ----------
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

static String buildIndexPage() {
  String page;
  page.reserve(2400);
  page += F(
  "<!doctype html><html><head><meta charset='utf-8'>"
  "<meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>ESP32 → Teensy OTA</title>"
  "<style>"
    "body{font-family:system-ui;margin:24px;max-width:760px}"
    "pre{background:#f6f8fa;padding:12px;border-radius:8px;overflow:auto}"
    "code{background:#f6f8fa;padding:2px 6px;border-radius:4px}"
    "#bar{width:100%;height:12px;background:#eee;border-radius:6px;overflow:hidden;margin:8px 0 4px 0;display:none}"
    "#fill{height:100%;width:0%}"
    ".ok{background:#2ecc71}.bad{background:#e74c3c}.info{background:#3498db}"
    "button{padding:8px 14px;border:0;border-radius:8px;background:#111;color:#fff;cursor:pointer}"
    "button[disabled]{opacity:.6;cursor:not-allowed}"
    "#status{min-height:1.2em}"
  "</style>"
  "</head><body><h2>ESP32 → Teensy OTA</h2>"
  "<form id='f' method='POST' action='/upload' enctype='multipart/form-data'>"
  "<p><b>Upload Teensy firmware</b> (<code>.hex</code> from <i>Export compiled Binary</i>)</p>"
  "<input id='file' type='file' name='firmware' accept='.hex' required>"
  "<p><button id='go' type='submit'>Upload & Flash</button></p>"
  "<div id='bar'><div id='fill' class='info'></div></div>"
  "<div id='status'></div>"
  "</form>"
  "<hr>"
  "<p>"
    "<button onclick=\"fetch('/ping').then(r=>r.text()).then(t=>alert(t)).catch(()=>alert('ping failed'))\">Ping Teensy</button> "
    "<button onclick=\"fetch('/version').then(r=>r.text()).then(t=>alert(t)).catch(()=>alert('version failed'))\">Get Version</button>"
  "</p>"
  );

  // Last result (uses your existing globals)
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

  // JS: progress + resilient finalize (checks /last on success/error/timeout)
  page += F(
  "<script>"
  "const f=document.getElementById('f');"
  "const btn=document.getElementById('go');"
  "const file=document.getElementById('file');"
  "const bar=document.getElementById('bar');"
  "const fill=document.getElementById('fill');"
  "const status=document.getElementById('status');"
  "function finalizeUI(ok){"
    "fill.className = ok ? 'ok' : 'bad';"
    "status.textContent = ok ? 'Flashing complete. Returning to home…' : 'Upload failed.';"
    "setTimeout(()=>{ window.location='/'; }, 900);"
  "}"
  "async function checkLastAndFinalize(){"
    "try{"
      "const r = await fetch('/last',{cache:'no-store'});"
      "const t = await r.text();"
      "finalizeUI(t.trim()==='OK');"
    "}catch(e){"
      "finalizeUI(false);"
    "}"
  "}"
  "f.addEventListener('submit',e=>{"
    "e.preventDefault();"
    "if(!file.files.length){alert('Choose a .hex first');return;}"
    "btn.disabled=true;"
    "status.textContent='Uploading…';"
    "bar.style.display='block';"
    "fill.style.width='0%';"
    "fill.className='info';"
    "const xhr=new XMLHttpRequest();"
    "xhr.open('POST','/upload');"
    "xhr.timeout = 120000;"
    "xhr.upload.onprogress=(ev)=>{"
      "if(ev.lengthComputable){"
        "const pct=Math.round((ev.loaded/ev.total)*100);"
        "fill.style.width=pct+'%';"
      "}else{fill.style.width='100%';}"
    "};"
    "xhr.onload=()=>{ checkLastAndFinalize(); };"
    "xhr.onerror=()=>{ checkLastAndFinalize(); };"
    "xhr.ontimeout=()=>{ checkLastAndFinalize(); };"
    "const fd=new FormData(f);"
    "xhr.send(fd);"
  "});"
  "</script>"
  "</body></html>");
  return page;
}

static String buildResultPage(bool success, const String& log) {
  String page;
  page.reserve(1200 + log.length());
  page += F("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>ESP32 → Teensy OTA result</title></head><body>");
  page += success ? F("<h2 style='color:#0a0'>OTA upload complete</h2>")
                  : F("<h2 style='color:#b00'>OTA upload failed</h2>");
  page += F("<p><a href='/'>Back</a></p><pre>");
  page += htmlEscape(log);
  page += F("</pre></body></html>");
  return page;
}

static void ensureLittleFS() {
  if (!LittleFS.begin(true)) Serial.println("[FS] LittleFS mount failed");
  else                       Serial.println("[FS] LittleFS mounted");
}

static void connectWifi() {
  Serial.println("\n[WiFi] Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-teensy");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250); Serial.print('.');
    if (millis() - t0 > 25000) { Serial.println("\n[WiFi] Failed. Rebooting."); delay(500); ESP.restart(); }
  }
  Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
  if (MDNS.begin("esp32-teensy")) { MDNS.addService("http","tcp",80); Serial.println("[mDNS] esp32-teensy.local ready"); }
  else Serial.println("[mDNS] start failed");
}

// Read a line from Teensy (CRLF tolerant); trims trailing spaces
static bool readLineFrom(Stream& s, String& out, uint32_t to_ms=3000) {
  out = ""; unsigned long t0 = millis();
  while (millis()-t0 < to_ms) {
    while (s.available()) {
      char c = (char)s.read();
      if (c=='\r') continue;
      if (c=='\n') { out.trim(); return true; }
      if (out.length() < 240) out += c;
    }
    delay(1); yield();
  }
  out.trim();
  return out.length() > 0;
}

static bool expectStartsWith(Stream& s, const char* prefix, String* captured=nullptr, uint32_t to_ms=3000) {
  String line; if (!readLineFrom(s, line, to_ms)) return false;
  Serial.print("<- "); Serial.println(line);
  if (captured) *captured = line;
  return line.startsWith(prefix);
}

// Validate file looks like Intel HEX and ends with EOF
static bool validateHexFile(File& f, String& err) {
  if (!f) { err="open failed"; return false; }
  if (!f.size()) { err="empty file"; return false; }

  // First non-empty line must start with ':'
  size_t pos = f.position();
  String first = f.readStringUntil('\n'); first.trim();
  while (first.length()==0 && f.available()) { first = f.readStringUntil('\n'); first.trim(); }
  if (first.length()==0 || first[0] != ':') { err="not Intel HEX (no leading colon)"; f.seek(pos); return false; }

  // Scan to last non-empty line for EOF check
  f.seek(0);
  String line, lastNonEmpty;
  while (f.available()) {
    line = f.readStringUntil('\n'); line.trim();
    if (line.length()) lastNonEmpty = line;
  }
  f.seek(0);
  if (lastNonEmpty != ":00000001FF") { err = "missing EOF (:00000001FF)"; return false; }
  return true;
}

// Talk to Teensy & flash from a .hex stored on LittleFS
static bool flashTeensyFromFS(const char* path, String& logOut) {
  File f = LittleFS.open(path, FILE_READ);
  String vErr;
  if (!validateHexFile(f, vErr)) {
    logOut += "HEX validate error: " + vErr + "\n";
    f.close(); return false;
  }

  // stats for UI
  const size_t totalBytes = f.size();
  size_t sentBytes = 0;
  auto logProgress = [&](size_t extra){ sentBytes += extra; if (totalBytes) {
      int pct = (int)( (100ULL * sentBytes) / totalBytes );
      logOut += "PROGRESS " + String(pct) + "% (" + String(sentBytes) + "/" + String(totalBytes) + ")\n";
    }};

  // clear any stale UART
  while (SerialTeensy.available()) SerialTeensy.read();

  // HELLO (token)
  SerialTeensy.printf("HELLO %s\r\n", OTA_TOKEN);
  logOut += "Sent HELLO\n";

  // READY (handle BUSY + retry)
  String resp;
  bool helloOk = false;
  for (uint8_t attempt=0; attempt<3; ++attempt) {
    if (!readLineFrom(SerialTeensy, resp, 2500)) { logOut += "HELLO timeout\n"; continue; }
    logOut += "<- " + resp + "\n";
    if (resp == "READY") { helloOk = true; break; }
    if (resp == "BUSY")  { delay(300); continue; }
    if (resp == "NACK")  { logOut += "Token mismatch (NACK)\n"; break; }
    logOut += "Unexpected after HELLO: " + resp + "\n";
  }
  if (!helloOk) { f.close(); return false; }

  // BEGIN
  SerialTeensy.print("BEGIN HEX\r\n");
  if (!expectStartsWith(SerialTeensy, "HEX BEGIN", &resp, 1500)) {
    logOut += "No HEX BEGIN (got: " + resp + ")\n";
    f.close(); return false;
  }
  logOut += "<- " + resp + "\n";

  // Stream lines
  f.seek(0);
  size_t lineNumber = 0, ok=0, bad=0;
  const uint16_t lineDelayUs = 200;  // gentle pacing; raise if needed
  while (f.available()) {
    String rec = f.readStringUntil('\n'); rec.trim();
    if (!rec.length()) continue;
    lineNumber++;

    SerialTeensy.print("L "); SerialTeensy.print(rec); SerialTeensy.print("\r\n");

    String ack;
    if (!readLineFrom(SerialTeensy, ack, 1500)) {
      logOut += "Line " + String(lineNumber) + " timeout\n";
      bad++; break;
    }
    logOut += "<- " + ack + "\n";

    if (ack.startsWith("OK ")) {
      // (optional) verify line number matches Teensy reply
      uint32_t n = ack.substring(3).toInt();
      if (n != lineNumber) logOut += "WARN: OK index mismatch (got " + String(n) + ")\n";
      ok++;
    } else if (ack.startsWith("BAD ")) {
      bad++;
      logOut += "Teensy reported BAD at line " + String(lineNumber) + "\n";
      break;
    } else {
      bad++;
      logOut += "Unexpected reply at line " + String(lineNumber) + ": " + ack + "\n";
      break;
    }

    logProgress(rec.length()+3); // rough progress
    delayMicroseconds(lineDelayUs);
    yield();
  }
  f.close();

  // END
  SerialTeensy.print("END\r\n");
  logOut += "Sent lines: " + String(lineNumber) + " (OK=" + String(ok) + " BAD=" + String(bad) + ")\n";

  bool success = false;
  unsigned long t0 = millis();
  while (millis()-t0 < 6000) { // Teensy may take a moment before reboot
    String summary;
    if (!readLineFrom(SerialTeensy, summary, 500)) { yield(); continue; }
    logOut += summary + "\n";
    if (summary.startsWith("HEX OK")) success = true;
    if (summary=="APPLIED" || summary.startsWith("HEX ERR")) break;
  }

  return success && (bad==0);
}

// ---------- HTTP handlers ----------
static void handleRoot() { server.send(200, "text/html", buildIndexPage()); }

static void handlePing() {
  // simple sanity ping
  while (SerialTeensy.available()) SerialTeensy.read();
  SerialTeensy.print("PING\r\n");
  String resp;
  if (readLineFrom(SerialTeensy, resp, 800)) server.send(200, "text/plain", resp);
  else server.send(504, "text/plain", "timeout");
}

static void handleVersion() {
  while (SerialTeensy.available()) SerialTeensy.read();
  SerialTeensy.print("VERSION\r\n");
  String a,b; String out;
  if (readLineFrom(SerialTeensy, a, 800)) out += a + "\n";
  if (readLineFrom(SerialTeensy, b, 200)) out += b + "\n";
  server.send(out.length()?200:504, "text/plain", out.length()?out:"timeout");
}

// NEW: return OK/ERR for the last OTA result (used by the web UI to finalize)
static void handleLast() {
  server.send(200, "text/plain", lastOtaSuccess ? "OK" : "ERR");
}

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

    Serial.println("[OTA RESULT]\n" + log);

    // We can return a tiny payload — the JS will check /last anyway.
    server.send(ok ? 200 : 500, "text/plain", ok ? "OK\n" : "ERR\n");
  }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== ESP32 → Teensy OTA ==");

  ensureLittleFS();
  connectWifi();

  SerialTeensy.setRxBufferSize(4096);
  SerialTeensy.setTxBufferSize(1024);
  SerialTeensy.begin(115200, SERIAL_8N1, TEENSY_RX, TEENSY_TX);
  Serial.printf("[UART] SerialTeensy on RX=%d TX=%d @115200\n", TEENSY_RX, TEENSY_TX);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/upload", HTTP_POST, [](){}, handleUpload);
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/version", HTTP_GET, handleVersion);
  server.on("/last", HTTP_GET, handleLast);   // <— new
  server.begin();
  Serial.println("[HTTP] Server ready.");
}

void loop() {
  server.handleClient();
  static unsigned long lastCheck = 0;
  if (millis()-lastCheck > 3000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) { Serial.println("[WiFi] Lost, rebooting"); delay(250); ESP.restart(); }
  }
}
