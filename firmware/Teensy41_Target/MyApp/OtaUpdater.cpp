#include "OtaUpdater.h"
#include "FXUtil.h"
extern "C" { #include "FlashTxx.h" }

static HardwareSerial* _ota = &Serial2;
static uint32_t        _baud = 115200;

// Must match your ESP32 + FlasherX token
static constexpr char  OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";

// What we return to VERSION (human-readable)
static const char*     LOADER_ID = "FlasherX v2.4 (in-app)";

struct OtaSession {
  bool handshakeReady;
  bool inProgress;
  bool fatal;
  uint32_t bufferAddr;
  uint32_t bufferSize;
  uint32_t bytes;
  uint32_t okLines;
  uint32_t badLines;
  char     lastError[96];
  hex_info_t hex;
  char     hexData[32] __attribute__((aligned(8)));
};

static OtaSession ota;

static void resetOtaSession() {
  ota.handshakeReady = false;
  ota.inProgress     = false;
  ota.fatal          = false;
  ota.bufferAddr     = 0;
  ota.bufferSize     = 0;
  ota.bytes          = 0;
  ota.okLines        = 0;
  ota.badLines       = 0;
  ota.lastError[0]   = 0;
  hex_info_reset(&ota.hex, ota.hexData);
}

static inline void sendLine(const char* s) {
  _ota->print(s);
  _ota->print("\r\n");
}

static void handleLine(const char* line);
static void beginHex();
static void handleHexRecord(const char* rec);
static void endHex();

void OtaUpdater::begin(HardwareSerial& otaPort, uint32_t baud) {
  _ota  = &otaPort;
  _baud = baud ? baud : 115200;
  _ota->begin(_baud);
  resetOtaSession();
}

bool OtaUpdater::inProgress() { return ota.inProgress; }

const char* OtaUpdater::loaderId() { return LOADER_ID; }

void OtaUpdater::tick() {
  static char buf[192];
  static size_t n = 0;

  while (_ota->available()) {
    char c = (char)_ota->read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = 0;
      if (n) handleLine(buf);
      n = 0;
    } else if (n + 1 < sizeof(buf)) {
      buf[n++] = c;
    }
  }
}

static void handleLine(const char* line) {
  if (!line || !line[0]) return;

  // Data line
  if (line[0] == 'L' && line[1] == ' ') {
    handleHexRecord(line + 2);
    return;
  }

  // Commands
  if (!strncmp(line, "HELLO", 5)) {
    if (ota.inProgress) { sendLine("BUSY"); return; }
    const char* tok = line + 5;
    while (*tok == ' ') tok++;
    if (!*tok) { sendLine("NACK"); return; }
    if (!strcmp(tok, OTA_TOKEN)) { ota.handshakeReady = true; sendLine("READY"); }
    else { ota.handshakeReady = false; sendLine("NACK"); }
    return;
  }

  if (!strcmp(line, "BEGIN HEX")) { beginHex(); return; }
  if (!strcmp(line, "END"))       { endHex();   return; }
  if (!strcmp(line, "PING"))      { sendLine("PONG"); return; }

  // VERSION: keep compatible with your ESP32 UI
  if (!strcmp(line, "VERSION")) {
    _ota->print("FW MyApp\r\n");
    _ota->print("FLASHERX "); _ota->print(LOADER_ID); _ota->print("\r\n");
    return;
  }

  sendLine("ERR");
}

static void beginHex() {
  if (!ota.handshakeReady || ota.inProgress) { sendLine("HEX IDLE"); return; }

  uint32_t addr = 0, size = 0;
  if (firmware_buffer_init(&addr, &size) == 0) {
    sendLine("HEX FAIL");
    return;
  }

  ota.bufferAddr = addr;
  ota.bufferSize = size;
  ota.inProgress = true;
  ota.fatal      = false;
  ota.bytes      = 0;
  ota.okLines    = 0;
  ota.badLines   = 0;
  ota.lastError[0] = 0;
  hex_info_reset(&ota.hex, ota.hexData);

  sendLine("HEX BEGIN");
}

static void handleHexRecord(const char* rec) {
  if (!ota.inProgress) { sendLine("HEX IDLE"); return; }

  ota.hex.lines++;
  unsigned int lineNo = ota.hex.lines;
  bool ok = !ota.fatal;

  if (ok && (parse_hex_line(rec, ota.hex.data, &ota.hex.addr, &ota.hex.num, &ota.hex.code) == 0)) {
    ok = false;
  }
  if (ok && (process_hex_record(&ota.hex) != 0)) {
    ok = false;
  }

  if (ok && ota.hex.code == 0) {
    if (ota.hex.max > (FLASH_BASE_ADDR + ota.bufferSize)) {
      ok = false;
    } else {
      uint32_t addr = ota.bufferAddr + ota.hex.base + ota.hex.addr - FLASH_BASE_ADDR;
      if (!IN_FLASH(ota.bufferAddr)) {
        memcpy((void*)addr, (void*)ota.hex.data, ota.hex.num);
      } else {
        int e = flash_write_block(addr, ota.hex.data, ota.hex.num);
        if (e) ok = false;
      }
      if (ok) ota.bytes += ota.hex.num;
    }
  }

  if (ok) {
    ota.okLines++;
    _ota->print("OK ");  _ota->print(lineNo);  _ota->print("\r\n");
  } else {
    ota.badLines++;
    ota.fatal = true;
    _ota->print("BAD "); _ota->print(lineNo);  _ota->print("\r\n");
  }
}

static void endHex() {
  if (!ota.inProgress) { sendLine("HEX IDLE"); return; }

  ota.inProgress     = false;
  ota.handshakeReady = false;

  uint32_t payload = 0;
  if (ota.hex.min != 0xFFFFFFFF && ota.hex.max > ota.hex.min) {
    payload = ota.hex.max - ota.hex.min;
  }

  bool success = (!ota.fatal && ota.badLines == 0);
  if (success && !ota.hex.eof) success = false;
  if (success && ota.bytes == 0) success = false;

#if defined(KINETISK) || defined(KINETISL)
  if (success) {
    uint32_t v = *(uint32_t*)(0x40C + ota.bufferAddr);
    if (v != 0xFFFFF9DE) success = false;
  }
#endif

  if (success && !check_flash_id(ota.bufferAddr, payload)) success = false;

  if (success) {
    _ota->print("HEX OK lines="); _ota->print(ota.hex.lines);
    _ota->print(" bytes=");       _ota->print(ota.bytes);
    _ota->print("\r\n");
    _ota->print("APPLIED\r\n");
    _ota->flush();

    flash_move(FLASH_BASE_ADDR, ota.bufferAddr, payload);
    REBOOT;
  } else {
    _ota->print("HEX ERR lines="); _ota->print(ota.hex.lines);
    _ota->print(" bad=");          _ota->print(ota.badLines);
    _ota->print("\r\n");
    firmware_buffer_free(ota.bufferAddr, ota.bufferSize);
  }

  resetOtaSession();
}
