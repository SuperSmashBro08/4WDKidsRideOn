#include "OtaUpdater.h"
#include "FXUtil.h"
#include "OtaConsole.h"
extern "C" {
  #include "FlashTxx.h"
}

// Weak default for app hook
extern "C" bool __attribute__((weak)) OtaUserHandleLine(const char* line) { (void)line; return false; }

static HardwareSerial* _ota = &Serial2;
static uint32_t        _baud = 115200;

static const char*     LOADER_ID = "FlasherX v2.4 (in-app)";
static const char*     _appVer   = "App (unknown)";

struct OtaSession {
  bool     active = false;
  bool     begun  = false;
  uint32_t lines  = 0;
  uint32_t bad    = 0;
  uint32_t bytes  = 0;
  uint32_t bufAddr = 0;
  uint32_t bufSize = 0;
  uint32_t xbase   = 0;
  uint32_t minAddr = 0xFFFFFFFFu;
  uint32_t maxAddr = 0;
  uint32_t lastMs  = 0;        // NEW: activity watchdog
} ota;

static char   _line[600];
static size_t _llen = 0;

static inline void sendLine(const char* s){ _ota->print(s); _ota->print("\r\n"); }
static inline void touch() { ota.lastMs = millis(); }

static inline void resetSession(){
  if (ota.bufAddr) firmware_buffer_free(ota.bufAddr, ota.bufSize);
  ota = OtaSession{};
  OtaConsole::setEnabled(true);    // ensure console is back on after any reset
}

static uint8_t hex2nib(char c){
  if (c>='0' && c<='9') return (uint8_t)(c - '0');
  if (c>='A' && c<='F') return (uint8_t)(c - 'A' + 10);
  if (c>='a' && c<='f') return (uint8_t)(c - 'a' + 10);
  return 0xFF;
}
static bool hex2byte(const char* p, uint8_t& out){
  uint8_t h = hex2nib(p[0]); uint8_t l = hex2nib(p[1]);
  if (h==0xFF || l==0xFF) return false;
  out = (uint8_t)((h<<4)|l); return true;
}

static bool parseIntelHexLine(const char* rec, uint32_t& outDataBytes){
  outDataBytes = 0;
  if (rec[0] != ':') return false;

  uint8_t count; if(!hex2byte(rec+1, count)) return false;
  uint8_t ah, al; if(!hex2byte(rec+3, ah) || !hex2byte(rec+5, al)) return false;
  uint16_t addr16 = ((uint16_t)ah<<8) | al;
  uint8_t rectype; if(!hex2byte(rec+7, rectype)) return false;

  uint8_t sum = count + ah + al + rectype;
  const char* pdata = rec + 9;

  if (rectype == 0x00) {
    for (uint32_t i=0; i<count; ++i){
      uint8_t b; if(!hex2byte(pdata + (i*2), b)) return false;
      sum += b;
    }
    uint8_t cks; if(!hex2byte(pdata + (count*2), cks)) return false;
    sum += cks;
    if ((sum & 0xFF) != 0) return false;

    uint32_t base = (ota.xbase << 16) | addr16;

    if (!ota.bufAddr){
      if (firmware_buffer_init(&ota.bufAddr, &ota.bufSize) == 0 || ota.bufAddr==0 || ota.bufSize==0) return false;
      memset((void*)ota.bufAddr, 0xFF, ota.bufSize);
    }

    for (uint32_t i=0; i<count; ++i){
      uint8_t b; hex2byte(pdata + (i*2), b);
      uint32_t a = base + i;

      if (a < ota.minAddr) ota.minAddr = a;
      if (a > ota.maxAddr) ota.maxAddr = a;

      uint32_t off = a - FLASH_BASE_ADDR;
      if (off < ota.bufSize){
        ((uint8_t*)ota.bufAddr)[off] = b;
        outDataBytes++;
      } else {
        return false;
      }
    }
    return true;
  }
  else if (rectype == 0x01) {
    uint8_t cks; if(!hex2byte(pdata, cks)) return false;
    sum += cks; if ((sum & 0xFF) != 0) return false;
    return true;
  }
  else if (rectype == 0x04) {
    uint8_t hi, lo; if(!hex2byte(pdata, hi) || !hex2byte(pdata+2, lo)) return false;
    sum += hi + lo;
    uint8_t cks; if(!hex2byte(pdata+4, cks)) return false;
    sum += cks; if ((sum & 0xFF) != 0) return false;
    ota.xbase = (((uint16_t)hi<<8)|lo);
    return true;
  }
  for (uint32_t i=0; i<count; ++i){
    uint8_t b; if(!hex2byte(pdata + (i*2), b)) return false;
    sum += b;
  }
  uint8_t cks; if(!hex2byte(pdata + (count*2), cks)) return false;
  sum += cks;
  return ((sum & 0xFF) == 0);
}

// --- NEW: stale-session auto-expire (called from pump) ---
static void watchdog(){
  if (!ota.active) return;
  const uint32_t now = millis();
  // If session is active but never began and we haven't heard anything for 5s, reset.
  if (!ota.begun && (uint32_t)(now - ota.lastMs) > 5000){
    resetSession();
  }
}

// Handle top-level command line. Returns true if OTA handled it.
static bool handleLine(const char* line){
  if (!line || !*line) return false;

  // VERSION
  if (strcmp(line, "VERSION") == 0){
    _ota->print("FLASHERX "); _ota->print(OtaUpdater::loaderId()); _ota->print("\r\n");
    _ota->print("FW "); _ota->print(_appVer); _ota->print("\r\n");
    return true;
  }

  // PING
  if (strcmp(line, "PING") == 0){
    sendLine("PONG");
    return true;
  }

  // ABORT (NEW): force-cancel any session
  if (strcmp(line, "ABORT") == 0){
    resetSession();
    sendLine("OK");
    return true;
  }

  // HELLO <token>
  if (strncmp(line, "HELLO ", 6) == 0){
    // idempotent HELLO:
    // - if not active → start, mute console, READY
    // - if active but not begun → re-READY (not BUSY)
    // - if begun → BUSY
    if (!ota.active){
      OtaConsole::setEnabled(false);   // hard-mute immediately
      ota.active = true;
      ota.begun  = false;
      ota.lines = ota.bad = ota.bytes = 0;
      ota.xbase = 0;
      if (ota.bufAddr){ firmware_buffer_free(ota.bufAddr, ota.bufSize); ota.bufAddr=0; ota.bufSize=0; }
      touch();
      sendLine("READY");
    } else if (!ota.begun){
      touch();
      sendLine("READY");               // idempotent reply
    } else {
      sendLine("BUSY");                // flashing in progress
    }
    return true;
  }

  // Busy guard
  if (ota.active){
    touch();

    if (strcmp(line, "BEGIN HEX") == 0){
      ota.begun = true;
      sendLine("HEX BEGIN");
      return true;
    }

    if (strncmp(line, "L ", 2) == 0){
      ota.lines++;
      if (!ota.begun){ sendLine("BAD 0"); return true; }

      const char* rec = line + 2;
      while (*rec==' ') ++rec;
      if (*rec != ':'){ ota.bad++; _ota->print("BAD "); _ota->print(ota.lines); _ota->print("\r\n"); return true; }

      uint32_t wrote = 0;
      bool ok = parseIntelHexLine(rec, wrote);
      if (!ok){ ota.bad++; _ota->print("BAD "); _ota->print(ota.lines); _ota->print("\r\n"); return true; }

      ota.bytes += wrote;
      _ota->print("OK "); _ota->print(ota.lines); _ota->print("\r\n");
      return true;
    }

    if (strcmp(line, "END") == 0){
      if (ota.bad==0 && ota.lines>0){
        _ota->print("HEX OK "); _ota->print(ota.lines); _ota->print(" bytes="); _ota->print(ota.bytes); _ota->print("\r\n");
        _ota->flush();

        uint32_t payload = 0;
        if (ota.minAddr <= ota.maxAddr && ota.minAddr != 0xFFFFFFFFu){
          payload = (ota.maxAddr - FLASH_BASE_ADDR) + 1u;
        }

        _ota->print("APPLIED\r\n");
        _ota->flush();

        flash_move(FLASH_BASE_ADDR, ota.bufAddr, payload ? payload : ota.bufSize);
        REBOOT; // no re-enable needed; rebooting
      } else {
        _ota->print("HEX ERR lines="); _ota->print(ota.lines);
        _ota->print(" bad=");          _ota->print(ota.bad);
        _ota->print("\r\n");
        resetSession();                // re-enable console
      }
      return true;
    }

    // swallow anything else during active session
    return true;
  }

  // Not OTA: let app consume first
  if (OtaUserHandleLine(line)) return true;

  // Not recognized
  sendLine("ERR");
  return true;
}

// Pump Serial2, collect lines, dispatch
static void pump(){
  watchdog(); // NEW
  while (_ota->available()){
    int c = _ota->read();
    if (c < 0) break;
    if (c == '\r') continue;

    if (c == '\n'){
      _line[_llen] = 0;
      size_t n = _llen; while (n && (_line[n-1]==' ' || _line[n-1]=='\t')) _line[--n] = 0;
      handleLine(_line);
      _llen = 0;
    } else {
      if (_llen + 1 < sizeof(_line)) _line[_llen++] = (char)c;
      else _llen = 0; // overflow -> reset
    }
  }
}

namespace OtaUpdater {
  void begin(HardwareSerial& otaPort, uint32_t baud){
    _ota  = &otaPort; _baud = baud;
    _ota->begin(_baud);
    resetSession();                 // start clean; console enabled
  }
  void setAppVersion(const char* name){
    _appVer = (name && *name) ? name : "App (unknown)";
  }
  void tick(){ pump(); }
  bool inProgress(){ return ota.active; }
  const char* loaderId(){ return LOADER_ID; }
}
