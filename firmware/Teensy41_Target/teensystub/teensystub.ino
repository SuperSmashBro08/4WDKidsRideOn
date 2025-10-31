/*
 * Teensy 4.1 OTA receiver (Stage-1 runtime, flash ops stubbed, safe scopy)
 * ------------------------------------------------------------------------
 * Wiring:
 *   Teensy 4.1 Serial2  RX2 pin 7  <- ESP32 TX
 *   Teensy 4.1 Serial2  TX2 pin 8  -> ESP32 RX
 *   GND shared
 *
 * NOTE: Flash ops are stubbed. We’ll replace them later with a RAM-only flasher.
 */

#include <Arduino.h>
#include <cstring>
#include <strings.h>
#include <imxrt.h>

// ---------------------------------------------------------------------------
// Safe string copy helper (always terminates)
// ---------------------------------------------------------------------------
static inline void scopy(char* dst, size_t dstsz, const char* src) {
  if (!dst || !dstsz) return;
  if (!src) { dst[0] = '\0'; return; }
  size_t n = strlen(src);
  if (n >= dstsz) n = dstsz - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

// ---------------------------------------------------------------------------
// Intel HEX types (must appear before use)
// ---------------------------------------------------------------------------
struct IntelHexRecord {
  uint8_t  length = 0;
  uint16_t address = 0;
  uint8_t  type    = 0;
  uint8_t  data[255];
};

static bool parseIntelHex(const char* rec, IntelHexRecord& out);
static bool applyIntelRecord(const IntelHexRecord& rec, char* errBuf, size_t errLen);

// ---------------------------------------------------------------------------
// User settings
// ---------------------------------------------------------------------------
static constexpr uint32_t BAUD_USB   = 115200;
static constexpr uint32_t BAUD_UART  = 115200;
static constexpr char     OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";
static constexpr char     FW_VERSION[] = "fw-ota-stage1-stub+v4";
static constexpr uint8_t  LED_PIN    = 13;

static constexpr uint32_t FLASH_BASE_ADDR   = 0x60000000u;
static constexpr uint32_t FLASH_TOTAL_SIZE  = 0x200000u;
static constexpr uint32_t FLASH_SECTOR_SIZE = 4096u;
static constexpr uint32_t FLASH_PAGE_SIZE   = 256u;

// ---------------------------------------------------------------------------
// Temp stubs (compile now; no real flash yet)
// ---------------------------------------------------------------------------
extern "C" void arm_dcache_delete(void *addr, unsigned int size);

static inline int flexspi_nor_flash_init(void*) { return 0; }
static inline int flexspi_nor_flash_erase_sector(void*, uint32_t) { return 0; }
static inline int flexspi_nor_flash_page_program(void*, uint32_t, const uint32_t*) { return 0; }

// ---------------------------------------------------------------------------
// Intel HEX parser
// ---------------------------------------------------------------------------
static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool parseIntelHex(const char* rec, IntelHexRecord& out) {
  size_t len = strlen(rec);
  if (len < 11) return false;
  if (rec[0] != ':') return false;

  auto getByte = [&](size_t pos) -> int {
    int hi = hexNibble(rec[pos]);
    int lo = hexNibble(rec[pos + 1]);
    if (hi < 0 || lo < 0) return -1;
    return (hi << 4) | lo;
  };

  int count   = getByte(1);
  int addrHi  = getByte(3);
  int addrLo  = getByte(5);
  int recType = getByte(7);
  if (count < 0 || addrHi < 0 || addrLo < 0 || recType < 0) return false;

  size_t dataStart = 9;
  size_t dataEnd   = dataStart + (size_t)count * 2u;
  if (dataEnd + 2u > len) return false;

  int checksum = count + addrHi + addrLo + recType;
  for (int i = 0; i < count; ++i) {
    int v = getByte(dataStart + (size_t)i*2u);
    if (v < 0) return false;
    out.data[i] = (uint8_t)v;
    checksum += v;
  }
  checksum = ((~checksum + 1) & 0xFF);
  int recCksum = getByte(dataEnd);
  if (recCksum < 0 || checksum != recCksum) return false;

  out.length  = (uint8_t)count;
  out.address = (uint16_t)(((uint16_t)addrHi << 8) | (uint16_t)addrLo);
  out.type    = (uint8_t)recType;
  return true;
}

// ---------------------------------------------------------------------------
// Flash writer
// ---------------------------------------------------------------------------
class FlashWriter {
public:
  bool begin() {
    errorFlag = false;
    errorMsg[0] = '\0';
    sectorLoaded = false;
    sectorBase = 0xFFFFFFFFu;
    sectorsErased = 0;
    pagesProgrammed = 0;
    seenData = false;
    memset(pageDirty, 0, sizeof(pageDirty));

    if (!flexspiInitialized) {
      if (flexspi_nor_flash_init(nullptr) != 0) {
        scopy(errorMsg, sizeof(errorMsg), "flexspi init failed");
        return false;
      }
      flexspiInitialized = true;
    }
    return true;
  }

  bool writeData(uint32_t address, const uint8_t* data, uint32_t len) {
    if (errorFlag) return false;
    if (len == 0) return true;

    if (address < FLASH_BASE_ADDR || (address + len) > (FLASH_BASE_ADDR + FLASH_TOTAL_SIZE)) {
      scopy(errorMsg, sizeof(errorMsg), "record outside flash range");
      errorFlag = true;
      return false;
    }

    seenData = true;
    while (len > 0) {
      uint32_t sectorAddr = address & ~(FLASH_SECTOR_SIZE - 1u);
      if (!loadSector(sectorAddr)) return false;

      uint32_t offset = address - sectorBase;
      uint32_t chunk  = min(len, FLASH_SECTOR_SIZE - offset);
      memcpy(sectorBuf + offset, data, chunk);

      uint32_t pStart = offset / FLASH_PAGE_SIZE;
      uint32_t pEnd   = (offset + chunk - 1u) / FLASH_PAGE_SIZE;
      for (uint32_t p = pStart; p <= pEnd; ++p) pageDirty[p] = true;

      address += chunk;
      data    += chunk;
      len     -= chunk;
    }
    return true;
  }

  bool finalize() {
    if (errorFlag) return false;
    return flushSector();
  }

  bool hadData() const { return seenData; }
  uint32_t getSectorsErased() const { return sectorsErased; }
  uint32_t getPagesProgrammed() const { return pagesProgrammed; }
  const char* lastError() const { return errorMsg; }

private:
  bool loadSector(uint32_t base) {
    if (sectorLoaded && base == sectorBase) return true;
    if (!flushSector()) return false;
    sectorBase = base;
    sectorLoaded = true;
    memcpy(sectorBuf, (const void*)sectorBase, FLASH_SECTOR_SIZE);
    memset(pageDirty, 0, sizeof(pageDirty));
    return true;
  }

  bool flushSector() {
    if (!sectorLoaded) return true;

    bool dirty = false;
    for (bool d : pageDirty) { if (d) { dirty = true; break; } }
    if (!dirty) { sectorLoaded = false; return true; }

    arm_dcache_delete((void*)sectorBase, FLASH_SECTOR_SIZE);
    __disable_irq();
    int e = flexspi_nor_flash_erase_sector(nullptr, sectorBase);
    __enable_irq();
    if (e != 0) { scopy(errorMsg, sizeof(errorMsg), "flash erase failed"); errorFlag = true; return false; }
    sectorsErased++;
    arm_dcache_delete((void*)sectorBase, FLASH_SECTOR_SIZE);

    uint8_t pageBuf[FLASH_PAGE_SIZE];
    for (uint32_t i = 0; i < (FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE); ++i) {
      if (!pageDirty[i]) continue;
      memcpy(pageBuf, sectorBuf + (i*FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);
      arm_dcache_delete(pageBuf, FLASH_PAGE_SIZE);
      __disable_irq();
      int p = flexspi_nor_flash_page_program(
                nullptr,
                sectorBase + (i*FLASH_PAGE_SIZE),
                reinterpret_cast<const uint32_t*>(pageBuf));
      __enable_irq();
      if (p != 0) { scopy(errorMsg, sizeof(errorMsg), "flash program failed"); errorFlag = true; return false; }
      pagesProgrammed++;
      arm_dcache_delete((void*)(sectorBase + (i*FLASH_PAGE_SIZE)), FLASH_PAGE_SIZE);
    }

    sectorLoaded = false;
    return true;
  }

  bool flexspiInitialized = false;
  bool sectorLoaded = false;
  bool errorFlag = false;
  bool seenData = false;
  uint32_t sectorBase = 0xFFFFFFFFu;
  uint32_t sectorsErased = 0;
  uint32_t pagesProgrammed = 0;
  bool     pageDirty[FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE];
  uint8_t  sectorBuf[FLASH_SECTOR_SIZE];
  char     errorMsg[96];
};

static FlashWriter flashWriter;

// ---------------------------------------------------------------------------
// OTA session
// ---------------------------------------------------------------------------
struct OtaSession {
  bool handshakeReady = false;
  bool inProgress     = false;
  uint32_t lines = 0, good = 0, bad = 0, bytes = 0;
  uint32_t extLinear = 0, extSegment = 0;
  bool seenEof = false;
  uint32_t minAddress = 0xFFFFFFFFu, maxAddress = 0u;
  char lastError[128];

  void resetCounters() {
    lines = good = bad = bytes = 0;
    extLinear = 0; extSegment = 0;
    seenEof = false;
    minAddress = 0xFFFFFFFFu; maxAddress = 0u;
    lastError[0] = '\0';
  }
} ota;

// ---------------------------------------------------------------------------
// Diagnostics and console
// ---------------------------------------------------------------------------
static void logStatus() {
  Serial.printf("HEX lines=%lu ok=%lu bad=%lu bytes=%lu sectors=%lu pages=%lu\n",
      ota.lines, ota.good, ota.bad, ota.bytes,
      flashWriter.getSectorsErased(), flashWriter.getPagesProgrammed());
  if (ota.minAddress <= ota.maxAddress) {
    Serial.printf("Addr range: 0x%08lX - 0x%08lX\n", ota.minAddress, ota.maxAddress);
  }
  if (ota.lastError[0]) {
    Serial.print("Last error: "); Serial.println(ota.lastError);
  }
}

static void handleUsbConsole() {
  static char buffer[96]; static size_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (len+1 < sizeof(buffer)) buffer[len++] = c; continue; }
    buffer[len] = '\0';
    if (!len) { len = 0; continue; }

    if      (!strcasecmp(buffer, "version")) { Serial.printf("FW %s\n", FW_VERSION); }
    else if (!strcasecmp(buffer, "status"))  { logStatus(); }
    else if (!strcasecmp(buffer, "help"))    { Serial.println("Commands: version, status, help, reboot"); }
    else if (!strcasecmp(buffer, "reboot"))  { Serial.println("Rebooting..."); delay(50); SCB_AIRCR = 0x05FA0004; }
    else                                     { Serial.println("Unknown. Try: help"); }

    len = 0;
  }
}

static void send2(const char* s) { Serial2.print(s); Serial2.print("\r\n"); }

// ---------------------------------------------------------------------------
// Apply one HEX record
// ---------------------------------------------------------------------------
static bool applyIntelRecord(const IntelHexRecord& rec, char* errBuf, size_t errLen) {
  errBuf[0] = '\0';
  switch (rec.type) {
    case 0x00: { // data
      uint32_t upper = ota.extLinear ? (ota.extLinear << 16) : (ota.extSegment << 4);
      uint32_t absolute = upper + rec.address;
      ota.bytes += rec.length;
      if (absolute < ota.minAddress) ota.minAddress = absolute;
      uint32_t endAddr = absolute + rec.length - 1u;
      if (endAddr > ota.maxAddress) ota.maxAddress = endAddr;
      if (!flashWriter.writeData(absolute, rec.data, rec.length)) {
        scopy(errBuf, errLen, flashWriter.lastError());
        return false;
      }
      return true;
    }
    case 0x01: ota.seenEof = true; return true;       // EOF
    case 0x02: // segment
      if (rec.length != 2) { scopy(errBuf, errLen, "bad segment rec"); return false; }
      ota.extSegment = ((uint32_t)rec.data[0] << 8) | rec.data[1]; ota.extLinear = 0; return true;
    case 0x04: // linear
      if (rec.length != 2) { scopy(errBuf, errLen, "bad linear rec"); return false; }
      ota.extLinear = ((uint32_t)rec.data[0] << 8) | rec.data[1]; ota.extSegment = 0; return true;
    case 0x05: return true; // start address (ignored)
    default: scopy(errBuf, errLen, "unsupported rec"); return false;
  }
}

// ---------------------------------------------------------------------------
// Serial2 command line
// ---------------------------------------------------------------------------
static void handleSerial2Line(const char* line) {
  if (!line[0]) return;
  Serial.print("[Teensy] RX: "); Serial.println(line);

  if (!strncasecmp(line, "HELLO", 5)) {
    if (ota.inProgress) { send2("BUSY"); return; }
    const char* tok = line + 5; while (*tok == ' ') tok++;
    if (!*tok) { send2("NACK"); Serial.println("[Teensy] HELLO missing token"); return; }
    if (!strcmp(tok, OTA_TOKEN)) { ota.handshakeReady = true; send2("READY"); Serial.println("[Teensy] HELLO OK"); }
    else { ota.handshakeReady = false; send2("NACK"); Serial.println("[Teensy] HELLO token mismatch"); }
    return;
  }

  if (!strcasecmp(line, "BEGIN HEX")) {
    if (!ota.handshakeReady || ota.inProgress) { send2("HEX IDLE"); return; }
    ota.resetCounters();
    ota.inProgress = flashWriter.begin();
    if (!ota.inProgress) {
      scopy(ota.lastError, sizeof(ota.lastError), flashWriter.lastError());
      send2("HEX FAIL");
      Serial.println("[Teensy] Flash writer begin failed");
      return;
    }
    send2("HEX BEGIN");
    Serial.println("[Teensy] HEX session begin");
    return;
  }

  if (!strncasecmp(line, "L ", 2) && ota.inProgress) {
    const char* recStr = line + 2;
    ota.lines++;
    IntelHexRecord rec;
    if (!parseIntelHex(recStr, rec)) { ota.bad++; send2("BAD"); Serial.println("[Teensy] Parse fail"); return; }
    char err[96];
    if (!applyIntelRecord(rec, err, sizeof(err))) {
      ota.bad++; scopy(ota.lastError, sizeof(ota.lastError), err);
      send2("BAD"); Serial.print("[Teensy] Record err: "); Serial.println(err); return;
    }
    ota.good++;
    Serial2.print("OK "); Serial2.print(ota.lines); Serial2.print("\r\n");
    return;
  }

  if (!strcasecmp(line, "END")) {
    if (!ota.inProgress) { send2("HEX IDLE"); return; }

    bool success = false;
    if (ota.bad == 0) {
      if (!flashWriter.finalize()) {
        scopy(ota.lastError, sizeof(ota.lastError), flashWriter.lastError());
      } else if (!ota.seenEof) {
        scopy(ota.lastError, sizeof(ota.lastError), "EOF missing");
      } else if (!flashWriter.hadData()) {
        scopy(ota.lastError, sizeof(ota.lastError), "no data");
      } else success = true;
    }

    ota.inProgress = false; ota.handshakeReady = false;

    if (success) {
      Serial2.print("HEX OK lines="); Serial2.print(ota.good);
      Serial2.print(" bytes=");       Serial2.print(ota.bytes);
      Serial2.print(" sectors=");     Serial2.print(flashWriter.getSectorsErased());
      Serial2.print("\r\n");
      send2("APPLIED");
      Serial.println("[Teensy] OTA applied (stubbed flash), rebooting in 1 s");
      delay(1000);
      SCB_AIRCR = 0x05FA0004;
    } else {
      Serial2.print("HEX ERR lines="); Serial2.print(ota.lines);
      Serial2.print(" bad=");          Serial2.print(ota.bad);
      if (ota.lastError[0]) { Serial2.print(" msg="); Serial2.print(ota.lastError); }
      Serial2.print("\r\n");
      Serial.print("[Teensy] OTA failed: "); Serial.println(ota.lastError);
    }
    return;
  }

  if (!strcasecmp(line, "PING"))    { send2("PONG"); return; }
  if (!strcasecmp(line, "VERSION")) { Serial2.print("FW "); Serial2.print(FW_VERSION); Serial2.print("\r\n"); return; }
  if (!
