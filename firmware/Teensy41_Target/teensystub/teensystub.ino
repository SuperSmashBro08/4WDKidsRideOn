/*
 * Teensy 4.1 OTA receiver & self-flasher
 * --------------------------------------
 * Wiring: Serial2 RX2 (pin 7) <- ESP32 GPIO43, Serial2 TX2 (pin 8) -> ESP32 GPIO44, share GND.
 * OTA protocol: HELLO <token> / BEGIN HEX / L <record> / END. The Teensy validates each Intel HEX
 * record, streams the data into 4 KB flash sectors, erases/programs the internal QSPI flash, and
 * finally reboots into the new image when flashing succeeds.
 */

#include <Arduino.h>
#include <cstring>
#include <strings.h>
#include <imxrt.h>

static constexpr uint8_t  LED_PIN    = 13;
static constexpr uint32_t BAUD_USB  = 115200;   // Serial (USB) to PC
static constexpr uint32_t BAUD_UART = 115200;   // Serial2 (pins 7/8) to ESP32
static constexpr char     OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";
static constexpr char     FW_VERSION[] = "fw-ota-flash-v1";
static constexpr uint8_t POT_PIN = 27;

static constexpr uint32_t FLASH_BASE_ADDR = 0x60000000u;
static constexpr uint32_t FLASH_TOTAL_SIZE = 0x200000u; // 2 MB program flash on Teensy 4.1
static constexpr uint32_t FLASH_SECTOR_SIZE = 4096u;
static constexpr uint32_t FLASH_PAGE_SIZE = 256u;

extern "C" {
  int flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
  int flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t address, const uint32_t *src);
  int flexspi_nor_flash_init(FLEXSPI_Type *base);
  void arm_dcache_delete(void *addr, unsigned int size);
}

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
      if (flexspi_nor_flash_init(FLEXSPI) != 0) {
        setError("flexspi init failed");
        return false;
      }
      flexspiInitialized = true;
    }
    return true;
  }

  bool writeData(uint32_t address, const uint8_t *data, uint32_t len) {
    if (errorFlag) return false;
    if (len == 0) return true;
    if (address < FLASH_BASE_ADDR || (address + len) > (FLASH_BASE_ADDR + FLASH_TOTAL_SIZE)) {
      setError("record outside flash range");
      return false;
    }

    seenData = true;

    while (len > 0) {
      uint32_t sectorAddr = address & ~(FLASH_SECTOR_SIZE - 1u);
      if (!loadSector(sectorAddr)) {
        return false;
      }

      uint32_t offset = address - sectorBase;
      uint32_t chunk = min(len, FLASH_SECTOR_SIZE - offset);
      memcpy(sectorBuf + offset, data, chunk);

      uint32_t pageStart = offset / FLASH_PAGE_SIZE;
      uint32_t pageEnd = (offset + chunk - 1u) / FLASH_PAGE_SIZE;
      for (uint32_t p = pageStart; p <= pageEnd; ++p) {
        pageDirty[p] = true;
      }

      address += chunk;
      data += chunk;
      len -= chunk;
    }
    return true;
  }

  bool finalize() {
    if (errorFlag) return false;
    if (!flushSector()) {
      return false;
    }
    return true;
  }

  bool hadData() const { return seenData; }
  uint32_t getSectorsErased() const { return sectorsErased; }
  uint32_t getPagesProgrammed() const { return pagesProgrammed; }
  const char* lastError() const { return errorMsg; }

private:
  bool loadSector(uint32_t base) {
    if (sectorLoaded && base == sectorBase) {
      return true;
    }
    if (!flushSector()) {
      return false;
    }

    sectorBase = base;
    sectorLoaded = true;
    memcpy(sectorBuf, (const void*)sectorBase, FLASH_SECTOR_SIZE);
    memset(pageDirty, 0, sizeof(pageDirty));
    return true;
  }

  bool flushSector() {
    if (!sectorLoaded) {
      return true;
    }

    bool dirty = false;
    for (bool flag : pageDirty) {
      if (flag) { dirty = true; break; }
    }
    if (!dirty) {
      sectorLoaded = false;
      return true;
    }

    arm_dcache_delete((void*)sectorBase, FLASH_SECTOR_SIZE);

    __disable_irq();
    int eraseStatus = flexspi_nor_flash_erase_sector(FLEXSPI, sectorBase);
    __enable_irq();
    if (eraseStatus != 0) {
      setError("flash erase failed");
      return false;
    }
    sectorsErased++;
    arm_dcache_delete((void*)sectorBase, FLASH_SECTOR_SIZE);

    uint8_t pageBuf[FLASH_PAGE_SIZE];
    for (uint32_t i = 0; i < (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE); ++i) {
      if (!pageDirty[i]) continue;
      memcpy(pageBuf, sectorBuf + (i * FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);
      arm_dcache_delete(pageBuf, FLASH_PAGE_SIZE);
      __disable_irq();
      int progStatus = flexspi_nor_flash_page_program(
        FLEXSPI, sectorBase + (i * FLASH_PAGE_SIZE), reinterpret_cast<const uint32_t*>(pageBuf));
      __enable_irq();
      if (progStatus != 0) {
        setError("flash program failed");
        return false;
      }
      pagesProgrammed++;
      arm_dcache_delete((void*)(sectorBase + (i * FLASH_PAGE_SIZE)), FLASH_PAGE_SIZE);
    }

    sectorLoaded = false;
    return true;
  }

  void setError(const char* msg) {
    errorFlag = true;
    strncpy(errorMsg, msg, sizeof(errorMsg) - 1);
    errorMsg[sizeof(errorMsg) - 1] = '\0';
  }

  bool flexspiInitialized = false;
  bool sectorLoaded = false;
  bool errorFlag = false;
  bool seenData = false;
  uint32_t sectorBase = 0xFFFFFFFFu;
  uint32_t sectorsErased = 0;
  uint32_t pagesProgrammed = 0;
  bool pageDirty[FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE];
  uint8_t sectorBuf[FLASH_SECTOR_SIZE];
  char errorMsg[96];
};

static FlashWriter flashWriter;

struct OtaSession {
  bool handshakeReady = false;
  bool inProgress = false;
  uint32_t lines = 0;
  uint32_t good = 0;
  uint32_t bad = 0;
  uint32_t bytes = 0;
  uint32_t extLinear = 0;
  uint32_t extSegment = 0;
  bool seenEof = false;
  uint32_t minAddress = 0xFFFFFFFFu;
  uint32_t maxAddress = 0u;
  char lastError[128];

  void resetCounters() {
    lines = good = bad = bytes = 0;
    extLinear = 0;
    extSegment = 0;
    seenEof = false;
    minAddress = 0xFFFFFFFFu;
    maxAddress = 0u;
    lastError[0] = '\0';
  }
} ota;

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

struct IntelHexRecord {
  uint8_t length = 0;
  uint16_t address = 0;
  uint8_t type = 0;
  uint8_t data[255];
};

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

  int count = getByte(1);
  int addrHigh = getByte(3);
  int addrLow = getByte(5);
  int recType = getByte(7);
  if (count < 0 || addrHigh < 0 || addrLow < 0 || recType < 0) return false;

  size_t dataStart = 9;
  size_t dataEnd = dataStart + (size_t)count * 2u;
  if (dataEnd + 2u > len) return false;

  int checksum = count + addrHigh + addrLow + recType;
  for (int i = 0; i < count; ++i) {
    int value = getByte(dataStart + (size_t)i * 2u);
    if (value < 0) return false;
    out.data[i] = (uint8_t)value;
    checksum += value;
  }
  checksum = ((~checksum + 1) & 0xFF);
  int recordChecksum = getByte(dataEnd);
  if (recordChecksum < 0 || checksum != recordChecksum) return false;

  out.length = (uint8_t)count;
  out.address = (uint16_t)(((uint16_t)addrHigh << 8) | (uint16_t)addrLow);
  out.type = (uint8_t)recType;
  return true;
}

static void logStatus() {
  Serial.printf(
    "HEX lines=%lu ok=%lu bad=%lu bytes=%lu sectors=%lu pages=%lu\n",
    ota.lines, ota.good, ota.bad, ota.bytes,
    flashWriter.getSectorsErased(), flashWriter.getPagesProgrammed());
  if (ota.minAddress <= ota.maxAddress) {
    Serial.printf("Addr range: 0x%08lX - 0x%08lX\n", ota.minAddress, ota.maxAddress);
  }
  if (ota.lastError[0]) {
    Serial.print("Last error: ");
    Serial.println(ota.lastError);
  }
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
      logStatus();
    } else if (strcasecmp(buffer, "help") == 0) {
      Serial.println("Commands: version, status, help");
    } else if (strcasecmp(buffer, "reboot") == 0) {
      Serial.println("Manual reboot requested...");
      delay(50);
      SCB_AIRCR = 0x05FA0004; // system reset
    } else {
      Serial.println("Unknown command. Try: help");
    }

    len = 0;
  }
}

static void sendLine(const char* s) {
  Serial2.print(s);
  Serial2.print('\r');
  Serial2.print('\n');
}

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
        strncpy(errBuf, flashWriter.lastError(), errLen - 1);
        errBuf[errLen - 1] = '\0';
        return false;
      }
      return true;
    }
    case 0x01: // EOF
      ota.seenEof = true;
      return true;
    case 0x02: // extended segment address
      if (rec.length != 2) {
        strncpy(errBuf, "bad segment record", errLen - 1);
        errBuf[errLen - 1] = '\0';
        return false;
      }
      ota.extSegment = ((uint32_t)rec.data[0] << 8) | rec.data[1];
      ota.extLinear = 0;
      return true;
    case 0x04: // extended linear address
      if (rec.length != 2) {
        strncpy(errBuf, "bad linear record", errLen - 1);
        errBuf[errLen - 1] = '\0';
        return false;
      }
      ota.extLinear = ((uint32_t)rec.data[0] << 8) | rec.data[1];
      ota.extSegment = 0;
      return true;
    case 0x05: // start linear address
      return true;
    default:
      strncpy(errBuf, "unsupported record", errLen - 1);
      errBuf[errLen - 1] = '\0';
      return false;
  }
}

static void handleSerial2Line(const char* line) {
  Serial.print("[Teensy] RX: ");
  Serial.println(line);
  if (!line[0]) return;

  if (strncasecmp(line, "HELLO", 5) == 0) {
    if (ota.inProgress) {
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
      ota.handshakeReady = true;
      Serial.println("[Teensy] HELLO accepted");
      sendLine("READY");
    } else {
      ota.handshakeReady = false;
      Serial.println("[Teensy] HELLO rejected (token mismatch)");
      sendLine("NACK");
    }
    return;
  }

  if (strcasecmp(line, "BEGIN HEX") == 0) {
    if (!ota.handshakeReady || ota.inProgress) {
      sendLine("HEX IDLE");
      return;
    }
    ota.resetCounters();
    ota.inProgress = flashWriter.begin();
    if (!ota.inProgress) {
      strncpy(ota.lastError, flashWriter.lastError(), sizeof(ota.lastError) - 1);
      ota.lastError[sizeof(ota.lastError) - 1] = '\0';
      sendLine("HEX FAIL");
      Serial.println("[Teensy] Failed to start flash writer");
      return;
    }
    Serial.println("[Teensy] HEX session begin");
    sendLine("HEX BEGIN");
    return;
  }

  if (strncasecmp(line, "L ", 2) == 0 && ota.inProgress) {
    const char* recStr = line + 2;
    ota.lines++;
    IntelHexRecord rec;
    if (!parseIntelHex(recStr, rec)) {
      ota.bad++;
      sendLine("BAD");
      Serial.println("[Teensy] Record parse failed");
      return;
    }
    char err[96];
    if (!applyIntelRecord(rec, err, sizeof(err))) {
      ota.bad++;
      strncpy(ota.lastError, err, sizeof(ota.lastError) - 1);
      ota.lastError[sizeof(ota.lastError) - 1] = '\0';
      sendLine("BAD");
      Serial.print("[Teensy] Record error: ");
      Serial.println(err);
      return;
    }
    ota.good++;
    Serial2.print("OK ");
    Serial2.print(ota.lines);
    Serial2.print('\r');
    Serial2.print('\n');
    return;
  }

  if (strcasecmp(line, "END") == 0) {
    if (!ota.inProgress) {
      sendLine("HEX IDLE");
      return;
    }

    bool success = false;
    if (ota.bad == 0) {
      if (!flashWriter.finalize()) {
        strncpy(ota.lastError, flashWriter.lastError(), sizeof(ota.lastError) - 1);
        ota.lastError[sizeof(ota.lastError) - 1] = '\0';
      } else if (!ota.seenEof) {
        strncpy(ota.lastError, "EOF record missing", sizeof(ota.lastError) - 1);
        ota.lastError[sizeof(ota.lastError) - 1] = '\0';
      } else if (!flashWriter.hadData()) {
        strncpy(ota.lastError, "no data records", sizeof(ota.lastError) - 1);
        ota.lastError[sizeof(ota.lastError) - 1] = '\0';
      } else {
        success = true;
      }
    }

    ota.inProgress = false;
    ota.handshakeReady = false;

    if (success) {
      Serial2.print("HEX OK lines=");
      Serial2.print(ota.good);
      Serial2.print(" bytes=");
      Serial2.print(ota.bytes);
      Serial2.print(" sectors=");
      Serial2.print(flashWriter.getSectorsErased());
      Serial2.print('\r');
      Serial2.print('\n');
      sendLine("APPLIED");
      Serial.println("[Teensy] OTA written to flash, rebooting in 1 s");
      delay(1000);
      SCB_AIRCR = 0x05FA0004; // system reset
    } else {
      Serial2.print("HEX ERR lines=");
      Serial2.print(ota.lines);
      Serial2.print(" bad=");
      Serial2.print(ota.bad);
      if (ota.lastError[0]) {
        Serial2.print(" msg=");
        Serial2.print(ota.lastError);
      }
      Serial2.print('\r');
      Serial2.print('\n');
      Serial.print("[Teensy] OTA failed: ");
      Serial.println(ota.lastError);
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
    Serial2.print(ota.lines);
    Serial2.print(" ok=");
    Serial2.print(ota.good);
    Serial2.print(" bad=");
    Serial2.print(ota.bad);
    Serial2.print(" bytes=");
    Serial2.print(ota.bytes);
    Serial2.print('\r');
    Serial2.print('\n');
    return;
  }

  sendLine("ERR");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);

  Serial.begin(BAUD_USB);
  Serial2.begin(BAUD_UART);

  delay(200);
  Serial.println("\n[Teensy] OTA HEX receiver ready on Serial2 (pins 7/8)");
  Serial.print("[Teensy] Firmware version: ");
  Serial.println(FW_VERSION);
  Serial.println("[Teensy] Type 'help' over USB for commands.");
}

void loop() {
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  handleUsbConsole();

  static char line[700];
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
