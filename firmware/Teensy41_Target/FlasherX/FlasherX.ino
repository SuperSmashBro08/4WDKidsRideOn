//****************************************************************************** 
// FlasherX -- firmware OTA update via Intel Hex file over serial or SD stream
//******************************************************************************
//
// (header comments unchanged)
//

#include <stdio.h>
#include <string.h>
#include "FXUtil.h"             // read_ascii_line(), hex file support
extern "C" {
  #include "FlashTxx.h"         // TLC/T3x/T4x/TMM flash primitives
}

static const int led = LED_BUILTIN;   // LED pin used for activity indication
static HardwareSerial& otaSerial = Serial2;

#define FLASHERX_VERSION "FlasherX v2.4"

// --------- NEW: add a simple, human-visible firmware tag for this stub ---------
#define FW_VERSION "fw-simple-ota-v5"   // <-- bump this when you reflash FlasherX itself

static constexpr char OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";
static const uint32_t OTA_BAUD = 115200;

#define LARGE_ARRAY (0)         // 1 = define large array to test large hex file

#if (LARGE_ARRAY)
// (unchanged large array test block)
PROGMEM const uint8_t a[16][16][16][16][16] = A4;
#endif

struct OtaSession {
  bool     handshakeReady;
  bool     inProgress;
  bool     fatal;
  uint32_t bufferAddr;
  uint32_t bufferSize;
  uint32_t bytes;
  uint32_t okLines;
  uint32_t badLines;
  char     lastError[96];
  hex_info_t hex;
  char     hexData[32] __attribute__ ((aligned (8)));
};

static OtaSession ota;

static void resetOtaSession();
static void otaSendLine(const char *msg);
static void processOtaSerial();
static void handleOtaLine(const char *line);
static void beginHexSession();
static void handleHexRecord(const char *record);
static void finalizeHexSession();

static inline void clearLastError() { ota.lastError[0] = 0; }
static inline void setLastError(const char *msg)
{
  if (!msg) { ota.lastError[0] = 0; return; }
  strncpy( ota.lastError, msg, sizeof(ota.lastError) - 1 );
  ota.lastError[ sizeof(ota.lastError) - 1 ] = 0;
}

void setup ()
{
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 2000) {}

  otaSerial.begin(OTA_BAUD);

  pinMode( led, OUTPUT );
  digitalWrite( led, HIGH );
  delay(200);
  digitalWrite( led, LOW );

  Serial.printf( "%s - %s %s\n", FLASHERX_VERSION, __DATE__, __TIME__ );
  Serial.printf( "WARNING: this can ruin your device!\n" );
  Serial.printf( "target = %s (%dK flash in %dK sectors)\n",
                        FLASH_ID, FLASH_SIZE/1024, FLASH_SECTOR_SIZE/1024);
  Serial.printf( "[FlasherX] OTA serial ready @%lu baud\n", (unsigned long)OTA_BAUD );

  // --------- NEW: print the FW_VERSION on USB and on the OTA UART ----------
  Serial.printf( "[FlasherX] FW_VERSION: %s\n", FW_VERSION );
  otaSerial.print( "FW " ); otaSerial.print( FW_VERSION ); otaSerial.print( "\r\n" );
  // Also expose the FlasherX version on the OTA UART once at boot (handy for your /version button)
  otaSerial.print( "FLASHERX " ); otaSerial.print( FLASHERX_VERSION ); otaSerial.print( "\r\n" );

  resetOtaSession();

#if (LARGE_ARRAY)
  Serial.printf( "Large Array -- %08lX\n", (uint32_t)&a[15][15][15][15][15] );
#endif
}

void loop ()
{
  processOtaSerial();
}

static void resetOtaSession()
{
  ota.handshakeReady = false;
  ota.inProgress     = false;
  ota.fatal          = false;
  ota.bufferAddr     = 0;
  ota.bufferSize     = 0;
  ota.bytes          = 0;
  ota.okLines        = 0;
  ota.badLines       = 0;
  clearLastError();
  hex_info_reset( &ota.hex, ota.hexData );
}

static void otaSendLine(const char *msg)
{
  otaSerial.print(msg);
  otaSerial.print("\r\n");
}

static void processOtaSerial()
{
  static char  lineBuf[192];
  static size_t len = 0;

  while (otaSerial.available()) {
    char c = (char)otaSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[len] = '\0';
      if (len > 0) {
        handleOtaLine( lineBuf );
      }
      len = 0;
    }
    else if (len + 1 < sizeof(lineBuf)) {
      lineBuf[len++] = c;
    }
  }
}

static void handleOtaLine(const char *line)
{
  if (!line[0]) return;

  if (line[0] == 'L' && line[1] == ' ') {
    handleHexRecord( line + 2 );
    return;
  }

  Serial.printf( "[FlasherX] CMD: %s\n", line );

  if (!strncmp( line, "HELLO", 5 )) {
    if (ota.inProgress) {
      otaSendLine( "BUSY" );
      return;
    }
    const char *tok = line + 5;
    while (*tok == ' ') tok++;
    if (!*tok) {
      otaSendLine( "NACK" );
      Serial.println( "[FlasherX] HELLO missing token" );
      return;
    }
    if (!strcmp( tok, OTA_TOKEN )) {
      ota.handshakeReady = true;
      otaSendLine( "READY" );
      Serial.println( "[FlasherX] HELLO OK" );
    }
    else {
      ota.handshakeReady = false;
      otaSendLine( "NACK" );
      Serial.println( "[FlasherX] HELLO token mismatch" );
    }
    return;
  }

  if (!strcmp( line, "BEGIN HEX" )) { beginHexSession(); return; }
  if (!strcmp( line, "END" ))       { finalizeHexSession(); return; }
  if (!strcmp( line, "PING" ))      { otaSendLine( "PONG" ); return; }

  // --------- UPDATED: VERSION now returns both FW and FlasherX versions -------
  if (!strcmp( line, "VERSION" )) {
    otaSerial.print( "FW " );        otaSerial.print( FW_VERSION );        otaSerial.print( "\r\n" );
    otaSerial.print( "FLASHERX " );  otaSerial.print( FLASHERX_VERSION );  otaSerial.print( "\r\n" );
    return;
  }

  otaSendLine( "ERR" );
}

static void beginHexSession()
{
  if (!ota.handshakeReady || ota.inProgress) {
    otaSendLine( "HEX IDLE" );
    return;
  }

  uint32_t addr = 0, size = 0;
  if (firmware_buffer_init( &addr, &size ) == 0) {
    otaSendLine( "HEX FAIL" );
    Serial.println( "[FlasherX] Buffer allocation failed" );
    return;
  }

  ota.bufferAddr = addr;
  ota.bufferSize = size;
  ota.inProgress = true;
  ota.fatal      = false;
  ota.bytes      = 0;
  ota.okLines    = 0;
  ota.badLines   = 0;
  clearLastError();
  hex_info_reset( &ota.hex, ota.hexData );

  digitalWrite( led, HIGH );

  Serial.printf( "[FlasherX] HEX session begin, buffer=%lu bytes (%s) @0x%08lX\n",
                 (unsigned long)size,
                 IN_FLASH(addr) ? "FLASH" : "RAM",
                 (unsigned long)addr );

  otaSendLine( "HEX BEGIN" );
}

static void handleHexRecord(const char *record)
{
  if (!ota.inProgress) {
    otaSendLine( "HEX IDLE" );
    return;
  }

  ota.hex.lines++;
  unsigned int lineNumber = ota.hex.lines;
  bool ok = !ota.fatal;

  if (ok && (parse_hex_line( record, ota.hex.data, &ota.hex.addr, &ota.hex.num, &ota.hex.code ) == 0)) {
    setLastError( "parse" );
    ok = false;
  }

  if (ok && (process_hex_record( &ota.hex ) != 0)) {
    snprintf( ota.lastError, sizeof(ota.lastError), "hex code %u", ota.hex.code );
    ok = false;
  }

  if (ok && ota.hex.code == 0) {
    if (ota.hex.max > (FLASH_BASE_ADDR + ota.bufferSize)) {
      setLastError( "address overflow" );
      ok = false;
    }
    else {
      uint32_t addr = ota.bufferAddr + ota.hex.base + ota.hex.addr - FLASH_BASE_ADDR;
      if (!IN_FLASH( ota.bufferAddr )) {
        memcpy( (void*)addr, (void*)ota.hex.data, ota.hex.num );
      }
      else {
        int err = flash_write_block( addr, ota.hex.data, ota.hex.num );
        if (err) {
          snprintf( ota.lastError, sizeof(ota.lastError), "flash err %02X", err );
          ok = false;
        }
      }
      if (ok) {
        ota.bytes += ota.hex.num;
      }
    }
  }

  if (ok) {
    ota.okLines++;
    otaSerial.print( "OK " );
    otaSerial.print( lineNumber );
    otaSerial.print( "\r\n" );
  }
  else {
    ota.badLines++;
    ota.fatal = true;
    otaSerial.print( "BAD " );
    otaSerial.print( lineNumber );
    otaSerial.print( "\r\n" );
    Serial.printf( "[FlasherX] BAD line %u: %s\n", lineNumber, ota.lastError );
  }
}

static void finalizeHexSession()
{
  if (!ota.inProgress) {
    otaSendLine( "HEX IDLE" );
    return;
  }

  ota.inProgress     = false;
  ota.handshakeReady = false;
  digitalWrite( led, LOW );

  uint32_t payloadBytes = 0;
  if (ota.hex.min != 0xFFFFFFFF && ota.hex.max > ota.hex.min) {
    payloadBytes = ota.hex.max - ota.hex.min;
  }

  bool success = (!ota.fatal && ota.badLines == 0);

  if (success && !ota.hex.eof) {
    setLastError( "EOF missing" );
    success = false;
  }
  if (success && ota.bytes == 0) {
    setLastError( "no data" );
    success = false;
  }

#if defined(KINETISK) || defined(KINETISL)
  if (success) {
    uint32_t value = *(uint32_t *)(0x40C + ota.bufferAddr);
    if (value != 0xfffff9de) {
      snprintf( ota.lastError, sizeof(ota.lastError), "FSEC %08lX", value );
      success = false;
    }
  }
#endif

  if (success && !check_flash_id( ota.bufferAddr, payloadBytes )) {
    setLastError( "missing target id" );
    success = false;
  }

  Serial.printf( "[FlasherX] HEX END lines=%d ok=%lu bad=%lu bytes=%lu\n",
                 ota.hex.lines,
                 (unsigned long)ota.okLines,
                 (unsigned long)ota.badLines,
                 (unsigned long)ota.bytes );

  if (success) {
    otaSerial.print( "HEX OK lines=" );
    otaSerial.print( ota.hex.lines );
    otaSerial.print( " bytes=" );
    otaSerial.print( ota.bytes );
    otaSerial.print( "\r\n" );
    otaSerial.print( "APPLIED\r\n" );
    otaSerial.flush();

    Serial.println( "[FlasherX] Applying update" );
    flash_move( FLASH_BASE_ADDR, ota.bufferAddr, payloadBytes );
    REBOOT;
  }
  else {
    otaSerial.print( "HEX ERR lines=" );
    otaSerial.print( ota.hex.lines );
    otaSerial.print( " bad=" );
    otaSerial.print( ota.badLines );
    if (ota.lastError[0]) {
      otaSerial.print( " msg=" );
      otaSerial.print( ota.lastError );
    }
    otaSerial.print( "\r\n" );

    Serial.print( "[FlasherX] HEX failed: " );
    Serial.println( ota.lastError );

    firmware_buffer_free( ota.bufferAddr, ota.bufferSize );
  }

  resetOtaSession();
}
