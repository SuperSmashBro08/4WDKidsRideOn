//******************************************************************************
// FlasherX -- firmware OTA update via Intel Hex file over serial or SD stream
//******************************************************************************
//
// Based on Flasher3 (Teensy 3.x) and Flasher4 (Teensy 4.x) by Jon Zeeff
//
// Jon Zeeff 2016, 2019, 2020 This code is in the public domain.
// Please retain my name in distributed copies, and let me know about any bugs
//
// I, Jon Zeeff, give no warranty, expressed or implied for this software and/or
// documentation provided, including, without limitation, warranty of
// merchantability and fitness for a particular purpose.
//
// WARNING: You can brick your Teensy with incorrect flash erase/write, such as
// incorrect flash config (0x400-40F). This code may or may not prevent that.
//
// 03/07/24 JRF - integrate ESP32 streaming uploader (HELLO/BEGIN HEX/L/END)
// 10/09/22 (v2.3) JWP - option for reading hex file from serial or SD
//   - move hex file support functions to new file FXUtil.cpp
//   - update_firmware() now takes two Stream* arguments ("in" and "out")
//   - FlasherX.ino lets user choose between hex file via serial or SD
// 09/01/22 (v2.2) JWP - change FlashTxx from CPP to C file
//   - rename FlashTxx.cpp to FlashTxx.c (resolve link error when calling from C)
//   - FlasherX.ino place #include "FlashTxx.h" inside extern "C" block
// 01/07/22 (v2.1) JWP - use TD 1.56 core functions for T4x wait/write/erase
//   - FlashTxx.h update FLASH_SIZE for Teensy Micromod from 8 to 16 MB
//   - option to artificially increase code size via const array (in flash)
// 11/18/21 JWP - bug fix in file FlashTXX.cpp
//   - fix logic in while loop in flash_block_write() in FlashTXX
// 10/27/21 JWP - add support for Teensy Micromod
//   - define macros for TEENSY_MICROMOD w/ same values as for TEENSY40
//   - update FLASH_SIZE for T4.1 and TMM from 2MB to 8MB
// JWP - merge of Flasher3/4 and new features
//   - FLASH buffer dynamically sized from top of existing code to FLASH_RESERVE
//   - optional RAM buffer option for T4.x via macro RAM_BUFFER_SIZE > 0
//   - Stream* (USB or UART) and buffer addr/size set at run-time
//   - incorporate Frank Boesing's FlashKinetis routines for T3.x
//   - add support for Teensy 4.1 and Teensy LC
//    This code is released into the public domain.
// JWP - Joe Pasquariello - modifications for T3.5 and T3.6 in Dec 2020
//    This code is released into the public domain
// Deb Hollenback at GiftCoder -- Modifications for teensy 3.5/3/6
//    This code is released into the public domain.
//    see https://forum.pjrc.com/threads/43165-Over-the-Air-firmware-updates-changes-for-flashing-Teensy-3-5-amp-3-6
// Jon Zeeff modifications
//    see https://forum.pjrc.com/threads/29607-Over-the-air-updates
// Original by Niels A. Moseley, 2015.
//    This code is released into the public domain.
//    https://namoseley.wordpress.com/2015/02/04/freescale-kinetis-mk20dx-series-flash-erasing/

#include <stdio.h>
#include <string.h>
#include "FXUtil.h"             // read_ascii_line(), hex file support
extern "C" {
  #include "FlashTxx.h"         // TLC/T3x/T4x/TMM flash primitives
}

static const int led = LED_BUILTIN;   // LED pin used for activity indication
static HardwareSerial& otaSerial = Serial2;

#define FLASHERX_VERSION "FlasherX v2.4"
static constexpr char OTA_TOKEN[] = "8d81ab8762c545dabe699044766a0b72";
static const uint32_t OTA_BAUD = 115200;

#define LARGE_ARRAY (0)         // 1 = define large array to test large hex file

#if (LARGE_ARRAY)
// nested arrays of integers to add code size for testing
#define A0 { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}  // 16  elements 64
#define A1 {A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0,A0}  // 256 elements 1KB
#define A2 {A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1,A1}  // 4K  elements 16KB
#define A3 {A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2,A2}  // 64K elements 256KB
#define A4 {A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3,A3}  // 1M  elements 4MB

// const variables reside in flash and get optimized out if never accessed
// use uint8_t -> 1MB, uint16_t -> 2MB, uint32_t -> 4MB, uint64_t -> 8MB)
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
  if (!strcmp( line, "VERSION" )) {
    otaSerial.print( "FW " );
    otaSerial.print( FLASHERX_VERSION );
    otaSerial.print( "\r\n" );
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
