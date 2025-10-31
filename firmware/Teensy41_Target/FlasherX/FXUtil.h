//******************************************************************************
// FXUTIL.H -- FlasherX utility functions
//******************************************************************************
#ifndef FXUTIL_H_
#define FXUTIL_H_

#include <Arduino.h>

// Intel HEX bookkeeping used by both the interactive updater and the
// ESP32-driven OTA workflow.
typedef struct {
  char *data;           // pointer to array allocated elsewhere
  unsigned int addr;    // address in intel hex record
  unsigned int code;    // intel hex record type (0=data, etc.)
  unsigned int num;     // number of data bytes in intel hex record

  uint32_t base;        // base address to be added to intel hex 16-bit addr
  uint32_t min;         // min address in hex file
  uint32_t max;         // max address in hex file

  int eof;              // set true on intel hex EOF (code = 1)
  int lines;            // number of hex records received
} hex_info_t;

void hex_info_reset( hex_info_t *hex, char *data_buf );
int  parse_hex_line( const char *theline, char *bytes,
        unsigned int *addr, unsigned int *num, unsigned int *code );
int  process_hex_record( hex_info_t *hex );
void read_ascii_line( Stream *serial, char *line, int maxbytes );
void update_firmware( Stream *in, Stream *out,
                        uint32_t buffer_addr, uint32_t buffer_size );

#endif
