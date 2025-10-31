#include "FXUtil.h"
#include <string.h>
#include <stdio.h>

extern "C" {
  #include "FlashTxx.h"
}

void hex_info_reset(hex_info_t* hex, char* data_buf) {
  hex->data  = data_buf;
  hex->addr  = 0u;
  hex->code  = 0u;
  hex->num   = 0u;
  hex->base  = 0u;
  hex->min   = 0xFFFFFFFFu;
  hex->max   = 0u;
  hex->eof   = 0;
  hex->lines = 0;
}

void read_ascii_line(Stream* serial, char* line, int maxbytes) {
  int c = 0, nchar = 0;

  // eat leading CR/LF
  while (serial->available()) {
    c = serial->read();
    if (c != '\n' && c != '\r') { line[nchar++] = c; break; }
  }
  // read until CR/LF or maxbytes
  while (nchar < maxbytes && !(c == '\n' || c == '\r')) {
    if (serial->available()) {
      c = serial->read();
      line[nchar++] = c;
    }
  }
  if (nchar == 0) { line[0] = 0; return; }
  line[nchar-1] = 0; // strip newline
}

int parse_hex_line(const char* theline, char* bytes,
                   unsigned int* addr, unsigned int* num, unsigned int* code) {
  unsigned sum, len, cksum;
  const char* ptr;
  int temp;

  *num = 0;
  if (theline[0] != ':') return 0;
  if (strlen(theline) < 11) return 0;

  ptr = theline + 1;
  if (!sscanf(ptr, "%02x", &len)) return 0; ptr += 2;
  if (strlen(theline) < (11 + (len * 2))) return 0;
  if (!sscanf(ptr, "%04x", addr)) return 0; ptr += 4;
  if (!sscanf(ptr, "%02x", code)) return 0; ptr += 2;

  sum = (len & 255) + (((*addr) >> 8) & 255) + ((*addr) & 255) + ((*code) & 255);
  while (*num != len) {
    if (!sscanf(ptr, "%02x", &temp)) return 0;
    bytes[*num] = temp;
    ptr += 2;
    sum += (unsigned)(bytes[*num] & 255);
    (*num)++;
    if (*num >= 256) return 0;
  }
  if (!sscanf(ptr, "%02x", &cksum)) return 0;
  if (((sum & 255) + (cksum & 255)) & 255) return 0; // checksum error
  return 1;
}

int process_hex_record(hex_info_t* hex) {
  if (hex->code == 0u) { // data
    uint32_t end = hex->base + hex->addr + hex->num;
    uint32_t beg = hex->base + hex->addr;
    if (end > hex->max) hex->max = end;
    if (beg < hex->min) hex->min = beg;
  } else if (hex->code == 1u) { // EOF
    hex->eof = 1;
  } else if (hex->code == 2u) { // extended segment address
    hex->base = (((uint8_t)hex->data[0] << 8) | (uint8_t)hex->data[1]) << 4;
  } else if (hex->code == 3u) { // start segment address (unused)
    return 1;
  } else if (hex->code == 4u) { // extended linear address
    hex->base = (((uint8_t)hex->data[0] << 8) | (uint8_t)hex->data[1]) << 16;
  } else if (hex->code == 5u) { // start linear address
    hex->base =  ((uint8_t)hex->data[0] << 24) |
                 ((uint8_t)hex->data[1] << 16) |
                 ((uint8_t)hex->data[2] <<  8) |
                 ((uint8_t)hex->data[3] <<  0);
  } else {
    return 1;
  }
  return 0;
}

void update_firmware(Stream* in, Stream* out,
                     uint32_t buffer_addr, uint32_t buffer_size) {
  static char line[96];                               // HEX line buffer
  static char data[32] __attribute__((aligned(8)));   // parsed data bytes
  hex_info_t hex;

  hex_info_reset(&hex, data);
  out->printf("reading hex lines...\n");

  while (!hex.eof) {
    read_ascii_line(in, line, sizeof(line));
    if (!line[0]) { yield(); continue; }

    // If in==out==Serial (USB loopback sample), echo line to improve reliability
    if (in == out && out == (Stream*)&Serial) { out->printf("%s\n", line); out->flush(); }

    unsigned int uaddr = 0, unum = 0, ucode = 0;
    if (parse_hex_line(line, hex.data, &uaddr, &unum, &ucode) == 0) {
      out->printf("abort - bad hex line %s\n", line);
      return;
    }
    hex.addr = uaddr; hex.num = unum; hex.code = ucode;

    if (process_hex_record(&hex) != 0) {
      out->printf("abort - invalid hex code %u\n", hex.code);
      return;
    } else if (hex.code == 0u) { // data record
      uint32_t addr = buffer_addr + hex.base + hex.addr - FLASH_BASE_ADDR;
      if (hex.max > (FLASH_BASE_ADDR + buffer_size)) {
        out->printf("abort - max address %08lX too large\n", (unsigned long)hex.max);
        return;
      } else if (!IN_FLASH(buffer_addr)) {
        memcpy((void*)addr, (void*)hex.data, hex.num);
      } else {
        int err = flash_write_block(addr, hex.data, hex.num);
        if (err) { out->printf("abort - error %02X in flash_write_block()\n", err); return; }
      }
    }
    hex.lines++;
  }

  out->printf("\nhex file: %d lines %lu bytes (%08lX - %08lX)\n",
              hex.lines,
              (unsigned long)(hex.max - hex.min),
              (unsigned long)hex.min,
              (unsigned long)hex.max);

#if defined(KINETISK) || defined(KINETISL)
  uint32_t value = *(uint32_t *)(0x40C + buffer_addr);
  if (value != 0xFFFFF9DE) { out->printf("abort - FSEC %08lX should be FFFFF9DE\n", value); return; }
  else { out->printf("FSEC ok: %08lX\n", value); }
#endif

  if (!check_flash_id(buffer_addr, hex.max - hex.min)) {
    out->printf("abort - new code missing target ID %s\n", FLASH_ID);
    return;
  } else {
    out->printf("target ID ok: %s\n", FLASH_ID);
  }

  out->printf("calling flash_move() to load new firmware...\n");
  out->flush();

  flash_move(FLASH_BASE_ADDR, buffer_addr, hex.max - hex.min);
  REBOOT; // should not return
}
