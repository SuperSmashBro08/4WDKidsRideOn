#!/usr/bin/env python3
"""Sanity check for staged Teensy 4.1 OTA images."""

from __future__ import annotations

import argparse
import pathlib
import sys

FLASH_BASE_ACTIVE = 0x60000000 + 512 * 1024
FLASH_SIZE_ACTIVE = 2 * 1024 * 1024
FLASH_SIZE_STAGING = 2 * 1024 * 1024


def parse_hex(path: pathlib.Path) -> tuple[int, int]:
    ela = 0
    min_addr: int | None = None
    max_addr = 0
    with path.open("r", encoding="utf-8") as fp:
        for lineno, raw in enumerate(fp, 1):
            line = raw.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise SystemExit(f"Line {lineno} missing colon")
            line = line[1:]
            if len(line) < 10:
                raise SystemExit(f"Line {lineno} too short")
            length = int(line[0:2], 16)
            address = int(line[2:6], 16)
            rectype = int(line[6:8], 16)
            data = bytes.fromhex(line[8:8 + length * 2])
            checksum = int(line[8 + length * 2:8 + length * 2 + 2], 16)
            calc = (length + (address >> 8) + (address & 0xFF) + rectype + sum(data)) & 0xFF
            calc = ((~calc + 1) & 0xFF)
            if calc != checksum:
                raise SystemExit(f"Checksum mismatch on line {lineno}")
            if rectype == 0x00:
                absolute = ela + address
                if min_addr is None:
                    min_addr = absolute
                else:
                    min_addr = min(min_addr, absolute)
                max_addr = max(max_addr, absolute + length)
            elif rectype == 0x01:
                break
            elif rectype == 0x04:
                if length != 2:
                    raise SystemExit(f"Invalid ELA length on line {lineno}")
                ela = int.from_bytes(data, byteorder="big") << 16
            else:
                # ignore other record types for sizing
                continue
    if min_addr is None:
        return 0, 0
    return min_addr, max_addr


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hex", type=pathlib.Path, help="Intel HEX file to validate")
    args = parser.parse_args()

    min_addr, max_addr = parse_hex(args.hex)
    image_size = max_addr - min_addr

    active_start = FLASH_BASE_ACTIVE
    active_end = FLASH_BASE_ACTIVE + FLASH_SIZE_ACTIVE
    fits_active_absolute = active_start <= min_addr and max_addr <= active_end
    fits_active_relative = image_size <= FLASH_SIZE_ACTIVE and max_addr <= FLASH_SIZE_ACTIVE

    if not (fits_active_absolute or fits_active_relative):
        print("ERROR: image does not fit within active region", file=sys.stderr)
        return 1
    if image_size > FLASH_SIZE_STAGING:
        print("ERROR: image does not fit within staging region", file=sys.stderr)
        return 1

    print(f"Image base: 0x{min_addr:08X}")
    print(f"Image size: {image_size} bytes")
    print("Active region: OK")
    print("Staging region: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
