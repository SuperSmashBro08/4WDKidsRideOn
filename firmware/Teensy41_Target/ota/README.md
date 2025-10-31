# Teensy 4.1 Two-Stage OTA Loader

This directory contains a minimal two-stage over-the-air update system for the Teensy 4.1. The loader keeps a tiny stage-0 flasher at the beginning of QSPI flash while the user application occupies the "ACTIVE" slot. Stage-1 (the runtime application) streams Intel-HEX images from `Serial2` (pins **7 = RX2**, **8 = TX2**) into a staging slot, validates them, and signals stage-0 to perform the swap on the next reset.

## Memory map

```
0x6000_0000  +---------------------------------------------------------+
            |  Stage-0 flasher (512 KiB, never overwritten)           |
            +---------------------------------------------------------+
0x6008_0000  |  ACTIVE application slot (2 MiB)                        |
            |  - vector table relocates VTOR here                      |
            |  - stage-0 erases/programs entire slot as needed         |
            +---------------------------------------------------------+
0x6028_0000  |  STAGING slot (2 MiB)                                   |
            |  - stage-1 streams pages here                            |
            |  - stage-0 copies verified image into ACTIVE             |
            +---------------------------------------------------------+
0x6048_0000  |  Unused / spare flash                                   |
            +---------------------------------------------------------+
```

A small 4 KiB sector at `0x6007_F000` stores the persistent update flag. The same flag is mirrored into RTC general-purpose registers to survive brown-out events during a swap.

## Update flow

Stage-1 runtime listens for an ASCII protocol on `Serial2`:

```
ESP32 -> Teensy : "HEX BEGIN"\n
Teensy -> ESP32 : "ACK BEGIN"\n
ESP32 -> Teensy : ":...." Intel-HEX lines (with extended linear address support)
ESP32 -> Teensy : "HEX END"\n
Teensy -> ESP32 : "OK FLASH"\n
```

On success the runtime stores the CRC32 and payload size, raises the update flag, and calls `NVIC_SystemReset()`. If parsing fails or the CRC does not match, the runtime replies with `ERR ...` and aborts the session without touching the flag.

## Components

* `stage0/stage0_flasher.cpp` – ITCM/DTCM resident swapper that copies from staging into the active slot and then jumps to the application.
* `stage0/stage0_flasher.ld` – Linker script placing code in ITCM/DTCM while keeping load addresses inside the reserved flash window.
* `../teensystub/teensystub.ino` – Arduino sketch handling Serial2, Intel-HEX parsing, staging writes, and CRC validation.
* `../teensystub/*.h/.cpp` – shared helpers: flash map, low-level FlexSPI wrappers, CRC32, update flag storage, Intel-HEX parser, and application jump shim, colocated with the Arduino sketch to satisfy the IDE.
* `mock/` – lightweight host-side flash emulation plus parser/CRC tests (`test_hex_parser.cpp`).
* `make_check.py` – command-line tool that checks whether a HEX image fits within the ACTIVE and STAGING slots.

## Building & testing

* Stage-0 should be compiled separately using the provided linker script so that the binary is copied into ITCM/DTCM at runtime.
* Stage-1 builds as a regular Teensy 4.1 Arduino sketch.
* For host verification: `g++ -std=c++17 mock/test_hex_parser.cpp ../teensystub/crc32.cpp ../teensystub/intel_hex.cpp ../teensystub/flash_lowlevel.cpp mock/flash_mock.cpp -I../teensystub -Imock && ./a.out`
* Before flashing an image, run `./make_check.py <image.hex>` to confirm the layout is valid.
