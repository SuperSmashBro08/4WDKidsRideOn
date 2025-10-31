# ESP32 ↔ Teensy 4.1 OTA Workflow

This repository contains two coordinated sketches that allow an ESP32-S3 to deliver over-the-air
(OTA) firmware updates to a Teensy 4.1-based TNC. The ESP32 hosts a small web page where you upload
the Intel HEX file exported by the Arduino IDE. The Teensy runs a purpose-built stub that
handshakes with the ESP32, receives the HEX records, writes them directly into its internal QSPI
flash, and automatically reboots into the freshly programmed image.

The sections below walk through every step from wiring to flashing.

## Hardware overview

* **Teensy 4.1** running the OTA stub sketch found under `firmware/Teensy41_Target/teensystub`.
* **ESP32-S3** running the uploader sketch in `firmware/ESP32_Uploader/esp32_uploader`.
* **Serial link** – connect ESP32 GPIO43 → Teensy RX2 (pin 7) and ESP32 GPIO44 ← Teensy TX2 (pin 8).
  Share ground between the boards. Keep the wiring short (<15 cm) for clean 115200 baud links.
* **Optional debug USB** – connect the Teensy micro‑USB to a computer to monitor verbose logs while
  testing. The ESP32 USB serial console is also helpful for Wi‑Fi and OTA diagnostics.

## Preparing the firmware

1. Open the Teensy project you want to deploy in the Arduino IDE and select **Teensy 4.1** as the
   target board.
2. Use **Sketch → Export compiled Binary**. This generates a `.hex` file in the sketch folder.
3. (Optional) Rename the HEX file with the build date or version so you can identify it easily on
the ESP32 upload page.

## Building and loading the OTA stub

1. In the Arduino IDE open `firmware/Teensy41_Target/teensystub/teensystub.ino`.
2. Verify that the `OTA_TOKEN` matches the value in `firmware/ESP32_Uploader/esp32_uploader/secrets.h`.
   Changing the token requires reflashing **both** boards.
3. Click **Upload** to flash the stub onto the Teensy 4.1. The stub prints its firmware version and
   provides a small serial console (commands: `help`, `status`, `version`, `reboot`).
4. Leave the stub in place. Every subsequent OTA update will completely replace it with your real
   TNC firmware, so be sure to keep a copy handy if you ever need to revert.

## Flashing workflow (ESP32 as the OTA server)

1. Flash the ESP32 with `firmware/ESP32_Uploader/esp32_uploader.ino`, update Wi‑Fi credentials in
   `secrets.h`, and allow it to join your network. The serial console prints the IP address and the
   optional mDNS hostname (`esp32-teensy.local`).
2. Visit `http://<esp32-ip>/` in a browser. The page exposes a single file upload form.
3. Click **Choose File**, select the Teensy `.hex` binary, and press **Upload & Flash**.
4. The ESP32 performs the following automatically:
   * Opens the serial link to the Teensy and sends `HELLO <token>`.
   * Streams each Intel HEX record line-by-line (`BEGIN HEX`, `L …`, `END`).
   * Displays a live log containing acknowledgements for every line.
5. The Teensy stub validates every record, accumulates data into 4 KB sectors, erases/programs its
   internal flash, and acknowledges each line with `OK <n>`. When the last record arrives it sends a
   summary (`HEX OK …`) followed by `APPLIED`, waits one second, and triggers a software reset to
   boot into the newly flashed firmware.
6. The ESP32 page updates with a success message showing transfer statistics. If anything fails you
   will see the Teensy-provided error reason (checksum mismatch, unsupported record type, flash
   write failure, missing EOF, etc.).

## Recovery and troubleshooting

* If the Teensy reports `HEX FAIL` immediately after `BEGIN HEX`, ensure the `OTA_TOKEN` matches and
  that both boards are wired correctly. Any lingering serial data from previous runs should be
  cleared by resetting the Teensy.
* When a transfer aborts mid-stream, the Teensy keeps the original firmware. The stub only commits
  data after successfully writing each 4 KB sector and it does not reset until the entire image
  passes validation.
* To recover the stub after flashing a faulty firmware, press and hold the Teensy **Program** button
  and upload the stub again from the Arduino IDE via USB. Once the stub is running you can retry the
  OTA process.
* Use the Teensy USB console command `status` to inspect how many lines, bytes, sectors, and pages
  were processed in the most recent session. This mirrors the data returned to the ESP32 uploader.

## Safety notes

* The stub only accepts data inside the Teensy 4.1 flash address window (`0x6000_0000 – 0x6020_0000`).
  Any record outside that range is rejected before any sector is erased.
* Flash programming runs with interrupts temporarily disabled while a page is written. The LED on
  pin 13 continues to heartbeat so you can see the board is alive during a long update.
* After a successful OTA the stub reboots automatically. No additional hardware toggling of the
  Program pin is required.

With the steps above the entire flow—uploading the HEX file, streaming it to the Teensy, flashing,
and rebooting—happens end-to-end without manual intervention.
