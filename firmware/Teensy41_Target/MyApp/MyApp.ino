#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"

// Bump this each build so you can see updates in logs and in the ESP32 `VERSION` reply
#define APP_FW_VERSION   "app-0.1.1"

// Pick your test LED (Teensy 4.1 onboard LED is 13)
#define LED_PIN          13

// Change this value between builds to prove OTA works (e.g., 800 -> 200)
#define BLINK_MS         50

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  // UART to ESP32 (pins 7/8): must match your wiring and ESP32 code
  Serial2.begin(115200);

  // Keep OTA forever
  OtaUpdater::begin(Serial2);

  // Tell the in-app OTA loader what to report for "FW ..."
  OtaUpdater::setAppVersion(APP_FW_VERSION);

  // Optional logging to ESP32 (prefix "S " so it never conflicts with OTA)
  OtaConsole::begin(Serial2);

  Serial.println("[MyApp] boot");
  OLOG("MyApp FW=%s  (blink=%d ms)", APP_FW_VERSION, BLINK_MS);
}

void loop() {
  // Always service OTA (cheap when idle)
  OtaUpdater::tick();

  // Optional: mute logs while an OTA is running
  OtaConsole::setEnabled(!OtaUpdater::inProgress());

  static uint32_t t_led = 0;
  static bool led_on = false;

  // Avoid blinking during flash (purely cosmetic)
  if (!OtaUpdater::inProgress()) {
    if (millis() - t_led >= BLINK_MS) {
      t_led = millis();
      led_on = !led_on;
      digitalWrite(LED_PIN, led_on ? HIGH : LOW);
      OLOG("heartbeat %lu, led=%d", (unsigned long)(millis()/1000), (int)led_on);
    }
  }

  // (Optional) forward anything you type on USB Serial to ESP32 console
  // OtaConsole::mirrorUsbStreamOnce();
}
