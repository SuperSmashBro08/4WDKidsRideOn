// ===== App identity =====
#define APP_NAME "ESP32_Uploader"
#define APP_VER  "v1.3.0"   // bump as you like

// ===== Teensy UART pins (unchanged) =====
#define TEENSY_TX 43
#define TEENSY_RX 44

// ===== Angle sensor wiring =====
// AS5600 on I2C  (SDA = 8, SCL = 9)
// Module's analog/PWM OUT -> GPIO 5  (optional)
#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      9
#define ANGLE_ANALOG_PIN 5

// ===== App print options =====
#define PRINT_OUT5_VOLTS   0   // 0 = hide analog voltage column, 1 = show it
#define PRINT_PERIOD_MS  100   // print every 100ms

// ===== Quiet window after steer direction change =====
// We skip fresh I2C reads for this period and reuse last-good value.
#define QUIET_MS          180  // try 120..250 as needed

#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"            // for esp_log_level_set(...)
#include "secrets.h"            // #define WIFI_SSID "...", WIFI_PASS "...", OTA_TOKEN "..."
#include "esp32_ota_hub.h"      // your OTA hub header

// ---- Web console logging shortcuts (mirror to USB + /esp32-console)
#define LOG(s)        OtaHub::ELOG(s)
#define LOGF(...)     OtaHub::ELOGF(__VA_ARGS__)

// ===== AS5600 registers / address =====
static const uint8_t  AS5600_ADDR     = 0x36;
static const uint8_t  REG_RAW_ANGLE   = 0x0C;   // 12-bit (0..4095)
static const uint8_t  REG_ANGLE       = 0x0E;   // scaled (also 12-bit)
static const uint32_t I2C_CLK_HZ      = 400000; // 400 kHz
static const uint8_t  I2C_RETRIES     = 3;      // retries per read
static const uint32_t I2C_XFER_TO_MS  = 40;     // per transfer timeout

// ===== Helpers =====
static inline float ticksToDeg(uint16_t t12) { return (360.0f * (float)t12) / 4096.0f; }
static inline int   wrapDist(int a, int b) { int d=b-a; while(d<-2048) d+=4096; while(d>=2048) d-=4096; return d; }

// Simple EMA (no templates to avoid past compile errors)
struct EMA {
  float y;
  float alpha;  // 0..1 (higher = snappier)
  EMA(float a=0.20f): y(NAN), alpha(a) {}
  float update(float x) {
    if (isnan(y)) y = x;
    else y = y + alpha * (x - y);
    return y;
  }
};

// ===== Steer flip quieting =====
// Call markSteerFlip() from your code at the instant you flip steer direction.
// (If you don't call it, we simply never enter the quiet window.)
static volatile uint32_t gSteerFlipMs = 0;
static inline void markSteerFlip() { gSteerFlipMs = millis(); }  // expose for your use

// ===== I2C read with retry (16-bit big-endian register) =====
static bool i2c_read16_retry(uint8_t addr, uint8_t reg, uint16_t& out) {
  for (uint8_t attempt = 0; attempt < I2C_RETRIES; ++attempt) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    // 'false' => repeated start
    if (Wire.endTransmission(false) != 0) { delay(1); continue; }

    int n = Wire.requestFrom((int)addr, 2, (int)true); // stop=true
    if (n == 2) {
      uint8_t hi = Wire.read();
      uint8_t lo = Wire.read();
      out = (uint16_t)((hi << 8) | lo);
      return true;
    }
    delay(1);
  }
  return false;
}

static bool as5600_read_raw(uint16_t& raw12) {
  uint16_t v;
  if (!i2c_read16_retry(AS5600_ADDR, REG_RAW_ANGLE, v)) return false;
  raw12 = (uint16_t)(v & 0x0FFF);
  return true;
}

// ===== Runtime state =====
static EMA   emaDeg(0.20f);     // smoothed degrees
#if PRINT_OUT5_VOLTS
static EMA   emaVolts(0.20f);   // smoothed analog volts
#endif
static uint16_t lastGoodRaw = 0;
static bool     haveGood    = false;
static unsigned long lastPrint = 0;

// ===== setup/loop =====
void setup() {
  Serial.begin(115200);
  delay(150);

  // Mute only the noisy ESP-IDF I2C driver lines, keep your own prints:
  esp_log_level_set("i2c",        ESP_LOG_WARN);
  esp_log_level_set("i2c.master", ESP_LOG_WARN);

  // Bring up OTA hub + web + consoles
  OtaHub::begin(WIFI_SSID, WIFI_PASS, OTA_TOKEN, "esp32-teensy");

  // I2C init
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLK_HZ);
  Wire.setTimeOut(I2C_XFER_TO_MS);

#if PRINT_OUT5_VOLTS
  analogReadResolution(12);         // 0..4095
  pinMode(ANGLE_ANALOG_PIN, INPUT);
#endif

  LOG("[AS5600] Ready. Keys: (none needed) — printing every 100ms.");
}

void loop() {
  // keep OTA hub alive
  OtaHub::loop();

  const unsigned long now = millis();

  // Decide whether to query I2C this cycle
  const bool inQuiet = (now - gSteerFlipMs) < QUIET_MS;

  uint16_t raw = 0;
  bool readOK = false;

  if (!inQuiet) {
    // Try a live I2C read
    readOK = as5600_read_raw(raw);
    if (readOK) { lastGoodRaw = raw; haveGood = true; }
  } else {
    // In quiet zone: reuse last-good value (no bus traffic during motor surge)
    if (haveGood) { raw = lastGoodRaw; readOK = true; }
  }

  // Convert to degree; if no data yet, keep deg as NaN (prints "NaN")
  float deg = NAN;
  if (readOK) {
    deg = ticksToDeg(raw);
    deg = emaDeg.update(deg);
  }

#if PRINT_OUT5_VOLTS
  // Optional analog OUT reading (smoothed)
  int   adc    = analogRead(ANGLE_ANALOG_PIN);
  float volts  = (3.3f * (float)adc) / 4095.0f;
  float v_sm   = emaVolts.update(volts);
#endif

  // Console print (mirrors to USB + /esp32-console)
  if ((now - lastPrint) >= PRINT_PERIOD_MS) {
    lastPrint = now;

    if (readOK) {
    #if PRINT_OUT5_VOLTS
      LOGF("AS5600 raw=%u  deg=%.2f  OUT5=%.3fV", raw, deg, v_sm);
    #else
      LOGF("AS5600 raw=%u  deg=%.2f", raw, deg);
    #endif
    } else {
    #if PRINT_OUT5_VOLTS
      LOGF("AS5600 raw=ERR  deg=NaN  OUT5=%.3fV", v_sm);
    #else
      LOG("AS5600 raw=ERR  deg=NaN");
    #endif
    }
  }
}

/* ===========================
   How to tell the code about a steer flip
   ---------------------------
   Wherever you flip your steering direction (the exact moment the H-bridge/MOSFETs
   change polarity), call:

       markSteerFlip();

   That starts a QUIET_MS-long window where we reuse the last good angle instead of
   talking to the I²C bus. This avoids transient NACKs and keeps your serial/web output
   smooth through the surge. Tune QUIET_MS at the top if needed.
*/
