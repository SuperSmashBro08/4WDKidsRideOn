// ===== App identity =====
#define APP_NAME "ESP32_Uploader"
#define APP_VER  "v1.2.1"   // <- bump as you like

// ===== Teensy UART pins (unchanged) =====
#define TEENSY_TX 43
#define TEENSY_RX 44

// ===== Angle sensor wiring =====
// AS5600 on I2C (SDA=8, SCL=9). Module's analog/PWM OUT -> GPIO 5
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define ANGLE_ANALOG_PIN 5   // reads module's OUT (0..3.3V), optional

#include <Wire.h>
#include "secrets.h"        // #define WIFI_SSID "...", WIFI_PASS "...", OTA_TOKEN "..."
#include "esp32_ota_hub.h"  // your OTA hub header

// ===== Minimal AS5600 helpers =====
static const uint8_t AS5600_ADDR = 0x36;     // default
static const uint8_t REG_RAW_ANGLE = 0x0C;   // RAW_ANGLE (12-bit, 0..4095)
static const uint8_t REG_ANGLE     = 0x0E;   // ANGLE (after internal scaling)

uint16_t as5600_read16(uint8_t reg)
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return 0xFFFF;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return (uint16_t)((hi << 8) | lo);
}

uint16_t as5600_raw()
{
  // 12-bit data is in lower bits of RAW_ANGLE high/low
  uint16_t v = as5600_read16(REG_RAW_ANGLE);
  if (v == 0xFFFF) return 0xFFFF;
  return (uint16_t)(v & 0x0FFF);
}

uint16_t as5600_angle()
{
  uint16_t v = as5600_read16(REG_ANGLE);
  if (v == 0xFFFF) return 0xFFFF;
  return (uint16_t)(v & 0x0FFF);
}

float ticksToDeg(uint16_t ticks12) { return (360.0f * (float)ticks12) / 4096.0f; }

// ===== Simple smoothing for analog & i2c angle =====
template<typename T>
T lerp(T a, T b, float t){ return a + (T)((b - a) * t); }

struct EMA {
  float y = NAN;
  float alpha;     // 0.0..1.0 (higher = snappier)
  EMA(float a=0.25f):alpha(a){}
  float update(float x){
    if (isnan(y)) y = x;
    else y = y + alpha * (x - y);
    return y;
  }
};

// ===== Interactive mapping capture =====
// Press 'l' (left), 'c' (center), 'r' (right) in Serial Monitor to capture.
// Press 's' to show current captures & mapping suggestion.
// Press 'i' to toggle invert direction (helpful if your GPO direction is flipped).
// Press 'x' to clear captures.
struct Cal {
  bool haveL=false, haveC=false, haveR=false, invert=false;
  uint16_t rawL=0, rawC=0, rawR=0;
  void clear(){ haveL=haveC=haveR=false; }
} cal;

void printHelp()
{
  Serial.println(F("\n[AS5600] Keys: l=cap LEFT, c=cap CENTER, r=cap RIGHT, i=toggle INVERT, s=show, x=clear\n"));
}

void showCal()
{
  Serial.println(F("=== Captured points (raw ticks 0..4095) ==="));
  if (cal.haveL) Serial.printf("Left:   %u (%.2f deg)\n", cal.rawL, ticksToDeg(cal.rawL));
  else           Serial.println("Left:   (not set)");
  if (cal.haveC) Serial.printf("Center: %u (%.2f deg)\n", cal.rawC, ticksToDeg(cal.rawC));
  else           Serial.println("Center: (not set)");
  if (cal.haveR) Serial.printf("Right:  %u (%.2f deg)\n", cal.rawR, ticksToDeg(cal.rawR));
  else           Serial.println("Right:  (not set)");

  Serial.printf("Invert: %s\n", cal.invert ? "YES" : "no");

  // If we have L/C/R, print mapping snippet you can paste into code later
  if (cal.haveL && cal.haveC && cal.haveR) {
    Serial.println(F("\n--- Suggested mapping snippet ---"));
    Serial.printf("// Paste into code:\n");
    Serial.printf("static const uint16_t RAW_LEFT=%u, RAW_CENTER=%u, RAW_RIGHT=%u;\n", cal.rawL, cal.rawC, cal.rawR);
    Serial.printf("static const bool     ANGLE_INVERT=%s;\n", cal.invert?"true":"false");
    Serial.println(F(
      "float mapAngleToSpan(uint16_t raw){\n"
      "  // Wrap-safe distances on circular scale\n"
      "  auto dist = [](int a,int b){\n"
      "    int d = (int)b - (int)a;\n"
      "    while(d <  -2048) d += 4096;\n"
      "    while(d >=  2048) d -= 4096;\n"
      "    return d;\n"
      "  };\n"
      "  int l = dist(RAW_CENTER, RAW_LEFT);\n"
      "  int r = dist(RAW_CENTER, RAW_RIGHT);\n"
      "  int x = dist(RAW_CENTER, raw);\n"
      "  // normalize to -1..+1 (left=-1, center=0, right=+1)\n"
      "  float spanL = (l==0) ? 1.0f : (float)x / (float)l;   // negative side\n"
      "  float spanR = (r==0) ? 1.0f : (float)x / (float)r;   // positive side\n"
      "  float n = (x<=0) ? spanL : spanR;\n"
      "  if (ANGLE_INVERT) n = -n;\n"
      "  if (n < -1) n = -1; if (n > 1) n = 1;\n"
      "  return n;\n"
      "}\n"
    ));
  }
}

// Wrap-safe delta on 12-bit circle
int wrapDist(int a, int b){
  int d = b - a;
  while (d <  -2048) d += 4096;
  while (d >=  2048) d -= 4096;
  return d;
}

// ===== Runtime state =====
EMA emaDeg(0.2f);     // smoothing for I2C angle (degrees)
EMA emaAin(0.2f);     // smoothing for analog volts
unsigned long lastPrintMs = 0;

void setup()
{
  // Bring up the OTA hub + web UI + consoles
  OtaHub::begin(WIFI_SSID, WIFI_PASS, OTA_TOKEN, "esp32-teensy");

  // Your app-specific init
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  analogReadResolution(12);           // 0..4095
  pinMode(ANGLE_ANALOG_PIN, INPUT);   // ensure input
  Serial.println(F("[AS5600] Ready. Press 's' for help."));
  printHelp();
}

void loop()
{
  OtaHub::loop();

  // ---- Handle Serial commands for capture ----
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch=='\r' || ch=='\n') continue;
    uint16_t current = as5600_raw();
    switch (ch) {
      case 'l': cal.rawL = current; cal.haveL = true; Serial.printf("[cap] LEFT   = %u (%.2f deg)\n", current, ticksToDeg(current)); break;
      case 'c': cal.rawC = current; cal.haveC = true; Serial.printf("[cap] CENTER = %u (%.2f deg)\n", current, ticksToDeg(current)); break;
      case 'r': cal.rawR = current; cal.haveR = true; Serial.printf("[cap] RIGHT  = %u (%.2f deg)\n", current, ticksToDeg(current)); break;
      case 'i': cal.invert = !cal.invert; Serial.printf("[cap] INVERT toggled -> %s\n", cal.invert ? "YES" : "no"); break;
      case 'x': cal.clear(); Serial.println("[cap] cleared L/C/R"); break;
      case 's': default: showCal(); break;
    }
  }

  // ---- Read I2C angle ----
  uint16_t raw = as5600_raw();     // or as5600_angle() if you prefer scaled angle
  float deg = (raw == 0xFFFF) ? NAN : ticksToDeg(raw);
  float deg_sm = isnan(deg) ? deg : emaDeg.update(deg);

  // ---- Read analog OUT on GPIO 5 (optional) ----
  int adc = analogRead(ANGLE_ANALOG_PIN);      // 0..4095
  float volts = (3.3f * (float)adc) / 4095.0f;
  float volts_sm = emaAin.update(volts);

  // ---- Print status at 10 Hz ----
  unsigned long now = millis();
  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;

    // If captures present, show a normalized span too
    float span = NAN;
    if (cal.haveC && (cal.haveL || cal.haveR) && raw != 0xFFFF) {
      // build normalized -1..+1 value around center with wrap safety
      int x = wrapDist((int)cal.rawC, (int)raw);
      float n = 0.0f;
      if (x <= 0 && cal.haveL) {
        int l = wrapDist((int)cal.rawC, (int)cal.rawL);
        n = (l==0) ? 0.0f : (float)x / (float)l; // negative side
      } else if (x > 0 && cal.haveR) {
        int r = wrapDist((int)cal.rawC, (int)cal.rawR);
        n = (r==0) ? 0.0f : (float)x / (float)r; // positive side
      }
      if (cal.invert) n = -n;
      if (n < -1) n = -1; if (n > 1) n = 1;
      span = n;
    }

    // Output line: raw ticks, deg (smoothed), analog volts (smoothed), optional span
    Serial.print(F("AS5600 raw="));
    if (raw == 0xFFFF) Serial.print(F("ERR"));
    else               Serial.print(raw);

    Serial.print(F("  deg="));
    if (isnan(deg_sm)) Serial.print(F("NaN"));
    else               Serial.printf("%.2f", deg_sm);

    Serial.print(F("  OUT5="));
    Serial.printf("%.3fV", volts_sm);

    if (!isnan(span)) {
      Serial.print(F("  span="));
      Serial.printf("%.3f", span);
    }

    Serial.println();
  }
}
