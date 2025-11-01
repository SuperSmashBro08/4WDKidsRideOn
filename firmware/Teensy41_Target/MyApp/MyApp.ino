#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"
#include <stdarg.h>   // for SLOGF mirroring helper

// ---- Arduino preprocessor guardrails ----
enum Direction : uint8_t;   // forward declare
struct Desired;              // forward declare

// ========== App build tag ==========
#define APP_FW_VERSION   "app-0.2.15-gear-only"

// ========== LED blink ==========
#define LED_PIN          13
#define BLINK_MS         500

// ========== IO PINS ==========
static constexpr uint8_t PIN_FWD    = 10;   // forward switch input (active LOW)
static constexpr uint8_t PIN_REV    = 11;   // reverse switch input (active LOW)

// Pots
static constexpr uint8_t PIN_POT_S1 = 3;    // Steering 1  (car)
static constexpr uint8_t PIN_POT_T1 = 27;   // Throttle 1  (car)
static constexpr uint8_t PIN_POT_S2 = 5;    // Steering 2  (RC controller)
static constexpr uint8_t PIN_POT_T2 = 4;    // Throttle 2  (RC controller)

// ======== ADC helpers ========
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_MAX  = 4095;

// ======== Channel watching (optional) ========
struct WatchGroup { bool s1=false, s2=false, t1=false, t2=false, fnr=false; } g_watch;

// ======== Gear / FNR ========
enum FnrState : uint8_t { FNR_NEU=0, FNR_FWD=1, FNR_REV=2, FNR_FAULT=3 };
static FnrState g_fnr = FNR_NEU;

static inline const char* fnrToken(FnrState s) {
  switch (s) { case FNR_FWD: return "FWD"; case FNR_REV: return "REV";
               case FNR_NEU: return "NEU"; default: return "FAULT"; }
}
static inline const char* fnrWord(FnrState s) {
  switch (s) { case FNR_FWD: return "Forward"; case FNR_REV: return "Reverse";
               case FNR_NEU: return "Neutral"; default: return "Fault"; }
}

// Last sampled values
static uint16_t g_s1=0, g_s2=0, g_t1=0, g_t2=0;

// ======== Source selection ========
static bool g_srcRc = true;   // true => RC has priority

// ======== Drive/Steer policy ========
enum Direction : uint8_t { DIR_NEU, DIR_FWD, DIR_REV };

struct Desired {
  Direction driveDir;       // for M1/M2
  int16_t   driveCmd;       // -1000..+1000, 0=stop
  int16_t   steerCmd;       // -1000..+1000
  bool      steerNeutralHold;
};

// ======== Mirrored logging helper (USB + OTA console) ========
static void SLOGF(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);                 // USB Serial
  OtaConsole::printf("%s", buf);    // OTA to ESP32
}

// ======== Utilities ========
static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo; if (v > hi) return hi; return (int16_t)v;
}
static int16_t scaleSteer(uint16_t raw) {
  int32_t v = (int32_t)raw - 2048; v = (v * 1000) / 2048; return clamp_i16(v, -1000, 1000);
}
static int16_t scaleThrottle(uint16_t raw) {
  int32_t v = (int32_t)raw * 1000 / 4095; return clamp_i16(v, 0, 1000);
}

// ======== Sampling ========
static void samplePots() {
  g_s1 = analogRead(PIN_POT_S1);
  g_t1 = analogRead(PIN_POT_T1);
  g_s2 = analogRead(PIN_POT_S2);
  g_t2 = analogRead(PIN_POT_T2);
}

// RAW (no debounce) FNR read every tick (active LOW with pullups)
static void sampleFnr() {
  const bool fwd_low = (digitalRead(PIN_FWD) == LOW);
  const bool rev_low = (digitalRead(PIN_REV) == LOW);
  if ( fwd_low &&  rev_low) g_fnr = FNR_FAULT;
  else if ( fwd_low && !rev_low) g_fnr = FNR_FWD;
  else if (!fwd_low &&  rev_low) g_fnr = FNR_REV;
  else                           g_fnr = FNR_NEU;
}

// ======== Policy: compute & apply ========
static constexpr int16_t DEAD_BAND = 20; // tweak as needed

static void computeDesired(Desired& out) {
  const bool useRc = g_srcRc;

  const int16_t steer_local = scaleSteer(g_s1);
  const int16_t thr_local   = scaleThrottle(g_t1);  // 0..1000 (unsigned today)

  const int16_t steer_rc = scaleSteer(g_s2);
  const int16_t thr_rc   = scaleThrottle(g_t2);     // 0..1000 for now

  if (useRc) {
    out.steerCmd = steer_rc;
    out.steerNeutralHold = false;

    if (thr_rc > DEAD_BAND) { out.driveDir = DIR_FWD; out.driveCmd = thr_rc; }
    else                    { out.driveDir = DIR_NEU; out.driveCmd = 0;      }
  } else {
    out.steerCmd = steer_local;
    out.steerNeutralHold = false;
    out.driveCmd = thr_local;

    switch (g_fnr) {
      case FNR_FWD:  out.driveDir = DIR_FWD;  break;
      case FNR_REV:  out.driveDir = DIR_REV;  break;
      case FNR_NEU:
      case FNR_FAULT:
      default:
        out.driveDir = DIR_NEU;
        out.driveCmd = 0;
        out.steerNeutralHold = true;
        break;
    }
  }
}

// No "S ..." prints here — keep OTA console clean
static void applyOutputs(const Desired& d) {
  (void)d;
  // TODO: real motor outputs
}

// ======== Commands (optional) ========
static void handleUsbCommandsOnce() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        String verb, a1;
        int sp1 = line.indexOf(' ');
        if (sp1 < 0) { verb = line; }
        else { verb = line.substring(0, sp1); a1 = line.substring(sp1+1); a1.trim(); }
        verb.toUpperCase(); a1.toUpperCase();
        if (verb == "SRC") {
          if      (a1 == "?")      Serial.printf("[MyApp] SRC=%s\r\n", g_srcRc ? "rc" : "local");
          else if (a1 == "RC")     { g_srcRc = true;  Serial.println("[MyApp] SRC set to rc"); }
          else if (a1 == "LOCAL")  { g_srcRc = false; Serial.println("[MyApp] SRC set to local"); }
          else                     Serial.println("usage: SRC rc|local|?");
        } else {
          Serial.println("Unknown. Try: SRC rc|local|?");
        }
      }
      line = "";
    } else if (line.length() < 120) line += c;
  }
}

// ======== Setup / Loop ========
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PIN_FWD, INPUT_PULLUP);
  pinMode(PIN_REV, INPUT_PULLUP);

  analogReadResolution(12);
  analogReadAveraging(8);

  Serial.begin(115200);
  Serial2.begin(115200);       // to ESP32 (pins 7/8 on Teensy)

  // OTA + console
  OtaUpdater::begin(Serial2);
  OtaUpdater::setAppVersion(APP_FW_VERSION);
  OtaConsole::begin(Serial2);

  Serial.println("[MyApp] boot");
  SLOGF("S MyApp FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);

  delay(10);
  SLOGF("S BOOT_PINS fwd=%d rev=%d (0=LOW,1=HIGH)\r\n",
        (int)digitalRead(PIN_FWD), (int)digitalRead(PIN_REV));
}

void loop() {
  OtaUpdater::tick();
  OtaConsole::setEnabled(!OtaUpdater::inProgress());

  static uint32_t t_sample = 0;
  if (millis() - t_sample >= 100) {
    t_sample = millis();

    samplePots();
    sampleFnr();

    // ——— The five lines you want ———
    SLOGF("S gear=%s\r\n", fnrWord(g_fnr));     // Forward / Neutral / Reverse / Fault
    SLOGF("S STEER1=%u\r\n",    g_s1);
    SLOGF("S THROTTLE1=%u\r\n", g_t1);
    SLOGF("S STEER2=%u\r\n",    g_s2);
    SLOGF("S THROTTLE2=%u\r\n", g_t2);

    Desired des; computeDesired(des); applyOutputs(des);

    // Optional watch channels (kept quiet by default)
    if (g_watch.s1) SLOGF("S S1 raw=%u v=%.3f steer=%d\r\n", g_s1, (double)g_s1*ADC_VREF/ADC_MAX, (int)scaleSteer(g_s1));
    if (g_watch.s2) SLOGF("S S2 raw=%u v=%.3f steer=%d\r\n", g_s2, (double)g_s2*ADC_VREF/ADC_MAX, (int)scaleSteer(g_s2));
    if (g_watch.t1) SLOGF("S T1 raw=%u v=%.3f thr=%d\r\n",  g_t1, (double)g_t1*ADC_VREF/ADC_MAX, (int)scaleThrottle(g_t1));
    if (g_watch.t2) SLOGF("S T2 raw=%u v=%.3f thr=%d\r\n",  g_t2, (double)g_t2*ADC_VREF/ADC_MAX, (int)scaleThrottle(g_t2));
    if (g_watch.fnr) SLOGF("S FNR=%s\r\n", fnrToken(g_fnr));
  }

  static uint32_t t_led = 0; static bool led = false;
  if (!OtaUpdater::inProgress() && millis() - t_led >= BLINK_MS) {
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led ? HIGH : LOW);
  }

  handleUsbCommandsOnce();
}
