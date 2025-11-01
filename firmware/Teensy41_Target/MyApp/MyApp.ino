#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"
#include <stdarg.h>   // for SLOGF mirroring helper

// ----- Teensy compatibility shims (IRAM_ATTR, digitalReadFast) -----
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

// ---- Arduino preprocessor guardrails ----
enum Direction : uint8_t;   // forward declare
struct Desired;              // forward declare

// ========== App build tag ==========
#define APP_FW_VERSION   "V 3.1.1"

// ========== LED blink ==========
#define LED_PIN          13
#define BLINK_MS         1000

// ========== IO PINS ==========
// Gear F/N/R switch inputs (active LOW with pullups)
static constexpr uint8_t PIN_FWD = 10;   // forward switch input (active LOW)
static constexpr uint8_t PIN_REV = 11;   // reverse switch input (active LOW)

// Local analog (car) pots
static constexpr uint8_t PIN_POT_S1 = 3;    // Steering 1 (analog)
static constexpr uint8_t PIN_POT_T1 = 27;   // Throttle 1 (analog)

// RC PWM inputs (through CD74HC4050 → 3.3V logic)
static constexpr uint8_t PIN_RC_S = 4;      // Steering 2 (RC PWM)
static constexpr uint8_t PIN_RC_T = 5;      // Throttle 2 (RC PWM)

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

// ======== Values ========
// Local analog raw (0..4095)
static uint16_t g_s1=0, g_t1=0;

// RC mapped commands: steer −1000..+1000, throttle 0..1000 (by default)
static int16_t g_s2_cmd = 0;
static int16_t g_t2_cmd = 0;

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
static int16_t scaleSteerAnalog(uint16_t raw) {
  // map 0..4095 -> -1000..+1000 around mid
  int32_t v = (int32_t)raw - 2048; v = (v * 1000) / 2048; return clamp_i16(v, -1000, 1000);
}
static int16_t scaleThrottleAnalog(uint16_t raw) {
  // 0..4095 -> 0..1000
  int32_t v = (int32_t)raw * 1000 / 4095; return clamp_i16(v, 0, 1000);
}

// ======== RC PWM capture (pins 5,4) ========
// Config: choose centered or unsigned throttle mapping
#define RC_THROTTLE_CENTERED 0   // 0 => 0..1000, 1 => -1000..+1000

// Pulse windows & deadband
static constexpr uint16_t RC_MIN_US = 1000;
static constexpr uint16_t RC_MID_US = 1500;
static constexpr uint16_t RC_MAX_US = 2000;
static constexpr uint16_t RC_DB_US  = 20;     // deadband around mid
static constexpr uint32_t RC_STALE_US = 50000; // 50 ms stale timeout

// Volatile state updated by ISRs
static volatile uint32_t rcS_rise_us = 0;
static volatile uint16_t rcS_width_us = 1500;
static volatile uint32_t rcS_last_update_us = 0;

static volatile uint32_t rcT_rise_us = 0;
static volatile uint16_t rcT_width_us = 1500;
static volatile uint32_t rcT_last_update_us = 0;

// Map helpers
static int16_t mapPwmSigned1000(uint16_t us) {
  if (us < RC_MIN_US) us = RC_MIN_US;
  if (us > RC_MAX_US) us = RC_MAX_US;
  int32_t x = (int32_t)us - RC_MID_US;
  if (abs(x) <= RC_DB_US) return 0;
  // 1000us => -1000, 1500 => 0, 2000 => +1000
  int32_t out = (x * 1000) / (RC_MAX_US - RC_MID_US); // / 500
  return clamp_i16(out, -1000, 1000);
}
static int16_t mapPwmUnsigned1000(uint16_t us) {
  if (us < RC_MIN_US) us = RC_MIN_US;
  if (us > RC_MAX_US) us = RC_MAX_US;
  // 1000us => 0, 2000us => 1000
  int32_t out = ((int32_t)us - RC_MIN_US) * 1000 / (RC_MAX_US - RC_MIN_US); // /1000
  if (out < 0) out = 0; if (out > 1000) out = 1000;
  return (int16_t)out;
}

// ISRs (Teensy-safe; IRAM_ATTR is a no-op here)
static void IRAM_ATTR isrRcSteer() {
  uint32_t now = micros();
  if (digitalReadFast(PIN_RC_S)) {
    rcS_rise_us = now;
  } else {
    uint32_t w = now - rcS_rise_us;
    if (w <= 3000) { rcS_width_us = (uint16_t)w; rcS_last_update_us = now; }
  }
}
static void IRAM_ATTR isrRcThrottle() {
  uint32_t now = micros();
  if (digitalReadFast(PIN_RC_T)) {
    rcT_rise_us = now;
  } else {
    uint32_t w = now - rcT_rise_us;
    if (w <= 3000) { rcT_width_us = (uint16_t)w; rcT_last_update_us = now; }
  }
}

// Sample + map RC into g_s2_cmd / g_t2_cmd (with stale fallback)
static void sampleRcPwm() {
  uint16_t s_us, t_us;
  uint32_t s_age, t_age, now = micros();
  noInterrupts();
  s_us = rcS_width_us;
  t_us = rcT_width_us;
  s_age = now - rcS_last_update_us;
  t_age = now - rcT_last_update_us;
  interrupts();

  if (s_age <= RC_STALE_US) g_s2_cmd = mapPwmSigned1000(s_us);
  else                      g_s2_cmd = 0;

#if RC_THROTTLE_CENTERED
  if (t_age <= RC_STALE_US) g_t2_cmd = mapPwmSigned1000(t_us);
  else                      g_t2_cmd = 0;
#else
  if (t_age <= RC_STALE_US) g_t2_cmd = mapPwmUnsigned1000(t_us);
  else                      g_t2_cmd = 0;
#endif
}

// ======== Sampling (local analog + gear) ========
static void samplePots() {
  g_s1 = analogRead(PIN_POT_S1);
  g_t1 = analogRead(PIN_POT_T1);
}

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

  // Local (analog) scaled
  const int16_t steer_local = scaleSteerAnalog(g_s1);
  const int16_t thr_local   = scaleThrottleAnalog(g_t1);  // 0..1000 (unsigned today)

  // RC already mapped to commands
  const int16_t steer_rc = g_s2_cmd;  // -1000..+1000
  const int16_t thr_rc   = g_t2_cmd;  // 0..1000 (or signed if you enable centered)

  if (useRc) {
    out.steerCmd = steer_rc;
    out.steerNeutralHold = false;

#if RC_THROTTLE_CENTERED
    if (thr_rc > +DEAD_BAND) { out.driveDir = DIR_FWD; out.driveCmd = thr_rc; }
    else if (thr_rc < -DEAD_BAND) { out.driveDir = DIR_REV; out.driveCmd = -thr_rc; }
    else { out.driveDir = DIR_NEU; out.driveCmd = 0; }
#else
    if (thr_rc > DEAD_BAND) { out.driveDir = DIR_FWD; out.driveCmd = thr_rc; }
    else                    { out.driveDir = DIR_NEU; out.driveCmd = 0;      }
#endif

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

// No "S ..." prints here — keep OTA console clean (we print in loop)
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

  // Gear pins with pullups
  pinMode(PIN_FWD, INPUT_PULLUP);
  pinMode(PIN_REV, INPUT_PULLUP);

  // Local analog
  analogReadResolution(12);
  analogReadAveraging(8);

  // RC PWM pins (inputs), attach ISRs
  pinMode(PIN_RC_S, INPUT);
  pinMode(PIN_RC_T, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_S), isrRcSteer,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_T), isrRcThrottle, CHANGE);

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

    // Local analog + gear + RC PWM
    samplePots();
    sampleFnr();
    sampleRcPwm();

    // ——— The five lines you want ———
    SLOGF("S gear=%s\r\n", fnrWord(g_fnr));            // Forward / Neutral / Reverse / Fault
    SLOGF("S STEER1=%u\r\n",    g_s1);                 // raw ADC 0..4095
    SLOGF("S THROTTLE1=%u\r\n", g_t1);                 // raw ADC 0..4095
    SLOGF("S STEER2=%d\r\n",    g_s2_cmd);             // RC mapped −1000..+1000
    SLOGF("S THROTTLE2=%d\r\n", g_t2_cmd);             // RC mapped 0..1000 (or signed if enabled)

    Desired des; computeDesired(des); applyOutputs(des);

    // Optional watch channels (quiet by default)
    if (g_watch.s1) SLOGF("S S1 raw=%u v=%.3f steer=%d\r\n", g_s1, (double)g_s1*ADC_VREF/ADC_MAX, (int)scaleSteerAnalog(g_s1));
    if (g_watch.t1) SLOGF("S T1 raw=%u v=%.3f thr=%d\r\n",  g_t1, (double)g_t1*ADC_VREF/ADC_MAX, (int)scaleThrottleAnalog(g_t1));
    if (g_watch.s2) SLOGF("S S2_CMD=%d\r\n", g_s2_cmd);
    if (g_watch.t2) SLOGF("S T2_CMD=%d\r\n", g_t2_cmd);
    if (g_watch.fnr) SLOGF("S FNR=%s\r\n", fnrToken(g_fnr));
  }

  static uint32_t t_led = 0; static bool led = false;
  if (!OtaUpdater::inProgress() && millis() - t_led >= BLINK_MS) {
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led ? HIGH : LOW);
  }

  handleUsbCommandsOnce();
}
