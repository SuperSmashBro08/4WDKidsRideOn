// ============ MyApp.ino (V 3.5.0 RAW + Steering/Throttle source handover + thirds) ============
//
// - RAW snapshot line every ~30 ms (S1/T1/S2/T2/FNR/AS)
// - ESP32 angle stream over the SAME UART (Serial2) via OtaUpdater hook
//   expected line from ESP32: "AS,<raw>,<deg>"
// - Motor driver: BTS7960-style on pins 15..18 (LPWM/RPWM/L_EN/R_EN)
// - 20 kHz PWM for quiet steering drive
// - Steering selection by thirds (S1 or S2), per your ranges:
//      S1: 420..2870 (left/center/right thirds)
//      S2: 1000..2000 µs (left/center/right thirds)
// - Control source switching:
//      • Default source: S1/T1
//      • Hold T2 reverse (≤1200 µs) continuously for 3 s → switch to S2/T2
//      • After handover, T2 throttle must pass through neutral (arm) before it takes effect
//      • When BOTH T1 and T2 are neutral (deadbanded) for 10 s → switch back to S1/T1
// - Motor drive is disabled (STEER_DRIVE_ENABLE=0) so we can verify targets/logic safely
//
// ------------------------------------------------------------------------------------------------------

#ifndef ENABLE_TEENSY_BINARY_TELEM
#define ENABLE_TEENSY_BINARY_TELEM 0
#endif

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

// ---------- App identity ----------
#define APP_FW_VERSION   "V 3.5.0"

// ---------- Blink ----------
#define LED_PIN          13
#define BLINK_MS         500

// ---------- Raw-mirror mode switch ----------
#define RAW_MIRROR_MODE  1   // keep raw stream on

// ---------- IO pins ----------
static constexpr uint8_t PIN_FWD    = 10;
static constexpr uint8_t PIN_REV    = 11;
static constexpr uint8_t PIN_POT_S1 = 14;  // steering analog (wheel/pot)
static constexpr uint8_t PIN_POT_T1 = 27;  // throttle analog (foot pedal)
static constexpr uint8_t PIN_RC_S   = 4;   // RC steer PWM in (S2)
static constexpr uint8_t PIN_RC_T   = 5;   // RC throttle PWM in (T2)

// ---------- ADC helpers ----------
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_MAX  = 4095;

// ---------- Logging sink control ----------
enum class LogSink : uint8_t { OTA, USB, BOTH };
static LogSink g_logSink = LogSink::OTA;   // OTA-only by default (avoid dupes)

// Force every line sent to ESP32 console to start with "S "
static void LOG_raw(const char* s){
  const bool alreadyS = (s && s[0]=='S' && s[1]==' ');
  if (g_logSink==LogSink::OTA || g_logSink==LogSink::BOTH){
    if (alreadyS) OtaConsole::printf("%s", s);
    else          OtaConsole::printf("S %s", s);
  }
  if (g_logSink==LogSink::USB || g_logSink==LogSink::BOTH) Serial.print(s);
}
static void SLOGF(const char* fmt, ...){
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  LOG_raw(buf);
}

// ================= NAMESPACE =================
namespace MM {

// ===== Gear / FNR =====
enum FnrState : uint8_t { FNR_NEU=0, FNR_FWD=1, FNR_REV=2, FNR_FAULT=3 };
static FnrState g_fnr = FNR_NEU;
static inline const char* fnrWord(FnrState s){
  switch(s){ case FNR_FWD: return "forward"; case FNR_REV: return "reverse";
             case FNR_NEU: return "neutral"; default: return "fault"; }
}

// ===== Live values =====
static uint16_t g_s1=0, g_t1=0;                  // raw analog (S1 wheel, T1 pedal)
static volatile uint32_t rcS_rise_us=0, rcS_last_update_us=0;
static volatile uint16_t rcS_width_us=1500;      // S2 µs
static volatile uint32_t rcT_rise_us=0, rcT_last_update_us=0;
static volatile uint16_t rcT_width_us=1500;      // T2 µs
static int16_t  g_s2_us=1500, g_t2_us=1500;      // copies each tick

// ===== RC PWM capture (interrupt) =====
static constexpr uint16_t RC_MIN_US=1000, RC_MID_US=1500, RC_MAX_US=2000, RC_DB_US=20;

static void IRAM_ATTR isrRcSteer(){
  uint32_t now=micros();
  if(digitalReadFast(PIN_RC_S)) rcS_rise_us=now;
  else { uint32_t w=now-rcS_rise_us; if(w<=3000){ rcS_width_us=(uint16_t)w; rcS_last_update_us=now; } }
}
static void IRAM_ATTR isrRcThrottle(){
  uint32_t now=micros();
  if(digitalReadFast(PIN_RC_T)) rcT_rise_us=now;
  else { uint32_t w=now-rcT_rise_us; if(w<=3000){ rcT_width_us=(uint16_t)w; rcT_last_update_us=now; } }
}

// ===== Convenience maps =====
static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi){ if(v<lo) return lo; if(v>hi) return hi; return (int16_t)v; }

static inline int16_t mapRcSteer_us_to_1000(int16_t us){
  if(us<RC_MIN_US) us=RC_MIN_US; if(us>RC_MAX_US) us=RC_MAX_US;
  int32_t x=(int32_t)us-RC_MID_US; if (abs(x)<=RC_DB_US) return 0;
  return clamp_i16((x*1000)/(RC_MAX_US-RC_MID_US), -1000, 1000);
}
static inline int16_t mapRcThr_us_to_1000(int16_t us){
  if(us<RC_MIN_US) us=RC_MIN_US; if(us>RC_MAX_US) us=RC_MAX_US;
  int32_t out=((int32_t)us-RC_MIN_US)*1000/(RC_MAX_US-RC_MIN_US);
  if(out<0)out=0; if(out>1000)out=1000; return (int16_t)out;
}

// A normalized view for T1 as 0..1000 (for neutral detection convenience)
static inline int16_t mapT1_to_1000(uint16_t t1raw){
  return (int16_t)((uint32_t)t1raw * 1000u / 4095u);
}

// ===== Samples =====
static void sampleAnalog(){ g_s1=analogRead(PIN_POT_S1); g_t1=analogRead(PIN_POT_T1); }
static void sampleFnr(){
  const bool fwd_low=(digitalRead(PIN_FWD)==LOW), rev_low=(digitalRead(PIN_REV)==LOW);
  if( fwd_low &&  rev_low) g_fnr=FNR_FAULT;
  else if( fwd_low && !rev_low) g_fnr=FNR_FWD;
  else if(!fwd_low &&  rev_low) g_fnr=FNR_REV;
  else g_fnr=FNR_NEU;
}

} // namespace MM

// ======= Angle stream (from ESP32) + helpers ==================================
static volatile uint16_t g_as_raw = 0;
static volatile float    g_as_deg = NAN;
static volatile uint32_t g_as_ms  = 0;

static inline bool angleFresh(uint32_t maxAgeMs);
static bool angleSnapshot(uint16_t& rawOut, float& degOut);

extern "C" bool OtaUserHandleLine(const char* line) {
  if (!line || !*line) return false;
  if (strcmp(line, "PING") == 0)    { Serial2.println("PONG"); return true; }
  if (strcmp(line, "VERSION") == 0) { Serial2.print("FW "); Serial2.println(APP_FW_VERSION); Serial2.println("FLASHERX READY"); return true; }

  // Angle packets from ESP32: AS,<raw>,<deg>
  if (line[0]=='A' && line[1]=='S' && line[2]==',') {
    const char* p = line + 3;
    const char* comma = strchr(p, ','); if (!comma) return false;
    uint16_t raw = (uint16_t)strtoul(p, nullptr, 10);
    float    deg = atof(comma + 1);
    noInterrupts(); g_as_raw=raw; g_as_deg=deg; g_as_ms=millis(); interrupts();
    return true;
  }
  return false;
}

static inline bool angleFresh(uint32_t maxAgeMs) {
  uint32_t age = millis() - g_as_ms;
  return (g_as_ms != 0) && (age <= maxAgeMs) && !isnan(g_as_deg);
}
static bool angleSnapshot(uint16_t& rawOut, float& degOut) {
  uint16_t raw; float deg; uint32_t stamp;
  noInterrupts(); raw=g_as_raw; deg=g_as_deg; stamp=g_as_ms; interrupts();
  if (stamp == 0 || isnan(deg)) return false;
  if ((millis() - stamp) > 200)  return false;
  rawOut = raw; degOut = deg; return true;
}

// ======= Telemetry (optional binary to ESP32) ==================================
static inline int16_t toCenti(float v){
  float scaled = v * 100.0f; scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
  long val = (long)scaled;
  if (val > INT16_MAX) val = INT16_MAX;
  if (val < INT16_MIN) val = INT16_MIN;
  return (int16_t)val;
}

#if ENABLE_TEENSY_BINARY_TELEM
// (unchanged binary telemetry types and emitTelemetry() … omit for brevity)
#endif

// ====================== LIMITS / THIRDS ======================
#define AS_LIMITS_SOURCE   1   // 0 = RAW ticks, 1 = degrees  (keep degrees for now)
#define AS_LEFT_RAW_PHOTO   1465
#define AS_RIGHT_RAW_PHOTO  2300
#define AS_CENTER_RAW_PHOTO ((AS_LEFT_RAW_PHOTO+AS_RIGHT_RAW_PHOTO)/2)
#define AS_LEFT_DEG_PHOTO    129.00f
#define AS_RIGHT_DEG_PHOTO   202.00f
#define AS_CENTER_DEG_PHOTO  ((AS_LEFT_DEG_PHOTO+AS_RIGHT_DEG_PHOTO)*0.5f)

static const float LIMIT_SOFT_CUSHION_DEG = 2.5f;
static const float LIMIT_HARD_CUSHION_DEG = 0.8f;

static inline float ticksToDeg_u12(uint16_t t12){ return (360.0f * (float)t12) / 4096.0f; }

// ====================== CONTROL SOURCE / HANDOVER ======================
enum class SteerSrc : uint8_t { S1, S2 };
enum class ThrSrc   : uint8_t { T1, T2 };
static SteerSrc g_steerSrc = SteerSrc::S1;   // default
static ThrSrc   g_thrSrc   = ThrSrc::T1;     // default

// Your S1 thirds (from you): 420..2870
static constexpr uint16_t S1_MIN = 420;
static constexpr uint16_t S1_MAX = 2870;
// S2 thirds (RC): 1000..2000 µs
static constexpr uint16_t S2_MIN = 1000;
static constexpr uint16_t S2_MAX = 2000;

// Cut points: [MIN .. LEFT_END) | [LEFT_END .. CENTER_END) | [CENTER_END .. MAX]
static inline void thirds_u16(uint16_t lo, uint16_t hi, uint16_t& L_end, uint16_t& C_end){
  uint32_t span = (uint32_t)hi - (uint32_t)lo;
  uint16_t third = (uint16_t)(span / 3u);
  L_end = (uint16_t)(lo + third);
  C_end = (uint16_t)(lo + 2u*third);
}
static uint16_t S1_LEFT_END=0, S1_CENTER_END=0;
static uint16_t S2_LEFT_END=0, S2_CENTER_END=0;

// Hand-over conditions
static constexpr uint16_t RC_REVERSE_THRESH_US = 1200;    // ≤ this is reverse
static constexpr uint32_t RC_REVERSE_HOLD_MS   = 3000;    // 3 s
static uint32_t g_rcReverseStartMs = 0;

// Neutral detection (deadbands)
static constexpr uint16_t T2_NEUTRAL_DB_US   = 40;        // ~±40 µs around 1500
static constexpr uint16_t T2_NEUTRAL_CTR_US  = 1500;
static constexpr uint16_t T1_NEUTRAL_DB_MAP  = 30;        // 0..1000 scale
// Auto-return timer when both are neutral
static constexpr uint32_t AUTO_RETURN_MS     = 10000;     // 10 s
static uint32_t g_bothNeutralStartMs = 0;

// T2 throttle arming (must pass through neutral after handover)
static bool g_t2Armed = false;

// Motor drive lockout until we’re ready
#define STEER_DRIVE_ENABLE 0

// ======== Steering controller ==================================================
namespace Steer {

#if AS_LIMITS_SOURCE == 0
  static const float STEER_LEFT_DEG   = ticksToDeg_u12((uint16_t)AS_LEFT_RAW_PHOTO);
  static const float STEER_RIGHT_DEG  = ticksToDeg_u12((uint16_t)AS_RIGHT_RAW_PHOTO);
  static const float STEER_CENTER_DEG = ticksToDeg_u12((uint16_t)AS_CENTER_RAW_PHOTO);
#else
  static const float STEER_LEFT_DEG   = AS_LEFT_DEG_PHOTO;
  static const float STEER_RIGHT_DEG  = AS_RIGHT_DEG_PHOTO;
  static const float STEER_CENTER_DEG = AS_CENTER_DEG_PHOTO;
#endif

  static const float STEER_TOL_DEG    = 1.5f;     // hold window
  static const float KP_PWM_PER_DEG   = 6.0f;     // proportional gain

  // BTS7960 pins
  static const uint8_t PIN_MOT_LPWM = 15;  // J7-6
  static const uint8_t PIN_MOT_RPWM = 16;  // J7-5
  static const uint8_t PIN_MOT_LEN  = 17;  // J7-4
  static const uint8_t PIN_MOT_REN  = 18;  // J7-3

  static const uint32_t STEER_PWM_FREQ = 20000; // 20 kHz
  static const int      PWM_MAX         = 255;   // 8-bit
  static const int      PWM_MIN         = 60;

  enum class Target : uint8_t { Left, Center, Right };
  enum class State  : uint8_t { Idle, ToLeft, ToCenter, ToRight, Holding, NoAngle };

  static State  g_ss      = State::Idle;
  static Target g_lastTgt = Target::Center;
  static float  g_lastErrDeg = 0.0f;
  static float  g_lastTgtDeg = STEER_CENTER_DEG;

  static inline Target pickTarget(uint16_t s1raw, uint16_t s2_us){
    if (g_steerSrc == SteerSrc::S1){
      if (s1raw < S1_LEFT_END)        return Target::Left;
      else if (s1raw < S1_CENTER_END) return Target::Center;
      else                             return Target::Right;
    } else {
      if (s2_us < S2_LEFT_END)        return Target::Left;
      else if (s2_us < S2_CENTER_END) return Target::Center;
      else                             return Target::Right;
    }
  }
  static inline float targetDegFor(Target t){
    switch (t){ case Target::Left:  return STEER_LEFT_DEG;
               case Target::Right: return STEER_RIGHT_DEG;
               default:            return STEER_CENTER_DEG; }
  }

  static void setMotor(int pwmSigned){
#if !STEER_DRIVE_ENABLE
    // Keep disabled for validation
    analogWrite(PIN_MOT_LPWM, 0);
    analogWrite(PIN_MOT_RPWM, 0);
    digitalWrite(PIN_MOT_LEN, LOW);
    digitalWrite(PIN_MOT_REN, LOW);
    return;
#endif
    int mag = pwmSigned; if (mag < 0) mag = -mag;
    if (mag > 0 && mag < PWM_MIN) mag = PWM_MIN;
    if (mag > PWM_MAX) mag = PWM_MAX;

    if (pwmSigned > 0){
      // drive RIGHT
      digitalWrite(PIN_MOT_LEN, LOW);
      digitalWrite(PIN_MOT_REN, HIGH);
      analogWrite(PIN_MOT_LPWM, 0);
      analogWrite(PIN_MOT_RPWM, mag);
    } else if (pwmSigned < 0){
      // drive LEFT
      digitalWrite(PIN_MOT_REN, LOW);
      digitalWrite(PIN_MOT_LEN, HIGH);
      analogWrite(PIN_MOT_RPWM, 0);
      analogWrite(PIN_MOT_LPWM, mag);
    } else {
      // stop
      analogWrite(PIN_MOT_LPWM, 0);
      analogWrite(PIN_MOT_RPWM, 0);
      digitalWrite(PIN_MOT_LEN, LOW);
      digitalWrite(PIN_MOT_REN, LOW);
    }
  }

  static void enter(State s, const char* note){
    g_ss = s;
    SLOGF("S STEER %s\r\n", note);
  }

  static inline State state(){ return g_ss; }
  static inline Target lastTarget(){ return g_lastTgt; }
  static inline float lastErrorDeg(){ return g_lastErrDeg; }
  static inline float lastTargetDeg(){ return g_lastTgtDeg; }

  static inline float clampTarget(float wantDeg){
    const float lo = STEER_LEFT_DEG  + LIMIT_HARD_CUSHION_DEG;
    const float hi = STEER_RIGHT_DEG - LIMIT_HARD_CUSHION_DEG;
    if (wantDeg < lo) return lo;
    if (wantDeg > hi) return hi;
    return wantDeg;
  }
  static inline int softenNearStops(float curDeg, int pwmSigned){
    if (curDeg <= STEER_LEFT_DEG + LIMIT_SOFT_CUSHION_DEG && pwmSigned < 0){
      float span = LIMIT_SOFT_CUSHION_DEG - LIMIT_HARD_CUSHION_DEG;
      float x = (curDeg - (STEER_LEFT_DEG + LIMIT_HARD_CUSHION_DEG)) / span; // 0..1
      if (x < 0) return 0; if (x > 1) x = 1;
      return (int)(pwmSigned * x);
    }
    if (curDeg >= STEER_RIGHT_DEG - LIMIT_SOFT_CUSHION_DEG && pwmSigned > 0){
      float span = LIMIT_SOFT_CUSHION_DEG - LIMIT_HARD_CUSHION_DEG;
      float x = ((STEER_RIGHT_DEG - LIMIT_HARD_CUSHION_DEG) - curDeg) / span; // 0..1
      if (x < 0) return 0; if (x > 1) x = 1;
      return (int)(pwmSigned * x);
    }
    return pwmSigned;
  }

  static void begin(){
    pinMode(PIN_MOT_LPWM, OUTPUT);
    pinMode(PIN_MOT_RPWM, OUTPUT);
    pinMode(PIN_MOT_LEN,  OUTPUT);
    pinMode(PIN_MOT_REN,  OUTPUT);
    analogWriteFrequency(PIN_MOT_LPWM, STEER_PWM_FREQ);
    analogWriteFrequency(PIN_MOT_RPWM, STEER_PWM_FREQ);
    analogWriteResolution(8);
    setMotor(0);

    // compute thirds and print
    thirds_u16(S1_MIN, S1_MAX, S1_LEFT_END, S1_CENTER_END);
    thirds_u16(S2_MIN, S2_MAX, S2_LEFT_END, S2_CENTER_END);

    SLOGF("S LIM left=%.2f center=%.2f right=%.2f  soft=%.2f hard=%.2f\r\n",
          (double)STEER_LEFT_DEG, (double)STEER_CENTER_DEG, (double)STEER_RIGHT_DEG,
          (double)LIMIT_SOFT_CUSHION_DEG, (double)LIMIT_HARD_CUSHION_DEG);

    SLOGF("S S1 thirds: [%u..%u) | [%u..%u) | [%u..%u]\r\n",
          (unsigned)S1_MIN, (unsigned)S1_LEFT_END,
          (unsigned)S1_LEFT_END, (unsigned)S1_CENTER_END,
          (unsigned)S1_CENTER_END, (unsigned)S1_MAX);

    SLOGF("S S2 thirds: [%u..%u) | [%u..%u) | [%u..%u]\r\n",
          (unsigned)S2_MIN, (unsigned)S2_LEFT_END,
          (unsigned)S2_LEFT_END, (unsigned)S2_CENTER_END,
          (unsigned)S2_CENTER_END, (unsigned)S2_MAX);

    SLOGF("S SRC steer=S1 throttle=T1 (default). Hold T2 reverse %u ms -> S2/T2; needs T2 neutral to arm.\r\n",
          (unsigned)RC_REVERSE_HOLD_MS);
  }

  static void tick(uint16_t s1raw){
    // decide target bucket
    Target tgt = pickTarget(s1raw, ::MM::g_s2_us);
    float  wantDeg = targetDegFor(tgt);
    float  tgtDeg  = clampTarget(wantDeg);

    if (!::angleFresh(220)){
      setMotor(0);
      if (g_ss != State::NoAngle) enter(State::NoAngle, "no-angle");
      g_lastErrDeg = 0.0f;
      g_lastTgtDeg = tgtDeg;
      g_lastTgt = tgt;
      return;
    }

    float cur; noInterrupts(); cur = ::g_as_deg; interrupts();
    float err = tgtDeg - cur;                      // +err => need to move RIGHT
    int   pwm = (int)(KP_PWM_PER_DEG * err);

    // Absolute “no push past” at hard cushions
    if ((cur <= STEER_LEFT_DEG  + LIMIT_HARD_CUSHION_DEG) && (pwm < 0))  pwm = 0;
    if ((cur >= STEER_RIGHT_DEG - LIMIT_HARD_CUSHION_DEG) && (pwm > 0))  pwm = 0;

    // Taper near soft zone
    pwm = softenNearStops(cur, pwm);

    g_lastErrDeg = err;
    g_lastTgtDeg = tgtDeg;

    // For transparency in UI while drive disabled
    const char* srcSteer = (g_steerSrc==SteerSrc::S1) ? "S1" : "S2";
    const char* srcThr   = (g_thrSrc  ==ThrSrc::T1)   ? "T1" : "T2";
    SLOGF("S STEER tgt=%s(%.2f) cur=%.2f err=%.2f src=%s thr=%s drive=%s\r\n",
          (tgt==Target::Left)?"L":(tgt==Target::Right)?"R":"C",
          (double)tgtDeg, (double)cur, (double)err, srcSteer, srcThr,
          STEER_DRIVE_ENABLE ? "ON" : "OFF");

    if (fabsf(err) <= STEER_TOL_DEG || pwm == 0){
      setMotor(0);
      if (g_ss != State::Holding || tgt != g_lastTgt){
        const char* where = (tgt==Target::Left)?"left":(tgt==Target::Right)?"right":"center";
        enter(State::Holding, where);
      }
    } else {
      setMotor(pwm);
      State want = (tgt==Target::Left)? State::ToLeft :
                   (tgt==Target::Right)? State::ToRight : State::ToCenter;
      if (g_ss != want || tgt != g_lastTgt){
        const char* where = (tgt==Target::Left)?"left":(tgt==Target::Right)?"right":"center";
        enter(want, where);
      }
    }

    g_lastTgt = tgt;
  }
} // namespace Steer

// ---- USB commands (optional small set) ----
static void handleUsbCommandsOnce() {
  static String line;
  while (Serial.available()){
    char c=(char)Serial.read(); if(c=='\r') continue;
    if(c=='\n'){
      line.trim();
      if(line.length()){
        String verb,a1; int sp1=line.indexOf(' ');
        if(sp1<0){ verb=line; } else { verb=line.substring(0,sp1); a1=line.substring(sp1+1); a1.trim(); }
        verb.toUpperCase(); a1.toUpperCase();

        if (verb=="LOG"){
          a1.toLowerCase();
          if(a1=="ota"){ g_logSink=LogSink::OTA; Serial.println("[MyApp] log sink: OTA"); }
          else if(a1=="usb"){ g_logSink=LogSink::USB; Serial.println("[MyApp] log sink: USB"); }
          else if(a1=="both"){ g_logSink=LogSink::BOTH; Serial.println("[MyApp] log sink: BOTH"); }
          else Serial.println("usage: LOG ota|usb|both");
        } else {
          Serial.println("Unknown. Try: LOG ota|usb|both");
        }
      }
      line="";
    } else if(line.length()<120) line+=c;
  }
}

// ===== Setup / Loop =====
void setup(){
  using namespace MM;

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  pinMode(PIN_FWD, INPUT_PULLUP);
  pinMode(PIN_REV, INPUT_PULLUP);

  analogReadResolution(12);
  analogReadAveraging(8);

  pinMode(PIN_RC_S, INPUT);
  pinMode(PIN_RC_T, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_S), isrRcSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_T), isrRcThrottle, CHANGE);

  Serial.begin(115200);
  Serial2.begin(230400);                  // Teensy OTA/console UART

  // Bring up OTA on Serial2 BEFORE enabling console chatter
  OtaUpdater::begin(Serial2, 230400);
  OtaUpdater::setAppVersion(APP_FW_VERSION);

  Serial.println("[MyApp] boot");
  delay(1200);                            // allow early noise to pass
  OtaConsole::begin(Serial2);             // logs to ESP32 (/console)
  OtaConsole::setEnabled(true);

  // Motor driver + thirds print
  Steer::begin();

  // Boot banner
  SLOGF("MyApp RAW+STEER FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);
  SLOGF("BOOT_PINS fwd=%d rev=%d (0=LOW,1=HIGH)\r\n",
        (int)digitalRead(PIN_FWD), (int)digitalRead(PIN_REV));
#if ENABLE_TEENSY_BINARY_TELEM
  SLOGF("UART telemetry: binary packets enabled\r\n");
#else
  SLOGF("UART telemetry: binary packets disabled (ASCII only)\r\n");
#endif
}

static inline bool isT2Neutral(uint16_t us){
  return (us >= (T2_NEUTRAL_CTR_US - T2_NEUTRAL_DB_US)) &&
         (us <= (T2_NEUTRAL_CTR_US + T2_NEUTRAL_DB_US));
}
static inline bool isT1Neutral(uint16_t t1raw){
  int16_t v = MM::mapT1_to_1000(t1raw);
  return (v <= T1_NEUTRAL_DB_MAP);
}

void loop(){
  using namespace MM;

  // OTA tick
  OtaUpdater::tick();

  // ~33 Hz sampling / logic tick
  static uint32_t t_sample=0;
  const uint32_t nowUs = micros();
  if (nowUs - t_sample >= 30000){ // ~30ms
    t_sample = nowUs;

    uint32_t rcS_last_us, rcT_last_us;
    // Copy RC widths atomically
    noInterrupts();
    g_s2_us = rcS_width_us;
    g_t2_us = rcT_width_us;
    rcS_last_us = rcS_last_update_us;
    rcT_last_us = rcT_last_update_us;
    interrupts();

    // Sample raw and FNR
    sampleAnalog();
    sampleFnr();

    // ---- Control handover logic ----
    // 1) Default S1/T1. If T2 held reverse 3s → switch both to S2/T2, require T2 neutral to arm throttle.
    if (g_steerSrc == SteerSrc::S1 && g_thrSrc == ThrSrc::T1){
      if (g_t2_us <= RC_REVERSE_THRESH_US){
        if (g_rcReverseStartMs == 0) g_rcReverseStartMs = millis();
        else if ((millis() - g_rcReverseStartMs) >= RC_REVERSE_HOLD_MS){
          g_steerSrc = SteerSrc::S2;
          g_thrSrc   = ThrSrc::T2;
          g_t2Armed  = false; // must pass through neutral first
          SLOGF("S SRC -> steer=S2 throttle=T2 (T2 reverse hold)\r\n");
        }
      } else {
        g_rcReverseStartMs = 0; // broke the hold
      }
    }

    // 2) If currently on T2, require neutral pass to arm (one-shot)
    if (g_thrSrc == ThrSrc::T2 && !g_t2Armed){
      if (isT2Neutral(g_t2_us)){
        g_t2Armed = true;
        SLOGF("S T2 ARMED (neutral seen)\r\n");
      }
    }

    // 3) Auto-return to S1/T1 when BOTH neutral for 10 s (jitter-safe)
    bool bothNeutral = isT1Neutral(g_t1) && isT2Neutral(g_t2_us);
    if (bothNeutral){
      if (g_bothNeutralStartMs == 0) g_bothNeutralStartMs = millis();
      else if ((millis() - g_bothNeutralStartMs) >= AUTO_RETURN_MS){
        if (g_steerSrc != SteerSrc::S1 || g_thrSrc != ThrSrc::T1){
          g_steerSrc = SteerSrc::S1;
          g_thrSrc   = ThrSrc::T1;
          g_rcReverseStartMs = 0;
          SLOGF("S SRC -> steer=S1 throttle=T1 (auto-return)\r\n");
        }
      }
    } else {
      g_bothNeutralStartMs = 0;
    }

    // Maps just for the console line (helps visualize)
    const int16_t s2_map = mapRcSteer_us_to_1000(g_s2_us);
    const int16_t t2_map = mapRcThr_us_to_1000(g_t2_us);
    const int16_t t1_map = mapT1_to_1000(g_t1);

    // Angle snapshot (optional in RAW line)
    uint16_t as_raw = 0;
    float    as_deg = 0.0f;
    const bool haveAngle = angleSnapshot(as_raw, as_deg);

#if RAW_MIRROR_MODE
    if (haveAngle) {
      SLOGF("S RAW S1=%u  T1=%u(%d)  S2_us=%d(%d)  T2_us=%d(%d)  AS=%u(%.2f)  FNR=%s  SRC=%s/%s %s\r\n",
            (unsigned)g_s1,
            (unsigned)g_t1, (int)t1_map,
            (int)g_s2_us, (int)s2_map,
            (int)g_t2_us, (int)t2_map,
            (unsigned)as_raw, (double)as_deg,
            fnrWord(g_fnr),
            (g_steerSrc==SteerSrc::S1)?"S1":"S2",
            (g_thrSrc  ==ThrSrc::T1)  ?"T1":"T2",
            (g_thrSrc==ThrSrc::T2 ? (g_t2Armed?"ARMED":"UNARMED") : ""));
    } else {
      SLOGF("S RAW S1=%u  T1=%u(%d)  S2_us=%d(%d)  T2_us=%d(%d)  AS=---  FNR=%s  SRC=%s/%s %s\r\n",
            (unsigned)g_s1,
            (unsigned)g_t1, (int)t1_map,
            (int)g_s2_us, (int)s2_map,
            (int)g_t2_us, (int)t2_map,
            fnrWord(g_fnr),
            (g_steerSrc==SteerSrc::S1)?"S1":"S2",
            (g_thrSrc  ==ThrSrc::T1)  ?"T1":"T2",
            (g_thrSrc==ThrSrc::T2 ? (g_t2Armed?"ARMED":"UNARMED") : ""));
    }
#endif
  }

  // ~50 Hz steering control loop (drive currently disabled)
  static uint32_t t_ctrl = 0;
  if (micros() - t_ctrl >= 20000){
    t_ctrl = micros();
    Steer::tick(MM::g_s1);
  }

  // Blink
  static uint32_t t_led=0; static bool led=false;
  if (millis() - t_led >= BLINK_MS){
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led? HIGH:LOW);
  }

  handleUsbCommandsOnce();
}
