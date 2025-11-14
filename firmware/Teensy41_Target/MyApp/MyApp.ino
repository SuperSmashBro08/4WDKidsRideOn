// ============ MyApp.ino (V 3.4.1 RAW + Steering Control, 1/3 bucket mapping) ============
//
// - RAW snapshot line every ~30 ms (S1/T1/S2/T2/FNR/AS)
// - ESP32 angle stream over the SAME UART (Serial2) via OtaUpdater hook
//   expected line from ESP32: "AS,<raw>,<deg>"
// - Motor driver: BTS7960-style on pins 15..18 (LPWM/RPWM/L_EN/R_EN) => J7-6..J7-3
// - 20 kHz PWM for quiet steering drive
// - Clamp + soften near physical limits; absolute “no push past hard cushion”
// - Left/Center/Right selection by splitting BOTH Steering1 (analog) and Steering2 (RC) into equal thirds
//
// ------------------------------------------------------------------------------------------

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
#define APP_FW_VERSION   "V 3.4.1"

// ---------- Blink ----------
#define LED_PIN          13
#define BLINK_MS         500

// ---------- Raw-mirror mode switch ----------
#define RAW_MIRROR_MODE  1   // keep raw stream on

// ================== USER-TUNABLE RANGES FOR 1/3 SPLIT ==================
// Steering1 (analog pot) nominal full-scale:
static constexpr uint16_t S1_MIN = 420;
static constexpr uint16_t S1_MAX = 2870;

// Steering2 (RC PWM width in microseconds) nominal full-scale:
static constexpr uint16_t RC_MIN_US = 1000;
static constexpr uint16_t RC_MID_US = 1500;
static constexpr uint16_t RC_MAX_US = 2000;
static constexpr uint16_t RC_DB_US  = 20;   // deadband around mid for mapping helpers

// If you want the center third slightly tighter (optional), shrink it a bit:
// e.g. 0.10 = shrink by 10% (wider left/right thirds). 0.00 = exact equal thirds.
static constexpr float CENTER_BAND_SHRINK_PCT = 0.00f;
// =======================================================================

// ---------- IO pins ----------
static constexpr uint8_t PIN_FWD    = 10;
static constexpr uint8_t PIN_REV    = 11;
static constexpr uint8_t PIN_POT_S1 = 14;  // steering analog
static constexpr uint8_t PIN_POT_T1 = 27;  // throttle analog
static constexpr uint8_t PIN_RC_S   = 4;   // RC steer PWM in
static constexpr uint8_t PIN_RC_T   = 5;   // RC throttle PWM in

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
static uint16_t g_s1=0, g_t1=0;                  // raw analog
static volatile uint32_t rcS_rise_us=0, rcS_last_update_us=0;
static volatile uint16_t rcS_width_us=1500;
static volatile uint32_t rcT_rise_us=0, rcT_last_update_us=0;
static volatile uint16_t rcT_width_us=1500;
static int16_t  g_s2_us=1500, g_t2_us=1500;      // copied atomically each tick

// ===== RC PWM capture (interrupt) =====
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
  int16_t u = us;
  if(u<RC_MIN_US) u=RC_MIN_US; if(u>RC_MAX_US) u=RC_MAX_US;
  int32_t x=(int32_t)u-RC_MID_US; if (abs(x)<=RC_DB_US) return 0;
  return clamp_i16((x*1000)/(RC_MAX_US-RC_MID_US), -1000, 1000);
}
static inline int16_t mapRcThr_us_to_1000(int16_t us){
  int16_t u=us;
  if(u<RC_MIN_US) u=RC_MIN_US; if(u>RC_MAX_US) u=RC_MAX_US;
  int32_t out=((int32_t)u-RC_MIN_US)*1000/(RC_MAX_US-RC_MIN_US);
  if(out<0)out=0; if(out>1000)out=1000; return (int16_t)out;
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
// The order matters: globals first, then helpers.
static volatile uint16_t g_as_raw = 0;
static volatile float    g_as_deg = NAN;
static volatile uint32_t g_as_ms  = 0;

// Forward decls WITHOUT defaults
static inline bool angleFresh(uint32_t maxAgeMs);
static bool angleSnapshot(uint16_t& rawOut, float& degOut);

extern "C" bool OtaUserHandleLine(const char* line) {
  if (!line || !*line) return false;

  if (strcmp(line, "PING") == 0)    { Serial2.println("PONG"); return true; }
  if (strcmp(line, "VERSION") == 0) {
    Serial2.print("FW "); Serial2.println(APP_FW_VERSION);
    Serial2.println("FLASHERX READY");
    return true;
  }

  // Angle packets from ESP32: AS,<raw>,<deg>
  if (line[0]=='A' && line[1]=='S' && line[2]==',') {
    const char* p = line + 3;
    const char* comma = strchr(p, ','); if (!comma) return false;
    uint16_t raw = (uint16_t)strtoul(p, nullptr, 10);
    float    deg = atof(comma + 1);
    noInterrupts(); g_as_raw=raw; g_as_deg=deg; g_as_ms=millis(); interrupts();
    return true;
  }
  return false; // allow flasher traffic (HELLO/BEGIN/L.../END)
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
  float scaled = v * 100.0f;
  scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
  long val = (long)scaled;
  if (val > INT16_MAX) val = INT16_MAX;
  if (val < INT16_MIN) val = INT16_MIN;
  return (int16_t)val;
}

#if ENABLE_TEENSY_BINARY_TELEM
static constexpr uint8_t TELEM_MAGIC0   = 0xA5;
static constexpr uint8_t TELEM_MAGIC1   = 0x5A;
static constexpr uint8_t TELEM_VERSION  = 1;
enum TelemetryFlags : uint16_t {
  TELEM_FLAG_INHIBIT     = 1u << 0,
  TELEM_FLAG_RC_ACTIVE   = 1u << 1,
  TELEM_FLAG_FNR_FAULT   = 1u << 2,
  TELEM_FLAG_ANGLE_FRESH = 1u << 3,
};
struct __attribute__((packed)) TelemetryPayload {
  uint16_t seq; uint32_t millis; uint16_t flags; uint16_t s1; uint16_t t1;
  int16_t s2_us; int16_t t2_us; int16_t s2_map; int16_t t2_map;
  uint16_t as_raw; int16_t as_deg_centi; uint8_t gear; uint8_t steer_target;
  uint8_t steer_state; uint8_t reserved; int16_t steer_err_centi;
};
struct __attribute__((packed)) TelemetryPacket {
  uint8_t magic0, magic1, version, length; TelemetryPayload payload;
};
static uint16_t g_telemSeq = 0;
static void emitTelemetry(const TelemetryPayload& payload){
  if (OtaUpdater::inProgress()) return;
  TelemetryPacket pkt{}; pkt.magic0=0xA5; pkt.magic1=0x5A; pkt.version=1;
  pkt.length=sizeof(TelemetryPayload); pkt.payload=payload;
  Serial2.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}
#endif

// ====================== CONFIGURE YOUR LIMITS HERE ======================
// Choose one format and fill the numbers from your photo.
// Set AS_LIMITS_SOURCE to 0 if your photo has RAW ticks (0..4095), or 1 if it has degrees.
#define AS_LIMITS_SOURCE   1   // 0 = RAW ticks, 1 = degrees

// If using RAW ticks from photo:
#define AS_LEFT_RAW_PHOTO   1465
#define AS_RIGHT_RAW_PHOTO  2300
#define AS_CENTER_RAW_PHOTO ((AS_LEFT_RAW_PHOTO+AS_RIGHT_RAW_PHOTO)/2)

// If using degrees from photo:
#define AS_LEFT_DEG_PHOTO    129.00f
#define AS_RIGHT_DEG_PHOTO   202.00f
#define AS_CENTER_DEG_PHOTO  ((AS_LEFT_DEG_PHOTO+AS_RIGHT_DEG_PHOTO)*0.5f)

// Soft/hard cushions (safety margins near stops)
static const float LIMIT_SOFT_CUSHION_DEG = 2.5f;   // taper effort inside this band
static const float LIMIT_HARD_CUSHION_DEG = 0.8f;   // absolute “do not push past”
// =======================================================================

static inline float ticksToDeg_u12(uint16_t t12){ return (360.0f * (float)t12) / 4096.0f; }

// ======== Steering controller ==================================================
namespace Steer {

// Build final left/right/center degrees from RAW or DEG inputs:
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

  // BTS7960 pins (J7 header)
  static const uint8_t PIN_MOT_LPWM = 15;  // J7-6  Steer L_PWM
  static const uint8_t PIN_MOT_RPWM = 16;  // J7-5  Steer R_PWM
  static const uint8_t PIN_MOT_LEN  = 17;  // J7-4  Steer L_EN
  static const uint8_t PIN_MOT_REN  = 18;  // J7-3  Steer R_EN

  static const uint32_t STEER_PWM_FREQ = 20000; // 20 kHz
  static const int      PWM_MAX         = 255;   // 8-bit
  static const int      PWM_MIN         = 60;

  // ======= 1/3 BUCKET SELECTION =======
  enum class Target : uint8_t { Left, Center, Right };
  enum class State  : uint8_t { Idle, ToLeft, ToCenter, ToRight, Holding, NoAngle };

  static State  g_ss      = State::Idle;
  static Target g_lastTgt = Target::Center;
  static float  g_lastErrDeg = 0.0f;
  static float  g_lastTgtDeg = STEER_CENTER_DEG;

  // Equal thirds with optional center shrink
  static inline void thirds_u16(uint16_t minv, uint16_t maxv, uint16_t& L_hi, uint16_t& C_hi){
    const uint32_t span = (uint32_t)maxv - (uint32_t)minv;
    uint32_t third = span / 3u;
    uint32_t cshrink = (uint32_t)((float)third * CENTER_BAND_SHRINK_PCT * 0.5f);
    uint32_t b1 = (uint32_t)minv + third - cshrink;         // end of Left
    uint32_t b2 = (uint32_t)minv + (2u*third) + cshrink;    // end of Center
    if (b1 <= minv) b1 = (uint32_t)minv + 1;
    if (b2 <= b1)   b2 = b1 + 1;
    if (b2 > maxv)  b2 = maxv;
    L_hi = (uint16_t)b1;
    C_hi = (uint16_t)b2;
  }

  static inline Target pickTargetS1(uint16_t s1raw){
    uint16_t Lhi, Chi; thirds_u16(S1_MIN, S1_MAX, Lhi, Chi);
    if (s1raw < Lhi)       return Target::Left;
    else if (s1raw < Chi)  return Target::Center;
    else                   return Target::Right;
  }

  static inline Target pickTargetS2_us(uint16_t rc_us){
    uint16_t Lhi, Chi; thirds_u16(RC_MIN_US, RC_MAX_US, Lhi, Chi);
    if (rc_us < Lhi)       return Target::Left;
    else if (rc_us < Chi)  return Target::Center;
    else                   return Target::Right;
  }

  static inline float targetDegFor(Target t){
    switch (t){
      case Target::Left:   return STEER_LEFT_DEG;
      case Target::Right:  return STEER_RIGHT_DEG;
      default:             return STEER_CENTER_DEG;
    }
  }

  static void setMotor(int pwmSigned){
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
    SLOGF("E STEER %s\r\n", note);
  }

  static inline State state(){ return g_ss; }
  static inline Target lastTarget(){ return g_lastTgt; }
  static inline float lastErrorDeg(){ return g_lastErrDeg; }
  static inline float lastTargetDeg(){ return g_lastTgtDeg; }

  // Safety helpers
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

  // Existing limits banner
  SLOGF("S LIM left=%.2f center=%.2f right=%.2f  soft=%.2f hard=%.2f\r\n",
        (double)STEER_LEFT_DEG, (double)STEER_CENTER_DEG, (double)STEER_RIGHT_DEG,
        (double)LIMIT_SOFT_CUSHION_DEG, (double)LIMIT_HARD_CUSHION_DEG);

  // 🟢 New section: show 1/3 bucket edges for S1 (analog) and S2 (RC)
  {
    uint16_t s1L, s1C; thirds_u16(S1_MIN, S1_MAX, s1L, s1C);
    uint16_t s2L, s2C; thirds_u16(RC_MIN_US, RC_MAX_US, s2L, s2C);
    SLOGF("S S1 thirds: [%u..%u) | [%u..%u) | [%u..%u]\r\n",
          (unsigned)S1_MIN, (unsigned)s1L,
          (unsigned)s1L,    (unsigned)s1C,
          (unsigned)s1C,    (unsigned)S1_MAX);
    SLOGF("S S2 thirds: [%u..%u) | [%u..%u) | [%u..%u]\r\n",
          (unsigned)RC_MIN_US, (unsigned)s2L,
          (unsigned)s2L,       (unsigned)s2C,
          (unsigned)s2C,       (unsigned)RC_MAX_US);
  }
}


  // Select target by S1 thirds (primary), but if you want to drive from RC thirds instead,
  // change the call below from pickTargetS1(MM::g_s1) to pickTargetS2_us(MM::g_s2_us).
  static void tick(uint16_t s1raw){
    Target tgt = pickTargetS1(s1raw);
    // Alternative: Target tgt = pickTargetS2_us((uint16_t)MM::g_s2_us);

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

    // Taper effort inside soft cushions
    pwm = softenNearStops(cur, pwm);

    g_lastErrDeg = err;
    g_lastTgtDeg = tgtDeg;

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

// ---- Live calibration helper (optional) ----
static float g_calLeftDeg   = Steer::STEER_LEFT_DEG;
static float g_calRightDeg  = Steer::STEER_RIGHT_DEG;
static float g_calCenterDeg = Steer::STEER_CENTER_DEG;

static bool angleNow(float& degOut){
  uint16_t raw; float deg;
  if (!angleSnapshot(raw, deg)) return false;
  degOut = deg; return true;
}
static void calShow(){
  SLOGF("S CAL left=%.2f center=%.2f right=%.2f\r\n",
        (double)g_calLeftDeg, (double)g_calCenterDeg, (double)g_calRightDeg);
  SLOGF("S CAL paste:\r\n"
        "  static const float STEER_LEFT_DEG   = %.2ff;\r\n"
        "  static const float STEER_RIGHT_DEG  = %.2ff;\r\n"
        "  static const float STEER_CENTER_DEG = %.2ff;\r\n",
        (double)g_calLeftDeg, (double)g_calRightDeg,
        (double)((g_calLeftDeg+g_calRightDeg)*0.5f));
}

// ===== USB commands (simple: LOG + CAL) =====
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
        } else if (verb=="CAL") {
          // CAL L | C | R | SHOW
          if (a1=="L") {
            float d; if (angleNow(d)) { g_calLeftDeg=d; calShow(); }
            else SLOGF("S CAL no angle yet\r\n");
          } else if (a1=="C") {
            float d; if (angleNow(d)) { g_calCenterDeg=d; calShow(); }
            else SLOGF("S CAL no angle yet\r\n");
          } else if (a1=="R") {
            float d; if (angleNow(d)) { g_calRightDeg=d; calShow(); }
            else SLOGF("S CAL no angle yet\r\n");
          } else if (a1=="SHOW") {
            calShow();
          } else {
            Serial.println("usage: CAL L|C|R|SHOW");
          }
        } else {
          Serial.println("Unknown. Try: LOG ota|usb|both  |  CAL L|C|R|SHOW");
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

  // Motor driver init
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

void loop(){
  using namespace MM;

  // OTA tick (console will be force-muted by the OTA module on HELLO)
  OtaUpdater::tick();

  // ~33 Hz sampling tick (RAW stream + optional binary telemetry)
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

    const int16_t s2_map = mapRcSteer_us_to_1000(g_s2_us);
    const int16_t t2_map = mapRcThr_us_to_1000(g_t2_us);

    uint16_t as_raw = 0;
    float    as_deg = 0.0f;
    const bool haveAngle = angleSnapshot(as_raw, as_deg);

#if RAW_MIRROR_MODE
    if (haveAngle) {
      SLOGF("S RAW S1=%u  T1=%u  S2_us=%d(%d)  T2_us=%d(%d)  AS=%u(%.2f)  FNR=%s\r\n",
            (unsigned)g_s1, (unsigned)g_t1,
            (int)g_s2_us, (int)s2_map,
            (int)g_t2_us, (int)t2_map,
            (unsigned)as_raw, (double)as_deg,
            fnrWord(g_fnr));
    } else {
      SLOGF("S RAW S1=%u  T1=%u  S2_us=%d(%d)  T2_us=%d(%d)  AS=---  FNR=%s\r\n",
            (unsigned)g_s1, (unsigned)g_t1,
            (int)g_s2_us, (int)s2_map,
            (int)g_t2_us, (int)t2_map,
            fnrWord(g_fnr));
    }
#endif

#if ENABLE_TEENSY_BINARY_TELEM
    if (!OtaUpdater::inProgress()){
      static uint16_t g_telemSeq = 0;
      static constexpr uint32_t RC_STALE_US = 250000; // 250 ms
      const bool rcFresh = ((uint32_t)(nowUs - rcS_last_us) <= RC_STALE_US) &&
                           ((uint32_t)(nowUs - rcT_last_us) <= RC_STALE_US);

      TelemetryPayload telem{};
      telem.seq              = ++g_telemSeq;
      telem.millis           = millis();
      telem.flags            = 0;
      if (haveAngle) telem.flags |= TELEM_FLAG_ANGLE_FRESH;
      if (g_fnr == FNR_FAULT) telem.flags |= TELEM_FLAG_FNR_FAULT;
      if (rcFresh) telem.flags |= TELEM_FLAG_RC_ACTIVE;
      telem.s1               = g_s1;
      telem.t1               = g_t1;
      telem.s2_us            = g_s2_us;
      telem.t2_us            = g_t2_us;
      telem.s2_map           = s2_map;
      telem.t2_map           = t2_map;
      telem.as_raw           = haveAngle ? as_raw : 0u;
      telem.as_deg_centi     = haveAngle ? toCenti(as_deg) : INT16_MIN;
      telem.gear             = static_cast<uint8_t>(g_fnr);
      telem.steer_target     = 0;
      telem.steer_state      = 0;
      telem.reserved         = 0;
      telem.steer_err_centi  = INT16_MIN;
      emitTelemetry(telem);
    }
#endif
  }

  // ~50 Hz steering control loop
  static uint32_t t_ctrl = 0;
  if (micros() - t_ctrl >= 20000){
    t_ctrl = micros();
    Steer::tick(MM::g_s1);              // or Steer::tick(MM::g_s2_us) if you want RC thirds to decide
  }

  // Blink
  static uint32_t t_led=0; static bool led=false;
  if (millis() - t_led >= BLINK_MS){
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led? HIGH:LOW);
  }

  handleUsbCommandsOnce();
}
