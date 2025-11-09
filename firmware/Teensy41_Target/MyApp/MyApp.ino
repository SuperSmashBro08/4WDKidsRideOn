// ============ MyApp.ino (V 3.3.6 RAW + Steering Control, single-UART) ============
//
// - RAW snapshot line every ~30 ms (S1/T1/S2/T2/FNR)
// - ESP32 angle stream over the SAME UART (Serial2) via OtaUpdater hook
//   expected line: "AS,<raw>,<deg>"
// - Motor driver: BTS7960-style on pins 15..18 (LPWM/RPWM/L_EN/R_EN)
// - 20 kHz PWM for quiet steering drive
// - OTA + console share Serial2; non-OTA lines are delegated to this sketch via hook
//
// If you want USB prints too:  LOG usb   or   LOG both   on the USB serial.

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
#define APP_FW_VERSION   "V 3.3.6"

// ---------- Blink ----------
#define LED_PIN          13
#define BLINK_MS         500

// ---------- Raw-mirror mode switch ----------
#define RAW_MIRROR_MODE  1   // keep raw stream on

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

static void LOG_raw(const char* s){
  if (g_logSink==LogSink::OTA || g_logSink==LogSink::BOTH) OtaConsole::printf("%s", s);
  if (g_logSink==LogSink::USB || g_logSink==LogSink::BOTH) Serial.print(s);
}
static void SLOGF(const char* fmt, ...){
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  LOG_raw(buf);
}

// ---------- Binary telemetry packet (to ESP32) ----------
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
  uint16_t seq;
  uint32_t millis;
  uint16_t flags;
  uint16_t s1;
  uint16_t t1;
  int16_t  s2_us;
  int16_t  t2_us;
  int16_t  s2_map;
  int16_t  t2_map;
  uint16_t as_raw;
  int16_t  as_deg_centi;
  uint8_t  gear;
  uint8_t  steer_target;
  uint8_t  steer_state;
  uint8_t  reserved;
  int16_t  steer_err_centi;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t           magic0;
  uint8_t           magic1;
  uint8_t           version;
  uint8_t           length;
  TelemetryPayload  payload;
};

static_assert(sizeof(TelemetryPayload) <= 64, "Telemetry payload too large");

static uint16_t g_telemSeq = 0;

static inline int16_t toCenti(float v){
  float scaled = v * 100.0f;
  if (scaled >= 0.0f) scaled += 0.5f;
  else                scaled -= 0.5f;
  long val = (long)scaled;
  if (val > INT16_MAX) val = INT16_MAX;
  if (val < INT16_MIN) val = INT16_MIN;
  return (int16_t)val;
}

static void emitTelemetry(const TelemetryPayload& payload){
  if (OtaUpdater::inProgress()) return;
  TelemetryPacket pkt{};
  pkt.magic0  = TELEM_MAGIC0;
  pkt.magic1  = TELEM_MAGIC1;
  pkt.version = TELEM_VERSION;
  pkt.length  = sizeof(TelemetryPayload);
  pkt.payload = payload;
  Serial2.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
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

// ======== ESP32 angle stream over Serial2 via OtaUpdater hook ========
static volatile uint16_t g_as_raw = 0;
static volatile float    g_as_deg = NAN;
static volatile uint32_t g_as_ms  = 0;

extern "C" bool OtaUserHandleLine(const char* line) {
  // Expect: "AS,<raw>,<deg>" from ESP32
  if (!line || line[0] != 'A' || line[1] != 'S' || line[2] != ',') return false;
  const char* p = line + 3;
  const char* comma = strchr(p, ',');
  if (!comma) return false;

  uint16_t raw = (uint16_t)strtoul(p, nullptr, 10);
  float deg = atof(comma + 1);

  noInterrupts();
  g_as_raw = raw;
  g_as_deg = deg;
  g_as_ms  = millis();
  interrupts();
  return true;
}
static inline bool angleFresh(uint32_t maxAgeMs=200){
  uint32_t age = millis() - g_as_ms;
  return (g_as_ms != 0) && (age <= maxAgeMs) && !isnan(g_as_deg);
}

static bool angleSnapshot(uint16_t& rawOut, float& degOut){
  uint16_t raw;
  float deg;
  uint32_t stamp;
  noInterrupts();
  raw = g_as_raw;
  deg = g_as_deg;
  stamp = g_as_ms;
  interrupts();
  if (stamp == 0 || isnan(deg)) return false;
  uint32_t age = millis() - stamp;
  if (age > 200) return false;
  rawOut = raw;
  degOut = deg;
  return true;
}

// ======== Steering controller in a namespace (prevents auto-proto issues) ========
namespace Steer {
  // From your calibration
  static const float STEER_LEFT_DEG   = 129.07f;
  static const float STEER_RIGHT_DEG  = 201.97f;
  static const float STEER_CENTER_DEG = (STEER_LEFT_DEG + STEER_RIGHT_DEG) * 0.5f;
  static const float STEER_TOL_DEG    = 1.5f;
  static const float KP_PWM_PER_DEG   = 6.0f;

  // Decide target from S1 raw bins (tune quickly from RAW prints)
  static const uint16_t S1_LEFT_MAX   = 1600;  // S1 ≤ this => Left
  static const uint16_t S1_RIGHT_MIN  = 2400;  // S1 ≥ this => Right

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

  static inline Target pickTarget(uint16_t s1raw){
    if (s1raw <= S1_LEFT_MAX)  return Target::Left;
    if (s1raw >= S1_RIGHT_MIN) return Target::Right;
    return Target::Center;
  }
  static inline float targetDegFor(Target t){
    switch (t){ case Target::Left:  return STEER_LEFT_DEG;
               case Target::Right: return STEER_RIGHT_DEG;
               default:            return STEER_CENTER_DEG; }
  }

  static void setMotor(int pwmSigned){
    int mag = pwmSigned;
    if (mag < 0) mag = -mag;
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

  static void begin(){
    pinMode(PIN_MOT_LPWM, OUTPUT);
    pinMode(PIN_MOT_RPWM, OUTPUT);
    pinMode(PIN_MOT_LEN,  OUTPUT);
    pinMode(PIN_MOT_REN,  OUTPUT);
    analogWriteFrequency(PIN_MOT_LPWM, STEER_PWM_FREQ);
    analogWriteFrequency(PIN_MOT_RPWM, STEER_PWM_FREQ);
    analogWriteResolution(8);
    setMotor(0);
  }

  static void tick(uint16_t s1raw){
    Target tgt = pickTarget(s1raw);
    float  tgtDeg = targetDegFor(tgt);

    if (!::angleFresh()){
      setMotor(0);
      if (g_ss != State::NoAngle) enter(State::NoAngle, "no-angle");
      g_lastErrDeg = 0.0f;
      g_lastTgtDeg = tgtDeg;
      g_lastTgt = tgt;
      return;
    }

    float cur = ::g_as_deg;
    float err = tgtDeg - cur;                      // +err => need to move RIGHT
    int   pwm = (int)(KP_PWM_PER_DEG * err);

    g_lastErrDeg = err;
    g_lastTgtDeg = tgtDeg;

    if (fabsf(err) <= STEER_TOL_DEG){
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

// ===== USB commands (simple: LOG sink only) =====
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
  Serial2.begin(230400);

  // Bring up OTA on Serial2 (used also for console & app hook)
  OtaUpdater::begin(Serial2, 230400);
  OtaUpdater::setAppVersion(APP_FW_VERSION);

  Serial.println("[MyApp] boot");
  delay(1200);                 // allow early noise to pass
  OtaConsole::begin(Serial2);  // logs to ESP32 (/console)
  OtaConsole::setEnabled(true);

  // Motor driver init
  Steer::begin();

  // Boot banner
  SLOGF("MyApp RAW+STEER FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);
  SLOGF("BOOT_PINS fwd=%d rev=%d (0=LOW,1=HIGH)\r\n",
        (int)digitalRead(PIN_FWD), (int)digitalRead(PIN_REV));
}

void loop(){
  using namespace MM;

  // OTA tick (console will be force-muted by the OTA module on HELLO)
  OtaUpdater::tick();

  // ~33 Hz sampling tick (RAW stream + telemetry)
  static uint32_t t_sample=0;
  const uint32_t nowUs = micros();
  if (nowUs - t_sample >= 30000){ // ~30ms
    t_sample = nowUs;
    const uint32_t nowMs = millis();

    uint32_t rcS_last_us, rcT_last_us;

    // Copy RC widths atomically
    noInterrupts();
    g_s2_us = rcS_width_us;
    g_t2_us = rcT_width_us;
    rcS_last_us = rcS_last_update_us;
    rcT_last_us = rcT_last_update_us;
    interrupts();

    // Sample raw
    sampleAnalog();
    sampleFnr();

    const int16_t s2_map = mapRcSteer_us_to_1000(g_s2_us);
    const int16_t t2_map = mapRcThr_us_to_1000(g_t2_us);

    uint16_t as_raw = 0;
    float    as_deg = 0.0f;
    const bool haveAngle = angleSnapshot(as_raw, as_deg);
    const int16_t as_deg_centi = haveAngle ? toCenti(as_deg) : INT16_MIN;
    const int16_t steer_err_centi = haveAngle ? toCenti(Steer::lastErrorDeg()) : INT16_MIN;

    uint16_t flags = 0;
    if (haveAngle) flags |= TELEM_FLAG_ANGLE_FRESH;
    if (g_fnr == FNR_FAULT) flags |= TELEM_FLAG_FNR_FAULT;

    static constexpr uint32_t RC_STALE_US = 250000; // 250 ms
    const bool rcFresh = ((uint32_t)(nowUs - rcS_last_us) <= RC_STALE_US) &&
                         ((uint32_t)(nowUs - rcT_last_us) <= RC_STALE_US);
    if (rcFresh) flags |= TELEM_FLAG_RC_ACTIVE;

#if RAW_MIRROR_MODE
    SLOGF("RAW S1=%u  T1=%u  S2_us=%d(%d)  T2_us=%d(%d)  AS=%u(%d)  FNR=%s\r\n",
          (unsigned)g_s1,
          (unsigned)g_t1,
          (int)g_s2_us, (int)s2_map,
          (int)g_t2_us, (int)t2_map,
          (unsigned)(haveAngle ? as_raw : 0u),
          (int)as_deg_centi,
          fnrWord(g_fnr));
#endif

    TelemetryPayload telem{};
    telem.flags           = flags;
    telem.s1              = g_s1;
    telem.t1              = g_t1;
    telem.s2_us           = g_s2_us;
    telem.t2_us           = g_t2_us;
    telem.s2_map          = s2_map;
    telem.t2_map          = t2_map;
    telem.as_raw          = haveAngle ? as_raw : 0u;
    telem.as_deg_centi    = as_deg_centi;
    telem.gear            = static_cast<uint8_t>(g_fnr);
    telem.steer_target    = static_cast<uint8_t>(Steer::lastTarget());
    telem.steer_state     = static_cast<uint8_t>(Steer::state());
    telem.reserved        = 0;
    telem.steer_err_centi = steer_err_centi;
    if (!OtaUpdater::inProgress()){
      telem.seq    = ++g_telemSeq;
      telem.millis = nowMs;
      emitTelemetry(telem);
    }
  }

  // ~50 Hz steering control loop
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
