#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"
#include <stdarg.h>

// ----- Teensy compatibility shims -----
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

#define APP_FW_VERSION   "V 3.1.4"
#define LED_PIN          13
#define BLINK_MS         500

// ========== IO PINS ==========
static constexpr uint8_t PIN_FWD    = 10; // forward switch input (active LOW)
static constexpr uint8_t PIN_REV    = 11; // reverse switch input (active LOW)
static constexpr uint8_t PIN_POT_S1 = 14; // Steering 1 (analog)
static constexpr uint8_t PIN_POT_T1 = 27; // Throttle 1 (analog)
static constexpr uint8_t PIN_RC_S   = 4;  // Steering 2 (RC PWM)
static constexpr uint8_t PIN_RC_T   = 5;  // Throttle 2 (RC PWM)

// ======== ADC helpers ========
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_MAX  = 4095;

// ======== Channel watching (optional) ========
struct WatchGroup { bool s1=false, s2=false, t1=false, t2=false, fnr=false; } g_watch;

// ======== Mirrored logging helper ========
static void SLOGF(const char* fmt, ...) {
  char buf[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  OtaConsole::printf("%s", buf);
}

// ================= APP LOGIC (namespace) =================
namespace MM {

// ---------- Gear / FNR ----------
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

// ---------- Values ----------
static uint16_t g_s1=0, g_t1=0;  // analog raw 0..4095
static int16_t  g_s2_cmd=0;      // RC steer -1000..+1000
static int16_t  g_t2_cmd=0;      // RC throttle 0..1000 (unsigned)
static bool     g_srcRc = true;  // RC priority

// ---------- Policy ----------
enum Direction : uint8_t { DIR_NEU, DIR_FWD, DIR_REV };
struct Desired { Direction driveDir; int16_t driveCmd; int16_t steerCmd; bool steerNeutralHold; };

// ---------- Small utils ----------
static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo; if (v > hi) return hi; return (int16_t)v;
}
static int16_t scaleSteerAnalog(uint16_t raw){ int32_t v=(int32_t)raw-2048; v=(v*1000)/2048; return clamp_i16(v,-1000,1000); }
static int16_t scaleThrottleAnalog(uint16_t raw){ int32_t v=((int32_t)raw*1000)/4095; return clamp_i16(v,0,1000); }

// ---------- RC PWM capture ----------
#define RC_THROTTLE_CENTERED 0
static constexpr uint16_t RC_MIN_US=1000, RC_MID_US=1500, RC_MAX_US=2000, RC_DB_US=20;
static constexpr uint32_t RC_STALE_US=50000;

static volatile uint32_t rcS_rise_us=0, rcS_last_update_us=0;
static volatile uint16_t rcS_width_us=1500;
static volatile uint32_t rcT_rise_us=0, rcT_last_update_us=0;
static volatile uint16_t rcT_width_us=1500;

static int16_t mapPwmSigned1000(uint16_t us){
  if(us<RC_MIN_US)us=RC_MIN_US; if(us>RC_MAX_US)us=RC_MAX_US;
  int32_t x=(int32_t)us-RC_MID_US; if(abs(x)<=RC_DB_US) return 0;
  int32_t out=(x*1000)/(RC_MAX_US-RC_MID_US); return clamp_i16(out,-1000,1000);
}
static int16_t mapPwmUnsigned1000(uint16_t us){
  if(us<RC_MIN_US)us=RC_MIN_US; if(us>RC_MAX_US)us=RC_MAX_US;
  int32_t out=((int32_t)us-RC_MIN_US)*1000/(RC_MAX_US-RC_MIN_US);
  if(out<0)out=0; if(out>1000)out=1000; return (int16_t)out;
}

static void IRAM_ATTR isrRcSteer() {
  uint32_t now=micros();
  if (digitalReadFast(PIN_RC_S)) rcS_rise_us=now;
  else { uint32_t w=now-rcS_rise_us; if(w<=3000){ rcS_width_us=(uint16_t)w; rcS_last_update_us=now; } }
}
static void IRAM_ATTR isrRcThrottle() {
  uint32_t now=micros();
  if (digitalReadFast(PIN_RC_T)) rcT_rise_us=now;
  else { uint32_t w=now-rcT_rise_us; if(w<=3000){ rcT_width_us=(uint16_t)w; rcT_last_update_us=now; } }
}

static void sampleRcPwm() {
  uint16_t s_us, t_us; uint32_t s_age,t_age, now=micros();
  noInterrupts(); s_us=rcS_width_us; t_us=rcT_width_us; s_age=now-rcS_last_update_us; t_age=now-rcT_last_update_us; interrupts();
  g_s2_cmd = (s_age<=RC_STALE_US) ? mapPwmSigned1000(s_us) : 0;
#if RC_THROTTLE_CENTERED
  g_t2_cmd = (t_age<=RC_STALE_US) ? mapPwmSigned1000(t_us) : 0;
#else
  g_t2_cmd = (t_age<=RC_STALE_US) ? mapPwmUnsigned1000(t_us) : 0;
#endif
}

// ---------- Sampling ----------
static void samplePots(){ g_s1=analogRead(PIN_POT_S1); g_t1=analogRead(PIN_POT_T1); }
static void sampleFnr(){
  const bool fwd_low=(digitalRead(PIN_FWD)==LOW), rev_low=(digitalRead(PIN_REV)==LOW);
  if( fwd_low &&  rev_low) g_fnr=FNR_FAULT;
  else if( fwd_low && !rev_low) g_fnr=FNR_FWD;
  else if(!fwd_low &&  rev_low) g_fnr=FNR_REV;
  else g_fnr=FNR_NEU;
}

// ---------- Policy ----------
static constexpr int16_t DEAD_BAND=20;
static void computeDesired(Desired& out){
  const bool useRc=g_srcRc;
  const int16_t steer_local=scaleSteerAnalog(g_s1);
  const int16_t thr_local  =scaleThrottleAnalog(g_t1);
  const int16_t steer_rc=g_s2_cmd;
  const int16_t thr_rc  =g_t2_cmd;

  if(useRc){
    out.steerCmd=steer_rc; out.steerNeutralHold=false;
#if RC_THROTTLE_CENTERED
    if(thr_rc>+DEAD_BAND){ out.driveDir=DIR_FWD; out.driveCmd=thr_rc; }
    else if(thr_rc<-DEAD_BAND){ out.driveDir=DIR_REV; out.driveCmd=-thr_rc; }
    else { out.driveDir=DIR_NEU; out.driveCmd=0; }
#else
    if(thr_rc>DEAD_BAND){ out.driveDir=DIR_FWD; out.driveCmd=thr_rc; }
    else { out.driveDir=DIR_NEU; out.driveCmd=0; }
#endif
  } else {
    out.steerCmd=steer_local; out.steerNeutralHold=false; out.driveCmd=thr_local;
    switch(g_fnr){
      case FNR_FWD: out.driveDir=DIR_FWD; break;
      case FNR_REV: out.driveDir=DIR_REV; break;
      default:      out.driveDir=DIR_NEU; out.driveCmd=0; out.steerNeutralHold=true; break;
    }
  }
}

static void applyOutputs(const Desired& d){ (void)d; /* TODO: motor outputs */ }

// ======================
// AUTO-CALIBRATION LAYER
// ======================
struct UserCal {
  // Steering 1 (analog raw): min=full left, max=full right
  uint16_t s1_min = 427;
  uint16_t s1_max = 2825;

  // Steering 2 (RC signed -1000..+1000)
  int16_t  s2_deadband = 200; // center band ±deadband

  // Throttle 1 (analog raw): idle-centered OFF band and full-press max
  uint16_t t1_idle     = 1108; // your neutral reading
  uint16_t t1_idle_db  = 60;   // ± band for OFF (tweak if needed)
  uint16_t t1_full_max = 3141; // full press

  // Throttle 2 (RC unsigned 0..1000)
  int16_t  t2_neutral  = 502;
  int16_t  t2_deadband = 30;
};
static UserCal g_ucal;

struct CalBins {
  // Steering 1
  uint16_t s1_left_max, s1_ctr_min, s1_ctr_max, s1_right_min;
  // Steering 2
  int16_t  s2_left_max, s2_ctr_min, s2_ctr_max, s2_right_min;
  // Throttle 1
  uint16_t t1_off_min, t1_off_max,
           t1_low_min, t1_low_max, t1_med_min, t1_med_max, t1_high_min, t1_high_max;
  // Throttle 2
  int16_t  t2_rev_hi_max, t2_rev_med_min, t2_rev_med_max,
           t2_rev_low_min, t2_rev_low_max,
           t2_off_min, t2_off_max,
           t2_fwd_low_min, t2_fwd_low_max,
           t2_fwd_med_min, t2_fwd_med_max,
           t2_fwd_hi_min, t2_fwd_hi_max;
};
static CalBins g_bins;

static void computeBinsFromUserCal(){
  // Steering 1: thirds
  const uint32_t span=(uint32_t)g_ucal.s1_max - g_ucal.s1_min;
  const uint16_t third=(uint16_t)(span/3);
  g_bins.s1_left_max = g_ucal.s1_min + third;
  g_bins.s1_ctr_min  = g_bins.s1_left_max + 1;
  g_bins.s1_ctr_max  = g_ucal.s1_min + (2*third);
  g_bins.s1_right_min= g_bins.s1_ctr_max + 1;

  // Steering 2: deadband
  const int16_t d2=g_ucal.s2_deadband;
  g_bins.s2_left_max = -d2 - 1;
  g_bins.s2_ctr_min  = -d2;
  g_bins.s2_ctr_max  = +d2;
  g_bins.s2_right_min= +d2 + 1;

  // Throttle 1: OFF band around idle, then split remainder into 3
  g_bins.t1_off_min  = (g_ucal.t1_idle > g_ucal.t1_idle_db) ? (g_ucal.t1_idle - g_ucal.t1_idle_db) : 0;
  g_bins.t1_off_max  = g_ucal.t1_idle + g_ucal.t1_idle_db;

  const uint32_t t1_active_span = (g_ucal.t1_full_max > g_bins.t1_off_max)
      ? ((uint32_t)g_ucal.t1_full_max - g_bins.t1_off_max)
      : 0;
  const uint16_t t1_bin = (uint16_t)((t1_active_span>0) ? (t1_active_span/3) : 0);

  g_bins.t1_low_min  = g_bins.t1_off_max + 1;
  g_bins.t1_low_max  = g_bins.t1_low_min + (t1_bin ? (t1_bin - 1) : 0);
  g_bins.t1_med_min  = g_bins.t1_low_max + 1;
  g_bins.t1_med_max  = g_bins.t1_med_min + (t1_bin ? (t1_bin - 1) : 0);
  g_bins.t1_high_min = g_bins.t1_med_max + 1;
  g_bins.t1_high_max = g_ucal.t1_full_max;

  // Throttle 2: neutral ±deadband, split both sides into 3 bins
  const int16_t n=g_ucal.t2_neutral, db=g_ucal.t2_deadband;
  g_bins.t2_off_min = n - db;
  g_bins.t2_off_max = n + db;

  const int16_t rev_span = (g_bins.t2_off_min>0)? g_bins.t2_off_min : 0;
  const int16_t fwd_span = 1000 - g_bins.t2_off_max;
  const int16_t rev_bin  = (rev_span>0)? (rev_span/3) : 0;
  const int16_t fwd_bin  = (fwd_span>0)? (fwd_span/3) : 0;

  // Reverse [0..off_min-1]
  g_bins.t2_rev_hi_max = (rev_bin>0)? (rev_bin-1) : 0;
  g_bins.t2_rev_med_min= g_bins.t2_rev_hi_max + 1;
  g_bins.t2_rev_med_max= (rev_bin>0)? ((2*rev_bin)-1) : 0;
  g_bins.t2_rev_low_min= g_bins.t2_rev_med_max + 1;
  g_bins.t2_rev_low_max= (g_bins.t2_off_min>0)? (g_bins.t2_off_min-1) : 0;

  // Forward [off_max+1..1000]
  g_bins.t2_fwd_low_min= g_bins.t2_off_max + 1;
  g_bins.t2_fwd_low_max= g_bins.t2_fwd_low_min + (fwd_bin? (fwd_bin-1):0);
  g_bins.t2_fwd_med_min= g_bins.t2_fwd_low_max + 1;
  g_bins.t2_fwd_med_max= g_bins.t2_fwd_med_min + (fwd_bin? (fwd_bin-1):0);
  g_bins.t2_fwd_hi_min = g_bins.t2_fwd_med_max + 1;
  g_bins.t2_fwd_hi_max = 1000;
}

// ======================
// LABELING / MAPPINGS
// ======================
enum class SteeringPos    : uint8_t { Left, Center, Right };
enum class ThrottleLvl    : uint8_t { Off, Low, Medium, High };
enum class Throttle1DirLv : uint8_t { RevHigh, RevMed, RevLow, Off, FwdLow, FwdMed, FwdHigh };
enum class Throttle2Lvl   : uint8_t { RevHigh, RevMed, RevLow, Off, FwdLow, FwdMed, FwdHigh };

static inline SteeringPos mapSteering1(uint16_t raw){
  if(raw <= g_bins.s1_left_max) return SteeringPos::Left;
  if(raw <= g_bins.s1_ctr_max)  return SteeringPos::Center;
  return SteeringPos::Right;
}
static inline SteeringPos mapSteering2(int16_t cmd){
  if(cmd <= g_bins.s2_left_max)  return SteeringPos::Left;
  if(cmd >= g_bins.s2_right_min) return SteeringPos::Right;
  return SteeringPos::Center;
}

// T1 base level, then directional with shifter
static inline ThrottleLvl mapThrottle1Level(uint16_t raw){
  if(raw >= g_bins.t1_off_min && raw <= g_bins.t1_off_max) return ThrottleLvl::Off;
  if(raw <= g_bins.t1_low_max)   return ThrottleLvl::Low;
  if(raw <= g_bins.t1_med_max)   return ThrottleLvl::Medium;
  return ThrottleLvl::High;
}
static inline Throttle1DirLv mapThrottle1Dir(uint16_t raw, FnrState gear){
  ThrottleLvl lvl=mapThrottle1Level(raw);
  if(lvl==ThrottleLvl::Off) return Throttle1DirLv::Off;
  if(gear==FNR_NEU || gear==FNR_FAULT) return Throttle1DirLv::Off;
  if(gear==FNR_FWD){
    switch(lvl){ case ThrottleLvl::Low: return Throttle1DirLv::FwdLow;
                 case ThrottleLvl::Medium: return Throttle1DirLv::FwdMed;
                 default: return Throttle1DirLv::FwdHigh; }
  } else {
    switch(lvl){ case ThrottleLvl::Low: return Throttle1DirLv::RevLow;
                 case ThrottleLvl::Medium: return Throttle1DirLv::RevMed;
                 default: return Throttle1DirLv::RevHigh; }
  }
}

static inline Throttle2Lvl mapThrottle2(int16_t v){
  if(v<0)v=0; if(v>1000)v=1000;
  if(v <= g_bins.t2_rev_hi_max)                    return Throttle2Lvl::RevHigh;
  if(v <= g_bins.t2_rev_med_max)                   return Throttle2Lvl::RevMed;
  if(v <= g_bins.t2_rev_low_max)                   return Throttle2Lvl::RevLow;
  if(v >= g_bins.t2_off_min && v <= g_bins.t2_off_max) return Throttle2Lvl::Off;
  if(v <= g_bins.t2_fwd_low_max)                   return Throttle2Lvl::FwdLow;
  if(v <= g_bins.t2_fwd_med_max)                   return Throttle2Lvl::FwdMed;
  return Throttle2Lvl::FwdHigh;
}

static inline const char* steerToText(SteeringPos s){
  switch(s){ case SteeringPos::Left: return "left";
             case SteeringPos::Center: return "center";
             default: return "right"; }
}
static inline const char* t1ToText(Throttle1DirLv t){
  switch(t){
    case Throttle1DirLv::RevHigh: return "reverse-high";
    case Throttle1DirLv::RevMed:  return "reverse-medium";
    case Throttle1DirLv::RevLow:  return "reverse-low";
    case Throttle1DirLv::Off:     return "off";
    case Throttle1DirLv::FwdLow:  return "forward-low";
    case Throttle1DirLv::FwdMed:  return "forward-medium";
    default:                      return "forward-high";
  }
}
static inline const char* t2ToText(Throttle2Lvl t){
  switch(t){
    case Throttle2Lvl::RevHigh: return "reverse-high";
    case Throttle2Lvl::RevMed:  return "reverse-medium";
    case Throttle2Lvl::RevLow:  return "reverse-low";
    case Throttle2Lvl::Off:     return "off";
    case Throttle2Lvl::FwdLow:  return "forward-low";
    case Throttle2Lvl::FwdMed:  return "forward-medium";
    default:                    return "forward-high";
  }
}

// ======================
// STABILITY / DEBOUNCE
// ======================
template<typename T>
struct Stable {
  T cur{};
  T cand{};
  uint32_t since_ms{0};
  bool initialized{false};
};

template<typename T>
static bool updateStable(Stable<T>& st, T newVal, uint32_t now_ms, uint16_t dwell_ms) {
  if (!st.initialized) {
    st.initialized = true; st.cur = newVal; st.cand = newVal; st.since_ms = now_ms;
    return true; // treat first as a "change" so you see the initial state once
  }
  if (newVal == st.cur) {
    st.cand = st.cur; st.since_ms = now_ms; // reset candidate timer while stable
    return false;
  }
  // newVal differs from current
  if (newVal != st.cand) {
    st.cand = newVal; st.since_ms = now_ms; // start new dwell window
    return false;
  }
  // same candidate persisting — check dwell
  if ((uint32_t)(now_ms - st.since_ms) >= dwell_ms) {
    st.cur = st.cand; // commit
    return true;
  }
  return false;
}

// Dwell choices (ms)
static constexpr uint16_t DWELL_STEER_MS   = 120;
static constexpr uint16_t DWELL_THR_MS     = 120;
static constexpr uint16_t DWELL_GEAR_MS    = 80;

// ======== CHANGE-ONLY PRINTER with stability ========
static void printStableIfChanged(){
  static Stable<SteeringPos>    s1St;
  static Stable<SteeringPos>    s2St;
  static Stable<Throttle1DirLv> t1St;
  static Stable<Throttle2Lvl>   t2St;
  static Stable<FnrState>       gSt;

  const uint32_t now = millis();

  const SteeringPos    s1Now = mapSteering1(g_s1);
  const SteeringPos    s2Now = mapSteering2(g_s2_cmd);
  const Throttle1DirLv t1Now = mapThrottle1Dir(g_t1, g_fnr);
  const Throttle2Lvl   t2Now = mapThrottle2(g_t2_cmd);
  const FnrState       gNow  = g_fnr;

  if (updateStable(s1St, s1Now, now, DWELL_STEER_MS)) SLOGF("S steering1: %s\r\n", steerToText(s1St.cur));
  if (updateStable(s2St, s2Now, now, DWELL_STEER_MS)) SLOGF("S steering2: %s\r\n", steerToText(s2St.cur));
  if (updateStable(t1St, t1Now, now, DWELL_THR_MS  )) SLOGF("S throttle1: %s\r\n", t1ToText(t1St.cur));
  if (updateStable(t2St, t2Now, now, DWELL_THR_MS  )) SLOGF("S throttle2: %s\r\n", t2ToText(t2St.cur));
  if (updateStable(gSt,  gNow,  now, DWELL_GEAR_MS )) SLOGF("S gear: %s\r\n",     fnrWord(gSt.cur));

  // Optional raw/watch debug
  if (g_watch.s1) SLOGF("S S1 raw=%u v=%.3f\r\n", g_s1, (double)g_s1*ADC_VREF/ADC_MAX);
  if (g_watch.t1) SLOGF("S T1 raw=%u v=%.3f\r\n", g_t1, (double)g_t1*ADC_VREF/ADC_MAX);
  if (g_watch.s2) SLOGF("S S2_CMD=%d\r\n", g_s2_cmd);
  if (g_watch.t2) SLOGF("S T2_CMD=%d\r\n", g_t2_cmd);
  if (g_watch.fnr) SLOGF("S FNR=%s\r\n", fnrToken(g_fnr));
}

} // namespace MM

// ================== Commands over USB ==================
static void handleUsbCommandsOnce() {
  using namespace MM;
  static String line;
  while (Serial.available()) {
    char c=(char)Serial.read(); if(c=='\r') continue;
    if(c=='\n'){
      line.trim();
      if(line.length()){
        String verb,a1; int sp1=line.indexOf(' ');
        if(sp1<0){ verb=line; } else { verb=line.substring(0,sp1); a1=line.substring(sp1+1); a1.trim(); }
        verb.toUpperCase(); a1.toUpperCase();
        if(verb=="SRC"){
          if(a1=="?") Serial.printf("[MyApp] SRC=%s\r\n", MM::g_srcRc? "rc":"local");
          else if(a1=="RC")    { MM::g_srcRc=true;  Serial.println("[MyApp] SRC set to rc"); }
          else if(a1=="LOCAL") { MM::g_srcRc=false; Serial.println("[MyApp] SRC set to local"); }
          else Serial.println("usage: SRC rc|local|?");
        } else if (verb=="WATCH"){
          String ch,onoff; int sp2=a1.indexOf(' ');
          if(sp2>0){ ch=a1.substring(0,sp2); onoff=a1.substring(sp2+1); }
          ch.toLowerCase(); onoff.toLowerCase();
          bool en=(onoff=="on"||onoff=="1"||onoff=="true");
          if(ch=="s1"||ch=="all") g_watch.s1=en;
          if(ch=="s2"||ch=="all") g_watch.s2=en;
          if(ch=="t1"||ch=="all") g_watch.t1=en;
          if(ch=="t2"||ch=="all") g_watch.t2=en;
          if(ch=="fnr"||ch=="all") g_watch.fnr=en;
          Serial.printf("[MyApp] watch: s1=%d s2=%d t1=%d t2=%d fnr=%d\r\n",
                        g_watch.s1,g_watch.s2,g_watch.t1,g_watch.t2,g_watch.fnr);
        } else {
          Serial.println("Unknown. Try: SRC rc|local|?  or WATCH <ch|all> on|off");
        }
      }
      line="";
    } else if(line.length()<120) line+=c;
  }
}

// ================== Setup / Loop ==================
void setup() {
  using namespace MM;

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
  pinMode(PIN_FWD, INPUT_PULLUP);
  pinMode(PIN_REV, INPUT_PULLUP);

  analogReadResolution(12);
  analogReadAveraging(8);

  pinMode(PIN_RC_S, INPUT);
  pinMode(PIN_RC_T, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_S),  MM::isrRcSteer,    CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_T),  MM::isrRcThrottle, CHANGE);

  Serial.begin(115200);
  Serial2.begin(115200);

  OtaUpdater::begin(Serial2);
  OtaUpdater::setAppVersion(APP_FW_VERSION);
  OtaConsole::begin(Serial2);

  MM::computeBinsFromUserCal();

  Serial.println("[MyApp] boot");
  SLOGF("S MyApp FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);
  delay(10);
  SLOGF("S BOOT_PINS fwd=%d rev=%d (0=LOW,1=HIGH)\r\n",
        (int)digitalRead(PIN_FWD), (int)digitalRead(PIN_REV));
}

void loop() {
  using namespace MM;

  OtaUpdater::tick();
  OtaConsole::setEnabled(!OtaUpdater::inProgress());

  static uint32_t t_sample=0;
  if (millis() - t_sample >= 50) {           // a bit faster for smoother stability timing
    t_sample = millis();

    MM::samplePots();
    MM::sampleFnr();
    MM::sampleRcPwm();

    MM::Desired des; MM::computeDesired(des); MM::applyOutputs(des);

    // Stable, change-only printing
    MM::printStableIfChanged();
  }

  static uint32_t t_led=0; static bool led=false;
  if (!OtaUpdater::inProgress() && millis() - t_led >= BLINK_MS) {
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led? HIGH:LOW);
  }

  handleUsbCommandsOnce();
}
