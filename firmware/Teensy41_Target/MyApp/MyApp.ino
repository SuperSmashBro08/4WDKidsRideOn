// ============ MyApp.ino (V 3.3.0) ============

#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"
#include <stdarg.h>
#include <string.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

#define APP_FW_VERSION   "V 3.3.0"
#define LED_PIN          13
#define BLINK_MS         500

// ----- IO pins -----
static constexpr uint8_t PIN_FWD    = 10;
static constexpr uint8_t PIN_REV    = 11;
static constexpr uint8_t PIN_POT_S1 = 14;
static constexpr uint8_t PIN_POT_T1 = 27;
static constexpr uint8_t PIN_RC_S   = 4;
static constexpr uint8_t PIN_RC_T   = 5;

// ----- ADC helpers -----
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_MAX  = 4095;

struct WatchGroup { bool s1=false, s2=false, t1=false, t2=false, fnr=false; } g_watch;

// ----- Logging sink control (default OTA only) -----
enum class LogSink : uint8_t { OTA, USB, BOTH };
static LogSink g_logSink = LogSink::OTA;
static void LOG_raw(const char* s){
  if (g_logSink==LogSink::OTA || g_logSink==LogSink::BOTH) OtaConsole::printf("%s", s);
  if (g_logSink==LogSink::USB || g_logSink==LogSink::BOTH) Serial.print(s);
}
static void SLOGF(const char* fmt, ...){
  char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  LOG_raw(buf);
}

// ================= NAMESPACE =================
namespace MM {
  // --- forward decl (throttle inhibit) ---
static inline bool throttleInhibitActive();


// ===== Gear / FNR =====
enum FnrState : uint8_t { FNR_NEU=0, FNR_FWD=1, FNR_REV=2, FNR_FAULT=3 };
static FnrState g_fnr = FNR_NEU;
static inline const char* fnrWord(FnrState s){
  switch(s){ case FNR_FWD: return "forward"; case FNR_REV: return "reverse";
             case FNR_NEU: return "neutral"; default: return "fault"; }
}

// ===== Live values =====
static uint16_t g_s1=0, g_t1=0;    // raw analog
static int16_t  g_s2_us=1500, g_t2_us=1500; // last RC pulse widths (us)
static bool     g_srcRc = true;

// ===== Policy =====
enum Direction : uint8_t { DIR_NEU, DIR_FWD, DIR_REV };
struct Desired { Direction driveDir; int16_t driveCmd; int16_t steerCmd; bool steerNeutralHold; };

// ===== Utilities =====
static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi){ if(v<lo) return lo; if(v>hi) return hi; return (int16_t)v; }
static int16_t scaleSteerAnalog(uint16_t raw){ int32_t v=(int32_t)raw-2048; v=(v*1000)/2048; return clamp_i16(v,-1000,1000); }
static int16_t scaleThrottleAnalog(uint16_t raw){ int32_t v=((int32_t)raw*1000)/4095; return clamp_i16(v,0,1000); }

// ===== RC PWM capture (interrupt) =====
static constexpr uint16_t RC_MIN_US=1000, RC_MID_US=1500, RC_MAX_US=2000, RC_DB_US=20;
static volatile uint32_t rcS_rise_us=0, rcS_last_update_us=0;
static volatile uint16_t rcS_width_us=1500;
static volatile uint32_t rcT_rise_us=0, rcT_last_update_us=0;
static volatile uint16_t rcT_width_us=1500;

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

// ===== Median filter helper =====
template<typename T, uint8_t N>
struct Median {
  T buf[N]{}; uint8_t idx=0; bool filled=false;
  void add(T v){ buf[idx]=v; idx=(idx+1)%N; if(idx==0) filled=true; }
  T get() const {
    T tmp[N];
    uint8_t count = filled? N : idx;
    for(uint8_t i=0;i<count;i++) tmp[i]=buf[i];
    // simple insertion sort (N small)
    for(uint8_t i=1;i<count;i++){ T k=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>k){ tmp[j+1]=tmp[j]; j--; } tmp[j+1]=k; }
    return count? tmp[count/2] : (T)0;
  }
  bool ready() const { return filled || idx>=3; }
};

// ===== User calibration (locked from you) =====
struct UserCal {
  // Steering 1 raw
  uint16_t s1_min = 427, s1_max = 2825;
  // Steering 2 deadband around zero (mapped after median)
  int16_t  s2_deadband = 220;

  // Throttle 1 (foot)
  uint16_t t1_idle = 1108, t1_idle_db = 60, t1_full_max = 3141;

  // Throttle 2 (RC)
  int16_t  t2_neutral = 502, t2_deadband = 80; // in 0..1000 domain after mapping
};
static UserCal g_ucal;

// Derived bins
struct CalBins {
  // S1
  uint16_t s1_left_max, s1_ctr_min, s1_ctr_max, s1_right_min;
  // S2
  int16_t  s2_left_max, s2_ctr_min, s2_ctr_max, s2_right_min;
  // T1
  uint16_t t1_off_min, t1_off_max, t1_low_min, t1_low_max, t1_med_min, t1_med_max, t1_high_min, t1_high_max;
  // T2
  int16_t  t2_rev_hi_max, t2_rev_med_min, t2_rev_med_max,
           t2_rev_low_min, t2_rev_low_max,
           t2_off_min, t2_off_max,
           t2_fwd_low_min, t2_fwd_low_max,
           t2_fwd_med_min, t2_fwd_med_max,
           t2_fwd_hi_min,  t2_fwd_hi_max;
};
static CalBins g_bins;

static void computeBinsFromUserCal(){
  // S1 thirds
  const uint32_t span=(uint32_t)g_ucal.s1_max - g_ucal.s1_min;
  const uint16_t third=(uint16_t)(span/3);
  g_bins.s1_left_max = g_ucal.s1_min + third;
  g_bins.s1_ctr_min  = g_bins.s1_left_max + 1;
  g_bins.s1_ctr_max  = g_ucal.s1_min + (2*third);
  g_bins.s1_right_min= g_bins.s1_ctr_max + 1;

  // S2 deadband around 0
  const int16_t d2=g_ucal.s2_deadband;
  g_bins.s2_left_max = -d2 - 1;
  g_bins.s2_ctr_min  = -d2;
  g_bins.s2_ctr_max  = +d2;
  g_bins.s2_right_min= +d2 + 1;

  // T1 off + thirds
  g_bins.t1_off_min  = (g_ucal.t1_idle > g_ucal.t1_idle_db) ? (g_ucal.t1_idle - g_ucal.t1_idle_db) : 0;
  g_bins.t1_off_max  = g_ucal.t1_idle + g_ucal.t1_idle_db;
  const uint32_t t1_span = (g_ucal.t1_full_max > g_bins.t1_off_max) ? ((uint32_t)g_ucal.t1_full_max - g_bins.t1_off_max) : 0;
  const uint16_t t1_bin  = (uint16_t)((t1_span>0)? (t1_span/3) : 0);
  g_bins.t1_low_min  = g_bins.t1_off_max + 1;
  g_bins.t1_low_max  = g_bins.t1_low_min + (t1_bin? (t1_bin-1):0);
  g_bins.t1_med_min  = g_bins.t1_low_max + 1;
  g_bins.t1_med_max  = g_bins.t1_med_min + (t1_bin? (t1_bin-1):0);
  g_bins.t1_high_min = g_bins.t1_med_max + 1;
  g_bins.t1_high_max = g_ucal.t1_full_max;

  // T2: neutral ± DB, thirds each side (0..1000 space)
  const int16_t n=g_ucal.t2_neutral, db=g_ucal.t2_deadband;
  g_bins.t2_off_min = n-db; g_bins.t2_off_max = n+db;
  const int16_t rev_span = (g_bins.t2_off_min>0)? g_bins.t2_off_min : 0;
  const int16_t fwd_span = 1000 - g_bins.t2_off_max;
  const int16_t rev_bin  = (rev_span>0)? (rev_span/3) : 0;
  const int16_t fwd_bin  = (fwd_span>0)? (fwd_span/3) : 0;
  g_bins.t2_rev_hi_max = (rev_bin>0)? (rev_bin-1):0;
  g_bins.t2_rev_med_min= g_bins.t2_rev_hi_max+1;
  g_bins.t2_rev_med_max= (rev_bin>0)? ((2*rev_bin)-1):0;
  g_bins.t2_rev_low_min= g_bins.t2_rev_med_max+1;
  g_bins.t2_rev_low_max= (g_bins.t2_off_min>0)? (g_bins.t2_off_min-1):0;
  g_bins.t2_fwd_low_min= g_bins.t2_off_max+1;
  g_bins.t2_fwd_low_max= g_bins.t2_fwd_low_min + (fwd_bin? (fwd_bin-1):0);
  g_bins.t2_fwd_med_min= g_bins.t2_fwd_low_max+1;
  g_bins.t2_fwd_med_max= g_bins.t2_fwd_med_min + (fwd_bin? (fwd_bin-1):0);
  g_bins.t2_fwd_hi_min = g_bins.t2_fwd_med_max+1;
  g_bins.t2_fwd_hi_max = 1000;
}

// ===== Labels =====
enum class SteeringPos    : uint8_t { Left, Center, Right };
enum class ThrottleLvl    : uint8_t { Off, Low, Medium, High };
enum class Throttle1DirLv : uint8_t { RevHigh, RevMed, RevLow, Off, FwdLow, FwdMed, FwdHigh };
enum class Throttle2Lvl   : uint8_t { RevHigh, RevMed, RevLow, Off, FwdLow, FwdMed, FwdHigh };

static inline const char* steerToText(SteeringPos s){
  switch(s){ case SteeringPos::Left: return "left"; case SteeringPos::Center: return "center"; default: return "right"; }
}
static inline const char* t1ToText(Throttle1DirLv t){
  switch(t){
    case Throttle1DirLv::RevHigh: return "reverse high";
    case Throttle1DirLv::RevMed:  return "reverse medium";
    case Throttle1DirLv::RevLow:  return "reverse low";
    case Throttle1DirLv::Off:     return "off";
    case Throttle1DirLv::FwdLow:  return "forward low";
    case Throttle1DirLv::FwdMed:  return "forward medium";
    default:                      return "forward high";
  }
}
static inline const char* t2ToText(Throttle2Lvl t){
  switch(t){
    case Throttle2Lvl::RevHigh: return "reverse high";
    case Throttle2Lvl::RevMed:  return "reverse medium";
    case Throttle2Lvl::RevLow:  return "reverse low";
    case Throttle2Lvl::Off:     return "off";
    case Throttle2Lvl::FwdLow:  return "forward low";
    case Throttle2Lvl::FwdMed:  return "forward medium";
    default:                    return "forward high";
  }
}

// ===== Channel classifiers (median + stability hold) =====
template<typename Tok, uint16_t HoldMs>
struct StableLatch {
  Tok cur{};
  Tok pending{};
  bool init{false};
  bool hasPending{false};
  uint32_t pendingStart{0};

  bool push(Tok incoming, uint32_t nowMs){
    if(!init){ cur=incoming; init=true; hasPending=false; pending=incoming; pendingStart=nowMs; return true; }
    if(incoming==cur){ hasPending=false; return false; }
    if(!hasPending || incoming!=pending){ pending=incoming; pendingStart=nowMs; hasPending=true; return false; }
    if((uint32_t)(nowMs - pendingStart) >= HoldMs){ cur=pending; hasPending=false; return true; }
    return false;
  }
};

static Median<uint16_t,8> medS1;
static Median<uint16_t,8> medT1;
static Median<int16_t,8>  medS2;   // mapped to -1000..+1000 before push
static Median<int16_t,8>  medT2;   // mapped to 0..1000 before push

static StableLatch<SteeringPos,200>    latchS1;
static StableLatch<SteeringPos,200>    latchS2;
static StableLatch<Throttle1DirLv,250> latchT1;
static StableLatch<Throttle2Lvl,250>   latchT2;
static StableLatch<FnrState,150>       latchGear;

// map functions use your bins
static inline SteeringPos mapSteering1(uint16_t raw){
  if(raw <= g_bins.s1_left_max) return SteeringPos::Left;
  if(raw <= g_bins.s1_ctr_max)  return SteeringPos::Center;
  return SteeringPos::Right;
}
static inline SteeringPos mapSteering2_val(int16_t v){
  if(v <= g_bins.s2_left_max)   return SteeringPos::Left;
  if(v <  g_bins.s2_right_min)  return SteeringPos::Center; // within deadband window
  return SteeringPos::Right;
}
static inline ThrottleLvl mapThrottle1Level(uint16_t raw){
  if(raw >= g_bins.t1_off_min && raw <= g_bins.t1_off_max) return ThrottleLvl::Off;
  if(raw <= g_bins.t1_low_max)   return ThrottleLvl::Low;
  if(raw <= g_bins.t1_med_max)   return ThrottleLvl::Medium;
  return ThrottleLvl::High;
}
static inline Throttle1DirLv mapThrottle1Dir(ThrottleLvl lvl, FnrState gear){
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
static inline Throttle2Lvl mapThrottle2_val(int16_t v){ // v = 0..1000
  if (throttleInhibitActive()) return Throttle2Lvl::Off;
  if(v<0)v=0; if(v>1000)v=1000;
  if(v <= g_bins.t2_rev_hi_max)                    return Throttle2Lvl::RevHigh;
  if(v <= g_bins.t2_rev_med_max)                   return Throttle2Lvl::RevMed;
  if(v <= g_bins.t2_rev_low_max)                   return Throttle2Lvl::RevLow;
  if(v >= g_bins.t2_off_min && v <= g_bins.t2_off_max) return Throttle2Lvl::Off;
  if(v <= g_bins.t2_fwd_low_max)                   return Throttle2Lvl::FwdLow;
  if(v <= g_bins.t2_fwd_med_max)                   return Throttle2Lvl::FwdMed;
  return Throttle2Lvl::FwdHigh;
}

// ===== Startup throttle inhibit =====
static const uint32_t STARTUP_INHIBIT_MS = 5000;
static uint32_t g_boot_ms = 0;
static inline bool throttleInhibitActive(){ return (uint32_t)(millis() - g_boot_ms) < STARTUP_INHIBIT_MS; }

// ===== Sample + classify + print (EVENT ONLY) =====
static void sampleAnalog(){ g_s1=analogRead(PIN_POT_S1); g_t1=analogRead(PIN_POT_T1); }
static void sampleFnr(){
  const bool fwd_low=(digitalRead(PIN_FWD)==LOW), rev_low=(digitalRead(PIN_REV)==LOW);
  if( fwd_low &&  rev_low) g_fnr=FNR_FAULT;
  else if( fwd_low && !rev_low) g_fnr=FNR_FWD;
  else if(!fwd_low &&  rev_low) g_fnr=FNR_REV;
  else g_fnr=FNR_NEU;
}
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

static void tickEventPipeline(){
  // 1) collect new raw
  uint16_t s1=g_s1, t1=g_t1;
  int16_t  s2_mapped = mapRcSteer_us_to_1000(g_s2_us);
  int16_t  t2_mapped = mapRcThr_us_to_1000(g_t2_us);

  // 2) push into medians
  medS1.add(s1); medT1.add(t1);
  medS2.add(s2_mapped); medT2.add(t2_mapped);

  if (!(medS1.ready() && medT1.ready() && medS2.ready() && medT2.ready())) return; // wait until warmed up

  // 3) classify medians
  SteeringPos s1Tok = mapSteering1(medS1.get());
  SteeringPos s2Tok = mapSteering2_val(medS2.get());
  ThrottleLvl t1Lvl = mapThrottle1Level(medT1.get());
  if (throttleInhibitActive()) t1Lvl = ThrottleLvl::Off;
  if (g_fnr==FNR_NEU || g_fnr==FNR_FAULT) t1Lvl = ThrottleLvl::Off;
  Throttle1DirLv t1Tok = mapThrottle1Dir(t1Lvl, g_fnr);
  Throttle2Lvl   t2Tok = mapThrottle2_val(medT2.get());
  FnrState       gTok  = g_fnr;

  const uint32_t nowMs = millis();

  if (latchS1.push(s1Tok, nowMs)) SLOGF("S steering: %s\r\n", steerToText(latchS1.cur));
  if (latchS2.push(s2Tok, nowMs)) SLOGF("S steering rc: %s\r\n", steerToText(latchS2.cur));
  if (latchT1.push(t1Tok, nowMs)) {
    if (latchT1.cur==Throttle1DirLv::Off) {
      SLOGF("S throttle: off\r\n");
    } else {
      SLOGF("S throttle: %s\r\n", t1ToText(latchT1.cur));
    }
  }
  if (latchT2.push(t2Tok, nowMs)) SLOGF("S throttle rc: %s\r\n", t2ToText(latchT2.cur));
  if (latchGear.push(gTok, nowMs)) SLOGF("S gear: %s\r\n", fnrWord(latchGear.cur));

  // optional raw watches
  if (g_watch.s1) SLOGF("S S1 raw=%u med=%u\r\n", s1, medS1.get());
  if (g_watch.t1) SLOGF("S T1 raw=%u med=%u\r\n", t1, medT1.get());
  if (g_watch.s2) SLOGF("S S2 us=%d map=%d med=%d\r\n", (int)g_s2_us, (int)s2_mapped, (int)medS2.get());
  if (g_watch.t2) SLOGF("S T2 us=%d map=%d med=%d\r\n", (int)g_t2_us, (int)t2_mapped, (int)medT2.get());
}

// ===== Policy compute/apply (unchanged behavior) =====
static constexpr int16_t DEAD_BAND=20;
static void computeDesired(struct Desired& out){
  const bool useRc=g_srcRc;
  const int16_t steer_local=scaleSteerAnalog(g_s1);
  const int16_t thr_local  =scaleThrottleAnalog(g_t1);
  const int16_t steer_rc   = clamp_i16(medS2.get(), -1000, 1000);
  const int16_t thr_rc     = clamp_i16(medT2.get(), 0, 1000);

  out.steerCmd = useRc ? steer_rc : steer_local;
  out.steerNeutralHold = false;

  if (throttleInhibitActive()){ out.driveDir=DIR_NEU; out.driveCmd=0; return; }

  if(useRc){
    if(thr_rc>DEAD_BAND){ out.driveDir=DIR_FWD; out.driveCmd=thr_rc; }
    else { out.driveDir=DIR_NEU; out.driveCmd=0; }
  } else {
    out.driveCmd = thr_local;
    switch(g_fnr){
      case FNR_FWD: out.driveDir=DIR_FWD; break;
      case FNR_REV: out.driveDir=DIR_REV; break;
      default:      out.driveDir=DIR_NEU; out.driveCmd=0; out.steerNeutralHold=true; break;
    }
  }
}
static void applyOutputs(const Desired& d){ (void)d; /* TODO: motor outputs */ }

} // namespace MM

// ===== USB commands =====
static void handleUsbCommandsOnce() {
  using namespace MM;
  static String line;
  while (Serial.available()){
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
          String ch,onoff; int sp2=a1.indexOf(' '); if(sp2>0){ ch=a1.substring(0,sp2); onoff=a1.substring(sp2+1); }
          ch.toLowerCase(); onoff.toLowerCase(); bool en=(onoff=="on"||onoff=="1"||onoff=="true");
          if(ch=="s1"||ch=="all") g_watch.s1=en;
          if(ch=="s2"||ch=="all") g_watch.s2=en;
          if(ch=="t1"||ch=="all") g_watch.t1=en;
          if(ch=="t2"||ch=="all") g_watch.t2=en;
          if(ch=="fnr"||ch=="all") g_watch.fnr=en;
          Serial.printf("[MyApp] watch: s1=%d s2=%d t1=%d t2=%d fnr=%d\r\n",
                        g_watch.s1,g_watch.s2,g_watch.t1,g_watch.t2,g_watch.fnr);
        } else if (verb=="LOG"){
          a1.toLowerCase();
          if(a1=="ota"){ g_logSink=LogSink::OTA; Serial.println("[MyApp] log sink: OTA"); }
          else if(a1=="usb"){ g_logSink=LogSink::USB; Serial.println("[MyApp] log sink: USB"); }
          else if(a1=="both"){ g_logSink=LogSink::BOTH; Serial.println("[MyApp] log sink: BOTH"); }
          else Serial.println("usage: LOG ota|usb|both");
        } else {
          Serial.println("Unknown. Try: SRC rc|local|?  |  WATCH <ch|all> on|off  |  LOG ota|usb|both");
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
  attachInterrupt(digitalPinToInterrupt(PIN_RC_S), MM::isrRcSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_T), MM::isrRcThrottle, CHANGE);

  Serial.begin(115200);
  Serial2.begin(115200);

  // Delay enabling OTA console to avoid boot-time echo bursts from the bridge
  OtaUpdater::begin(Serial2);
  OtaUpdater::setAppVersion(APP_FW_VERSION);

  MM::g_boot_ms = millis();
  MM::computeBinsFromUserCal();

  Serial.println("[MyApp] boot");
  // Hold console for 1.2s, then begin
  delay(1200);
  OtaConsole::begin(Serial2);

  SLOGF("S MyApp FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);
  SLOGF("S BOOT_PINS fwd=%d rev=%d (0=LOW,1=HIGH)\r\n",
        (int)digitalRead(PIN_FWD), (int)digitalRead(PIN_REV));
  SLOGF("S failsafe: throttle-inhibit ON (%lu ms)\r\n", (unsigned long)MM::STARTUP_INHIBIT_MS);
}

void loop(){
  using namespace MM;

  OtaUpdater::tick();
  OtaConsole::setEnabled(!OtaUpdater::inProgress());

  static bool printed_inhibit_off=false;
  if (!printed_inhibit_off && !throttleInhibitActive()){
    printed_inhibit_off=true;
    SLOGF("S failsafe: throttle-inhibit OFF\r\n");
  }

  // 100 Hz sampling tick
  static uint32_t t_sample=0;
  if (micros() - t_sample >= 10000){ // 10ms
    t_sample = micros();

    // Pull latest RC widths atomically
    noInterrupts(); g_s2_us = MM::rcS_width_us; g_t2_us = MM::rcT_width_us; interrupts();

    sampleAnalog();
    sampleFnr();
    tickEventPipeline();       // <- event-only classifier + prints on change

    Desired des; computeDesired(des); applyOutputs(des);
  }

  static uint32_t t_led=0; static bool led=false;
  if (!OtaUpdater::inProgress() && millis() - t_led >= BLINK_MS){
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led? HIGH:LOW);
  }

  handleUsbCommandsOnce();
}
