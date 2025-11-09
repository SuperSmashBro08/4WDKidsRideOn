// ============ MyApp.ino (V 3.3.1 RAW MIRROR) ============

#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"
#include <stdarg.h>
#include <string.h>

// ---------- Build / board shims ----------
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

// ---------- App identity ----------
#define APP_FW_VERSION   "V 3.3.1-raw"

// ---------- Blink ----------
#define LED_PIN          13
#define BLINK_MS         500

// ---------- Raw-mirror mode switch ----------
// 1 = print ONLY raw snapshot lines on a fixed cadence
// 0 = (legacy) event/labels printing (disabled by default here)
#define RAW_MIRROR_MODE  1

// ---------- IO pins ----------
static constexpr uint8_t PIN_FWD    = 10;
static constexpr uint8_t PIN_REV    = 11;
static constexpr uint8_t PIN_POT_S1 = 14;
static constexpr uint8_t PIN_POT_T1 = 27;
static constexpr uint8_t PIN_RC_S   = 4;
static constexpr uint8_t PIN_RC_T   = 5;

// ---------- ADC helpers ----------
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_MAX  = 4095;

// ---------- Logging sink control ----------
enum class LogSink : uint8_t { OTA, USB, BOTH };
static LogSink g_logSink = LogSink::OTA;   // default OTA-only to avoid duplicates

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

// ===== Maps used for raw line convenience =====
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

// ===== USB commands (keep simple: LOG sink only) =====
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
  Serial2.begin(115200);

  // Bring up OTA bridge on Serial2; delay console until loader quiet
  OtaUpdater::begin(Serial2);
  OtaUpdater::setAppVersion(APP_FW_VERSION);

  Serial.println("[MyApp] boot");
  delay(1200);                 // allow early noise to pass
  OtaConsole::begin(Serial2);  // logs to ESP32 (/console)
  OtaConsole::setEnabled(true);

  // Boot banner (prefixed "S " by OtaConsole)
  SLOGF("MyApp RAW mirror FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);
  SLOGF("BOOT_PINS fwd=%d rev=%d (0=LOW,1=HIGH)\r\n",
        (int)digitalRead(PIN_FWD), (int)digitalRead(PIN_REV));
}

void loop(){
  using namespace MM;

  // OTA tick & mute console during active OTA
  OtaUpdater::tick();
  OtaConsole::setEnabled(!OtaUpdater::inProgress());

  // 100 Hz sampling tick
  static uint32_t t_sample=0;
  if (micros() - t_sample >= 10000){ // 10ms
    t_sample = micros();

    // Copy RC widths atomically
    noInterrupts(); g_s2_us = rcS_width_us; g_t2_us = rcT_width_us; interrupts();

    // Sample raw
    sampleAnalog();
    sampleFnr();

#if RAW_MIRROR_MODE
    // --- RAW snapshot line every tick (no dedup, no states) ---
    const int16_t s2_map = mapRcSteer_us_to_1000(g_s2_us);
    const int16_t t2_map = mapRcThr_us_to_1000(g_t2_us);

    // One concise line, parseable & human readable.
    // NOTE: OtaConsole prefixes "S " and appends CRLF.
    SLOGF("RAW S1=%u  T1=%u  S2_us=%d(%d)  T2_us=%d(%d)  FNR=%s\r\n",
          (unsigned)g_s1,
          (unsigned)g_t1,
          (int)g_s2_us, (int)s2_map,
          (int)g_t2_us, (int)t2_map,
          fnrWord(g_fnr));
#else
    // (Legacy path disabled in this RAW build)
    // tickEventPipeline(); // prints on-change with labels
    // Desired des; computeDesired(des); applyOutputs(des);
#endif
  }

  // Blink (paused during OTA by console mute only)
  static uint32_t t_led=0; static bool led=false;
  if (!OtaUpdater::inProgress() && millis() - t_led >= BLINK_MS){
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led? HIGH:LOW);
  }

  handleUsbCommandsOnce();

  // Optional: mirror USB input to ESP32 console as lines (commented by default)
  // OtaConsole::mirrorUsbStreamOnce();
}
