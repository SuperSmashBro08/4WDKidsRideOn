#include <Arduino.h>
#include "OtaUpdater.h"
#include "OtaConsole.h"

// ========== App build tag (shows up in VERSION & logs) ==========
#define APP_FW_VERSION   "app-0.2.5"

// ========== LED blink just to prove OTA changes ==========
#define LED_PIN          13
#define BLINK_MS         50   // change this value between builds to verify OTA

// ========== IO PINS (your exact map) ==========
// F/N/R switch inputs (active LOW, we use pullups)
static constexpr uint8_t PIN_FWD    = 10;   // forward switch input
static constexpr uint8_t PIN_REV    = 11;   // reverse switch input

// Pots
static constexpr uint8_t PIN_POT_S1 = 3;    // Steering 1  (car)
static constexpr uint8_t PIN_POT_T1 = 27;   // Throttle 1  (car)
static constexpr uint8_t PIN_POT_S2 = 5;    // Steering 2  (RC controller)
static constexpr uint8_t PIN_POT_T2 = 4;    // Throttle 2  (RC controller)

// ======== ADC helpers (Teensy 4.1 is 12-bit by default) ========
static constexpr float ADC_VREF = 3.3f;
static constexpr int   ADC_MAX  = 4095;

// ======== Simple channel watching (toggle-able streaming) ========
struct WatchGroup {
  bool s1=false, s2=false, t1=false, t2=false, fnr=false;
} g_watch;

enum FnrState { FNR_NEU, FNR_FWD, FNR_REV };
static FnrState g_fnr = FNR_NEU;

// Last sampled values
static uint16_t g_s1=0, g_s2=0, g_t1=0, g_t2=0;

// ======== Source selection ========
// true  => RC controller has priority (ignores FNR for direction)
// false => Local pedals + FNR determine direction
static bool g_srcRc = true;

// ======== Drive/Steer policy (M1/M2 drive, M3 steer) ========
// - FNR affects ONLY M1/M2 (drive). NEVER affects M3 (steering).
// - If FNR==NEU (and RC not active), command drive=0 and neutral-hold steering.
enum Direction { DIR_NEU, DIR_FWD, DIR_REV };

struct Desired {
  Direction driveDir;       // for M1/M2
  int16_t   driveCmd;       // -1000..+1000, 0=stop (we generate + only here)
  int16_t   steerCmd;       // -1000..+1000
  bool      steerNeutralHold;
};

// ======== Utilities ========
static inline int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static int16_t scaleSteer(uint16_t raw) {
  // map 0..4095 -> -1000..+1000 around mid
  int32_t v = (int32_t)raw - 2048;
  v = (v * 1000) / 2048;
  return clamp_i16(v, -1000, 1000);
}

static int16_t scaleThrottle(uint16_t raw) {
  // 0..4095 -> 0..1000
  int32_t v = (int32_t)raw * 1000 / 4095;
  return clamp_i16(v, 0, 1000);
}

// ======== Sampling ========
static void samplePots() {
  g_s1 = analogRead(PIN_POT_S1);
  g_t1 = analogRead(PIN_POT_T1);
  g_s2 = analogRead(PIN_POT_S2);
  g_t2 = analogRead(PIN_POT_T2);
}

static void sampleFnr() {
  bool fwd = (digitalRead(PIN_FWD) == LOW);
  bool rev = (digitalRead(PIN_REV) == LOW);
  g_fnr = (fwd && !rev) ? FNR_FWD : (!fwd && rev) ? FNR_REV : FNR_NEU;
}

// ======== Policy: compute & apply ========
static void computeDesired(Desired& out) {
  // Choose source
  bool useRc = g_srcRc;

  // Compute commands
  int16_t steer = useRc ? scaleSteer(g_s2) : scaleSteer(g_s1);
  int16_t thr   = useRc ? scaleThrottle(g_t2) : scaleThrottle(g_t1);

  out.steerCmd = steer;
  out.steerNeutralHold = false;
  out.driveCmd = thr;

  if (useRc) {
    // RC has ultimate control; keep forward (you can later add RC reverse)
    out.driveDir = DIR_FWD;
  } else {
    // Local obeys FNR
    if (g_fnr == FNR_NEU) {
      out.driveDir = DIR_NEU;
      out.driveCmd = 0;
      out.steerNeutralHold = true;   // hold steering while stationary
    } else if (g_fnr == FNR_FWD) {
      out.driveDir = DIR_FWD;
    } else {
      out.driveDir = DIR_REV;
    }
  }
}

static void applyOutputs(const Desired& d) {
  // TODO: Write to your real motor drivers (PWMs/DIR pins) here.
  // For now we just log (goes out over Wi-Fi via ESP32 because of OtaConsole).
  const char* dirStr = (d.driveDir==DIR_FWD)?"FWD":(d.driveDir==DIR_REV)?"REV":"NEU";
  OtaConsole::printf("S OUT driveDir=%s driveCmd=%d steerCmd=%d steerNeutral=%d\r\n",
                     dirStr, (int)d.driveCmd, (int)d.steerCmd, (int)d.steerNeutralHold);
}

// ======== Simple command parser over USB Serial ========
// Examples:
//   SRC rc         => RC has priority
//   SRC local      => use local pedals + FNR
//   SRC ?          => show current source
//   WATCH S1 ON    => start streaming Steering1
//   WATCH S1 OFF   => stop  streaming Steering1
//   WATCH T1 ON/OFF, WATCH S2 ON/OFF, WATCH T2 ON/OFF, WATCH FNR ON/OFF
static void handleUsbCommandsOnce() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        // Parse
        String verb, a1, a2;
        int sp1 = line.indexOf(' ');
        if (sp1 < 0) { verb = line; }
        else {
          verb = line.substring(0, sp1);
          String rest = line.substring(sp1+1);
          rest.trim();
          int sp2 = rest.indexOf(' ');
          if (sp2 < 0) a1 = rest;
          else { a1 = rest.substring(0, sp2); a2 = rest.substring(sp2+1); a2.trim(); }
        }

        verb.toUpperCase();
        a1.toUpperCase();
        a2.toUpperCase();

        if (verb == "SRC") {
          if (a1 == "?") {
            Serial.printf("[MyApp] SRC=%s\r\n", g_srcRc ? "rc" : "local");
          } else if (a1 == "RC") {
            g_srcRc = true;  Serial.println("[MyApp] SRC set to rc");
          } else if (a1 == "LOCAL") {
            g_srcRc = false; Serial.println("[MyApp] SRC set to local");
          } else {
            Serial.println("usage: SRC rc|local|?");
          }
        }
        else if (verb == "WATCH") {
          bool on = (a2 == "ON");
          bool off= (a2 == "OFF");
          if (!(on||off)) { Serial.println("usage: WATCH <S1|S2|T1|T2|FNR> ON|OFF"); }
          else {
            if (a1=="S1") g_watch.s1 = on;
            else if (a1=="S2") g_watch.s2 = on;
            else if (a1=="T1") g_watch.t1 = on;
            else if (a1=="T2") g_watch.t2 = on;
            else if (a1=="FNR") g_watch.fnr = on;
            else Serial.println("unknown channel (use S1,S2,T1,T2,FNR)");
          }
        }
        else if (verb == "HELP") {
          Serial.println("Commands:");
          Serial.println("  SRC rc|local|?");
          Serial.println("  WATCH S1|S2|T1|T2|FNR ON|OFF");
        }
        else {
          Serial.println("Unknown. Try HELP");
        }
      }
      line = "";
    } else {
      if (line.length() < 120) line += c;
    }
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

  // OTA forever, logs routed to ESP32 safely
  OtaUpdater::begin(Serial2);
  OtaConsole::begin(Serial2);

  Serial.println("[MyApp] boot");
  OtaConsole::printf("S MyApp FW=%s  (blink=%d ms)\r\n", APP_FW_VERSION, BLINK_MS);
}

void loop() {
  // Always service OTA (cheap when idle)
  OtaUpdater::tick();

  // Mute chatty logs during an active flash (optional)
  OtaConsole::setEnabled(!OtaUpdater::inProgress());

  // Sample at ~10 Hz
  static uint32_t t_sample = 0;
  if (millis() - t_sample >= 100) {
    t_sample = millis();
    samplePots();
    sampleFnr();

    // Stream only what you asked for
    if (g_watch.s1) OtaConsole::printf("S S1 raw=%u v=%.3f steer=%d\r\n", g_s1, (double)g_s1*ADC_VREF/ADC_MAX, (int)scaleSteer(g_s1));
    if (g_watch.s2) OtaConsole::printf("S S2 raw=%u v=%.3f steer=%d\r\n", g_s2, (double)g_s2*ADC_VREF/ADC_MAX, (int)scaleSteer(g_s2));
    if (g_watch.t1) OtaConsole::printf("S T1 raw=%u v=%.3f thr=%d\r\n",  g_t1, (double)g_t1*ADC_VREF/ADC_MAX, (int)scaleThrottle(g_t1));
    if (g_watch.t2) OtaConsole::printf("S T2 raw=%u v=%.3f thr=%d\r\n",  g_t2, (double)g_t2*ADC_VREF/ADC_MAX, (int)scaleThrottle(g_t2));
    if (g_watch.fnr) {
      const char* s = (g_fnr==FNR_FWD)?"FWD":(g_fnr==FNR_REV)?"REV":"NEU";
      OtaConsole::printf("S FNR=%s\r\n", s);
    }

    // Compute & apply desired outputs (now just logs)
    Desired des; computeDesired(des); applyOutputs(des);
  }

  // Blink (don’t blink during flash just to keep UART quiet)
  static uint32_t t_led = 0; static bool led = false;
  if (!OtaUpdater::inProgress() && millis() - t_led >= BLINK_MS) {
    t_led = millis(); led = !led; digitalWrite(LED_PIN, led ? HIGH : LOW);
  }

  // Handle USB Serial commands
  handleUsbCommandsOnce();
}
