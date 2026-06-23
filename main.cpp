#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include <ctype.h>
#include <string.h>
#include <SPIFFS.h>

// ============================================================
// CONFIG
// ============================================================

#if defined(ARDUINO_XIAO_ESP32C5)
  #define MOTOR_PIN        D0   // unused — no haptic motor on this build
  #define USE_MOTOR        0
  #define MOTOR_PWM_CHANNEL   0
  #define MOTOR_PWM_FREQ_HZ   20000
  #define MOTOR_PWM_RES_BITS  8

  #define USE_GPS          0    // no GPS module on this build
  #define GPS_RX_PIN       -1
  #define GPS_TX_PIN       -1
  #define GPS_BAUD         115200

  #define LED_PIN          LED_BUILTIN
  #define USE_LED          1
  #define LED_ACTIVE_HIGH  0    // assumed active-low, matching XIAO S3 convention — confirm on first boot
  #define LED_FLASH_MS     120

  #define MIRROR_SERIAL    0    // disabled: no verified-free GPIO for UART mirror on C5 yet
  #define MIRROR_TX_PIN    -1
  #define MIRROR_BAUD      115200
#else
  // Vibration-motor driver module (coin motor + transistor on a 3-pin
  // VCC/GND/Signal breakout) on the same wire the piezo buzzer used to sit on.
  #define MOTOR_PIN           3
  #define USE_MOTOR           1
  #define MOTOR_PWM_CHANNEL   0
  #define MOTOR_PWM_FREQ_HZ   20000   // above audible range, smooth ERM drive
  #define MOTOR_PWM_RES_BITS  8       // duty 0-255

  // RYS352A GNSS module: TXD -> GPIO44 (board "RX"/D7). We never send
  // commands to the module, so TX is left unused (GPIO43 stays free for
  // the Serial1 debug mirror below).
  #define USE_GPS          1
  #define GPS_RX_PIN       44
  #define GPS_TX_PIN       -1
  #define GPS_BAUD         115200

  // Onboard user LED on Seeed XIAO ESP32-S3 is GPIO21 and is ACTIVE LOW
  // (driving the pin LOW lights the LED).
  #define LED_PIN          21
  #define USE_LED          1
  #define LED_ACTIVE_HIGH  0
  #define LED_FLASH_MS     120

  #define MIRROR_SERIAL    1
  #define MIRROR_TX_PIN    43
  #define MIRROR_BAUD      115200
#endif

#define CHANNEL_MODE_FULL_HOP   0
#define CHANNEL_MODE_CUSTOM     1
#define CHANNEL_MODE_SINGLE     2

#define CHANNEL_MODE CHANNEL_MODE_CUSTOM
#define CHANNEL_DWELL_MS 350
#define SINGLE_CHANNEL 1

#if defined(ARDUINO_XIAO_ESP32C5)
// Mostly 2.4GHz (preserves drive-by revisit cadence), one 5GHz channel
// woven in per group so the full 5GHz set sweeps once every ~11.2s.
static const uint8_t customChannels[] = {
  1, 6, 11, 36,
  1, 6, 11, 40,
  1, 6, 11, 44,
  1, 6, 11, 48,
  1, 6, 11, 149,
  1, 6, 11, 153,
  1, 6, 11, 157,
  1, 6, 11, 161
};
#else
static const uint8_t customChannels[]  = {1, 6, 11};
#endif
static const size_t  customChannelCount = sizeof(customChannels) / sizeof(customChannels[0]);

static const uint8_t fullHopChannels[] = {1,2,3,4,5,6,7,8,9,10,11};
static const size_t  fullHopChannelCount = sizeof(fullHopChannels) / sizeof(fullHopChannels[0]);

#define HEARTBEAT_MS    30000
#define RSSI_MIN        -95
#define ALERT_COOLDOWN_MS 5000

// Haptic cadence: two short vibration pulses on a NEW MAC, then while any
// target is still in range (seen within HB_DEVICE_ACTIVE_MS), two heartbeat
// pulses every HB_BEEP_INTERVAL_MS.
#define HB_DEVICE_ACTIVE_MS    3000
#define HB_BEEP_INTERVAL_MS    10000
// A MAC we haven't heard from in REDISCOVER_MS counts as a fresh discovery
// next time it shows up — fires the double-pulse again. Shorter than a
// Flock's burst-sleep gap would mean false pulses; longer means you'd miss
// a drive-away/return. 30 s is a good middle ground.
#define REDISCOVER_MS          30000
// An ERM motor coasts for tens of ms after power-off, so pulses need real
// on-time to spin up and a real gap to fully stop, or they blur into one
// continuous buzz. These are tuned for "feel," not the old piezo timings.
#define NEW_CHIRP_NOTE_MS      120
#define NEW_CHIRP_GAP_MS       150
#define HB_BEEP_NOTE_MS        150
#define HB_BEEP_GAP_MS         220

// ============================================================
// HUNT MODE  — handheld RSSI proximity hunting ("fox hunt")
// ============================================================
//
// Press the BOOT button to LOCK onto the strongest Flock device seen in the
// last few seconds. While locked we (a) pin to that device's channel and stop
// hopping, and (b) drive the vibration motor like a Geiger counter: pulses
// get faster AND stronger as the smoothed RSSI rises. Sweep the unit (or a
// directional antenna) and walk toward the peak. Press again to release.
//
// Two RF realities this handles:
//   * Channel hopping is suspended so the target is heard continuously, not
//     ~1/3 of the time.
//   * Flock devices burst-then-sleep, so the displayed RSSI peak-holds and
//     decays between bursts — the pulse strength glides down instead of
//     cutting out.
//
// Motor-only build (S3). The C5 has no haptic motor, so hunt is compiled out there.
#if USE_MOTOR
  #define HUNT_MODE_ENABLED 1
#else
  #define HUNT_MODE_ENABLED 0
#endif

#define HUNT_BUTTON_PIN        0       // XIAO BOOT button, active-low
#define HUNT_BTN_DEBOUNCE_MS   40
#define HUNT_LOCK_RECENT_MS    6000    // lock candidate must be seen this recently
#define HUNT_REACQUIRE_MS      9000    // locked target silent longer than this → release

// RSSI → feedback mapping (clamped). FAR = weak/distant, NEAR = strong/close.
#define HUNT_RSSI_FAR          -90
#define HUNT_RSSI_NEAR         -40
#define HUNT_INT_FAR_MS        750     // click spacing when weak
#define HUNT_INT_NEAR_MS       55      // click spacing when strong (near-solid buzz)
#define HUNT_DUTY_FAR          120     // pulse intensity when weak
#define HUNT_DUTY_NEAR         255     // pulse intensity when strong
#define HUNT_CLICK_MS          70      // long enough for the motor to actually spin up
#define HUNT_DECAY_DB_PER_S    18.0f   // how fast the held RSSI sags between bursts
#define HUNT_ATTACK            0.6f    // EMA weight toward a fresh (lower) sample

#define ENABLE_SSID_MATCH 0
#define CHECK_ADDR1 1   // dst/rx — catches Flock STAs receiving probe responses
#define CHECK_ADDR3 0   // bssid fallback for randomised addr2
static const char* target_ssid_keywords[] = { "flock" };
static const size_t SSID_KEYWORD_COUNT = sizeof(target_ssid_keywords) / sizeof(target_ssid_keywords[0]);

#define STOP_ON_SSID_HIT 0
#define STOP_ON_OUI_HIT  0
#define PROCESS_MGMT_FRAMES 1
#define PROCESS_DATA_FRAMES 1

// Persistence
#define MAX_DETECTIONS       200
#define FY_SESSION_FILE      "/session.json"
#define FY_SESSION_TMP       "/session.tmp"
#define FY_PREV_FILE         "/prev_session.json"
#define AUTOSAVE_INTERVAL_MS 60000

// ============================================================
// TARGET OUI LIST  (all lowercase, colons only)
// ============================================================

static const char* target_ouis[] = {
  "70:c9:4e", "3c:91:80", "d8:f3:bc", "80:30:49", "b8:35:32",
  "14:5a:fc", "74:4c:a1", "08:3a:88", "9c:2f:9d", "c0:35:32",
  "94:08:53", "e4:aa:ea", "f4:6a:dd", "f8:a2:d6", "24:b2:b9",
  "00:f4:8d", "d0:39:57", "e8:d0:fc", "e0:4f:43", "b8:1e:a4",
  "70:08:94", "58:8e:81", "ec:1b:bd", "3c:71:bf", "58:00:e3",
  "90:35:ea", "5c:93:a2", "64:6e:69", "48:27:ea", "a4:cf:12",
  // Contributed by Michael / DeFlockJoplin — discovered via wildcard-probe
  // + OUI signature during field testing. The 12th camera in his drive-test
  // used this prefix and wasn't in @NitekryDPaul's original 30.
  "82:6b:f2"
};
static const size_t OUI_COUNT = sizeof(target_ouis) / sizeof(target_ouis[0]);

// Pre-compiled byte table — populated once in setup(), never touched again.
// Keeps matchOuiRaw entirely in IRAM with no flash-resident function calls.
static uint8_t oui_bytes[OUI_COUNT][3];

// ============================================================
// ALERT QUEUE  (callback → loop, avoids Serial in WiFi task)
// ============================================================

#define ALERT_QUEUE_SIZE 32

typedef enum : uint8_t {
  ALERT_OUI_ADDR2       = 0,
  ALERT_OUI_ADDR1       = 1,
  ALERT_OUI_ADDR3       = 2,
  ALERT_SSID            = 3,
  // Probe Request + wildcard SSID (tag 0, length 0) from a known-OUI addr2.
  // Tight signature from Michael / DeFlockJoplin field research:
  //   https://github.com/DeflockJoplin/flock-you
  ALERT_WILDCARD_PROBE  = 4,
} AlertType;

typedef struct {
  AlertType type;
  uint8_t   mac[6];
  int8_t    rssi;
  uint8_t   channel;
  char      ssid[33];     // populated for SSID hits
  char      frameKind[12];
} AlertEntry;

static volatile AlertEntry alertQueue[ALERT_QUEUE_SIZE];
static volatile size_t alertHead = 0;  // written by callback
static volatile size_t alertTail = 0;  // read by loop()
static portMUX_TYPE    queueMux  = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR enqueueAlert(AlertType type, const uint8_t* mac, int8_t rssi,
                                    uint8_t ch, const char* ssid, const char* kind) {
  portENTER_CRITICAL_ISR(&queueMux);
  size_t next = (alertHead + 1) % ALERT_QUEUE_SIZE;
  if (next == alertTail) {                         // drop if full — loop() is behind
    portEXIT_CRITICAL_ISR(&queueMux);
    return;
  }

  AlertEntry* e = (AlertEntry*)&alertQueue[alertHead];
  e->type    = type;
  e->rssi    = rssi;
  e->channel = ch;
  memcpy((void*)e->mac, mac, 6);

  if (ssid)  { strncpy((char*)e->ssid,      ssid, 32); ((char*)e->ssid)[32] = '\0'; }
  else        { ((char*)e->ssid)[0] = '\0'; }

  if (kind)  { strncpy((char*)e->frameKind, kind, 11); ((char*)e->frameKind)[11] = '\0'; }
  else        { ((char*)e->frameKind)[0] = '\0'; }

  alertHead = next;
  portEXIT_CRITICAL_ISR(&queueMux);
}

// ============================================================
// DETECTION TABLE  (on-device storage, persisted to SPIFFS)
// ============================================================
//
// Single-threaded: only touched from loop() — drainAlertQueue() adds, and
// fySaveSession() reads. No mutex needed. The WiFi-task callback never
// touches this table; it only writes to the lock-free alert ring buffer.

typedef struct {
  char     mac[18];
  char     method[16];     // "oui_addr2" / "oui_addr1" / "oui_addr3" / "ssid"
  int8_t   rssi;
  uint8_t  channel;
  uint32_t firstSeen;      // millis() at first hit
  uint32_t lastSeen;       // millis() at latest hit
  uint16_t count;
  char     ssid[33];       // "" unless an SSID hit populated it
} FYDetection;

static FYDetection fyDet[MAX_DETECTIONS];
static int           fyDetCount       = 0;
static bool          fySpiffsReady    = false;
static bool          fyDirty          = false;
static unsigned long fyLastSaveAt     = 0;
static int           fyLastSaveCount  = 0;

// ============================================================
// STATE
// ============================================================

static uint8_t  currentChannel = 1;
static size_t   customChannelIndex = 0;
static size_t   fullHopIndex = 0;
static unsigned long lastHop = 0;
static unsigned long lastHeartbeat = 0;
static volatile bool sniffingStopped = false;

// Dedupe table (small circular, avoids single-slot eviction bug).
// This is the *serial-rate-limit* dedup — it suppresses beep + emit within
// ALERT_COOLDOWN_MS of a prior hit on the same MAC. The detection table
// (above) still counts every hit regardless of this suppression.
#define DEDUPE_SLOTS 8
static struct {
  char mac[18];
  unsigned long ts;
} dedupeTable[DEDUPE_SLOTS];
static size_t dedupeIdx = 0;

// LED one-shot pulse timer
static volatile unsigned long ledOffAt = 0;

// Heartbeat audio state: last time any target was seen, last time the
// heartbeat beep-pair was played. When nothing has been seen for
// HB_DEVICE_ACTIVE_MS the heartbeat stops until the next new detection.
static unsigned long fyLastTargetSeen  = 0;
static unsigned long fyLastHeartbeatAt = 0;

// Hunt mode: locked target + smoothed RSSI used to drive the Geiger audio.
static bool          huntMode        = false;
static uint8_t       huntMacBytes[6] = {0};
static char          huntMacStr[18]  = {0};
static uint8_t       huntChannel     = 0;
static float         huntDisplayRssi = -100.0f;  // peak-held, decaying RSSI
static unsigned long huntLastPktMs   = 0;        // last packet from the locked MAC
static unsigned long huntLastClickAt = 0;
static unsigned long huntLastDecayAt = 0;

// ============================================================
// 802.11 HEADER
// ============================================================

typedef struct __attribute__((packed)) {
  uint16_t frame_ctrl;
  uint16_t duration;
  uint8_t  addr1[6];
  uint8_t  addr2[6];
  uint8_t  addr3[6];
  uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

// ============================================================
// HELPERS
// ============================================================

// Dual-output: prints to both Serial (USB) and Serial1 (GPIO43)
static char _dualBuf[384];

static void dualPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void dualPrintf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(_dualBuf, sizeof(_dualBuf), fmt, args);
  va_end(args);
  if (n > 0) {
    Serial.write(_dualBuf, n);
#if MIRROR_SERIAL
    Serial1.write(_dualBuf, n);
#endif
  }
}

static void dualPrintln(const char* str) {
  Serial.println(str);
#if MIRROR_SERIAL
  Serial1.println(str);
#endif
}

static inline void ledSet(bool on) {
#if USE_LED
#if LED_ACTIVE_HIGH
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#else
  digitalWrite(LED_PIN, on ? LOW  : HIGH);
#endif
#endif
}

static void ledFlash(unsigned ms) {
#if USE_LED
  ledSet(true);
  ledOffAt = millis() + ms;
  if (ledOffAt == 0) ledOffAt = 1;  // avoid the "off" sentinel
#endif
}

static void ledTick() {
#if USE_LED
  if (ledOffAt && (long)(millis() - ledOffAt) >= 0) {
    ledSet(false);
    ledOffAt = 0;
  }
#endif
}

// Non-blocking pulse, same pattern as ledFlash/ledTick above: set a duty,
// remember when to cut it, and let motorTick() turn it off on time. This is
// what lets huntAudioTick() fire pulses without blocking the scan loop, the
// same way tone(pin, hz, ms) used to for the piezo.
static uint32_t motorOffAt = 0;

static inline void motorSet(uint8_t duty) {
#if USE_MOTOR
  ledcWrite(MOTOR_PWM_CHANNEL, duty);
#endif
}

static void motorPulse(unsigned ms, uint8_t duty = 255) {
#if USE_MOTOR
  motorSet(duty);
  motorOffAt = millis() + ms;
  if (motorOffAt == 0) motorOffAt = 1;  // avoid the "off" sentinel
#endif
}

static void motorTick() {
#if USE_MOTOR
  if (motorOffAt && (long)(millis() - motorOffAt) >= 0) {
    motorSet(0);
    motorOffAt = 0;
  }
#endif
}

static void motorBeep(unsigned int ms) {
#if USE_MOTOR
  motorPulse(ms);
  delay(ms);
  motorSet(0);
  motorOffAt = 0;
#endif
}

// Two short pulses — played on the FIRST sighting of a MAC. The old piezo
// used two rising tones here; a motor can't convey pitch, so the "new MAC"
// signature is now a quick double-buzz instead.
static void newDetectChirp() {
#if USE_MOTOR
  motorSet(255); delay(NEW_CHIRP_NOTE_MS); motorSet(0);
  delay(NEW_CHIRP_GAP_MS);
  motorSet(255); delay(NEW_CHIRP_NOTE_MS); motorSet(0);
#endif
}

// Two equal pulses — periodic heartbeat while at least one target is still
// in range (last seen within HB_DEVICE_ACTIVE_MS). Different note/gap timing
// than newDetectChirp keeps the two cues distinguishable by feel.
static void heartbeatBeep() {
#if USE_MOTOR
  motorSet(255); delay(HB_BEEP_NOTE_MS); motorSet(0);
  delay(HB_BEEP_GAP_MS);
  motorSet(255); delay(HB_BEEP_NOTE_MS); motorSet(0);
#endif
}
// "Ready to scan" signal — called at the end of setup() once SPIFFS and the
// WiFi promiscuous sniffer have actually come up, so feeling this means the
// device is live, not just that code reached this line.
static void startupBeep() {
#if USE_MOTOR
  // 3 distinct pulses, well separated so each one is felt as its own buzz
  // instead of blurring into one continuous vibration (see NEW_CHIRP_GAP_MS
  // comment above on ERM motor coast-down).
  for (int i = 0; i < 3; i++) {
    motorSet(255);
    delay(150);
    motorSet(0);
    if (i < 2) delay(200);
  }
#endif
}

// One long pulse — distinct from the success rhythm above — if SPIFFS or the
// WiFi sniffer failed to come up during setup().
static void bootFailBeep() {
#if USE_MOTOR
  motorSet(255);
  delay(800);
  motorSet(0);
#endif
}

// ============================================================
// GPS (RYS352A, NMEA 0183 over UART) — Serial2, RX-only on GPS_RX_PIN.
// Hand-rolled GGA parser — no GPS library, matching the project's
// zero-extra-dependency stance (see README "Build and flash").
// ============================================================

static double        gpsLat       = 0.0;
static double        gpsLon       = 0.0;
static float         gpsHdop      = 99.9f;
static bool          gpsFixValid  = false;
static unsigned long  gpsLastFixAt = 0;

static char   gpsLineBuf[96];
static size_t gpsLineLen = 0;

// sentence points at '$'; validates the trailing "*XX" checksum.
static bool gpsChecksumOk(const char* sentence) {
  const char* star = strchr(sentence, '*');
  if (!star || strlen(star) < 3) return false;
  uint8_t sum = 0;
  for (const char* p = sentence + 1; p < star; p++) sum ^= (uint8_t)*p;
  uint8_t want = (uint8_t)strtol(star + 1, nullptr, 16);
  return sum == want;
}

// NMEA "(d)ddmm.mmmm" + hemisphere char -> signed decimal degrees. Works for
// both 2-digit (lat) and 3-digit (lon) degree prefixes — the minutes portion
// is always < 100, so truncating raw/100 isolates the degrees either way.
static double gpsToDecimalDegrees(const char* field, char hemi) {
  if (!field || !*field) return 0.0;
  double raw = atof(field);
  long   degPart = (long)(raw / 100.0);
  double minPart = raw - (degPart * 100.0);
  double dd = degPart + minPart / 60.0;
  if (hemi == 'S' || hemi == 'W') dd = -dd;
  return dd;
}

// line is NUL-terminated, starts with '$', no trailing CR/LF.
static void gpsHandleSentence(char* line) {
  if (line[0] != '$') return;
  if (!gpsChecksumOk(line)) return;

  const char* star = strchr(line, '*');
  size_t bodyLen = star ? (size_t)(star - line) : strlen(line);
  // Match sentence type by suffix so any talker ID (GP/GN/GL/GA/GB) works.
  if (bodyLen < 6 || strncmp(line + bodyLen - 3, "GGA", 3) != 0) return;

  char body[96];
  size_t copyLen = (bodyLen < sizeof(body) - 1) ? bodyLen : sizeof(body) - 1;
  memcpy(body, line, copyLen);
  body[copyLen] = '\0';

  // $..GGA,time,lat,N/S,lon,E/W,quality,numSats,hdop,alt,...
  char* fields[15] = {0};
  int n = 0;
  char* tok = strtok(body, ",");
  while (tok && n < 15) { fields[n++] = tok; tok = strtok(nullptr, ","); }
  if (n < 9) return;

  int quality = atoi(fields[6]);
  if (quality <= 0) { gpsFixValid = false; return; }

  gpsLat       = gpsToDecimalDegrees(fields[2], fields[3][0]);
  gpsLon       = gpsToDecimalDegrees(fields[4], fields[5][0]);
  gpsHdop      = atof(fields[8]);
  gpsFixValid  = true;
  gpsLastFixAt = millis();
}

static void gpsInit() {
#if USE_GPS
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif
}

static void gpsTick() {
#if USE_GPS
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      gpsLineBuf[gpsLineLen] = '\0';
      if (gpsLineLen > 0) gpsHandleSentence(gpsLineBuf);
      gpsLineLen = 0;
    } else if (c != '\r') {
      if (gpsLineLen < sizeof(gpsLineBuf) - 1) gpsLineBuf[gpsLineLen++] = c;
      else gpsLineLen = 0;  // overflow — drop the malformed line
    }
  }
#endif
}

static void macToStr(const uint8_t* mac, char* buf, size_t len) {
  snprintf(buf, len, "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
static void ouiFromMac(const uint8_t* mac, char* buf, size_t len) {
  snprintf(buf, len, "%02x:%02x:%02x", mac[0], mac[1], mac[2]);
}

static void precompileOuis() {
  for (size_t i = 0; i < OUI_COUNT; i++) {
    const char* o  = target_ouis[i];
    oui_bytes[i][0] = (uint8_t)strtol(o,     nullptr, 16);
    oui_bytes[i][1] = (uint8_t)strtol(o + 3, nullptr, 16);
    oui_bytes[i][2] = (uint8_t)strtol(o + 6, nullptr, 16);
  }
}

// Bit 0 of byte 0 set = multicast/broadcast — never a real device transmitter or receiver
// we care about. Guards addr1 checks against 01:xx, 33:33:xx, ff:ff:ff:ff:ff:ff etc.
static inline bool IRAM_ATTR isMulticast(const uint8_t* mac) {
  return mac[0] & 0x01;
}

static bool IRAM_ATTR matchOuiRaw(const uint8_t* mac) {
  // Locally-administered (randomised) MACs have bit 1 of byte 0 set.
  // Fixed infrastructure devices never use them — skip immediately.
  if (mac[0] & 0x02) return false;

  for (size_t i = 0; i < OUI_COUNT; i++) {
    if (mac[0] == oui_bytes[i][0] &&
        mac[1] == oui_bytes[i][1] &&
        mac[2] == oui_bytes[i][2]) return true;
  }
  return false;
}

static char* strcasestr_local(const char* haystack, const char* needle) {
  if (!*needle) return (char*)haystack;
  for (; *haystack; ++haystack) {
    const char* h = haystack; const char* n = needle;
    while (*h && *n && tolower((unsigned char)*h) == tolower((unsigned char)*n)) { ++h; ++n; }
    if (!*n) return (char*)haystack;
  }
  return nullptr;
}
static bool matchSsidKeyword(const char* ssid) {
  for (size_t i = 0; i < SSID_KEYWORD_COUNT; i++)
    if (strcasestr_local(ssid, target_ssid_keywords[i])) return true;
  return false;
}

static const char* channelModeName() {
  switch (CHANNEL_MODE) {
    case CHANNEL_MODE_FULL_HOP: return "FULL_HOP";
    case CHANNEL_MODE_CUSTOM:   return "CUSTOM";
    case CHANNEL_MODE_SINGLE:   return "SINGLE";
    default:                    return "UNKNOWN";
  }
}

static inline uint16_t channelFreqMhz(uint8_t ch) {
  if (ch >= 1 && ch <= 14) return (uint16_t)(2407 + 5 * ch);
  if (ch >= 36 && ch <= 165) return (uint16_t)(5000 + 5 * ch);
  return 0;
}

static bool shouldSuppressDuplicate(const char* macStr) {
  unsigned long now = millis();
  for (size_t i = 0; i < DEDUPE_SLOTS; i++) {
    if (strcmp(dedupeTable[i].mac, macStr) == 0) {
      if ((now - dedupeTable[i].ts) < ALERT_COOLDOWN_MS) return true;
      dedupeTable[i].ts = now;
      return false;
    }
  }
  // Not found — insert into next slot
  strlcpy(dedupeTable[dedupeIdx].mac, macStr, 18);
  dedupeTable[dedupeIdx].ts = now;
  dedupeIdx = (dedupeIdx + 1) % DEDUPE_SLOTS;
  return false;
}

static void stopSniffing(const char* reason) {
  if (sniffingStopped) return;
  sniffingStopped = true;
  esp_wifi_set_promiscuous(false);
  dualPrintf("[flockyou] sniffing stopped: %s\n", reason);
}

static void applyInitialChannel() {
#if CHANNEL_MODE == CHANNEL_MODE_SINGLE
  currentChannel = SINGLE_CHANNEL;
#elif CHANNEL_MODE == CHANNEL_MODE_CUSTOM
  currentChannel = customChannels[0];
#else
  currentChannel = fullHopChannels[0];
#endif
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  lastHop = millis();  // start dwell timer precisely when channel is first set
}

static void updateChannelMode() {
  if (sniffingStopped) return;
#if HUNT_MODE_ENABLED
  // Locked onto a target: hold its channel, suspend hopping entirely.
  if (huntMode) {
    if (currentChannel != huntChannel) {
      currentChannel = huntChannel;
      esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    }
    return;
  }
#endif
#if CHANNEL_MODE == CHANNEL_MODE_SINGLE
  if (currentChannel != SINGLE_CHANNEL) {
    currentChannel = SINGLE_CHANNEL;
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  }
  return;
#else
  if (millis() - lastHop < CHANNEL_DWELL_MS) return;
  #if CHANNEL_MODE == CHANNEL_MODE_CUSTOM
    customChannelIndex = (customChannelIndex + 1) % customChannelCount;
    currentChannel = customChannels[customChannelIndex];
  #else
    fullHopIndex = (fullHopIndex + 1) % fullHopChannelCount;
    currentChannel = fullHopChannels[fullHopIndex];
  #endif
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  lastHop = millis();
#endif
}

static void printHeartbeat() {
  if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
    dualPrintf("[flockyou] scanning (ch=%u mode=%s det=%d)\n",
                  currentChannel, channelModeName(), fyDetCount);
    lastHeartbeat = millis();
  }
}

// ============================================================
// DETECTION TABLE OPS
// ============================================================

static const char* alertTypeToMethod(AlertType t) {
  switch (t) {
    case ALERT_OUI_ADDR2:      return "oui_addr2";
    case ALERT_OUI_ADDR1:      return "oui_addr1";
    case ALERT_OUI_ADDR3:      return "oui_addr3";
    case ALERT_SSID:           return "ssid";
    case ALERT_WILDCARD_PROBE: return "wildcard_probe";
    default:                   return "unknown";
  }
}

// Returns index of entry (new or updated), or -1 if table is full.
// Returns index, and sets *outChirpWorthy = true when the caller should fire
// the ascending new-discovery chirp. Chirp-worthy means either (a) MAC is
// brand new to this session, or (b) MAC is known but hasn't been seen in
// REDISCOVER_MS — i.e. it left RF range and came back.
static int fyAddDetection(const char* mac, const char* method,
                          int8_t rssi, uint8_t ch, const char* ssid,
                          bool* outChirpWorthy) {
  uint32_t now = millis();
  for (int i = 0; i < fyDetCount; i++) {
    if (strcasecmp(fyDet[i].mac, mac) == 0) {
      bool rediscover = (now - fyDet[i].lastSeen) > REDISCOVER_MS;
      if (fyDet[i].count < 0xFFFF) fyDet[i].count++;
      fyDet[i].lastSeen = now;
      fyDet[i].rssi     = rssi;
      fyDet[i].channel  = ch;
      if (ssid && ssid[0] && !fyDet[i].ssid[0]) {
        strlcpy(fyDet[i].ssid, ssid, sizeof(fyDet[i].ssid));
      }
      fyDirty = true;
      if (outChirpWorthy) *outChirpWorthy = rediscover;
      return i;
    }
  }
  if (fyDetCount >= MAX_DETECTIONS) {
    if (outChirpWorthy) *outChirpWorthy = false;
    return -1;
  }
  FYDetection& d = fyDet[fyDetCount];
  strlcpy(d.mac,    mac,                       sizeof(d.mac));
  strlcpy(d.method, method ? method : "",      sizeof(d.method));
  d.rssi      = rssi;
  d.channel   = ch;
  d.firstSeen = now;
  d.lastSeen  = now;
  d.count     = 1;
  if (ssid && ssid[0]) strlcpy(d.ssid, ssid, sizeof(d.ssid));
  else                 d.ssid[0] = '\0';
  fyDetCount++;
  fyDirty = true;
  if (outChirpWorthy) *outChirpWorthy = true;
  return fyDetCount - 1;
}

// ============================================================
// JSON ESCAPE  — only needed for SSIDs (user-controlled bytes)
// ============================================================

static size_t jsonEscape(char* dst, size_t cap, const char* src) {
  size_t o = 0;
  if (cap == 0) return 0;
  for (size_t i = 0; src[i]; i++) {
    char c = src[i];
    if (c == '"' || c == '\\') {
      if (o + 2 >= cap) break;
      dst[o++] = '\\'; dst[o++] = c;
    } else if ((unsigned char)c < 0x20) {
      if (o + 6 >= cap) break;
      int n = snprintf(dst + o, cap - o, "\\u%04x", (unsigned)(unsigned char)c);
      if (n <= 0 || (size_t)n >= cap - o) break;
      o += (size_t)n;
    } else {
      if (o + 1 >= cap) break;
      dst[o++] = c;
    }
  }
  dst[o] = '\0';
  return o;
}

// ============================================================
// CRC32  (zlib / SPIFFS-tool compatible polynomial 0xEDB88320)
// ============================================================

static uint32_t fyCRC32Update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1));
  }
  return ~crc;
}

// ============================================================
// SPIFFS SESSION PERSISTENCE  — bulletproof envelope format
// ============================================================
//
// Wire format on disk:
//   Line 1: {"v":1,"count":N,"bytes":B,"crc":"0xXXXXXXXX"}\n
//   Line 2+: [{"mac":...},...]     (exactly B bytes, CRC32 == X)
//
// Atomic write procedure:
//   1. Compute payload size + CRC (pass 1)
//   2. Write envelope + payload to /session.tmp (pass 2)
//   3. Re-validate /session.tmp from disk
//   4. Remove /session.json, rename tmp → main (with copy+delete fallback)
//
// Boot-time recovery:
//   - Try /session.json. If missing or CRC-invalid, try /session.tmp.
//   - Copy whichever validates to /prev_session.json, then delete both.

static size_t fySerializeDet(const FYDetection& d, char* dst, size_t cap) {
  char ssidEsc[sizeof(d.ssid) * 6 + 1];
  jsonEscape(ssidEsc, sizeof(ssidEsc), d.ssid);
  int n = snprintf(dst, cap,
      "{\"mac\":\"%s\",\"method\":\"%s\",\"rssi\":%d,\"channel\":%u,"
      "\"first\":%lu,\"last\":%lu,\"count\":%u,\"ssid\":\"%s\"}",
      d.mac, d.method, d.rssi, (unsigned)d.channel,
      (unsigned long)d.firstSeen, (unsigned long)d.lastSeen, (unsigned)d.count,
      ssidEsc);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

static uint32_t fyComputePayloadCRC(size_t& outBytes) {
  char line[384];
  uint32_t crc = 0;
  outBytes = 0;
  crc = fyCRC32Update(crc, (const uint8_t*)"[", 1); outBytes += 1;
  for (int i = 0; i < fyDetCount; i++) {
    if (i > 0) { crc = fyCRC32Update(crc, (const uint8_t*)",", 1); outBytes += 1; }
    size_t n = fySerializeDet(fyDet[i], line, sizeof(line));
    if (n == 0) continue;
    crc = fyCRC32Update(crc, (const uint8_t*)line, n);
    outBytes += n;
  }
  crc = fyCRC32Update(crc, (const uint8_t*)"]", 1); outBytes += 1;
  return crc;
}

// Minimal envelope parser: pulls bytes + crc fields by substring search.
// Robust to field reordering; rejects anything without both required keys.
static bool fyParseEnvelope(const char* hdr, size_t& outBytes, uint32_t& outCrc) {
  const char* b = strstr(hdr, "\"bytes\":");
  const char* c = strstr(hdr, "\"crc\":\"0x");
  if (!b || !c) return false;
  b += 8;
  long long bv = 0;
  if (sscanf(b, "%lld", &bv) != 1 || bv < 0) return false;
  c += 9;
  unsigned cv = 0;
  if (sscanf(c, "%x", &cv) != 1) return false;
  outBytes = (size_t)bv;
  outCrc   = (uint32_t)cv;
  return true;
}

static bool fyValidateSessionFile(const char* path) {
  if (!SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;

  String hdr = f.readStringUntil('\n');
  if (hdr.length() < 10 || hdr[0] != '{') { f.close(); return false; }

  size_t   expectedBytes = 0;
  uint32_t expectedCRC   = 0;
  if (!fyParseEnvelope(hdr.c_str(), expectedBytes, expectedCRC)) {
    f.close(); return false;
  }

  size_t bodyOffset = hdr.length() + 1;
  size_t fileSize   = f.size();
  if (fileSize < bodyOffset + expectedBytes) { f.close(); return false; }
  if ((fileSize - bodyOffset) != expectedBytes) { f.close(); return false; }

  uint8_t buf[256];
  uint32_t crc = 0;
  size_t remaining = expectedBytes;
  while (remaining > 0) {
    int n = f.read(buf, remaining < sizeof(buf) ? remaining : sizeof(buf));
    if (n <= 0) break;
    crc = fyCRC32Update(crc, buf, (size_t)n);
    remaining -= (size_t)n;
  }
  f.close();
  return (remaining == 0 && crc == expectedCRC);
}

static bool fySpiffsCopy(const char* src, const char* dst) {
  File s = SPIFFS.open(src, "r");
  if (!s) return false;
  File d = SPIFFS.open(dst, "w");
  if (!d) { s.close(); return false; }
  uint8_t buf[256];
  int n;
  bool ok = true;
  while ((n = s.read(buf, sizeof(buf))) > 0) {
    if (d.write(buf, (size_t)n) != (size_t)n) { ok = false; break; }
  }
  s.close();
  d.close();
  return ok;
}

static bool fyAtomicPromote(const char* src, const char* dst) {
  if (SPIFFS.rename(src, dst)) return true;
  if (!fySpiffsCopy(src, dst)) return false;
  SPIFFS.remove(src);
  return true;
}

static void fySaveSession() {
  if (!fySpiffsReady) return;
  if (!fyDirty && fyDetCount == fyLastSaveCount) return;

  size_t   payloadBytes = 0;
  uint32_t crc          = fyComputePayloadCRC(payloadBytes);
  int      savedCount   = fyDetCount;

  File f = SPIFFS.open(FY_SESSION_TMP, "w");
  if (!f) {
    dualPrintf("[flockyou] save failed: cannot open %s\n", FY_SESSION_TMP);
    return;
  }
  f.printf("{\"v\":1,\"count\":%d,\"bytes\":%u,\"crc\":\"0x%08lX\"}\n",
           savedCount, (unsigned)payloadBytes, (unsigned long)crc);

  char line[384];
  size_t wrote = 0;
  f.write((uint8_t*)"[", 1); wrote++;
  for (int i = 0; i < fyDetCount; i++) {
    if (i > 0) { f.write((uint8_t*)",", 1); wrote++; }
    size_t n = fySerializeDet(fyDet[i], line, sizeof(line));
    if (n == 0) continue;
    f.write((uint8_t*)line, n);
    wrote += n;
  }
  f.write((uint8_t*)"]", 1); wrote++;
  f.close();

  if (wrote != payloadBytes) {
    dualPrintf("[flockyou] save WARNING: wrote %u expected %u — aborting\n",
               (unsigned)wrote, (unsigned)payloadBytes);
    return;
  }

  if (!fyValidateSessionFile(FY_SESSION_TMP)) {
    dualPrintf("[flockyou] save verify FAILED — old session preserved\n");
    return;
  }

  SPIFFS.remove(FY_SESSION_FILE);
  if (!fyAtomicPromote(FY_SESSION_TMP, FY_SESSION_FILE)) {
    dualPrintf("[flockyou] promote FAILED — data in %s for recovery\n", FY_SESSION_TMP);
    return;
  }

  fyLastSaveAt    = millis();
  fyLastSaveCount = savedCount;
  fyDirty         = false;
  dualPrintf("[flockyou] session saved: %d det, %u bytes, crc=0x%08lX\n",
             savedCount, (unsigned)payloadBytes, (unsigned long)crc);
}

// Promote any valid session file from last boot into /prev_session.json, then
// start this boot with a fresh empty table. Preserves history across power cycles.
static void fyPromotePrevSession() {
  if (!fySpiffsReady) return;

  const char* source = nullptr;
  if      (fyValidateSessionFile(FY_SESSION_FILE)) source = FY_SESSION_FILE;
  else if (fyValidateSessionFile(FY_SESSION_TMP))  source = FY_SESSION_TMP;

  if (!source) {
    if (SPIFFS.exists(FY_SESSION_FILE)) SPIFFS.remove(FY_SESSION_FILE);
    if (SPIFFS.exists(FY_SESSION_TMP))  SPIFFS.remove(FY_SESSION_TMP);
    dualPrintln("[flockyou] no valid prior session to promote");
    return;
  }

  if (!fySpiffsCopy(source, FY_PREV_FILE)) {
    dualPrintf("[flockyou] failed to promote %s → %s\n", source, FY_PREV_FILE);
    return;
  }
  if (SPIFFS.exists(FY_SESSION_FILE)) SPIFFS.remove(FY_SESSION_FILE);
  if (SPIFFS.exists(FY_SESSION_TMP))  SPIFFS.remove(FY_SESSION_TMP);

  File v = SPIFFS.open(FY_PREV_FILE, "r");
  size_t sz = v ? v.size() : 0;
  if (v) v.close();
  dualPrintf("[flockyou] prior session promoted from %s (%u bytes)\n",
             source, (unsigned)sz);
}

// ============================================================
// FLASK-COMPATIBLE JSON EMISSION
// ============================================================
//
// The Flask app (flock-you/api/flockyou.py) reads one JSON object per line
// from the USB CDC serial port. It filters by presence of `detection_method`
// and extracts these fields:  mac_address, rssi, channel, frequency, ssid,
// device_name, gps.latitude, gps.longitude, gps.accuracy.
//
// GPS now comes from the on-device RYS352A (see gpsLat/gpsLon/gpsFixValid
// above) instead of a separate USB NMEA puck — `gps` is `null` until the
// module gets its first fix.

static void gpsJson(char* buf, size_t cap) {
  if (gpsFixValid) {
    snprintf(buf, cap, "{\"latitude\":%.7f,\"longitude\":%.7f,\"accuracy\":%.1f}",
              gpsLat, gpsLon, gpsHdop);
  } else {
    snprintf(buf, cap, "null");
  }
}

static void emitDetectionJSON(const char* mac, const char* method,
                              int8_t rssi, uint8_t ch, const char* ssid) {
  char ssidEsc[sizeof(((FYDetection*)0)->ssid) * 6 + 1];
  jsonEscape(ssidEsc, sizeof(ssidEsc), ssid ? ssid : "");
  char oui[9];
  uint8_t mbytes[6] = {0};
  sscanf(mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mbytes[0], &mbytes[1], &mbytes[2], &mbytes[3], &mbytes[4], &mbytes[5]);
  ouiFromMac(mbytes, oui, sizeof(oui));
  const char* band = (ch >= 1 && ch <= 14) ? "2_4ghz" : "5ghz";
  char gpsBuf[80];
  gpsJson(gpsBuf, sizeof(gpsBuf));

  dualPrintf(
      "{\"event\":\"detection\","
      "\"detection_method\":\"wifi_%s\","
      "\"protocol\":\"wifi_%s\","
      "\"mac_address\":\"%s\","
      "\"oui\":\"%s\","
      "\"device_name\":\"\","
      "\"rssi\":%d,"
      "\"channel\":%u,"
      "\"frequency\":%u,"
      "\"ssid\":\"%s\","
      "\"gps\":%s}\n",
      method, band, mac, oui, rssi,
      (unsigned)ch, (unsigned)channelFreqMhz(ch), ssidEsc, gpsBuf);
}

// ============================================================
// PROMISCUOUS CALLBACK  — keep it fast, no Serial, no malloc
// ============================================================

static bool IRAM_ATTR extractSsidFromMgmtBody(const uint8_t* body, int len,
                                     char* outSsid, size_t outLen) {
  if (!body || len <= 0 || !outSsid || outLen == 0) return false;
  while (len >= 2) {
    uint8_t id = body[0], elen = body[1];
    if ((int)elen + 2 > len) break;
    if (id == 0) {
      size_t n = (elen < (outLen - 1)) ? elen : (outLen - 1);
      memcpy(outSsid, body + 2, n);
      outSsid[n] = '\0';
      return true;
    }
    body += elen + 2; len -= elen + 2;
  }
  return false;
}

// Returns:
//   1  = wildcard SSID IE found (tag 0, length 0)  → Flock-style probe
//   0  = SSID IE found, non-zero length            → directed probe, not ours
//  -1  = no SSID IE found at all                   → caller should retry with
//                                                    FCS-stripped length, then bail
static int IRAM_ATTR isWildcardProbeIE(const uint8_t* body, int len) {
  if (!body || len < 2) return -1;
  while (len >= 2) {
    uint8_t id   = body[0];
    uint8_t elen = body[1];
    if ((int)elen + 2 > len) break;
    if (id == 0) return (elen == 0) ? 1 : 0;
    body += elen + 2;
    len  -= elen + 2;
  }
  return -1;
}

static void IRAM_ATTR wifiSniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (!buf || sniffingStopped) return;

#if PROCESS_MGMT_FRAMES && PROCESS_DATA_FRAMES
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
#elif PROCESS_MGMT_FRAMES
  if (type != WIFI_PKT_MGMT) return;
#elif PROCESS_DATA_FRAMES
  if (type != WIFI_PKT_DATA) return;
#else
  return;  // nothing configured to process
#endif

  wifi_promiscuous_pkt_t*      pkt = (wifi_promiscuous_pkt_t*)buf;
  if (pkt->rx_ctrl.sig_len < sizeof(wifi_ieee80211_mac_hdr_t)) return;
  wifi_ieee80211_mac_hdr_t*    hdr = (wifi_ieee80211_mac_hdr_t*)pkt->payload;
  int8_t rssi = pkt->rx_ctrl.rssi;

  if (rssi < RSSI_MIN) return;

  uint8_t ch = (uint8_t)pkt->rx_ctrl.channel;  // actual rx channel from driver

  // --- OUI check: addr2 (transmitter/source) ---
  //
  // For mgmt Probe Requests (type=0 subtype=4) from a matched OUI, tighten
  // to the DeFlockJoplin wildcard-probe signature: SSID IE (tag 0) length
  // must be zero. This reduces false positives dramatically (Michael's field
  // test: 11/12 true-positive with only 2 false-positives in Joplin).
  //
  // Non-probe frames from the same OUI still emit the broad ADDR2 alert.
  // See: https://github.com/DeflockJoplin/flock-you
  if (matchOuiRaw(hdr->addr2)) {
    bool emitted = false;
    if (type == WIFI_PKT_MGMT) {
      uint8_t fc0     = hdr->frame_ctrl & 0xFF;
      uint8_t ftype   = (fc0 >> 2) & 0x03;
      uint8_t subtype = (fc0 >> 4) & 0x0F;
      if (ftype == 0 && subtype == 4) {                        // Probe Request
        int sigLen  = (int)pkt->rx_ctrl.sig_len;
        int bodyLen = sigLen - (int)sizeof(wifi_ieee80211_mac_hdr_t);
        const uint8_t* body = pkt->payload + sizeof(wifi_ieee80211_mac_hdr_t);
        int r = (bodyLen > 0) ? isWildcardProbeIE(body, bodyLen) : -1;
        // FCS-trailer retry: only when the first parse found no SSID IE AT
        // ALL (-1). A found-but-nonzero (0) means legit directed probe; do
        // not retry — it would mis-classify.
        if (r == -1 && bodyLen > 4) r = isWildcardProbeIE(body, bodyLen - 4);
        if (r == 1) {
          enqueueAlert(ALERT_WILDCARD_PROBE, hdr->addr2, rssi, ch,
                       nullptr, "probe_req");
          emitted = true;
        }
      }
    }
    if (!emitted) {
      enqueueAlert(ALERT_OUI_ADDR2, hdr->addr2, rssi, ch, nullptr, "addr2");
    }
  }

#if CHECK_ADDR1
  // addr1 (receiver/destination): catches Flock STAs that appear only as the
  // dst of probe responses and data frames — never transmitting in the capture
  // window due to their burst-sleep duty cycle. Multicast guard is mandatory
  // here since addr1 is broadcast (ff:ff:ff:ff:ff:ff) in beacons/broadcasts.
  if (!isMulticast(hdr->addr1) && matchOuiRaw(hdr->addr1)) {
    enqueueAlert(ALERT_OUI_ADDR1, hdr->addr1, rssi, ch, nullptr, "addr1");
  }
#endif

#if CHECK_ADDR3
  // addr3 fallback: catches cases where addr2 is randomised but addr3
  // carries the real BSSID OUI (management frames only).
  if (type == WIFI_PKT_MGMT && matchOuiRaw(hdr->addr3)) {
    enqueueAlert(ALERT_OUI_ADDR3, hdr->addr3, rssi, ch, nullptr, "addr3");
  }
#endif

#if ENABLE_SSID_MATCH
  if (type == WIFI_PKT_MGMT) {
    uint8_t fc0     = hdr->frame_ctrl & 0xFF;
    uint8_t subtype = (fc0 >> 4) & 0x0F;
    uint8_t ftype   = (fc0 >> 2) & 0x03;

    if (ftype == 0) {
      int sigLen = pkt->rx_ctrl.sig_len - 4;  // strip 4-byte FCS
      if (sigLen < (int)sizeof(wifi_ieee80211_mac_hdr_t)) return;

      const uint8_t* mgmtBody    = nullptr;
      int            mgmtBodyLen = 0;
      const char*    frameKind   = nullptr;

      if (subtype == 8 || subtype == 5) {
        // Beacon / Probe Response: fixed params = 12 bytes after MAC hdr
        int off = sizeof(wifi_ieee80211_mac_hdr_t) + 12;
        if (sigLen > off) {
          frameKind   = (subtype == 8) ? "beacon" : "probe_resp";
          mgmtBody    = pkt->payload + off;
          mgmtBodyLen = sigLen - off;
        }
      } else if (subtype == 4) {
        // Probe Request: IEs follow directly after MAC hdr
        int off = sizeof(wifi_ieee80211_mac_hdr_t);
        if (sigLen > off) {
          frameKind   = "probe_req";
          mgmtBody    = pkt->payload + off;
          mgmtBodyLen = sigLen - off;
        }
      }

      if (mgmtBody && mgmtBodyLen > 0) {
        char ssid[33] = {0};
        if (extractSsidFromMgmtBody(mgmtBody, mgmtBodyLen, ssid, sizeof(ssid))) {
          if (matchSsidKeyword(ssid)) {
            enqueueAlert(ALERT_SSID, hdr->addr2, rssi, ch, ssid, frameKind);
          }
        }
      }
    }
  }
#endif
}

// ============================================================
// DRAIN QUEUE — called from loop(), safe to Serial.print here
// ============================================================

static void drainAlertQueue() {
  while (true) {
    portENTER_CRITICAL(&queueMux);
    if (alertTail == alertHead) { portEXIT_CRITICAL(&queueMux); break; }
    AlertEntry e;
    memcpy(&e, (const void*)&alertQueue[alertTail], sizeof(AlertEntry));
    alertTail = (alertTail + 1) % ALERT_QUEUE_SIZE;
    portEXIT_CRITICAL(&queueMux);

    char macStr[18];
    macToStr(e.mac, macStr, sizeof(macStr));
    const char* method = alertTypeToMethod(e.type);

    // Always update the on-device detection table (survives reboot via SPIFFS).
    // chirpWorthy = true for brand-new MACs AND for MACs rediscovered after
    // REDISCOVER_MS of silence (drove away and came back).
    bool chirpWorthy = false;
    int idx = fyAddDetection(macStr, method, e.rssi, e.channel,
                             (e.type == ALERT_SSID) ? e.ssid : nullptr,
                             &chirpWorthy);

    // Refresh the global "still around" timer for the heartbeat tick.
    // Done unconditionally so a device counts as active even when serial is
    // rate-limited (still audible via heartbeat, just quieter on the wire).
    fyLastTargetSeen = millis();

#if HUNT_MODE_ENABLED
    // Hunt mode owns proximity feedback: update the held RSSI for the locked
    // MAC on EVERY packet (not rate-limited), peak-biased so it snaps up to a
    // strong burst instantly and only the decay path brings it back down.
    if (huntMode && memcmp(e.mac, huntMacBytes, 6) == 0) {
      if (e.rssi > huntDisplayRssi) huntDisplayRssi = e.rssi;
      else huntDisplayRssi += HUNT_ATTACK * (e.rssi - huntDisplayRssi);
      huntLastPktMs   = millis();
      huntLastDecayAt = huntLastPktMs;
    }
#endif

    // Serial-rate-limit: suppress emit/beep/flash within ALERT_COOLDOWN_MS.
    if (shouldSuppressDuplicate(macStr)) continue;

    // Human-readable line (for serial terminal / mirror).
    char oui[9];
    ouiFromMac(e.mac, oui, sizeof(oui));
    if (e.type == ALERT_SSID) {
      dualPrintf("[flockyou] DETECT-SSID type=%s mac=%s ssid=\"%s\" rssi=%d ch=%u count=%d\n",
                 e.frameKind, macStr, e.ssid, e.rssi, e.channel,
                 (idx >= 0) ? (int)fyDet[idx].count : 0);
    } else {
      dualPrintf("[flockyou] DETECT-OUI mac=%s oui=%s rssi=%d ch=%u addr=%s count=%d\n",
                 macStr, oui, e.rssi, e.channel,
                 e.frameKind[0] ? e.frameKind : "addr2",
                 (idx >= 0) ? (int)fyDet[idx].count : 0);
    }

    // Flask-compatible JSON line (parsed by api/flockyou.py over USB CDC).
    emitDetectionJSON(macStr, method, e.rssi, e.channel,
                      (e.type == ALERT_SSID) ? e.ssid : "");

    // Audio feedback:
    //   - NEW MAC  → two fast ascending beeps (clearly distinct sound)
    //   - REPEAT   → silent; the heartbeat tick covers continued presence
    // LED flashes on every emitted detection either way.
    if (chirpWorthy && !huntMode) {
      newDetectChirp();
      // Reset the heartbeat phase so the first follow-up beep lands
      // HB_BEEP_INTERVAL_MS after the initial chirp, not mid-window.
      fyLastHeartbeatAt = millis();
    }
    ledFlash(LED_FLASH_MS);

#if STOP_ON_OUI_HIT
    if (e.type != ALERT_SSID) stopSniffing("OUI hit");
#endif
#if STOP_ON_SSID_HIT
    if (e.type == ALERT_SSID) stopSniffing("SSID hit");
#endif
  }
}

// ============================================================
// AUTOSAVE
// ============================================================

static void autosaveTick() {
  if (!fySpiffsReady || !fyDirty) return;
  if (millis() - fyLastSaveAt < AUTOSAVE_INTERVAL_MS) return;
  fySaveSession();
}

// Heartbeat beep while at least one target was seen in the last
// HB_DEVICE_ACTIVE_MS. Fires HB_BEEP_INTERVAL_MS apart.
static void heartbeatTick() {
#if HUNT_MODE_ENABLED
  if (huntMode) return;             // hunt audio owns the buzzer while locked
#endif
  if (fyLastTargetSeen == 0) return;                           // never seen one
  unsigned long now = millis();
  if (now - fyLastTargetSeen > HB_DEVICE_ACTIVE_MS) return;    // gone silent
  if (now - fyLastHeartbeatAt < HB_BEEP_INTERVAL_MS) return;   // too soon
  heartbeatBeep();
  fyLastHeartbeatAt = now;
}

// ============================================================
// HUNT MODE OPS
// ============================================================
#if HUNT_MODE_ENABLED

// Linear interpolate an output range across the FAR..NEAR RSSI window.
static long huntMap(float rssi, long outFar, long outNear) {
  float r = rssi;
  if (r < HUNT_RSSI_FAR)  r = HUNT_RSSI_FAR;
  if (r > HUNT_RSSI_NEAR) r = HUNT_RSSI_NEAR;
  float t = (r - HUNT_RSSI_FAR) / (float)(HUNT_RSSI_NEAR - HUNT_RSSI_FAR);  // 0..1
  return outFar + (long)((outNear - outFar) * t + 0.5f);
}

// Pick the strongest Flock device seen within HUNT_LOCK_RECENT_MS.
static bool huntAcquireStrongest() {
  int    best     = -1;
  int8_t bestRssi = -127;
  uint32_t now = millis();
  for (int i = 0; i < fyDetCount; i++) {
    if ((now - fyDet[i].lastSeen) > HUNT_LOCK_RECENT_MS) continue;
    if (fyDet[i].rssi > bestRssi) { bestRssi = fyDet[i].rssi; best = i; }
  }
  if (best < 0) return false;

  strlcpy(huntMacStr, fyDet[best].mac, sizeof(huntMacStr));
  sscanf(huntMacStr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &huntMacBytes[0], &huntMacBytes[1], &huntMacBytes[2],
         &huntMacBytes[3], &huntMacBytes[4], &huntMacBytes[5]);
  huntChannel     = fyDet[best].channel ? fyDet[best].channel : currentChannel;
  huntDisplayRssi = fyDet[best].rssi;
  huntLastPktMs   = now;
  huntLastDecayAt = now;
  huntLastClickAt = 0;
  return true;
}

static void huntEnter() {
  if (!huntAcquireStrongest()) {
    motorPulse(140, 140);            // weak blip = nothing recent to lock onto
    dualPrintln("[flockyou] HUNT: no recent target to lock");
    return;
  }
  huntMode = true;
  esp_wifi_set_channel(huntChannel, WIFI_SECOND_CHAN_NONE);
  currentChannel = huntChannel;
  motorSet(140); delay(80); motorSet(255); delay(130); motorSet(0);  // rising "locked"
  dualPrintf("[flockyou] HUNT: locked %s ch=%u rssi=%.0f\n",
             huntMacStr, huntChannel, huntDisplayRssi);
}

static void huntExit(const char* why) {
  huntMode = false;
  motorSet(255); delay(80); motorSet(140); delay(130); motorSet(0);  // falling "released"
  dualPrintf("[flockyou] HUNT: released (%s) → scan\n", why);
}

// Debounced BOOT-button toggle: press to lock, press again to release.
static void huntButtonTick() {
  static int           lastReading = HIGH;
  static int           stable      = HIGH;
  static unsigned long lastChange  = 0;
  int reading = digitalRead(HUNT_BUTTON_PIN);
  unsigned long now = millis();
  if (reading != lastReading) { lastReading = reading; lastChange = now; }
  if ((now - lastChange) > HUNT_BTN_DEBOUNCE_MS && reading != stable) {
    stable = reading;
    if (stable == LOW) {            // falling edge = a fresh press
      if (huntMode) huntExit("button");
      else          huntEnter();
    }
  }
}

// Geiger-counter audio: faster + higher as the held RSSI rises. Called every
// loop(). Decays the held RSSI between bursts and auto-releases if the target
// goes silent past HUNT_REACQUIRE_MS.
static void huntAudioTick() {
  if (!huntMode) return;
  unsigned long now = millis();

  // Sag the held RSSI toward the floor so the pulse strength glides down
  // between the target's transmit bursts rather than cutting to silence.
  float dt = (now - huntLastDecayAt) / 1000.0f;
  huntLastDecayAt = now;
  if (dt > 0) huntDisplayRssi -= HUNT_DECAY_DB_PER_S * dt;
  if (huntDisplayRssi < HUNT_RSSI_FAR - 5) huntDisplayRssi = HUNT_RSSI_FAR - 5;

  if ((now - huntLastPktMs) > HUNT_REACQUIRE_MS) {  // lost it — back to scanning
    huntExit("target lost");
    return;
  }

  long interval = huntMap(huntDisplayRssi, HUNT_INT_FAR_MS, HUNT_INT_NEAR_MS);
  if ((long)(now - huntLastClickAt) >= interval) {
    long duty = huntMap(huntDisplayRssi, HUNT_DUTY_FAR, HUNT_DUTY_NEAR);
    motorPulse(HUNT_CLICK_MS, (uint8_t)duty);  // non-blocking (motorTick turns it off)
    ledFlash(20);
    huntLastClickAt = now;
  }
}

#endif  // HUNT_MODE_ENABLED

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  // Crucial for USB-optional operation: without this, Serial.write() will
  // block indefinitely on an ESP32-S3 USB-CDC port when no host is attached.
  Serial.setTxTimeoutMs(0);
  delay(300);

#if MIRROR_SERIAL
  Serial1.begin(MIRROR_BAUD, SERIAL_8N1, -1, MIRROR_TX_PIN);  // TX-only on GPIO43
#endif

#if USE_GPS
  gpsInit();
#endif

#if USE_MOTOR
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
  ledcAttachPin(MOTOR_PIN, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, 0);
#endif

#if USE_LED
  pinMode(LED_PIN, OUTPUT);
  ledSet(false);
#endif

#if HUNT_MODE_ENABLED
  pinMode(HUNT_BUTTON_PIN, INPUT_PULLUP);   // BOOT button = lock/release toggle
#endif

  precompileOuis();
  memset(dedupeTable, 0, sizeof(dedupeTable));

  // SPIFFS — format on first boot if missing. Non-fatal if it fails.
  if (SPIFFS.begin(true)) {
    fySpiffsReady = true;
    dualPrintln("[flockyou] SPIFFS ready");
    fyPromotePrevSession();
  } else {
    dualPrintln("[flockyou] SPIFFS init FAILED — running without persistence");
  }

  esp_netif_init();
  esp_err_t evLoopErr = esp_event_loop_create_default();
  if (evLoopErr != ESP_OK && evLoopErr != ESP_ERR_INVALID_STATE) {
    dualPrintln("[flockyou] esp_event_loop_create_default FAILED");
  }

  WiFi.mode(WIFI_MODE_NULL);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_err_t wifiInitErr  = esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_err_t wifiStartErr = esp_wifi_start();

#if defined(ARDUINO_XIAO_ESP32C5)
  esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
#endif

  applyInitialChannel();

  wifi_promiscuous_filter_t filt = {
    .filter_mask = 0
#if PROCESS_MGMT_FRAMES
        | WIFI_PROMIS_FILTER_MASK_MGMT
#endif
#if PROCESS_DATA_FRAMES
        | WIFI_PROMIS_FILTER_MASK_DATA
#endif
  };
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&wifiSniffer);
  esp_err_t promErr = esp_wifi_set_promiscuous(true);

  dualPrintln("[flockyou] merged WiFi detector started");
  dualPrintf("[flockyou] mode=%s dwell_ms=%u start_channel=%u rssi_min=%d spiffs=%d\n",
                channelModeName(), CHANNEL_DWELL_MS, currentChannel,
                RSSI_MIN, fySpiffsReady ? 1 : 0);

  bool wifiOk = (wifiInitErr == ESP_OK) && (wifiStartErr == ESP_OK) && (promErr == ESP_OK);
  if (fySpiffsReady && wifiOk) {
    dualPrintln("[flockyou] BOOT OK — ready to scan");
    startupBeep();
#if USE_LED
    ledFlash(200);
#endif
  } else {
    dualPrintln("[flockyou] BOOT FAILED — spiffs/wifi init did not succeed");
    bootFailBeep();
  }

  lastHeartbeat = millis();
  fyLastSaveAt  = millis();
}

void loop() {
#if HUNT_MODE_ENABLED
  huntButtonTick();    // BOOT button: lock onto / release the strongest target
#endif
  updateChannelMode();
  drainAlertQueue();   // Serial.printf happens here, not in callback
  autosaveTick();      // periodic SPIFFS write if dirty
  heartbeatTick();     // vibration pulse-pair while a target is still in range
#if HUNT_MODE_ENABLED
  huntAudioTick();     // Geiger-counter proximity feedback while locked
#endif
#if USE_GPS
  gpsTick();           // drain Serial2, parse any complete NMEA sentence
#endif
  ledTick();           // turn off LED after LED_FLASH_MS
  motorTick();         // turn off motor after its pulse duration
  printHeartbeat();
  delay(1);
}
