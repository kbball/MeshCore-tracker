#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#else
  #error "gps_tracker currently supports nRF52 targets only"
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <target.h>
#include "TrackerGPS.h"
#include "TrackerMotion.h"

// From the base64 library. Not #included: it is a header-only implementation that BaseChatMesh.cpp already
// compiles in, so including it here too would define every function twice.
unsigned int decode_base64(const unsigned char input[], unsigned int input_length, unsigned char output[]);

// Startup/shutdown beeps (boards with a buzzer) and a long-press power-off button (T-1000-E)
#ifdef PIN_BUZZER
  #include <helpers/ui/buzzer.h>
  static genericBuzzer buzzer;
#endif
#if defined(T1000_E) && defined(PIN_USER_BTN)
  #include <helpers/ui/MomentaryButton.h>
  #define TRACKER_POWER_BUTTON 1
  #ifndef TRACKER_POWER_OFF_HOLD_MS
    #define TRACKER_POWER_OFF_HOLD_MS  1500
  #endif
  static MomentaryButton power_btn(PIN_USER_BTN, TRACKER_POWER_OFF_HOLD_MS, false, true, false);   // active high, pulled down
#endif

/* ---------------------------------- CONFIGURATION ------------------------------------- */
// Everything below is only the *default*; the values are stored in flash and changed over the
// serial CLI (see tools/provision_tracker.py).

#define FIRMWARE_VER_TEXT   "gps_tracker v1"

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

// A tracker never keeps contacts; BaseChatMesh just needs a non-zero table.
#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         1
#endif

#ifndef MAX_GROUP_CHANNELS   // without it BaseChatMesh has no channel support and addChannel() just returns NULL
  #error "gps_tracker needs -D MAX_GROUP_CHANNELS=1 (or more)"
#endif

#include <helpers/BaseChatMesh.h>

#ifndef TRACKER_CHANNEL_NAME
  #define TRACKER_CHANNEL_NAME   "tracker-test"
#endif
#ifndef TRACKER_CHANNEL_PSK
  #define TRACKER_CHANNEL_PSK    "dHJhY2tlci10ZXN0LWtleQ=="   // base64 of 16 bytes; test key only
#endif
#ifndef TRACKER_NODE_NAME
  #define TRACKER_NODE_NAME      "Tracker"
#endif

// Reports are flooded to the whole channel, so there is a hard floor on how often we may send.
// Enforced by the CLI; a dedicated race build may lower it with -D TRACKER_MIN_INTERVAL_SEC=...
#ifndef TRACKER_MIN_INTERVAL_SEC
  #define TRACKER_MIN_INTERVAL_SEC   300
#endif
#ifndef TRACKER_INTERVAL_SEC
  #define TRACKER_INTERVAL_SEC       TRACKER_MIN_INTERVAL_SEC
#endif
#if TRACKER_INTERVAL_SEC < TRACKER_MIN_INTERVAL_SEC
  #error "TRACKER_INTERVAL_SEC must be >= TRACKER_MIN_INTERVAL_SEC"
#endif

// delay before the first report after boot
#ifndef TRACKER_FIRST_SEND_DELAY_SEC
  #define TRACKER_FIRST_SEND_DELAY_SEC  10
#endif

// What to do at a reporting tick when there is no GPS fix:
//   0 = stay silent, 1 = send a short "no fix" notice (rate-limited)
#ifndef TRACKER_NOFIX_MODE
  #define TRACKER_NOFIX_MODE   1
#endif
// Minimum spacing between "no fix" notices (never less than the normal report interval)
#ifndef TRACKER_NOFIX_NOTIFY_INTERVAL_SEC
  #define TRACKER_NOFIX_NOTIFY_INTERVAL_SEC   1800
#endif

// Motion-driven reporting (state machine arrives in a later commit; settings are stored already).
// interval_sec is the report interval while moving; idle_interval_sec is the heartbeat while idle.
#ifndef TRACKER_IDLE_INTERVAL_SEC
  #define TRACKER_IDLE_INTERVAL_SEC   1800
#endif
// Stillness required before the tracker declares itself idle (keeps a walk around a tent from flipping state)
#ifndef TRACKER_IDLE_HOLDOFF_SEC
  #define TRACKER_IDLE_HOLDOFF_SEC    300
#endif
// How long to wait for a GPS fix when a report is due
#ifndef TRACKER_FIX_TIMEOUT_SEC
  #define TRACKER_FIX_TIMEOUT_SEC     90
#endif
// GPS displacement (metres, across fixes) that confirms movement
#ifndef TRACKER_MOVE_DIST_M
  #define TRACKER_MOVE_DIST_M         25
#endif
// Accelerometer motion sensitivity (units defined by the driver)
#ifndef TRACKER_MOTION_THRESHOLD
  #define TRACKER_MOTION_THRESHOLD    32
#endif

/* -------------------------------------------------------------------------------------- */

#define PREFS_FILE    "/tracker_prefs"
#define PREFS_MAGIC   0x54524B31   // 'TRK1'
#define PSK_B64_MAX   48

struct TrackerPrefs {  // persisted to file
  uint32_t magic;
  char node_name[32];
  char channel_name[32];
  char channel_psk[PSK_B64_MAX];   // base64
  uint32_t interval_sec;
  uint32_t nofix_notify_sec;
  uint8_t nofix_mode;              // 0 = silent, 1 = notify
  uint8_t sf, cr;
  int8_t tx_power_dbm;
  float freq, bw;
  // appended in v2; a v1 file is padded with defaults when loaded
  uint32_t idle_interval_sec;
  uint32_t idle_holdoff_sec;
  uint32_t fix_timeout_sec;
  uint16_t move_dist_m;
  uint16_t motion_threshold;
};
#define PREFS_V1_SIZE  offsetof(TrackerPrefs, idle_interval_sec)

// metres -> whole feet, rounded to nearest
static long meters_to_feet(int32_t m) {
  int64_t x = (int64_t)m * 32808;   // 1 m = 3.2808 ft
  return (long)((x + (x >= 0 ? 5000 : -5000)) / 10000);
}

// printf("%f") is unreliable on some embedded libcs, so format degrees from fixed point.
static int fmt_degrees(char* dest, size_t sz, int32_t e6) {
  const char* sign = e6 < 0 ? "-" : "";
  uint32_t a = e6 < 0 ? (uint32_t)(-(int64_t)e6) : (uint32_t)e6;
  return snprintf(dest, sz, "%s%lu.%05lu", sign, (unsigned long)(a / 1000000), (unsigned long)((a % 1000000) / 10));
}

static bool valid_bw(float bw) {
  static const float ok[] = { 7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f };
  for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
    if (fabsf(bw - ok[i]) < 0.01f) return true;
  }
  return false;
}

// parse a whole-string unsigned integer; false on junk
static bool parse_uint(const char* s, uint32_t& out) {
  if (*s == 0) return false;
  uint32_t n = 0;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
    n = n * 10 + (*s - '0');
  }
  out = n;
  return true;
}

class TrackerMesh : public BaseChatMesh {
  FILESYSTEM* _fs;
  TrackerPrefs _prefs;
  ChannelDetails* _channel;
  unsigned long _next_send;

  // report state
  bool _reported;               // a position report has gone out since boot
  bool _have_last_fix;          // we have seen a fix at some point since boot
  int32_t _last_lat_e6, _last_lon_e6;
  unsigned long _last_fix_ms;
  bool _nofix_notified;         // a no-fix notice has gone out since the last good report
  unsigned long _last_nofix_ms;
  unsigned long _last_sent_ms;
  bool _have_sent;
  bool _moving;                 // reported as "mv" (true) or "idle" (false) at the end of every message

  const char* stateMarker() const { return _moving ? "mv" : "idle"; }

  // CLI
  char _cmd[160];
  int _cmd_len;

  uint32_t intervalMillis() const { return _prefs.interval_sec * 1000UL; }
  uint32_t nofixNoticeMillis() const {
    uint32_t sec = _prefs.nofix_notify_sec;
    if (sec < _prefs.interval_sec) sec = _prefs.interval_sec;   // never faster than a normal report
    return sec * 1000UL;
  }

  void setDefaults() {
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.magic = PREFS_MAGIC;
    StrHelper::strncpy(_prefs.node_name, TRACKER_NODE_NAME, sizeof(_prefs.node_name));
    StrHelper::strncpy(_prefs.channel_name, TRACKER_CHANNEL_NAME, sizeof(_prefs.channel_name));
    StrHelper::strncpy(_prefs.channel_psk, TRACKER_CHANNEL_PSK, sizeof(_prefs.channel_psk));
    _prefs.interval_sec = TRACKER_INTERVAL_SEC;
    _prefs.nofix_notify_sec = TRACKER_NOFIX_NOTIFY_INTERVAL_SEC;
    _prefs.nofix_mode = TRACKER_NOFIX_MODE ? 1 : 0;
    _prefs.sf = LORA_SF;
    _prefs.cr = LORA_CR;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.freq = LORA_FREQ;
    _prefs.bw = LORA_BW;
    _prefs.idle_interval_sec = TRACKER_IDLE_INTERVAL_SEC;
    _prefs.idle_holdoff_sec = TRACKER_IDLE_HOLDOFF_SEC;
    _prefs.fix_timeout_sec = TRACKER_FIX_TIMEOUT_SEC;
    _prefs.move_dist_m = TRACKER_MOVE_DIST_M;
    _prefs.motion_threshold = TRACKER_MOTION_THRESHOLD;
  }

  void loadPrefs() {
    setDefaults();
    if (_fs->exists(PREFS_FILE)) {
      File file = _fs->open(PREFS_FILE);
      if (file) {
        TrackerPrefs tmp = _prefs;   // defaults first: fields missing from an older file keep them
        int got = file.read((uint8_t *) &tmp, sizeof(tmp));
        file.close();
        if (got >= (int)PREFS_V1_SIZE && tmp.magic == PREFS_MAGIC) {
          tmp.node_name[sizeof(tmp.node_name) - 1] = 0;
          tmp.channel_name[sizeof(tmp.channel_name) - 1] = 0;
          tmp.channel_psk[sizeof(tmp.channel_psk) - 1] = 0;
          if (tmp.interval_sec < TRACKER_MIN_INTERVAL_SEC) tmp.interval_sec = TRACKER_MIN_INTERVAL_SEC;
          if (tmp.idle_interval_sec < TRACKER_MIN_INTERVAL_SEC) tmp.idle_interval_sec = TRACKER_MIN_INTERVAL_SEC;
          _prefs = tmp;
        }
      }
    }
  }

  bool savePrefs() {
    _fs->remove(PREFS_FILE);
    File file = _fs->open(PREFS_FILE, FILE_O_WRITE);
    if (!file) return false;
    bool ok = file.write((const uint8_t *)&_prefs, sizeof(_prefs)) == sizeof(_prefs);
    file.close();
    return ok;
  }

  void sendText(const char* text, int len) {
    if (sendGroupMessage(getRTCClock()->getCurrentTime(), _channel->channel, _prefs.node_name, text, len)) {
      _last_sent_ms = millis();
      _have_sent = true;
      Serial.printf("sent: %s: %s\n", _prefs.node_name, text);
    } else {
      Serial.println("send: unable to queue packet");
    }
  }

  void sendPosition(const TrackerFix& fix) {
    char lat[16], lon[16];
    fmt_degrees(lat, sizeof(lat), fix.lat_e6);
    fmt_degrees(lon, sizeof(lon), fix.lon_e6);

    uint16_t mv = board.getBattMilliVolts();
    char text[96];
    int len = snprintf(text, sizeof(text), "%s,%s alt=%ldft sats=%u bat=%u.%02uV %s", lat, lon,
                       meters_to_feet(fix.alt_m), (unsigned)fix.sats, (unsigned)(mv / 1000), (unsigned)((mv % 1000) / 10),
                       stateMarker());
    sendText(text, len);

    _reported = true;
    _nofix_notified = false;
  }

  void handleNoFix() {
    if (_prefs.nofix_mode == 0) {
      Serial.println("no fix (silent)");
      return;
    }
    unsigned long now = millis();
    if (_nofix_notified && (now - _last_nofix_ms) < nofixNoticeMillis()) {
      Serial.println("no fix (notice rate-limited)");
      return;
    }

    char text[96];
    int len;
    if (_have_last_fix) {
      char lat[16], lon[16];
      fmt_degrees(lat, sizeof(lat), _last_lat_e6);
      fmt_degrees(lon, sizeof(lon), _last_lon_e6);
      uint32_t mins = (now - _last_fix_ms) / 60000UL;
      if (mins < 120) {
        len = snprintf(text, sizeof(text), "no fix (last %s,%s %lum ago) %s", lat, lon, (unsigned long)mins, stateMarker());
      } else {
        len = snprintf(text, sizeof(text), "no fix (last %s,%s %luh ago) %s", lat, lon, (unsigned long)(mins / 60), stateMarker());
      }
    } else {
      len = snprintf(text, sizeof(text), "no fix (no position yet) %s", stateMarker());
    }
    sendText(text, len);

    _nofix_notified = true;
    _last_nofix_ms = now;
  }

  /* ------------------------------------- CLI ----------------------------------------- */

  static bool psk_ok(const char* b64) {
    if (strlen(b64) == 0 || strlen(b64) >= PSK_B64_MAX) return false;
    uint8_t tmp[40];
    int len = decode_base64((unsigned char *) b64, strlen(b64), tmp);
    return len == 16 || len == 32;
  }

  void printConfig() {
    Serial.printf("name=%s\n", _prefs.node_name);
    Serial.printf("channel=%s\n", _prefs.channel_name);
    Serial.printf("psk=%s\n", _prefs.channel_psk);
    Serial.printf("interval=%lu\n", (unsigned long)_prefs.interval_sec);
    Serial.printf("min_interval=%d\n", TRACKER_MIN_INTERVAL_SEC);
    Serial.printf("nofix=%s\n", _prefs.nofix_mode ? "notify" : "silent");
    Serial.printf("nofix_interval=%lu\n", (unsigned long)_prefs.nofix_notify_sec);
    Serial.printf("freq=%s\n", StrHelper::ftoa3(_prefs.freq));
    Serial.printf("bw=%s\n", StrHelper::ftoa3(_prefs.bw));
    Serial.printf("sf=%u\n", (unsigned)_prefs.sf);
    Serial.printf("cr=%u\n", (unsigned)_prefs.cr);
    Serial.printf("tx=%d\n", (int)_prefs.tx_power_dbm);
    Serial.printf("idle_interval=%lu\n", (unsigned long)_prefs.idle_interval_sec);
    Serial.printf("idle_holdoff=%lu\n", (unsigned long)_prefs.idle_holdoff_sec);
    Serial.printf("fix_timeout=%lu\n", (unsigned long)_prefs.fix_timeout_sec);
    Serial.printf("move_dist=%u\n", (unsigned)_prefs.move_dist_m);
    Serial.printf("motion=%u\n", (unsigned)_prefs.motion_threshold);
  }

  void printStatus() {
    printConfig();
    Serial.printf("state=%s\n", stateMarker());
    Serial.printf("gps=%s\n", tracker_gps_powered() ? "on" : "off");
    Serial.printf("accel=%s irqs=%lu\n", tracker_motion_available() ? "yes" : "no", (unsigned long)tracker_motion_irq_count());
    TrackerFix fix;
    if (tracker_gps_get_fix(fix) && fix.valid) {
      char lat[16], lon[16];
      fmt_degrees(lat, sizeof(lat), fix.lat_e6);
      fmt_degrees(lon, sizeof(lon), fix.lon_e6);
      Serial.printf("fix=yes %s,%s alt=%ldft sats=%u\n", lat, lon, meters_to_feet(fix.alt_m), (unsigned)fix.sats);
    } else {
      Serial.println("fix=no");
    }
    if (_have_sent) {
      Serial.printf("last_sent=%lus ago\n", (unsigned long)((millis() - _last_sent_ms) / 1000));
    } else {
      Serial.println("last_sent=never");
    }
    Serial.printf("next_report=%lds\n", (long)(_next_send - millis()) / 1000);
    Serial.printf("battery_mv=%u\n", (unsigned)board.getBattMilliVolts());
  }

  // returns an error string, or NULL on success
  const char* handleSet(char* args) {
    char* val = strchr(args, ' ');
    if (val) { *val++ = 0; while (*val == ' ') val++; } else { val = args + strlen(args); }
    uint32_t n;

    if (strcmp(args, "name") == 0) {
      if (*val == 0 || strlen(val) >= sizeof(_prefs.node_name)) return "name must be 1-31 chars";
      StrHelper::strncpy(_prefs.node_name, val, sizeof(_prefs.node_name));
    } else if (strcmp(args, "channel") == 0) {   // set channel <psk_base64> <name, may contain spaces>
      char* name = strchr(val, ' ');
      if (!name) return "usage: set channel <psk_base64> <name>";
      *name++ = 0;
      while (*name == ' ') name++;
      if (*name == 0 || strlen(name) >= sizeof(_prefs.channel_name)) return "channel name must be 1-31 chars";
      if (!psk_ok(val)) return "psk must be base64 of 16 or 32 bytes";
      StrHelper::strncpy(_prefs.channel_name, name, sizeof(_prefs.channel_name));
      StrHelper::strncpy(_prefs.channel_psk, val, sizeof(_prefs.channel_psk));
    } else if (strcmp(args, "interval") == 0) {
      if (!parse_uint(val, n) || n > 86400) return "interval must be whole seconds";
      if (n < TRACKER_MIN_INTERVAL_SEC) return "interval below minimum";
      _prefs.interval_sec = n;
    } else if (strcmp(args, "nofix") == 0) {
      if (strcmp(val, "silent") == 0) _prefs.nofix_mode = 0;
      else if (strcmp(val, "notify") == 0) _prefs.nofix_mode = 1;
      else return "nofix must be silent or notify";
    } else if (strcmp(args, "nofix_interval") == 0) {
      if (!parse_uint(val, n) || n > 604800) return "nofix_interval must be whole seconds";
      if (n < TRACKER_MIN_INTERVAL_SEC) return "nofix_interval below minimum";
      _prefs.nofix_notify_sec = n;
    } else if (strcmp(args, "idle_interval") == 0) {
      if (!parse_uint(val, n) || n > 86400) return "idle_interval must be whole seconds";
      if (n < TRACKER_MIN_INTERVAL_SEC) return "idle_interval below minimum";
      _prefs.idle_interval_sec = n;
    } else if (strcmp(args, "idle_holdoff") == 0) {
      if (!parse_uint(val, n) || n < 60 || n > 3600) return "idle_holdoff must be 60-3600 seconds";
      _prefs.idle_holdoff_sec = n;
    } else if (strcmp(args, "fix_timeout") == 0) {
      if (!parse_uint(val, n) || n < 10 || n > 300) return "fix_timeout must be 10-300 seconds";
      _prefs.fix_timeout_sec = n;
    } else if (strcmp(args, "move_dist") == 0) {
      if (!parse_uint(val, n) || n < 5 || n > 500) return "move_dist must be 5-500 metres";
      _prefs.move_dist_m = n;
    } else if (strcmp(args, "motion") == 0) {
      if (!parse_uint(val, n) || n < 1 || n > 255) return "motion must be 1-255";
      _prefs.motion_threshold = n;
      tracker_motion_set_threshold(n);   // applies immediately
    } else if (strcmp(args, "freq") == 0) {
      float f = atof(val);
      if (f < 150.0f || f > 960.0f) return "freq must be 150-960 MHz";
      _prefs.freq = f;
    } else if (strcmp(args, "bw") == 0) {
      float b = atof(val);
      if (!valid_bw(b)) return "bw must be one of 7.8 10.4 15.6 20.8 31.25 41.7 62.5 125 250 500";
      _prefs.bw = b;
    } else if (strcmp(args, "sf") == 0) {
      if (!parse_uint(val, n) || n < 5 || n > 12) return "sf must be 5-12";
      _prefs.sf = n;
    } else if (strcmp(args, "cr") == 0) {
      if (!parse_uint(val, n) || n < 5 || n > 8) return "cr must be 5-8";
      _prefs.cr = n;
    } else if (strcmp(args, "tx") == 0) {
      int t = atoi(val);
      if ((t == 0 && val[0] != '0') || t < -9 || t > 22) return "tx must be -9..22 dBm";
      _prefs.tx_power_dbm = t;
    } else {
      return "unknown setting";
    }
    return savePrefs() ? NULL : "unable to save";
  }

  // Every command ends with a line starting "OK" or "ERR", so a script can tell when it is done
  // (the tracker may print "sent:" log lines in between).
  void handleCommand(char* command) {
    while (*command == ' ') command++;

    if (strcmp(command, "get") == 0 || strcmp(command, "status") == 0) {
      printStatus();
      Serial.println("OK");
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* err = handleSet(&command[4]);
      if (err) Serial.printf("ERR: %s\n", err);
      else Serial.println("OK (channel, radio and name changes apply after 'reboot')");
    } else if (strcmp(command, "reboot") == 0) {
      Serial.println("OK rebooting");
      Serial.flush();
      delay(100);
      board.reboot();
    } else if (memcmp(command, "gps ", 4) == 0) {   // diagnostics: power the receiver on/off
      if (strcmp(&command[4], "on") == 0) tracker_gps_power(true);
      else if (strcmp(&command[4], "off") == 0) tracker_gps_power(false);
      else { Serial.println("ERR: gps on|off"); return; }
      Serial.printf("gps=%s\n", tracker_gps_powered() ? "on" : "off");
      Serial.println("OK");
    } else if (strcmp(command, "accel") == 0) {   // diagnostics: raw accelerometer + interrupt counter
      tracker_motion_dump(Serial);
      Serial.println("OK");
    } else if (strcmp(command, "ver") == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
      Serial.println("OK");
    } else if (strcmp(command, "help") == 0) {
      Serial.println("Commands:");
      Serial.println("   get | status");
      Serial.println("   set name <name>");
      Serial.println("   set channel <psk_base64> <name>");
      Serial.printf("   set interval <sec>          (min %d)\n", TRACKER_MIN_INTERVAL_SEC);
      Serial.println("   set nofix silent|notify");
      Serial.println("   set nofix_interval <sec>");
      Serial.printf("   set idle_interval <sec>     (heartbeat while idle, min %d)\n", TRACKER_MIN_INTERVAL_SEC);
      Serial.println("   set idle_holdoff <sec>      (stillness before going idle, 60-3600)");
      Serial.println("   set fix_timeout <sec>       (10-300)");
      Serial.println("   set move_dist <metres>      (GPS displacement that confirms movement)");
      Serial.println("   set motion <1-255>          (accelerometer sensitivity)");
      Serial.println("   set freq|bw|sf|cr|tx <value>");
      Serial.println("   accel                       (accelerometer diagnostics)");
      Serial.println("   gps on|off                  (power the GPS; the tracker will switch it back as it needs)");
      Serial.println("   reboot");
      Serial.println("OK");
    } else if (*command) {
      Serial.printf("ERR: unknown command: %s\n", command);
    }
  }

  void pollSerial() {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (_cmd_len > 0) {
          _cmd[_cmd_len] = 0;
          _cmd_len = 0;
          handleCommand(_cmd);
        }
      } else if (_cmd_len < (int)sizeof(_cmd) - 1) {
        _cmd[_cmd_len++] = c;
      } else {   // overlong line: discard
        _cmd_len = 0;
        Serial.println("ERR: line too long");
      }
    }
  }

protected:
  float getAirtimeBudgetFactor() const override { return 1.0f; }
  int calcRxDelay(float score, uint32_t air_time) const override { return 0; }

  // A tracker is a pure sender: it never repeats other nodes' packets.
  bool allowPacketForward(const mesh::Packet* packet) override { return false; }

  // We never advertise, and we ignore everyone else's adverts (no contacts are kept).
  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override { }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override { }
  void onContactPathUpdated(const ContactInfo& contact) override { }
  ContactInfo* processAck(const uint8_t *data) override { return NULL; }
  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override { }
  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override { }
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override { }
  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override { }
  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override { return 0; }
  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override { }
  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override { return 500 + 16.0f * pkt_airtime_millis; }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override { return 500 + 6.0f * pkt_airtime_millis * ((path_len & 63) + 1); }
  void onSendTimeout() override { }

public:
  TrackerMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    _channel = NULL;
    _next_send = 0;
    _reported = _have_last_fix = _nofix_notified = _have_sent = false;
    _moving = true;   // until the motion state machine exists (and on boards with no accelerometer)
    _last_lat_e6 = _last_lon_e6 = 0;
    _last_fix_ms = _last_nofix_ms = _last_sent_ms = 0;
    _cmd_len = 0;
    setDefaults();
  }

  float getFreqPref() const { return _prefs.freq; }
  float getBwPref() const { return _prefs.bw; }
  uint8_t getSfPref() const { return _prefs.sf; }
  uint8_t getCrPref() const { return _prefs.cr; }
  int8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }
  uint8_t getMotionPref() const { return _prefs.motion_threshold; }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;
    BaseChatMesh::begin();

    // identity is only needed to satisfy the mesh layer; generate silently on first boot
    IdentityStore store(fs, "");
    char unused_name[32];
    if (!store.load("_main", self_id, unused_name, sizeof(unused_name))) {
      self_id = radio_new_identity();
      store.save("_main", self_id);
    }

    loadPrefs();

    _channel = addChannel(_prefs.channel_name, _prefs.channel_psk);
    if (!_channel) {   // corrupt PSK in flash: fall back to the compiled-in default rather than not reporting
      _channel = addChannel(TRACKER_CHANNEL_NAME, TRACKER_CHANNEL_PSK);
    }
    if (!_channel) Serial.println("ERROR: unable to set up the report channel; nothing will be sent");
    _next_send = millis() + (uint32_t)TRACKER_FIRST_SEND_DELAY_SEC * 1000UL;
  }

  void showWelcome() {
    Serial.println("===== MeshCore GPS Tracker =====");
    Serial.println(FIRMWARE_VER_TEXT);
    Serial.println("   (enter 'help' for commands)");
    printConfig();
  }

  void loop() {
    BaseChatMesh::loop();
    pollSerial();

    tracker_gps_loop();
    if (!_channel) return;

    TrackerFix fix;
    bool have_fix = tracker_gps_get_fix(fix) && fix.valid;
    if (have_fix) {
      _have_last_fix = true;
      _last_lat_e6 = fix.lat_e6;
      _last_lon_e6 = fix.lon_e6;
      _last_fix_ms = millis();
    }

    // Report on the interval; and after boot, report as soon as the first fix arrives
    // rather than waiting out the rest of the interval.
    bool due = (long)(millis() - _next_send) >= 0;
    if (due || (have_fix && !_reported)) {
      _next_send = millis() + intervalMillis();
      if (have_fix) {
        sendPosition(fix);
      } else {
        handleNoFix();
      }
    }
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
TrackerMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

#ifdef TRACKER_POWER_BUTTON
// Long press: play the shutdown tune (if any) and power the board off.
// powerOff() waits for the button to be released, then sleeps until the button is pressed again.
static void shutdown_tracker() {
  Serial.println("power off");
#ifdef PIN_BUZZER
  buzzer.shutdown();
  uint32_t started = millis();
  while (buzzer.isPlaying() && (uint32_t)(millis() - started) < 2500) {   // fail-safe cap
    buzzer.loop();
  }
#endif
  board.powerOff();
}
#endif

void setup() {
  Serial.begin(115200);

  board.begin();

  if (!radio_init()) { halt(); }

  tracker_gps_begin();

  fast_rng.begin(radio_driver.getRngSeed());

  InternalFS.begin();
  the_mesh.begin(InternalFS);

  if (!tracker_motion_begin(the_mesh.getMotionPref())) {
    Serial.println("accelerometer: not available (no motion detection)");
  }

  radio_driver.setParams(the_mesh.getFreqPref(), the_mesh.getBwPref(), the_mesh.getSfPref(), the_mesh.getCrPref());
  radio_driver.setTxPower(the_mesh.getTxPowerPref());

  the_mesh.showWelcome();
  // NOTE: deliberately no initial advert - trackers never advertise.

#ifdef TRACKER_POWER_BUTTON
  power_btn.begin();
#endif
#ifdef PIN_BUZZER
  buzzer.begin();
  buzzer.startup();   // plays out from loop(); we're up and running at this point
#endif
}

void loop() {
  the_mesh.loop();
  rtc_clock.tick();

#ifdef PIN_BUZZER
  if (buzzer.isPlaying()) buzzer.loop();
#endif
#ifdef TRACKER_POWER_BUTTON
  if (power_btn.check() == BUTTON_EVENT_LONG_PRESS) shutdown_tracker();
#endif
}
