#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#else
  #error "runner_alert currently supports nRF52 targets only"
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <target.h>
#include "AlertPolicy.h"
#include "CameraLine.h"

// From the base64 library. Not #included: it is a header-only implementation that BaseChatMesh.cpp already
// compiles in, so including it here too would define every function twice.
unsigned int decode_base64(const unsigned char input[], unsigned int input_length, unsigned char output[]);

/*
 * Runner alert node for the early-warning system. It sits next to the early-warning camera, reads the camera's
 * UART, and posts a short text on one MeshCore channel when the camera reports a runner. Built like the GPS
 * tracker: send-only (never advertises, never forwards, ignores everything it hears), the radio sleeps between
 * sends, and everything is provisioned over the USB serial CLI. See docs/runner_alert.md.
 */

/* ---------------------------------- CONFIGURATION ------------------------------------- */
// Everything below is only the *default*; the values are stored in flash and changed over the serial CLI.

#define FIRMWARE_VER_TEXT   "runner_alert v1"

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

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         1
#endif

#ifndef MAX_GROUP_CHANNELS   // without it BaseChatMesh has no channel support and addChannel() just returns NULL
  #error "runner_alert needs -D MAX_GROUP_CHANNELS=1 (or more)"
#endif

#include <helpers/BaseChatMesh.h>

#ifndef RUNNER_CHANNEL_NAME
  #define RUNNER_CHANNEL_NAME   "runner-test"
#endif
#ifndef RUNNER_CHANNEL_PSK
  #define RUNNER_CHANNEL_PSK    "cnVubmVyLXRlc3Qta2V5MQ=="   // base64 of 16 bytes; test key only
#endif
#ifndef RUNNER_NODE_NAME
  #define RUNNER_NODE_NAME      "Runner"
#endif

// Alerts are flooded to the whole channel, so there is a floor on how often one may be sent.
// A real alert needs no more than one per runner group; repeated PERSON verdicts inside the gap are counted only.
#ifndef RUNNER_MIN_ALERT_INTERVAL_SEC
  #define RUNNER_MIN_ALERT_INTERVAL_SEC   10
#endif
#ifndef RUNNER_ALERT_INTERVAL_SEC
  #define RUNNER_ALERT_INTERVAL_SEC       20
#endif
// Status heartbeat so the fleet can see the node is alive. Flooded too, so the floor matches the tracker's.
#ifndef RUNNER_MIN_STATUS_INTERVAL_SEC
  #define RUNNER_MIN_STATUS_INTERVAL_SEC  300
#endif
#ifndef RUNNER_STATUS_INTERVAL_SEC
  #define RUNNER_STATUS_INTERVAL_SEC      3600
#endif
#ifndef RUNNER_FIRST_STATUS_SEC
  #define RUNNER_FIRST_STATUS_SEC         15
#endif
// The camera's UART (Serial1: D7 = RX; D6 = TX, unused)
#ifndef RUNNER_UART_BAUD
  #define RUNNER_UART_BAUD                115200
#endif

/* -------------------------------------------------------------------------------------- */

#define PREFS_FILE    "/runner_prefs"
#define PREFS_MAGIC   0x52414C31   // 'RAL1'
#define PSK_B64_MAX   48

struct RunnerPrefs {  // persisted to file
  uint32_t magic;
  char node_name[32];
  char channel_name[32];
  char channel_psk[PSK_B64_MAX];   // base64
  uint32_t alert_interval_sec;
  uint32_t status_interval_sec;
  uint8_t sf, cr;
  int8_t tx_power_dbm;
  float freq, bw;
  uint32_t saved_time;             // last known epoch time, so a reboot does not drop the clock back to 2024
};

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

extern RADIO_CLASS radio;   // the RadioLib object behind radio_driver (defined in target.cpp)

class RunnerMesh : public BaseChatMesh {
  FILESYSTEM* _fs;
  RunnerPrefs _prefs;
  ChannelDetails* _channel;
  runner::AlertPolicy _policy;

  unsigned long _last_sent_ms;
  bool _have_sent;
  bool _radio_awake;            // false while the radio is in sleep mode between sends
  int _tx_outstanding;          // packets queued whose transmission has not finished
  unsigned long _tx_deadline;

  // camera UART
  char _cam[96];
  int _cam_len;
  bool _cam_overflow;
  uint32_t _cam_ok, _cam_noise;

  // CLI
  char _cmd[160];
  int _cmd_len;

  runner::AlertPolicy::Config policyConfig() const {
    runner::AlertPolicy::Config c;
    c.min_alert_ms = _prefs.alert_interval_sec * 1000UL;
    c.status_ms = _prefs.status_interval_sec * 1000UL;
    c.first_status_ms = RUNNER_FIRST_STATUS_SEC * 1000UL;
    return c;
  }

  void setDefaults() {
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.magic = PREFS_MAGIC;
    StrHelper::strncpy(_prefs.node_name, RUNNER_NODE_NAME, sizeof(_prefs.node_name));
    StrHelper::strncpy(_prefs.channel_name, RUNNER_CHANNEL_NAME, sizeof(_prefs.channel_name));
    StrHelper::strncpy(_prefs.channel_psk, RUNNER_CHANNEL_PSK, sizeof(_prefs.channel_psk));
    _prefs.alert_interval_sec = RUNNER_ALERT_INTERVAL_SEC;
    _prefs.status_interval_sec = RUNNER_STATUS_INTERVAL_SEC;
    _prefs.sf = LORA_SF;
    _prefs.cr = LORA_CR;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.freq = LORA_FREQ;
    _prefs.bw = LORA_BW;
    _prefs.saved_time = 0;
  }

  void loadPrefs() {
    setDefaults();
    if (_fs->exists(PREFS_FILE)) {
      File file = _fs->open(PREFS_FILE);
      if (file) {
        RunnerPrefs tmp = _prefs;   // defaults first
        int got = file.read((uint8_t *) &tmp, sizeof(tmp));
        file.close();
        if (got == (int)sizeof(tmp) && tmp.magic == PREFS_MAGIC) {
          tmp.node_name[sizeof(tmp.node_name) - 1] = 0;
          tmp.channel_name[sizeof(tmp.channel_name) - 1] = 0;
          tmp.channel_psk[sizeof(tmp.channel_psk) - 1] = 0;
          if (tmp.alert_interval_sec < RUNNER_MIN_ALERT_INTERVAL_SEC) tmp.alert_interval_sec = RUNNER_MIN_ALERT_INTERVAL_SEC;
          if (tmp.status_interval_sec < RUNNER_MIN_STATUS_INTERVAL_SEC) tmp.status_interval_sec = RUNNER_MIN_STATUS_INTERVAL_SEC;
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

  // The node only transmits, so between sends the radio sleeps instead of listening.
  // A send is outstanding from queueing until the dispatcher reports it done (logTx / logTxFail);
  // the deadline stops a lost packet from keeping the radio awake forever.
  bool txPending() const {
    return _tx_outstanding > 0 && (long)(millis() - _tx_deadline) < 0;
  }

  void wakeRadio() {
    if (_radio_awake) return;
    radio.standby();            // also wakes the chip (configuration is retained across sleep)
    _radio_awake = true;
  }

  void sleepRadioIfIdle() {
    if (_radio_awake && !txPending()) {
      radio.standby();
      radio.sleep();            // warm sleep: lowest power, configuration retained
      _radio_awake = false;
    }
  }

  void sendText(const char* text, int len) {
    wakeRadio();
    if (sendGroupMessage(getRTCClock()->getCurrentTime(), _channel->channel, _prefs.node_name, text, len)) {
      _tx_outstanding++;
      _tx_deadline = millis() + 30000;   // generous: includes any airtime-budget delay
      _last_sent_ms = millis();
      _have_sent = true;
      Serial.printf("sent: %s: %s\n", _prefs.node_name, text);
    } else {
      Serial.println("send: unable to queue packet");
    }
  }

  // Message formats (parsed by the fleet tooling, keep them stable):
  //   alert:  "runner score=<0-100> bat=<V>"
  //   status: "alive bat=<V> alerts=<n since boot>"
  //   test:   "test bat=<V>"   (CLI only, so provisioning can check radio and channel without raising an alert)
  void battText(char* dest, size_t sz) {
    uint16_t mv = board.getBattMilliVolts();
    snprintf(dest, sz, "bat=%u.%02uV", (unsigned)(mv / 1000), (unsigned)((mv % 1000) / 10));
  }

  void sendAlert(uint8_t score) {
    char bat[16], text[64];
    battText(bat, sizeof(bat));
    int len = snprintf(text, sizeof(text), "runner score=%u %s", (unsigned)score, bat);
    sendText(text, len);
  }

  void sendStatus() {
    char bat[16], text[64];
    battText(bat, sizeof(bat));
    int len = snprintf(text, sizeof(text), "alive %s alerts=%lu", bat, (unsigned long)_policy.alerts());
    sendText(text, len);
    saveTime();   // hourly at most: keeps the clock roughly right across a reboot without wearing the flash
  }

  void sendTest() {
    char bat[16], text[64];
    battText(bat, sizeof(bat));
    int len = snprintf(text, sizeof(text), "test %s", bat);
    sendText(text, len);
  }

  void saveTime() {
    uint32_t now = getRTCClock()->getCurrentTime();
    if (now > _prefs.saved_time) {
      _prefs.saved_time = now;
      savePrefs();
    }
  }

  /* ------------------------------------ camera UART ---------------------------------- */

  void handleCameraLine(const char* line) {
    runner::CameraMsg m;
    if (!runner::parseCameraLine(line, m)) {
      _cam_noise++;
      return;
    }
    _cam_ok++;
    Serial.printf("cam: %s\n", line);
    if (m.type != runner::CAM_VERDICT || !_channel) return;

    runner::AlertPolicy::Result r = _policy.onVerdict(m.person, m.score, millis());
    if (r.action == runner::AlertPolicy::SEND_ALERT) sendAlert(r.score);
    else if (m.person) Serial.println("alert suppressed (rate limit)");
  }

  void pollCamera() {
    while (Serial1.available()) {
      int c = Serial1.read();
      if (c < 0 || c == '\r') continue;
      if (c == '\n') {
        if (!_cam_overflow && _cam_len > 0) {
          _cam[_cam_len] = 0;
          handleCameraLine(_cam);
        }
        _cam_len = 0;
        _cam_overflow = false;
        continue;
      }
      if (_cam_len < (int)sizeof(_cam) - 1) _cam[_cam_len++] = (char)c;
      else _cam_overflow = true;
    }
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
    Serial.printf("alert_interval=%lu\n", (unsigned long)_prefs.alert_interval_sec);
    Serial.printf("min_alert_interval=%d\n", RUNNER_MIN_ALERT_INTERVAL_SEC);
    Serial.printf("status_interval=%lu\n", (unsigned long)_prefs.status_interval_sec);
    Serial.printf("min_status_interval=%d\n", RUNNER_MIN_STATUS_INTERVAL_SEC);
    Serial.printf("freq=%s\n", StrHelper::ftoa3(_prefs.freq));
    Serial.printf("bw=%s\n", StrHelper::ftoa3(_prefs.bw));
    Serial.printf("sf=%u\n", (unsigned)_prefs.sf);
    Serial.printf("cr=%u\n", (unsigned)_prefs.cr);
    Serial.printf("tx=%d\n", (int)_prefs.tx_power_dbm);
  }

  void printStatus() {
    printConfig();
    Serial.printf("time=%lu\n", (unsigned long)getRTCClock()->getCurrentTime());
    Serial.printf("camera_lines=%lu noise_lines=%lu\n", (unsigned long)_cam_ok, (unsigned long)_cam_noise);
    Serial.printf("alerts=%lu suppressed=%lu none_verdicts=%lu\n", (unsigned long)_policy.alerts(),
                  (unsigned long)_policy.suppressed(), (unsigned long)_policy.noneVerdicts());
    if (_have_sent) {
      Serial.printf("last_sent=%lus ago\n", (unsigned long)((millis() - _last_sent_ms) / 1000));
    } else {
      Serial.println("last_sent=never");
    }
    Serial.printf("next_status=%lds\n", (long)(_policy.nextStatusIn(millis()) / 1000));
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
    } else if (strcmp(args, "alert_interval") == 0) {
      if (!parse_uint(val, n) || n > 3600) return "alert_interval must be whole seconds, at most 3600";
      if (n < RUNNER_MIN_ALERT_INTERVAL_SEC) return "alert_interval below minimum";
      _prefs.alert_interval_sec = n;
    } else if (strcmp(args, "status_interval") == 0) {
      if (!parse_uint(val, n) || n > 604800) return "status_interval must be whole seconds";
      if (n < RUNNER_MIN_STATUS_INTERVAL_SEC) return "status_interval below minimum";
      _prefs.status_interval_sec = n;
    } else if (strcmp(args, "time") == 0) {   // set time <unix epoch seconds>; the node has no clock of its own
      if (!parse_uint(val, n) || n < 1700000000UL) return "time must be a unix epoch in seconds";
      getRTCClock()->setCurrentTime(n);
      _prefs.saved_time = n;
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
    _policy.setConfig(policyConfig());
    return savePrefs() ? NULL : "unable to save";
  }

  // Every command ends with a line starting "OK" or "ERR", so a script can tell when it is done
  // (the node may print "sent:" and "cam:" log lines in between).
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
    } else if (strcmp(command, "test") == 0) {   // diagnostics: send a "test" message now, to check radio and channel
      if (!_channel) { Serial.println("ERR: no channel"); return; }
      sendTest();
      Serial.println("OK");
    } else if (memcmp(command, "cam ", 4) == 0) {   // diagnostics: feed a camera line in as if it came over the UART
      handleCameraLine(&command[4]);
      Serial.println("OK");
    } else if (strcmp(command, "ver") == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
      Serial.println("OK");
    } else if (strcmp(command, "help") == 0) {
      Serial.println("Commands:");
      Serial.println("   get | status");
      Serial.println("   set name <name>");
      Serial.println("   set channel <psk_base64> <name>");
      Serial.printf("   set alert_interval <sec>    (min gap between alerts, min %d)\n", RUNNER_MIN_ALERT_INTERVAL_SEC);
      Serial.printf("   set status_interval <sec>   (heartbeat, min %d)\n", RUNNER_MIN_STATUS_INTERVAL_SEC);
      Serial.println("   set time <unix epoch>       (the node has no clock; saved hourly across reboots)");
      Serial.println("   set freq|bw|sf|cr|tx <value>");
      Serial.println("   test                        (send a test message now)");
      Serial.println("   cam <line>                  (feed a camera line, e.g. cam EW,VERDICT,1,PERSON,90)");
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
  void logTx(mesh::Packet* packet, int len) override {
    if (_tx_outstanding > 0) _tx_outstanding--;
    Serial.printf("tx done (%d bytes)\n", len);
  }
  void logTxFail(mesh::Packet* packet, int len) override {
    if (_tx_outstanding > 0) _tx_outstanding--;
    Serial.printf("tx FAILED (%d bytes)\n", len);
  }

  float getAirtimeBudgetFactor() const override { return 1.0f; }
  int calcRxDelay(float score, uint32_t air_time) const override { return 0; }

  // A pure sender: it never repeats other nodes' packets.
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
  RunnerMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    _channel = NULL;
    _radio_awake = true;   // radio_init() leaves it in standby
    _tx_outstanding = 0;
    _tx_deadline = 0;
    _have_sent = false;
    _last_sent_ms = 0;
    _cam_len = 0;
    _cam_overflow = false;
    _cam_ok = _cam_noise = 0;
    _cmd_len = 0;
    setDefaults();
  }

  float getFreqPref() const { return _prefs.freq; }
  float getBwPref() const { return _prefs.bw; }
  uint8_t getSfPref() const { return _prefs.sf; }
  uint8_t getCrPref() const { return _prefs.cr; }
  int8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }

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

    // No battery-backed clock: start from the last saved time (or whatever 'set time' stored) instead of 2024.
    if (_prefs.saved_time > getRTCClock()->getCurrentTime()) getRTCClock()->setCurrentTime(_prefs.saved_time + 1);

    _channel = addChannel(_prefs.channel_name, _prefs.channel_psk);
    if (!_channel) {   // corrupt PSK in flash: fall back to the compiled-in default rather than not reporting
      _channel = addChannel(RUNNER_CHANNEL_NAME, RUNNER_CHANNEL_PSK);
    }
    if (!_channel) Serial.println("ERROR: unable to set up the alert channel; nothing will be sent");
    _policy.begin(policyConfig(), millis());
  }

  void showWelcome() {
    Serial.println("===== MeshCore Runner Alert =====");
    Serial.println(FIRMWARE_VER_TEXT);
    Serial.println("   (enter 'help' for commands)");
    printConfig();
  }

  void loop() {
    if (_radio_awake) BaseChatMesh::loop();   // moves queued packets out and completes transmissions
    pollSerial();
    pollCamera();
    if (!_channel) return;

    if (_policy.tick(millis()).action == runner::AlertPolicy::SEND_STATUS) sendStatus();

    sleepRadioIfIdle();
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;
RunnerMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

  board.begin();

  if (!radio_init()) { halt(); }

  Serial1.begin(RUNNER_UART_BAUD);   // camera UART: D7 = RX (D6 = TX, unused). Needs XIAO_NO_I2C: D6/D7 are I2C pins too.

  fast_rng.begin(radio_driver.getRngSeed());

  InternalFS.begin();
  the_mesh.begin(InternalFS);

  radio_driver.setParams(the_mesh.getFreqPref(), the_mesh.getBwPref(), the_mesh.getSfPref(), the_mesh.getCrPref());
  radio_driver.setTxPower(the_mesh.getTxPowerPref());

  the_mesh.showWelcome();
  // NOTE: deliberately no initial advert - the node never advertises.
}

void loop() {
  the_mesh.loop();
  rtc_clock.tick();

  // Let the CPU sleep between passes. The camera's lines are short and sparse, but its boot text can burst
  // at power-up, so keep the pause small enough that the UART buffer is drained.
  delay(2);
}
