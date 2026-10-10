# MeshCore-tracker (fork)

Fork of MeshCore firmware (`origin` = `kbball/MeshCore-tracker`, upstream = `meshcore-dev/MeshCore`)
with a GPS tracker firmware added. Primary boards: Seeed XIAO nRF52840 + Wio-SX1262 (with the
L76K GNSS module) and the Seeed T-1000-E.

## Sync with upstream first

Before starting work on anything, pull from upstream so we build on current code:

1. Make sure an `upstream` remote exists; if not: `git remote add upstream https://github.com/meshcore-dev/MeshCore.git`
2. Make sure the working tree is clean (commit or stash first; don't pull over uncommitted work).
3. `git fetch upstream`, then bring `upstream/main` into local `main` (fast-forward/merge).
4. If there are conflicts, stop and ask rather than resolving silently.

Only then create a branch or start editing. Re-check this at the start of each new piece of work.

## GPS tracker

Code is in `examples/gps_tracker/`; docs in `docs/gps_tracker.md`; provisioning script in
`tools/provision_tracker.py`. Build envs: `t1000e_gps_tracker` (`variants/t1000-e/`) and
`Xiao_nrf52_gps_tracker` (`variants/xiao_nrf52/`).

```
pio run -e t1000e_gps_tracker
pio run -e Xiao_nrf52_gps_tracker
```

Design decisions (keep these unless the user changes them):

- Send-only: never advertise, never forward (`allowPacketForward` is false), ignore received adverts.
- Position goes out as plain text on one channel: `<name>: lat,lon alt=..ft sats=.. bat=..V mv|idle`.
  The last field is always the state marker (`mv` = moving, `idle`); no-fix messages carry it too.
  Parsed by `tools/parse_tracker_message.py` - keep the format stable and in sync with the firmware.
- T-1000-E is motion aware (`ReportScheduler.h`, pure logic with host tests): the accelerometer alone
  decides moving vs idle; a missing fix is never evidence of idle. Moving reports every 300 s; idle sends
  a 30 min heartbeat; motion start reports once a fix is obtained; after a hold-off of stillness one last
  fix is sent marked `idle`. GPS displacement over `move_dist` overrides a still accelerometer.
- GPS is powered only while acquiring a fix (T-1000-E); the radio sleeps between sends; the CPU idles.
  XIAO + L76K can't power-gate the GPS and has no accelerometer: it stays on and always reports `mv`.
- Interval floor is `TRACKER_MIN_INTERVAL_SEC` (default 300 s), enforced by the firmware CLI and the
  provisioning script. Reports are flooded, so don't lower it casually.
- No fix: runtime setting `nofix` = `notify` (default, rate-limited by `nofix_interval`) or `silent`;
  state-change messages are always sent.
- Node name is set per unit with `--name`; it is intentionally not part of the JSON config.
- Board GPS glue is in `TrackerGPS.cpp`, accelerometer in `TrackerMotion.cpp`. XIAO + L76K uses `Serial1`
  (D6 TX / D7 RX, 9600 baud), has no enable pin, and the build sets `XIAO_NO_I2C` (D6/D7 shared with I2C).

Gotchas:

- Tracker envs must define `MAX_GROUP_CHANNELS`; without it `BaseChatMesh::addChannel()` is a stub that
  returns NULL and nothing is ever sent (main.cpp has a `#error` for this).
- Don't `#include <base64.hpp>` in the example; it's header-only and already compiled into BaseChatMesh.cpp.
- Provisioning configs (`tools/*.json`, e.g. `gdr-2027.json`) contain channel keys and are gitignored.
  Only `tools/tracker_config.example.json` is tracked.
- The USB port disappears on `reboot`/reflash; scripts must reconnect.
- Prefs are in `/tracker_prefs`; the struct is append-only so existing units keep their settings on upgrade.

Tests: `examples/gps_tracker/tests/scheduler_test.cpp` (g++), `tools/test_provision_tracker.py`,
`tools/test_parse_tracker_message.py`.

Status: T-1000-E verified on hardware (flash, provisioning, reports received on the channel, accelerometer
interrupt, boot -> idle -> motion -> mv -> idle cycle, radio sleep + later sends complete). Not yet done:
XIAO + L76K on hardware; the no-advert/no-forward behaviour on air; tuning `motion`/`idle_holdoff` in a real
vehicle; measuring current draw (the estimates in discussion were never measured).
