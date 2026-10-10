# GPS Tracker Firmware

A send-only tracker: it **never sends adverts**, **never repeats** other nodes' packets, and
broadcasts its GPS position as a plain-text message to one channel. Any normal MeshCore client on
that channel sees the reports as ordinary channel messages.

On the T-1000-E it is motion aware: it reports frequently while moving, sends a slow heartbeat
while idle, and powers the GPS and radio down in between.

Source: `examples/gps_tracker/`.

## Supported boards

| Board | Build env | GPS | Motion / power-down |
|---|---|---|---|
| Seeed T-1000-E | `t1000e_gps_tracker` | built in (`Serial1`, 115200 baud) | accelerometer (QMA6100P), GPS and radio power-cycled |
| Seeed XIAO nRF52840 + Wio-SX1262 | `Xiao_nrf52_gps_tracker` | external L76K module (`Serial1`, 9600 baud) | none: always reports as moving, GPS stays on |

```
pio run -e t1000e_gps_tracker
pio run -e Xiao_nrf52_gps_tracker
./build.sh build-gps-tracker-firmwares
```

The XIAO build is **not yet tested on hardware**.

### T-1000-E buttons and sounds

A startup tune plays once the tracker is running. **Hold the button for 1.5 s** to power off (shutdown
tune, then off); press it again to start. The hold time is `TRACKER_POWER_OFF_HOLD_MS`.

### XIAO + L76K wiring

The L76K module talks NMEA over the XIAO hardware UART (D6 = TX, D7 = RX, 9600 baud). It has no
enable pin, so the GPS runs continuously. D6/D7 are normally also the I2C pins on this variant, so the
tracker build sets `XIAO_NO_I2C` and does not start I2C. The clock is set from GPS time.

## How reporting works

The **accelerometer alone decides** whether the unit is moving or idle. A missing GPS fix is never
treated as being idle, because a moving vehicle loses its fix under tree cover.

| State | Behaviour |
|---|---|
| **moving** | one report every `interval_sec` (default 300) |
| motion starts while idle | the GPS powers up and a report is sent as soon as a fix is obtained (typically 30-60 s) |
| motion stops | after `idle_holdoff_sec` (default 300) of stillness, one last fix is sent, marked `idle` |
| **idle** | one heartbeat every `idle_interval_sec` (default 1800); GPS and radio asleep in between |

* **GPS confirmation:** if the accelerometer has been still but the final fix is more than
  `move_distance_m` (default 25 m) from the previous report, the unit stays `mv` (e.g. a smooth ride).
* **No fix:** the state is unchanged. A no-fix message is sent as set by `nofix`: `silent` sends nothing,
  `notify` sends a short notice, rate limited to one per `nofix_interval` (default 30 min). A state
  change (motion started / went idle) is always announced, even if it has to be a no-fix message.
* **Power:** the GPS is on only while acquiring a fix (the first fix takes roughly 30-60 s); the radio
  sleeps between sends (the tracker never listens); the CPU idles between loop passes.
* **Boot:** it reports as soon as it has a fix, as `mv`, and then goes idle after the hold-off if nothing moves.
* A unit with no working accelerometer (XIAO) never goes idle and reports every `interval_sec`.

## Message format

The sender name is the node name (not part of the payload). The last field is always the state marker,
`mv` (moving) or `idle`.

```
Sweep1: 33.89057,-84.16948 alt=955ft sats=10 bat=3.77V mv
Sweep1: no fix (no position yet) mv
Sweep1: no fix (last 33.89057,-84.16948 25m ago) idle        ("3h ago" after two hours)
```

`tools/parse_tracker_message.py` parses all three (and `tools/test_parse_tracker_message.py` pins the
format). Altitude is in feet. Coordinates have 5 decimals.

## Airtime: the minimum interval

Every report is flooded to the whole channel and re-transmitted by repeaters, so channel load grows with
(number of trackers / interval). The firmware refuses `interval_sec`, `idle_interval_sec` or
`nofix_interval` below `TRACKER_MIN_INTERVAL_SEC` (default **300 s**). A dedicated build can lower the
floor with `-D TRACKER_MIN_INTERVAL_SEC=...`; do that only knowing how many trackers share the channel.
Use a dedicated channel for trackers.

## Provisioning

Settings are stored in flash. Set them over the USB serial CLI (115200 baud) or, for several units, with
the provisioning script and a JSON file:

```
python3 tools/provision_tracker.py tools/tracker_config.example.json --name "Sweep1"
```

The script needs `pyserial` (`pip install pyserial`, or use the PlatformIO python:
`~/.platformio/penv/bin/python tools/provision_tracker.py ...`). It validates the config (including the
unit's own interval floor), applies each setting, reads everything back to verify, then reboots the unit.
The node name is **not** in the JSON, so one config can be applied to many units; give each its own
`--name`. Without `--name` the unit keeps its name. `--dry-run` validates and prints the commands without a
device; `--port` selects the serial port; `--no-reboot` skips the reboot.

**Config files contain the channel key, so only `tools/tracker_config.example.json` is tracked by git;
every other `tools/*.json` is ignored.** Keep your real config out of any public repository.

```json
{
  "channel_name": "GDR Sweeps",
  "channel_psk": "<hex (32/64 chars) or base64, 16 or 32 bytes>",
  "interval_sec": 300,
  "idle_interval_sec": 1800,
  "idle_holdoff_sec": 300,
  "fix_timeout_sec": 90,
  "move_distance_m": 25,
  "motion_threshold": 32,
  "nofix_mode": "notify",
  "nofix_notify_interval_sec": 1800,
  "freq": 910.525, "bw": 62.5, "sf": 7, "cr": 5, "tx_power": 22
}
```

All keys are optional, except `channel_name` and `channel_psk` go together. The channel name is only a
label and may contain spaces; the PSK is the secret the companion apps use.

## Serial CLI

Every command ends with a line starting `OK` or `ERR`. Log lines (`sent:`, `tx done`) may appear in between.

| Command | Description |
|---|---|
| `get` / `status` | settings as `key=value`, plus state, GPS power, fix, last send, battery |
| `set name <name>` | node name (1-31 chars) |
| `set channel <psk_base64> <name>` | channel to report to (the name may contain spaces) |
| `set interval <sec>` | report interval while moving (>= minimum) |
| `set idle_interval <sec>` | heartbeat while idle (>= minimum) |
| `set idle_holdoff <sec>` | stillness before going idle (60-3600) |
| `set fix_timeout <sec>` | give up waiting for a fix (10-300) |
| `set move_dist <metres>` | GPS displacement that confirms movement (5-500) |
| `set motion <1-255>` | accelerometer sensitivity; applies immediately (higher = less sensitive) |
| `set nofix silent\|notify`, `set nofix_interval <sec>` | no-fix policy |
| `set freq\|bw\|sf\|cr\|tx <value>` | radio parameters |
| `accel` | accelerometer diagnostics: raw x/y/z and the interrupt counter |
| `gps on\|off` | power the GPS by hand (diagnostics; the tracker takes it back over) |
| `reboot` | apply channel, radio and name changes |
| `ver`, `help` | |

Each `set` is saved immediately; channel, radio and name changes take effect after `reboot`.

## Tuning the motion threshold

`motion` is the accelerometer's any-motion threshold. The default (32) stayed quiet on a unit at rest and
fired on a hand shake, but it has **not been tuned in a vehicle**. If a parked unit keeps reporting `mv`,
raise it; if a moving unit goes idle, lower it. `accel` shows the interrupt counter, which should stay
flat while still. `idle_holdoff_sec` already absorbs short stops.

## Tests

* `g++ -std=c++14 -o /tmp/sched examples/gps_tracker/tests/scheduler_test.cpp && /tmp/sched` - the
  moving/idle state machine (`ReportScheduler.h`, hardware independent)
* `python3 tools/test_provision_tracker.py`, `python3 tools/test_parse_tracker_message.py`

## Compile-time defaults

Used until a unit is provisioned: `TRACKER_CHANNEL_NAME`, `TRACKER_CHANNEL_PSK`, `TRACKER_NODE_NAME`,
`TRACKER_INTERVAL_SEC`, `TRACKER_MIN_INTERVAL_SEC`, `TRACKER_IDLE_INTERVAL_SEC`, `TRACKER_IDLE_HOLDOFF_SEC`,
`TRACKER_FIX_TIMEOUT_SEC`, `TRACKER_FIX_SETTLE_SEC`, `TRACKER_MOVE_DIST_M`, `TRACKER_MOTION_THRESHOLD`,
`TRACKER_NOFIX_MODE`, `TRACKER_NOFIX_NOTIFY_INTERVAL_SEC`, `TRACKER_GPS_BAUD` (XIAO), and the usual
`LORA_FREQ`/`LORA_BW`/`LORA_SF`/`LORA_CR`/`LORA_TX_POWER`.
