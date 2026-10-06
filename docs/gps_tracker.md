# GPS Tracker Firmware

A send-only tracker: it **never sends adverts**, **never repeats** other nodes' packets, and
broadcasts its GPS position as a plain-text message to one channel at a fixed interval.
Any normal MeshCore client on that channel sees the reports as ordinary channel messages.

Source: `examples/gps_tracker/`.

## Supported boards

| Board | Build env | GPS |
|---|---|---|
| Seeed Studio T-1000-E | `t1000e_gps_tracker` | built in (`Serial1`, 115200 baud) |
| Seeed XIAO nRF52840 + Wio-SX1262 | `Xiao_nrf52_gps_tracker` | external Seeed L76K GNSS module for XIAO (`Serial1`, 9600 baud) |

```
pio run -e t1000e_gps_tracker
pio run -e Xiao_nrf52_gps_tracker
./build.sh build-gps-tracker-firmwares
```

### XIAO + L76K wiring

The L76K module talks NMEA over the XIAO hardware UART (D6 = TX, D7 = RX, 9600 baud).
It has no enable pin, so the GPS runs continuously. D6/D7 are normally also the I2C pins on
this variant, so the tracker build sets `XIAO_NO_I2C` and does not start I2C (no external RTC
auto-discovery). The clock is set from GPS time.

## Message format

```
<node name>: 37.77490,-122.41940 alt=39ft sats=9 bat=3.92V
```

When there is no fix at a reporting tick, behaviour depends on `nofix`:

* `silent` - nothing is sent.
* `notify` - a short notice such as `no fix (last 37.77490,-122.41940 25m ago)` (or
  `no fix (no position yet)`). Notices are rate-limited to one per `nofix_interval`
  (default 30 min, never faster than `interval`). The first notice after a good report goes out
  at the next tick, and a good report resets the limiter.

After boot, the first report is sent as soon as the first fix is obtained.

## Airtime: the minimum interval

Every report is flooded to the whole channel and re-transmitted by repeaters, so channel load
grows with (number of trackers / interval). The firmware refuses an interval below
`TRACKER_MIN_INTERVAL_SEC` (default **300 s**). A dedicated build can lower the floor with
`-D TRACKER_MIN_INTERVAL_SEC=...`; do that only knowing how many trackers share the channel.
Use a dedicated channel for trackers.

## Provisioning

Settings are stored in flash. Set them over the USB serial CLI (115200 baud) or, for several
units, with the provisioning script and a JSON file:

```
python3 tools/provision_tracker.py tools/tracker_config.example.json --name "Sweep 1"
```

The script needs `pyserial` (`pip install pyserial`, or use the PlatformIO python:
`~/.platformio/penv/bin/python tools/provision_tracker.py ...`). It validates the config
(including the unit's own interval floor), applies each setting, reads everything back to
verify, then reboots the unit. The node name is **not** in the JSON, so one config can be
applied to many units; give each its own `--name`. Without `--name` the unit keeps its name.
`--dry-run` validates and prints the commands without a device; `--port` selects the serial
port when more than one is present; `--no-reboot` skips the reboot.

```json
{
  "channel_name": "race",
  "channel_psk": "<base64, 16 or 32 bytes>",
  "interval_sec": 300,
  "nofix_mode": "notify",
  "nofix_notify_interval_sec": 1800,
  "freq": 910.525, "bw": 62.5, "sf": 7, "cr": 5, "tx_power": 22
}
```

All keys are optional, except `channel_name` and `channel_psk` go together. The channel PSK is the secret the companion apps use: base64, or hex (32/64 hex chars), which the script converts. The channel name is only a label.
The example config uses the firmware's built-in test key; generate your own for real use.

## Serial CLI

Every command ends with a line starting `OK` or `ERR`. Log lines (`sent: ...`) may appear in between.

| Command | Description |
|---|---|
| `get` / `status` | print settings (`key=value`), fix state, last send, battery |
| `set name <name>` | node name (1-31 chars) |
| `set channel <psk_base64> <name>` | channel to report to (name may contain spaces) |
| `set interval <sec>` | report interval (>= minimum) |
| `set nofix silent\|notify` | no-fix policy |
| `set nofix_interval <sec>` | spacing between no-fix notices |
| `set freq\|bw\|sf\|cr\|tx <value>` | radio parameters |
| `reboot` | apply channel, radio and name changes |
| `ver`, `help` | |

Each `set` is saved immediately; channel, radio and name changes take effect after `reboot`.

## Compile-time defaults

Used until a unit is provisioned: `TRACKER_CHANNEL_NAME`, `TRACKER_CHANNEL_PSK`,
`TRACKER_NODE_NAME`, `TRACKER_INTERVAL_SEC`, `TRACKER_MIN_INTERVAL_SEC`, `TRACKER_NOFIX_MODE`,
`TRACKER_NOFIX_NOTIFY_INTERVAL_SEC`, `TRACKER_FIRST_SEND_DELAY_SEC`, `TRACKER_GPS_BAUD` (XIAO),
and the usual `LORA_FREQ`/`LORA_BW`/`LORA_SF`/`LORA_CR`/`LORA_TX_POWER`.
