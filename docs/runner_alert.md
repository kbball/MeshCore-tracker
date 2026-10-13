# Runner alert node

Firmware for the early-warning system's MeshCore board. It sits next to the early-warning camera (XIAO ESP32-S3
Sense), reads the camera's UART, and posts a short text on one MeshCore channel when the camera reports a runner.
It is built like the GPS tracker: send-only, provisioned over the USB serial CLI, radio asleep between sends. An
optional Seeed L76K GNSS module sets its clock (time only; the position is not used or sent).

Code: `examples/runner_alert/`. Build env: `Xiao_nrf52_runner_alert` (`variants/xiao_nrf52/`), XIAO nRF52840 +
Wio-SX1262.

```
pio run -e Xiao_nrf52_runner_alert
g++ -std=c++11 -o /tmp/alert_test examples/runner_alert/tests/alert_policy_test.cpp && /tmp/alert_test
```

## How it fits

```
PIR node --(BLE trigger)--> trigger nRF --(EN)--> boost --> camera (ESP32-S3)
                                                              |  UART, ASCII "EW,..." lines
                                                              +--> trigger nRF (logs, drops EN)
                                                              +--> this node (D7 RX) --> MeshCore channel
```

| This node's pin | Goes to |
|---|---|
| D0 (RX of `Serial2`) | the camera's D6 (TX), the UART line. Common ground. Nothing is ever sent back |
| D6 / D7 (`Serial1`) | the L76K GNSS module, if fitted |

The camera is on D0 because the L76K module uses D6/D7, so the two could not share D7. The env defines
`PIN_SERIAL2_RX=0` and `PIN_SERIAL2_TX=13`, which makes the core build `Serial2` on the second UART (UARTE1). TX is
parked on the green LED pin only so the core has a valid pin; the node never transmits on it. MeshCore treats D0 as a
user button and sets an internal pull-up on it, which is harmless to the camera's push-pull line, but pressing a
button on D0 (if the board has one) would short the line to ground. The build also sets `XIAO_NO_I2C`, because
D6/D7 are the I2C pins on this variant.

## Camera lines

Defined by the early-warning camera firmware (repo `runner-early-warning`, `common/lib/CameraMsg`); `CameraLine.h`
here is a copy of the parse half. Keep them in sync.

```
EW,BOOT,<ms>
EW,FRAME,<ms>,<n>
EW,VERDICT,<ms>,PERSON|NONE,<score 0..100>
EW,DONE,<ms>
```

Anything else on the UART (the ESP32's boot text) is ignored. Only a `VERDICT ... PERSON` produces a message.

## Messages on the channel

Sent as a normal channel text from the node name (`<name>: <text>`). Keep the formats stable; fleet tooling parses them.

| Text | When |
|---|---|
| `runner score=<0-100> bat=<V>` | the camera reported PERSON; at most one per `alert_interval` (default 20 s, floor 10 s). Extra PERSON verdicts inside the gap are counted, not sent |
| `alive bat=<V> alerts=<n>` | heartbeat: first 15 s after boot, then every `status_interval` (default 3600 s, floor 300 s). `alerts` counts alerts sent since boot |
| `test bat=<V>` | the `test` CLI command only, so provisioning can check radio and channel without raising an alert |

## Serial CLI (USB, 115200)

Same shape as the tracker's. Every command ends with a line starting `OK` or `ERR`.

```
get | status
set name <name>
set channel <psk_base64> <name>
set alert_interval <sec>      min 10
set status_interval <sec>     min 300
set time <unix epoch>
set freq|bw|sf|cr|tx <value>
test                          send a test message now
cam <line>                    feed a camera line in as if it came over the UART, e.g. cam EW,VERDICT,1,PERSON,90
reboot | ver | help
```

Channel, name and radio changes apply after `reboot`. The compiled-in defaults (`runner-test` channel with a test
key, name `Runner`, 915 MHz BW250 SF10 CR5, 20 dBm) are for the bench only.

`tools/provision_tracker.py` is written for the tracker's settings and is **not** adapted for this node yet. The
`name`, `channel`, `freq`, `bw`, `sf`, `cr`, `tx` and `time` commands are the same, so a runner profile can reuse most of it.

## Things to know

- **Send-only.** It never advertises, never forwards, and ignores everything it hears, so the radio sleeps between
  sends. That also means it cannot receive anything over MeshCore: remote arm/disarm from the server would need the
  radio to listen (more power), which this build does not do.
- **Clock.** There is no battery-backed RTC; the clock starts at 15 May 2024. With the L76K fitted, the GPS provider
  sets the clock once a fix has held for a few seconds (and re-syncs every 30 minutes); `status` shows
  `source=gps`. Without a fix, `set time <epoch>` sets it, and the node saves the time hourly (on each heartbeat) and
  restores it on boot, so a reboot loses only the downtime. Whether receivers care about a wrong sender timestamp on
  channel messages has not been checked.
- **GPS power.** The XIAO + L76K module has no enable pin, so the GPS runs the whole time at an unmeasured current,
  probably tens of mA. That is accepted for deployments of up to about 36 hours. Gating its power (a load switch on
  the carrier board) is a later option. Build with `-D RUNNER_GPS=0` for a unit without a module.
- **GPS fix.** A node under trees may take a long time to get a fix, or never; `set time` is the fallback.
- **Camera power-up noise.** The ESP32's ROM prints text on the same UART when it powers up; it is counted as
  `noise_lines` and ignored.
- **Battery.** `bat=` is the XIAO's battery divider, valid with a cell on the BAT pads.

## Fleet tooling

This is a new device type. The fleet tooling does not know it yet and needs patching before it can flash and
provision these nodes: see the "Fleet tooling" note in `docs/ares_firmware.md` and the early-warning notes. In short:
a `kind` for the firmware manifest (`.github/ares-firmware.json` env `Xiao_nrf52_runner_alert`), the fleet-setup app
(profile, Event tab, CLI provisioning), and whatever parses channel messages for display.

## Status

Compiles and the policy and parser have host tests. **Not yet run on hardware.**
