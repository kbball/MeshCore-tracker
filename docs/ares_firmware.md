# ARES firmware releases

The ARES team uses a fixed set of boards. One tag builds all of them and publishes a GitHub Release that
[meshcore-fleet-setup](https://github.com/kbball/meshcore-fleet-setup) flashes from and that the
[aredn-go-box](https://github.com/kbball/aredn-go-box) downloads before an event.

## What is built

`.github/ares-firmware.json` is the list (env, board, kind). Both the build matrix and `manifest.json` come from it.

| Env | Board | Use |
|---|---|---|
| `Xiao_nrf52_companion_radio_usb_ble` | Seeed XIAO nRF52840 | companion (people), USB + BLE |
| `t1000e_companion_radio_usb_ble` | Seeed T-1000-E | companion, USB + BLE |
| `RAK_4631_companion_radio_usb_ble` | RAK4631 | companion, USB + BLE |
| `RAK_4631_companion_radio_usb_ethernet` | RAK4631 + RAK13800 | gateway the meshcore-mqtt bridge reaches over TCP (port 5000, DHCP, PoE); also has USB serial so the fleet tool can configure it (the plain `_ethernet` env has no USB or BLE) |
| `t1000e_gps_tracker` | Seeed T-1000-E | sweep tracker |
| `Xiao_nrf52_gps_tracker` | Seeed XIAO nRF52840 | sweep tracker |

The gateway is listed as kind `companion` in the manifest: the fleet tool's manifest has no `gateway` kind yet.
To add or drop a board, edit the JSON file; nothing else changes.

## Releasing

```sh
git tag ares-v1.0.0
git push origin ares-v1.0.0
```

The tag needs **exactly one hyphen**: the firmware version is everything after the last one (`v1.0.0`), as in
`.github/actions/setup-build-environment`. So `ares-v1.2.0`, not `ares-v1.2.0-rc1`. The version a device reports is
`<tag version>-<short commit>`, for example `v1.0.0-5f3eef0`; the manifest uses the same string.

Run the **ARES firmware** workflow by hand (Actions, Run workflow) to build every board without publishing; the
bundle is kept as a workflow artifact.

## What a release contains

| Asset | What it is |
|---|---|
| `ares-firmware-<version>.tar.gz` | `manifest.json` plus `<version>/<env>.zip`, laid out the way meshcore-fleet-setup serves them. Extract it into the fleet tool's `firmware/` folder. |
| `manifest.json` | The same manifest on its own: board, env, kind, version, file and sha256 for each zip. |
| `<env>.zip` | One DFU package per board, for flashing by hand. |

Only one version is in a release. Tracker firmware must keep sending reports that sweep-tracker's parser reads
(`Name: lat,lon alt=…ft sats=… bat=…V mv|idle`); say so in the release notes if that format changes.
