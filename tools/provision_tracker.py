#!/usr/bin/env python3
"""Provision a MeshCore GPS tracker over its USB serial CLI from a JSON config file.

    python3 tools/provision_tracker.py tools/tracker_config.example.json --name "Sweep 1"

The node name is deliberately NOT part of the JSON (so one config can be applied to several
units); pass it with --name. Without --name the unit keeps whatever name it already has.

Needs pyserial (pip install pyserial; it is also in the PlatformIO venv:
~/.platformio/penv/bin/python tools/provision_tracker.py ...).
"""
import argparse
import base64
import json
import sys
import time

DEFAULT_MIN_INTERVAL = 300   # firmware default floor; the device's own floor is used when connected
VALID_BW = [7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500]

# JSON key -> (device setting, readback key). channel_name/channel_psk are handled together.
SIMPLE_KEYS = {
    "interval_sec": ("interval", "interval"),
    "nofix_mode": ("nofix", "nofix"),
    "nofix_notify_interval_sec": ("nofix_interval", "nofix_interval"),
    "freq": ("freq", "freq"),
    "bw": ("bw", "bw"),
    "sf": ("sf", "sf"),
    "cr": ("cr", "cr"),
    "tx_power": ("tx", "tx"),
    "idle_interval_sec": ("idle_interval", "idle_interval"),
    "idle_holdoff_sec": ("idle_holdoff", "idle_holdoff"),
    "fix_timeout_sec": ("fix_timeout", "fix_timeout"),
    "move_distance_m": ("move_dist", "move_dist"),
    "motion_threshold": ("motion", "motion"),
}
ALLOWED_KEYS = set(SIMPLE_KEYS) | {"channel_name", "channel_psk"}


def normalize_psk(psk):
    """The device wants base64. Accept the hex form too (32 or 64 hex chars -> 16 or 32 bytes)."""
    if isinstance(psk, str) and len(psk) in (32, 64) and all(c in "0123456789abcdefABCDEF" for c in psk):
        return base64.b64encode(bytes.fromhex(psk)).decode()
    return psk


class ProvisionError(Exception):
    pass


def validate_config(cfg, min_interval):
    """Check a config dict. Returns a list of problems (empty if fine)."""
    problems = []
    if not isinstance(cfg, dict):
        return ["config must be a JSON object"]
    for k in cfg:
        if k not in ALLOWED_KEYS:
            hint = " (the node name is set with --name, not in the JSON)" if k in ("name", "node_name") else ""
            problems.append(f"unknown key '{k}'{hint}")
    if ("channel_name" in cfg) != ("channel_psk" in cfg):
        problems.append("channel_name and channel_psk must be given together")
    for k in ("channel_name", "channel_psk"):
        if k in cfg and (not isinstance(cfg[k], str) or not cfg[k] or " " in cfg[k] and k == "channel_psk"):
            problems.append(f"{k} must be a non-empty string" + (" without spaces" if k == "channel_psk" else ""))
    if "channel_name" in cfg and isinstance(cfg["channel_name"], str):
        if len(cfg["channel_name"]) > 31:
            problems.append("channel_name must be at most 31 characters")

    def num(key, lo=None, hi=None, integer=True):
        if key not in cfg:
            return
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int,) if integer else (int, float)):
            problems.append(f"{key} must be a{'n integer' if integer else ' number'}")
        elif (lo is not None and v < lo) or (hi is not None and v > hi):
            problems.append(f"{key} must be between {lo} and {hi}")

    num("interval_sec", min_interval, 86400)
    num("nofix_notify_interval_sec", min_interval, 604800)
    num("idle_interval_sec", min_interval, 86400)
    num("idle_holdoff_sec", 60, 3600)
    num("fix_timeout_sec", 10, 300)
    num("move_distance_m", 5, 500)
    num("motion_threshold", 1, 255)
    num("freq", 150, 960, integer=False)
    num("sf", 5, 12)
    num("cr", 5, 8)
    num("tx_power", -9, 22)
    if "bw" in cfg:
        if not isinstance(cfg["bw"], (int, float)) or isinstance(cfg["bw"], bool) or not any(abs(cfg["bw"] - b) < 0.01 for b in VALID_BW):
            problems.append(f"bw must be one of {VALID_BW}")
    if "nofix_mode" in cfg and cfg["nofix_mode"] not in ("silent", "notify"):
        problems.append("nofix_mode must be 'silent' or 'notify'")
    return problems


def build_commands(cfg, name=None):
    """Ordered list of (command, readback_key, expected_value) for the config."""
    cmds = []
    if name is not None:
        cmds.append((f"set name {name}", "name", name))
    if "channel_name" in cfg:
        psk = normalize_psk(cfg["channel_psk"])
        cmds.append((f"set channel {psk} {cfg['channel_name']}", "channel", cfg["channel_name"]))
        cmds.append((None, "psk", psk))   # verify-only
    for key, (setting, rb) in SIMPLE_KEYS.items():
        if key in cfg:
            cmds.append((f"set {setting} {cfg[key]}", rb, cfg[key]))
    return cmds


def values_match(expected, actual):
    if actual is None:
        return False
    try:
        return abs(float(expected) - float(actual)) < 0.0015
    except (TypeError, ValueError):
        return str(expected) == str(actual)


class Device:
    def __init__(self, ser, timeout=5.0):
        self.ser = ser
        self.timeout = timeout

    def command(self, cmd):
        """Send a command, return its output lines (excluding the final OK/ERR line). Raises on ERR."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        lines = []
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line.startswith("OK"):
                return lines
            if line.startswith("ERR"):
                raise ProvisionError(f"'{cmd}' failed: {line}")
            lines.append(line)   # includes unrelated log lines such as "sent: ..."
        raise ProvisionError(f"timeout waiting for reply to '{cmd}'")

    def get(self):
        out = {}
        for line in self.command("get"):
            if "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
        return out


def find_port(requested):
    if requested:
        return requested
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid is not None]   # USB serial devices only
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise ProvisionError("no USB serial device found; plug in the tracker or pass --port")
    raise ProvisionError("several USB serial devices found, pass --port: " + ", ".join(p.device for p in ports))


def provision(dev, cfg, name, reboot=True, log=print):
    state = dev.get()
    if "interval" not in state:
        raise ProvisionError("device did not answer 'get'; is this tracker firmware running?")
    min_interval = int(state.get("min_interval", DEFAULT_MIN_INTERVAL))
    problems = validate_config(cfg, min_interval)
    if problems:
        raise ProvisionError("invalid config:\n  " + "\n  ".join(problems))

    cmds = build_commands(cfg, name)
    for cmd, _, _ in cmds:
        if cmd:
            shown = cmd if not cmd.startswith("set channel ") else "set channel <psk> " + cmd.split(" ", 3)[3]
            log(f"  > {shown}")
            dev.command(cmd)

    after = dev.get()
    bad = [(rb, exp, after.get(rb)) for _, rb, exp in cmds if not values_match(exp, after.get(rb))]
    if bad:
        raise ProvisionError("readback mismatch:\n  " + "\n  ".join(f"{k}: expected {e!r}, device has {a!r}" for k, e, a in bad))
    log("  verified: " + ", ".join(sorted({rb for _, rb, _ in cmds})))

    if reboot:
        log("  > reboot")
        try:
            dev.command("reboot")
        except ProvisionError:
            pass   # the device drops the port while rebooting
    return after


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="JSON config file")
    ap.add_argument("--name", help="node name for this unit (not part of the JSON)")
    ap.add_argument("--port", help="serial port (auto-detected if there is only one USB serial device)")
    ap.add_argument("--no-reboot", action="store_true", help="do not reboot afterwards (channel/radio/name changes apply on reboot)")
    ap.add_argument("--dry-run", action="store_true", help="validate the config and print the commands without a device")
    args = ap.parse_args()

    try:
        with open(args.config) as f:
            cfg = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        sys.exit(f"cannot read {args.config}: {e}")

    if args.name is not None and not (1 <= len(args.name) <= 31):
        sys.exit("--name must be 1-31 characters")

    if args.dry_run:
        problems = validate_config(cfg, DEFAULT_MIN_INTERVAL)
        if problems:
            sys.exit("invalid config:\n  " + "\n  ".join(problems))
        for cmd, _, _ in build_commands(cfg, args.name):
            if cmd:
                print(cmd if not cmd.startswith("set channel ") else "set channel <psk> " + cmd.split(" ", 3)[3])
        return

    try:
        import serial
    except ImportError:
        sys.exit("pyserial is required: pip install pyserial")

    try:
        port = find_port(args.port)
        print(f"provisioning {port}")
        with serial.Serial(port, 115200, timeout=0.5) as ser:
            time.sleep(0.3)
            provision(Device(ser), cfg, args.name, reboot=not args.no_reboot)
    except ProvisionError as e:
        sys.exit(f"error: {e}")
    print("done")


if __name__ == "__main__":
    main()
