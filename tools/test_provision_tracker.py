import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import provision_tracker as pt

EXAMPLE = os.path.join(os.path.dirname(__file__), "tracker_config.example.json")


class FakeSerial:
    """Mimics the firmware CLI closely enough to exercise the script."""

    def __init__(self, min_interval=300, drop_psk=False):
        self.s = dict(name="Tracker", channel="tracker-test", psk="dHJhY2tlci10ZXN0LWtleQ==", interval="300",
                      min_interval=str(min_interval), nofix="notify", nofix_interval="1800",
                      freq="915.000", bw="250.000", sf="10", cr="5", tx="20",
                      idle_interval="1800", idle_holdoff="300", fix_timeout="90", move_dist="25", motion="32")
        self.out = []
        self.drop_psk = drop_psk
        self.rebooted = False

    def reset_input_buffer(self):
        self.out.clear()

    def write(self, b):
        cmd = b.decode().strip()
        self.out.append(b"sent: Tracker: 1,2 alt=3ft mv\n")   # unrelated log noise
        if cmd == "get":
            self.out += [f"{k}={v}\n".encode() for k, v in self.s.items()] + [b"fix=no\n", b"OK\n"]
        elif cmd == "reboot":
            self.rebooted = True
            self.out.append(b"OK rebooting\n")
        elif cmd.startswith("set "):
            _, key, *rest = cmd.split(" ", 2)
            val = rest[0] if rest else ""
            if key in ("interval", "idle_interval", "nofix_interval") and int(val) < int(self.s["min_interval"]):
                self.out.append(b"ERR: interval below minimum\n")
                return
            if key == "channel":
                p, n = val.split(" ", 1)
                self.s["channel"] = n
                if not self.drop_psk:
                    self.s["psk"] = p
            elif key in ("freq", "bw"):
                self.s[key] = f"{float(val):.3f}"
            else:
                self.s[key] = val
            self.out.append(b"OK\n")

    def readline(self):
        return self.out.pop(0) if self.out else b""


def run(dev, cfg, name=None, reboot=True):
    return pt.provision(pt.Device(dev, timeout=0.2), cfg, name, reboot=reboot, log=lambda *_: None)


class Provision(unittest.TestCase):
    def setUp(self):
        with open(EXAMPLE) as f:
            self.cfg = json.load(f)

    def test_happy_path_applies_and_reboots(self):
        d = FakeSerial()
        after = run(d, self.cfg, "Sweep 1")
        self.assertEqual((after["name"], after["freq"], after["bw"]), ("Sweep 1", "910.525", "62.500"))
        self.assertTrue(d.rebooted)

    def test_no_name_keeps_name(self):
        d = FakeSerial()
        run(d, self.cfg)
        self.assertEqual(d.s["name"], "Tracker")

    def test_motion_settings_are_applied(self):
        d = FakeSerial()
        run(d, dict(self.cfg, idle_interval_sec=1200, idle_holdoff_sec=420, fix_timeout_sec=120,
                    move_distance_m=40, motion_threshold=50), reboot=False)
        self.assertEqual((d.s["idle_interval"], d.s["idle_holdoff"], d.s["fix_timeout"], d.s["move_dist"], d.s["motion"]),
                         ("1200", "420", "120", "40", "50"))

    def test_floor_rejected_before_touching_device(self):
        d = FakeSerial()
        with self.assertRaises(pt.ProvisionError) as cm:
            run(d, dict(self.cfg, interval_sec=120))
        self.assertIn("300", str(cm.exception))
        self.assertEqual(d.s["interval"], "300")

    def test_idle_interval_also_has_a_floor(self):
        with self.assertRaises(pt.ProvisionError):
            run(FakeSerial(), dict(self.cfg, idle_interval_sec=100))

    def test_lower_floor_build_accepts_lower_interval(self):
        d = FakeSerial(min_interval=60)
        run(d, dict(self.cfg, interval_sec=120), reboot=False)
        self.assertEqual(d.s["interval"], "120")
        self.assertFalse(d.rebooted)

    def test_readback_mismatch_is_caught(self):
        d = FakeSerial(drop_psk=True)
        with self.assertRaises(pt.ProvisionError) as cm:
            run(d, dict(self.cfg, channel_psk="cmFjZS1rZXktMTIzNDU2Nw=="))
        self.assertIn("psk", str(cm.exception))

    def test_spaced_channel_name_and_hex_key(self):
        d = FakeSerial()
        run(d, dict(self.cfg, channel_name="GDR Sweeps", channel_psk="00112233445566778899aabbccddeeff"), "Sweep1", reboot=False)
        self.assertEqual((d.s["channel"], d.s["psk"]), ("GDR Sweeps", "ABEiM0RVZneImaq7zN3u/w=="))

    def test_unknown_and_invalid_keys(self):
        problems = pt.validate_config({"name": "x", "bw": 100, "sf": 20, "nofix_mode": "loud"}, 300)
        self.assertTrue(any("name" in p for p in problems))
        self.assertTrue(any("bw" in p for p in problems))
        self.assertTrue(any("sf" in p for p in problems))
        self.assertTrue(any("nofix_mode" in p for p in problems))


if __name__ == "__main__":
    unittest.main()
