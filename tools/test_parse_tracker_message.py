import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from parse_tracker_message import parse


class ParseTrackerMessage(unittest.TestCase):
    def test_position_moving(self):   # captured from a T-1000-E
        m = parse("Sweep1: 33.89037,-84.16927 alt=1037ft sats=6 bat=3.85V mv")
        self.assertEqual((m["name"], m["kind"], m["state"], m["moving"]), ("Sweep1", "position", "mv", True))
        self.assertEqual((m["lat"], m["lon"], m["alt_ft"], m["sats"], m["bat_v"]), (33.89037, -84.16927, 1037, 6, 3.85))

    def test_position_idle(self):
        m = parse("Sweep1: 33.89044,-84.16927 alt=1004ft sats=7 bat=3.85V idle")
        self.assertEqual((m["state"], m["moving"]), ("idle", False))

    def test_negative_altitude_and_southern_hemisphere(self):
        m = parse("Sweep2: -33.86880,151.20930 alt=-12ft sats=9 bat=4.01V mv")
        self.assertEqual((m["lat"], m["lon"], m["alt_ft"]), (-33.8688, 151.2093, -12))

    def test_name_with_spaces(self):
        self.assertEqual(parse("Sweep 1: 1.00000,2.00000 alt=3ft sats=4 bat=3.70V idle")["name"], "Sweep 1")

    def test_nofix_no_position(self):
        m = parse("Sweep1: no fix (no position yet) mv")
        self.assertEqual((m["kind"], m["state"], m["lat"], m["age_min"]), ("nofix", "mv", None, None))

    def test_nofix_with_last_position(self):
        m = parse("Sweep1: no fix (last 33.89057,-84.16948 25m ago) idle")
        self.assertEqual((m["kind"], m["state"], m["lat"], m["lon"], m["age_min"]), ("nofix", "idle", 33.89057, -84.16948, 25))

    def test_nofix_hours(self):
        self.assertEqual(parse("Sweep1: no fix (last 33.89057,-84.16948 3h ago) mv")["age_min"], 180)

    def test_body_without_name(self):
        self.assertIsNone(parse("33.89037,-84.16927 alt=1037ft sats=6 bat=3.85V mv")["name"])

    def test_rejects_other_messages(self):
        self.assertIsNone(parse("Bob: lunch at noon?"))
        self.assertIsNone(parse("Sweep1: 33.89037,-84.16927 alt=1037ft sats=6 bat=3.85V"))   # marker missing
        self.assertIsNone(parse("Sweep1: 33.89037,-84.16927 alt=1037ft sats=6 bat=3.85V moving"))


if __name__ == "__main__":
    unittest.main()
