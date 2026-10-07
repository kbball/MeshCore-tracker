#!/usr/bin/env python3
"""Parse messages sent by the MeshCore GPS tracker firmware (examples/gps_tracker).

The channel text is "<node name>: <body>"; the node name is the sender and is not part of the payload.
The last field is always the state marker: "mv" (moving) or "idle".

    position:  33.89057,-84.16948 alt=955ft sats=10 bat=3.77V mv
    no fix:    no fix (no position yet) mv
               no fix (last 33.89057,-84.16948 25m ago) idle       ("Nh ago" after two hours)

parse() accepts the whole "<name>: <body>" line, or just the body, and returns a dict (or None if it is
not a tracker message). The patterns are the stable contract; keep them in sync with the firmware.
"""
import re
import sys

COORD = r"-?\d+\.\d{5}"
POSITION = re.compile(
    rf"^(?P<lat>{COORD}),(?P<lon>{COORD}) alt=(?P<alt_ft>-?\d+)ft sats=(?P<sats>\d+) "
    rf"bat=(?P<bat_v>\d+\.\d{{2}})V (?P<state>mv|idle)$")
NOFIX = re.compile(
    rf"^no fix \((?:no position yet|last (?P<lat>{COORD}),(?P<lon>{COORD}) (?P<age>\d+)(?P<unit>[mh]) ago)\) "
    rf"(?P<state>mv|idle)$")


def parse(text):
    name = None
    body = text.strip()
    # "<name>: <body>": split on the first ": " that leaves a recognisable body (names may contain spaces)
    for m in re.finditer(r": ", body):
        head, rest = body[:m.start()], body[m.end():]
        if POSITION.match(rest) or NOFIX.match(rest):
            name, body = head, rest
            break

    m = POSITION.match(body)
    if m:
        d = m.groupdict()
        return {"name": name, "kind": "position", "lat": float(d["lat"]), "lon": float(d["lon"]),
                "alt_ft": int(d["alt_ft"]), "sats": int(d["sats"]), "bat_v": float(d["bat_v"]),
                "state": d["state"], "moving": d["state"] == "mv"}
    m = NOFIX.match(body)
    if m:
        d = m.groupdict()
        out = {"name": name, "kind": "nofix", "state": d["state"], "moving": d["state"] == "mv",
               "lat": None, "lon": None, "age_min": None}
        if d["lat"] is not None:
            out["lat"], out["lon"] = float(d["lat"]), float(d["lon"])
            out["age_min"] = int(d["age"]) * (60 if d["unit"] == "h" else 1)
        return out
    return None


if __name__ == "__main__":
    for line in (sys.argv[1:] or sys.stdin.read().splitlines()):
        print(parse(line))
