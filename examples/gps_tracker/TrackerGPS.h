#pragma once

#include <Arduino.h>

struct TrackerFix {
  bool valid;
  int32_t lat_e6, lon_e6;   // degrees * 1e6
  int32_t alt_m;
  uint8_t sats;
};

// Board-specific GPS glue (see TrackerGPS.cpp). The GPS is powered on in begin() and left running.
void tracker_gps_begin();
void tracker_gps_loop();                  // call every loop(); feeds the NMEA parser
bool tracker_gps_get_fix(TrackerFix& fix);   // true (and fix.valid) only if the receiver currently has a fix
