#include "TrackerGPS.h"
#include <target.h>

#if defined(T1000_E)

#include <helpers/sensors/LocationProvider.h>

void tracker_gps_begin() {
  sensors.begin();                        // opens Serial1 to the GNSS chip
  sensors.setSettingValue("gps", "1");    // power sequencing; stays on for the life of the tracker
}

void tracker_gps_loop() {
  sensors.loop();                         // pumps NMEA, syncs the RTC from GPS time
}

bool tracker_gps_get_fix(TrackerFix& fix) {
  LocationProvider* gps = sensors.getLocationProvider();
  fix.valid = gps != NULL && gps->isValid();
  if (!fix.valid) return false;

  fix.lat_e6 = gps->getLatitude();
  fix.lon_e6 = gps->getLongitude();
  fix.alt_m = gps->getAltitude() / 1000;  // mm -> m
  fix.sats = gps->satellitesCount();
  return true;
}

#elif defined(XIAO_NRF52)

// Seeed L76K GNSS module for XIAO: NMEA over the XIAO hardware UART (Serial1: D6 = TX, D7 = RX).
// The module has no enable pin, so the receiver runs continuously. D6/D7 are shared with I2C
// on this variant, so the tracker build must not start Wire (see XIAO_NO_I2C in target.cpp).
#include <helpers/sensors/MicroNMEALocationProvider.h>

#ifndef TRACKER_GPS_BAUD
  #define TRACKER_GPS_BAUD   9600   // L76K factory default
#endif

static MicroNMEALocationProvider gps_nmea(Serial1, &rtc_clock);

void tracker_gps_begin() {
  Serial1.begin(TRACKER_GPS_BAUD);
  gps_nmea.begin();
}

void tracker_gps_loop() {
  gps_nmea.loop();   // pumps NMEA, syncs the RTC from GPS time
}

bool tracker_gps_get_fix(TrackerFix& fix) {
  fix.valid = gps_nmea.isValid();
  if (!fix.valid) return false;

  fix.lat_e6 = gps_nmea.getLatitude();
  fix.lon_e6 = gps_nmea.getLongitude();
  fix.alt_m = gps_nmea.getAltitude() / 1000;   // mm -> m
  fix.sats = gps_nmea.satellitesCount();
  return true;
}

#else
  #error "gps_tracker: no GPS support for this board yet (see TrackerGPS.cpp)"
#endif
