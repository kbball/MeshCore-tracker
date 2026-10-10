#pragma once

#include <Arduino.h>

// Accelerometer motion detection (T-1000-E: QMA6100P any-motion interrupt on INT1).
// Boards without one report tracker_motion_available() == false and never see motion.

bool tracker_motion_begin(uint8_t threshold);   // true if the sensor was found and armed
bool tracker_motion_available();
void tracker_motion_set_threshold(uint8_t threshold);
bool tracker_motion_poll();                     // true if motion was seen since the last call (clears the flag)

// Bring-up/diagnostics
uint32_t tracker_motion_irq_count();            // interrupts seen since boot
void tracker_motion_dump(Print& out);           // addr, chip id, raw xyz, key registers
