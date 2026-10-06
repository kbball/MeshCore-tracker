#pragma once

// Decides when the tracker wakes its GPS, when it sends, and whether it is "moving" or "idle".
// Pure logic (no Arduino/hardware dependencies) so it can be tested on the host; see tests/.
//
//   * The accelerometer alone decides the state. A missing GPS fix is never evidence of being idle.
//   * moving -> idle: no motion for holdoff_ms, then one final fix cycle. If the GPS shows the unit has
//     actually moved more than move_dist_m since its last report, it stays moving instead.
//   * idle -> moving: any motion starts a fix cycle immediately.
//   * moving reports every interval_ms, idle heartbeats every idle_interval_ms.
//   * The GPS is powered only while a cycle is acquiring a fix (gps_on).

#include <stdint.h>
#include <math.h>

class ReportScheduler {
public:
  struct Config {
    uint32_t interval_ms;        // report spacing while moving
    uint32_t idle_interval_ms;   // heartbeat spacing while idle
    uint32_t holdoff_ms;         // stillness required before going idle
    uint32_t fix_timeout_ms;     // give up on a fix after this long
    uint32_t settle_ms;          // a fix must hold this long before it is used
    uint32_t move_dist_m;        // GPS displacement that overrides a still accelerometer
  };

  enum SendKind { SEND_NONE, SEND_POSITION, SEND_NOFIX };

  struct Input {
    uint32_t now;       // millis()
    bool motion;        // accelerometer saw motion since the last step
    bool fix_valid;     // a FRESH fix (obtained since the GPS was last powered on)
    int32_t lat_e6, lon_e6;
  };

  struct Output {
    bool gps_on;        // desired GPS power
    SendKind send;
    bool moving;        // state marker to put in the message
    bool forced;        // a state change: send even if no-fix notices are being rate limited
  };

  void begin(const Config& cfg, uint32_t now) {
    c = cfg;
    moving_ = true;            // report promptly after boot, then settle into idle if nothing moves
    last_motion_ = now;
    next_due_ = now;
    acquiring_ = settling_ = transition_ = have_last_ = false;
    acquire_start_ = settle_since_ = 0;
    last_lat_ = last_lon_ = 0;
  }

  void setConfig(const Config& cfg) { c = cfg; }
  bool moving() const { return moving_; }
  bool acquiring() const { return acquiring_; }
  uint32_t nextDue() const { return next_due_; }

  Output step(const Input& in) {
    Output out = { acquiring_, SEND_NONE, moving_, false };

    if (in.motion) {
      last_motion_ = in.now;                     // also cancels any go-idle decision pending in a running cycle
      if (!moving_) {
        moving_ = true;
        transition_ = true;
        if (!acquiring_) startCycle(in.now);     // report right away, not on the next timer
      }
    }

    if (!acquiring_) {
      bool still_long = moving_ && reached(in.now, last_motion_ + c.holdoff_ms);
      if (still_long || reached(in.now, next_due_)) startCycle(in.now);
    }

    if (acquiring_) {
      if (in.fix_valid) {
        if (!settling_) { settling_ = true; settle_since_ = in.now; }
        if (reached(in.now, settle_since_ + c.settle_ms)) return finish(in, true);
      } else {
        settling_ = false;
      }
      if (reached(in.now, acquire_start_ + c.fix_timeout_ms)) return finish(in, false);
    }

    out.gps_on = acquiring_;
    out.moving = moving_;
    return out;
  }

  static uint32_t distanceM(int32_t lat1_e6, int32_t lon1_e6, int32_t lat2_e6, int32_t lon2_e6) {
    double dlat = (double)(lat2_e6 - lat1_e6) * 1e-6 * 111320.0;
    double mid = ((double)lat1_e6 + (double)lat2_e6) * 0.5e-6 * 3.14159265358979 / 180.0;
    double dlon = (double)(lon2_e6 - lon1_e6) * 1e-6 * 111320.0 * cos(mid);
    return (uint32_t)sqrt(dlat * dlat + dlon * dlon);
  }

private:
  Config c;
  bool moving_, acquiring_, settling_, transition_, have_last_;
  uint32_t last_motion_, next_due_, acquire_start_, settle_since_;
  int32_t last_lat_, last_lon_;

  static bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }   // wrap-safe

  void startCycle(uint32_t now) {
    acquiring_ = true;
    settling_ = false;
    acquire_start_ = now;
  }

  Output finish(const Input& in, bool have_fix) {
    bool forced = transition_;
    // moving -> idle is decided when a cycle finishes: has the unit been still for the whole hold-off?
    if (moving_ && reached(in.now, last_motion_ + c.holdoff_ms)) {
      bool moved = have_fix && have_last_ && distanceM(last_lat_, last_lon_, in.lat_e6, in.lon_e6) > c.move_dist_m;
      if (moved) {
        last_motion_ = in.now;     // GPS says we are still travelling: stay moving, look again after another holdoff
      } else {
        moving_ = false;           // accelerometer still, GPS agrees or has no opinion
        forced = true;
      }
    }

    Output out = { false, have_fix ? SEND_POSITION : SEND_NOFIX, moving_, forced };
    if (have_fix) {
      last_lat_ = in.lat_e6;
      last_lon_ = in.lon_e6;
      have_last_ = true;
    }

    acquiring_ = settling_ = transition_ = false;
    if (moving_) {
      next_due_ = acquire_start_ + c.interval_ms;
      if (reached(in.now, next_due_)) next_due_ = in.now + c.interval_ms;
    } else {
      next_due_ = in.now + c.idle_interval_ms;
    }
    return out;
  }
};
