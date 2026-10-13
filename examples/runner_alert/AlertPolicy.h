#pragma once
// When the runner alert node sends something. Pure logic, no Arduino headers: unit tested on the host
// (examples/runner_alert/tests/alert_policy_test.cpp).
//
//   - A PERSON verdict from the camera becomes an alert, but never more often than min_alert_ms, because every
//     alert is flooded to the whole channel. Extra PERSON verdicts inside that gap are counted, not sent.
//   - A status heartbeat goes out status_ms after boot's first one, so the fleet can see the node is alive.
//
// Times are millis() values; comparisons use signed differences so they survive the 49-day wrap.

#include <stdint.h>

namespace runner {

class AlertPolicy {
public:
  enum Action : uint8_t { NONE, SEND_ALERT, SEND_STATUS };

  struct Config {
    uint32_t min_alert_ms;     // minimum gap between alerts
    uint32_t status_ms;        // heartbeat period
    uint32_t first_status_ms;  // delay from begin() to the first heartbeat
  };

  struct Result {
    Action action;
    uint8_t score;   // SEND_ALERT: the camera's score
  };

  void begin(const Config& c, uint32_t now) {
    cfg_ = c;
    next_status_ = now + c.first_status_ms;
    have_alert_ = false;
    alerts_ = suppressed_ = none_verdicts_ = 0;
  }

  void setConfig(const Config& c) { cfg_ = c; }

  // The camera reported a verdict. Only PERSON can produce an alert.
  Result onVerdict(bool person, uint32_t score, uint32_t now) {
    if (!person) {
      none_verdicts_++;
      return {NONE, 0};
    }
    if (have_alert_ && (int32_t)(now - last_alert_) < (int32_t)cfg_.min_alert_ms) {
      suppressed_++;
      return {NONE, 0};
    }
    have_alert_ = true;
    last_alert_ = now;
    alerts_++;
    return {SEND_ALERT, (uint8_t)(score > 100 ? 100 : score)};
  }

  // Call regularly. Returns SEND_STATUS when a heartbeat is due.
  Result tick(uint32_t now) {
    if ((int32_t)(now - next_status_) >= 0) {
      next_status_ = now + cfg_.status_ms;
      return {SEND_STATUS, 0};
    }
    return {NONE, 0};
  }

  uint32_t alerts() const { return alerts_; }
  uint32_t suppressed() const { return suppressed_; }
  uint32_t noneVerdicts() const { return none_verdicts_; }
  int32_t nextStatusIn(uint32_t now) const { return (int32_t)(next_status_ - now); }

private:
  Config cfg_ = {20000, 3600000, 15000};
  bool have_alert_ = false;
  uint32_t last_alert_ = 0;
  uint32_t next_status_ = 0;
  uint32_t alerts_ = 0, suppressed_ = 0, none_verdicts_ = 0;
};

}  // namespace runner
