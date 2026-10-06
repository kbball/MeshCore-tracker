// Host test for ReportScheduler:  g++ -std=c++11 -o /tmp/sched_test examples/gps_tracker/tests/scheduler_test.cpp && /tmp/sched_test
#include "../ReportScheduler.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define MIN(x) ((x) * 60000UL)
#define SEC(x) ((x) * 1000UL)

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

struct Event { uint32_t t; ReportScheduler::SendKind kind; bool moving; bool forced; };

// Drives the scheduler one second at a time. motion(t) / fix(t) describe the world.
struct Sim {
  ReportScheduler s;
  uint32_t now = 0;
  bool (*motion)(uint32_t) = nullptr;
  bool (*fix)(uint32_t, int32_t&, int32_t&) = nullptr;   // only honoured while the GPS is on
  uint32_t gps_on_ms = 0;
  bool gps_on = false;
  uint32_t gps_on_since = 0;
  std::vector<Event> events;

  void init() {
    ReportScheduler::Config c = { MIN(5), MIN(30), MIN(5), SEC(90), SEC(3), 25 };
    s.begin(c, 0);
  }
  void run(uint32_t until) {
    for (; now <= until; now += 1000) {
      ReportScheduler::Input in = { now, motion ? motion(now) : false, false, 0, 0 };
      int32_t la = 0, lo = 0;
      // GPS gives a fresh fix 5 s after power-on (hot start), if the scenario has one
      if (gps_on && fix && now - gps_on_since >= 5000 && fix(now, la, lo)) { in.fix_valid = true; in.lat_e6 = la; in.lon_e6 = lo; }
      auto out = s.step(in);
      if (gps_on) gps_on_ms += 1000;
      if (out.gps_on && !gps_on) gps_on_since = now;
      gps_on = out.gps_on;
      if (out.send != ReportScheduler::SEND_NONE) events.push_back({ now, out.send, out.moving, out.forced });
    }
  }
  int count(bool moving, ReportScheduler::SendKind k, uint32_t from, uint32_t to) {
    int n = 0;
    for (auto& e : events) if (e.moving == moving && e.kind == k && e.t >= from && e.t < to) n++;
    return n;
  }
};

static bool never(uint32_t) { return false; }
static bool always_fix(uint32_t, int32_t& la, int32_t& lo) { la = 33890570; lo = -84169480; return true; }
static bool no_fix(uint32_t, int32_t&, int32_t&) { return false; }

// moving for the first 20 minutes, then still
static bool moves_20min(uint32_t t) { return t < MIN(20); }
// parked, then picked up at 2 h
static bool pickup_at_2h(uint32_t t) { return t >= MIN(120) && t < MIN(130); }
// fix that moves 100 m north between each 5-minute report
static bool travelling_fix(uint32_t t, int32_t& la, int32_t& lo) { la = 33890570 + (int32_t)(t / MIN(5)) * 900; lo = -84169480; return true; }

static void test_boot_then_idle() {
  printf("boot, no motion: one 'mv' report at boot, final 'idle' after hold-off, then 30-min heartbeats\n");
  Sim sim; sim.init(); sim.motion = never; sim.fix = always_fix; sim.run(MIN(100));
  CHECK(sim.events.size() >= 3, "got %zu events", sim.events.size());
  CHECK(sim.events[0].moving && sim.events[0].t < SEC(15), "first report is an mv report at boot (t=%u)", (unsigned)sim.events[0].t);
  CHECK(!sim.events[1].moving && sim.events[1].forced, "second report is the forced idle marker");
  CHECK(sim.events[1].t >= MIN(5) && sim.events[1].t < MIN(5) + SEC(15), "idle declared right after hold-off (t=%u)", (unsigned)sim.events[1].t);
  // heartbeats at ~ +30 min spacing
  for (size_t i = 2; i < sim.events.size(); i++) {
    uint32_t gap = sim.events[i].t - sim.events[i - 1].t;
    CHECK(!sim.events[i].moving, "heartbeat is marked idle");
    CHECK(gap >= MIN(30) && gap <= MIN(30) + SEC(15), "heartbeat gap %u s", (unsigned)(gap / 1000));
  }
}

static void test_moving_cadence() {
  printf("continuous motion: report every 5 min, always mv\n");
  Sim sim; sim.init(); sim.motion = [](uint32_t) { return true; }; sim.fix = always_fix; sim.run(MIN(60));
  CHECK(sim.count(false, ReportScheduler::SEND_POSITION, 0, MIN(60)) == 0, "never idle while moving");
  int n = sim.count(true, ReportScheduler::SEND_POSITION, 0, MIN(60));
  CHECK(n >= 11 && n <= 13, "%d reports in an hour", n);
  for (size_t i = 1; i < sim.events.size(); i++) {
    uint32_t gap = sim.events[i].t - sim.events[i - 1].t;
    CHECK(gap >= MIN(5) - SEC(2) && gap <= MIN(5) + SEC(15), "gap %u s", (unsigned)(gap / 1000));
  }
}

static void test_stop_then_idle() {
  printf("motion stops at 20 min: last fix marked idle ~ hold-off later, then heartbeat\n");
  Sim sim; sim.init(); sim.motion = moves_20min; sim.fix = always_fix; sim.run(MIN(120));
  uint32_t idle_at = 0;
  for (auto& e : sim.events) if (!e.moving) { idle_at = e.t; break; }
  CHECK(idle_at >= MIN(25) && idle_at <= MIN(25) + SEC(15), "idle at %u s (expect ~25 min)", (unsigned)(idle_at / 1000));
  CHECK(sim.count(true, ReportScheduler::SEND_POSITION, idle_at, MIN(120)) == 0, "no more mv reports once idle");
}

static void test_pickup_immediate() {
  printf("parked, then picked up: immediate mv report (not next timer)\n");
  Sim sim; sim.init(); sim.motion = pickup_at_2h; sim.fix = always_fix; sim.run(MIN(140));
  uint32_t first_mv = 0;
  for (auto& e : sim.events) if (e.moving && e.t >= MIN(120)) { first_mv = e.t; break; }
  CHECK(first_mv >= MIN(120) && first_mv <= MIN(120) + SEC(15), "mv report %u s after pickup", (unsigned)((first_mv - MIN(120)) / 1000));
  bool forced = false;
  for (auto& e : sim.events) if (e.moving && e.t == first_mv) forced = e.forced;
  CHECK(forced, "state-change report is forced");
}

static void test_brief_motion_holdoff() {
  printf("brief motion (3 min stop) does not flip to idle\n");
  Sim sim; sim.init();
  sim.motion = [](uint32_t t) { return t < MIN(10) || (t >= MIN(13) && t < MIN(30)); };
  sim.fix = always_fix; sim.run(MIN(30));
  CHECK(sim.count(false, ReportScheduler::SEND_POSITION, 0, MIN(30)) == 0, "stayed moving");
}

static void test_no_fix_not_idle() {
  printf("moving with no GPS fix: stays mv, sends no-fix, GPS cycles\n");
  Sim sim; sim.init(); sim.motion = [](uint32_t) { return true; }; sim.fix = no_fix; sim.run(MIN(30));
  CHECK(sim.count(false, ReportScheduler::SEND_NOFIX, 0, MIN(30)) == 0, "never idle on no fix");
  int n = sim.count(true, ReportScheduler::SEND_NOFIX, 0, MIN(30));
  CHECK(n >= 5, "%d no-fix reports", n);
}

static void test_still_no_fix_goes_idle() {
  printf("accelerometer still, GPS has no fix: accelerometer wins -> idle (forced no-fix message)\n");
  Sim sim; sim.init(); sim.motion = never; sim.fix = no_fix; sim.run(MIN(10));
  bool saw = false;
  for (auto& e : sim.events) if (!e.moving && e.kind == ReportScheduler::SEND_NOFIX && e.forced) saw = true;
  CHECK(saw, "idle transition sent as forced no-fix");
}

static void test_gps_overrides_still_accel() {
  printf("accelerometer quiet but GPS shows 100 m travel: stays moving\n");
  Sim sim; sim.init(); sim.motion = never; sim.fix = travelling_fix; sim.run(MIN(60));
  CHECK(sim.count(false, ReportScheduler::SEND_POSITION, 0, MIN(60)) == 0, "never declared idle while GPS shows travel");
}

static void test_gps_off_between() {
  printf("GPS power: on only while acquiring\n");
  Sim sim; sim.init(); sim.motion = never; sim.fix = always_fix; sim.run(MIN(60));
  CHECK(sim.gps_on_ms < SEC(60) * 6, "GPS on %u s of 3600 s", (unsigned)(sim.gps_on_ms / 1000));
}

static void test_motion_aborts_idle_check() {
  printf("motion arriving during the go-idle fix cycle cancels it\n");
  Sim sim; sim.init();
  sim.motion = [](uint32_t t) { return t == MIN(5) + SEC(2); };   // one bump while the final cycle is acquiring
  sim.fix = always_fix; sim.run(MIN(6));
  CHECK(sim.count(false, ReportScheduler::SEND_POSITION, 0, MIN(6)) == 0, "not idle after the bump");
}

int main() {
  test_boot_then_idle(); test_moving_cadence(); test_stop_then_idle(); test_pickup_immediate();
  test_brief_motion_holdoff(); test_no_fix_not_idle(); test_still_no_fix_goes_idle();
  test_gps_overrides_still_accel(); test_gps_off_between(); test_motion_aborts_idle_check();
  printf(failures ? "\n%d FAILURES\n" : "\nall passed\n", failures);
  return failures ? 1 : 0;
}
