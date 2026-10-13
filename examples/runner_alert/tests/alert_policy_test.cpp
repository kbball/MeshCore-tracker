// Host test for AlertPolicy and CameraLine:
//   g++ -std=c++11 -o /tmp/alert_test examples/runner_alert/tests/alert_policy_test.cpp && /tmp/alert_test
#include "../AlertPolicy.h"
#include "../CameraLine.h"
#include <stdio.h>

using namespace runner;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { printf("  FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

#define SEC(x) ((uint32_t)((x) * 1000UL))   // uint32_t like millis(), even where unsigned long is 64-bit

static AlertPolicy make(uint32_t min_alert_s = 20, uint32_t status_s = 3600, uint32_t first_s = 15, uint32_t now = 0) {
  AlertPolicy p;
  p.begin({SEC(min_alert_s), SEC(status_s), SEC(first_s)}, now);
  return p;
}

static void test_person_becomes_an_alert() {
  AlertPolicy p = make();
  auto r = p.onVerdict(true, 87, SEC(100));
  CHECK(r.action == AlertPolicy::SEND_ALERT, "person should alert");
  CHECK(r.score == 87, "score carried through, got %u", r.score);
  CHECK(p.alerts() == 1, "alerts=%u", (unsigned)p.alerts());
}

static void test_none_verdict_never_alerts() {
  AlertPolicy p = make();
  auto r = p.onVerdict(false, 4, SEC(100));
  CHECK(r.action == AlertPolicy::NONE, "NONE verdict must not alert");
  CHECK(p.noneVerdicts() == 1, "counted");
  CHECK(p.alerts() == 0, "no alerts");
}

static void test_alerts_are_rate_limited() {
  AlertPolicy p = make(20);
  CHECK(p.onVerdict(true, 90, SEC(100)).action == AlertPolicy::SEND_ALERT, "first sends");
  CHECK(p.onVerdict(true, 90, SEC(110)).action == AlertPolicy::NONE, "10 s later is suppressed");
  CHECK(p.onVerdict(true, 90, SEC(119)).action == AlertPolicy::NONE, "19 s later is suppressed");
  CHECK(p.onVerdict(true, 90, SEC(120)).action == AlertPolicy::SEND_ALERT, "20 s later sends again");
  CHECK(p.alerts() == 2, "alerts=%u", (unsigned)p.alerts());
  CHECK(p.suppressed() == 2, "suppressed=%u", (unsigned)p.suppressed());
}

static void test_suppressed_alert_does_not_extend_the_gap() {
  AlertPolicy p = make(20);
  p.onVerdict(true, 90, SEC(100));
  p.onVerdict(true, 90, SEC(115));   // suppressed; the gap is still measured from 100 s
  CHECK(p.onVerdict(true, 90, SEC(121)).action == AlertPolicy::SEND_ALERT, "gap measured from the last sent alert");
}

static void test_score_is_capped() {
  AlertPolicy p = make();
  CHECK(p.onVerdict(true, 250, SEC(1)).score == 100, "score capped at 100");
}

static void test_first_status_after_the_delay_then_periodic() {
  AlertPolicy p = make(20, 3600, 15, SEC(5));
  CHECK(p.tick(SEC(5)).action == AlertPolicy::NONE, "not at boot");
  CHECK(p.tick(SEC(19)).action == AlertPolicy::NONE, "not before 15 s after begin");
  CHECK(p.tick(SEC(20)).action == AlertPolicy::SEND_STATUS, "first status 15 s after begin");
  CHECK(p.tick(SEC(21)).action == AlertPolicy::NONE, "only once");
  CHECK(p.tick(SEC(20 + 3599)).action == AlertPolicy::NONE, "not before the period");
  CHECK(p.tick(SEC(20 + 3600)).action == AlertPolicy::SEND_STATUS, "then every period");
}

static void test_status_and_alerts_are_independent() {
  AlertPolicy p = make(20, 3600, 15);
  p.onVerdict(true, 80, SEC(10));
  CHECK(p.tick(SEC(15)).action == AlertPolicy::SEND_STATUS, "alert does not reset the heartbeat");
}

static void test_survives_millis_wrap() {
  const uint32_t start = 0xFFFFF000u;
  AlertPolicy p;
  p.begin({SEC(20), SEC(3600), SEC(15)}, start);
  CHECK(p.onVerdict(true, 80, start + 100).action == AlertPolicy::SEND_ALERT, "alert near the wrap");
  CHECK(p.onVerdict(true, 80, start + 5000).action == AlertPolicy::NONE, "suppressed across the wrap");
  CHECK(p.onVerdict(true, 80, start + 20100).action == AlertPolicy::SEND_ALERT, "sends after the gap, past the wrap");
  CHECK(p.tick(start + 15000).action == AlertPolicy::SEND_STATUS, "status past the wrap");
}

static void test_parses_the_camera_lines() {
  CameraMsg m;
  CHECK(parseCameraLine("EW,BOOT,812", m) && m.type == CAM_BOOT && m.cam_ms == 812, "boot");
  CHECK(parseCameraLine("EW,FRAME,1450,3", m) && m.type == CAM_FRAME && m.frame == 3, "frame");
  CHECK(parseCameraLine("EW,VERDICT,2100,PERSON,87", m) && m.type == CAM_VERDICT && m.person && m.score == 87, "person");
  CHECK(parseCameraLine("EW,VERDICT,2200,NONE,4", m) && !m.person && m.score == 4, "none");
  CHECK(parseCameraLine("EW,DONE,31000", m) && m.type == CAM_DONE, "done");
}

static void test_rejects_noise_and_malformed() {
  CameraMsg m = {CAM_DONE, 7, 8, true, 9};
  const char* bad[] = {"", "ets Jun  8 2016 00:22:57", "rst:0x1 (POWERON_RESET)", "EW", "EWX,BOOT,1", "EW,BOOT", "EW,BOOT,",
                       "EW,BOOT,12x", "EW,BOOT,12,34", "EW,BOOTS,12", "EW,FRAME,100", "EW,VERDICT,100,MAYBE,50",
                       "EW,VERDICT,100,PERSON,101", "EW,VERDICT,100,PERSON", "EW,BOOT,99999999999"};
  for (const char* line : bad) CHECK(!parseCameraLine(line, m), "should reject: %s", line);
  CHECK(m.cam_ms == 7 && m.frame == 8 && m.score == 9, "a failed parse leaves the output alone");
}

int main() {
  test_person_becomes_an_alert();
  test_none_verdict_never_alerts();
  test_alerts_are_rate_limited();
  test_suppressed_alert_does_not_extend_the_gap();
  test_score_is_capped();
  test_first_status_after_the_delay_then_periodic();
  test_status_and_alerts_are_independent();
  test_survives_millis_wrap();
  test_parses_the_camera_lines();
  test_rejects_noise_and_malformed();
  if (failures == 0) printf("all runner_alert tests passed\n");
  else printf("%d FAILURE(S)\n", failures);
  return failures ? 1 : 0;
}
