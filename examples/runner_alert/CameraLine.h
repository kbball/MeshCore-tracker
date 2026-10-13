#pragma once
// Parser for the lines the early-warning camera prints on its UART. Plain C++, no Arduino headers, so it is
// unit tested on the host (tests/alert_policy_test.cpp).
//
// Copied from the runner-early-warning firmware repo (common/lib/CameraMsg/CameraMsg.h, parse half only);
// keep the two in sync. The camera also prints ESP32 boot text on the same UART, so every real message starts
// with "EW," and anything else is ignored.
//
//   EW,BOOT,<ms>
//   EW,FRAME,<ms>,<n>
//   EW,VERDICT,<ms>,PERSON|NONE,<score 0..100>
//   EW,DONE,<ms>

#include <stdint.h>

namespace runner {

enum CameraMsgType : uint8_t { CAM_BOOT, CAM_FRAME, CAM_VERDICT, CAM_DONE };

struct CameraMsg {
  CameraMsgType type;
  uint32_t cam_ms;   // camera millis()
  uint32_t frame;    // CAM_FRAME: index
  bool person;       // CAM_VERDICT: PERSON
  uint32_t score;    // CAM_VERDICT: 0..100
};

namespace detail {

// Reads a decimal number up to the next ',' or the end of the string, advancing s. False if empty or not digits.
inline bool takeNumber(const char*& s, uint32_t& out) {
  if (*s < '0' || *s > '9') return false;
  uint64_t v = 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (uint64_t)(*s - '0');
    if (v > 0xFFFFFFFFull) return false;
    s++;
  }
  if (*s == ',') s++;
  else if (*s != '\0') return false;
  out = (uint32_t)v;
  return true;
}

// True if s starts with word followed by ',' or the end; advances s past the word and the comma.
inline bool takeWord(const char*& s, const char* word) {
  const char* p = s;
  while (*word) {
    if (*p != *word) return false;
    p++;
    word++;
  }
  if (*p == ',') p++;
  else if (*p != '\0') return false;
  s = p;
  return true;
}

}  // namespace detail

// Parses one line without its newline. False for noise or a malformed message; out is untouched then.
inline bool parseCameraLine(const char* line, CameraMsg& out) {
  const char* s = line;
  if (!detail::takeWord(s, "EW")) return false;
  CameraMsg m = {CAM_BOOT, 0, 0, false, 0};
  if (detail::takeWord(s, "BOOT")) {
    m.type = CAM_BOOT;
    if (!detail::takeNumber(s, m.cam_ms) || *s != '\0') return false;
  } else if (detail::takeWord(s, "FRAME")) {
    m.type = CAM_FRAME;
    if (!detail::takeNumber(s, m.cam_ms) || !detail::takeNumber(s, m.frame) || *s != '\0') return false;
  } else if (detail::takeWord(s, "VERDICT")) {
    m.type = CAM_VERDICT;
    if (!detail::takeNumber(s, m.cam_ms)) return false;
    if (detail::takeWord(s, "PERSON")) m.person = true;
    else if (detail::takeWord(s, "NONE")) m.person = false;
    else return false;
    if (!detail::takeNumber(s, m.score) || *s != '\0' || m.score > 100) return false;
  } else if (detail::takeWord(s, "DONE")) {
    m.type = CAM_DONE;
    if (!detail::takeNumber(s, m.cam_ms) || *s != '\0') return false;
  } else {
    return false;
  }
  out = m;
  return true;
}

}  // namespace runner
