#include "TrackerMotion.h"
#include <target.h>

#if defined(T1000_E) && defined(HAS_QMA6100P) && defined(QMA_6100P_INT_PIN)

#include <Wire.h>

// QMA6100P registers (QST datasheet Rev A1)
#define QMA_CHIP_ID      0x00   // reads 0x90
#define QMA_DATA_X_L     0x01   // 6 bytes: x lo/hi, y lo/hi, z lo/hi (14-bit, left aligned)
#define QMA_INT_ST0      0x09
#define QMA_FSR          0x0F   // full scale range
#define QMA_PM           0x11   // bit 7: active mode
#define QMA_INT_EN2      0x18   // bits 0-2: any-motion enable X/Y/Z
#define QMA_INT_MAP1     0x1A   // bit 0: INT1_ANY_MOT
#define QMA_INTPIN_CONF  0x20   // bit 0: INT1 active high; bit 1: INT1 open drain
#define QMA_INT_CFG      0x21   // bit 0: latch interrupts
#define QMA_MOT_CONF0    0x2C   // bits 1:0: any-motion duration (samples - 1)
#define QMA_MOT_CONF2    0x2E   // any-motion threshold, units of 16 LSB of slope
#define QMA_SOFT_RESET   0x36

static uint8_t qma_addr = 0;
static bool qma_ready = false;
static volatile bool motion_flag = false;
static volatile uint32_t irq_count = 0;

static bool qma_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(qma_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool qma_read(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(qma_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom(qma_addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static void motion_isr() {
  motion_flag = true;
  irq_count++;
}

bool tracker_motion_begin(uint8_t threshold) {
#ifdef PIN_3V3_ACC_EN
  pinMode(PIN_3V3_ACC_EN, OUTPUT);
  digitalWrite(PIN_3V3_ACC_EN, HIGH);   // sensor rail
  delay(20);                            // datasheet: 10 ms power-on startup
#endif

  // address depends on the AD0 strap: 0x12 (GND) or 0x13 (VDD)
  qma_ready = false;
  uint8_t id = 0;
  for (uint8_t addr = 0x12; addr <= 0x13; addr++) {
    qma_addr = addr;
    if (qma_read(QMA_CHIP_ID, &id, 1) && id == 0x90) { qma_ready = true; break; }
  }
  if (!qma_ready) return false;

  qma_write(QMA_SOFT_RESET, 0xB6);      // known register state
  delay(5);
  qma_write(QMA_SOFT_RESET, 0x00);
  delay(10);

  qma_write(QMA_FSR, 0x01);             // +-2g
  qma_write(QMA_PM, 0x80);              // active mode (default clock)
  qma_write(QMA_MOT_CONF0, 0x01);       // 2 consecutive samples over threshold
  qma_write(QMA_MOT_CONF2, threshold);
  qma_write(QMA_INTPIN_CONF, 0x05);     // INT1 push-pull, active high (reset default)
  qma_write(QMA_INT_CFG, 0x00);         // non-latched: INT1 follows the any-motion condition
  qma_write(QMA_INT_MAP1, 0x01);        // any-motion -> INT1
  qma_write(QMA_INT_EN2, 0x07);         // any-motion on X, Y and Z

  pinMode(QMA_6100P_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(QMA_6100P_INT_PIN), motion_isr, RISING);
  motion_flag = false;
  return true;
}

bool tracker_motion_available() { return qma_ready; }

void tracker_motion_set_threshold(uint8_t threshold) {
  if (qma_ready) qma_write(QMA_MOT_CONF2, threshold);
}

bool tracker_motion_poll() {
  if (!motion_flag) return false;
  motion_flag = false;
  return true;
}

uint32_t tracker_motion_irq_count() { return irq_count; }

void tracker_motion_dump(Print& out) {
  if (!qma_ready) { out.println("accel: not found"); return; }
  uint8_t id = 0, d[6] = {0}, st = 0, pm = 0, th = 0, en = 0, map = 0;
  qma_read(QMA_CHIP_ID, &id, 1);
  qma_read(QMA_DATA_X_L, d, 6);
  qma_read(QMA_INT_ST0, &st, 1);
  qma_read(QMA_PM, &pm, 1);
  qma_read(QMA_MOT_CONF2, &th, 1);
  qma_read(QMA_INT_EN2, &en, 1);
  qma_read(QMA_INT_MAP1, &map, 1);
  // raw counts for bring-up only (scale not calibrated); the any-motion detector works on the sensor's own slope
  int16_t x = ((int16_t)((d[1] << 8) | d[0])) >> 2;
  int16_t y = ((int16_t)((d[3] << 8) | d[2])) >> 2;
  int16_t z = ((int16_t)((d[5] << 8) | d[4])) >> 2;
  out.printf("accel: addr=0x%02X id=0x%02X pm=0x%02X int_en=0x%02X map=0x%02X thr=%u st0=0x%02X\n",
             qma_addr, id, pm, en, map, th, st);
  out.printf("accel: x=%d y=%d z=%d (raw counts) irq_count=%lu pin=%d\n",
             x, y, z, (unsigned long)irq_count, digitalRead(QMA_6100P_INT_PIN));
}

#else   // no accelerometer on this board

bool tracker_motion_begin(uint8_t) { return false; }
bool tracker_motion_available() { return false; }
void tracker_motion_set_threshold(uint8_t) { }
bool tracker_motion_poll() { return false; }
uint32_t tracker_motion_irq_count() { return 0; }
void tracker_motion_dump(Print& out) { out.println("accel: not available on this board"); }

#endif
