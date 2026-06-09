#include "AutoDiscoverRTCClock.h"
#include "RTClib.h"
#include <Melopero_RV3028.h>
#include "RTC_RX8130CE.h"

static RTC_DS3231 rtc_3231;
static bool ds3231_success = false;

static Melopero_RV3028 rtc_rv3028;
static bool rv3028_success = false;

static RTC_PCF8563 rtc_8563;
static bool rtc_8563_success = false;

static RTC_RX8130CE rtc_8130;
static bool rtc_8130_success = false;

#define DS3231_ADDRESS   0x68
#define RV3028_ADDRESS   0x52
#define PCF8563_ADDRESS  0x51
#define RX8130CE_ADDRESS 0x32
// 2024-05-15. Times older than this are treated as unset/stale for display and
// diagnostics, while still allowing GPS/network time to repair the RTC later.
#define RTC_MIN_VALID_TIME 1715770351UL

bool AutoDiscoverRTCClock::i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

uint8_t AutoDiscoverRTCClock::getSource() const {
  if (ds3231_success) return SOURCE_DS3231;
  if (rv3028_success) return SOURCE_RV3028;
  if (rtc_8563_success) return SOURCE_PCF8563;
  if (rtc_8130_success) return SOURCE_RX8130CE;
  return SOURCE_FALLBACK;
}

const char* AutoDiscoverRTCClock::getSourceName() const {
  switch (getSource()) {
    case SOURCE_DS3231: return "DS3231";
    case SOURCE_RV3028: return "RV3028";
    case SOURCE_PCF8563: return "PCF8563";
    case SOURCE_RX8130CE: return "RX8130CE";
    case SOURCE_FALLBACK:
    default: return "volatile";
  }
}

bool AutoDiscoverRTCClock::hasHardwareRTC() const {
  return getSource() != SOURCE_FALLBACK;
}

bool AutoDiscoverRTCClock::isTimeValid() {
  uint32_t time = getCurrentTime();
  return time >= RTC_MIN_VALID_TIME;
}

bool AutoDiscoverRTCClock::getLastSetMillis(uint32_t& set_ms) const {
  if (!_time_was_set) return false;
  set_ms = _last_set_ms;
  return true;
}

void AutoDiscoverRTCClock::begin(TwoWire& wire) {
  #if !defined(DISABLE_DS3231_PROBE)
  if (i2c_probe(wire, DS3231_ADDRESS)) {
    ds3231_success = rtc_3231.begin(&wire);
  }
  #endif

  if (i2c_probe(wire, RV3028_ADDRESS)) {
    rtc_rv3028.initI2C(wire);
    rtc_rv3028.writeToRegister(0x35, 0x00);
    // Direct Switching Mode: when VDD drops below VBACKUP, the RV3028 switches
    // to backup supply without firmware intervention.
    rtc_rv3028.writeToRegister(0x37, 0xB4);
    rtc_rv3028.set24HourMode();
    rv3028_success = true;
  }

  if (i2c_probe(wire, PCF8563_ADDRESS)) {
    rtc_8563_success = rtc_8563.begin(&wire);
  }

  if (i2c_probe(wire, RX8130CE_ADDRESS)) {
    MESH_DEBUG_PRINTLN("RX8130CE: Found");
    rtc_8130.begin(&wire);
    rtc_8130_success = true;
    MESH_DEBUG_PRINTLN("RX8130CE: Initialized");
  }
}

bool AutoDiscoverRTCClock::beginRX8130CE(TwoWire& wire) {
  if (!i2c_probe(wire, RX8130CE_ADDRESS)) {
    return false;
  }

  MESH_DEBUG_PRINTLN("RX8130CE: Found");
  rtc_8130.begin(&wire);
  rtc_8130_success = true;
  MESH_DEBUG_PRINTLN("RX8130CE: Initialized");
  return true;
}

uint32_t AutoDiscoverRTCClock::getCurrentTime() {
  if (ds3231_success) {
    return rtc_3231.now().unixtime();
  }

  if (rv3028_success) {
    return DateTime(
        rtc_rv3028.getYear(),
        rtc_rv3028.getMonth(),
        rtc_rv3028.getDate(),
        rtc_rv3028.getHour(),
        rtc_rv3028.getMinute(),
        rtc_rv3028.getSecond()
    ).unixtime();
  }

  if (rtc_8563_success) {
    return rtc_8563.now().unixtime();
  }

  if (rtc_8130_success) {
    return rtc_8130.now().unixtime();
  }

  return _fallback->getCurrentTime();
}

void AutoDiscoverRTCClock::setCurrentTime(uint32_t time) { 
  if (ds3231_success) {
    rtc_3231.adjust(DateTime(time));
  } else if (rv3028_success) {
    auto dt = DateTime(time);
    uint8_t weekday = dt.dayOfTheWeek();
    rtc_rv3028.setTime(dt.year(), dt.month(), weekday, dt.day(), dt.hour(), dt.minute(), dt.second());
  } else if (rtc_8563_success) {
    rtc_8563.adjust(DateTime(time));
  } else if (rtc_8130_success) {
    MESH_DEBUG_PRINTLN("RX8130CE: Setting time");
    rtc_8130.adjust(DateTime(time));
  } else {
    _fallback->setCurrentTime(time);
  }
  _time_was_set = true;
  _last_set_ms = millis();
  _last_set_time = time;
}
