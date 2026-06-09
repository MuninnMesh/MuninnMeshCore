#pragma once

#include <Mesh.h>
#include <Arduino.h>
#include <Wire.h>

class AutoDiscoverRTCClock : public mesh::RTCClock {
  mesh::RTCClock* _fallback;
  bool _time_was_set;
  uint32_t _last_set_ms;
  uint32_t _last_set_time;

  bool i2c_probe(TwoWire& wire, uint8_t addr);
public:
  enum Source : uint8_t {
    SOURCE_FALLBACK = 0,
    SOURCE_DS3231,
    SOURCE_RV3028,
    SOURCE_PCF8563,
    SOURCE_RX8130CE
  };

  AutoDiscoverRTCClock(mesh::RTCClock& fallback)
    : _fallback(&fallback), _time_was_set(false), _last_set_ms(0), _last_set_time(0) { }

  void begin(TwoWire& wire);
  bool beginRX8130CE(TwoWire& wire);
  uint32_t getCurrentTime() override;
  void setCurrentTime(uint32_t time) override;
  uint8_t getSource() const;
  const char* getSourceName() const;
  bool hasHardwareRTC() const;
  bool wasTimeSet() const { return _time_was_set; }
  bool isTimeValid();
  bool getLastSetMillis(uint32_t& set_ms) const;
  uint32_t getLastSetTime() const { return _last_set_time; }

  void tick() override {
    _fallback->tick();   // is typically VolatileRTCClock, which now needs tick()
  }
};
