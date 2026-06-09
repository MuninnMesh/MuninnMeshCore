#pragma once

#include "DisplayDriver.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#define SH110X_NO_SPLASH
#include <Adafruit_SH110X.h>

#ifndef PIN_OLED_RESET
#define PIN_OLED_RESET -1
#endif

#ifndef DISPLAY_ADDRESS
#define DISPLAY_ADDRESS 0x3C
#endif

#ifndef SCREEN_ENABLE_ACTIVE
#define SCREEN_ENABLE_ACTIVE HIGH
#endif

#ifndef SCREEN_ENABLE_SETTLE_MS
#define SCREEN_ENABLE_SETTLE_MS 250
#endif

#ifndef SH1107_I2C_PROBE_ATTEMPTS
#define SH1107_I2C_PROBE_ATTEMPTS 3
#endif

#ifndef SH1107_CONTRAST
#define SH1107_CONTRAST 120
#endif

class SH1107Display : public DisplayDriver
{
  Adafruit_SH1107 display;
  bool _isOn;
  bool _begun;
  uint8_t _i2cAddress;
  uint8_t _screenPowerActive;
  uint8_t _color;

  bool i2c_probe(TwoWire &wire, uint8_t addr);
  void setScreenPower(bool on);
  void applyFlickerTuning();

public:
  SH1107Display() : DisplayDriver(128, 128), display(128, 128, &Wire, PIN_OLED_RESET) {
    _isOn = false;
    _begun = false;
    _i2cAddress = DISPLAY_ADDRESS;
    _screenPowerActive = SCREEN_ENABLE_ACTIVE;
  }
  bool begin();

  bool isOn() override { return _isOn; }
  void turnOn() override;
  void turnOff() override;
  void clear() override;
  void startFrame(Color bkg = DARK) override;
  void setTextSize(int sz) override;
  void setColor(Color c) override;
  void setCursor(int x, int y) override;
  void print(const char *str) override;
  void fillRect(int x, int y, int w, int h) override;
  void drawRect(int x, int y, int w, int h) override;
  void drawLine(int x0, int y0, int x1, int y1) override;
  void drawCircle(int x, int y, int r) override;
  void drawXbm(int x, int y, const uint8_t *bits, int w, int h) override;
  uint16_t getTextWidth(const char *str) override;
  void endFrame() override;
};
