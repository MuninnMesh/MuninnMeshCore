#include "SH1107Display.h"
#include <Adafruit_GrayOLED.h>
#include "Adafruit_SH110X.h"

bool SH1107Display::i2c_probe(TwoWire &wire, uint8_t addr)
{
  wire.beginTransmission(addr);
  uint8_t error = wire.endTransmission();
  return (error == 0);
}

void SH1107Display::setScreenPower(bool on)
{
#ifdef PIN_SCREEN_ENABLE
  pinMode(PIN_SCREEN_ENABLE, OUTPUT);
  digitalWrite(PIN_SCREEN_ENABLE, on ? _screenPowerActive : !_screenPowerActive);
#endif
}

void SH1107Display::applyFlickerTuning()
{
  static const uint8_t clock_div[] = { SH110X_SETDISPLAYCLOCKDIV, 0xF0 };
  display.oled_commandList(clock_div, sizeof(clock_div));
  display.setContrast(SH1107_CONTRAST);
}

bool SH1107Display::begin()
{
  Wire.begin();

  bool found = false;
  _screenPowerActive = SCREEN_ENABLE_ACTIVE;
  setScreenPower(true);
  delay(SCREEN_ENABLE_SETTLE_MS);

  for (uint8_t i = 0; i < SH1107_I2C_PROBE_ATTEMPTS; i++) {
    if (i2c_probe(Wire, DISPLAY_ADDRESS)) {
      found = true;
      break;
    }
    delay(100);
  }

  if (!found) {
    setScreenPower(false);
    _isOn = false;
    return false;
  }

  if (display.begin(DISPLAY_ADDRESS, true)) {
    _i2cAddress = DISPLAY_ADDRESS;
    _begun = true;
#ifdef DISPLAY_ROTATION
    display.setRotation(DISPLAY_ROTATION);
#endif
    applyFlickerTuning();
    _isOn = true;
    return true;
  }

  setScreenPower(false);
  _begun = false;
  _isOn = false;
  return false;
}

void SH1107Display::turnOn()
{
  if (!_begun) return;

  setScreenPower(true);
  delay(SCREEN_ENABLE_SETTLE_MS);
  display.oled_command(SH110X_DISPLAYON);
  applyFlickerTuning();
  _isOn = true;
}

void SH1107Display::turnOff()
{
  if (_isOn) {
    display.oled_command(SH110X_DISPLAYOFF);
  }
  // Keep the rail asserted for sleep/wake. Cutting panel power requires a full
  // controller init sequence and caused blank-panel restarts on Super IO.
  _isOn = false;
}

void SH1107Display::clear()
{
  display.clearDisplay();
  display.display();
}

void SH1107Display::startFrame(Color bkg)
{
  (void)bkg;
  display.setContrast(SH1107_CONTRAST);
  display.clearDisplay();
  _color = SH110X_WHITE;
  display.setTextColor(_color);
  display.setTextSize(1);
  display.cp437(true); // Use full 256 char 'Code Page 437' font
}

void SH1107Display::setTextSize(int sz)
{
  display.setTextSize(sz);
}

void SH1107Display::setColor(Color c)
{
  _color = (c != 0) ? SH110X_WHITE : SH110X_BLACK;
  display.setTextColor(_color);
}

void SH1107Display::setCursor(int x, int y)
{
  display.setCursor(x, y);
}

void SH1107Display::print(const char *str)
{
  display.print(str);
}

void SH1107Display::fillRect(int x, int y, int w, int h)
{
  display.fillRect(x, y, w, h, _color);
}

void SH1107Display::drawRect(int x, int y, int w, int h)
{
  display.drawRect(x, y, w, h, _color);
}

void SH1107Display::drawLine(int x0, int y0, int x1, int y1)
{
  display.drawLine(x0, y0, x1, y1, _color);
}

void SH1107Display::drawCircle(int x, int y, int r)
{
  display.drawCircle(x, y, r, _color);
}

void SH1107Display::drawXbm(int x, int y, const uint8_t *bits, int w, int h)
{
  display.drawBitmap(x, y, bits, w, h, SH110X_WHITE);
}

uint16_t SH1107Display::getTextWidth(const char *str)
{
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
  return w;
}

void SH1107Display::endFrame()
{
  display.display();
}
