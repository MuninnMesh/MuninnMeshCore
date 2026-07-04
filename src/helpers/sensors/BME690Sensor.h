#pragma once

// Minimal BME690 (Bosch BME69x) I2C driver.
//
// The BME690 uses a different compensation model than the BME680/BME688: 11
// pressure coefficients, 6 humidity coefficients, a 24-bit pressure/temperature
// ADC, and a different calibration register layout. The Adafruit/Bosch BME680
// library decodes a BME690 incorrectly, so this sensor needs a dedicated path.

#include <Arduino.h>
#include <Wire.h>

#ifndef BME690_HEATER_DURATION_MS
  #define BME690_HEATER_DURATION_MS 150
#endif

#ifndef BME690_SAMPLE_INTERVAL_MS
  #define BME690_SAMPLE_INTERVAL_MS 5000
#endif

class BME690Sensor {
public:
  struct Sample {
    float temperature_c;
    float pressure_hpa;
    float humidity_pct;
    float gas_resistance_ohm;
    bool  gas_valid;
    bool  heat_stable;
  };

  bool begin(TwoWire* wire, uint8_t addr) {
    _wire = wire;
    _addr = addr;

    writeReg(REG_RESET, SOFT_RESET_CMD);
    delay(10);

    uint8_t id = 0;
    if (!readReg(REG_CHIP_ID, id) || id != CHIP_ID_BME69X) return false;

    readReg(REG_VARIANT_ID, _variant_id);

    if (!readCalibration()) return false;

    writeReg(REG_CTRL_HUM, OSRS_2X);
    writeReg(REG_CONFIG, IIR_3 << 2);
    configureHeater(_heater_temp_c, _heater_dur_ms);
    startConversion();
    return true;
  }

  bool sample(Sample& out) {
    writeReg(REG_CTRL_HUM, OSRS_2X);
    uint8_t ctrl_meas = (OSRS_4X << 5) | (OSRS_4X << 2) | MODE_FORCED;
    writeReg(REG_CTRL_MEAS, ctrl_meas);

    uint32_t wait_ms = 25 + _heater_dur_ms;
    uint32_t start = millis();
    for (;;) {
      delay(10);
      uint8_t status = 0;
      if (!readReg(REG_MEAS_STATUS_0, status)) return false;
      bool new_data = (status & 0x80) != 0;
      bool measuring = (status & 0x60) != 0;
      if (new_data && !measuring) break;
      if (millis() - start > wait_ms + 200) return false;
    }

    uint8_t buf[LEN_FIELD];
    if (!readRegs(REG_PRESS_MSB, buf, LEN_FIELD)) return false;

    uint32_t adc_p = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    uint32_t adc_t = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    uint32_t adc_h = ((uint32_t)buf[6] << 8) | buf[7];
    uint8_t gas_lsb = buf[14];
    uint16_t adc_g = ((uint16_t)buf[13] << 2) | (gas_lsb >> 6);
    uint8_t gas_range = gas_lsb & 0x0F;

    double temp_c = calcTemp(adc_t);
    _amb_temp = (float)temp_c;

    out.temperature_c = (float)temp_c;
    out.pressure_hpa = (float)(calcPressure(adc_p) / 100.0);
    out.humidity_pct = (float)calcHumidity(adc_h);
    out.gas_resistance_ohm = (float)calcGasRes(adc_g, gas_range);
    out.gas_valid = (gas_lsb & 0x20) != 0;
    out.heat_stable = (gas_lsb & 0x10) != 0;
    return true;
  }

  void update() {
    if (!_wire) return;

    uint32_t now = millis();
    if (_converting) {
      if ((int32_t)(now - _ready_after_ms) < 0) {
        return;
      }

      uint8_t status = 0;
      if (!readReg(REG_MEAS_STATUS_0, status)) {
        finishConversion(false);
        return;
      }

      bool new_data = (status & 0x80) != 0;
      bool measuring = (status & 0x60) != 0;
      if (!new_data || measuring) {
        if ((int32_t)(now - _timeout_at_ms) >= 0) {
          finishConversion(false);
        }
        return;
      }

      Sample next;
      bool ok = readMeasurement(next);
      finishConversion(ok);
      if (ok) {
        _last_sample = next;
      }
      return;
    }

    if (!_has_sample || (int32_t)(now - _next_start_ms) >= 0) {
      startConversion();
    }
  }

  bool readCached(Sample& out) const {
    if (!_has_sample) return false;
    out = _last_sample;
    return true;
  }

private:
  static const uint8_t REG_CHIP_ID       = 0xD0;
  static const uint8_t REG_RESET         = 0xE0;
  static const uint8_t REG_VARIANT_ID    = 0xF0;
  static const uint8_t REG_CONFIG        = 0x75;
  static const uint8_t REG_CTRL_MEAS     = 0x74;
  static const uint8_t REG_CTRL_HUM      = 0x72;
  static const uint8_t REG_CTRL_GAS_1    = 0x71;
  static const uint8_t REG_CTRL_GAS_0    = 0x70;
  static const uint8_t REG_GAS_WAIT_0    = 0x64;
  static const uint8_t REG_RES_HEAT_0    = 0x5A;
  static const uint8_t REG_MEAS_STATUS_0 = 0x1D;
  static const uint8_t REG_PRESS_MSB     = 0x1F;
  static const uint8_t CHIP_ID_BME69X    = 0x61;
  static const uint8_t SOFT_RESET_CMD    = 0xB6;
  static const uint8_t OSRS_4X           = 0x03;
  static const uint8_t OSRS_2X           = 0x02;
  static const uint8_t IIR_3             = 0x02;
  static const uint8_t MODE_FORCED       = 0x01;
  static const uint8_t NB_CONV_0         = 0x00;
  static const size_t  LEN_FIELD         = 15;

  TwoWire* _wire = nullptr;
  uint8_t  _addr = 0x76;
  uint8_t  _variant_id = 0;
  double   _temp_c = 0.0;
  uint16_t _heater_temp_c = 320;
  uint16_t _heater_dur_ms = BME690_HEATER_DURATION_MS;
  float    _amb_temp = 25.0f;
  bool     _converting = false;
  bool     _has_sample = false;
  uint32_t _ready_after_ms = 0;
  uint32_t _timeout_at_ms = 0;
  uint32_t _next_start_ms = 0;
  Sample   _last_sample{};

  struct Calib {
    uint16_t t1; uint16_t t2; int8_t t3;
    int16_t p1; uint16_t p2; int8_t p3; int8_t p4; int16_t p5; int16_t p6;
    int8_t p7; int8_t p8; int16_t p9; int8_t p10; int8_t p11;
    int16_t h1; int8_t h2; uint8_t h3; int8_t h4; int16_t h5; uint8_t h6;
    int8_t g1; int16_t g2; int8_t g3;
    uint8_t res_heat_range; int8_t res_heat_val; int8_t range_sw_err;
  } _c{};

  double calcTemp(uint32_t adc_t) {
    int32_t do1 = (int32_t)_c.t1 << 8;
    double dtk1 = (double)_c.t2 / (double)(1ULL << 30);
    double dtk2 = (double)_c.t3 / (double)(1ULL << 48);
    double cf = (double)adc_t - (double)do1;
    _temp_c = cf * dtk1 + cf * cf * dtk2;
    return _temp_c;
  }

  double calcPressure(uint32_t adc_p) {
    double t = _temp_c;
    double o = (double)((uint32_t)_c.p1 << 3);
    double tk10 = (double)_c.p2 / (double)(1ULL << 6);
    double tk20 = (double)_c.p3 / (double)(1ULL << 8);
    double tk30 = (double)_c.p4 / (double)(1ULL << 15);
    double s = ((double)_c.p5 - (double)(1ULL << 14)) / (double)(1ULL << 20);
    double tk1s = ((double)_c.p6 - (double)(1ULL << 14)) / (double)(1ULL << 29);
    double tk2s = (double)_c.p7 / (double)(1ULL << 32);
    double tk3s = (double)_c.p8 / (double)(1ULL << 37);
    double nls = (double)_c.p9 / (double)(1ULL << 48);
    double tknls = (double)_c.p10 / (double)(1ULL << 48);
    double nls3 = (double)_c.p11 / ((double)(1ULL << 35) * (double)(1ULL << 30));
    double pa = (double)adc_p;
    double tmp1 = o + tk10 * t + tk20 * t * t + tk30 * t * t * t;
    double tmp2 = pa * (s + tk1s * t + tk2s * t * t + tk3s * t * t * t);
    double tmp3 = pa * pa * (nls + tknls * t);
    double tmp4 = pa * pa * pa * nls3;
    double press = tmp1 + tmp2 + tmp3 + tmp4;
    return press <= 0.0 ? 0.0 : press;
  }

  double calcHumidity(uint32_t adc_h) {
    double t = _temp_c;
    double temp_comp = t * 5120.0 - 76800.0;
    double oh = (double)_c.h1 * (double)(1ULL << 6);
    double sh = (double)_c.h5 / (double)(1ULL << 16);
    double tk10h = (double)_c.h2 / (double)(1ULL << 14);
    double tk1sh = (double)_c.h4 / (double)(1ULL << 26);
    double tk2sh = (double)_c.h3 / (double)(1ULL << 26);
    double hlin2 = (double)_c.h6 / (double)(1ULL << 19);
    double hoff = (double)adc_h - (oh + tk10h * temp_comp);
    double hsens = hoff * sh * (1.0 + tk1sh * temp_comp + tk1sh * tk2sh * temp_comp * temp_comp);
    double hum = hsens * (1.0 - hlin2 * hsens);
    if (hum < 0.0) hum = 0.0;
    if (hum > 100.0) hum = 100.0;
    return hum;
  }

  double calcGasRes(uint16_t adc_g, uint8_t gas_range) {
    if (gas_range > 15) return 0.0;
    double var1 = 262144.0 / (double)(1u << gas_range);
    double var2 = ((double)adc_g - 512.0) * 3.0 + 4096.0;
    return 1000000.0 * var1 / var2;
  }

  void configureHeater(uint16_t temp_c, uint16_t dur_ms) {
    _heater_temp_c = temp_c;
    _heater_dur_ms = dur_ms;
    writeReg(REG_RES_HEAT_0, calcResHeat(temp_c));
    writeReg(REG_GAS_WAIT_0, calcGasWait(dur_ms));
    writeReg(REG_CTRL_GAS_0, NB_CONV_0);
    const uint8_t RUN_GAS = 0x20;
    writeReg(REG_CTRL_GAS_1, RUN_GAS | NB_CONV_0);
  }

  uint8_t calcResHeat(uint16_t temp_c) {
    // Heater DAC transfer per Bosch BME690_SensorAPI (bme69x.c calc_res_heat,
    // integer version). The BME690 formula differs from the BME680/688 one;
    // using the older formula programs a wrong setpoint, the gas plate rails
    // at ADC max, and the resistance reads back as a constant bogus value
    // with the valid/heat-stable bits still set.
    int32_t temp = (temp_c > 400) ? 400 : temp_c;
    int32_t amb = (int32_t)_amb_temp;
    int32_t var1 = ((amb * (int32_t)_c.g3) / 1000) * 256;
    int64_t var2 = ((int64_t)_c.g1 + 784) *
                   ((((((int64_t)_c.g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    int64_t var3 = (int64_t)var1 + (var2 / 2);
    int64_t var4 = var3 / ((int32_t)_c.res_heat_range + 4);
    int32_t var5 = (131 * (int32_t)_c.res_heat_val) + 65536;
    int32_t heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    int32_t heatr_res = (heatr_res_x100 + 50) / 100;
    if (heatr_res < 0) heatr_res = 0;
    if (heatr_res > 255) heatr_res = 255;
    return (uint8_t)heatr_res;
  }

  uint8_t calcGasWait(uint16_t dur_ms) {
    uint32_t d = dur_ms > 4032 ? 4032 : dur_ms;
    if (d < 64) return (uint8_t)d;
    if (d < 256) return (uint8_t)(0x40 | (d / 4));
    if (d < 1024) return (uint8_t)(0x80 | (d / 16));
    return (uint8_t)(0xC0 | (d / 64));
  }

  bool readCalibration() {
    uint8_t a[42];
    if (!readRegs(0x8A, &a[0], 23)) return false;
    if (!readRegs(0xE1, &a[23], 14)) return false;
    if (!readRegs(0x00, &a[37], 5)) return false;

    auto cc = [&](int msb, int lsb) -> uint16_t {
      return ((uint16_t)a[msb] << 8) | a[lsb];
    };

    _c.t1 = cc(32, 31);
    _c.t2 = cc(1, 0);
    _c.t3 = (int8_t)a[2];

    _c.p5 = (int16_t)cc(5, 4);
    _c.p6 = (int16_t)cc(7, 6);
    _c.p7 = (int8_t)a[8];
    _c.p8 = (int8_t)a[9];
    _c.p1 = (int16_t)cc(11, 10);
    _c.p2 = cc(13, 12);
    _c.p3 = (int8_t)a[14];
    _c.p4 = (int8_t)a[15];
    _c.p9 = (int16_t)cc(19, 18);
    _c.p10 = (int8_t)a[20];
    _c.p11 = (int8_t)a[21];

    int16_t h5 = ((int16_t)a[23] << 4) | (a[24] >> 4);
    if (h5 > 2047) h5 -= 4096;
    _c.h5 = h5;
    int16_t h1 = ((int16_t)a[25] << 4) | (a[24] & 0x0F);
    if (h1 > 2047) h1 -= 4096;
    _c.h1 = h1;
    _c.h2 = (int8_t)a[26];
    _c.h4 = (int8_t)a[27];
    _c.h3 = a[28];
    _c.h6 = a[29];

    _c.g1 = (int8_t)a[35];
    _c.g2 = (int16_t)cc(34, 33);
    _c.g3 = (int8_t)a[36];
    _c.res_heat_val = (int8_t)a[37];
    _c.res_heat_range = (a[39] & 0x30) >> 4;
    _c.range_sw_err = ((int8_t)(a[41] & 0xF0)) / 16;
    return true;
  }

  bool startConversion() {
    if (!_wire) return false;
    if (!writeReg(REG_CTRL_HUM, OSRS_2X)) return false;
    uint8_t ctrl_meas = (OSRS_4X << 5) | (OSRS_4X << 2) | MODE_FORCED;
    if (!writeReg(REG_CTRL_MEAS, ctrl_meas)) return false;

    uint32_t now = millis();
    _ready_after_ms = now + 25 + _heater_dur_ms;
    _timeout_at_ms = _ready_after_ms + 200;
    _converting = true;
    return true;
  }

  void finishConversion(bool ok) {
    _converting = false;
    if (ok) {
      _has_sample = true;
    }
    _next_start_ms = millis() + BME690_SAMPLE_INTERVAL_MS;
  }

  bool readMeasurement(Sample& out) {
    uint8_t buf[LEN_FIELD];
    if (!readRegs(REG_PRESS_MSB, buf, LEN_FIELD)) return false;

    uint32_t adc_p = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    uint32_t adc_t = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    uint32_t adc_h = ((uint32_t)buf[6] << 8) | buf[7];
    uint8_t gas_lsb = buf[14];
    uint16_t adc_g = ((uint16_t)buf[13] << 2) | (gas_lsb >> 6);
    uint8_t gas_range = gas_lsb & 0x0F;

    double temp_c = calcTemp(adc_t);
    _amb_temp = (float)temp_c;

    out.temperature_c = (float)temp_c;
    out.pressure_hpa = (float)(calcPressure(adc_p) / 100.0);
    out.humidity_pct = (float)calcHumidity(adc_h);
    out.gas_resistance_ohm = (float)calcGasRes(adc_g, gas_range);
    out.gas_valid = (gas_lsb & 0x20) != 0;
    out.heat_stable = (gas_lsb & 0x10) != 0;
    return true;
  }

  bool writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t& val) {
    return readRegs(reg, &val, 1);
  }

  bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    size_t got = _wire->requestFrom((int)_addr, (int)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = _wire->read();
    return true;
  }
};
