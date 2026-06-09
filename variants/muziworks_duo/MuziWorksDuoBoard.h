#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRF52Board.h>

#if defined(MUZIWORKS_DUO) || defined(MUZI_BASE_DUO) || defined(MUZIWORKS_UNO) || defined(MUZI_BASE_UNO)

#define PIN_VBAT_READ       BATTERY_PIN
#define BATTERY_SAMPLES     8

class MuziWorksDuoBoard : public NRF52BoardDCDC {
protected:
#ifdef NRF52_POWER_MANAGEMENT
  void initiateShutdown(uint8_t reason) override;
#endif

public:
  MuziWorksDuoBoard() : NRF52Board(
#if defined(MUZIWORKS_UNO) || defined(MUZI_BASE_UNO)
    "MuziBaseUno_OTA"
#else
    "MuziBaseDuo_OTA"
#endif
  ) {}
  void begin();

#ifdef NRF52_POWER_MANAGEMENT
  void powerOff() override {
    initiateShutdown(SHUTDOWN_REASON_USER);
    reboot();
  }
#endif

  uint16_t getBattMilliVolts() override {
    uint32_t raw = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / BATTERY_SAMPLES;
    // raw / 2^ADC_RESOLUTION gives the ADC fraction of AREF. ADC_MULTIPLIER
    // compensates the board's VBAT divider back to pack voltage.
    return (uint16_t)((raw * ADC_MULTIPLIER * AREF_VOLTAGE * 1000.0F) / (1 << ADC_RESOLUTION));
  }

  bool hasBatteryChargingStatus() const {
#ifdef PIN_BATTERY_CHARGING
    return true;
#else
    return false;
#endif
  }

  bool isBatteryCharging() const {
#ifdef PIN_BATTERY_CHARGING
    // Charger status pins are wired active-low on the MuziWorks variants.
    return digitalRead(PIN_BATTERY_CHARGING) == LOW;
#else
    return false;
#endif
  }

  bool hasChargerFaultStatus() const {
#ifdef PIN_CHARGER_FAULT
    return true;
#else
    return false;
#endif
  }

  bool isChargerFaultActive() const {
#ifdef PIN_CHARGER_FAULT
    // Keep fault as a separate diagnostic from charging so the UI can show
    // "external power present, charger fault" instead of a false not-charging.
    return digitalRead(PIN_CHARGER_FAULT) == LOW;
#else
    return false;
#endif
  }

  const char* getManufacturerName() const override {
#if defined(MUZIWORKS_UNO) || defined(MUZI_BASE_UNO)
    return "Muzi Base Uno";
#else
    return "Muzi Base Duo";
#endif
  }

#if defined(LED_GREEN) && !defined(MUZIWORKS_DUO_SUPER_IO)
  void onBeforeTransmit() override {
    digitalWrite(LED_GREEN, LED_STATE_ON);
  }
  void onAfterTransmit() override {
    digitalWrite(LED_GREEN, !LED_STATE_ON);
  }
#endif
};

#endif
