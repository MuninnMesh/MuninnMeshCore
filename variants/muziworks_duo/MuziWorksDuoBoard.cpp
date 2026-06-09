#if defined(MUZIWORKS_DUO) || defined(MUZI_BASE_DUO) || defined(MUZIWORKS_UNO) || defined(MUZI_BASE_UNO)

#include "MuziWorksDuoBoard.h"

#include <Arduino.h>
#include <Wire.h>
#ifdef BLE_PIN_CODE
#include <bluefruit.h>
#endif

#ifdef NRF52_POWER_MANAGEMENT
const PowerMgtConfig power_config = {
  .lpcomp_ain_channel = PWRMGT_LPCOMP_AIN,
  .lpcomp_refsel = PWRMGT_LPCOMP_REFSEL,
  .voltage_bootlock = PWRMGT_VOLTAGE_BOOTLOCK
};

void MuziWorksDuoBoard::initiateShutdown(uint8_t reason) {
  if (reason == SHUTDOWN_REASON_LOW_VOLTAGE ||
      reason == SHUTDOWN_REASON_BOOT_PROTECT) {
    // Low-voltage exits arm LPCOMP voltage wake. User switch shutdown stays in
    // system-off until the external power/switch path wakes the nRF52.
    configureVoltageWake(power_config.lpcomp_ain_channel, power_config.lpcomp_refsel);
  }

#if defined(PIN_SCREEN_ENABLE) && defined(SCREEN_ENABLE_ACTIVE)
  pinMode(PIN_SCREEN_ENABLE, OUTPUT);
  digitalWrite(PIN_SCREEN_ENABLE, !SCREEN_ENABLE_ACTIVE);
#endif

  enterSystemOff(reason);
}
#endif

void MuziWorksDuoBoard::begin() {
  NRF52BoardDCDC::begin();

  pinMode(PIN_VBAT_READ, INPUT);
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(ADC_RESOLUTION);

#ifdef PIN_BATTERY_CHARGING
  pinMode(PIN_BATTERY_CHARGING, INPUT_PULLUP);
#endif
#ifdef PIN_CHARGER_FAULT
  pinMode(PIN_CHARGER_FAULT, INPUT_PULLUP);
#endif

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // LEDs off (active LOW)
  digitalWrite(LED_GREEN, !LED_STATE_ON);
  digitalWrite(LED_BLUE, !LED_STATE_ON);

#if defined(BLE_PIN_CODE) && defined(PIN_MSG_LED) && (PIN_MSG_LED == LED_BLUE)
  Bluefruit.autoConnLed(false);
#endif

#if defined(PIN_WIRE_SDA) && defined(PIN_WIRE_SCL)
  Wire.setPins(PIN_WIRE_SDA, PIN_WIRE_SCL);
#endif

  Wire.begin();

#ifdef NRF52_POWER_MANAGEMENT
  // Run after ADC/reference setup so boot protection uses the same VBAT path as
  // normal battery reporting.
  checkBootVoltage(&power_config);
#endif

  delay(10);
}

#endif
