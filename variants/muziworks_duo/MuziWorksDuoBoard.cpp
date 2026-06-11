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

#ifndef PIN_GPS_EN_ACTIVE
#define PIN_GPS_EN_ACTIVE HIGH   // matches MicroNMEALocationProvider default
#endif

void MuziWorksDuoBoard::initiateShutdown(uint8_t reason) {
  if (reason == SHUTDOWN_REASON_LOW_VOLTAGE ||
      reason == SHUTDOWN_REASON_BOOT_PROTECT) {
    // Low-voltage exits arm LPCOMP voltage wake. User switch shutdown stays in
    // system-off until the external power/switch path wakes the nRF52.
    configureVoltageWake(power_config.lpcomp_ain_channel, power_config.lpcomp_refsel);
  }

  // nRF52 System OFF retains GPIO output and pull configuration, so every
  // power-relevant pin must be forced to its lowest-power state here. This
  // board hook is the catch-all for ALL shutdown paths (user hibernate, boot
  // protect, low voltage) regardless of what the app layer did first.

#if defined(PIN_SCREEN_ENABLE) && defined(SCREEN_ENABLE_ACTIVE)
  pinMode(PIN_SCREEN_ENABLE, OUTPUT);
  digitalWrite(PIN_SCREEN_ENABLE, !SCREEN_ENABLE_ACTIVE);
#endif

#if defined(PIN_GPS_EN) && (PIN_GPS_EN >= 0)
  // GPS rail: Smart GPS may have the receiver powered (e.g. hibernating right
  // after a trip with the switch at ON-GPS) — ~25-40mA retained in SYSTEMOFF
  // would drain the pack in about a day.
  pinMode(PIN_GPS_EN, OUTPUT);
  digitalWrite(PIN_GPS_EN, !PIN_GPS_EN_ACTIVE);
#endif

#if defined(PIN_GPS_TX) && (PIN_GPS_TX >= 0)
  // Stop the GPS UART and float its pins so TX doesn't idle HIGH into the
  // now-unpowered GNSS module (back-powering risk through the RX ESD path).
  Serial1.end();
  pinMode(PIN_GPS_RX, INPUT);
  pinMode(PIN_GPS_TX, INPUT);
#endif

#ifdef PIN_ACTIVE_BUZZER
  // If shutdown lands mid-beep the buzzer pin would be retained driven-on.
  pinMode(PIN_ACTIVE_BUZZER, OUTPUT);
  digitalWrite(PIN_ACTIVE_BUZZER, !ACTIVE_BUZZER_ON);
#elif defined(PIN_BUZZER)
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
#endif

#if defined(SWITCH_MODE1) && defined(SWITCH_MODE2)
  // The slide-switch inputs use internal pullups; detents that hold a pin low
  // leak VDD/13k (~250uA each) for the whole time the device is "off".
  pinMode(SWITCH_MODE1, INPUT);
  pinMode(SWITCH_MODE2, INPUT);
#endif

  // Force both status LEDs off in case shutdown lands mid-blink (active LOW).
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, !LED_STATE_ON);
  digitalWrite(LED_BLUE, !LED_STATE_ON);

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
