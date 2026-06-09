#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>
#if ENV_INCLUDE_GPS == 1
#include <helpers/sensors/MicroNMEALocationProvider.h>
#endif
#if defined(MUZIWORKS_DUO_SUPER_IO) && defined(MUZIWORKS_SUPER_IO_HAS_ICM20948)
#include <helpers/sensors/ICM20948Compass.h>
#endif

MuziWorksDuoBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

#if defined(MUZIWORKS_DUO_SUPER_IO)
VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
#else
VolatileRTCClock rtc_clock;
#endif
#if ENV_INCLUDE_GPS == 1
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
static EnvironmentSensorManager environment_sensors = EnvironmentSensorManager(nmea);
#else
static EnvironmentSensorManager environment_sensors;
#endif

#if defined(MUZIWORKS_DUO_SUPER_IO)
static bool muzi_i2c_probe(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  return wire.endTransmission() == 0;
}

struct MuziI2CBusSnapshot {
  bool scanned;
  bool truncated;
  uint8_t count;
  uint8_t addresses[SensorManager::DIAGNOSTICS_MAX_I2C_ADDRESSES];
};

static MuziI2CBusSnapshot muzi_wire_scan = {};
static MuziI2CBusSnapshot muzi_wire1_scan = {};

static void muzi_wire1_begin() {
  static bool wire1_begun = false;
  if (wire1_begun) return;
#if WIRE_INTERFACES_COUNT > 1
  Wire1.setPins(PIN_WIRE1_SDA, PIN_WIRE1_SCL);
  Wire1.setClock(100000);
  Wire1.begin();
#endif
  wire1_begun = true;
}

static void muzi_scan_i2c_bus(TwoWire& wire, const char* name, MuziI2CBusSnapshot& snapshot) {
  // Diagnostics use the boot-time scan snapshot. Re-scanning from the UI path
  // would add I2C traffic while the display, RTC, and IMU are active.
  memset(&snapshot, 0, sizeof(snapshot));
  snapshot.scanned = true;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (muzi_i2c_probe(wire, addr)) {
      if (snapshot.count < SensorManager::DIAGNOSTICS_MAX_I2C_ADDRESSES) {
        snapshot.addresses[snapshot.count++] = addr;
      } else {
        snapshot.truncated = true;
      }
#if MUZIWORKS_SUPER_IO_I2C_SCAN
      MESH_DEBUG_PRINTLN("Muzi Super IO %s I2C found 0x%02X", name, addr);
#endif
    }
  }
#if !MUZIWORKS_SUPER_IO_I2C_SCAN
  (void)name;
#endif
}

static void muzi_rtc_begin() {
  static bool rtc_begun = false;
  if (rtc_begun) return;

  Wire.begin();
  rtc_clock.begin(Wire);
  muzi_scan_i2c_bus(Wire, "Wire", muzi_wire_scan);

  muzi_wire1_begin();
#if WIRE_INTERFACES_COUNT > 1
  // Wire1 also hosts ICM-20948 at 0x68/0x69. Probe only the known RX8130CE
  // address here so the IMU cannot be mis-detected as a DS3231 RTC.
  rtc_clock.beginRX8130CE(Wire1);
  muzi_scan_i2c_bus(Wire1, "Wire1", muzi_wire1_scan);
#endif
  rtc_begun = true;
}
#endif

#if defined(MUZIWORKS_DUO_SUPER_IO) && defined(MUZIWORKS_SUPER_IO_HAS_ICM20948)
#ifndef MUZIWORKS_SMART_GPS
#define MUZIWORKS_SMART_GPS 0
#endif
#ifndef MUZIWORKS_SMART_GPS_MOTION_SECONDS
#define MUZIWORKS_SMART_GPS_MOTION_SECONDS 10
#endif
#ifndef MUZIWORKS_SMART_GPS_STATIONARY_HOLD_MS
#define MUZIWORKS_SMART_GPS_STATIONARY_HOLD_MS 60000
#endif
#ifndef MUZIWORKS_SMART_GPS_MIN_ON_MS
#define MUZIWORKS_SMART_GPS_MIN_ON_MS 30000
#endif
#ifndef MUZIWORKS_SMART_GPS_NO_FIX_TIMEOUT_MS
#define MUZIWORKS_SMART_GPS_NO_FIX_TIMEOUT_MS 90000
#endif
#ifndef MUZIWORKS_SMART_GPS_BOOT_START_DELAY_MS
#define MUZIWORKS_SMART_GPS_BOOT_START_DELAY_MS 30000
#endif
#ifndef MUZIWORKS_SMART_GPS_REQUIRE_PERSISTED_FIX_BEFORE_STOP
#define MUZIWORKS_SMART_GPS_REQUIRE_PERSISTED_FIX_BEFORE_STOP 1
#endif
#ifndef MUZIWORKS_GPS_TIME_FRESH_MS
#define MUZIWORKS_GPS_TIME_FRESH_MS 3600000UL
#endif

class MuziWorksDuoSensorManager : public EnvironmentSensorManager {
  ICM20948Compass compass_imu;
  uint8_t smart_gps_state = SensorManager::SMART_GPS_UNAVAILABLE;
  uint32_t smart_gps_started_ms = 0;
  uint32_t smart_gps_stationary_since_ms = 0;
  uint32_t smart_gps_last_log_ms = 0;
  uint32_t smart_gps_start_allowed_ms = 0;
  uint16_t smart_gps_start_pending_seconds = 0;
  uint16_t smart_gps_motion_arming_seconds = 0;
  uint16_t smart_gps_idle_shutdown_seconds = 0;
  uint8_t power_profile = SensorManager::POWER_PROFILE_NORMAL;
  uint16_t smart_gps_motion_seconds = MUZIWORKS_SMART_GPS_MOTION_SECONDS;
  uint32_t smart_gps_stationary_hold_ms = MUZIWORKS_SMART_GPS_STATIONARY_HOLD_MS;
  uint32_t smart_gps_min_on_ms = MUZIWORKS_SMART_GPS_MIN_ON_MS;
  uint32_t smart_gps_no_fix_timeout_ms = MUZIWORKS_SMART_GPS_NO_FIX_TIMEOUT_MS;
  uint32_t smart_gps_boot_start_delay_ms = MUZIWORKS_SMART_GPS_BOOT_START_DELAY_MS;

  struct PowerProfileConfig {
    uint16_t motion_seconds;
    uint32_t stationary_hold_ms;
    uint32_t min_on_ms;
    uint32_t no_fix_timeout_ms;
    uint32_t boot_start_delay_ms;
  };

  static const PowerProfileConfig& powerProfileConfig(uint8_t profile) {
    // Profiles tune only Smart GPS timing. Sensor math, telemetry format, and
    // persisted-location rules stay identical so users can change profile
    // without changing what a fix means.
    static const PowerProfileConfig profiles[] = {
      {MUZIWORKS_SMART_GPS_MOTION_SECONDS, MUZIWORKS_SMART_GPS_STATIONARY_HOLD_MS,
       MUZIWORKS_SMART_GPS_MIN_ON_MS, MUZIWORKS_SMART_GPS_NO_FIX_TIMEOUT_MS,
       MUZIWORKS_SMART_GPS_BOOT_START_DELAY_MS},  // Normal
      {20, 120000UL, 45000UL, 120000UL, 45000UL},  // Expedition
      {30, 300000UL, 60000UL, 180000UL, 60000UL}   // Stationary
    };
    if (profile >= SensorManager::POWER_PROFILE_COUNT) profile = SensorManager::POWER_PROFILE_NORMAL;
    return profiles[profile];
  }

  void publishSmartGPSStatus() {
    compass_imu.setSmartGPSStatus(smart_gps_state,
                                  smart_gps_start_pending_seconds,
                                  smart_gps_motion_arming_seconds,
                                  smart_gps_idle_shutdown_seconds);
  }

  void applyPowerProfileConfig(uint8_t profile, bool reset_start_delay) {
    const PowerProfileConfig& config = powerProfileConfig(profile);
    power_profile = profile < SensorManager::POWER_PROFILE_COUNT ? profile : SensorManager::POWER_PROFILE_NORMAL;
    smart_gps_motion_seconds = config.motion_seconds;
    smart_gps_stationary_hold_ms = config.stationary_hold_ms;
    smart_gps_min_on_ms = config.min_on_ms;
    smart_gps_no_fix_timeout_ms = config.no_fix_timeout_ms;
    smart_gps_boot_start_delay_ms = config.boot_start_delay_ms;
    compass_imu.setMotionWindowSeconds(smart_gps_motion_seconds);

    if (reset_start_delay && !gps_active) {
      smart_gps_start_allowed_ms = millis() + smart_gps_boot_start_delay_ms;
    }
  }

  static void copyI2CSnapshot(const MuziI2CBusSnapshot& snapshot, bool& scanned, bool& truncated,
                              uint8_t& count, uint8_t addresses[]) {
    scanned = snapshot.scanned;
    truncated = snapshot.truncated;
    count = snapshot.count;
    for (uint8_t i = 0; i < SensorManager::DIAGNOSTICS_MAX_I2C_ADDRESSES; i++) {
      addresses[i] = i < snapshot.count ? snapshot.addresses[i] : 0;
    }
  }

public:
#if ENV_INCLUDE_GPS == 1
  MuziWorksDuoSensorManager(LocationProvider& location) : EnvironmentSensorManager(location) { }
#else
  MuziWorksDuoSensorManager() : EnvironmentSensorManager() { }
#endif

  bool begin() override {
    muzi_rtc_begin();
    applyPowerProfileConfig(power_profile, false);
    bool base_ok = EnvironmentSensorManager::begin();
    muzi_wire1_begin();
    bool imu_ok = compass_imu.begin(Wire1);
    smart_gps_start_allowed_ms = millis() + smart_gps_boot_start_delay_ms;
    publishSmartGPSStatus();
    return base_ok || imu_ok;
  }

  void loop() override {
    EnvironmentSensorManager::loop();
    compass_imu.update(false);
    updateSmartGPS();
  }

  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override {
    bool ok = EnvironmentSensorManager::querySensors(requester_permissions, telemetry);
    SensorManager::CompassReading reading = {};
    if ((requester_permissions & TELEM_PERM_ENVIRONMENT) && getCompass(reading)) {
      if (reading.flat) {
        telemetry.addDirection(next_available_channel++, reading.heading_deg);
      }
      const float* acc_g = compass_imu.accelG();
      const float* gyro_dps = compass_imu.gyroDps();
      telemetry.addAccelerometer(next_available_channel++, acc_g[0], acc_g[1], acc_g[2]);
      telemetry.addGyrometer(next_available_channel++, gyro_dps[0], gyro_dps[1], gyro_dps[2]);
      ok = true;
    }
    return ok;
  }

  int getNumSettings() const override {
    return EnvironmentSensorManager::getNumSettings() + (compass_imu.isInitialized() ? 1 : 0);
  }

  const char* getSettingName(int i) const override {
    int base = EnvironmentSensorManager::getNumSettings();
    if (i < base) return EnvironmentSensorManager::getSettingName(i);
    if (compass_imu.isInitialized() && i == base) return "compass_cal";
    return NULL;
  }

  const char* getSettingValue(int i) const override {
    int base = EnvironmentSensorManager::getNumSettings();
    if (i < base) return EnvironmentSensorManager::getSettingValue(i);
    if (compass_imu.isInitialized() && i == base) return compass_imu.calibrationSettingValue();
    return NULL;
  }

  bool setSettingValue(const char* name, const char* value) override {
#if MUZIWORKS_SMART_GPS && ENV_INCLUDE_GPS == 1
    if (strcmp(name, "gps") == 0) {
      if (strcmp(value, "0") == 0) {
        stop_gps();
        smart_gps_started_ms = 0;
        smart_gps_stationary_since_ms = 0;
      }
      MESH_DEBUG_PRINTLN("Muzi smart GPS owns GPS setting; requested=%s active=%d", value, gps_active ? 1 : 0);
      return true;
    }
#endif
    if (strcmp(name, "compass_cal") == 0) {
      if (strcmp(value, "start") == 0 || strcmp(value, "calibrate") == 0 || strcmp(value, "1") == 0) {
        compass_imu.startCalibration(MUZIWORKS_COMPASS_CAL_SECONDS);
        return true;
      }
      if (strcmp(value, "cancel") == 0 || strcmp(value, "abort") == 0) {
        return compass_imu.cancelCalibration();
      }
      if (strcmp(value, "reset") == 0 || strcmp(value, "0") == 0) {
        compass_imu.resetCalibration(true);
        MESH_DEBUG_PRINTLN("Muzi ICM-20948 compass calibration reset");
        return true;
      }
      return false;
    }
    return EnvironmentSensorManager::setSettingValue(name, value);
  }

  bool getCompass(SensorManager::CompassReading& reading) override {
    return compass_imu.getReading(reading);
  }

  bool getGPSStatus(SensorManager::GPSStatus& status) override {
    bool ok = EnvironmentSensorManager::getGPSStatus(status);
    status.smart_gps_state = smart_gps_state;
    status.motion_state = compass_imu.getMotionState();
    status.motion_score = compass_imu.getMotionScore();
    status.gps_switch_state = gps_switch_get_stable();
    status.start_pending_seconds = smart_gps_start_pending_seconds;
    status.motion_arming_seconds = smart_gps_motion_arming_seconds;
    status.idle_shutdown_seconds = smart_gps_idle_shutdown_seconds;
    status.telemetry_using_persisted_location =
      status.persisted_location_valid && (!status.live_fix_valid || !gps_active);
    SensorManager::TimeStatus time = {};
    if (getTimeStatus(time)) {
      status.time_source_state = time.time_source_state;
      status.rtc_source = time.rtc_source;
      status.rtc_hardware_present = time.rtc_hardware_present;
      status.last_gps_time_sync_known = time.last_gps_time_sync_known;
      status.last_gps_time_sync_age_sec = time.last_gps_time_sync_age_sec;
    }
    return ok || smart_gps_state != SensorManager::SMART_GPS_UNAVAILABLE;
  }

  bool setGPSEnabled(bool enabled) override {
#if MUZIWORKS_SMART_GPS && ENV_INCLUDE_GPS == 1
    return setSettingValue("gps", enabled ? "1" : "0");
#else
    return EnvironmentSensorManager::setGPSEnabled(enabled);
#endif
  }

  bool setPowerProfile(uint8_t profile) override {
    if (profile >= SensorManager::POWER_PROFILE_COUNT) profile = SensorManager::POWER_PROFILE_NORMAL;
    applyPowerProfileConfig(profile, true);
    MESH_DEBUG_PRINTLN("Muzi power profile: %s", SensorManager::powerProfileName(power_profile));
    publishSmartGPSStatus();
    return true;
  }

  uint8_t getPowerProfile() const override {
    return power_profile;
  }

  bool getTimeStatus(SensorManager::TimeStatus& status) override {
    memset(&status, 0, sizeof(status));
    status.rtc_source = rtc_clock.getSource();
    status.rtc_source_name = rtc_clock.getSourceName();
    status.rtc_hardware_present = rtc_clock.hasHardwareRTC();
    status.current_time = rtc_clock.getCurrentTime();
    bool raw_rtc_time_valid = rtc_clock.isTimeValid();
    status.rtc_time_valid = raw_rtc_time_valid && (rtc_clock.wasTimeSet() || status.rtc_hardware_present);

#if ENV_INCLUDE_GPS == 1
    uint32_t gps_sync_ms = 0;
    if (_location != NULL && _location->getLastTimeSyncMillis(gps_sync_ms)) {
      status.last_gps_time_sync_known = true;
      status.last_gps_time_sync_age_sec = (millis() - gps_sync_ms) / 1000UL;
    }
#endif

    if (status.last_gps_time_sync_known &&
        status.last_gps_time_sync_age_sec <= (MUZIWORKS_GPS_TIME_FRESH_MS / 1000UL)) {
      status.time_source_state = SensorManager::TIME_SOURCE_GPS;
    } else if (rtc_clock.wasTimeSet() || (status.rtc_hardware_present && raw_rtc_time_valid)) {
      status.time_source_state = SensorManager::TIME_SOURCE_RTC_SET;
    } else {
      status.time_source_state = SensorManager::TIME_SOURCE_UNKNOWN_OR_STALE;
    }
    return true;
  }

  bool getDiagnostics(SensorManager::DiagnosticsStatus& status) override {
    memset(&status, 0, sizeof(status));
    status.power_profile = power_profile;
    status.battery_mv_known = true;
    status.battery_mv = board.getBattMilliVolts();
    status.external_power_known = true;
    status.external_powered = board.isExternalPowered();
    status.charge_state_known = board.hasBatteryChargingStatus();
    status.charging = board.isBatteryCharging();
    status.charger_fault_known = board.hasChargerFaultStatus();
    status.charger_fault = board.isChargerFaultActive();

    copyI2CSnapshot(muzi_wire_scan, status.i2c_wire_scanned, status.i2c_wire_truncated,
                    status.i2c_wire_count, status.i2c_wire_addresses);
    copyI2CSnapshot(muzi_wire1_scan, status.i2c_wire1_scanned, status.i2c_wire1_truncated,
                    status.i2c_wire1_count, status.i2c_wire1_addresses);

    SensorManager::TimeStatus time = {};
    getTimeStatus(time);
    status.rtc_source = time.rtc_source;
    status.rtc_source_name = time.rtc_source_name;
    status.rtc_hardware_present = time.rtc_hardware_present;
    status.rtc_time_valid = time.rtc_time_valid;
    status.time_source_state = time.time_source_state;
    status.last_gps_time_sync_known = time.last_gps_time_sync_known;
    status.last_gps_time_sync_age_sec = time.last_gps_time_sync_age_sec;

    bool switch_mode1 = false;
    bool switch_mode2 = false;
    GpsSwitchState raw_switch_state = GPS_SWITCH_UNKNOWN;
    if (gps_switch_get_raw(switch_mode1, switch_mode2, raw_switch_state)) {
      status.gps_switch_known = true;
      status.gps_switch_mode1 = switch_mode1;
      status.gps_switch_mode2 = switch_mode2;
      status.gps_switch_raw_state = raw_switch_state;
      status.gps_switch_stable_state = gps_switch_get_stable();
    }
    return true;
  }

  bool shouldPersistLocationNow() const override {
#if MUZIWORKS_SMART_GPS && ENV_INCLUDE_GPS == 1
    return gps_active &&
           _location != NULL &&
           _location->isValid() &&
           smart_gps_started_ms != 0 &&
           !hasPersistedLocationSince(smart_gps_started_ms);
#else
    return false;
#endif
  }

private:
  void updateSmartGPS() {
#if MUZIWORKS_SMART_GPS && ENV_INCLUDE_GPS == 1
    // Smart GPS policy:
    // - physical ON-GPS switch must permit GPS
    // - boot delay prevents early rail/I2C/GNSS contention
    // - sustained IMU motion arms and starts GPS
    // - stationary hold waits before sleeping GPS again
    // - with REQUIRE_PERSISTED_FIX, GPS may not sleep until a post-start fix
    //   has been persisted, so telemetry can continue sharing last-known
    //   location while the receiver is off.
    uint32_t now = millis();
    GpsSwitchState switch_state = gps_switch_get_stable();
    bool permitted = switch_state == GPS_SWITCH_ON_GPS;
    smart_gps_start_pending_seconds = 0;
    smart_gps_motion_arming_seconds = 0;
    smart_gps_idle_shutdown_seconds = 0;

    if (!permitted || !compass_imu.isInitialized()) {
      if (gps_active) {
        MESH_DEBUG_PRINTLN("Muzi smart GPS stop: not permitted state=%d", (int)switch_state);
        stop_gps();
      }
      smart_gps_started_ms = 0;
      smart_gps_stationary_since_ms = 0;
      smart_gps_state = permitted ? SensorManager::SMART_GPS_UNAVAILABLE : SensorManager::SMART_GPS_NOT_PERMITTED;
      publishSmartGPSStatus();
      return;
    }

    if (!gps_active && (int32_t)(now - smart_gps_start_allowed_ms) < 0) {
      uint32_t remaining_ms = smart_gps_start_allowed_ms - now;
      smart_gps_start_pending_seconds = (remaining_ms + 999UL) / 1000UL;
      smart_gps_state = SensorManager::SMART_GPS_START_PENDING;
      publishSmartGPSStatus();
      return;
    }

    bool moving = compass_imu.getMotionState() == SensorManager::MOTION_MOVING;
    if (moving) {
      smart_gps_stationary_since_ms = 0;
      if (!gps_active) {
        start_gps();
        smart_gps_started_ms = now;
        MESH_DEBUG_PRINTLN("Muzi smart GPS start: sustained movement");
      }
    } else if (gps_active && smart_gps_stationary_since_ms == 0) {
      smart_gps_stationary_since_ms = now;
    }

    bool fix_valid = _location != NULL && _location->isValid();
    if (gps_active) {
      bool stationary_hold_elapsed = smart_gps_stationary_since_ms != 0 &&
        (uint32_t)(now - smart_gps_stationary_since_ms) >= smart_gps_stationary_hold_ms;
      bool min_on_elapsed = smart_gps_started_ms == 0 ||
        (uint32_t)(now - smart_gps_started_ms) >= smart_gps_min_on_ms;
      bool persisted_since_start = smart_gps_started_ms == 0 || hasPersistedLocationSince(smart_gps_started_ms);
      bool no_fix_timeout = !fix_valid && smart_gps_started_ms != 0 &&
        (uint32_t)(now - smart_gps_started_ms) >= smart_gps_no_fix_timeout_ms;
      if (smart_gps_stationary_since_ms != 0 && !stationary_hold_elapsed) {
        uint32_t remaining_ms = smart_gps_stationary_hold_ms - (now - smart_gps_stationary_since_ms);
        smart_gps_idle_shutdown_seconds = (remaining_ms + 999UL) / 1000UL;
      }

      bool may_stop_without_fix =
        !MUZIWORKS_SMART_GPS_REQUIRE_PERSISTED_FIX_BEFORE_STOP && no_fix_timeout;
      bool may_stop_after_persist =
        stationary_hold_elapsed && persisted_since_start;

      if (!moving && min_on_elapsed && (may_stop_after_persist || may_stop_without_fix)) {
        MESH_DEBUG_PRINTLN("Muzi smart GPS stop: stationary=%d no_fix=%d",
                           stationary_hold_elapsed ? 1 : 0,
                           no_fix_timeout ? 1 : 0);
        stop_gps();
        smart_gps_started_ms = 0;
        smart_gps_stationary_since_ms = 0;
        smart_gps_state = SensorManager::SMART_GPS_STATIONARY;
      } else if (!moving && fix_valid) {
        smart_gps_state = SensorManager::SMART_GPS_STATIONARY_HOLD;
      } else {
        smart_gps_state = fix_valid ? SensorManager::SMART_GPS_ACTIVE : SensorManager::SMART_GPS_ACQUIRING;
      }
    } else if (compass_imu.getMotionState() == SensorManager::MOTION_PENDING) {
      uint32_t window_ms = (uint32_t)smart_gps_motion_seconds * 1000UL;
      uint32_t motion_accum_ms = compass_imu.getMotionAccumMillis();
      uint32_t remaining_ms = motion_accum_ms >= window_ms ? 0 : window_ms - motion_accum_ms;
      smart_gps_motion_arming_seconds = (remaining_ms + 999UL) / 1000UL;
      smart_gps_state = SensorManager::SMART_GPS_MOTION_PENDING;
    } else {
      smart_gps_state = SensorManager::SMART_GPS_STATIONARY;
    }

    if ((uint32_t)(now - smart_gps_last_log_ms) >= 5000) {
      MESH_DEBUG_PRINTLN("Muzi smart GPS state=%u motion=%u score=%u gps=%d",
                         smart_gps_state,
                         compass_imu.getMotionState(),
                         compass_imu.getMotionScore(),
                         gps_active ? 1 : 0);
      smart_gps_last_log_ms = now;
    }
    publishSmartGPSStatus();
#else
    smart_gps_state = SensorManager::SMART_GPS_UNAVAILABLE;
    publishSmartGPSStatus();
#endif
  }
};

#if ENV_INCLUDE_GPS == 1
static MuziWorksDuoSensorManager muzi_sensors(nmea);
#else
static MuziWorksDuoSensorManager muzi_sensors;
#endif
SensorManager& sensors = muzi_sensors;
#else
SensorManager& sensors = environment_sensors;
#endif

#ifdef DISPLAY_CLASS
DISPLAY_CLASS display;
MomentaryButton user_btn(PIN_USER_BTN, 1000, true, MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS, false);
  #if UI_HAS_DPAD
MomentaryButton joystick_up(JOYSTICK_UP, 1000, true, MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS, false);
MomentaryButton joystick_down(JOYSTICK_DOWN, 1000, true, MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS, false);
  #endif
MomentaryButton joystick_left(JOYSTICK_LEFT, 1000, true, MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS, false);
MomentaryButton joystick_right(JOYSTICK_RIGHT, 1000, true, MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS, false);
MomentaryButton back_btn(PIN_BACK_BTN, 1000, true, MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS, true);
#endif

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef RF_SWITCH_TABLE
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC,
  RADIOLIB_NC,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6
  { LR11x0::MODE_STBY,   {LOW,  LOW }},
  { LR11x0::MODE_RX,     {HIGH, LOW }},
  { LR11x0::MODE_TX,     {LOW,  HIGH}},
  { LR11x0::MODE_TX_HP,  {LOW,  HIGH}},
  { LR11x0::MODE_TX_HF,  {LOW,  LOW }},
  { LR11x0::MODE_GNSS,   {LOW,  LOW }},
  { LR11x0::MODE_WIFI,   {LOW,  LOW }},
  END_OF_MODE_TABLE,
};
#endif

bool radio_init() {
#if defined(USE_SX1262)
  return radio.std_init(&SPI);
#else
#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 3.0f;
#endif

  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();

  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;
  }

  radio.setCRC(2);
  radio.explicitHeader();

#ifdef RF_SWITCH_TABLE
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
#endif

#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

  return true;
#endif
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(int8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);
}

#if defined(MUZIWORKS_DUO_SUPER_IO)
#ifndef MUZIWORKS_SUPER_IO_SWITCH_DEBOUNCE_MS
#define MUZIWORKS_SUPER_IO_SWITCH_DEBOUNCE_MS 200
#endif
#ifndef MUZIWORKS_SUPER_IO_SWITCH_OFF_CONFIRM_MS
#define MUZIWORKS_SUPER_IO_SWITCH_OFF_CONFIRM_MS 3000
#endif
#ifndef MUZIWORKS_SUPER_IO_SWITCH_OFF_TRANSITION_GRACE_MS
#define MUZIWORKS_SUPER_IO_SWITCH_OFF_TRANSITION_GRACE_MS 1200
#endif

static GpsSwitchState switch_stable_state = GPS_SWITCH_UNKNOWN;
static GpsSwitchState switch_pending_state = GPS_SWITCH_UNKNOWN;
static GpsSwitchState switch_last_raw_state = GPS_SWITCH_UNKNOWN;
static unsigned long switch_pending_since = 0;
static unsigned long switch_last_non_off_raw_ms = 0;

static bool switch_pin_asserted(uint8_t pin) {
  return digitalRead(pin) == MUZIWORKS_SWITCH_ACTIVE;
}

void gps_switch_begin() {
#if MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS
  pinMode(SWITCH_MODE1, INPUT_PULLUP);
  pinMode(SWITCH_MODE2, INPUT_PULLUP);
#else
  pinMode(SWITCH_MODE1, INPUT);
  pinMode(SWITCH_MODE2, INPUT);
#endif
  switch_stable_state = GPS_SWITCH_UNKNOWN;
  switch_pending_state = GPS_SWITCH_UNKNOWN;
  switch_last_raw_state = GPS_SWITCH_UNKNOWN;
  switch_pending_since = millis();
  switch_last_non_off_raw_ms = switch_pending_since;
}

static GpsSwitchState gps_switch_decode(bool mode1, bool mode2) {
  // Physical Super IO detents: TOP = neither asserted (ON+GPS),
  // MID = P0.12 asserted (ON), BOTTOM = P1.09 asserted (OFF).
  if (!mode1 && !mode2) return GPS_SWITCH_ON_GPS;
  if (!mode1 && mode2) return GPS_SWITCH_ON;
  if (mode1 && !mode2) return GPS_SWITCH_OFF;
  return GPS_SWITCH_UNKNOWN;
}

static GpsSwitchState gps_switch_read_raw(bool& mode1, bool& mode2) {
  mode1 = switch_pin_asserted(SWITCH_MODE1);
  mode2 = switch_pin_asserted(SWITCH_MODE2);
  return gps_switch_decode(mode1, mode2);
}

bool gps_switch_get_raw(bool& mode1, bool& mode2, GpsSwitchState& state) {
  state = gps_switch_read_raw(mode1, mode2);
  return true;
}

static GpsSwitchState gps_switch_read() {
  bool mode1 = false;
  bool mode2 = false;
  return gps_switch_read_raw(mode1, mode2);
}

GpsSwitchState gps_switch_get_stable() {
  return switch_stable_state;
}

static bool gps_switch_read_stable(GpsSwitchState& state, uint32_t stable_ms) {
  GpsSwitchState first = gps_switch_read();
  uint32_t started = millis();
  while ((uint32_t)(millis() - started) < stable_ms) {
    delay(10);
    if (gps_switch_read() != first) {
      state = GPS_SWITCH_UNKNOWN;
      return false;
    }
  }
  state = first;
  switch_stable_state = first;
  switch_pending_state = first;
  switch_pending_since = millis();
  MESH_DEBUG_PRINTLN("Muzi Super IO switch confirmed state=%d", (int)switch_stable_state);
  return true;
}

bool gps_switch_confirm_initial_state(GpsSwitchState& state, uint32_t normal_ms, uint32_t off_ms) {
  bool mode1 = false;
  bool mode2 = false;
  GpsSwitchState first = gps_switch_read_raw(mode1, mode2);
  MESH_DEBUG_PRINTLN("Muzi Super IO switch boot raw mode1=%d mode2=%d state=%d",
                     mode1 ? 1 : 0, mode2 ? 1 : 0, (int)first);
  uint32_t confirm_ms = first == GPS_SWITCH_OFF ? off_ms : normal_ms;
  return gps_switch_read_stable(state, confirm_ms);
}

bool gps_switch_poll(GpsSwitchState& state) {
  bool mode1 = false;
  bool mode2 = false;
  GpsSwitchState raw_state = gps_switch_read_raw(mode1, mode2);
  unsigned long now = millis();

  if (raw_state != GPS_SWITCH_OFF) {
    switch_last_non_off_raw_ms = now;
  }

  if (raw_state != switch_last_raw_state) {
    switch_last_raw_state = raw_state;
    MESH_DEBUG_PRINTLN("Muzi Super IO switch raw mode1=%d mode2=%d state=%d stable=%d",
                       mode1 ? 1 : 0, mode2 ? 1 : 0,
                       (int)raw_state, (int)switch_stable_state);
  }

  if (raw_state == GPS_SWITCH_OFF &&
      switch_stable_state != GPS_SWITCH_OFF &&
      (unsigned long)(now - switch_last_non_off_raw_ms) <
          MUZIWORKS_SUPER_IO_SWITCH_OFF_TRANSITION_GRACE_MS) {
    // The slide switch is break-before-make. During a fast MID <-> UP move,
    // the two inputs can briefly look like DOWN/OFF; do not arm shutdown.
    return false;
  }

  if (raw_state != switch_pending_state) {
    switch_pending_state = raw_state;
    switch_pending_since = now;
    return false;
  }

  if (raw_state != switch_stable_state &&
      (unsigned long)(now - switch_pending_since) >=
          (raw_state == GPS_SWITCH_OFF ? MUZIWORKS_SUPER_IO_SWITCH_OFF_CONFIRM_MS
                                       : MUZIWORKS_SUPER_IO_SWITCH_DEBOUNCE_MS)) {
    switch_stable_state = raw_state;
    state = switch_stable_state;
    MESH_DEBUG_PRINTLN("Muzi Super IO switch stable state=%d", (int)switch_stable_state);
    return true;
  }

  return false;
}
#endif
