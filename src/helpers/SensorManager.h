#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <CayenneLPP.h>
#include "sensors/LocationProvider.h"

#define TELEM_PERM_BASE         0x01   // 'base' permission includes battery
#define TELEM_PERM_LOCATION     0x02
#define TELEM_PERM_ENVIRONMENT  0x04   // permission to access environment sensors

#define TELEM_CHANNEL_SELF   1   // LPP data channel for 'self' device

class SensorManager {
public:
  static const uint8_t DIAGNOSTICS_MAX_I2C_ADDRESSES = 16;

  enum PowerProfile : uint8_t {
    POWER_PROFILE_NORMAL = 0,
    // Longer arm/hold timings for walking/hiking use where fewer GPS wakeups
    // are preferable to fast response after every small movement.
    POWER_PROFILE_EXPEDITION,
    // Longest hold/timeout timings for mostly fixed installations.
    POWER_PROFILE_STATIONARY,
    POWER_PROFILE_COUNT
  };

  enum TimeSourceState : uint8_t {
    TIME_SOURCE_UNKNOWN_OR_STALE = 0,
    TIME_SOURCE_RTC_SET,
    TIME_SOURCE_GPS
  };

  enum CompassCalibrationState : uint8_t {
    COMPASS_CAL_UNKNOWN = 0,
    COMPASS_CAL_MISSING,
    COMPASS_CAL_LOADED,
    COMPASS_CAL_BAD,
    COMPASS_CAL_CALIBRATING
  };

  enum CompassCalibrationResult : uint8_t {
    COMPASS_CAL_RESULT_NONE = 0,
    COMPASS_CAL_RESULT_GOOD,
    COMPASS_CAL_RESULT_FAIR,
    COMPASS_CAL_RESULT_POOR,
    COMPASS_CAL_RESULT_CANCELLED
  };

  enum CompassHeadingValidity : uint8_t {
    COMPASS_HEADING_UNKNOWN = 0,
    COMPASS_HEADING_VALID,
    COMPASS_HEADING_UNCALIBRATED,
    COMPASS_HEADING_STALE,
    COMPASS_HEADING_TILTED,
    COMPASS_HEADING_MAG_INVALID,
    COMPASS_HEADING_MOTION_UNSTABLE,
    COMPASS_HEADING_ORIENTATION_INVALID
  };

  enum MotionState : uint8_t {
    MOTION_UNKNOWN = 0,
    MOTION_STATIONARY,
    MOTION_PENDING,
    MOTION_MOVING
  };

  enum SmartGPSState : uint8_t {
    SMART_GPS_UNAVAILABLE = 0,
    SMART_GPS_NOT_PERMITTED,
    SMART_GPS_STATIONARY,
    SMART_GPS_MOTION_PENDING,
    SMART_GPS_ACQUIRING,
    SMART_GPS_ACTIVE,
    SMART_GPS_STATIONARY_HOLD,
    SMART_GPS_START_PENDING
  };

  struct GPSStatus {
    bool gps_available;
    bool gps_active;
    bool live_fix_valid;
    bool persisted_location_valid;
    bool persisted_location_age_known;
    bool telemetry_using_persisted_location;
    double live_lat;
    double live_lon;
    double live_altitude;
    double persisted_lat;
    double persisted_lon;
    double persisted_altitude;
    uint32_t live_fix_age_sec;
    uint32_t persisted_location_age_sec;
    uint8_t smart_gps_state;
    uint8_t motion_state;
    uint8_t motion_score;
    uint8_t gps_switch_state;
    uint16_t start_pending_seconds;
    uint16_t motion_arming_seconds;
    uint16_t idle_shutdown_seconds;
    uint8_t time_source_state;
    uint8_t rtc_source;
    bool rtc_hardware_present;
    bool last_gps_time_sync_known;
    uint32_t last_gps_time_sync_age_sec;
  };

  struct TimeStatus {
    uint8_t time_source_state;
    uint8_t rtc_source;
    const char* rtc_source_name;
    bool rtc_hardware_present;
    bool rtc_time_valid;
    bool last_gps_time_sync_known;
    uint32_t current_time;
    uint32_t last_gps_time_sync_age_sec;
  };

  struct DiagnosticsStatus {
    uint8_t power_profile;
    bool battery_mv_known;
    uint16_t battery_mv;
    bool external_power_known;
    bool external_powered;
    bool charge_state_known;
    bool charging;
    bool charger_fault_known;
    bool charger_fault;

    bool i2c_wire_scanned;
    bool i2c_wire_truncated;
    uint8_t i2c_wire_count;
    uint8_t i2c_wire_addresses[DIAGNOSTICS_MAX_I2C_ADDRESSES];
    bool i2c_wire1_scanned;
    bool i2c_wire1_truncated;
    uint8_t i2c_wire1_count;
    uint8_t i2c_wire1_addresses[DIAGNOSTICS_MAX_I2C_ADDRESSES];

    uint8_t rtc_source;
    const char* rtc_source_name;
    bool rtc_hardware_present;
    bool rtc_time_valid;
    uint8_t time_source_state;
    bool last_gps_time_sync_known;
    uint32_t last_gps_time_sync_age_sec;

    bool gps_switch_known;
    bool gps_switch_mode1;
    bool gps_switch_mode2;
    uint8_t gps_switch_raw_state;
    uint8_t gps_switch_stable_state;
  };

  struct CompassReading {
    float heading_deg;
    float pitch_deg;
    float roll_deg;
    bool flat;
    bool calibrated;
    bool calibrating;
    uint8_t calibration_seconds_remaining;
    uint8_t calibration_state;
    uint8_t calibration_quality;
    uint8_t calibration_progress;
    uint8_t calibration_result;
    uint8_t calibration_final_quality;
    uint8_t calibration_step;
    uint8_t calibration_step_count;
    uint16_t calibration_samples;
    uint8_t heading_validity;
    uint8_t motion_state;
    uint8_t motion_score;
    uint8_t smart_gps_state;
    uint16_t smart_gps_start_pending_seconds;
    uint16_t smart_gps_motion_arming_seconds;
    uint16_t smart_gps_idle_shutdown_seconds;
    uint32_t age_ms;
    float antenna_heading_deg;
    bool antenna_heading_valid;
    float antenna_elevation_deg;
    bool orientation_valid;
    float east_x, east_y, east_z;
    float north_x, north_y, north_z;
    float up_x, up_y, up_z;
  };

  double node_lat, node_lon;  // modify these, if you want to affect Advert location
  double node_altitude;       // altitude in meters

protected:
  double persisted_lat = 0.0;
  double persisted_lon = 0.0;
  double persisted_altitude = 0.0;
  bool persisted_location_valid = false;
  bool persisted_location_age_known = false;
  uint32_t persisted_location_ms = 0;

  double live_lat = 0.0;
  double live_lon = 0.0;
  double live_altitude = 0.0;
  bool live_location_valid = false;
  uint32_t live_location_ms = 0;

  static bool validLocation(double lat, double lon) {
    return (lat != 0.0 || lon != 0.0) &&
           lat >= -90.0 && lat <= 90.0 &&
           lon >= -180.0 && lon <= 180.0;
  }

  void noteLiveLocation(double lat, double lon, double altitude) {
    if (!validLocation(lat, lon)) return;
    // Live fixes are intentionally separate from persisted fixes. Smart GPS can
    // turn the receiver off after persisting a fix while telemetry continues to
    // advertise the last known coordinates.
    live_lat = lat;
    live_lon = lon;
    live_altitude = altitude;
    live_location_valid = true;
    live_location_ms = millis();
  }

  bool hasPersistedLocationSince(uint32_t since_ms) const {
    return persisted_location_valid &&
           persisted_location_age_known &&
           (int32_t)(persisted_location_ms - since_ms) >= 0;
  }

  void fillBaseGPSStatus(GPSStatus& status) const {
    memset(&status, 0, sizeof(status));
    uint32_t now = millis();
    status.live_fix_valid = live_location_valid;
    status.live_lat = live_lat;
    status.live_lon = live_lon;
    status.live_altitude = live_altitude;
    status.live_fix_age_sec = live_location_valid ? (now - live_location_ms) / 1000UL : 0;
    status.persisted_location_valid = persisted_location_valid;
    status.persisted_location_age_known = persisted_location_age_known;
    status.persisted_lat = persisted_lat;
    status.persisted_lon = persisted_lon;
    status.persisted_altitude = persisted_altitude;
    status.persisted_location_age_sec =
      (persisted_location_valid && persisted_location_age_known) ? (now - persisted_location_ms) / 1000UL : 0;
    // UI/telemetry caveat: when the live fix is absent or stale, callers should
    // label coordinates as last-known rather than live GPS.
    status.telemetry_using_persisted_location =
      persisted_location_valid && (!live_location_valid || status.live_fix_age_sec > 5);
  }

public:
  SensorManager() { node_lat = 0; node_lon = 0; node_altitude = 0; }
  virtual bool begin() { return false; }
  virtual bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) { return false; }
  virtual void loop() { }
  virtual int getNumSettings() const { return 0; }
  virtual const char* getSettingName(int i) const { return NULL; }
  virtual const char* getSettingValue(int i) const { return NULL; }
  virtual bool setSettingValue(const char* name, const char* value) { return false; }
  virtual LocationProvider* getLocationProvider() { return NULL; }
  virtual bool getCompass(CompassReading& reading) { return false; }
  virtual bool getGPSStatus(GPSStatus& status) {
    fillBaseGPSStatus(status);
    return status.live_fix_valid || status.persisted_location_valid;
  }
  virtual bool isGPSEnabled() const { return false; }
  virtual bool setGPSEnabled(bool enabled) { return setSettingValue("gps", enabled ? "1" : "0"); }
  virtual bool setPowerProfile(uint8_t profile) { return profile == POWER_PROFILE_NORMAL; }
  virtual uint8_t getPowerProfile() const { return POWER_PROFILE_NORMAL; }
  static const char* powerProfileName(uint8_t profile) {
    switch (profile) {
      case POWER_PROFILE_EXPEDITION: return "Expedition";
      case POWER_PROFILE_STATIONARY: return "Stationary";
      case POWER_PROFILE_NORMAL:
      default: return "Normal";
    }
  }
  virtual const char* getPowerProfileName() const { return powerProfileName(getPowerProfile()); }
  virtual bool getTimeStatus(TimeStatus& status) {
    memset(&status, 0, sizeof(status));
    status.time_source_state = TIME_SOURCE_UNKNOWN_OR_STALE;
    return false;
  }
  virtual bool getDiagnostics(DiagnosticsStatus& status) {
    memset(&status, 0, sizeof(status));
    status.power_profile = getPowerProfile();
    return false;
  }
  virtual bool shouldPersistLocationNow() const { return false; }
  virtual void notePersistedLocation(double lat, double lon, double altitude, bool age_known) {
    if (!validLocation(lat, lon)) return;
    persisted_lat = lat;
    persisted_lon = lon;
    persisted_altitude = altitude;
    persisted_location_valid = true;
    persisted_location_age_known = age_known;
    persisted_location_ms = millis();
  }

  // Helper functions to manage setting by keys (useful in many places ...)
  const char* getSettingByKey(const char* key) {
    int num = getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(getSettingName(i), key) == 0) {
        return getSettingValue(i);
      }
    }
    return NULL;
  }
};
