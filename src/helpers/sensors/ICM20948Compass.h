#pragma once

#include <Arduino.h>
#include <helpers/SensorManager.h>

#if defined(MUZIWORKS_DUO_SUPER_IO) && defined(MUZIWORKS_SUPER_IO_HAS_ICM20948)

#include <ICM_20948.h>
#include <Wire.h>

// MuziWorks Super IO orientation stack.
//
// Primary references:
// - NXP/Freescale AN4248, "Implementing a Tilt-Compensated eCompass using
//   Accelerometer and Magnetometer Sensors": tilt-compensated magnetic heading
//   basis construction.
// - NXP/Freescale AN4246, "Calibrating an eCompass in the Presence of Hard
//   and Soft-Iron Interference": hard-iron offset and soft-iron correction
//   model. This implementation uses the diagonal approximation documented
//   below, not a full 3x3 ellipsoid fit.
// - TDK/InvenSense ICM-20948 Datasheet DS-000189: accel, gyro, and AK09916
//   magnetometer units provided by the SparkFun ICM-20948 driver.

#ifndef MUZIWORKS_ICM20948_UPDATE_MS
#define MUZIWORKS_ICM20948_UPDATE_MS 200
#endif
#ifndef MUZIWORKS_COMPASS_STALE_MS
#define MUZIWORKS_COMPASS_STALE_MS 2000
#endif
#ifndef MUZIWORKS_COMPASS_FLAT_MIN_Z_RATIO
#define MUZIWORKS_COMPASS_FLAT_MIN_Z_RATIO 0.75f
#endif
#ifndef MUZIWORKS_COMPASS_MIN_MAG_UT
#define MUZIWORKS_COMPASS_MIN_MAG_UT 10.0f
#endif
#ifndef MUZIWORKS_COMPASS_MAX_MAG_UT
#define MUZIWORKS_COMPASS_MAX_MAG_UT 100.0f
#endif
#ifndef MUZIWORKS_COMPASS_CAL_MIN_SAMPLES
#define MUZIWORKS_COMPASS_CAL_MIN_SAMPLES 20
#endif
#ifndef MUZIWORKS_COMPASS_CAL_MIN_SPAN_UT
#define MUZIWORKS_COMPASS_CAL_MIN_SPAN_UT 8.0f
#endif
#ifndef MUZIWORKS_COMPASS_CAL_SECONDS
#define MUZIWORKS_COMPASS_CAL_SECONDS 30
#endif
#ifndef MUZIWORKS_COMPASS_CAL_FILE
#define MUZIWORKS_COMPASS_CAL_FILE "/muzi_compass_cal"
#endif
#ifndef MUZIWORKS_COMPASS_DECLINATION_DEG
#define MUZIWORKS_COMPASS_DECLINATION_DEG 0.0f
#endif
#ifndef MUZIWORKS_COMPASS_HEADING_OFFSET_DEG
#define MUZIWORKS_COMPASS_HEADING_OFFSET_DEG 90.0f
#endif
#ifndef MUZIWORKS_IMU_ROTATION_DEG
#define MUZIWORKS_IMU_ROTATION_DEG 270
#endif
#ifndef MUZIWORKS_MAG_AXIS_X_SIGN
#define MUZIWORKS_MAG_AXIS_X_SIGN 1.0f
#endif
#ifndef MUZIWORKS_MAG_AXIS_Y_SIGN
#define MUZIWORKS_MAG_AXIS_Y_SIGN -1.0f
#endif
#ifndef MUZIWORKS_MAG_AXIS_Z_SIGN
#define MUZIWORKS_MAG_AXIS_Z_SIGN -1.0f
#endif
#ifndef MUZIWORKS_DIRECTIONAL_AXIS_X
#define MUZIWORKS_DIRECTIONAL_AXIS_X 0.0f
#endif
#ifndef MUZIWORKS_DIRECTIONAL_AXIS_Y
#define MUZIWORKS_DIRECTIONAL_AXIS_Y 0.0f
#endif
#ifndef MUZIWORKS_DIRECTIONAL_AXIS_Z
#define MUZIWORKS_DIRECTIONAL_AXIS_Z 1.0f
#endif
#ifndef MUZIWORKS_ANTENNA_MIN_HORIZONTAL_PROJECTION
#define MUZIWORKS_ANTENNA_MIN_HORIZONTAL_PROJECTION 0.20f
#endif
#ifndef MUZIWORKS_COMPASS_CAL_RESULT_HOLD_MS
#define MUZIWORKS_COMPASS_CAL_RESULT_HOLD_MS 2500
#endif
#ifndef MUZIWORKS_COMPASS_CAL_FAIR_QUALITY
#define MUZIWORKS_COMPASS_CAL_FAIR_QUALITY 40
#endif
#ifndef MUZIWORKS_COMPASS_CAL_GOOD_QUALITY
#define MUZIWORKS_COMPASS_CAL_GOOD_QUALITY 70
#endif
#ifndef MUZIWORKS_COMPASS_CAL_STEP_COUNT
#define MUZIWORKS_COMPASS_CAL_STEP_COUNT 3
#endif
#ifndef MUZIWORKS_MOTION_ACCEL_DELTA_G
#define MUZIWORKS_MOTION_ACCEL_DELTA_G 0.06f
#endif
#ifndef MUZIWORKS_MOTION_ACCEL_NORM_DELTA_G
#define MUZIWORKS_MOTION_ACCEL_NORM_DELTA_G 0.10f
#endif
#ifndef MUZIWORKS_MOTION_GYRO_DPS
#define MUZIWORKS_MOTION_GYRO_DPS 8.0f
#endif

/**
 * Dedicated MuziWorks Super IO ICM-20948/eCompass driver.
 *
 * This class owns sensor readout, magnetometer calibration, tilt-compensated
 * orientation math, antenna-axis projection, and the motion classifier used by
 * Smart GPS. Board policy code should only feed it timing/profile state and
 * consume `SensorManager::CompassReading`.
 */
class ICM20948Compass {
public:
  static constexpr float DEG2RAD = 0.01745329252f;
  static constexpr float RAD2DEG = 57.295779513f;

  ICM20948Compass();

  bool begin(TwoWire& wire);
  bool update(bool force);
  bool getReading(SensorManager::CompassReading& reading);

  /** True after I2C probe and wake configuration succeed. */
  bool isInitialized() const { return imu_initialized; }

  /** Latest acceleration sample in g, board-aligned by `applyYawRotation`. */
  const float* accelG() const { return acc_g; }

  /** Latest gyro sample in degrees/second, board-aligned by `applyYawRotation`. */
  const float* gyroDps() const { return gyro_dps; }

  /** Motion state enum consumed by Smart GPS. */
  uint8_t getMotionState() const { return motion_state; }

  /** 0..100 motion confidence/arming score for UI and Smart GPS. */
  uint8_t getMotionScore() const { return motion_score; }

  /** Sustained-motion accumulator in milliseconds for GPS arming countdowns. */
  uint32_t getMotionAccumMillis() const { return motion_accum_ms; }

  void setMotionWindowSeconds(uint16_t seconds);
  void setSmartGPSStatus(uint8_t state, uint16_t start_pending_seconds,
                         uint16_t motion_arming_seconds, uint16_t idle_shutdown_seconds);

  const char* calibrationSettingValue() const;
  void startCalibration(uint16_t seconds = MUZIWORKS_COMPASS_CAL_SECONDS);
  bool cancelCalibration();
  void resetCalibration(bool remove_file);

private:
  struct CalibrationRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    float min_v[3];
    float max_v[3];
  };

  static constexpr uint32_t CAL_MAGIC = 0x4D43414CUL;  // "MCAL"
  static constexpr uint16_t CAL_VERSION = 1;
  static constexpr float VECTOR_MIN_NORM = 0.001f;
  static constexpr float ACCEL_MIN_VALID_NORM_G = 0.1f;
  static constexpr float MAX_MOTION_DT_MS = 1000.0f;

  bool beginIMU(TwoWire& wire);
  static bool compassRangeValid(float mn, float mx);
  static uint8_t clampPercent(float value);
  static const char* calStateName(uint8_t state);
  uint8_t calibrationQualityFor(const float mn[3], const float mx[3], uint16_t samples) const;
  uint8_t calibrationResultForQuality(uint8_t quality) const;
  uint8_t calibrationStepForCurrentCoverage() const;
  void clearCalibrationResult();
  void publishCompassStatus();
  bool applyCompassCalibrationExtrema();
  bool loadCompassCalibration();
  bool saveCompassCalibration();
  void finishCompassCalibration();
  void applyYawRotation(float& x, float& y) const;
  static float normalizeHeading(float heading);
  static void cross3(const float a[3], const float b[3], float out[3]);
  static float dot3(const float a[3], const float b[3]);
  static float clampUnit(float v);
  static bool normalize3(float v[3]);
  static void copyBasisToReading(SensorManager::CompassReading& reading, const float east[3], const float north[3],
                                 const float up[3]);
  bool computeOrientationBasis(float mx, float my, float mz, float acc_norm, float east[3], float north[3],
                               float up[3]) const;
  bool computeBodyAxisHeading(const float body_axis[3], const float east[3], const float north[3], const float up[3],
                              float& heading_deg, float& elevation_deg) const;
  void updateMotionClassifier(uint32_t now, float acc_norm);

  ICM_20948_I2C imu;
  bool imu_initialized = false;
  bool imu_ad0 = false;
  uint32_t next_imu_update = 0;
  uint32_t last_imu_update = 0;
  float acc_g[3] = {0, 0, 0};
  float gyro_dps[3] = {0, 0, 0};
  float mag_ut[3] = {0, 0, 0};
  float mag_min[3] = {0, 0, 0};
  float mag_max[3] = {0, 0, 0};
  float mag_cal_min[3] = {0, 0, 0};
  float mag_cal_max[3] = {0, 0, 0};
  float mag_offset[3] = {0, 0, 0};
  float mag_scale[3] = {1, 1, 1};
  uint16_t mag_samples = 0;
  uint16_t mag_cal_samples = 0;
  bool mag_cal_valid = false;
  bool mag_calibrating = false;
  uint32_t mag_cal_end_ms = 0;
  uint8_t mag_cal_state = SensorManager::COMPASS_CAL_MISSING;
  uint8_t mag_cal_quality = 0;
  uint8_t mag_cal_progress = 0;
  uint8_t mag_cal_result = SensorManager::COMPASS_CAL_RESULT_NONE;
  uint8_t mag_cal_final_quality = 0;
  uint8_t mag_cal_step = 0;
  uint8_t mag_cal_state_before_cal = SensorManager::COMPASS_CAL_MISSING;
  uint8_t mag_cal_quality_before_cal = 0;
  uint16_t mag_cal_rejected = 0;
  uint32_t mag_cal_result_until_ms = 0;
  float last_motion_acc_g[3] = {0, 0, 0};
  bool last_motion_acc_valid = false;
  uint32_t last_motion_sample_ms = 0;
  uint32_t motion_accum_ms = 0;
  uint8_t motion_state = SensorManager::MOTION_UNKNOWN;
  uint8_t motion_score = 0;
  uint16_t motion_window_seconds = 10;
  SensorManager::CompassReading compass = {};
};

#endif
