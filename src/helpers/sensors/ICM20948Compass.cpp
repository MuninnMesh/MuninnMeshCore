/**
 * MuziWorks Super IO compass math and calibration notes.
 *
 * References used to design this implementation:
 * - NXP/Freescale AN4248, "Implementing a Tilt-Compensated eCompass using
 *   Accelerometer and Magnetometer Sensors".
 * - NXP/Freescale AN4246, "Calibrating an eCompass in the Presence of Hard
 *   and Soft-Iron Interference".
 * - TDK/InvenSense ICM-20948 datasheet DS-000189. The ICM-20948 integrates a
 *   3-axis accelerometer, 3-axis gyroscope, and AK09916 3-axis magnetometer;
 *   this file uses the SparkFun ICM-20948 driver only for raw sensor access.
 *
 * Sensor model.
 * AN4248 models the board-fixed accelerometer and magnetometer as vectors that
 * change with PCB yaw, pitch, and roll. A flat board can derive heading from
 * the two horizontal magnetic components, but a handheld device is rarely flat,
 * so heading must use all three accelerometer and magnetometer axes. The
 * accelerometer is treated as a gravity vector while the unit is not undergoing
 * significant linear acceleration; that gravity vector defines the local "up"
 * direction used for tilt compensation.
 *
 * Magnetic interference model.
 * AN4246 describes raw magnetometer samples in the presence of hard- and
 * soft-iron interference as an ellipsoid rather than a sphere. The hard-iron
 * component is the ellipsoid center V and is subtracted from each sample. The
 * full soft-iron correction is a 3x3 inverse matrix W^-1 applied to (Braw - V),
 * transforming the ellipsoid back toward a sphere centered at the origin.
 *
 * Embedded calibration policy.
 * This firmware intentionally uses a lightweight diagonal approximation instead
 * of a full 10-parameter ellipsoid fit:
 *
 *   offset_i    = (min_i + max_i) / 2
 *   span_i      = max_i - min_i
 *   avg_span    = mean(span_x, span_y, span_z)
 *   m_cal_i     = (m_raw_i - offset_i) * (avg_span / span_i)
 *
 * The min/max extrema are collected while the user rotates the unit through as
 * many orientations as possible. This removes the dominant hard-iron offset and
 * approximates axis gain/stretch, but it does not estimate cross-axis coupling
 * or non-orthogonality. If future hardware needs higher compass accuracy in a
 * magnetically difficult enclosure, replace the persisted calibration record
 * with a versioned ellipsoid fit and full 3x3 correction matrix.
 *
 * Tilt-compensated heading logic.
 * After calibration, the code builds an ENU basis directly with vector algebra:
 *
 *   up          = normalize(accel_g)
 *   mag         = normalize(m_cal)
 *   north_mag   = normalize(mag - up * dot(mag, up))
 *   east        = normalize(cross(north_mag, up))
 *   north       = rotate_in_horizontal_plane(north_mag, declination + offset)
 *
 * This is equivalent to the AN4248 roll/pitch de-rotation idea, but avoids
 * depending on Euler-angle formulas for the final heading. Degenerate vectors
 * fail closed: stale samples, out-of-range magnetic magnitude, near-zero
 * projections, unstable motion, or excessive tilt mark the public reading as
 * invalid instead of returning a plausible but wrong bearing.
 *
 * Body-axis projection.
 * Once east/north/up are known in board coordinates, any board-fixed axis can
 * be projected into that basis:
 *
 *   north_component = dot(axis, north)
 *   east_component  = dot(axis, east)
 *   heading_deg     = atan2(east_component, north_component)
 *   elevation_deg   = asin(dot(axis, up))
 *
 * The compass page uses the board heading; the directional-antenna UI uses the
 * same basis to report where the antenna axis points. Compile-time axis signs,
 * board yaw rotation, declination, and render offsets adapt the general math to
 * the physical Super IO assembly.
 *
 * Non-goals.
 * This is not a Kalman/Madgwick/Mahony fusion stack and it does not use the
 * ICM-20948 DMP. The gyroscope is currently used for motion classification and
 * telemetry, while compass heading comes from calibrated magnetometer data plus
 * accelerometer tilt compensation.
 */

#include "ICM20948Compass.h"

#if defined(MUZIWORKS_DUO_SUPER_IO) && defined(MUZIWORKS_SUPER_IO_HAS_ICM20948)

#include <math.h>
#include <string.h>
#include <MeshCore.h>
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
#include <InternalFileSystem.h>
#endif

/**
 * Construct with identity calibration and unknown motion/orientation state.
 *
 * Production behavior:
 * - Does not touch I2C or LittleFS; hardware access starts in `begin()`.
 */
ICM20948Compass::ICM20948Compass() {
  resetCalibration(false);
}

/**
 * Probe and initialize the ICM-20948 on a caller-owned I2C bus.
 *
 * Inputs:
 * - `wire`: already configured I2C bus. Super IO uses Wire1 because the main
 *   Wire bus is shared with display/RTC.
 *
 * Production behavior:
 * - Tries AD0 low (0x68) then AD0 high (0x69).
 * - Wakes the IMU and disables low-power mode for deterministic accel/gyro/mag
 *   reads during UI and Smart GPS state updates.
 * - Loads the persisted min/max magnetometer calibration if present.
 */
bool ICM20948Compass::begin(TwoWire& wire) {
  return beginIMU(wire);
}

/**
 * Return the latest compass status if the IMU has produced a fresh sample.
 *
 * Inputs/units:
 * - `reading`: receives heading/pitch/roll in degrees, accel/gyro-derived
 *   motion status, calibration status, and ENU basis vectors.
 *
 * Production behavior:
 * - Performs a throttled `update(false)` first so UI callers do not need to run
 *   their own sensor pump.
 * - Rejects stale samples older than `MUZIWORKS_COMPASS_STALE_MS`.
 */
bool ICM20948Compass::getReading(SensorManager::CompassReading& reading) {
  if (!imu_initialized) return false;
  update(false);
  if (last_imu_update == 0) return false;
  uint32_t age = millis() - last_imu_update;
  if (age > MUZIWORKS_COMPASS_STALE_MS) return false;
  reading = compass;
  reading.age_ms = age;
  return true;
}

/**
 * Set the Smart GPS motion accumulation window used by the classifier.
 *
 * Inputs/units:
 * - `seconds`: profile-selected sustained-motion time before GPS may wake.
 *
 * Production behavior:
 * - Clamps the current accumulation into the new window so switching power
 *   profiles cannot leave a stale "moving" state above the new threshold.
 */
void ICM20948Compass::setMotionWindowSeconds(uint16_t seconds) {
  motion_window_seconds = seconds;
  uint32_t window_ms = (uint32_t)motion_window_seconds * 1000UL;
  if (motion_accum_ms > window_ms) motion_accum_ms = window_ms;
  publishCompassStatus();
}

/**
 * Publish Smart GPS state into the same reading object as compass/motion.
 *
 * Inputs/units:
 * - State values use `SensorManager::SmartGPSState`.
 * - Countdown values are seconds for display only.
 *
 * Production behavior:
 * - Keeps the UI's compass/GPS pages driven by one status struct while leaving
 *   GPS start/stop policy in the board sensor manager.
 */
void ICM20948Compass::setSmartGPSStatus(uint8_t state, uint16_t start_pending_seconds,
                                        uint16_t motion_arming_seconds, uint16_t idle_shutdown_seconds) {
  compass.smart_gps_state = state;
  compass.smart_gps_start_pending_seconds = start_pending_seconds;
  compass.smart_gps_motion_arming_seconds = motion_arming_seconds;
  compass.smart_gps_idle_shutdown_seconds = idle_shutdown_seconds;
}

/**
 * Return the string value used by the generic SensorManager setting API.
 *
 * Production behavior:
 * - Keeps UI/app code decoupled from internal calibration enum names.
 */
const char* ICM20948Compass::calibrationSettingValue() const {
  return calStateName(mag_calibrating ? SensorManager::COMPASS_CAL_CALIBRATING : mag_cal_state);
}

/**
 * Start a timed min/max magnetometer calibration run.
 *
 * Inputs/units:
 * - `seconds`: collection duration. The UI progress and result are derived
 *   from accepted samples and axis coverage, not just elapsed time.
 *
 * Reference:
 * - AN4246 describes hard/soft-iron calibration as fitting the magnetic sample
 *   cloud. This lightweight embedded path collects extrema for a diagonal
 *   approximation to that model.
 */
void ICM20948Compass::startCalibration(uint16_t seconds) {
  mag_cal_state_before_cal = mag_cal_state;
  mag_cal_quality_before_cal = mag_cal_quality;
  for (uint8_t i = 0; i < 3; i++) {
    mag_cal_min[i] = 10000.0f;
    mag_cal_max[i] = -10000.0f;
  }
  mag_cal_samples = 0;
  mag_cal_rejected = 0;
  mag_cal_progress = 0;
  mag_cal_quality = 0;
  mag_cal_step = 1;
  clearCalibrationResult();
  mag_cal_state = SensorManager::COMPASS_CAL_CALIBRATING;
  mag_calibrating = true;
  mag_cal_end_ms = millis() + ((uint32_t)seconds * 1000UL);
  compass.calibration_seconds_remaining = seconds > 255 ? 255 : (uint8_t)seconds;
  publishCompassStatus();
  MESH_DEBUG_PRINTLN("Muzi ICM-20948 compass calibration started for %us", seconds);
}

/**
 * Cancel an active calibration without destroying the previous good result.
 *
 * Production behavior:
 * - Restores the pre-calibration state/quality if one existed.
 * - Publishes a short `CANCELLED` result for UI feedback.
 */
bool ICM20948Compass::cancelCalibration() {
  if (!mag_calibrating) return false;

  mag_calibrating = false;
  mag_cal_end_ms = 0;
  mag_cal_samples = 0;
  mag_cal_rejected = 0;
  mag_cal_step = 0;
  compass.calibration_seconds_remaining = 0;

  if (mag_cal_valid) {
    mag_cal_state = mag_cal_state_before_cal == SensorManager::COMPASS_CAL_CALIBRATING
      ? SensorManager::COMPASS_CAL_LOADED
      : mag_cal_state_before_cal;
    mag_cal_quality = mag_cal_quality_before_cal;
  } else {
    mag_cal_state = mag_cal_state_before_cal == SensorManager::COMPASS_CAL_CALIBRATING
      ? SensorManager::COMPASS_CAL_MISSING
      : mag_cal_state_before_cal;
    mag_cal_quality = 0;
  }
  if (mag_cal_state == SensorManager::COMPASS_CAL_UNKNOWN) {
    mag_cal_state = mag_cal_valid ? SensorManager::COMPASS_CAL_LOADED : SensorManager::COMPASS_CAL_MISSING;
  }

  mag_cal_progress = mag_cal_quality;
  mag_cal_result = SensorManager::COMPASS_CAL_RESULT_CANCELLED;
  mag_cal_final_quality = 0;
  mag_cal_result_until_ms = millis() + MUZIWORKS_COMPASS_CAL_RESULT_HOLD_MS;
  publishCompassStatus();
  MESH_DEBUG_PRINTLN("Muzi ICM-20948 compass calibration cancelled");
  return true;
}

/**
 * Reset in-RAM calibration state and optionally remove the persisted file.
 *
 * Inputs:
 * - `remove_file`: true for a user-requested destructive reset, false for
 *   constructor/init cleanup.
 *
 * Production behavior:
 * - Resets correction to identity: offset = 0, scale = 1.
 * - Uses sentinel extrema so the next accepted sample becomes both min and max.
 */
void ICM20948Compass::resetCalibration(bool remove_file) {
  for (uint8_t i = 0; i < 3; i++) {
    mag_min[i] = 10000.0f;
    mag_max[i] = -10000.0f;
    mag_cal_min[i] = 10000.0f;
    mag_cal_max[i] = -10000.0f;
    mag_offset[i] = 0.0f;
    mag_scale[i] = 1.0f;
  }
  mag_samples = 0;
  mag_cal_samples = 0;
  mag_cal_valid = false;
  mag_calibrating = false;
  mag_cal_end_ms = 0;
  mag_cal_quality = 0;
  mag_cal_progress = 0;
  mag_cal_step = 0;
  mag_cal_state_before_cal = SensorManager::COMPASS_CAL_MISSING;
  mag_cal_quality_before_cal = 0;
  mag_cal_rejected = 0;
  mag_cal_state = remove_file ? SensorManager::COMPASS_CAL_MISSING : mag_cal_state;
  if (remove_file) {
    mag_cal_result = SensorManager::COMPASS_CAL_RESULT_CANCELLED;
    mag_cal_final_quality = 0;
    mag_cal_result_until_ms = millis() + MUZIWORKS_COMPASS_CAL_RESULT_HOLD_MS;
  }
  if (mag_cal_state == SensorManager::COMPASS_CAL_UNKNOWN) {
    mag_cal_state = SensorManager::COMPASS_CAL_MISSING;
  }
  compass.calibration_seconds_remaining = 0;
  publishCompassStatus();

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  if (remove_file) InternalFS.remove(MUZIWORKS_COMPASS_CAL_FILE);
#else
  (void)remove_file;
#endif
}

/**
 * Probe and configure the ICM-20948, then load any persisted calibration.
 *
 * Inputs/units:
 * - `wire`: I2C bus. AD0=false probes 0x68; AD0=true probes 0x69.
 */
bool ICM20948Compass::beginIMU(TwoWire& wire) {
  const bool ad0_trials[] = { false, true };
  for (uint8_t i = 0; i < sizeof(ad0_trials) / sizeof(ad0_trials[0]); i++) {
    imu.begin(wire, ad0_trials[i]);
    if (imu.status == ICM_20948_Stat_Ok) {
      imu_ad0 = ad0_trials[i];
      imu.sleep(false);
      imu.lowPower(false);
      imu_initialized = true;
      if (loadCompassCalibration()) {
        MESH_DEBUG_PRINTLN("Muzi ICM-20948 compass calibration loaded");
      }
      MESH_DEBUG_PRINTLN("Muzi ICM-20948 initialized at 0x%02X", imu_ad0 ? 0x69 : 0x68);
      return true;
    }
  }

  imu_initialized = false;
  MESH_DEBUG_PRINTLN("Muzi ICM-20948 not detected");
  return false;
}

/**
 * Validate that an axis extrema pair is usable for calibration.
 *
 * Inputs/units:
 * - `mn`, `mx`: magnetometer extrema in microtesla.
 *
 * Production behavior:
 * - Rejects NaN, inverted, and under-covered axes. The span threshold is a
 *   practical movement-coverage gate, not an earth-field model.
 */
bool ICM20948Compass::compassRangeValid(float mn, float mx) {
  return (mn == mn) && (mx == mx) && mx > mn && (mx - mn) >= MUZIWORKS_COMPASS_CAL_MIN_SPAN_UT;
}

/**
 * Clamp a floating score into UI percentage range.
 *
 * Inputs/units:
 * - `value`: 0..100 nominal percentage.
 */
uint8_t ICM20948Compass::clampPercent(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 100.0f) return 100;
  return (uint8_t)(value + 0.5f);
}

/**
 * Convert calibration enum state to the persisted setting string.
 */
const char* ICM20948Compass::calStateName(uint8_t state) {
  switch (state) {
    case SensorManager::COMPASS_CAL_LOADED: return "ready";
    case SensorManager::COMPASS_CAL_BAD: return "bad";
    case SensorManager::COMPASS_CAL_CALIBRATING: return "calibrating";
    case SensorManager::COMPASS_CAL_MISSING: return "uncalibrated";
    case SensorManager::COMPASS_CAL_UNKNOWN:
    default: return "unknown";
  }
}

/**
 * Score whether a calibration pass covered enough of the magnetic ellipsoid.
 *
 * Inputs/units:
 * - `mn`, `mx`: per-axis accepted magnetometer extrema in microtesla.
 * - `samples`: accepted sample count.
 *
 * Formula/reference:
 * - AN4246 describes hard/soft-iron calibration as fitting an ellipsoid. This
 *   embedded implementation stores only extrema, so quality is the minimum of:
 *   sample coverage and per-axis span coverage.
 * - axis_score = span / (3 * minimum_span) * 100.
 *
 * Production behavior:
 * - This is a user-guidance/save-threshold score, not a heading accuracy bound.
 */
uint8_t ICM20948Compass::calibrationQualityFor(const float mn[3], const float mx[3], uint16_t samples) const {
  float sample_score = ((float)samples * 100.0f) / (float)MUZIWORKS_COMPASS_CAL_MIN_SAMPLES;
  float span_score = 100.0f;
  for (uint8_t i = 0; i < 3; i++) {
    float span = mx[i] - mn[i];
    float axis_score = (span * 100.0f) / (MUZIWORKS_COMPASS_CAL_MIN_SPAN_UT * 3.0f);
    if (axis_score < span_score) span_score = axis_score;
  }
  float quality = sample_score < span_score ? sample_score : span_score;
  return clampPercent(quality);
}

/**
 * Map final calibration quality to the UI result enum.
 *
 * Inputs/units:
 * - `quality`: 0..100 coverage score.
 */
uint8_t ICM20948Compass::calibrationResultForQuality(uint8_t quality) const {
  if (quality >= MUZIWORKS_COMPASS_CAL_GOOD_QUALITY) return SensorManager::COMPASS_CAL_RESULT_GOOD;
  if (quality >= MUZIWORKS_COMPASS_CAL_FAIR_QUALITY) return SensorManager::COMPASS_CAL_RESULT_FAIR;
  return SensorManager::COMPASS_CAL_RESULT_POOR;
}

/**
 * Derive a simple calibration instruction step from current coverage.
 *
 * Production behavior:
 * - Step 1: collect enough broad X/Y samples.
 * - Step 2: ask for tilt/roll when Z span is still weak.
 * - Step 3: coverage is sufficient; keep moving until timer ends.
 */
uint8_t ICM20948Compass::calibrationStepForCurrentCoverage() const {
  if (!mag_calibrating) return 0;
  if (mag_cal_samples < (MUZIWORKS_COMPASS_CAL_MIN_SAMPLES / 2)) return 1;

  float span_x = mag_cal_max[0] - mag_cal_min[0];
  float span_y = mag_cal_max[1] - mag_cal_min[1];
  float span_z = mag_cal_max[2] - mag_cal_min[2];
  float target_span = MUZIWORKS_COMPASS_CAL_MIN_SPAN_UT * 2.0f;
  if (span_x < target_span || span_y < target_span) return 1;
  if (span_z < target_span) return 2;
  return 3;
}

/**
 * Clear the short-lived terminal calibration result shown by the UI.
 */
void ICM20948Compass::clearCalibrationResult() {
  mag_cal_result = SensorManager::COMPASS_CAL_RESULT_NONE;
  mag_cal_final_quality = 0;
  mag_cal_result_until_ms = 0;
}

/**
 * Copy internal calibration/motion state into `compass`.
 *
 * Production behavior:
 * - Expires terminal calibration result after
 *   `MUZIWORKS_COMPASS_CAL_RESULT_HOLD_MS`.
 * - Keeps GPS-related fields untouched except where target.cpp updates them
 *   through `setSmartGPSStatus`.
 */
void ICM20948Compass::publishCompassStatus() {
  uint32_t now = millis();
  if (mag_cal_result != SensorManager::COMPASS_CAL_RESULT_NONE &&
      mag_cal_result_until_ms != 0 &&
      (int32_t)(now - mag_cal_result_until_ms) >= 0) {
    clearCalibrationResult();
  }
  compass.calibrated = mag_cal_valid;
  compass.calibrating = mag_calibrating;
  compass.calibration_state = mag_calibrating ? SensorManager::COMPASS_CAL_CALIBRATING : mag_cal_state;
  compass.calibration_quality = mag_cal_quality;
  compass.calibration_progress = mag_calibrating ? mag_cal_progress : mag_cal_quality;
  compass.calibration_result = mag_cal_result;
  compass.calibration_final_quality = mag_cal_final_quality;
  compass.calibration_step = mag_calibrating ? mag_cal_step : 0;
  compass.calibration_step_count = mag_calibrating ? MUZIWORKS_COMPASS_CAL_STEP_COUNT : 0;
  compass.calibration_samples = mag_calibrating ? mag_cal_samples : mag_samples;
  compass.motion_state = motion_state;
  compass.motion_score = motion_score;
}

/**
 * Convert collected magnetometer extrema into correction coefficients.
 *
 * Formula/reference:
 * - AN4246 hard-iron offset is the ellipsoid center:
 *   offset_i = (min_i + max_i) / 2.
 * - Full soft-iron correction is a 3x3 matrix. For this constrained firmware
 *   path we store a diagonal scale only:
 *   span_i = max_i - min_i
 *   avg_span = mean(span_x, span_y, span_z)
 *   corrected_i = (raw_i - offset_i) * (avg_span / span_i)
 *
 * Production behavior:
 * - Requires enough samples and span on all axes before marking calibration
 *   loaded. Bad or stale files become `COMPASS_CAL_BAD`.
 */
bool ICM20948Compass::applyCompassCalibrationExtrema() {
  if (mag_samples < MUZIWORKS_COMPASS_CAL_MIN_SAMPLES) return false;
  if (!compassRangeValid(mag_min[0], mag_max[0]) ||
      !compassRangeValid(mag_min[1], mag_max[1]) ||
      !compassRangeValid(mag_min[2], mag_max[2])) {
    return false;
  }

  float span[3] = {
    mag_max[0] - mag_min[0],
    mag_max[1] - mag_min[1],
    mag_max[2] - mag_min[2]
  };
  float avg_span = (span[0] + span[1] + span[2]) / 3.0f;
  for (uint8_t i = 0; i < 3; i++) {
    mag_offset[i] = (mag_min[i] + mag_max[i]) * 0.5f;
    mag_scale[i] = avg_span / span[i];
  }
  mag_cal_valid = true;
  mag_cal_quality = calibrationQualityFor(mag_min, mag_max, mag_samples);
  mag_cal_state = SensorManager::COMPASS_CAL_LOADED;
  publishCompassStatus();
  return true;
}

/**
 * Load the persisted calibration record from LittleFS.
 *
 * Production behavior:
 * - Record is versioned because the extrema/diagonal-scale model is a policy
 *   choice. Any future ellipsoid fit must bump `CAL_VERSION`.
 */
bool ICM20948Compass::loadCompassCalibration() {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  Adafruit_LittleFS_Namespace::File file =
    InternalFS.open(MUZIWORKS_COMPASS_CAL_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ);
  if (!file) {
    mag_cal_state = SensorManager::COMPASS_CAL_MISSING;
    publishCompassStatus();
    return false;
  }

  CalibrationRecord record = {};
  size_t bytes_read = file.read((uint8_t*)&record, sizeof(record));
  file.close();
  if (bytes_read != sizeof(record) ||
      record.magic != CAL_MAGIC ||
      record.version != CAL_VERSION ||
      record.reserved != 0) {
    mag_cal_state = SensorManager::COMPASS_CAL_BAD;
    publishCompassStatus();
    return false;
  }

  for (uint8_t i = 0; i < 3; i++) {
    mag_min[i] = record.min_v[i];
    mag_max[i] = record.max_v[i];
  }
  mag_samples = MUZIWORKS_COMPASS_CAL_MIN_SAMPLES;
  bool ok = applyCompassCalibrationExtrema();
  if (!ok) {
    mag_cal_state = SensorManager::COMPASS_CAL_BAD;
    publishCompassStatus();
  }
  return ok;
#else
  mag_cal_state = SensorManager::COMPASS_CAL_MISSING;
  publishCompassStatus();
  return false;
#endif
}

/**
 * Persist the current calibration extrema to LittleFS.
 *
 * Production behavior:
 * - Removes the previous record before writing the new one so a failed write
 *   cannot look like a valid mixed-version record.
 */
bool ICM20948Compass::saveCompassCalibration() {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  if (!mag_cal_valid) return false;
  mag_cal_quality = calibrationQualityFor(mag_min, mag_max, mag_samples);
  CalibrationRecord record = {
    CAL_MAGIC,
    CAL_VERSION,
    0,
    {mag_min[0], mag_min[1], mag_min[2]},
    {mag_max[0], mag_max[1], mag_max[2]}
  };

  InternalFS.remove(MUZIWORKS_COMPASS_CAL_FILE);
  Adafruit_LittleFS_Namespace::File file =
    InternalFS.open(MUZIWORKS_COMPASS_CAL_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  if (!file) return false;
  size_t written = file.write((const uint8_t*)&record, sizeof(record));
  file.close();
  return written == sizeof(record);
#else
  return false;
#endif
}

/**
 * Finish calibration after the timer expires and publish the terminal result.
 *
 * Production behavior:
 * - Requires minimum accepted sample count and valid span on all axes.
 * - Saves only fair/good results; poor results remain visible briefly but are
 *   not persisted.
 */
void ICM20948Compass::finishCompassCalibration() {
  mag_calibrating = false;
  mag_cal_end_ms = 0;
  compass.calibration_seconds_remaining = 0;
  bool collect_ready = mag_cal_samples >= MUZIWORKS_COMPASS_CAL_MIN_SAMPLES &&
                       compassRangeValid(mag_cal_min[0], mag_cal_max[0]) &&
                       compassRangeValid(mag_cal_min[1], mag_cal_max[1]) &&
                       compassRangeValid(mag_cal_min[2], mag_cal_max[2]);
  if (collect_ready) {
    for (uint8_t i = 0; i < 3; i++) {
      mag_min[i] = mag_cal_min[i];
      mag_max[i] = mag_cal_max[i];
    }
    mag_samples = mag_cal_samples;
  }
  bool applied = collect_ready && applyCompassCalibrationExtrema();
  bool save_ready = applied && mag_cal_quality >= MUZIWORKS_COMPASS_CAL_FAIR_QUALITY;
  if (save_ready && saveCompassCalibration()) {
    mag_cal_result = calibrationResultForQuality(mag_cal_quality);
    mag_cal_final_quality = mag_cal_quality;
    MESH_DEBUG_PRINTLN("Muzi ICM-20948 compass calibration saved");
  } else {
    // The run failed (poor coverage or FS error). Report POOR for the run,
    // but restore the previously persisted calibration instead of leaving the
    // compass UNCALIBRATED until reboot — mirrors cancelCalibration()'s
    // keep-the-last-good-result behavior.
    uint8_t run_quality = calibrationQualityFor(mag_cal_min, mag_cal_max, mag_cal_samples);
    if (!loadCompassCalibration()) {
      mag_cal_valid = false;
      mag_cal_state = SensorManager::COMPASS_CAL_BAD;
      mag_cal_quality = run_quality;
    }
    mag_cal_result = SensorManager::COMPASS_CAL_RESULT_POOR;
    mag_cal_final_quality = run_quality;
    MESH_DEBUG_PRINTLN("Muzi ICM-20948 compass calibration failed (restored=%d)",
                       mag_cal_valid ? 1 : 0);
  }
  mag_cal_result_until_ms = millis() + MUZIWORKS_COMPASS_CAL_RESULT_HOLD_MS;
  mag_cal_step = 0;
  mag_cal_progress = mag_cal_quality;
  publishCompassStatus();
}

/**
 * Rotate sensor X/Y axes around board Z into the Super IO body frame.
 *
 * Inputs/units:
 * - `x`, `y`: same-unit vector components. Used for accel in g, gyro in dps,
 *   and magnetometer in microtesla.
 *
 * Production behavior:
 * - Only compile-time 90-degree multiples are supported. This is board-mount
 *   compensation, not a live attitude rotation.
 */
void ICM20948Compass::applyYawRotation(float& x, float& y) const {
#if MUZIWORKS_IMU_ROTATION_DEG == 90
  float ox = x;
  x = y;
  y = -ox;
#elif MUZIWORKS_IMU_ROTATION_DEG == 180
  x = -x;
  y = -y;
#elif MUZIWORKS_IMU_ROTATION_DEG == 270
  float ox = x;
  x = -y;
  y = ox;
#endif
}

/**
 * Normalize heading into [0, 360) degrees.
 */
float ICM20948Compass::normalizeHeading(float heading) {
  while (heading < 0.0f) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

/**
 * Compute 3D cross product `out = a x b`.
 */
void ICM20948Compass::cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * Compute 3D dot product. The tilt-compensation basis uses dot products to
 * project vectors into/out of the gravity plane.
 */
float ICM20948Compass::dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Clamp a scalar to [-1, 1] before inverse trig.
 *
 * Production behavior:
 * - Prevents `asin` from returning NaN when float roundoff produces 1.0000001.
 */
float ICM20948Compass::clampUnit(float v) {
  if (v < -1.0f) return -1.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

/**
 * Normalize a 3D vector in place.
 *
 * Inputs/units:
 * - Any vector unit; output is unitless length 1.
 *
 * Production behavior:
 * - Rejects near-zero vectors to avoid noisy headings from degenerate magnetic
 *   or body-axis projections.
 */
bool ICM20948Compass::normalize3(float v[3]) {
  float norm = sqrtf(dot3(v, v));
  if (norm < VECTOR_MIN_NORM) return false;
  v[0] /= norm;
  v[1] /= norm;
  v[2] /= norm;
  return true;
}

/**
 * Store the computed ENU basis vectors in the public reading.
 *
 * Inputs:
 * - `east`, `north`, `up`: orthonormal unit vectors in board/body coordinates.
 */
void ICM20948Compass::copyBasisToReading(SensorManager::CompassReading& reading, const float east[3],
                                         const float north[3], const float up[3]) {
  reading.east_x = east[0];
  reading.east_y = east[1];
  reading.east_z = east[2];
  reading.north_x = north[0];
  reading.north_y = north[1];
  reading.north_z = north[2];
  reading.up_x = up[0];
  reading.up_y = up[1];
  reading.up_z = up[2];
}

/**
 * Build a tilt-compensated east/north/up basis from accel + magnetometer.
 *
 * Inputs/units:
 * - `mx/my/mz`: calibrated magnetometer vector in microtesla. Magnitude is not
 *   used after normalization.
 * - `acc_norm`: norm of `acc_g` in g.
 *
 * Formula/reference:
 * - AN4248 tilt compensation can be expressed as vector projection:
 *   up = normalize(accel)
 *   magnetic_north = normalize(mag - up * dot(mag, up))
 *   east = normalize(cross(magnetic_north, up))
 * - Declination and board heading offset rotate magnetic north in the
 *   horizontal plane before rebuilding east.
 *
 * Production behavior:
 * - Uses ENU basis vectors directly instead of Euler roll/pitch heading
 *   formulas. Degenerate vectors fail closed and mark heading invalid.
 */
bool ICM20948Compass::computeOrientationBasis(float mx, float my, float mz, float acc_norm, float east[3],
                                              float north[3], float up[3]) const {
  up[0] = acc_g[0] / acc_norm;
  up[1] = acc_g[1] / acc_norm;
  up[2] = acc_g[2] / acc_norm;
  if (!normalize3(up)) return false;

  float mag[3] = {mx, my, mz};
  if (!normalize3(mag)) return false;

  float mag_dot_up = dot3(mag, up);
  float mag_north[3] = {
    mag[0] - up[0] * mag_dot_up,
    mag[1] - up[1] * mag_dot_up,
    mag[2] - up[2] * mag_dot_up
  };
  if (!normalize3(mag_north)) return false;

  float mag_east[3];
  cross3(mag_north, up, mag_east);
  if (!normalize3(mag_east)) return false;

  const float decl = (MUZIWORKS_COMPASS_DECLINATION_DEG + MUZIWORKS_COMPASS_HEADING_OFFSET_DEG) * DEG2RAD;
  const float cd = cosf(decl);
  const float sd = sinf(decl);
  north[0] = mag_north[0] * cd - mag_east[0] * sd;
  north[1] = mag_north[1] * cd - mag_east[1] * sd;
  north[2] = mag_north[2] * cd - mag_east[2] * sd;
  if (!normalize3(north)) return false;

  cross3(north, up, east);
  if (!normalize3(east)) return false;
  cross3(east, north, up);
  return normalize3(up);
}

/**
 * Project a board-fixed body axis into the ENU basis.
 *
 * Inputs/units:
 * - `body_axis`: unit or non-unit axis in board/body coordinates.
 * - `east/north/up`: orthonormal ENU basis from `computeOrientationBasis`.
 *
 * Formula:
 * - north_component = dot(axis, north)
 * - east_component = dot(axis, east)
 * - heading = atan2(east_component, north_component)
 * - elevation = asin(dot(axis, up))
 *
 * Production behavior:
 * - Rejects axes with too little horizontal projection because heading is
 *   numerically unstable when the axis points mostly up/down.
 */
bool ICM20948Compass::computeBodyAxisHeading(const float body_axis[3], const float east[3], const float north[3],
                                             const float up[3], float& heading_deg, float& elevation_deg) const {
  float axis[3] = {body_axis[0], body_axis[1], body_axis[2]};
  if (!normalize3(axis)) return false;

  float north_component = dot3(axis, north);
  float east_component = dot3(axis, east);
  float horizontal_projection = sqrtf(north_component * north_component + east_component * east_component);
  elevation_deg = asinf(clampUnit(dot3(axis, up))) * RAD2DEG;
  if (horizontal_projection < MUZIWORKS_ANTENNA_MIN_HORIZONTAL_PROJECTION) return false;

  heading_deg = normalizeHeading(atan2f(east_component, north_component) * RAD2DEG);
  return true;
}

/**
 * Classify motion for Smart GPS wake/sleep policy.
 *
 * Inputs/units:
 * - `now`: milliseconds from `millis()`.
 * - `acc_norm`: current accelerometer norm in g.
 *
 * Formula:
 * - accel_delta = norm(current_acc_g - previous_acc_g)
 * - norm_delta = abs(norm(acc_g) - 1g)
 * - gyro_norm = norm(gyro_dps)
 * - instant score is the max of each signal normalized to its threshold:
 *   accel_delta / MUZIWORKS_MOTION_ACCEL_DELTA_G,
 *   norm_delta / MUZIWORKS_MOTION_ACCEL_NORM_DELTA_G,
 *   gyro_norm / MUZIWORKS_MOTION_GYRO_DPS.
 *
 * Production behavior:
 * - A score >= 100 increments sustained-motion accumulation.
 * - Non-motion decays accumulation twice as fast as real time, so brief bumps
 *   do not immediately wake GPS.
 * - This is a power-control heuristic, not inertial navigation.
 */
void ICM20948Compass::updateMotionClassifier(uint32_t now, float acc_norm) {
#if MUZIWORKS_SMART_GPS
  uint32_t dt = last_motion_sample_ms == 0 ? MUZIWORKS_ICM20948_UPDATE_MS : now - last_motion_sample_ms;
  if (dt > (uint32_t)MAX_MOTION_DT_MS) dt = (uint32_t)MAX_MOTION_DT_MS;

  float acc_delta = 0.0f;
  if (last_motion_acc_valid) {
    float dx = acc_g[0] - last_motion_acc_g[0];
    float dy = acc_g[1] - last_motion_acc_g[1];
    float dz = acc_g[2] - last_motion_acc_g[2];
    acc_delta = sqrtf(dx * dx + dy * dy + dz * dz);
  }

  float gyro_norm = sqrtf(gyro_dps[0] * gyro_dps[0] +
                          gyro_dps[1] * gyro_dps[1] +
                          gyro_dps[2] * gyro_dps[2]);
  float norm_delta = fabsf(acc_norm - 1.0f);
  float score = 0.0f;
  float acc_score = (acc_delta * 100.0f) / MUZIWORKS_MOTION_ACCEL_DELTA_G;
  float norm_score = (norm_delta * 100.0f) / MUZIWORKS_MOTION_ACCEL_NORM_DELTA_G;
  float gyro_score = (gyro_norm * 100.0f) / MUZIWORKS_MOTION_GYRO_DPS;
  if (acc_score > score) score = acc_score;
  if (norm_score > score) score = norm_score;
  if (gyro_score > score) score = gyro_score;
  if (!last_motion_acc_valid) score = 0.0f;

  bool moving_now = score >= 100.0f;
  uint32_t window_ms = (uint32_t)motion_window_seconds * 1000UL;
  if (moving_now) {
    uint32_t next_accum = motion_accum_ms + dt;
    motion_accum_ms = next_accum > window_ms ? window_ms : next_accum;
  } else {
    uint32_t decay = dt * 2;
    motion_accum_ms = decay >= motion_accum_ms ? 0 : motion_accum_ms - decay;
  }

  uint8_t accum_score = window_ms == 0 ? 100 : clampPercent(((float)motion_accum_ms * 100.0f) / (float)window_ms);
  uint8_t instant_score = clampPercent(score);
  motion_score = accum_score > instant_score ? accum_score : instant_score;
  if (motion_accum_ms >= window_ms) {
    motion_state = SensorManager::MOTION_MOVING;
  } else if (motion_accum_ms > 0) {
    motion_state = SensorManager::MOTION_PENDING;
  } else {
    motion_state = SensorManager::MOTION_STATIONARY;
  }
#else
  (void)now;
  (void)acc_norm;
  motion_state = SensorManager::MOTION_UNKNOWN;
  motion_score = 0;
#endif

  last_motion_acc_g[0] = acc_g[0];
  last_motion_acc_g[1] = acc_g[1];
  last_motion_acc_g[2] = acc_g[2];
  last_motion_acc_valid = true;
  last_motion_sample_ms = now;
}

/**
 * Poll the IMU, update calibration state, and publish compass/motion reading.
 *
 * Inputs:
 * - `force`: bypasses update interval and data-ready guard.
 *
 * Units:
 * - SparkFun driver accel is mg -> converted to g.
 * - Gyro is degrees/second.
 * - Magnetometer is microtesla.
 *
 * Formula/reference:
 * - Roll/pitch display uses standard accelerometer gravity angles:
 *   roll = atan2(acc_y, acc_z)
 *   pitch = atan2(-acc_x, sqrt(acc_y^2 + acc_z^2))
 * - Heading uses the AN4248 vector basis in `computeOrientationBasis`.
 *
 * Production behavior:
 * - Calibration accepts only plausible magnetic magnitudes.
 * - Heading is marked invalid when uncalibrated, magnetometer magnitude is out
 *   of range, motion is unstable, or orientation projection fails.
 */
bool ICM20948Compass::update(bool force) {
  if (!imu_initialized) return false;

  uint32_t now = millis();
  if (mag_calibrating) {
    long remaining_ms = (long)(mag_cal_end_ms - now);
    if (remaining_ms <= 0) {
      finishCompassCalibration();
    } else {
      compass.calibrating = true;
      uint32_t remaining_s = ((uint32_t)remaining_ms + 999UL) / 1000UL;
      compass.calibration_seconds_remaining = remaining_s > 255 ? 255 : (uint8_t)remaining_s;
    }
  }

  // Signed-diff comparison is millis()-wraparound-safe (matches the idiom
  // used by the other timers in this file).
  if (!force && (int32_t)(now - next_imu_update) < 0) return last_imu_update != 0;
  next_imu_update = now + MUZIWORKS_ICM20948_UPDATE_MS;

  if (!imu.dataReady() && !force) {
    return last_imu_update != 0;
  }

  imu.getAGMT();
  if (imu.status != ICM_20948_Stat_Ok) {
    return false;
  }

  acc_g[0] = imu.accX() / 1000.0f;
  acc_g[1] = imu.accY() / 1000.0f;
  acc_g[2] = imu.accZ() / 1000.0f;
  gyro_dps[0] = imu.gyrX();
  gyro_dps[1] = imu.gyrY();
  gyro_dps[2] = imu.gyrZ();
  mag_ut[0] = imu.magX();
  mag_ut[1] = imu.magY();
  mag_ut[2] = imu.magZ();

  // SparkFun's ICM-20948 DMP examples use this compass mount matrix to align
  // the AK09916 magnetometer frame with the accelerometer/gyro frame.
  mag_ut[0] *= MUZIWORKS_MAG_AXIS_X_SIGN;
  mag_ut[1] *= MUZIWORKS_MAG_AXIS_Y_SIGN;
  mag_ut[2] *= MUZIWORKS_MAG_AXIS_Z_SIGN;

  applyYawRotation(acc_g[0], acc_g[1]);
  applyYawRotation(gyro_dps[0], gyro_dps[1]);
  applyYawRotation(mag_ut[0], mag_ut[1]);

  float mag_norm = sqrtf(mag_ut[0] * mag_ut[0] + mag_ut[1] * mag_ut[1] + mag_ut[2] * mag_ut[2]);
  bool mag_confident = mag_norm >= MUZIWORKS_COMPASS_MIN_MAG_UT &&
                       mag_norm <= MUZIWORKS_COMPASS_MAX_MAG_UT;
  if (mag_calibrating) {
    if (mag_confident) {
      for (uint8_t i = 0; i < 3; i++) {
        if (mag_ut[i] < mag_cal_min[i]) mag_cal_min[i] = mag_ut[i];
        if (mag_ut[i] > mag_cal_max[i]) mag_cal_max[i] = mag_ut[i];
      }
      if (mag_cal_samples < 0xFFFF) mag_cal_samples++;
    } else if (mag_cal_rejected < 0xFFFF) {
      mag_cal_rejected++;
    }
    mag_cal_quality = calibrationQualityFor(mag_cal_min, mag_cal_max, mag_cal_samples);
    mag_cal_progress = mag_cal_quality;
    mag_cal_step = calibrationStepForCurrentCoverage();
  }

  publishCompassStatus();

  float mx = mag_ut[0];
  float my = mag_ut[1];
  float mz = mag_ut[2];
  if (mag_cal_valid) {
    mx = (mx - mag_offset[0]) * mag_scale[0];
    my = (my - mag_offset[1]) * mag_scale[1];
    mz = (mz - mag_offset[2]) * mag_scale[2];
  }

  float acc_norm = sqrtf(acc_g[0] * acc_g[0] + acc_g[1] * acc_g[1] + acc_g[2] * acc_g[2]);
  if (acc_norm < ACCEL_MIN_VALID_NORM_G) return false;
  updateMotionClassifier(now, acc_norm);
  publishCompassStatus();

  float east[3] = {0.0f, 0.0f, 0.0f};
  float north[3] = {0.0f, 0.0f, 0.0f};
  float up[3] = {0.0f, 0.0f, 0.0f};
  bool orientation_valid = mag_confident && mag_cal_valid && computeOrientationBasis(mx, my, mz, acc_norm, east, north, up);
  compass.orientation_valid = orientation_valid;
  if (orientation_valid) {
    copyBasisToReading(compass, east, north, up);
  } else {
    compass.east_x = compass.east_y = compass.east_z = 0.0f;
    compass.north_x = compass.north_y = compass.north_z = 0.0f;
    compass.up_x = compass.up_y = compass.up_z = 0.0f;
  }

  const float compass_axis[3] = {0.0f, 1.0f, 0.0f};
  const float antenna_axis[3] = {
    MUZIWORKS_DIRECTIONAL_AXIS_X,
    MUZIWORKS_DIRECTIONAL_AXIS_Y,
    MUZIWORKS_DIRECTIONAL_AXIS_Z
  };
  float compass_elevation = 0.0f;
  bool heading_valid = orientation_valid && computeBodyAxisHeading(compass_axis, east, north, up,
                                                                   compass.heading_deg,
                                                                   compass_elevation);
  if (!heading_valid) compass.heading_deg = 0.0f;

  compass.antenna_elevation_deg = 0.0f;
  compass.antenna_heading_valid = orientation_valid && computeBodyAxisHeading(antenna_axis, east, north, up,
                                                                              compass.antenna_heading_deg,
                                                                              compass.antenna_elevation_deg);
  float roll = atan2f(acc_g[1], acc_g[2]);
  float pitch = atan2f(-acc_g[0], sqrtf(acc_g[1] * acc_g[1] + acc_g[2] * acc_g[2]));
  compass.roll_deg = roll * RAD2DEG;
  compass.pitch_deg = pitch * RAD2DEG;
  compass.flat = (fabsf(acc_g[2]) / acc_norm) >= MUZIWORKS_COMPASS_FLAT_MIN_Z_RATIO && mag_confident;
  if (!mag_cal_valid) {
    compass.heading_validity = SensorManager::COMPASS_HEADING_UNCALIBRATED;
  } else if (!mag_confident) {
    compass.heading_validity = SensorManager::COMPASS_HEADING_MAG_INVALID;
  } else if (motion_state == SensorManager::MOTION_MOVING) {
    compass.heading_validity = SensorManager::COMPASS_HEADING_MOTION_UNSTABLE;
  } else if (!orientation_valid || !heading_valid) {
    compass.heading_validity = SensorManager::COMPASS_HEADING_ORIENTATION_INVALID;
  } else if (!compass.flat) {
    compass.heading_validity = SensorManager::COMPASS_HEADING_TILTED;
  } else {
    compass.heading_validity = SensorManager::COMPASS_HEADING_VALID;
  }
  compass.age_ms = 0;
  last_imu_update = now;
  publishCompassStatus();
  return true;
}

#endif
