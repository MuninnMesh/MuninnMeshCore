#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include "MuziWorksDuoBoard.h"
#if defined(USE_SX1262)
#include <helpers/radiolib/CustomSX1262Wrapper.h>
#else
#include <helpers/radiolib/CustomLR1121Wrapper.h>
#endif
#include <helpers/ArduinoHelpers.h>
#if defined(MUZIWORKS_DUO_SUPER_IO)
#include <helpers/AutoDiscoverRTCClock.h>
#endif
#include <helpers/sensors/EnvironmentSensorManager.h>
#ifdef DISPLAY_CLASS
  #include <helpers/ui/SH1107Display.h>
  #include <helpers/ui/MomentaryButton.h>
#endif

#if defined(MUZIWORKS_DUO_SUPER_IO)
#define UI_HAS_GPS_SWITCH 1

enum GpsSwitchState {
  GPS_SWITCH_UNKNOWN = 0,
  GPS_SWITCH_ON,
  GPS_SWITCH_ON_GPS,
  GPS_SWITCH_OFF
};
#endif

extern MuziWorksDuoBoard board;
extern WRAPPER_CLASS radio_driver;
#if defined(MUZIWORKS_DUO_SUPER_IO)
extern AutoDiscoverRTCClock rtc_clock;
#else
extern VolatileRTCClock rtc_clock;
#endif
extern SensorManager& sensors;
#ifdef DISPLAY_CLASS
extern DISPLAY_CLASS display;
extern MomentaryButton user_btn;
  #if UI_HAS_DPAD
extern MomentaryButton joystick_up;
extern MomentaryButton joystick_down;
  #endif
extern MomentaryButton joystick_left;
extern MomentaryButton joystick_right;
extern MomentaryButton back_btn;
#endif

bool radio_init();
uint32_t radio_get_rng_seed();
void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(int8_t dbm);
mesh::LocalIdentity radio_new_identity();

#if defined(MUZIWORKS_DUO_SUPER_IO)
void gps_switch_begin();
GpsSwitchState gps_switch_get_stable();
bool gps_switch_get_raw(bool& mode1, bool& mode2, GpsSwitchState& state);
bool gps_switch_confirm_initial_state(GpsSwitchState& state, uint32_t normal_ms, uint32_t off_ms);
bool gps_switch_poll(GpsSwitchState& state);
#endif
