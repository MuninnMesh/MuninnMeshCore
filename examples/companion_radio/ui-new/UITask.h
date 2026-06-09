#pragma once

#include <MeshCore.h>
#include <helpers/ui/DisplayDriver.h>
#include <helpers/ui/UIScreen.h>
#include <helpers/SensorManager.h>
#include <helpers/BaseSerialInterface.h>
#include <Arduino.h>
#include <helpers/sensors/LPPDataHelpers.h>

#ifndef LED_STATE_ON
  #define LED_STATE_ON 1
#endif

#ifdef PIN_BUZZER
  #include <helpers/ui/buzzer.h>
#endif

#if defined(PIN_BUZZER) && defined(PIN_ACTIVE_BUZZER)
  #error "PIN_BUZZER and PIN_ACTIVE_BUZZER are mutually exclusive"
#endif

#ifdef PIN_VIBRATION
  #include <helpers/ui/GenericVibration.h>
#endif

#include "../AbstractUITask.h"
#include "../NodePrefs.h"

class UITask : public AbstractUITask {
  DisplayDriver* _display;
  SensorManager* _sensors;
#ifdef PIN_BUZZER
  genericBuzzer buzzer;
#endif
#ifdef PIN_ACTIVE_BUZZER
  bool active_buzzer_quiet = true;
  unsigned long active_buzzer_until = 0;
  const uint16_t* active_buzzer_pattern = NULL;
  uint8_t active_buzzer_pattern_len = 0;
  uint8_t active_buzzer_pattern_pos = 0;
#endif
#ifdef PIN_VIBRATION
  GenericVibration vibration;
#endif
  unsigned long _next_refresh, _auto_off;
  NodePrefs* _node_prefs;
  char _alert[80];
  unsigned long _alert_expiry;
  int _msgcount;
  unsigned long ui_started_at, next_batt_chck;
  unsigned long next_battery_beep_check;
  unsigned long next_low_battery_beep;
  bool battery_external_known;
  bool battery_external_powered;
  int next_backlight_btn_check = 0;
#ifdef PIN_STATUS_LED
  int led_state = 0;
  int next_led_change = 0;
  int last_led_increment = 0;
#endif
#ifdef PIN_USER_BTN_ANA
  unsigned long _analogue_pin_read_millis = millis();
#endif

  UIScreen* splash;
  UIScreen* home;
  UIScreen* msg_preview;
  UIScreen* curr;

  void userLedHandler();

  // Button action handlers
  char checkDisplayOn(char c);
  char handleLongPress(char c);
  char handleDoubleClick(char c);
  char handleTripleClick(char c);
  char handleCancelPress();
  bool setGPSState(bool enabled, bool persist, bool alert);
#ifdef PIN_ACTIVE_BUZZER
  void activeBuzzerBegin();
  void activeBuzzerPlay(uint16_t duration_ms);
  void activeBuzzerPlayPattern(const uint16_t* pattern, uint8_t pattern_len);
  uint32_t activeBuzzerPatternDuration(const uint16_t* pattern, uint8_t pattern_len);
  void activeBuzzerAdvance();
  void activeBuzzerStop();
  void activeBuzzerLoop();
  void activeBuzzerQuiet(bool quiet);
#endif
#ifdef PIN_MSG_LED
  void setMsgLed(bool on);
#endif
#ifdef UI_HAS_GPS_SWITCH
  bool applyGPSSwitchPolicy(int state, bool alert);
  bool applyGPSSwitchAction(int state, bool alert);
  void showGPSSwitchFeedback(int state);
#endif

  void setCurrScreen(UIScreen* c);
  void batteryBeepHandler();
  void playBatteryBeep();

public:

  UITask(mesh::MainBoard* board, BaseSerialInterface* serial) : AbstractUITask(board, serial), _display(NULL), _sensors(NULL) {
    next_batt_chck = _next_refresh = 0;
    next_battery_beep_check = next_low_battery_beep = 0;
    battery_external_known = false;
    battery_external_powered = false;
    ui_started_at = 0;
    _msgcount = 0;
    curr = NULL;
  }
  void begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs);

  void gotoHomeScreen() { setCurrScreen(home); }
  void showAlert(const char* text, int duration_millis);
  int  getMsgCount() const { return _msgcount; }
  bool hasDisplay() const { return _display != NULL; }
  bool isButtonPressed() const;

  bool isBuzzerQuiet() { 
#ifdef PIN_BUZZER
    return buzzer.isQuiet();
#elif defined(PIN_ACTIVE_BUZZER)
    return active_buzzer_quiet;
#else
    return true;
#endif
  }

  void toggleBuzzer();
  bool getGPSState();
  void toggleGPS();


  // from AbstractUITask
  void msgRead(int msgcount) override;
  void newMsg(uint8_t path_len, const char* from_name, const char* text, int msgcount) override;
  void notify(UIEventType t = UIEventType::none) override;
  void loop() override;

  void shutdown(bool restart = false, const char* message = "Powering off");
};
