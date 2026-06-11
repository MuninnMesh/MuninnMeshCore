#include "UITask.h"
#include <helpers/TxtDataHelpers.h>
#include "../MyMesh.h"
#include "target.h"
#include <math.h>
#ifdef WIFI_SSID
  #include <WiFi.h>
#endif

#ifndef AUTO_OFF_MILLIS
  #define AUTO_OFF_MILLIS     15000   // 15 seconds
#endif
#define BOOT_SCREEN_MILLIS   3000   // 3 seconds

#ifdef PIN_STATUS_LED
#ifndef LED_ON_MILLIS
#define LED_ON_MILLIS     20
#endif
#ifndef LED_ON_MSG_MILLIS
#define LED_ON_MSG_MILLIS 200
#endif
#ifndef LED_CYCLE_MILLIS
#define LED_CYCLE_MILLIS  4000
#endif
#ifndef STATUS_LED_HEARTBEAT_REQUIRES_GPS
#define STATUS_LED_HEARTBEAT_REQUIRES_GPS 0
#endif
#endif

#ifndef MUNINN_CODE_GIT_HASH
#define MUNINN_CODE_GIT_HASH "unknown"
#endif

#define LONG_PRESS_MILLIS   1200

#ifndef ACTIVE_BUZZER_ACK_MS
#define ACTIVE_BUZZER_ACK_MS 6
#endif
#ifndef ACTIVE_BUZZER_MSG_MS
#define ACTIVE_BUZZER_MSG_MS 12
#endif
#ifndef ACTIVE_BUZZER_STARTUP_MS
#define ACTIVE_BUZZER_STARTUP_MS 0
#endif
#ifndef ACTIVE_BUZZER_SHUTDOWN_MS
#define ACTIVE_BUZZER_SHUTDOWN_MS 0
#endif
#ifndef BATTERY_CHARGING_STATE_ON
#define BATTERY_CHARGING_STATE_ON LOW
#endif
#ifndef ACTIVE_BUZZER_GAP_MS
#define ACTIVE_BUZZER_GAP_MS 18
#endif
#ifndef BATTERY_BEEP_MILLIVOLTS
#define BATTERY_BEEP_MILLIVOLTS 0
#endif
#ifndef BATTERY_BEEP_INTERVAL_MS
#define BATTERY_BEEP_INTERVAL_MS 60000
#endif
#ifndef BATTERY_POWER_ATTACH_BEEP
#define BATTERY_POWER_ATTACH_BEEP 0
#endif

#ifdef PIN_ACTIVE_BUZZER
static const uint16_t active_buzzer_startup_pattern[] = {
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_ACK_MS + 4, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_STARTUP_MS
};
static const uint16_t active_buzzer_shutdown_pattern[] = {
  ACTIVE_BUZZER_SHUTDOWN_MS, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS / 2,
  ACTIVE_BUZZER_ACK_MS
};
static const uint16_t active_buzzer_ack_pattern[] = {
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_ACK_MS + 4
};
static const uint16_t active_buzzer_msg_pattern[] = {
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_MSG_MS, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS,
  ACTIVE_BUZZER_MSG_MS + (ACTIVE_BUZZER_ACK_MS / 2)
};
static const uint16_t active_buzzer_battery_pattern[] = {
  ACTIVE_BUZZER_MSG_MS, ACTIVE_BUZZER_GAP_MS / 2,
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS / 2,
  ACTIVE_BUZZER_ACK_MS
};
static const uint16_t active_buzzer_power_pattern[] = {
  ACTIVE_BUZZER_ACK_MS, ACTIVE_BUZZER_GAP_MS / 2,
  ACTIVE_BUZZER_ACK_MS + 8
};
#endif

#ifndef GPS_SWITCH_BOOT_DEBOUNCE_MS
#define GPS_SWITCH_BOOT_DEBOUNCE_MS 200
#endif
#ifndef GPS_SWITCH_BOOT_OFF_CONFIRM_MS
#define GPS_SWITCH_BOOT_OFF_CONFIRM_MS 3000
#endif

#ifndef UI_HAS_COMPASS
#define UI_HAS_COMPASS 0
#endif

#ifndef UI_QUICK_SEND
#define UI_QUICK_SEND 0
#endif

#ifndef UI_DIRECTIONAL_ANTENNA
#define UI_DIRECTIONAL_ANTENNA 0
#endif
#ifndef UI_HEADING_RENDER_OFFSET_DEG
#define UI_HEADING_RENDER_OFFSET_DEG 0.0f
#endif
#ifndef UI_DIRECTIONAL_PING_TIMEOUT_MS
#define UI_DIRECTIONAL_PING_TIMEOUT_MS 2000
#endif
#ifndef UI_DIRECTIONAL_PING_COOLDOWN_MS
#define UI_DIRECTIONAL_PING_COOLDOWN_MS 750
#endif
#ifndef UI_DIRECTIONAL_MAX_TARGETS
#define UI_DIRECTIONAL_MAX_TARGETS REPEATER_DISCOVERY_TABLE_SIZE
#endif
#ifndef UI_REPEATER_DISCOVERY_MS
#define UI_REPEATER_DISCOVERY_MS 60000
#endif
#if UI_DIRECTIONAL_ANTENNA == 1 && UI_QUICK_SEND != 1
#error "UI_DIRECTIONAL_ANTENNA currently requires UI_QUICK_SEND helper support"
#endif

#ifndef UI_MUZIWORKS_SPLASH
#define UI_MUZIWORKS_SPLASH 0
#endif

#ifndef UI_MESHTASTIC_BATTERY_ICON
#define UI_MESHTASTIC_BATTERY_ICON 0
#endif

#ifndef UI_HOME_CLOCK
#define UI_HOME_CLOCK 0
#endif

#ifndef UI_CLOCK_UTC_OFFSET_MINUTES
#define UI_CLOCK_UTC_OFFSET_MINUTES 0
#endif
#ifndef UI_CLOCK_TZ_US_CENTRAL
#define UI_CLOCK_TZ_US_CENTRAL 0
#endif
#ifndef UI_CLOCK_Y
#define UI_CLOCK_Y 0
#endif
#ifndef UI_HOME_ACTION_Y_OFFSET
#define UI_HOME_ACTION_Y_OFFSET 0
#endif
#ifndef UI_COMPASS_LEVEL_MAX_TILT_DEG
#define UI_COMPASS_LEVEL_MAX_TILT_DEG 30.0f
#endif
#ifndef UI_COMPASS_MODE_MAX_TILT_DEG
#define UI_COMPASS_MODE_MAX_TILT_DEG 25.0f
#endif
#ifndef UI_COMPASS_REFRESH_MS
#define UI_COMPASS_REFRESH_MS 100
#endif
// Cadence while the displayed compass values are unchanged (device static).
#ifndef UI_COMPASS_REFRESH_IDLE_MS
#define UI_COMPASS_REFRESH_IDLE_MS 250
#endif

// Battery indicator defaults (see batteryPercent()). BATT_USE_OCV_PERCENT=1
// switches the flat min/max mapping to the BATT_OCV_* discharge-curve lookup.
#ifndef BATT_MIN_MILLIVOLTS
#define BATT_MIN_MILLIVOLTS 3000
#endif
#ifndef BATT_MAX_MILLIVOLTS
#define BATT_MAX_MILLIVOLTS 4200
#endif
#ifndef BATT_USE_OCV_PERCENT
#define BATT_USE_OCV_PERCENT 0
#endif
#ifndef BATT_OCV_100_MILLIVOLTS
#define BATT_OCV_100_MILLIVOLTS 4190
#endif
#ifndef BATT_OCV_90_MILLIVOLTS
#define BATT_OCV_90_MILLIVOLTS 4050
#endif
#ifndef BATT_OCV_80_MILLIVOLTS
#define BATT_OCV_80_MILLIVOLTS 3990
#endif
#ifndef BATT_OCV_70_MILLIVOLTS
#define BATT_OCV_70_MILLIVOLTS 3890
#endif
#ifndef BATT_OCV_60_MILLIVOLTS
#define BATT_OCV_60_MILLIVOLTS 3800
#endif
#ifndef BATT_OCV_50_MILLIVOLTS
#define BATT_OCV_50_MILLIVOLTS 3720
#endif
#ifndef BATT_OCV_40_MILLIVOLTS
#define BATT_OCV_40_MILLIVOLTS 3630
#endif
#ifndef BATT_OCV_30_MILLIVOLTS
#define BATT_OCV_30_MILLIVOLTS 3530
#endif
#ifndef BATT_OCV_20_MILLIVOLTS
#define BATT_OCV_20_MILLIVOLTS 3420
#endif
#ifndef BATT_OCV_10_MILLIVOLTS
#define BATT_OCV_10_MILLIVOLTS 3300
#endif
#ifndef BATT_OCV_0_MILLIVOLTS
#define BATT_OCV_0_MILLIVOLTS 3100
#endif
#ifndef UI_COMPASS_HEADING_ALPHA
#define UI_COMPASS_HEADING_ALPHA 0.18f
#endif
#ifndef UI_COMPASS_TILT_ALPHA
#define UI_COMPASS_TILT_ALPHA 0.24f
#endif
#ifndef UI_COMPASS_MODE_HYSTERESIS_DEG
#define UI_COMPASS_MODE_HYSTERESIS_DEG 5.0f
#endif
#ifndef UI_COMPASS_DISPLAY_DEADBAND_DEG
#define UI_COMPASS_DISPLAY_DEADBAND_DEG 1.5f
#endif
#ifndef UI_COMPASS_FILTER_RESET_MS
#define UI_COMPASS_FILTER_RESET_MS 1200
#endif

#ifndef QUICK_SEND_MAX_CHANNELS
#define QUICK_SEND_MAX_CHANNELS 40
#endif
#ifndef QUICK_SEND_MAX_CONTACTS
#define QUICK_SEND_MAX_CONTACTS 120
#endif
#ifndef QUICK_SEND_MAX_TARGETS
#define QUICK_SEND_MAX_TARGETS 120
#endif
#ifndef QUICK_SEND_LABEL_CHARS
#define QUICK_SEND_LABEL_CHARS 13
#endif

#ifndef MSG_LED_STATE_ON
#define MSG_LED_STATE_ON LED_STATE_ON
#endif

#ifndef UI_RECENT_LIST_SIZE
  #define UI_RECENT_LIST_SIZE 4
#endif

#if UI_HAS_JOYSTICK
  #define PRESS_LABEL "press Enter"
#else
  #define PRESS_LABEL "long press"
#endif

#include "icons.h"
#if UI_MUZIWORKS_SPLASH == 1
#include "muziworks_splash.h"
#endif

#ifndef MUZIWORKS_SPLASH_PHASE_MS
#define MUZIWORKS_SPLASH_PHASE_MS 1000
#endif

#if UI_MUZIWORKS_SPLASH == 1 || UI_MESHTASTIC_BATTERY_ICON == 1
static void drawXbmLsb(DisplayDriver& display, int x, int y, const uint8_t* bits, int w, int h) {
  int width_bytes = (w + 7) / 8;
  for (int by = 0; by < h; by++) {
    for (int bx = 0; bx < w; bx++) {
      if (bits[by * width_bytes + bx / 8] & (1 << (bx & 7))) {
        display.drawLine(x + bx, y + by, x + bx, y + by);
      }
    }
  }
}
#endif

#if UI_HOME_CLOCK == 1 && UI_CLOCK_TZ_US_CENTRAL == 1
static bool isLeapYear(int year) {
  return ((year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0));
}

static uint32_t daysBeforeYear(int year) {
  uint32_t days = 0;
  for (int y = 1970; y < year; y++) {
    days += isLeapYear(y) ? 366 : 365;
  }
  return days;
}

static uint32_t daysBeforeMonth(int year, int month) {
  static const uint8_t month_days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  uint32_t days = 0;
  for (int m = 1; m < month; m++) {
    days += month_days[m - 1];
    if (m == 2 && isLeapYear(year)) days++;
  }
  return days;
}

static int dayOfWeek(int year, int month, int day) {
  // 1970-01-01 was Thursday. Return 0=Sunday.
  return (daysBeforeYear(year) + daysBeforeMonth(year, month) + day - 1 + 4) % 7;
}

static int nthSundayOfMonth(int year, int month, int nth) {
  int first_dow = dayOfWeek(year, month, 1);
  return 1 + ((7 - first_dow) % 7) + ((nth - 1) * 7);
}

static int64_t unixUtcAt(int year, int month, int day, int hour) {
  uint32_t days = daysBeforeYear(year) + daysBeforeMonth(year, month) + day - 1;
  return ((int64_t)days * 86400) + ((int64_t)hour * 3600);
}

static int clockUtcOffsetMinutes(uint32_t now) {
  uint32_t days = now / 86400;
  int year = 1970;
  while (true) {
    uint16_t year_days = isLeapYear(year) ? 366 : 365;
    if (days < year_days) break;
    days -= year_days;
    year++;
  }

  int dst_start_day = nthSundayOfMonth(year, 3, 2);
  int dst_end_day = nthSundayOfMonth(year, 11, 1);
  int64_t dst_start_utc = unixUtcAt(year, 3, dst_start_day, 8);  // 02:00 CST
  int64_t dst_end_utc = unixUtcAt(year, 11, dst_end_day, 7);     // 02:00 CDT
  return ((int64_t)now >= dst_start_utc && (int64_t)now < dst_end_utc) ? -300 : -360;
}
#else
static int clockUtcOffsetMinutes(uint32_t) {
  return UI_CLOCK_UTC_OFFSET_MINUTES;
}
#endif

#if UI_MESHTASTIC_BATTERY_ICON == 1
// Battery icon assets derived from Meshtastic src/graphics/images.h.
static const uint8_t meshtastic_battery_v[] = {
  0b00011100, 0b00111110, 0b01000001, 0b01000001, 0b00000000, 0b00000000,
  0b00000000, 0b01000001, 0b01000001, 0b01000001, 0b00111110
};
static const uint8_t meshtastic_battery_sidegaps_v[] = {0b10000010, 0b10000010, 0b10000010};
static const uint8_t meshtastic_lightning_bolt_v[] = {0b00000100, 0b00000110, 0b00011111, 0b00001100, 0b00000100};
#endif

// --- Geometry / unit constants (shared by the compass + distance/bearing helpers) ---
static constexpr float DEG2RAD = 0.01745329252f;        // pi / 180
static constexpr float RAD2DEG = 57.2957795f;           // 180 / pi
static constexpr float EARTH_RADIUS_MILES = 3958.7613f; // mean Earth radius, miles

class SplashScreen : public UIScreen {
  UITask* _task;
  unsigned long dismiss_after;
#if UI_MUZIWORKS_SPLASH == 1
  unsigned long started_at;
#endif
  char _version_info[12];
  char _muninn_code_info[24];

public:
  SplashScreen(UITask* task) : _task(task) {
    // strip off dash and commit hash by changing dash to null terminator
    // e.g: v1.2.3-abcdef -> v1.2.3
    const char *ver = FIRMWARE_VERSION;
    const char *dash = strchr(ver, '-');

    int len = dash ? dash - ver : strlen(ver);
    if (len >= sizeof(_version_info)) len = sizeof(_version_info) - 1;
    memcpy(_version_info, ver, len);
    _version_info[len] = 0;
    snprintf(_muninn_code_info, sizeof(_muninn_code_info), "MuninnCode %s", MUNINN_CODE_GIT_HASH);

#if UI_MUZIWORKS_SPLASH == 1
    started_at = millis();
    dismiss_after = started_at + (MUZIWORKS_SPLASH_PHASE_MS * 2);
#else
    dismiss_after = millis() + BOOT_SCREEN_MILLIS;
#endif
  }

  int render(DisplayDriver& display) override {
#if UI_MUZIWORKS_SPLASH == 1
    unsigned long elapsed = millis() - started_at;
    if (elapsed < MUZIWORKS_SPLASH_PHASE_MS) {
      display.setColor(DisplayDriver::LIGHT);
      drawXbmLsb(display,
                 (display.width() - MUZIWORKS_SPLASH_WIDTH) / 2,
                 (display.height() - MUZIWORKS_SPLASH_HEIGHT) / 2,
                 muziworks_splash,
                 MUZIWORKS_SPLASH_WIDTH,
                 MUZIWORKS_SPLASH_HEIGHT);
      return MUZIWORKS_SPLASH_PHASE_MS - elapsed;
    }

    // meshcore logo
    display.setColor(DisplayDriver::BLUE);
    int logoWidth = 128;
    display.drawXbm((display.width() - logoWidth) / 2, 3, meshcore_logo, logoWidth, 13);

    // meshcore website
    const char* website = "https://meshcore.io";
    display.setColor(DisplayDriver::LIGHT);
    display.setTextSize(1);
    uint16_t websiteWidth = display.getTextWidth(website);
    display.setCursor((display.width() - websiteWidth) / 2, 30);
    display.print(website);

    // version info
    display.setColor(DisplayDriver::LIGHT);
    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 50, _version_info);

    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 70, FIRMWARE_BUILD_DATE);

    display.drawTextCentered(display.width()/2, 90, _muninn_code_info);
#else
    // meshcore logo
    display.setColor(DisplayDriver::BLUE);
    int logoWidth = 128;
    display.drawXbm((display.width() - logoWidth) / 2, 3, meshcore_logo, logoWidth, 13);

    // version info
    display.setColor(DisplayDriver::LIGHT);
    display.setTextSize(2);
    display.drawTextCentered(display.width()/2, 40, _version_info);

    display.setTextSize(1);
    display.drawTextCentered(display.width()/2, 66, FIRMWARE_BUILD_DATE);

    display.drawTextCentered(display.width()/2, 86, _muninn_code_info);
#endif

    return 1000;
  }

  void poll() override {
    if (millis() >= dismiss_after) {
      _task->gotoHomeScreen();
    }
  }
};

class HomeScreen : public UIScreen {
  enum HomePage {
    FIRST,
    RECENT,
#if UI_QUICK_SEND == 1
    CHANNELS,
    GROUP,
#endif
    RADIO,
    BLUETOOTH,
    ADVERT,
#if ENV_INCLUDE_GPS == 1
    GPS,
#endif
#if UI_HAS_COMPASS == 1
    COMPASS,
#endif
#if UI_DIRECTIONAL_ANTENNA == 1
    DIRECTIONAL,
#endif
#if UI_SENSORS_PAGE == 1
    SENSORS,
#endif
    PROFILE,
    SHUTDOWN,
    Count    // keep as last
  };

  UITask* _task;
  mesh::RTCClock* _rtc;
  SensorManager* _sensors;
  NodePrefs* _node_prefs;
  uint8_t _page;
  bool _shutdown_init;
  AdvertPath recent[UI_RECENT_LIST_SIZE];

  // --- Shared UI layout constants + primitives (SH1107 128x128 panel) ---
  static constexpr int TITLE_Y = 20;   // y baseline of the yellow page title
  static constexpr int ROW_H   = 11;   // vertical pitch of a label/value row

  void drawPageTitle(DisplayDriver& display, const char* title) {
    display.setColor(DisplayDriver::YELLOW);
    display.setTextSize(1);
    display.drawTextCentered(display.width() / 2, TITLE_Y, title);
  }

  // Left-aligned label + right-aligned value on one row; returns the next row y.
  int drawLabelValueRow(DisplayDriver& display, int y, const char* label, const char* value) {
    display.setColor(DisplayDriver::LIGHT);
    display.drawTextLeftAlign(2, y, label);
    display.drawTextRightAlign(display.width() - 1, y, value);
    return y + ROW_H;
  }

#if UI_DIRECTIONAL_ANTENNA == 1
  struct DirectionalTargetRef {
    uint8_t slot;
    float distance_mi;
    bool has_distance;
    int8_t rssi;
    uint32_t heard_at;
  };

  bool directional_discovery_started = false;
  int directional_selected = 0;
  unsigned long directional_last_ping_ms = 0;
  DirectionalTargetRef directional_targets[UI_DIRECTIONAL_MAX_TARGETS];
  int directional_target_count = 0;
  bool directional_dirty = true;   // rebuild the discovery list at most once per frame
#endif

#if UI_QUICK_SEND == 1
  enum QuickSendStage {
    QUICK_TARGETS,
    QUICK_ACTIONS,
    QUICK_KEYBOARD
  };

  enum QuickKeyboardKey {
    QUICK_KB_SHIFT = 1,
    QUICK_KB_DELETE = 2,
    QUICK_KB_SEND = 3
  };

  struct QuickTarget {
    bool is_channel;
    uint16_t idx;
    char prefix[4];
    char name[32];
  };

  uint8_t quick_stage = QUICK_TARGETS;
  int quick_channel_selected = 0;
  int quick_contact_selected = 0;
  int quick_action_selected = 0;
  uint8_t quick_keyboard_row = 0;
  uint8_t quick_keyboard_col = 0;
  bool quick_keyboard_caps = false;
  size_t quick_keyboard_len = 0;
  char quick_keyboard_text[96];
  AdvertPath group_recent[ADVERT_PATH_TABLE_SIZE];
  unsigned long group_recent_refresh_ms = 0;  // next time to re-sort recently-heard
  // Recipient pinned when leaving QUICK_TARGETS. The actions/keyboard stages
  // and the final send resolve against this identity (pubkey for contacts),
  // not the list ordinal — the companion app can add/remove/unfavorite
  // contacts over BLE mid-compose, which shifts every ordinal after the edit.
  bool quick_pinned_valid = false;
  QuickTarget quick_pinned_target = {};
  uint8_t quick_pinned_pubkey[PUB_KEY_SIZE] = {0};

  const char* quickActionLabel(int idx) {
    switch (idx) {
      case 0: return "Send location";
      case 1: return "OK";
      case 2: return "I am safe";
      case 3: return "Need help";
      case 4: return "On my way";
      case 5: return "Heading back";
      case 6: return "At camp";
      case 7: return "Running late";
      case 8: return "Battery low";
      case 9: return "Check in please";
      case 10: return "All clear";
      case 11: return "Testing";
      default: return "";
    }
  }

  int quickPresetActionCount() const { return 12; }

  int quickActionCount() const { return quickPresetActionCount() + 1; }

  int quickMinInt(int lhs, int rhs) const {
    return lhs < rhs ? lhs : rhs;
  }

  int quickChannelLimit() const {
#ifdef MAX_GROUP_CHANNELS
    return quickMinInt(MAX_GROUP_CHANNELS, QUICK_SEND_MAX_CHANNELS);
#else
    return 0;
#endif
  }

  int quickContactLimit() const {
    return quickMinInt(the_mesh.getNumContacts(), QUICK_SEND_MAX_CONTACTS);
  }

  bool isQuickSendContact(const ContactInfo& contact) const {
    if ((contact.flags & 0x01) == 0) return false;
    return contact.type == ADV_TYPE_CHAT || contact.type == ADV_TYPE_NONE;
  }

  bool getQuickChannelTarget(int ordinal, QuickTarget& target) {
    int n = 0;
#ifdef MAX_GROUP_CHANNELS
    int channel_limit = quickChannelLimit();
    for (int i = 0; i < channel_limit && n < QUICK_SEND_MAX_TARGETS; i++) {
      ChannelDetails channel;
      if (the_mesh.getChannel(i, channel) && channel.name[0]) {
        if (n++ == ordinal) {
          target.is_channel = true;
          target.idx = i;
          if (strcmp(channel.name, "Public") == 0) {
            StrHelper::strncpy(target.prefix, "PUB", sizeof(target.prefix));
            StrHelper::strncpy(target.name, channel.name, sizeof(target.name));
          } else if (channel.name[0] == '#') {
            StrHelper::strncpy(target.prefix, "###", sizeof(target.prefix));
            StrHelper::strncpy(target.name, &channel.name[1], sizeof(target.name));
          } else {
            StrHelper::strncpy(target.prefix, "PRV", sizeof(target.prefix));
            StrHelper::strncpy(target.name, channel.name, sizeof(target.name));
          }
          return true;
        }
      }
    }
#endif
    return false;
  }

  int getQuickChannelCount() {
    int n = 0;
#ifdef MAX_GROUP_CHANNELS
    int channel_limit = quickChannelLimit();
    for (int i = 0; i < channel_limit && n < QUICK_SEND_MAX_TARGETS; i++) {
      ChannelDetails channel;
      if (the_mesh.getChannel(i, channel) && channel.name[0]) n++;
    }
#endif
    return n;
  }

  bool getQuickContactTarget(int ordinal, QuickTarget& target) {
    int n = 0;
    int contact_limit = quickContactLimit();
    for (int i = 0; i < contact_limit && n < QUICK_SEND_MAX_TARGETS; i++) {
      ContactInfo contact;
      if (the_mesh.getContactByIdx(i, contact) && isQuickSendContact(contact)) {
        if (n++ == ordinal) {
          target.is_channel = false;
          target.idx = i;
          StrHelper::strncpy(target.prefix, "ND", sizeof(target.prefix));
          StrHelper::strncpy(target.name, contact.name[0] ? contact.name : "Contact", sizeof(target.name));
          return true;
        }
      }
    }
    return false;
  }

  int getQuickContactCount() {
    int n = 0;
    int contact_limit = quickContactLimit();
    for (int i = 0; i < contact_limit && n < QUICK_SEND_MAX_TARGETS; i++) {
      ContactInfo contact;
      if (the_mesh.getContactByIdx(i, contact) && isQuickSendContact(contact)) n++;
    }
    return n;
  }

  bool getCurrentQuickTarget(int ordinal, QuickTarget& target) {
    return _page == HomePage::CHANNELS ? getQuickChannelTarget(ordinal, target) : getQuickContactTarget(ordinal, target);
  }

  // Capture the currently highlighted target as the pinned recipient.
  bool pinQuickTarget() {
    quick_pinned_valid = false;
    if (!getCurrentQuickTarget(currentQuickSelected(), quick_pinned_target)) return false;
    if (!quick_pinned_target.is_channel) {
      ContactInfo contact;
      if (!the_mesh.getContactByIdx(quick_pinned_target.idx, contact)) return false;
      memcpy(quick_pinned_pubkey, contact.id.pub_key, PUB_KEY_SIZE);
    }
    quick_pinned_valid = true;
    return true;
  }

  // Re-resolve the pinned recipient by identity. For contacts the slot is
  // re-validated against the pinned pubkey (and re-found if the table shifted);
  // returns false if the contact no longer exists.
  bool resolvePinnedQuickTarget(QuickTarget& target) {
    if (!quick_pinned_valid) return false;
    target = quick_pinned_target;
    if (target.is_channel) {
      ChannelDetails channel;
      return the_mesh.getChannel(target.idx, channel) && channel.name[0] != 0;
    }
    ContactInfo contact;
    if (the_mesh.getContactByIdx(target.idx, contact) &&
        memcmp(contact.id.pub_key, quick_pinned_pubkey, PUB_KEY_SIZE) == 0) {
      return true;
    }
    int num = the_mesh.getNumContacts();
    for (int i = 0; i < num; i++) {
      if (the_mesh.getContactByIdx(i, contact) &&
          memcmp(contact.id.pub_key, quick_pinned_pubkey, PUB_KEY_SIZE) == 0) {
        target.idx = i;
        quick_pinned_target.idx = i;
        return true;
      }
    }
    return false;
  }

  int getCurrentQuickTargetCount() {
    return _page == HomePage::CHANNELS ? getQuickChannelCount() : getQuickContactCount();
  }

  int& currentQuickSelected() {
    return _page == HomePage::CHANNELS ? quick_channel_selected : quick_contact_selected;
  }

  void clampQuickSendSelection() {
    int count = getCurrentQuickTargetCount();
    int& selected = currentQuickSelected();
    if (count <= 0) {
      selected = 0;
    } else if (selected >= count) {
      selected = count - 1;
    }
    if (quick_action_selected >= quickActionCount()) {
      quick_action_selected = quickActionCount() - 1;
    }
  }

  void copyQuickDisplayChars(char* dest, const char* src, size_t dest_size) {
    size_t len = 0;
    while (src[len] && len < QUICK_SEND_LABEL_CHARS && len < dest_size - 1) {
      dest[len] = src[len];
      len++;
    }
    dest[len] = 0;
  }

  void formatQuickTarget(DisplayDriver& display, const QuickTarget& target, char* dest, size_t dest_size) {
    char filtered_name[sizeof(target.name)];
    char trimmed_name[sizeof(target.name)];
    display.translateUTF8ToBlocks(filtered_name, target.name, sizeof(filtered_name));
    copyQuickDisplayChars(trimmed_name, filtered_name, sizeof(trimmed_name));
    snprintf(dest, dest_size, "[%s] %s", target.prefix, trimmed_name);
  }

  void formatQuickAction(int idx, char* dest, size_t dest_size) {
    if (idx == 0) {
      StrHelper::strncpy(dest, "[[KEYBOARD]]", dest_size);
      return;
    }
    copyQuickDisplayChars(dest, quickActionLabel(idx - 1), dest_size);
  }

  bool getRecentForContact(const ContactInfo& contact, AdvertPath& found) {
    for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
      AdvertPath* path = &group_recent[i];
      if (path->recv_timestamp != 0 &&
          memcmp(path->pubkey_prefix, contact.id.pub_key, sizeof(path->pubkey_prefix)) == 0) {
        found = *path;
        return true;
      }
    }
    return false;
  }

  uint8_t rssiBars(bool heard, int8_t rssi) {
    if (!heard) return 0;
    if (rssi >= -75) return 5;
    if (rssi >= -90) return 4;
    if (rssi >= -105) return 3;
    if (rssi >= -118) return 2;
    return 1;
  }

  void drawSignalBars(DisplayDriver& display, int x, int y, uint8_t bars) {
    const uint8_t heights[] = {2, 3, 5, 7, 9};
    int bottom = y + 9;
    for (uint8_t i = 0; i < sizeof(heights) / sizeof(heights[0]); i++) {
      int bx = x + i * 3;
      int h = heights[i];
      if (i < bars) {
        display.fillRect(bx, bottom - h + 1, 2, h);
      } else {
        display.drawLine(bx, bottom, bx + 1, bottom);
      }
    }
  }

  bool hasLatLon(double lat, double lon) const {
    return lat != 0.0 || lon != 0.0;
  }

  // Wrap an angle into [0, 360).
  float normalizeDeg(float deg) const {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
  }

  // Great-circle distance in miles via the haversine formula.
  //   a = sin^2(dphi/2) + cos(phi1) cos(phi2) sin^2(dlambda/2)
  //   d = 2 R atan2(sqrt(a), sqrt(1-a))
  // Used for contact/repeater distance readouts. Ref: https://en.wikipedia.org/wiki/Haversine_formula
  float distanceMiles(double lat1, double lon1, double lat2, double lon2) const {
    float p1 = lat1 * DEG2RAD;
    float p2 = lat2 * DEG2RAD;
    float dp = (lat2 - lat1) * DEG2RAD;
    float dl = (lon2 - lon1) * DEG2RAD;
    float a = sinf(dp * 0.5f) * sinf(dp * 0.5f) +
              cosf(p1) * cosf(p2) * sinf(dl * 0.5f) * sinf(dl * 0.5f);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return EARTH_RADIUS_MILES * c;
  }

  bool getDistanceFromLocal(double target_lat, double target_lon, float& mi) const {
    if (_sensors == NULL || !hasLatLon(_sensors->node_lat, _sensors->node_lon) ||
        !hasLatLon(target_lat, target_lon)) {
      return false;
    }
    mi = distanceMiles(_sensors->node_lat, _sensors->node_lon, target_lat, target_lon);
    return true;
  }

  // Initial great-circle bearing (forward azimuth) from point 1 to point 2, degrees [0,360).
  //   theta = atan2( sin(dlambda) cos(phi2),
  //                  cos(phi1) sin(phi2) - sin(phi1) cos(phi2) cos(dlambda) )
  // Drives the direction arrows to contacts/repeaters. Ref: https://www.movable-type.co.uk/scripts/latlong.html
  float bearingDeg(double lat1, double lon1, double lat2, double lon2) const {
    float p1 = lat1 * DEG2RAD;
    float p2 = lat2 * DEG2RAD;
    float dl = (lon2 - lon1) * DEG2RAD;
    float y = sinf(dl) * cosf(p2);
    float x = cosf(p1) * sinf(p2) - sinf(p1) * cosf(p2) * cosf(dl);
    return normalizeDeg(atan2f(y, x) * RAD2DEG);
  }

  void formatDistanceMiles(bool valid, float mi, char* dest, size_t dest_size) {
    if (!valid) {
      StrHelper::strncpy(dest, "--", dest_size);
      return;
    }
    if (mi > 100.0f) {
      StrHelper::strncpy(dest, "N/A", dest_size);
      return;
    }
    if (mi < 10.0f) {
      snprintf(dest, dest_size, "%.1f", mi);
    } else {
      snprintf(dest, dest_size, "%.0f", mi);
    }
  }

  void formatDistanceTo(double target_lat, double target_lon, char* dest, size_t dest_size) {
    float mi = 0.0f;
    formatDistanceMiles(getDistanceFromLocal(target_lat, target_lon, mi), mi, dest, dest_size);
  }

  void formatDistance(const ContactInfo& contact, char* dest, size_t dest_size) {
    formatDistanceTo(contact.gps_lat / 1000000.0, contact.gps_lon / 1000000.0, dest, dest_size);
  }

  // Variant for list renderers: takes a pre-fetched compass reading so a
  // 7-row list does one ~130-byte getCompass() copy per frame, not per row.
  bool getRelativeDirectionWith(const SensorManager::CompassReading& compass, bool compass_ok,
                                double target_lat, double target_lon, float& relative) {
    if (!compass_ok || _sensors == NULL || !hasLatLon(_sensors->node_lat, _sensors->node_lon) ||
        !hasLatLon(target_lat, target_lon)) {
      return false;
    }
    float bearing = bearingDeg(_sensors->node_lat, _sensors->node_lon, target_lat, target_lon);
    relative = normalizeDeg(bearing - normalizeDeg(compass.heading_deg + UI_HEADING_RENDER_OFFSET_DEG));
    return true;
  }

  bool getRelativeDirectionTo(double target_lat, double target_lon, float& relative) {
    SensorManager::CompassReading compass;
    bool compass_ok = _sensors != NULL && _sensors->getCompass(compass) && compass.flat;
    return getRelativeDirectionWith(compass, compass_ok, target_lat, target_lon, relative);
  }


  // Small arrow pointing at relative_deg (0 = straight up/ahead). Screen frame: x = sin, y = -cos.
  void drawDirectionArrow(DisplayDriver& display, int cx, int cy, float relative_deg) {
    float rad = relative_deg * DEG2RAD;
    int tip_x = cx + (int)(sinf(rad) * 5.0f);
    int tip_y = cy - (int)(cosf(rad) * 5.0f);
    int tail_x = cx - (int)(sinf(rad) * 3.0f);
    int tail_y = cy + (int)(cosf(rad) * 3.0f);
    display.drawLine(tail_x, tail_y, tip_x, tip_y);

    // arrowhead: two short barbs at +/-145 deg from the shaft direction
    float left = (relative_deg + 145.0f) * DEG2RAD;
    float right = (relative_deg - 145.0f) * DEG2RAD;
    display.drawLine(tip_x, tip_y, tip_x + (int)(sinf(left) * 3.0f), tip_y - (int)(cosf(left) * 3.0f));
    display.drawLine(tip_x, tip_y, tip_x + (int)(sinf(right) * 3.0f), tip_y - (int)(cosf(right) * 3.0f));
  }

  void drawRowMarker(DisplayDriver& display, int y, bool selected) {
    if (!selected) return;
    y -= 2;
    int right = display.width() - 1;
    display.fillRect(0, y + 5, 2, 2);
    display.fillRect(right - 1, y + 5, 2, 2);
  }

  char quickKeyboardChar(uint8_t row, uint8_t col) const {
    if (row == 0) return "QWERTYUIOP"[col];
    if (row == 1) return "ASDFGHJKL "[col];
    if (row == 2) return "ZXCVBNM,-."[col];
    if (row == 3) return "0123456789"[col];
    const char keys[10] = {QUICK_KB_SHIFT, '@', '!', '?', '#', '\'', ':', '/', QUICK_KB_DELETE, QUICK_KB_SEND};
    return keys[col];
  }

  void quickKeyboardLabel(char key, char* dest, size_t dest_size) const {
    if (key == QUICK_KB_SHIFT) {
      StrHelper::strncpy(dest, quick_keyboard_caps ? "aa" : "Aa", dest_size);
    } else if (key == QUICK_KB_DELETE) {
      StrHelper::strncpy(dest, "BS", dest_size);
    } else if (key == QUICK_KB_SEND) {
      StrHelper::strncpy(dest, "TX", dest_size);
    } else if (key == ' ') {
      StrHelper::strncpy(dest, "_", dest_size);
    } else {
      dest[0] = quick_keyboard_caps && key >= 'A' && key <= 'Z' ? (char)(key + 32) : key;
      dest[1] = 0;
    }
  }

  void resetQuickKeyboard() {
    quick_keyboard_row = 0;
    quick_keyboard_col = 0;
    quick_keyboard_caps = false;
    quick_keyboard_len = 0;
    quick_keyboard_text[0] = 0;
  }

  void appendQuickKeyboardChar(char key) {
    if (quick_keyboard_len >= sizeof(quick_keyboard_text) - 1) {
      _task->showAlert("Message full", 700);
      return;
    }
    if (quick_keyboard_caps && key >= 'A' && key <= 'Z') key = (char)(key + 32);
    quick_keyboard_text[quick_keyboard_len++] = key;
    quick_keyboard_text[quick_keyboard_len] = 0;
  }

  void deleteQuickKeyboardChar() {
    if (quick_keyboard_len == 0) return;
    quick_keyboard_text[--quick_keyboard_len] = 0;
  }

  void sendQuickKeyboardMessage() {
    if (quick_keyboard_len == 0) {
      _task->showAlert("Type message", 800);
      return;
    }

    QuickTarget target;
    if (!resolvePinnedQuickTarget(target)) {
      _task->showAlert("No chat", 800);
      resetQuickSendStage();
      return;
    }

    int result = sendQuickMessage(target, quick_keyboard_text);
    if (result == MSG_SEND_FAILED) {
      _task->showAlert("Send failed", 1000);
      return;
    }
    _task->notify(UIEventType::ack);
    _task->showAlert(result == MSG_SEND_SENT_DIRECT ? "Sent direct" : "Sent flood", 1000);
    resetQuickKeyboard();
    quick_stage = QUICK_TARGETS;
  }

  bool handleQuickKeyboardInput(char c) {
    const uint8_t rows = 5;
    const uint8_t cols = 10;
    if (c == KEY_UP || c == KEY_PREV) {
      quick_keyboard_row = (quick_keyboard_row + rows - 1) % rows;
      return true;
    }
    if (c == KEY_DOWN || c == KEY_NEXT) {
      quick_keyboard_row = (quick_keyboard_row + 1) % rows;
      return true;
    }
    if (c == KEY_LEFT) {
      quick_keyboard_col = (quick_keyboard_col + cols - 1) % cols;
      return true;
    }
    if (c == KEY_RIGHT) {
      quick_keyboard_col = (quick_keyboard_col + 1) % cols;
      return true;
    }
    if (c == KEY_CANCEL) {
      if (quick_keyboard_len > 0) {
        deleteQuickKeyboardChar();
      } else {
        quick_stage = QUICK_ACTIONS;
      }
      return true;
    }
    if (c == KEY_ENTER) {
      char key = quickKeyboardChar(quick_keyboard_row, quick_keyboard_col);
      if (key == QUICK_KB_SHIFT) {
        quick_keyboard_caps = !quick_keyboard_caps;
      } else if (key == QUICK_KB_DELETE) {
        deleteQuickKeyboardChar();
      } else if (key == QUICK_KB_SEND) {
        sendQuickKeyboardMessage();
      } else {
        appendQuickKeyboardChar(key);
      }
      return true;
    }
    return true;
  }

  void renderQuickKeyboard(DisplayDriver& display) {
    QuickTarget target;
    if (!resolvePinnedQuickTarget(target)) {
      resetQuickSendStage();
      if (_page == HomePage::CHANNELS) renderChannelList(display);
      else renderGroupList(display);
      return;
    }

    char label[70];
    char typed[24];
    formatQuickTarget(display, target, label, sizeof(label));

    size_t start = quick_keyboard_len > 18 ? quick_keyboard_len - 18 : 0;
    StrHelper::strncpy(typed, &quick_keyboard_text[start], sizeof(typed));
    // Guard on the VISIBLE length (<=18), not the total typed length — the
    // cursor must not disappear once the message grows past the window.
    size_t visible = quick_keyboard_len - start;
    if (visible + 1 < sizeof(typed)) {
      typed[visible] = '_';
      typed[visible + 1] = 0;
    }

    display.setTextSize(1);
    display.setColor(DisplayDriver::YELLOW);
    display.drawTextCentered(display.width() / 2, 18, "Keyboard");
    display.setColor(DisplayDriver::LIGHT);
    display.drawTextEllipsized(6, 30, display.width() - 12, label);
    display.setColor(DisplayDriver::GREEN);
    display.drawTextEllipsized(6, 43, display.width() - 12, typed);
    display.setColor(DisplayDriver::LIGHT);
    display.drawLine(4, 56, display.width() - 5, 56);

    const int cell_w = 12;
    const int cell_h = 12;
    const int start_x = (display.width() - cell_w * 10) / 2;
    const int start_y = 62;
    for (uint8_t row = 0; row < 5; row++) {
      for (uint8_t col = 0; col < 10; col++) {
        int x = start_x + col * cell_w;
        int y = start_y + row * cell_h;
        bool selected = row == quick_keyboard_row && col == quick_keyboard_col;
        char key = quickKeyboardChar(row, col);
        char key_label[4];
        quickKeyboardLabel(key, key_label, sizeof(key_label));
        display.setColor(selected ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
        if (selected) {
          display.drawRect(x, y, cell_w - 1, cell_h - 1);
          display.fillRect(x + 1, y + 1, cell_w - 3, cell_h - 3);
          display.setColor(DisplayDriver::DARK);
        }
        display.drawTextCentered(x + cell_w / 2, y + 2, key_label);
      }
    }
  }

  void renderChannelList(DisplayDriver& display) {
    display.setTextSize(1);
    display.setColor(DisplayDriver::YELLOW);
    display.drawTextCentered(display.width() / 2, 20, "Text Channels");

    int count = getQuickChannelCount();
    if (count == 0) {
      display.setColor(DisplayDriver::RED);
      display.drawTextCentered(display.width() / 2, 52, "No text channels");
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, 66, "Add in app");
      return;
    }

    clampQuickSendSelection();
    const int visible = 6;
    int first = quick_channel_selected - 2;
    if (first < 0) first = 0;
    if (first > count - visible) first = count > visible ? count - visible : 0;

    int y = 34;
    for (int row = 0; row < visible && first + row < count; row++, y += 12) {
      QuickTarget target;
      char label[70];
      bool selected = first + row == quick_channel_selected;
      if (!getQuickChannelTarget(first + row, target)) continue;  // don't format an unset target
      formatQuickTarget(display, target, label, sizeof(label));
      display.setColor(selected ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
      drawRowMarker(display, y, selected);
      display.drawTextLeftAlign(10, y, label);
    }

    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 116, "Up/Down  Center");
  }

  void renderGroupList(DisplayDriver& display) {
    display.setTextSize(1);
    display.setColor(DisplayDriver::YELLOW);
    display.drawTextCentered(display.width() / 2, 20, "Quick Chat");

    int count = getQuickContactCount();
    if (count == 0) {
      display.setColor(DisplayDriver::RED);
      display.drawTextCentered(display.width() / 2, 52, "No favorites");
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, 66, "Favorite nodes");
      return;
    }

    clampQuickSendSelection();
    // getRecentlyHeard() qsorts the live advert table — refresh at most every
    // 2s instead of every 500ms frame; row redraws don't need fresher data.
    unsigned long now_ms = millis();
    if (group_recent_refresh_ms == 0 || (long)(now_ms - group_recent_refresh_ms) >= 0) {
      the_mesh.getRecentlyHeard(group_recent, ADVERT_PATH_TABLE_SIZE);
      group_recent_refresh_ms = now_ms + 2000;
    }
    // One compass fetch for all rows in this frame.
    SensorManager::CompassReading row_compass;
    bool row_compass_ok = _sensors != NULL && _sensors->getCompass(row_compass) && row_compass.flat;

    const int visible = 7;
    int first = quick_contact_selected - 3;
    if (first < 0) first = 0;
    if (first > count - visible) first = count > visible ? count - visible : 0;

    int y = 34;
    for (int row = 0; row < visible && first + row < count; row++, y += 12) {
      QuickTarget target;
      if (!getQuickContactTarget(first + row, target)) continue;

      ContactInfo contact;
      if (!the_mesh.getContactByIdx(target.idx, contact)) continue;

      AdvertPath recent_path;
      bool heard = getRecentForContact(contact, recent_path);

      char filtered_name[sizeof(target.name)];
      char short_name[10];
      display.translateUTF8ToBlocks(filtered_name, target.name, sizeof(filtered_name));
      size_t len = 0;
      while (filtered_name[len] && len < sizeof(short_name) - 1) {
        short_name[len] = filtered_name[len];
        len++;
      }
      short_name[len] = 0;

      char dist[6];
      float relative_dir = 0.0f;
      bool has_dir = getRelativeDirectionWith(row_compass, row_compass_ok,
                                              contact.gps_lat / 1000000.0,
                                              contact.gps_lon / 1000000.0, relative_dir);
      formatDistance(contact, dist, sizeof(dist));

      bool selected = first + row == quick_contact_selected;
      display.setColor(selected ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
      drawRowMarker(display, y, selected);
      drawSignalBars(display, 10, y - 2, rssiBars(heard, recent_path.last_rssi));
      display.drawTextLeftAlign(28, y, short_name);
      if (has_dir) {
        drawDirectionArrow(display, 94, y + 4, relative_dir);
      }
      display.drawTextRightAlign(118, y, dist);
    }

    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 116, "Center send");
  }

  void renderQuickActionList(DisplayDriver& display) {
    QuickTarget target;
    if (!resolvePinnedQuickTarget(target)) {
      resetQuickSendStage();
      if (_page == HomePage::CHANNELS) renderChannelList(display);
      else renderGroupList(display);
      return;
    }

    display.setTextSize(1);
    char label[70];
    formatQuickTarget(display, target, label, sizeof(label));
    display.setColor(DisplayDriver::YELLOW);
    display.drawTextEllipsized(6, 20, display.width() - 12, label);
    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 32, "Send:");

    const int count = quickActionCount();
    const int visible = 6;
    int first = quick_action_selected - 2;
    if (first < 0) first = 0;
    if (first > count - visible) first = count > visible ? count - visible : 0;

    int y = 44;
    for (int row = 0; row < visible && first + row < count; row++, y += 12) {
      char action[32];
      bool selected = first + row == quick_action_selected;
      formatQuickAction(first + row, action, sizeof(action));
      display.setColor(selected ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
      drawRowMarker(display, y, selected);
      display.drawTextEllipsized(10, y, display.width() - 20, action);
    }

    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 116, "Center send  Back");
  }

  bool makeQuickLocationMessage(char* dest, size_t dest_size) {
    LocationProvider* nmea = _sensors != NULL ? _sensors->getLocationProvider() : NULL;
    if (nmea != NULL && nmea->isValid()) {
      snprintf(dest, dest_size, "My location: %.5f, %.5f alt %.0fm",
               nmea->getLatitude() / 1000000.0,
               nmea->getLongitude() / 1000000.0,
               nmea->getAltitude() / 1000.0);
      return true;
    }
    if (_sensors != NULL && (_sensors->node_lat != 0.0 || _sensors->node_lon != 0.0)) {
      snprintf(dest, dest_size, "Last location: %.5f, %.5f",
               _sensors->node_lat,
               _sensors->node_lon);
      return true;
    }
    return false;
  }

  int sendQuickMessage(const QuickTarget& target, const char* text) {
    uint32_t timestamp = the_mesh.getRTCClock()->getCurrentTimeUnique();
    if (target.is_channel) {
      ChannelDetails channel;
      if (!the_mesh.getChannel(target.idx, channel) || channel.name[0] == 0) return MSG_SEND_FAILED;
      return the_mesh.sendGroupMessage(timestamp, channel.channel, _node_prefs->node_name, text, strlen(text))
        ? MSG_SEND_SENT_FLOOD
        : MSG_SEND_FAILED;
    }

    ContactInfo contact;
    if (!the_mesh.getContactByIdx(target.idx, contact)) return MSG_SEND_FAILED;
    uint32_t expected_ack;
    uint32_t est_timeout;
    return the_mesh.sendMessage(contact, timestamp, 0, text, expected_ack, est_timeout);
  }

  void sendQuickSelection() {
    QuickTarget target;
    if (!resolvePinnedQuickTarget(target)) {
      _task->showAlert("No chat", 800);
      resetQuickSendStage();
      return;
    }

    char msg[96];
    if (quick_action_selected == 0) {
      resetQuickKeyboard();
      quick_stage = QUICK_KEYBOARD;
      return;
    } else if (quick_action_selected == 1) {
      if (!makeQuickLocationMessage(msg, sizeof(msg))) {
        _task->showAlert("No location", 1000);
        return;
      }
    } else {
      StrHelper::strncpy(msg, quickActionLabel(quick_action_selected - 1), sizeof(msg));
    }

    int result = sendQuickMessage(target, msg);
    if (result == MSG_SEND_FAILED) {
      _task->showAlert("Send failed", 1000);
    } else {
      _task->notify(UIEventType::ack);
      _task->showAlert(result == MSG_SEND_SENT_DIRECT ? "Sent direct" : "Sent flood", 1000);
    }
  }

#if UI_DIRECTIONAL_ANTENNA == 1
  bool directionalComesBefore(const DirectionalTargetRef& lhs, const DirectionalTargetRef& rhs) const {
    if (lhs.has_distance != rhs.has_distance) return lhs.has_distance;
    if (lhs.has_distance && lhs.distance_mi != rhs.distance_mi) return lhs.distance_mi < rhs.distance_mi;
    if (lhs.rssi != rhs.rssi) return lhs.rssi > rhs.rssi;
    return lhs.heard_at > rhs.heard_at;
  }

  void insertDirectionalTarget(uint8_t slot, const RepeaterDiscoveryInfo& hit, float distance_mi, bool has_distance) {
    DirectionalTargetRef next;
    next.slot = slot;
    next.distance_mi = distance_mi;
    next.has_distance = has_distance;
    next.rssi = hit.last_rssi;
    next.heard_at = hit.recv_timestamp;

    int pos = directional_target_count;
    if (pos < UI_DIRECTIONAL_MAX_TARGETS) {
      directional_target_count++;
    } else {
      pos = UI_DIRECTIONAL_MAX_TARGETS - 1;
      if (!directionalComesBefore(next, directional_targets[pos])) return;
    }

    while (pos > 0 && directionalComesBefore(next, directional_targets[pos - 1])) {
      directional_targets[pos] = directional_targets[pos - 1];
      pos--;
    }

    directional_targets[pos] = next;
  }

  void refreshDirectionalTargets() {
    if (!directional_dirty) return;   // already rebuilt this frame
    directional_dirty = false;
    directional_target_count = 0;
    if (!directional_discovery_started) return;

    double local_lat = 0.0;
    double local_lon = 0.0;
    bool have_local = getLocalLatLon(local_lat, local_lon);
    for (uint8_t i = 0; i < REPEATER_DISCOVERY_TABLE_SIZE; i++) {
      RepeaterDiscoveryInfo hit;
      if (!the_mesh.getDiscoveredRepeaterBySlot(i, hit)) continue;
      bool has_distance = have_local && hasLatLon(hit.gps_lat, hit.gps_lon);
      float dist = has_distance
        ? distanceMiles(local_lat, local_lon, hit.gps_lat / 1000000.0, hit.gps_lon / 1000000.0)
        : 0.0f;
      insertDirectionalTarget(i, hit, dist, has_distance);
    }
  }

  int getDirectionalTargetCount() {
    refreshDirectionalTargets();
    return directional_target_count;
  }

  bool getDirectionalTarget(int ordinal, RepeaterDiscoveryInfo& target, DirectionalTargetRef* ref = NULL) {
    refreshDirectionalTargets();
    if (ordinal < 0 || ordinal >= directional_target_count) return false;
    if (ref != NULL) *ref = directional_targets[ordinal];
    return the_mesh.getDiscoveredRepeaterBySlot(directional_targets[ordinal].slot, target);
  }

  void clampDirectionalSelection() {
    int count = getDirectionalTargetCount();
    if (count <= 0) {
      directional_selected = 0;
    } else if (directional_selected >= count) {
      directional_selected = count - 1;
    }
  }

  bool getLocalLatLon(double& lat, double& lon) const {
    if (_sensors == NULL || !hasLatLon(_sensors->node_lat, _sensors->node_lon)) return false;
    lat = _sensors->node_lat;
    lon = _sensors->node_lon;
    return true;
  }

  void formatDirectionalName(DisplayDriver& display, const RepeaterDiscoveryInfo& hit, char* dest, size_t dest_size, size_t max_chars) {
    char filtered[sizeof(hit.name)];
    display.translateUTF8ToBlocks(filtered, hit.name[0] ? hit.name : "Repeater", sizeof(filtered));
    size_t len = 0;
    while (filtered[len] && len < max_chars && len < dest_size - 1) {
      dest[len] = filtered[len];
      len++;
    }
    dest[len] = 0;
  }

  void formatDirectionalDistance(const DirectionalTargetRef& ref, char* dest, size_t dest_size) {
    formatDistanceMiles(ref.has_distance, ref.distance_mi, dest, dest_size);
  }

  void renderDirectionalList(DisplayDriver& display) {
    directional_dirty = true;   // permit one discovery-table rebuild for this frame
    drawPageTitle(display, "Repeaters");

    int count = getDirectionalTargetCount();
    if (count == 0) {
      bool scanning = directional_discovery_started && the_mesh.isRepeaterDiscoveryActive();
      display.setColor(scanning ? DisplayDriver::YELLOW : DisplayDriver::RED);
      display.drawTextCentered(display.width() / 2, 50, scanning ? "Scanning..." : "No repeaters");
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, 64, scanning ? "please wait" : "OK to scan");
      return;
    }

    clampDirectionalSelection();

    const int visible = 7;
    int first = directional_selected - 3;
    if (first < 0) first = 0;
    if (first > count - visible) first = count > visible ? count - visible : 0;

    int y = 34;
    for (int row = 0; row < visible && first + row < count; row++, y += 12) {
      RepeaterDiscoveryInfo hit;
      DirectionalTargetRef ref;
      if (!getDirectionalTarget(first + row, hit, &ref)) continue;

      float relative_dir = 0.0f;
      bool has_dir = getRelativeDirectionTo(hit.gps_lat / 1000000.0, hit.gps_lon / 1000000.0, relative_dir);
      char name[12];
      char dist[6];
      formatDirectionalName(display, hit, name, sizeof(name), has_dir ? 9 : 11);
      formatDirectionalDistance(ref, dist, sizeof(dist));

      bool selected = first + row == directional_selected;
      display.setColor(selected ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
      drawRowMarker(display, y, selected);
      drawSignalBars(display, 4, y - 2, rssiBars(true, ref.rssi));
      display.drawTextLeftAlign(24, y, name);
      if (has_dir) {
        drawDirectionArrow(display, 100, y + 4, relative_dir);
      }
      display.drawTextRightAlign(127, y, dist);
    }

    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 116,
                             the_mesh.isRepeaterDiscoveryActive() ? "Scanning..." : "OK scan  hold ping");

    RepeaterDiscoveryInfo selected_hit;
    if (getDirectionalTarget(directional_selected, selected_hit)) {
      renderDirectionalPingPopup(display, selected_hit);
    }
  }

  bool directionalPingStatusFor(const RepeaterDiscoveryInfo& hit, DirectionalPingStatus& status) {
    if (!the_mesh.getDirectionalPingStatus(status)) return false;
    return memcmp(status.pub_key, hit.pub_key, PUB_KEY_SIZE) == 0;
  }

  bool renderDirectionalPingPopup(DisplayDriver& display, const RepeaterDiscoveryInfo& hit) {
    DirectionalPingStatus status;
    if (!directionalPingStatusFor(hit, status) || status.pending) return false;

    const int x = 16;
    const int y = 45;
    const int w = 96;
    const int h = 34;
    char tmp[24];

    display.setTextSize(1);
    display.setColor(DisplayDriver::DARK);
    display.fillRect(x, y, w, h);
    display.setColor(status.success ? DisplayDriver::GREEN : DisplayDriver::RED);
    display.drawRect(x, y, w, h);

    if (status.success) {
      display.drawTextCentered(display.width() / 2, y + 5, "PING OK");
      snprintf(tmp, sizeof(tmp), "RSSI %ddBm", status.rssi);
      display.drawTextCentered(display.width() / 2, y + 17, tmp);
    } else {
      display.drawTextCentered(display.width() / 2, y + 8, "NO RESPONSE");
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, y + 20, "timeout");
    }
    return true;
  }

  void sendDirectionalPing() {
    RepeaterDiscoveryInfo hit;
    if (!getDirectionalTarget(directional_selected, hit)) {
      _task->showAlert("No repeater", 800);
      return;
    }

    unsigned long now = millis();
    if (now - directional_last_ping_ms < UI_DIRECTIONAL_PING_COOLDOWN_MS) {
      return;
    }

    DirectionalPingStatus status;
    if (the_mesh.getDirectionalPingStatus(status) && status.pending) {
      _task->showAlert("Ping busy", 600);
      return;
    }

    int result = the_mesh.startRepeaterPing(hit.pub_key, UI_DIRECTIONAL_PING_TIMEOUT_MS);
    directional_last_ping_ms = now;
    if (result == MSG_SEND_FAILED) {
      _task->showAlert("Ping failed", 900);
    } else {
      _task->notify(UIEventType::ack);
      _task->showAlert("Ping sent", 700);
    }
  }
#endif
#endif

  // Leaving a quick-send context (page change, etc.) must drop the stage and
  // pinned recipient, otherwise re-entering the page lands in a stale actions
  // menu aimed at a target the user never picked on this visit.
  void resetQuickSendStage() {
#if UI_QUICK_SEND == 1
    quick_stage = QUICK_TARGETS;
    quick_pinned_valid = false;
#endif
  }

  bool isBatteryCharging() const {
#ifdef PIN_BATTERY_CHARGING
    return digitalRead(PIN_BATTERY_CHARGING) == BATTERY_CHARGING_STATE_ON;
#else
    return false;
#endif
  }

  void drawChargingBolt(DisplayDriver& display, int x, int y) {
    display.fillRect(x + 5, y + 0, 2, 2);
    display.fillRect(x + 4, y + 2, 2, 2);
    display.fillRect(x + 2, y + 4, 5, 1);
    display.fillRect(x + 4, y + 5, 2, 1);
    display.fillRect(x + 3, y + 6, 2, 2);
  }

  int batteryPercent(uint16_t batteryMilliVolts) const {
#if BATT_USE_OCV_PERCENT == 1
    static const uint16_t ocvMilliVolts[] = {
      BATT_OCV_100_MILLIVOLTS,
      BATT_OCV_90_MILLIVOLTS,
      BATT_OCV_80_MILLIVOLTS,
      BATT_OCV_70_MILLIVOLTS,
      BATT_OCV_60_MILLIVOLTS,
      BATT_OCV_50_MILLIVOLTS,
      BATT_OCV_40_MILLIVOLTS,
      BATT_OCV_30_MILLIVOLTS,
      BATT_OCV_20_MILLIVOLTS,
      BATT_OCV_10_MILLIVOLTS,
      BATT_OCV_0_MILLIVOLTS
    };

    // Open-circuit-voltage (OCV) lookup: ocvMilliVolts[] holds the cell voltage at
    // 100,90,...,0 % (10 % steps, from the board's BATT_OCV_* config). Find the bracket the
    // reading falls in and linearly interpolate within it for a smooth %. LiPo discharge is
    // non-linear, so this per-point curve tracks state-of-charge far better than a flat map.
    if (batteryMilliVolts >= ocvMilliVolts[0]) return 100;
    const int bucketCount = sizeof(ocvMilliVolts) / sizeof(ocvMilliVolts[0]);
    for (int i = 1; i < bucketCount; i++) {
      if (batteryMilliVolts >= ocvMilliVolts[i]) {
        const int highPct = 100 - ((i - 1) * 10);
        const int lowPct = 100 - (i * 10);
        const int highMv = ocvMilliVolts[i - 1];
        const int lowMv = ocvMilliVolts[i];
        const int spanMv = highMv - lowMv;
        int batteryPercentage = lowPct;
        if (spanMv > 0) {
          batteryPercentage += ((batteryMilliVolts - lowMv) * (highPct - lowPct)) / spanMv;
        }
        if (batteryPercentage < 0) batteryPercentage = 0;
        if (batteryPercentage > 100) batteryPercentage = 100;
        if (batteryPercentage >= 97) batteryPercentage = 100;
        return batteryPercentage;
      }
    }
    return 0;
#else
    const int minMilliVolts = BATT_MIN_MILLIVOLTS;
    const int maxMilliVolts = BATT_MAX_MILLIVOLTS;
    int batteryPercentage = ((batteryMilliVolts - minMilliVolts) * 100) / (maxMilliVolts - minMilliVolts);
    if (batteryPercentage < 0) batteryPercentage = 0; // Clamp to 0%
    if (batteryPercentage > 100) batteryPercentage = 100; // Clamp to 100%
    if (batteryPercentage >= 97) batteryPercentage = 100;
    return batteryPercentage;
#endif
  }

  int displayed_batt_pct = -1;   // hysteresis: hold the shown %, ignore <2% jitter near OCV boundaries
  bool renderBatteryIndicator(DisplayDriver& display, uint16_t batteryMilliVolts) {
    int raw_pct = batteryPercent(batteryMilliVolts);
    if (displayed_batt_pct < 0 || abs(raw_pct - displayed_batt_pct) >= 2) displayed_batt_pct = raw_pct;
    int batteryPercentage = displayed_batt_pct;
    bool charging = isBatteryCharging();
    static unsigned long last_battery_blink = 0;
    static bool charging_bolt_visible = true;
    if (charging && millis() - last_battery_blink >= 500) {
      charging_bolt_visible = !charging_bolt_visible;
      last_battery_blink = millis();
    }

#if UI_MESHTASTIC_BATTERY_ICON == 1
    int iconX = display.width() - 8;
    int iconY = 1;
    display.setColor(DisplayDriver::GREEN);

    drawXbmLsb(display, iconX, iconY, meshtastic_battery_v, 7, 11);
    if (charging && charging_bolt_visible) {
      drawXbmLsb(display, iconX + 1, iconY + 3, meshtastic_lightning_bolt_v, 5, 5);
    } else {
      drawXbmLsb(display, iconX - 1, iconY + 4, meshtastic_battery_sidegaps_v, 8, 3);
      int fillHeight = (8 * batteryPercentage) / 100;
      int fillY = iconY - fillHeight;
      display.fillRect(iconX + 1, fillY + 10, 5, fillHeight);
    }
#else
    // battery icon
    int iconWidth = 24;
    int iconHeight = 10;
    int iconX = display.width() - iconWidth - 5; // Position the icon near the top-right corner
    int iconY = 0;
    display.setColor(DisplayDriver::GREEN);

    // battery outline
    display.drawRect(iconX, iconY, iconWidth, iconHeight);

    // battery "cap"
    display.fillRect(iconX + iconWidth, iconY + (iconHeight / 4), 3, iconHeight / 2);

    // fill the battery based on the percentage
    int fillWidth = (batteryPercentage * (iconWidth - 4)) / 100;
    if (charging && charging_bolt_visible) {
      drawChargingBolt(display, iconX + 8, iconY + 1);
    } else {
      display.fillRect(iconX + 2, iconY + 2, fillWidth, iconHeight - 4);
    }
#endif

    char percent[5];
    snprintf(percent, sizeof(percent), "%d%%", batteryPercentage);
    int percentWidth = display.getTextWidth(percent);
    int percentX = iconX - percentWidth - 2;
    display.setColor(DisplayDriver::LIGHT);
    display.setCursor(percentX, UI_CLOCK_Y);
    display.print(percent);

    // show muted icon if buzzer is muted
#if defined(PIN_BUZZER) || defined(PIN_ACTIVE_BUZZER)
    if (_task->isBuzzerQuiet()) {
      display.setColor(DisplayDriver::RED);
      display.drawXbm(percentX - 9, iconY + 1, muted_icon, 8, 8);
    }
#endif
    return charging;
  }

  int batteryIndicatorLeft(DisplayDriver& display) const {
#if UI_MESHTASTIC_BATTERY_ICON == 1
    int iconX = display.width() - 8;
#else
    int iconWidth = 24;
    int iconX = display.width() - iconWidth - 5;
#endif
    int left = iconX - display.getTextWidth("100%") - 2;
#if defined(PIN_BUZZER) || defined(PIN_ACTIVE_BUZZER)
    if (_task->isBuzzerQuiet()) left -= 9;
#endif
    return left;
  }

  int renderClockIndicator(DisplayDriver& display, int right_edge) {
#if UI_HOME_CLOCK == 1
    if (_rtc == NULL) return right_edge;

    uint32_t now = _rtc->getCurrentTime();
    int64_t adjusted = (int64_t)now + ((int64_t)clockUtcOffsetMinutes(now) * 60);
    int32_t day_seconds = (int32_t)(adjusted % 86400);
    if (day_seconds < 0) day_seconds += 86400;

    char clock[6];
    snprintf(clock, sizeof(clock), "%02d:%02d", day_seconds / 3600, (day_seconds / 60) % 60);
    int width = display.getTextWidth(clock);
    int x = right_edge - width - 2;
    if (x < 0) return right_edge;

    display.setColor(DisplayDriver::LIGHT);
    display.setCursor(x, UI_CLOCK_Y);
    display.print(clock);
    return x;
#else
    return right_edge;
#endif
  }

  const char* motionStatusLabel(uint8_t state) const {
    switch (state) {
      case SensorManager::MOTION_STATIONARY: return "still";
      case SensorManager::MOTION_PENDING: return "arming";
      case SensorManager::MOTION_MOVING: return "moving";
      case SensorManager::MOTION_UNKNOWN:
      default: return "--";
    }
  }

  const char* smartGpsStatusLabel(uint8_t state, bool gps_on) const {
    switch (state) {
      case SensorManager::SMART_GPS_NOT_PERMITTED: return "locked";
      case SensorManager::SMART_GPS_STATIONARY: return "idle";
      case SensorManager::SMART_GPS_MOTION_PENDING: return "arming";
      case SensorManager::SMART_GPS_ACQUIRING: return "search";
      case SensorManager::SMART_GPS_ACTIVE: return "active";
      case SensorManager::SMART_GPS_STATIONARY_HOLD: return "hold";
      case SensorManager::SMART_GPS_UNAVAILABLE:
      default: return gps_on ? "on" : "off";
    }
  }

  CayenneLPP sensors_lpp;
  int sensors_nb = 0;
  bool sensors_scroll = false;
  int sensors_scroll_offset = 0;
  int next_sensors_refresh = 0;

#if UI_HAS_COMPASS == 1
  SensorManager::CompassReading compass_filtered = {};
  bool compass_filter_valid = false;
  bool compass_mode_is_compass = true;
  bool compass_mode_valid = false;
  unsigned long compass_filter_updated = 0;
  // Last-drawn integer values: when the rendered numbers haven't changed we
  // drop to the idle cadence — each full-frame I2C flush blocks the loop for
  // ~50ms, so 10Hz is only paid while the compass is actually turning.
  int compass_last_primary = -9999;   // heading (compass mode) or tilt
  int compass_last_pitch = -9999;
  int compass_last_roll = -9999;
  bool compass_last_mode = true;

  static float clampFloat(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
  }

  static float deadbandFloat(float v, float deadband) {
    return fabsf(v) < deadband ? 0.0f : v;
  }

  // Exponential moving average for an angle, taking the shortest signed path across the
  // 360-degree wrap (so 350 -> 10 smooths forward through 0, not backward through 180).
  // alpha in (0,1]: higher = snappier (heading ~0.18, tilt ~0.24, empirically tuned).
  float smoothDeg(float current, float target, float alpha) const {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return normalizeDeg(current + (diff * alpha));
  }

  // EMA of a 3D basis vector, re-normalized to unit length each step so the smoothed
  // orientation basis stays on the unit sphere; falls back to the raw target if the
  // smoothed vector collapses (norm ~ 0).
  void smoothUnitVector(float& x, float& y, float& z, float tx, float ty, float tz, float alpha) {
    x += (tx - x) * alpha;
    y += (ty - y) * alpha;
    z += (tz - z) * alpha;
    float norm = sqrtf(x * x + y * y + z * z);
    if (norm > 0.001f) {   // guard divide-by-zero on a degenerate (collapsed) vector
      x /= norm;
      y /= norm;
      z /= norm;
    } else {
      x = tx;
      y = ty;
      z = tz;
    }
  }

  SensorManager::CompassReading smoothCompassReading(const SensorManager::CompassReading& raw) {
    unsigned long now = millis();
    if (!compass_filter_valid || now - compass_filter_updated > UI_COMPASS_FILTER_RESET_MS) {
      compass_filtered = raw;
      compass_filter_valid = true;
      compass_filter_updated = now;
      return compass_filtered;
    }

    compass_filtered.heading_deg = smoothDeg(compass_filtered.heading_deg, raw.heading_deg, UI_COMPASS_HEADING_ALPHA);
    if (raw.antenna_heading_valid) {
      compass_filtered.antenna_heading_deg = smoothDeg(compass_filtered.antenna_heading_deg,
                                                       raw.antenna_heading_deg,
                                                       UI_COMPASS_HEADING_ALPHA);
    } else {
      compass_filtered.antenna_heading_deg = raw.antenna_heading_deg;
    }
    compass_filtered.antenna_heading_valid = raw.antenna_heading_valid;
    compass_filtered.antenna_elevation_deg += (raw.antenna_elevation_deg - compass_filtered.antenna_elevation_deg) *
                                              UI_COMPASS_TILT_ALPHA;
    compass_filtered.pitch_deg += (raw.pitch_deg - compass_filtered.pitch_deg) * UI_COMPASS_TILT_ALPHA;
    compass_filtered.roll_deg += (raw.roll_deg - compass_filtered.roll_deg) * UI_COMPASS_TILT_ALPHA;
    if (raw.orientation_valid) {
      smoothUnitVector(compass_filtered.east_x, compass_filtered.east_y, compass_filtered.east_z,
                       raw.east_x, raw.east_y, raw.east_z, UI_COMPASS_HEADING_ALPHA);
      smoothUnitVector(compass_filtered.north_x, compass_filtered.north_y, compass_filtered.north_z,
                       raw.north_x, raw.north_y, raw.north_z, UI_COMPASS_HEADING_ALPHA);
      smoothUnitVector(compass_filtered.up_x, compass_filtered.up_y, compass_filtered.up_z,
                       raw.up_x, raw.up_y, raw.up_z, UI_COMPASS_HEADING_ALPHA);
    }
    compass_filtered.orientation_valid = raw.orientation_valid;
    compass_filtered.flat = raw.flat;
    compass_filtered.calibrated = raw.calibrated;
    compass_filtered.calibrating = raw.calibrating;
    compass_filtered.calibration_seconds_remaining = raw.calibration_seconds_remaining;
    compass_filtered.calibration_state = raw.calibration_state;
    compass_filtered.calibration_quality = raw.calibration_quality;
    compass_filtered.calibration_progress = raw.calibration_progress;
    compass_filtered.calibration_samples = raw.calibration_samples;
    compass_filtered.heading_validity = raw.heading_validity;
    compass_filtered.motion_state = raw.motion_state;
    compass_filtered.motion_score = raw.motion_score;
    compass_filtered.smart_gps_state = raw.smart_gps_state;
    compass_filtered.age_ms = raw.age_ms;
    compass_filter_updated = now;
    return compass_filtered;
  }

  bool shouldShowCompassMode(const SensorManager::CompassReading& compass) {
    return fabsf(compass.pitch_deg) <= UI_COMPASS_MODE_MAX_TILT_DEG &&
           fabsf(compass.roll_deg) <= UI_COMPASS_MODE_MAX_TILT_DEG;
  }

  bool resolveCompassMode(const SensorManager::CompassReading& compass) {
    if (!compass_mode_valid) {
      compass_mode_is_compass = shouldShowCompassMode(compass);
      compass_mode_valid = true;
      return compass_mode_is_compass;
    }

    float abs_pitch = fabsf(compass.pitch_deg);
    float abs_roll = fabsf(compass.roll_deg);
    if (compass_mode_is_compass) {
      if (abs_pitch > UI_COMPASS_MODE_MAX_TILT_DEG + UI_COMPASS_MODE_HYSTERESIS_DEG ||
          abs_roll > UI_COMPASS_MODE_MAX_TILT_DEG + UI_COMPASS_MODE_HYSTERESIS_DEG) {
        compass_mode_is_compass = false;
      }
    } else {
      if (abs_pitch <= UI_COMPASS_MODE_MAX_TILT_DEG - UI_COMPASS_MODE_HYSTERESIS_DEG &&
          abs_roll <= UI_COMPASS_MODE_MAX_TILT_DEG - UI_COMPASS_MODE_HYSTERESIS_DEG) {
        compass_mode_is_compass = true;
      }
    }
    return compass_mode_is_compass;
  }

  float antennaPitchErrorDeg(float pitch_deg) {
    float target = pitch_deg >= 0.0f ? 90.0f : -90.0f;
    return pitch_deg - target;
  }

  // Combined tilt magnitude for antenna alignment: root-sum-square of pitch & roll errors
  // (deg). 0 = antenna vertical/aligned; grows as the device tilts off-axis.
  float antennaAlignmentTiltDeg(const SensorManager::CompassReading& compass) {
    float pitch_error = deadbandFloat(antennaPitchErrorDeg(compass.pitch_deg), UI_COMPASS_DISPLAY_DEADBAND_DEG);
    float roll_error = deadbandFloat(compass.roll_deg, UI_COMPASS_DISPLAY_DEADBAND_DEG);
    return sqrtf((pitch_error * pitch_error) + (roll_error * roll_error));
  }

  void drawAntennaTiltVector(DisplayDriver& display, int x, int top_y, int base_y, float tilt_deg, const char* label) {
    const int h = base_y - top_y;
    const int max_dx = 20;
    float tilt = clampFloat(tilt_deg, -UI_COMPASS_LEVEL_MAX_TILT_DEG, UI_COMPASS_LEVEL_MAX_TILT_DEG);
    int tip_x = x + (int)((tilt / UI_COMPASS_LEVEL_MAX_TILT_DEG) * max_dx);
    int tip_y = base_y - h;

    display.drawTextCentered(x, top_y - 11, label);
    display.drawLine(x, base_y, x, tip_y);
    display.drawLine(x - 5, base_y, x + 5, base_y);
    display.drawLine(x, base_y, tip_x, tip_y);
    display.fillRect(tip_x - 2, tip_y - 2, 5, 5);
    if (abs(tip_x - x) <= 2) {
      display.drawRect(x - 4, tip_y - 4, 9, 9);
    }
  }

  void drawLevelBubble(DisplayDriver& display, int cx, int cy, int r, float x_error_deg, float y_error_deg, bool draw_crosshair) {
    x_error_deg = deadbandFloat(x_error_deg, UI_COMPASS_DISPLAY_DEADBAND_DEG);
    y_error_deg = deadbandFloat(y_error_deg, UI_COMPASS_DISPLAY_DEADBAND_DEG);
    int max_offset = r - 5;
    int bx = cx + (int)((clampFloat(x_error_deg, -UI_COMPASS_LEVEL_MAX_TILT_DEG, UI_COMPASS_LEVEL_MAX_TILT_DEG) /
                         UI_COMPASS_LEVEL_MAX_TILT_DEG) * max_offset);
    int by = cy + (int)((clampFloat(y_error_deg, -UI_COMPASS_LEVEL_MAX_TILT_DEG, UI_COMPASS_LEVEL_MAX_TILT_DEG) /
                         UI_COMPASS_LEVEL_MAX_TILT_DEG) * max_offset);

    display.drawCircle(cx, cy, r);
    if (draw_crosshair) {
      display.drawLine(cx - 5, cy, cx + 5, cy);
      display.drawLine(cx, cy - 5, cx, cy + 5);
    }

    display.fillRect(bx - 2, by - 2, 5, 5);
    if (abs(bx - cx) <= 2 && abs(by - cy) <= 2) {
      display.drawRect(cx - 5, cy - 5, 11, 11);
    }
  }

  const char* compassCalLabel(const SensorManager::CompassReading& compass) const {
    if (compass.calibrating) return "CAL...";
    switch (compass.calibration_state) {
      case SensorManager::COMPASS_CAL_LOADED: return "CAL OK";
      case SensorManager::COMPASS_CAL_BAD: return "BAD CAL";
      case SensorManager::COMPASS_CAL_MISSING: return "NO CAL";
      case SensorManager::COMPASS_CAL_CALIBRATING: return "CAL...";
      case SensorManager::COMPASS_CAL_UNKNOWN:
      default: return compass.calibrated ? "CAL OK" : "NO CAL";
    }
  }

  void drawTinyProgress(DisplayDriver& display, int x, int y, int w, int h, uint8_t pct) {
    if (pct > 100) pct = 100;
    display.drawRect(x, y, w, h);
    int fill = ((w - 2) * pct) / 100;
    if (fill > 0) display.fillRect(x + 1, y + 1, fill, h - 2);
  }

  // Rotating compass ring with N/E/S/W ticks. Each cardinal sits at screen angle
  // (bearing - heading), so the ring counter-rotates under a fixed "up = ahead" reference.
  // Polar -> screen: x = cx + sin(a)*radius, y = cy - cos(a)*radius (screen y grows downward).
  void drawCompassWidget(DisplayDriver& display, int cx, int cy, int r, const SensorManager::CompassReading& compass) {
    float heading_deg = normalizeDeg(compass.heading_deg + UI_HEADING_RENDER_OFFSET_DEG);
    display.drawCircle(cx, cy, r);

    const char* labels[] = { "N", "E", "S", "W" };
    const int bearings[] = { 0, 90, 180, 270 };
    for (int i = 0; i < 4; i++) {
      int bearing = bearings[i];
      float rad = (bearing - heading_deg) * DEG2RAD;
      int outer_x = cx + (int)(sinf(rad) * (r - 1));
      int outer_y = cy - (int)(cosf(rad) * (r - 1));
      int inner_x = cx + (int)(sinf(rad) * (r - 11));
      int inner_y = cy - (int)(cosf(rad) * (r - 11));
      display.drawLine(inner_x, inner_y, outer_x, outer_y);

      int lx = cx + (int)(sinf(rad) * (r - 8));
      int ly = cy - (int)(cosf(rad) * (r - 8)) - 3;
      display.drawTextCentered(lx, ly, labels[i]);
    }

    drawLevelBubble(display, cx, cy, r - 17, compass.roll_deg, -compass.pitch_deg, false);
  }

  void drawAntennaAlignment(DisplayDriver& display, const SensorManager::CompassReading& compass) {
    float pitch_error = deadbandFloat(antennaPitchErrorDeg(compass.pitch_deg), UI_COMPASS_DISPLAY_DEADBAND_DEG);
    float roll_error = deadbandFloat(compass.roll_deg, UI_COMPASS_DISPLAY_DEADBAND_DEG);

    display.drawLine(17, 88, 111, 88);
    display.drawLine(64, 32, 64, 88);
    display.drawLine(60, 37, 64, 32);
    display.drawLine(68, 37, 64, 32);

    drawLevelBubble(display, 64, 63, 20, roll_error, pitch_error, true);
    drawAntennaTiltVector(display, 22, 47, 88, pitch_error, "P");
    drawAntennaTiltVector(display, 106, 47, 88, roll_error, "R");
  }
#endif

  void refresh_sensors() {
    if (millis() > next_sensors_refresh) {
      sensors_lpp.reset();
      sensors_nb = 0;
      sensors_lpp.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
      if (_sensors != NULL) {
        _sensors->querySensors(0xFF, sensors_lpp);
      }
      LPPReader reader (sensors_lpp.getBuffer(), sensors_lpp.getSize());
      uint8_t channel, type;
      while(reader.readHeader(channel, type)) {
        reader.skipData(type);
        sensors_nb ++;
      }
      sensors_scroll = sensors_nb > UI_RECENT_LIST_SIZE;
#if AUTO_OFF_MILLIS > 0
      next_sensors_refresh = millis() + 5000; // refresh sensor values every 5 sec
#else
      next_sensors_refresh = millis() + 60000; // refresh sensor values every 1 min
#endif
    }
  }

public:
  HomeScreen(UITask* task, mesh::RTCClock* rtc, SensorManager* sensors, NodePrefs* node_prefs)
     : _task(task), _rtc(rtc), _sensors(sensors), _node_prefs(node_prefs), _page(0),
       _shutdown_init(false), sensors_lpp(200) {  }

  void poll() override {
    if (_shutdown_init && !_task->isButtonPressed()) {  // must wait for USR button to be released
      _task->shutdown(false, "Hibernating");
    }
  }

  // True while a typing surface owns the keys (incoming messages must not
  // switch screens out from under the user mid-compose).
  bool isModalInputActive() const {
#if UI_QUICK_SEND == 1
    return quick_stage == QUICK_KEYBOARD;
#else
    return false;
#endif
  }

  // Smart GPS state -> short label (+countdown) for the GPS status box.
  void smartGpsLabel(uint8_t st, const SensorManager::GPSStatus& gs, bool live, char* dest, size_t size) {
    switch (st) {
      case SensorManager::SMART_GPS_START_PENDING:   snprintf(dest, size, "Start %us", (unsigned)gs.start_pending_seconds); break;
      case SensorManager::SMART_GPS_MOTION_PENDING:  snprintf(dest, size, "Arming %us", (unsigned)gs.motion_arming_seconds); break;
      case SensorManager::SMART_GPS_ACQUIRING:       snprintf(dest, size, "Searching"); break;
      case SensorManager::SMART_GPS_ACTIVE:          snprintf(dest, size, "%s", live ? "Active fix" : "Active"); break;
      case SensorManager::SMART_GPS_STATIONARY:      snprintf(dest, size, "Sleeping"); break;
      case SensorManager::SMART_GPS_STATIONARY_HOLD: snprintf(dest, size, "Hold %us", (unsigned)gs.idle_shutdown_seconds); break;
      case SensorManager::SMART_GPS_NOT_PERMITTED:   snprintf(dest, size, "GPS off"); break;
      default:                                       snprintf(dest, size, "%s", gs.gps_available ? "GPS off" : "No GPS"); break;
    }
  }

  void formatAgeSecs(uint32_t sec, char* dest, size_t size) {
    if (sec < 60)         snprintf(dest, size, "%us", (unsigned)sec);
    else if (sec < 3600)  snprintf(dest, size, "%um", (unsigned)(sec / 60));
    else if (sec < 86400) snprintf(dest, size, "%uh", (unsigned)(sec / 3600));
    else                  snprintf(dest, size, "%ud", (unsigned)(sec / 86400));
  }

  // GPS page: Smart GPS lifecycle (state + countdown) and live-vs-last-known location.
  // Reads a single getGPSStatus() snapshot (cheap, no I2C/IMU); provider used only for sats/time.
  int renderGpsPage(DisplayDriver& display) {
    char buf[40];
    char line[28];
    drawPageTitle(display, "GPS Status");

    SensorManager::GPSStatus gs = {};  // zero-init: read below even when getGPSStatus() fails
    bool ok = (_sensors != NULL) && _sensors->getGPSStatus(gs);
    LocationProvider* nmea = (_sensors != NULL) ? _sensors->getLocationProvider() : NULL;
    int sat = (nmea != NULL) ? nmea->satellitesCount() : 0;
    if (sat < 0) sat = 0;
    bool live = ok && gs.live_fix_valid;
    uint8_t st = ok ? gs.smart_gps_state : (uint8_t)SensorManager::SMART_GPS_UNAVAILABLE;

    // Status box: Smart GPS state (+countdown) left, sat count right.
    display.setColor(DisplayDriver::LIGHT);
    display.drawRect(2, 30, 124, 22);
    smartGpsLabel(st, gs, live, line, sizeof(line));
    auto scol = DisplayDriver::RED;
    switch (st) {
      case SensorManager::SMART_GPS_ACTIVE:          scol = live ? DisplayDriver::GREEN : DisplayDriver::YELLOW; break;
      case SensorManager::SMART_GPS_ACQUIRING:
      case SensorManager::SMART_GPS_MOTION_PENDING:
      case SensorManager::SMART_GPS_START_PENDING:
      case SensorManager::SMART_GPS_STATIONARY_HOLD: scol = DisplayDriver::YELLOW; break;
      case SensorManager::SMART_GPS_STATIONARY:      scol = DisplayDriver::LIGHT;  break;
      default:                                       scol = DisplayDriver::RED;    break;
    }
    display.setColor(scol);
    display.drawTextLeftAlign(7, 36, line);
    display.setColor(DisplayDriver::LIGHT);
    snprintf(buf, sizeof(buf), "%d sat", sat);
    display.drawTextRightAlign(display.width() - 7, 36, buf);

    // Location: live fix, else last-known/persisted, else none.
    int y = 58;
    if (live) {
      snprintf(buf, sizeof(buf), "%.5f", gs.live_lat);       y = drawLabelValueRow(display, y, "lat", buf);
      snprintf(buf, sizeof(buf), "%.5f", gs.live_lon);       y = drawLabelValueRow(display, y, "lon", buf);
      snprintf(buf, sizeof(buf), "%.0fm", gs.live_altitude); y = drawLabelValueRow(display, y, "alt", buf);
    } else if (ok && gs.persisted_location_valid) {
      display.setColor(DisplayDriver::YELLOW);
      if (gs.persisted_location_age_known) {
        formatAgeSecs(gs.persisted_location_age_sec, buf, sizeof(buf));
        snprintf(line, sizeof(line), "Last fix %s", buf);
      } else {
        snprintf(line, sizeof(line), "Last fix (saved)");
      }
      display.drawTextLeftAlign(2, y, line); y += ROW_H;
      snprintf(buf, sizeof(buf), "%.5f", gs.persisted_lat); y = drawLabelValueRow(display, y, "lat", buf);
      snprintf(buf, sizeof(buf), "%.5f", gs.persisted_lon); y = drawLabelValueRow(display, y, "lon", buf);
    } else {
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, y + 6, "No location yet");
      y += 18;
    }

    // Time since the last GPS fix (how stale the shared position is).
    if (ok && gs.live_fix_valid) formatAgeSecs(gs.live_fix_age_sec, buf, sizeof(buf));
    else if (ok && gs.persisted_location_valid && gs.persisted_location_age_known) formatAgeSecs(gs.persisted_location_age_sec, buf, sizeof(buf));
    else strcpy(buf, "--");
    y = drawLabelValueRow(display, y, "last fix", buf);

    // Time of day + source confidence (gps-synced / rtc / unknown).
    const char* tlabel = (gs.time_source_state == SensorManager::TIME_SOURCE_GPS) ? "gps time"
                       : (gs.time_source_state == SensorManager::TIME_SOURCE_RTC_SET) ? "rtc time"
                       : "time?";
    if (nmea != NULL && nmea->isValid()) {
      uint32_t t = nmea->getTimestamp();
      int64_t adj = (int64_t)t + ((int64_t)clockUtcOffsetMinutes(t) * 60);
      int32_t ds = (int32_t)(adj % 86400); if (ds < 0) ds += 86400;
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d", ds / 3600, (ds / 60) % 60, ds % 60);
    } else {
      strcpy(buf, "--");
    }
    y = drawLabelValueRow(display, y, tlabel, buf);

    // Make it unmistakable when telemetry is sharing a stale last-known fix.
    if (ok && gs.telemetry_using_persisted_location && !live) {
      display.setColor(DisplayDriver::YELLOW);
      display.drawTextCentered(display.width() / 2, 120, "sharing last fix");
    }
    return 1000;
  }

  bool _diag_open = false;   // hidden diagnostics page (long-press on home opens it)

  // Power-mode selector: Normal / Expedition / Stationary (ENTER cycles + applies).
  int renderProfilePage(DisplayDriver& display) {
    drawPageTitle(display, "Power Mode");
    uint8_t cur = _node_prefs ? _node_prefs->power_profile : 0;
    if (cur >= SensorManager::POWER_PROFILE_COUNT) cur = 0;
    int y = 42;
    for (uint8_t p = 0; p < SensorManager::POWER_PROFILE_COUNT; p++, y += 16) {
      bool sel = (p == cur);
      display.setColor(sel ? DisplayDriver::GREEN : DisplayDriver::LIGHT);
      char row[28];
      snprintf(row, sizeof(row), "%s%s", sel ? "> " : "  ", SensorManager::powerProfileName(p));
      display.drawTextLeftAlign(20, y, row);
    }
    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 116, "Press to change");
    return 1000;
  }

  // Hidden field-diagnostics page (battery/charge/fault, power mode, RTC/time source, I2C scan).
  int renderDiagnostics(DisplayDriver& display) {
    char ln[40];
    drawPageTitle(display, "Diagnostics");
    SensorManager::DiagnosticsStatus d;
    if (_sensors == NULL || !_sensors->getDiagnostics(d)) {
      display.setColor(DisplayDriver::RED);
      display.drawTextCentered(display.width() / 2, 60, "unavailable");
      display.setColor(DisplayDriver::LIGHT);
      display.drawTextCentered(display.width() / 2, 116, "Back to exit");
      return 1000;
    }
    int y = 30;
    if (d.battery_mv_known) snprintf(ln, sizeof(ln), "%umV", (unsigned)d.battery_mv);
    else                    strcpy(ln, "--");
    y = drawLabelValueRow(display, y, "batt", ln);
    snprintf(ln, sizeof(ln), "%s %s%s",
             d.external_power_known ? (d.external_powered ? "ext" : "batt") : "?",
             d.charge_state_known ? (d.charging ? "chg" : "idle") : "",
             (d.charger_fault_known && d.charger_fault) ? " FLT" : "");
    y = drawLabelValueRow(display, y, "pwr", ln);
    y = drawLabelValueRow(display, y, "mode", SensorManager::powerProfileName(d.power_profile));
    y = drawLabelValueRow(display, y, "rtc", d.rtc_source_name ? d.rtc_source_name : "?");
    const char* ts = (d.time_source_state == SensorManager::TIME_SOURCE_GPS) ? "gps"
                   : (d.time_source_state == SensorManager::TIME_SOURCE_RTC_SET) ? "rtc" : "stale";
    y = drawLabelValueRow(display, y, "time src", ts);
    snprintf(ln, sizeof(ln), "%d / %d", (int)d.i2c_wire_count, (int)d.i2c_wire1_count);
    y = drawLabelValueRow(display, y, "i2c dev", ln);
#ifdef UI_HAS_GPS_SWITCH
    {
      // Raw slide-switch pins plus decoded state for field characterization.
      const char* sw = "?";
      if (d.gps_switch_known) {
        switch (d.gps_switch_raw_state) {
          case GPS_SWITCH_ON:     sw = "ON";     break;
          case GPS_SWITCH_ON_GPS: sw = "ON-GPS"; break;
          case GPS_SWITCH_OFF:    sw = "OFF";    break;
          default:                sw = "?";      break;
        }
        snprintf(ln, sizeof(ln), "%u%u %s",
                 d.gps_switch_mode1 ? 1 : 0,
                 d.gps_switch_mode2 ? 1 : 0,
                 sw);
      } else {
        strcpy(ln, "--");
      }
      y = drawLabelValueRow(display, y, "switch", ln);
    }
#endif
    display.setColor(DisplayDriver::LIGHT);
    display.drawTextCentered(display.width() / 2, 120, "Back to exit");
    return 1000;
  }

  int render(DisplayDriver& display) override {
    char tmp[80];
    if (_diag_open) return renderDiagnostics(display);
    display.setTextSize(1);
    int battery_left = batteryIndicatorLeft(display);
    int clock_left = renderClockIndicator(display, battery_left);

    // node name
    display.setColor(DisplayDriver::GREEN);
    char filtered_name[sizeof(_node_prefs->node_name)];
    display.translateUTF8ToBlocks(filtered_name, _node_prefs->node_name, sizeof(filtered_name));
    display.drawTextEllipsized(0, 0, clock_left - 2, filtered_name);

    // battery voltage
    bool battery_charging = renderBatteryIndicator(display, _task->getBattMilliVolts());

    // curr page indicator
    int y = 14;
    int x = display.width() / 2 - 5 * (HomePage::Count-1);
    for (uint8_t i = 0; i < HomePage::Count; i++, x += 10) {
      if (i == _page) {
        display.fillRect(x-1, y-1, 3, 3);
      } else {
        display.fillRect(x, y, 1, 1);
      }
    }

    if (_page == HomePage::FIRST) {
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(2);
      snprintf(tmp, sizeof(tmp), "MSG: %d", _task->getMsgCount());
      display.drawTextCentered(display.width() / 2, 20 + UI_HOME_ACTION_Y_OFFSET, tmp);

      #ifdef WIFI_SSID
        IPAddress ip = WiFi.localIP();
        snprintf(tmp, sizeof(tmp), "IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 54 + UI_HOME_ACTION_Y_OFFSET, tmp);
      #endif
      if (_task->hasConnection()) {
        display.setColor(DisplayDriver::GREEN);
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 43 + UI_HOME_ACTION_Y_OFFSET, "< Connected >");

      } else if (the_mesh.getBLEPin() != 0) { // BT pin
        display.setColor(DisplayDriver::RED);
        display.setTextSize(2);
        snprintf(tmp, sizeof(tmp), "Pin:%d", the_mesh.getBLEPin());
        display.drawTextCentered(display.width() / 2, 43 + UI_HOME_ACTION_Y_OFFSET, tmp);
      }
    } else if (_page == HomePage::RECENT) {
      the_mesh.getRecentlyHeard(recent, UI_RECENT_LIST_SIZE);
      drawPageTitle(display, "Recent ADV");
      display.setColor(DisplayDriver::GREEN);
      int y = 34;
      for (int i = 0; i < UI_RECENT_LIST_SIZE; i++, y += 11) {
        auto a = &recent[i];
        if (a->name[0] == 0) continue;  // empty slot
        int secs = _rtc->getCurrentTime() - a->recv_timestamp;
        if (secs < 60) {
          snprintf(tmp, sizeof(tmp), "%ds", secs);
        } else if (secs < 60*60) {
          snprintf(tmp, sizeof(tmp), "%dm", secs / 60);
        } else {
          snprintf(tmp, sizeof(tmp), "%dh", secs / (60*60));
        }

        int timestamp_width = display.getTextWidth(tmp);
        int max_name_width = display.width() - timestamp_width - 1;

        char filtered_recent_name[sizeof(a->name)];
        display.translateUTF8ToBlocks(filtered_recent_name, a->name, sizeof(filtered_recent_name));
        display.drawTextEllipsized(0, y, max_name_width, filtered_recent_name);
        display.setCursor(display.width() - timestamp_width - 1, y);
        display.print(tmp);
      }
#if UI_QUICK_SEND == 1
    } else if (_page == HomePage::CHANNELS || _page == HomePage::GROUP) {
      if (quick_stage == QUICK_KEYBOARD) {
        renderQuickKeyboard(display);
      } else if (quick_stage == QUICK_ACTIONS) {
        renderQuickActionList(display);
      } else if (_page == HomePage::CHANNELS) {
        renderChannelList(display);
      } else {
        renderGroupList(display);
      }
      return 500;
#endif
    } else if (_page == HomePage::RADIO) {
      drawPageTitle(display, "LoRa Radio");
      display.setColor(DisplayDriver::GREEN);
      snprintf(tmp, sizeof(tmp), "%.3f MHz", _node_prefs->freq);
      display.drawTextCentered(display.width() / 2, 36, tmp);

      display.setColor(DisplayDriver::LIGHT);
      snprintf(tmp, sizeof(tmp), "SF%-2d  BW %.1f", _node_prefs->sf, _node_prefs->bw);
      display.drawTextCentered(display.width() / 2, 52, tmp);
      snprintf(tmp, sizeof(tmp), "CR 4/%d  TX %ddBm", _node_prefs->cr, _node_prefs->tx_power_dbm);
      display.drawTextCentered(display.width() / 2, 64, tmp);

      int noise = radio_driver.getNoiseFloor();
      snprintf(tmp, sizeof(tmp), "noise %ddBm", noise);
      display.drawTextCentered(display.width() / 2, 84, tmp);
      int bar = constrain(noise + 130, 0, 60);
      display.drawRect(33, 100, 62, 7);
      display.fillRect(34, 101, bar, 5);
    } else if (_page == HomePage::BLUETOOTH) {
      display.setColor(DisplayDriver::GREEN);
      display.drawXbm((display.width() - 32) / 2, 18 + UI_HOME_ACTION_Y_OFFSET,
          _task->isSerialEnabled() ? bluetooth_on : bluetooth_off,
          32, 32);
      display.setTextSize(1);
      display.drawTextCentered(display.width() / 2, 64 - 11 + UI_HOME_ACTION_Y_OFFSET, "toggle: " PRESS_LABEL);
    } else if (_page == HomePage::ADVERT) {
      display.setColor(DisplayDriver::GREEN);
      display.drawXbm((display.width() - 32) / 2, 18 + UI_HOME_ACTION_Y_OFFSET, advert_icon, 32, 32);
      display.drawTextCentered(display.width() / 2, 64 - 11 + UI_HOME_ACTION_Y_OFFSET, "advert: " PRESS_LABEL);
#if ENV_INCLUDE_GPS == 1
    } else if (_page == HomePage::GPS) {
      return renderGpsPage(display);
#endif
#if UI_HAS_COMPASS == 1
    } else if (_page == HomePage::COMPASS) {
      SensorManager::CompassReading raw_compass;
      SensorManager::CompassReading compass;
      bool compass_mode;
      display.setColor(DisplayDriver::YELLOW);
      display.setTextSize(1);

      if (_sensors == NULL || !_sensors->getCompass(raw_compass)) {
        display.setColor(DisplayDriver::RED);
        display.drawTextCentered(display.width() / 2, display.height() / 2 - 4, "Compass unavailable");
        compass_filter_valid = false;
        compass_mode_valid = false;
        return 1000;
      }
      // Calibration result screen (firmware holds it ~2.5s after finish/cancel).
      if (raw_compass.calibration_result != SensorManager::COMPASS_CAL_RESULT_NONE) {
        const char* word = "--";
        auto rcol = DisplayDriver::LIGHT;
        switch (raw_compass.calibration_result) {
          case SensorManager::COMPASS_CAL_RESULT_GOOD:      word = "GOOD";      rcol = DisplayDriver::GREEN;  break;
          case SensorManager::COMPASS_CAL_RESULT_FAIR:      word = "FAIR";      rcol = DisplayDriver::YELLOW; break;
          case SensorManager::COMPASS_CAL_RESULT_POOR:      word = "POOR";      rcol = DisplayDriver::RED;    break;
          case SensorManager::COMPASS_CAL_RESULT_CANCELLED: word = "Cancelled"; rcol = DisplayDriver::LIGHT;  break;
        }
        display.drawTextCentered(display.width() / 2, 28, "Calibration");
        display.setColor(rcol);
        display.setTextSize(2);
        display.drawTextCentered(display.width() / 2, 56, word);
        display.setTextSize(1);
        if (raw_compass.calibration_result != SensorManager::COMPASS_CAL_RESULT_CANCELLED) {
          display.setColor(DisplayDriver::LIGHT);
          snprintf(tmp, sizeof(tmp), "quality %u", raw_compass.calibration_final_quality);
          display.drawTextCentered(display.width() / 2, 90, tmp);
        }
        compass_filter_valid = false;
        return 300;
      }
      if (raw_compass.calibrating) {
        // Instruction is driven by the firmware-reported step, not progress thresholds.
        uint8_t steps = raw_compass.calibration_step_count ? raw_compass.calibration_step_count : 3;
        display.drawTextCentered(display.width() / 2, 14, "Compass Cal");
        display.setColor(DisplayDriver::LIGHT);
        snprintf(tmp, sizeof(tmp), "Step %u/%u", raw_compass.calibration_step, steps);
        display.drawTextCentered(display.width() / 2, 30, tmp);
        const char* instr;
        switch (raw_compass.calibration_step) {
          case 1:  instr = "Rotate flat"; break;
          case 2:  instr = "Tilt & roll sides"; break;
          default: instr = "Move all angles"; break;
        }
        display.drawTextCentered(display.width() / 2, 44, instr);
        drawTinyProgress(display, 19, 62, 90, 8, raw_compass.calibration_progress);
        snprintf(tmp, sizeof(tmp), "%u%%  q%u", raw_compass.calibration_progress, raw_compass.calibration_quality);
        display.drawTextCentered(display.width() / 2, 80, tmp);
        display.setTextSize(2);
        snprintf(tmp, sizeof(tmp), "%02us", raw_compass.calibration_seconds_remaining);
        display.drawTextCentered(display.width() / 2, 100, tmp);
        display.setTextSize(1);
        display.drawTextCentered(display.width() / 2, 120, "Back: cancel");
        compass_filter_valid = false;
        return 250;
      }

      compass = smoothCompassReading(raw_compass);
      compass_mode = resolveCompassMode(compass);
      display.drawTextCentered(display.width() / 2, 16, compass_mode ? "Compass" : "Alignment");
      display.setColor(DisplayDriver::GREEN);
      if (compass_mode) {
        drawCompassWidget(display, display.width() / 2, 62, 32, compass);
      } else {
        drawAntennaAlignment(display, compass);
      }
      display.setColor(DisplayDriver::LIGHT);
      display.setTextSize(2);
      int primary_val;
      if (compass_mode) {
        primary_val = ((int)(normalizeDeg(compass.heading_deg + UI_HEADING_RENDER_OFFSET_DEG) + 0.5f)) % 360;
        snprintf(tmp, sizeof(tmp), "%03d deg", primary_val);
      } else {
        primary_val = (int)(antennaAlignmentTiltDeg(compass) + 0.5f);
        snprintf(tmp, sizeof(tmp), "TILT %02d", primary_val);
      }
      display.drawTextCentered(display.width() / 2, 97, tmp);
      display.setTextSize(1);
      float pitch_status = compass_mode ? compass.pitch_deg : antennaPitchErrorDeg(compass.pitch_deg);
      pitch_status = deadbandFloat(pitch_status, UI_COMPASS_DISPLAY_DEADBAND_DEG);
      float roll_status = deadbandFloat(compass.roll_deg, UI_COMPASS_DISPLAY_DEADBAND_DEG);
      int pitch_val = (int)(pitch_status + (pitch_status >= 0 ? 0.5f : -0.5f));
      int roll_val = (int)(roll_status + (roll_status >= 0 ? 0.5f : -0.5f));
      snprintf(tmp, sizeof(tmp), "%s P%+03d R%+03d",
              compassCalLabel(compass), pitch_val, roll_val);
      display.drawTextCentered(display.width() / 2, 119, tmp);

      // Adaptive cadence: full rate only while the drawn values change.
      bool changed = primary_val != compass_last_primary ||
                     pitch_val != compass_last_pitch ||
                     roll_val != compass_last_roll ||
                     compass_mode != compass_last_mode;
      compass_last_primary = primary_val;
      compass_last_pitch = pitch_val;
      compass_last_roll = roll_val;
      compass_last_mode = compass_mode;
      return changed ? UI_COMPASS_REFRESH_MS : UI_COMPASS_REFRESH_IDLE_MS;
#endif
#if UI_DIRECTIONAL_ANTENNA == 1
    } else if (_page == HomePage::DIRECTIONAL) {
      renderDirectionalList(display);
      DirectionalPingStatus status;
      return (the_mesh.isRepeaterDiscoveryActive() || the_mesh.getDirectionalPingStatus(status)) ? 500 : 2000;
#endif
#if UI_SENSORS_PAGE == 1
    } else if (_page == HomePage::SENSORS) {
      int y = 18;
      refresh_sensors();
      char buf[40];
      char name[30];
      LPPReader r(sensors_lpp.getBuffer(), sensors_lpp.getSize());

      for (int i = 0; i < sensors_scroll_offset; i++) {
        uint8_t channel, type;
        r.readHeader(channel, type);
        r.skipData(type);
      }

      for (int i = 0; i < (sensors_scroll?UI_RECENT_LIST_SIZE:sensors_nb); i++) {
        uint8_t channel, type;
        if (!r.readHeader(channel, type)) { // reached end, reset
          r.reset();
          r.readHeader(channel, type);
        }

        display.setCursor(0, y);
        float v;
        switch (type) {
          case LPP_GPS: // GPS
            float lat, lon, alt;
            r.readGPS(lat, lon, alt);
            strcpy(name, "gps"); snprintf(buf, sizeof(buf), "%.4f %.4f", lat, lon);
            break;
          case LPP_VOLTAGE:
            r.readVoltage(v);
            strcpy(name, "voltage"); snprintf(buf, sizeof(buf), "%6.2f", v);
            break;
          case LPP_CURRENT:
            r.readCurrent(v);
            strcpy(name, "current"); snprintf(buf, sizeof(buf), "%.3f", v);
            break;
          case LPP_TEMPERATURE:
            r.readTemperature(v);
            strcpy(name, "temperature"); snprintf(buf, sizeof(buf), "%.2f", v);
            break;
          case LPP_RELATIVE_HUMIDITY:
            r.readRelativeHumidity(v);
            strcpy(name, "humidity"); snprintf(buf, sizeof(buf), "%.2f", v);
            break;
          case LPP_BAROMETRIC_PRESSURE:
            r.readPressure(v);
            strcpy(name, "pressure"); snprintf(buf, sizeof(buf), "%.2f", v);
            break;
          case LPP_ALTITUDE:
            r.readAltitude(v);
            strcpy(name, "altitude"); snprintf(buf, sizeof(buf), "%.0f", v);
            break;
          case LPP_DIRECTION:
            r.readDirection(v);
            strcpy(name, "heading"); snprintf(buf, sizeof(buf), "%.0f deg", v);
            break;
          case LPP_ACCELEROMETER:
            float ax, ay, az;
            r.readAccelerometer(ax, ay, az);
            strcpy(name, "accel"); snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", ax, ay, az);
            break;
          case LPP_GYROMETER:
            float gx, gy, gz;
            r.readGyrometer(gx, gy, gz);
            strcpy(name, "gyro"); snprintf(buf, sizeof(buf), "%.0f %.0f %.0f", gx, gy, gz);
            break;
          case LPP_POWER:
            r.readPower(v);
            strcpy(name, "power"); snprintf(buf, sizeof(buf), "%6.2f", v);
            break;
          default:
            r.skipData(type);
            strcpy(name, "unk"); buf[0] = 0;
        }
        display.setCursor(0, y);
        display.print(name);
        display.setCursor(
          display.width()-display.getTextWidth(buf)-1, y
        );
        display.print(buf);
        y = y + 12;
      }
      if (sensors_scroll) sensors_scroll_offset = (sensors_scroll_offset+1)%sensors_nb;
      else sensors_scroll_offset = 0;
#endif
    } else if (_page == HomePage::PROFILE) {
      return renderProfilePage(display);
    } else if (_page == HomePage::SHUTDOWN) {
      display.setColor(DisplayDriver::GREEN);
      display.setTextSize(1);
      if (_shutdown_init) {
        display.drawTextCentered(display.width() / 2, 34 + UI_HOME_ACTION_Y_OFFSET, "hibernating...");
      } else {
        display.drawXbm((display.width() - 32) / 2, 18 + UI_HOME_ACTION_Y_OFFSET, power_icon, 32, 32);
        display.drawTextCentered(display.width() / 2, 64 - 11 + UI_HOME_ACTION_Y_OFFSET, "hibernate:" PRESS_LABEL);
      }
    }
    return battery_charging ? 500 : 5000;   // blink the charging indicator while plugged in
  }

  bool handleInput(char c) override {
    if (_diag_open) {   // hidden diagnostics page: Back closes it, swallow other keys
      if (c == KEY_CANCEL) _diag_open = false;
      return true;
    }
#if UI_QUICK_SEND == 1
    if (_page == HomePage::CHANNELS || _page == HomePage::GROUP) {
      if (quick_stage == QUICK_KEYBOARD) {
        return handleQuickKeyboardInput(c);
      }

      int count = quick_stage == QUICK_ACTIONS ? quickActionCount() : getCurrentQuickTargetCount();
      int* selected = quick_stage == QUICK_ACTIONS ? &quick_action_selected : &currentQuickSelected();

      if (c == KEY_CANCEL) {
        if (quick_stage == QUICK_ACTIONS) {
          quick_stage = QUICK_TARGETS;
        } else {
          _page = HomePage::FIRST;
        }
        return true;
      }
      if (c == KEY_UP || c == KEY_PREV) {
        if (count > 0) *selected = (*selected + count - 1) % count;
        return true;
      }
      if (c == KEY_DOWN || c == KEY_NEXT) {
        if (count > 0) *selected = (*selected + 1) % count;
        return true;
      }
      if (c == KEY_LEFT) {
        if (quick_stage == QUICK_ACTIONS) {
          quick_stage = QUICK_TARGETS;
          return true;
        }
      }
      if (c == KEY_ENTER) {
        if (quick_stage == QUICK_TARGETS) {
          if (count == 0) {
            _task->showAlert(_page == HomePage::CHANNELS ? "No text channels" : "No favorites", 800);
          } else if (pinQuickTarget()) {  // capture recipient identity now
            quick_stage = QUICK_ACTIONS;
            quick_action_selected = 0;
          } else {
            _task->showAlert("No chat", 800);
          }
        } else {
          sendQuickSelection();
        }
        return true;
      }
    }
#endif
#if UI_DIRECTIONAL_ANTENNA == 1
    if (_page == HomePage::DIRECTIONAL) {
      directional_dirty = true;   // one rebuild for this input pass
      int count = getDirectionalTargetCount();
      if (c == KEY_CANCEL) {
        _page = HomePage::FIRST;
        return true;
      }
      if (c == KEY_UP || c == KEY_PREV) {
        if (count > 0) directional_selected = (directional_selected + count - 1) % count;
        return true;
      }
      if (c == KEY_DOWN || c == KEY_NEXT) {
        if (count > 0) directional_selected = (directional_selected + 1) % count;
        return true;
      }
      if (c == KEY_ENTER) {
        directional_discovery_started = true;
        directional_selected = 0;
        the_mesh.clearDirectionalPingStatus();
        if (the_mesh.startRepeaterDiscovery(UI_REPEATER_DISCOVERY_MS)) {
          _task->notify(UIEventType::ack);
          _task->showAlert("Scanning repeaters", 1200);
        } else {
          _task->showAlert("Scan failed", 1000);
        }
        return true;
      }
      if (c == KEY_CONTEXT_MENU) {
        if (count == 0) {
          _task->showAlert("No repeater", 800);
        } else {
          sendDirectionalPing();
        }
        return true;
      }
    }
#endif
    if (c == KEY_LEFT || c == KEY_UP || c == KEY_PREV) {
      _page = (_page + HomePage::Count - 1) % HomePage::Count;
      resetQuickSendStage();
      return true;
    }
    if (c == KEY_NEXT || c == KEY_RIGHT || c == KEY_DOWN) {
      _page = (_page + 1) % HomePage::Count;
      resetQuickSendStage();
      if (_page == HomePage::RECENT) {
        _task->showAlert("Recent adverts", 800);
      }
      return true;
    }
    if (c == KEY_ENTER && _page == HomePage::BLUETOOTH) {
      if (_task->isSerialEnabled()) {  // toggle Bluetooth on/off
        _task->disableSerial();
      } else {
        _task->enableSerial();
      }
      return true;
    }
    if (c == KEY_ENTER && _page == HomePage::ADVERT) {
      _task->notify(UIEventType::ack);
      if (the_mesh.advert()) {
        _task->showAlert("Advert sent!", 1000);
      } else {
        _task->showAlert("Advert failed", 1000);
      }
      return true;
    }
#if ENV_INCLUDE_GPS == 1
    if (c == KEY_ENTER && _page == HomePage::GPS) {
      _task->toggleGPS();
      return true;
    }
#endif
#if UI_HAS_COMPASS == 1
    if (c == KEY_CONTEXT_MENU && _page == HomePage::COMPASS) {
      if (_sensors != NULL && _sensors->setSettingValue("compass_cal", "start")) {
        compass_filter_valid = false;
        compass_mode_valid = false;
        _task->showAlert("Rotate 30s", 1000);
      } else {
        _task->showAlert("Compass unavailable", 800);
      }
      return true;
    }
    if (c == KEY_CANCEL && _page == HomePage::COMPASS) {
      SensorManager::CompassReading r;
      if (_sensors != NULL && _sensors->getCompass(r) && r.calibrating) {
        _sensors->setSettingValue("compass_cal", "cancel");  // abort, keep the prior saved cal
        compass_filter_valid = false;
        _task->showAlert("Cal cancelled", 800);
        return true;
      }
    }
    // ENTER intentionally unbound on the compass page: calibration starts on long-press.
    // Back during an active calibration cancels non-destructively (keeps the prior good cal).
#endif
#if UI_SENSORS_PAGE == 1
    if (c == KEY_ENTER && _page == HomePage::SENSORS) {
      _task->toggleGPS();
      next_sensors_refresh=0;
      return true;
    }
#endif
    if (c == KEY_CONTEXT_MENU && _page == HomePage::FIRST) {  // hidden: long-press home -> diagnostics
      _diag_open = true;
      _task->showAlert("Diagnostics", 600);
      return true;
    }
    if (c == KEY_ENTER && _page == HomePage::PROFILE) {       // cycle power profile + apply
      uint8_t next = (uint8_t)((_node_prefs->power_profile + 1) % SensorManager::POWER_PROFILE_COUNT);
      _node_prefs->power_profile = next;
      the_mesh.savePrefs();
      the_mesh.applyPowerProfile();
      _task->notify(UIEventType::ack);
      char m[24]; snprintf(m, sizeof(m), "Mode: %s", SensorManager::powerProfileName(next));
      _task->showAlert(m, 1000);
      return true;
    }
    if (c == KEY_ENTER && _page == HomePage::SHUTDOWN) {
      _shutdown_init = true;  // need to wait for button to be released
      return true;
    }
    return false;
  }
};

class MsgPreviewScreen : public UIScreen {
  UITask* _task;
  mesh::RTCClock* _rtc;

  struct MsgEntry {
    uint32_t timestamp;
    char origin[62];
    char msg[78];
  };
  #define MAX_UNREAD_MSGS   32
  int num_unread;
  int head = MAX_UNREAD_MSGS - 1; // index of latest unread message
  MsgEntry unread[MAX_UNREAD_MSGS];

public:
  MsgPreviewScreen(UITask* task, mesh::RTCClock* rtc) : _task(task), _rtc(rtc) { num_unread = 0; }

  void addPreview(uint8_t path_len, const char* from_name, const char* msg) {
    head = (head + 1) % MAX_UNREAD_MSGS;
    if (num_unread < MAX_UNREAD_MSGS) num_unread++;

    auto p = &unread[head];
    p->timestamp = _rtc->getCurrentTime();
    if (path_len == 0xFF) {
      snprintf(p->origin, sizeof(p->origin), "(D) %s:", from_name);
    } else {
      snprintf(p->origin, sizeof(p->origin), "(%d) %s:", (uint32_t) path_len, from_name);
    }
    StrHelper::strncpy(p->msg, msg, sizeof(p->msg));
  }

  int render(DisplayDriver& display) override {
    char tmp[16];
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setColor(DisplayDriver::GREEN);
    snprintf(tmp, sizeof(tmp), "Unread: %d", num_unread);
    display.print(tmp);

    auto p = &unread[head];

    int secs = _rtc->getCurrentTime() - p->timestamp;
    if (secs < 60) {
      snprintf(tmp, sizeof(tmp), "%ds", secs);
    } else if (secs < 60*60) {
      snprintf(tmp, sizeof(tmp), "%dm", secs / 60);
    } else {
      snprintf(tmp, sizeof(tmp), "%dh", secs / (60*60));
    }
    display.setCursor(display.width() - display.getTextWidth(tmp) - 2, 0);
    display.print(tmp);

    display.drawRect(0, 11, display.width(), 1);  // horiz line

    display.setCursor(0, 14);
    display.setColor(DisplayDriver::YELLOW);
    char filtered_origin[sizeof(p->origin)];
    display.translateUTF8ToBlocks(filtered_origin, p->origin, sizeof(filtered_origin));
    display.print(filtered_origin);

    display.setCursor(0, 25);
    display.setColor(DisplayDriver::LIGHT);
    char filtered_msg[sizeof(p->msg)];
    display.translateUTF8ToBlocks(filtered_msg, p->msg, sizeof(filtered_msg));
    display.printWordWrap(filtered_msg, display.width());

#if AUTO_OFF_MILLIS==0 // probably e-ink
    return 10000; // 10 s
#else
    return 1000;  // next render after 1000 ms
#endif
  }

  bool handleInput(char c) override {
    if (c == KEY_NEXT || c == KEY_RIGHT) {
      head = (head + MAX_UNREAD_MSGS - 1) % MAX_UNREAD_MSGS;
      num_unread--;
      if (num_unread == 0) {
        _task->gotoHomeScreen();
      }
      return true;
    }
    if (c == KEY_ENTER) {
      num_unread = 0;  // clear unread queue
      _task->gotoHomeScreen();
      return true;
    }
    return false;
  }
};

void UITask::begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs) {
  _display = display;
  _sensors = sensors;
  _auto_off = millis() + AUTO_OFF_MILLIS;
  _node_prefs = node_prefs;
  // Buzzer mute is a persistent user preference: honor the saved buzzer_quiet
  // value across reboots. Fresh devices default to 0 (buzzer on), so the
  // off-the-shelf experience is still "buzzer on" without overriding a user
  // who has deliberately muted it. (Previously MUZIWORKS_SUPER_IO_FORCE_BUZZER_ON
  // cleared the saved mute on every boot, discarding user intent.)
  _msgcount = 0;

#if defined(PIN_USER_BTN)
  user_btn.begin();
#endif
#if UI_HAS_DPAD
  joystick_up.begin();
  joystick_down.begin();
#endif
#if defined(PIN_USER_BTN_ANA)
  analog_btn.begin();
#endif
#if UI_HAS_JOYSTICK
  joystick_left.begin();
  joystick_right.begin();
  back_btn.begin();
#endif

#ifdef PIN_MSG_LED
  pinMode(PIN_MSG_LED, OUTPUT);
  setMsgLed(false);
#endif

#ifdef PIN_STATUS_LED
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, !LED_STATE_ON);
#endif

#ifdef PIN_BUZZER
  buzzer.begin();
  buzzer.quiet(_node_prefs->buzzer_quiet);
  buzzer.startup();
#endif
#ifdef PIN_ACTIVE_BUZZER
  activeBuzzerBegin();
  activeBuzzerQuiet(_node_prefs->buzzer_quiet);
#if ACTIVE_BUZZER_STARTUP_MS > 0
  activeBuzzerPlayPattern(active_buzzer_startup_pattern, sizeof(active_buzzer_startup_pattern) / sizeof(active_buzzer_startup_pattern[0]));
#endif
#endif

#if BATTERY_POWER_ATTACH_BEEP == 1
  battery_external_powered = _board != NULL && _board->isExternalPowered();
  battery_external_known = true;
#endif

#ifdef PIN_VIBRATION
  vibration.begin();
#endif

#ifdef UI_HAS_GPS_SWITCH
  gps_switch_begin();
#endif

#ifdef UI_HAS_GPS_SWITCH
  GpsSwitchState initial_switch_state = GPS_SWITCH_UNKNOWN;
  if (gps_switch_confirm_initial_state(initial_switch_state,
                                       GPS_SWITCH_BOOT_DEBOUNCE_MS,
                                       GPS_SWITCH_BOOT_OFF_CONFIRM_MS) &&
      applyGPSSwitchAction(initial_switch_state, false)) {
    // true = boot-held OFF initiated shutdown. Abort init here: the screen
    // objects (splash/home/msg_preview) are intentionally never constructed,
    // so nothing below this point may be moved above the early return.
    return;
  } else if (initial_switch_state == GPS_SWITCH_UNKNOWN) {
    MESH_DEBUG_PRINTLN("UITask: GPS switch boot state unstable");
  }
#endif

  if (_display != NULL) {
    _display->turnOn();
  }

  ui_started_at = millis();
  _alert_expiry = 0;

  splash = new SplashScreen(this);
  home = new HomeScreen(this, &rtc_clock, sensors, node_prefs);
  msg_preview = new MsgPreviewScreen(this, &rtc_clock);
  setCurrScreen(splash);
}

void UITask::showAlert(const char* text, int duration_millis) {
  strncpy(_alert, text, sizeof(_alert) - 1);   // _alert is a bounded buffer; truncate, never overflow
  _alert[sizeof(_alert) - 1] = 0;
  _alert_expiry = millis() + duration_millis;
}

void UITask::notify(UIEventType t) {
#if defined(PIN_BUZZER)
switch(t){
  case UIEventType::contactMessage:
    // gemini's pick
    buzzer.play("MsgRcv3:d=4,o=6,b=200:32e,32g,32b,16c7");
    break;
  case UIEventType::channelMessage:
    buzzer.play("kerplop:d=16,o=6,b=120:32g#,32c#");
    break;
  case UIEventType::ack:
    buzzer.play("ack:d=32,o=8,b=120:c");
    break;
  case UIEventType::roomMessage:
  case UIEventType::newContactMessage:
  case UIEventType::none:
  default:
    break;
}
#endif

#ifdef PIN_ACTIVE_BUZZER
switch(t){
  case UIEventType::contactMessage:
  case UIEventType::channelMessage:
    activeBuzzerPlayPattern(active_buzzer_msg_pattern, sizeof(active_buzzer_msg_pattern) / sizeof(active_buzzer_msg_pattern[0]));
    break;
  case UIEventType::ack:
    activeBuzzerPlayPattern(active_buzzer_ack_pattern, sizeof(active_buzzer_ack_pattern) / sizeof(active_buzzer_ack_pattern[0]));
    break;
  case UIEventType::roomMessage:
  case UIEventType::newContactMessage:
  case UIEventType::none:
  default:
    break;
}
#endif

#ifdef PIN_VIBRATION
  // Trigger vibration for all UI events except none
  if (t != UIEventType::none) {
    vibration.trigger();
  }
#endif
}

void UITask::playBatteryBeep() {
#ifdef PIN_ACTIVE_BUZZER
  activeBuzzerPlayPattern(active_buzzer_battery_pattern, sizeof(active_buzzer_battery_pattern) / sizeof(active_buzzer_battery_pattern[0]));
#elif defined(PIN_BUZZER)
  buzzer.play("batt:d=32,o=6,b=160:c,p,c5");
#endif
}

void UITask::batteryBeepHandler() {
#if BATTERY_BEEP_MILLIVOLTS > 0 || BATTERY_POWER_ATTACH_BEEP == 1
  unsigned long now = millis();
  if (now < next_battery_beep_check) return;
  next_battery_beep_check = now + 2000;

  bool external_powered = _board != NULL && _board->isExternalPowered();

#if BATTERY_POWER_ATTACH_BEEP == 1
  if (!battery_external_known) {
    battery_external_known = true;
  } else if (!battery_external_powered && external_powered) {
  #ifdef PIN_ACTIVE_BUZZER
    activeBuzzerPlayPattern(active_buzzer_power_pattern, sizeof(active_buzzer_power_pattern) / sizeof(active_buzzer_power_pattern[0]));
  #elif defined(PIN_BUZZER)
    buzzer.play("usb:d=32,o=6,b=180:e,g");
  #endif
  }
  battery_external_powered = external_powered;
#endif

#if BATTERY_BEEP_MILLIVOLTS > 0
  uint16_t milliVolts = getBattMilliVolts();
  if (external_powered || milliVolts > BATTERY_BEEP_MILLIVOLTS + 100) {
    next_low_battery_beep = now;
    return;
  }
  if (milliVolts > 0 && milliVolts <= BATTERY_BEEP_MILLIVOLTS && now >= next_low_battery_beep) {
    playBatteryBeep();
    showAlert("Battery low", 1200);
    next_low_battery_beep = now + BATTERY_BEEP_INTERVAL_MS;
  }
#endif
#endif
}


void UITask::msgRead(int msgcount) {
  _msgcount = msgcount;
#ifdef PIN_MSG_LED
  setMsgLed(msgcount > 0);
#endif
  if (msgcount == 0) {
    gotoHomeScreen();
  }
}

void UITask::newMsg(uint8_t path_len, const char* from_name, const char* text, int msgcount) {
  _msgcount = msgcount;
#ifdef PIN_MSG_LED
  setMsgLed(msgcount > 0);
#endif

  ((MsgPreviewScreen *) msg_preview)->addPreview(path_len, from_name, text);
  // Don't yank the user out of a modal typing surface: the next keystroke
  // would land in MsgPreviewScreen and could dismiss the whole unread queue.
  // The preview stays queued; notification (LED/buzzer) still fires below.
  bool typing = curr == home && home != NULL && ((HomeScreen *) home)->isModalInputActive();
  if (!typing) {
    setCurrScreen(msg_preview);
  }

  if (_display != NULL) {
    if (!_display->isOn() && !hasConnection()) {
      _display->turnOn();
    }
    if (_display->isOn()) {
    _auto_off = millis() + AUTO_OFF_MILLIS;  // extend the auto-off timer
    _next_refresh = 100;  // trigger refresh
    }
  }
}

void UITask::userLedHandler() {
#ifdef PIN_STATUS_LED
  unsigned long cur_time = millis();
#if STATUS_LED_HEARTBEAT_REQUIRES_GPS
  // getGPSState() walks the sensor settings table (strcmp scan) — only
  // re-check once per second instead of every loop() pass.
  if ((long)(cur_time - next_led_gps_check) >= 0) {
    led_gps_enabled = getGPSState();
    next_led_gps_check = cur_time + 1000;
  }
  if (!led_gps_enabled) {
    if (led_state != 0) {
      led_state = 0;
      digitalWrite(PIN_STATUS_LED, !LED_STATE_ON);
    }
    next_led_change = 0;
    last_led_increment = 0;
    return;
  }
#endif
  if ((long)(cur_time - next_led_change) >= 0) {
    if (led_state == 0) {
      led_state = 1;
      if (_msgcount > 0) {
        last_led_increment = LED_ON_MSG_MILLIS;
      } else {
        last_led_increment = LED_ON_MILLIS;
      }
      next_led_change = cur_time + last_led_increment;
    } else {
      led_state = 0;
      next_led_change = cur_time + LED_CYCLE_MILLIS - last_led_increment;
    }
    digitalWrite(PIN_STATUS_LED, led_state ? LED_STATE_ON : !LED_STATE_ON);
  }
#endif
}

void UITask::setCurrScreen(UIScreen* c) {
  curr = c;
  _next_refresh = 100;
}

/*
  hardware-agnostic pre-shutdown activity should be done here
*/
void UITask::shutdown(bool restart, const char* message){

  // Visible confirmation for deliberate power-state transitions.
  if (!restart && message != NULL && _display != NULL && _display->isOn()) {
    _display->startFrame();
    _display->setColor(DisplayDriver::GREEN);
    _display->setTextSize(1);
    _display->drawTextCentered(_display->width() / 2, _display->height() / 2 - 4, message);
    _display->endFrame();
    delay(700);
  }

  #ifdef PIN_BUZZER
  /* note: we have a choice here -
     we can do a blocking buzzer.loop() with non-deterministic consequences
     or we can set a flag and delay the shutdown for a couple of seconds
     while a non-blocking buzzer.loop() plays out in UITask::loop()
  */
  buzzer.shutdown();
  uint32_t buzzer_timer = millis(); // fail-safe shutdown
  while (buzzer.isPlaying() && (millis() - 2500) < buzzer_timer)
    buzzer.loop();

  #endif // PIN_BUZZER

#ifdef PIN_ACTIVE_BUZZER
#if ACTIVE_BUZZER_SHUTDOWN_MS > 0
  uint32_t active_buzzer_wait = activeBuzzerPatternDuration(active_buzzer_shutdown_pattern, sizeof(active_buzzer_shutdown_pattern) / sizeof(active_buzzer_shutdown_pattern[0])) + 5;
  activeBuzzerPlayPattern(active_buzzer_shutdown_pattern, sizeof(active_buzzer_shutdown_pattern) / sizeof(active_buzzer_shutdown_pattern[0]));
  uint32_t active_buzzer_timer = millis();
  while (active_buzzer_until != 0 &&
         (uint32_t)(millis() - active_buzzer_timer) <= active_buzzer_wait) {
    activeBuzzerLoop();
  }
#endif
  activeBuzzerStop();
#endif

#ifdef PIN_MSG_LED
  setMsgLed(false);
#endif

  if (restart) {
    _board->reboot();
  } else {
    if (_display != NULL) {
      _display->turnOff();
    }
    radio_driver.powerOff();
    _board->powerOff();
  }
}

bool UITask::isButtonPressed() const {
#ifdef PIN_USER_BTN
  return user_btn.isPressed();
#else
  return false;
#endif
}

void UITask::loop() {
  char c = 0;
#if UI_HAS_JOYSTICK
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_CONTEXT_MENU);
  }
#if UI_HAS_DPAD
  ev = joystick_up.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_UP);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_UP);
  }
  ev = joystick_down.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_DOWN);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_DOWN);
  }
#endif
  ev = joystick_left.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_LEFT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_LEFT);
  }
  ev = joystick_right.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_RIGHT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_RIGHT);
  }
  ev = back_btn.check();
#if UI_HAS_DPAD
  if (ev == BUTTON_EVENT_CLICK) {
    c = handleCancelPress();
  } else
#endif
  if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
#elif defined(PIN_USER_BTN)
  int ev = user_btn.check();
  if (ev == BUTTON_EVENT_CLICK) {
    c = checkDisplayOn(KEY_NEXT);
  } else if (ev == BUTTON_EVENT_LONG_PRESS) {
    c = handleLongPress(KEY_ENTER);
  } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
    c = handleDoubleClick(KEY_PREV);
  } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
    c = handleTripleClick(KEY_SELECT);
  }
#endif
#if defined(PIN_USER_BTN_ANA)
  if (abs(millis() - _analogue_pin_read_millis) > 10) {
    int ev = analog_btn.check();
    if (ev == BUTTON_EVENT_CLICK) {
      c = checkDisplayOn(KEY_NEXT);
    } else if (ev == BUTTON_EVENT_LONG_PRESS) {
      c = handleLongPress(KEY_ENTER);
    } else if (ev == BUTTON_EVENT_DOUBLE_CLICK) {
      c = handleDoubleClick(KEY_PREV);
    } else if (ev == BUTTON_EVENT_TRIPLE_CLICK) {
      c = handleTripleClick(KEY_SELECT);
    }
    _analogue_pin_read_millis = millis();
  }
#endif
#ifdef UI_HAS_GPS_SWITCH
  GpsSwitchState switch_state;
  if (gps_switch_poll(switch_state)) {
    if (applyGPSSwitchAction(switch_state, true)) return;
  }
#endif
#if defined(BACKLIGHT_BTN)
  if (millis() > next_backlight_btn_check) {
    bool touch_state = digitalRead(PIN_BUTTON2);
#if defined(DISP_BACKLIGHT)
    digitalWrite(DISP_BACKLIGHT, !touch_state);
#elif defined(EXP_PIN_BACKLIGHT)
    expander.digitalWrite(EXP_PIN_BACKLIGHT, !touch_state);
#endif
    next_backlight_btn_check = millis() + 300;
  }
#endif

  if (c != 0 && curr) {
    curr->handleInput(c);
    _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
    _next_refresh = 100;  // trigger refresh
  }

  userLedHandler();

#ifdef PIN_BUZZER
  if (buzzer.isPlaying())  buzzer.loop();
#endif
#ifdef PIN_ACTIVE_BUZZER
  activeBuzzerLoop();
#endif
  batteryBeepHandler();

  if (curr) curr->poll();

  if (_display != NULL && _display->isOn()) {
    if (millis() >= _next_refresh && curr) {
      _display->startFrame();
      int delay_millis = curr->render(*_display);
      if (millis() < _alert_expiry) {  // render alert popup
        _display->setTextSize(1);
        int y = _display->height() / 3;
        int p = _display->height() / 32;
        _display->setColor(DisplayDriver::DARK);
        _display->fillRect(p, y, _display->width() - p*2, y);
        _display->setColor(DisplayDriver::LIGHT);  // draw box border
        _display->drawRect(p, y, _display->width() - p*2, y);
        _display->drawTextCentered(_display->width() / 2, y + p*3, _alert);
        _next_refresh = _alert_expiry;   // will need refresh when alert is dismissed
      } else {
        _next_refresh = millis() + delay_millis;
      }
      _display->endFrame();
    }
#if AUTO_OFF_MILLIS > 0
#ifdef KEEP_DISPLAY_ON_USB
    // Opt-in: refresh the auto-off deadline while externally powered, so the
    // timer counts from the moment external power is removed. Off by default
    // because OLED panels burn in quickly; only enable for LCD targets or
    // where the display is replaceable.
    if (board.isExternalPowered()) {
      _auto_off = millis() + AUTO_OFF_MILLIS;
    }
#endif
    if (millis() > _auto_off) {
      _display->turnOff();
    }
#endif
  }

#ifdef PIN_VIBRATION
  vibration.loop();
#endif

#ifdef AUTO_SHUTDOWN_MILLIVOLTS
  if (millis() > next_batt_chck) {
    uint16_t milliVolts = getBattMilliVolts();
    if (milliVolts > 0 && milliVolts < AUTO_SHUTDOWN_MILLIVOLTS) {
      if(!board.isExternalPowered()) {
        if (_display != NULL) {
          _display->startFrame();
          _display->setTextSize(2);
          _display->setColor(DisplayDriver::RED);
          _display->drawTextCentered(_display->width() / 2, 20, "Low Battery.");
          _display->drawTextCentered(_display->width() / 2, 40, "Shutting Down!");
          _display->endFrame();
          if (_display->isEink() == false) { delay(3000); }
        }
        shutdown();
      }
    }
    next_batt_chck = millis() + 8000;
  }
#endif
}

char UITask::checkDisplayOn(char c) {
  if (_display != NULL) {
    if (!_display->isOn()) {
      _display->turnOn();   // turn display on and consume event
      c = 0;
    }
    _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
    _next_refresh = 0;  // trigger refresh
  }
  return c;
}

char UITask::handleLongPress(char c) {
  if (millis() - ui_started_at < 8000) {   // long press in first 8 seconds since startup -> CLI/rescue
    the_mesh.enterCLIRescue();
    return 0;   // consume event
  }
  // Long-press bindings include destructive actions (compass calibration,
  // repeater ping TX, diagnostics). With the display off — e.g. squeezed in a
  // pocket — just wake the screen and consume the event, mirroring
  // checkDisplayOn() for short presses.
  return checkDisplayOn(c);
}

char UITask::handleDoubleClick(char c) {
  MESH_DEBUG_PRINTLN("UITask: double-click triggered");
  checkDisplayOn(c);
  return c;
}

char UITask::handleTripleClick(char c) {
  MESH_DEBUG_PRINTLN("UITask: triple click triggered");
  checkDisplayOn(c);
  toggleBuzzer();
  c = 0;
  return c;
}

char UITask::handleCancelPress() {
  if (_display != NULL && !_display->isOn()) {
    _display->turnOn();
    return 0;
  }
  if (millis() < _alert_expiry) {
    _alert_expiry = 0;
    _next_refresh = 0;
    return 0;
  }
  if (curr != home) {
    gotoHomeScreen();
    return 0;
  }
  return KEY_CANCEL;
}

bool UITask::getGPSState() {
  if (_sensors != NULL) {
    int num = _sensors->getNumSettings();
    for (int i = 0; i < num; i++) {
      if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
        return !strcmp(_sensors->getSettingValue(i), "1");
      }
    }
  }
  return false;
}

bool UITask::setGPSState(bool enabled, bool persist, bool alert) {
  if (_sensors == NULL) return false;

  int num = _sensors->getNumSettings();
  for (int i = 0; i < num; i++) {
    if (strcmp(_sensors->getSettingName(i), "gps") == 0) {
      const char* desired = enabled ? "1" : "0";
      bool changed = strcmp(_sensors->getSettingValue(i), desired) != 0;
      _sensors->setSettingValue("gps", desired);
      if (persist && _node_prefs != NULL) {
        _node_prefs->gps_enabled = enabled ? 1 : 0;
      }
      if (persist) {
        the_mesh.savePrefs();
      }
      if (alert && changed) {
        notify(UIEventType::ack);
        showAlert(enabled ? "GPS: Enabled" : "GPS: Disabled", 800);
        _next_refresh = 0;
      }
      return true;
    }
  }
  return false;
}

void UITask::toggleGPS() {
#ifdef UI_HAS_GPS_SWITCH
  GpsSwitchState switch_state = gps_switch_get_stable();
  applyGPSSwitchPolicy(switch_state, false);
  showGPSSwitchFeedback(switch_state);
  return;
#endif
  setGPSState(!getGPSState(), true, true);
}

#ifdef PIN_MSG_LED
void UITask::setMsgLed(bool on) {
  digitalWrite(PIN_MSG_LED, on ? MSG_LED_STATE_ON : !MSG_LED_STATE_ON);
}
#endif

#ifdef PIN_ACTIVE_BUZZER
void UITask::activeBuzzerBegin() {
  pinMode(PIN_ACTIVE_BUZZER, OUTPUT);
  activeBuzzerStop();
}

void UITask::activeBuzzerPlayPattern(const uint16_t* pattern, uint8_t pattern_len) {
  if (active_buzzer_quiet || pattern == NULL || pattern_len == 0) return;
  active_buzzer_pattern = pattern;
  active_buzzer_pattern_len = pattern_len;
  active_buzzer_pattern_pos = 0;
  activeBuzzerAdvance();
}

uint32_t UITask::activeBuzzerPatternDuration(const uint16_t* pattern, uint8_t pattern_len) {
  uint32_t duration = 0;
  if (pattern == NULL) return 0;
  for (uint8_t i = 0; i < pattern_len; i++) {
    duration += pattern[i];
  }
  return duration;
}

void UITask::activeBuzzerAdvance() {
  while (active_buzzer_pattern != NULL && active_buzzer_pattern_pos < active_buzzer_pattern_len) {
    uint16_t duration_ms = active_buzzer_pattern[active_buzzer_pattern_pos];
    bool on = (active_buzzer_pattern_pos % 2) == 0;
    active_buzzer_pattern_pos++;
    if (duration_ms == 0) continue;
    digitalWrite(PIN_ACTIVE_BUZZER, on ? ACTIVE_BUZZER_ON : !ACTIVE_BUZZER_ON);
    active_buzzer_until = millis() + duration_ms;
    return;
  }
  activeBuzzerStop();
}

void UITask::activeBuzzerStop() {
  digitalWrite(PIN_ACTIVE_BUZZER, !ACTIVE_BUZZER_ON);
  active_buzzer_until = 0;
  active_buzzer_pattern = NULL;
  active_buzzer_pattern_len = 0;
  active_buzzer_pattern_pos = 0;
}

void UITask::activeBuzzerLoop() {
  if (active_buzzer_until != 0 && (long)(millis() - active_buzzer_until) >= 0) {
    if (active_buzzer_pattern != NULL) {
      activeBuzzerAdvance();
    } else {
      activeBuzzerStop();
    }
  }
}

void UITask::activeBuzzerQuiet(bool quiet) {
  active_buzzer_quiet = quiet;
  if (active_buzzer_quiet) activeBuzzerStop();
}
#endif

#ifdef UI_HAS_GPS_SWITCH
bool UITask::applyGPSSwitchPolicy(int state, bool alert) {
  // Neutral, switch-position wording (not GPS-state wording) so both the
  // physical-switch path and the soft path read the same. Enum names are owned
  // by systems and kept as-is; only the user-facing strings live here.
  bool ok = false;
  const char* label = NULL;
  switch (state) {
    case GPS_SWITCH_ON:                          // device on, GPS off
      ok = setGPSState(false, false, false);
      label = "switch: ON";
      break;
    case GPS_SWITCH_ON_GPS:
#if MUZIWORKS_SMART_GPS
      ok = true;                                 // Smart GPS controls the receiver
#else
      ok = setGPSState(true, false, false);
#endif
      label = "switch: ON-GPS";
      break;
    default:
      return false;
  }
  if (alert && label != NULL) {
    notify(UIEventType::ack);
    showAlert(label, 800);
    _next_refresh = 0;
  }
  return ok;
}

bool UITask::applyGPSSwitchAction(int state, bool alert) {
  MESH_DEBUG_PRINTLN("UITask: GPS switch action state=%d", state);
  switch (state) {
    case GPS_SWITCH_ON:
    case GPS_SWITCH_ON_GPS:
      applyGPSSwitchPolicy(state, alert);
      return false;
    case GPS_SWITCH_OFF:
      setGPSState(false, false, false);
      if (!alert) {
        // Boot-held OFF is deliberate and can enter system-off after the long
        // boot confirm. At runtime the slide switch can briefly present the OFF
        // GPIO combo while moving between ON and ON-GPS, so keep runtime OFF
        // non-destructive until the hardware transition behavior is fully
        // characterized on production units.
        shutdown();
        return true;
      }
      notify(UIEventType::ack);
      showAlert("switch: OFF", 1000);
      _next_refresh = 0;
      return false;
    case GPS_SWITCH_UNKNOWN:
    default:
      return false;
  }
}

void UITask::showGPSSwitchFeedback(int state) {
  const char* label;
  switch (state) {
    case GPS_SWITCH_ON:     label = "switch: ON";      break;  // device on, GPS off
    case GPS_SWITCH_ON_GPS: label = "switch: ON-GPS";  break;
    case GPS_SWITCH_OFF:    label = "switch: OFF";      break;
    case GPS_SWITCH_UNKNOWN:
    default:                label = "switch: ?";       break;
  }
  showAlert(label, 800);
  _next_refresh = 0;
}
#endif

void UITask::toggleBuzzer() {
    // Toggle buzzer quiet mode
  #ifdef PIN_BUZZER
    if (buzzer.isQuiet()) {
      buzzer.quiet(false);
      notify(UIEventType::ack);
    } else {
      buzzer.quiet(true);
    }
    _node_prefs->buzzer_quiet = buzzer.isQuiet();
    the_mesh.savePrefs();
    showAlert(buzzer.isQuiet() ? "Buzzer: OFF" : "Buzzer: ON", 800);
    _next_refresh = 0;  // trigger refresh
  #elif defined(PIN_ACTIVE_BUZZER)
    if (active_buzzer_quiet) {
      activeBuzzerQuiet(false);
      notify(UIEventType::ack);
    } else {
      activeBuzzerQuiet(true);
    }
    _node_prefs->buzzer_quiet = active_buzzer_quiet;
    the_mesh.savePrefs();
    showAlert(active_buzzer_quiet ? "Buzzer: OFF" : "Buzzer: ON", 800);
    _next_refresh = 0;  // trigger refresh
  #endif
}
