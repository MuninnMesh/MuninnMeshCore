#ifdef ESP_PLATFORM

#include "ESP32Board.h"

#if defined(ADMIN_PASSWORD) && !defined(DISABLE_WIFI_OTA)   // Repeater or Room Server only
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <SPIFFS.h>

#ifndef WIFI_OTA_IDLE_TIMEOUT_MINS
  #define WIFI_OTA_IDLE_TIMEOUT_MINS 0   // 0 = AP stays up until reboot (upstream behavior)
#endif

#if WIFI_OTA_IDLE_TIMEOUT_MINS > 0
#include <esp_timer.h>

static esp_timer_handle_t ota_idle_timer = NULL;
static volatile uint32_t ota_deadline_millis = 0;
static ESP32Board* ota_board = NULL;

// Runs every 60s while OTA mode is active. A connected station (someone on the
// /update page or mid-upload) keeps extending the deadline; once the AP has
// been idle past the timeout, tear it down so an unattended `start ota` can't
// leave an open AP running forever.
static void ota_idle_check(void*) {
  if (WiFi.softAPgetStationNum() > 0) {
    ota_deadline_millis = millis() + 5UL * 60UL * 1000UL;
    return;
  }
  if ((int32_t)(millis() - ota_deadline_millis) >= 0 && ota_board) {
    ota_board->endOTAUpdate();
  }
}
#endif

bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  inhibit_sleep = true;   // prevent sleep during OTA
  WiFi.softAP("MeshCore-OTA", NULL);

  sprintf(reply, "Started: http://%s/update", WiFi.softAPIP().toString().c_str());
  MESH_DEBUG_PRINTLN("startOTAUpdate: %s", reply);

  static char id_buf[60];
  sprintf(id_buf, "%s (%s)", id, getManufacturerName());
  static char home_buf[90];
  sprintf(home_buf, "<H2>Hi! I am a MeshCore Repeater. ID: %s</H2>", id);

  // The server survives an idle-timeout teardown (only WiFi is stopped), so a
  // later `start ota` must not create/register it a second time.
  static AsyncWebServer* server = NULL;
  if (server == NULL) {
    server = new AsyncWebServer(80);

    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html", home_buf);
    });
    server->on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/packet_log", "text/plain");
    });

    AsyncElegantOTA.setID(id_buf);
    AsyncElegantOTA.begin(server);    // Start ElegantOTA
    server->begin();
  }

#if WIFI_OTA_IDLE_TIMEOUT_MINS > 0
  ota_board = this;
  ota_deadline_millis = millis() + WIFI_OTA_IDLE_TIMEOUT_MINS * 60UL * 1000UL;
  if (ota_idle_timer == NULL) {
    const esp_timer_create_args_t args = {
      .callback = &ota_idle_check, .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK, .name = "ota_idle", .skip_unhandled_events = true
    };
    esp_timer_create(&args, &ota_idle_timer);
  }
  esp_timer_stop(ota_idle_timer);   // no-op if not running
  esp_timer_start_periodic(ota_idle_timer, 60UL * 1000UL * 1000UL);   // check every 60s
#endif

  return true;
}

void ESP32Board::endOTAUpdate() {
#if WIFI_OTA_IDLE_TIMEOUT_MINS > 0
  if (ota_idle_timer) esp_timer_stop(ota_idle_timer);
#endif
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  inhibit_sleep = false;
  MESH_DEBUG_PRINTLN("OTA mode ended (idle timeout)");
}

#else
bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  return false; // not supported
}
void ESP32Board::endOTAUpdate() {
  // not supported
}
#endif

#endif
