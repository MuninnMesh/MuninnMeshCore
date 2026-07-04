#if defined(ESP32) && defined(ENABLE_WIFI_OTA)

#include "ESP32WiFiOTA.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <SPIFFS.h>
#include <MeshCore.h>
#include <helpers/TxtDataHelpers.h>

// A successful OTA reboots the node, which wipes the in-RAM phase — so
// `ota status` could only ever read "idle" afterward. Persist the last outcome
// to SPIFFS so it survives the reboot and `ota status` can report it.
#define OTA_RESULT_FILE  "/ota_result"

static void save_ota_result(const char* result) {
  File f = SPIFFS.open(OTA_RESULT_FILE, "w", true);
  if (f) { f.print(result); f.close(); }
}

#ifndef WIFI_OTA_CONNECT_TIMEOUT_MS
  #define WIFI_OTA_CONNECT_TIMEOUT_MS   20000UL
#endif
#ifndef WIFI_OTA_SESSION_TIMEOUT_MS
  #define WIFI_OTA_SESSION_TIMEOUT_MS   (10UL * 60UL * 1000UL)
#endif
// Reboot delay after a successful flash: long enough for the CLI reply (and
// its LoRa retransmits) to leave the node before it restarts.
#ifndef WIFI_OTA_REBOOT_DELAY_MS
  #define WIFI_OTA_REBOOT_DELAY_MS      8000UL
#endif
// An idle async scan (operator ran `wifi scan` then never polled back) must not
// leave the STA radio powered on a solar node — tear it down after this long.
#ifndef WIFI_OTA_SCAN_IDLE_TIMEOUT_MS
  #define WIFI_OTA_SCAN_IDLE_TIMEOUT_MS (60UL * 1000UL)
#endif

enum OTAPhase : uint8_t {
  OTA_IDLE = 0,
  OTA_WIFI_CONNECTING,
  OTA_HTTP_STARTING,
  OTA_DOWNLOADING,
  OTA_SUCCESS_PENDING_REBOOT,
  OTA_FAILED,
};

static OTAPhase phase = OTA_IDLE;
static char ota_ssid[33];
static char ota_pass[65];
static char ota_url[152];
static char last_err[48] = "none";
static unsigned long connect_deadline, session_deadline, reboot_at, scan_idle_deadline;
static int total_len = 0, written_len = 0;
static bool scan_pending = false;      // an async scan's radio is up, awaiting readout
static bool we_own_update = false;     // this module (not ElegantOTA) started the flash

static WiFiClient* client = NULL;      // plain or TLS; owned here, freed in cleanup()
static HTTPClient* http = NULL;

static bool session_active() {
  return phase == OTA_WIFI_CONNECTING || phase == OTA_HTTP_STARTING || phase == OTA_DOWNLOADING;
}

bool WiFiOTA::isActive() {
  // True whenever WiFi/flash work is in flight or a reboot is queued, so the
  // main loop can suppress light sleep (which would drop the STA link) for the
  // whole session — the softAP OTA uses inhibit_sleep for the same reason.
  return session_active() || phase == OTA_SUCCESS_PENDING_REBOOT || scan_pending;
}

// The ElegantOTA softAP flow (ESP32Board::startOTAUpdate) owns WiFi in AP mode
// and drives the same global Update object. Refuse to stomp on it.
static bool softap_ota_active() {
  return (WiFi.getMode() & WIFI_MODE_AP) != 0;
}

static void cleanup(bool wifi_off) {
  if (http) { http->end(); delete http; http = NULL; }
  if (client) { delete client; client = NULL; }
  if (we_own_update && Update.isRunning()) Update.abort();
  we_own_update = false;
  if (wifi_off) {
    WiFi.scanDelete();
    scan_pending = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  memset(ota_pass, 0, sizeof(ota_pass));   // credentials are session-only
#ifdef ESP32_CPU_FREQ
  setCpuFrequencyMhz(ESP32_CPU_FREQ);      // restore the configured clock
#endif
}

static void fail(const char* why) {
  StrHelper::strncpy(last_err, why, sizeof(last_err));
  MESH_DEBUG_PRINTLN("WiFiOTA: failed: %s", why);
  char rec[64];
  snprintf(rec, sizeof(rec), "FAIL: %s (at %d/%d)", why, written_len, total_len);
  save_ota_result(rec);
  cleanup(true);
  phase = OTA_FAILED;
}

void WiFiOTA::scan(char* reply) {
  if (session_active() || phase == OTA_SUCCESS_PENDING_REBOOT) {
    strcpy(reply, "ERR: ota session active");
    return;
  }
  if (softap_ota_active()) {
    strcpy(reply, "ERR: softAP OTA active");
    return;
  }
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    strcpy(reply, "scanning, retry in a few secs");
    return;
  }
  if (n < 0) {   // not started (or failed) -> kick off an async scan
    WiFi.scanNetworks(true);
    scan_pending = true;
    scan_idle_deadline = millis() + WIFI_OTA_SCAN_IDLE_TIMEOUT_MS;
    strcpy(reply, "scan started, repeat cmd for results");
    return;
  }

  // Scan finished. Sort by RSSI (strongest first) via an index table, then emit
  // as many "<name5>:<rssi>" entries as fit a conservative reply budget. Names
  // are trimmed to 5 chars — the operator only needs enough to recognize their
  // AP, and untrimmed attacker-broadcast SSIDs could otherwise overflow the CLI
  // reply buffer (160 bytes at the smaller call site).
  const int REPLY_CAP = 160;      // smaller of the two call-site buffers (serial)
  const int SUFFIX_MAX = 14;      // " (+NNN more)" + NUL
  const int REPLY_BUDGET = REPLY_CAP - SUFFIX_MAX;   // entries must leave room for the suffix
  int order[24];
  int m = (n < 24) ? n : 24;
  for (int i = 0; i < m; i++) order[i] = i;
  for (int i = 0; i < m - 1; i++) {   // selection sort, m<=24 so O(m^2) is fine
    int best = i;
    for (int j = i + 1; j < m; j++) {
      if (WiFi.RSSI(order[j]) > WiFi.RSSI(order[best])) best = j;
    }
    if (best != i) { int t = order[i]; order[i] = order[best]; order[best] = t; }
  }

  int pos = 0;
  int shown = 0;
  for (int i = 0; i < m; i++) {
    char name[6];
    StrHelper::strncpy(name, WiFi.SSID(order[i]).c_str(), sizeof(name));   // <=5 chars + NUL
    char entry[24];
    int len = snprintf(entry, sizeof(entry), "%s%s:%d", (shown ? "\n" : ""), name, (int)WiFi.RSSI(order[i]));
    if (pos + len >= REPLY_BUDGET) break;   // stop before overflowing the reply
    memcpy(&reply[pos], entry, len);
    pos += len;
    shown++;
  }
  reply[pos] = 0;
  if (shown == 0) strcpy(reply, "no APs found");
  else if (shown < n) snprintf(&reply[pos], REPLY_CAP - pos, "\n(+%d more)", n - shown);

  WiFi.scanDelete();
  scan_pending = false;
  WiFi.mode(WIFI_OFF);   // scan is standalone; a session starts its own STA
}

void WiFiOTA::begin(char* args, char* reply) {
  if (session_active()) {
    strcpy(reply, "ERR: ota already running");
    return;
  }
  if (phase == OTA_SUCCESS_PENDING_REBOOT) {
    strcpy(reply, "ERR: rebooting into new fw");
    return;
  }
  if (softap_ota_active()) {
    strcpy(reply, "ERR: softAP OTA active");
    return;
  }

  // parse "<ssid>|<pass>|<url>" in place ('|' is not valid in URLs and is
  // vanishingly rare in SSIDs; an open AP is "<ssid>||<url>")
  char* sep1 = strchr(args, '|');
  char* sep2 = sep1 ? strchr(sep1 + 1, '|') : NULL;
  if (!sep1 || !sep2) {
    strcpy(reply, "ERR: use <ssid>|<pass>|<url>");
    return;
  }
  *sep1 = 0; *sep2 = 0;
  const char* url = sep2 + 1;
  if (memcmp(url, "http://", 7) != 0 && memcmp(url, "https://", 8) != 0) {
    strcpy(reply, "ERR: url must be http(s)://");
    return;
  }
  // The mesh CLI transport caps the whole command near ~163 bytes, so a URL
  // that fills ota_url exactly was almost certainly truncated upstream — say so
  // instead of later failing with a generic 'bad url'.
  if (strlen(url) >= sizeof(ota_url) - 1) {
    strcpy(reply, "ERR: url too long / truncated");
    return;
  }
  StrHelper::strncpy(ota_ssid, args, sizeof(ota_ssid));
  StrHelper::strncpy(ota_pass, sep1 + 1, sizeof(ota_pass));
  StrHelper::strncpy(ota_url, url, sizeof(ota_url));

  strcpy(last_err, "none");
  total_len = written_len = 0;

#ifdef ESP32_CPU_FREQ
  setCpuFrequencyMhz(160);   // full clock for TLS + download; restored in cleanup()
#endif

  WiFi.scanDelete();         // drop any leftover async-scan state before connecting
  scan_pending = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ota_ssid, ota_pass[0] ? ota_pass : NULL);

  connect_deadline = millis() + WIFI_OTA_CONNECT_TIMEOUT_MS;
  session_deadline = millis() + WIFI_OTA_SESSION_TIMEOUT_MS;
  phase = OTA_WIFI_CONNECTING;
  sprintf(reply, "connecting to %s, poll: ota status", ota_ssid);
}

void WiFiOTA::status(char* reply) {
  switch (phase) {
    case OTA_IDLE: {
      // Surface the persisted outcome of the previous session — after a
      // successful OTA the node has rebooted, so this is the only way to see
      // that it worked (vs "never ran").
      char last[80] = "";
      File f = SPIFFS.open(OTA_RESULT_FILE, "r");
      if (f) { int n = f.read((uint8_t*)last, sizeof(last) - 1); if (n > 0) last[n] = 0; f.close(); }
      if (scan_pending) strcpy(reply, "idle (scan running)");
      else if (last[0]) snprintf(reply, 150, "idle | last: %s", last);
      else strcpy(reply, "idle | last: (none)");
      break;
    }
    case OTA_WIFI_CONNECTING: sprintf(reply, "wifi connecting (%.32s)", ota_ssid); break;
    case OTA_HTTP_STARTING:   strcpy(reply, "starting download"); break;
    case OTA_DOWNLOADING:
      sprintf(reply, "downloading %d/%d (%d%%)", written_len, total_len,
              total_len > 0 ? (int)(100LL * written_len / total_len) : 0);
      break;
    case OTA_SUCCESS_PENDING_REBOOT: strcpy(reply, "flashed OK, rebooting"); break;
    case OTA_FAILED:    sprintf(reply, "failed: %.40s", last_err); break;
  }
}

void WiFiOTA::abort(char* reply) {
  if (phase == OTA_SUCCESS_PENDING_REBOOT) {
    strcpy(reply, "ERR: already flashed, rebooting");
    return;
  }
  if (!session_active() && !scan_pending) {
    strcpy(reply, "nothing to abort");
    return;
  }
  cleanup(true);
  phase = OTA_IDLE;
  strcpy(reply, "OK - ota aborted");
}

void WiFiOTA::loop() {
  // Tear down an abandoned async scan so the STA radio doesn't stay powered on
  // a solar node after `wifi scan` with no follow-up poll. A scan can be kicked
  // in OTA_IDLE or OTA_FAILED, so this must run before the FAILED early-return.
  if (scan_pending && (long)(millis() - scan_idle_deadline) >= 0) {
    WiFi.scanDelete();
    WiFi.mode(WIFI_OFF);
    scan_pending = false;
  }
  if (phase == OTA_IDLE || phase == OTA_FAILED) return;

  if (phase == OTA_SUCCESS_PENDING_REBOOT) {
    if ((long)(millis() - reboot_at) >= 0) {
      ESP.restart();
    }
    return;
  }

  if ((long)(millis() - session_deadline) >= 0) {
    fail("session timeout");
    return;
  }

  if (phase == OTA_WIFI_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      MESH_DEBUG_PRINTLN("WiFiOTA: connected, ip=%s rssi=%d", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
      phase = OTA_HTTP_STARTING;
    } else if ((long)(millis() - connect_deadline) >= 0) {
      fail("wifi connect timeout");
    }
    return;
  }

  if (phase == OTA_HTTP_STARTING) {
    http = new HTTPClient();
    // Short timeouts: this GET runs synchronously in one loop() pass, so the
    // mesh stops forwarding until headers arrive. Keep that window small; a
    // dead host fails fast rather than freezing the repeater for tens of secs.
    http->setConnectTimeout(6000);
    http->setTimeout(8000);
    http->setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // CDN/GitHub 302s
    // Cap redirect hops: the whole GET runs synchronously in one loop() pass,
    // so 10 hops * (6s connect + 8s read) could exceed the 60s task WDT and
    // panic-reboot mid-command. 3 covers real CDN/GitHub chains.
    http->setRedirectLimit(3);
    bool ok;
    if (memcmp(ota_url, "https://", 8) == 0) {
      WiFiClientSecure* tls = new WiFiClientSecure();
      tls->setInsecure();   // no cert validation; the firmware source is trusted
      client = tls;
      ok = http->begin(*client, ota_url);
    } else {
      client = new WiFiClient();
      ok = http->begin(*client, ota_url);
    }
    if (!ok) { fail("bad url"); return; }

    int code = http->GET();
    if (code != HTTP_CODE_OK) { fail("http error"); return; }

    total_len = http->getSize();
    if (total_len <= 0) { fail("no content-length"); return; }
    // sanity: must fit the OTA slot, and be big enough to be a real app image
    if (total_len > (int)(ESP.getFreeSketchSpace()) || total_len < 65536) {
      fail("bad image size");
      return;
    }
    if (!Update.begin(total_len)) { fail("Update.begin (slot?)"); return; }
    we_own_update = true;

    written_len = 0;
    phase = OTA_DOWNLOADING;
    return;
  }

  // OTA_DOWNLOADING: move at most one chunk per loop() pass so LoRa dispatch
  // latency stays bounded and the task WDT keeps getting fed by the main loop.
  static uint8_t buf[2048];
  WiFiClient* stream = http->getStreamPtr();
  int avail = stream->available();
  if (avail > 0) {
    int r = stream->readBytes(buf, min(avail, (int)sizeof(buf)));
    if (r > 0) {
      // Update.write() validates the ESP image magic (0xE9) on the first
      // chunk — a wrong file (nRF zip, HTML error page) fails right here.
      if (Update.write(buf, r) != (size_t)r) { fail("flash write"); return; }
      written_len += r;
    }
  }
  if (written_len >= total_len) {
    if (!Update.end()) { fail("image verify"); return; }
    MESH_DEBUG_PRINTLN("WiFiOTA: flashed %d bytes OK", written_len);
    char rec[64];
    snprintf(rec, sizeof(rec), "OK: flashed %d bytes, rebooted", written_len);
    save_ota_result(rec);    // persisted so `ota status` shows it after the reboot
    we_own_update = false;   // committed; don't let cleanup() abort it
    cleanup(true);           // WiFi off before reboot; boot partition already switched
    phase = OTA_SUCCESS_PENDING_REBOOT;
    reboot_at = millis() + WIFI_OTA_REBOOT_DELAY_MS;
    return;
  }
  // Only treat a dropped connection as fatal once the rx buffer is also drained
  // — servers that send "Connection: close" FIN right after the last body byte,
  // so connected() can be false while the image tail is still buffered.
  if (avail == 0 && !stream->connected() && written_len < total_len) {
    fail("connection lost");
  }
}

#endif
