#pragma once

// WiFi-client OTA: join an existing AP (triggered remotely over the mesh
// admin CLI) and pull a firmware image from an HTTP(S) URL into the inactive
// OTA slot. Counterpart to the ESP32Board softAP/ElegantOTA flow — this one
// never opens an AP and works while the node keeps repeating.
//
// CLI surface (see CommonCLI):
//   wifi scan                        async AP scan; call again for results
//   start wifi-ota <ssid>|<pass>|<url>
//   ota status                       phase, progress %, last error
//   ota abort                        cancel + WiFi off
//
// The state machine is driven from the main loop (WiFiOTA::loop()) in small
// chunks so mesh forwarding keeps running and the task watchdog stays fed.

#if defined(ESP32) && defined(ENABLE_WIFI_OTA)

class WiFiOTA {
public:
  // All return a human-readable reply into `reply` (CLI-sized, keep < 150 chars).
  static void scan(char* reply);
  static void begin(char* args, char* reply);   // args = "<ssid>|<pass>|<url>", parsed in place
  static void status(char* reply);
  static void abort(char* reply);

  // True while any WiFi/flash work (or a queued reboot, or an idle scan radio)
  // is in flight — the main loop must suppress light sleep for the whole
  // session, or sleeping would drop the STA link mid-download.
  static bool isActive();

  static void loop();
};

#endif
