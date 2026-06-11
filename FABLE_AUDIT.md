# FABLE_AUDIT — MuziWorks Duo Custom Firmware Deep Audit

**Date:** 2026-06-10
**Scope:** All custom code in baseline commit `37103e32` ("Add Muninn MuziWorks Duo baseline") vs upstream parent `07a3ca9e` — ~7,300 lines across 39 files.
**Dimensions:** reliability, performance, clean code & structure, documentation, power consumption, bug fixes.
**Method:** six parallel audit passes (UI correctness; UI perf/power/structure; mesh/app layer; sensors/IMU; board/target/power; helpers/drivers/build), each reading the full files plus the `07a3ca9e..37103e32` diff, cross-checked against upstream conventions, the SparkFun ICM-20948 driver, the CustomLFS chip table, and the SH1107/SSD1306 driver family.

**Known open items tracked elsewhere (not re-reported):** GPS slide-switch truth table pending hardware detent readings (runtime OFF fail-safed); splash asset licensing; QSPI flash capacity (8 vs 16 MB).

Ownership tags: **APPDEV** = app/UI/lifecycle/config (me), **SYSENG** = driver/radio/register/hardware-truth, **MANAGER** = product decision.

---

## Executive summary

The baseline is in good shape structurally — calibration persistence, wrap-safe timer idioms (mostly), buffer sizing in the UI renderers, the switch debounce/grace logic, battery math, and the discovery protocol reuse of upstream's 0x80/0x90 control types all checked out clean. The audit found **no memory-corruption-class bugs**.

The serious findings cluster around **power** (the product's key constraint) and **one hard functional break**:

1. **USB companion builds are broken** by the CLI-rescue auto-enter on `Serial.available()` (F-01).
2. **"Hibernate" can leave the GPS receiver powered** in SYSTEMOFF — a dead battery in ~a day (F-02).
3. **Smart GPS can run the receiver forever** when no fix is obtainable (indoors/garage) because the no-fix timeout is compile-time dead (F-03).
4. **The IMU runs in full continuous 9-axis mode (~1.3 mA) 24/7** with no duty-cycling (F-04, SYSENG).
5. **The compass page spends ~40–50% of loop wall-time in blocking I2C flushes** at 10 Hz (F-05).

---

## Findings

### Critical / High

#### F-01 [HIGH][bug][APPDEV] `MyMesh.cpp:2492` — CLI-rescue auto-enter on any USB byte breaks USB companion builds
`loop()` does `if (!_cli_rescue && Serial.available()) enterCLIRescue();` unconditionally. On nRF52 **USB** companion builds, `serial_interface.begin(Serial)` wraps the *same* `Serial` stream — the first frame the phone app sends lands in `Serial.available()` before `checkSerialInterface()` runs, the device enters CLI rescue, and the companion protocol is never serviced again (CLI rescue has no exit command other than `reboot`, after which it recurs on the next app frame). On BLE builds, any stray USB byte (Linux ModemManager probing the CDC port, a terminal opened by accident) halts BLE frame servicing until power-cycle.
**Fix:** never auto-enter on builds where the companion interface is the USB serial; on BLE builds keep the rescue path (it's the only headless rescue) but it remains byte-triggered by design — documented residual risk.

#### F-02 [HIGH][power][APPDEV] `MuziWorksDuoBoard.cpp:18-32` — Hibernate enters SYSTEMOFF with GPS_EN (and buzzer/switch pulls) still asserted
`initiateShutdown()` only de-asserts the screen rail before `enterSystemOff()`. The user shutdown path (`UITask::shutdown` → `_board->powerOff()`) never stops GPS; nRF52840 retains GPIO output state in System OFF, so if Smart GPS had the receiver on (switch at ON-GPS after a trip — the typical hibernate moment), the GNSS module (~25–40 mA) runs while the device "is off". Also unhandled: `PIN_ACTIVE_BUZZER` retained HIGH if shutdown lands mid-beep; Serial1 TX idling HIGH into the unpowered GPS (possible back-powering — SYSENG to confirm topology); switch `INPUT_PULLUP`s leak ~250 µA at detents that hold a pin low (F-06).
**Fix:** in `initiateShutdown()`, unconditionally force the GPS rail off, drive the buzzer pin low, release switch/button pulls to no-pull inputs, park Serial1, then enter SYSTEMOFF. The board hook covers every shutdown path.

#### F-03 [HIGH][power][APPDEV] `target.cpp:474-479` — Smart GPS no-fix timeout is compile-time dead; receiver can run until the battery dies
`may_stop_without_fix = !MUZIWORKS_SMART_GPS_REQUIRE_PERSISTED_FIX_BEFORE_STOP && no_fix_timeout;` — all super-IO envs set `REQUIRE_PERSISTED_FIX_BEFORE_STOP=1`, so the advertised 90 s `NO_FIX_TIMEOUT_MS` can never fire. Walk indoors → motion arms GPS → no fix ever → `SMART_GPS_ACQUIRING` forever.
**Fix:** add a hard no-fix stop (generous, e.g. several × the timeout) that applies even when a persisted fix is required, then re-arm on the next motion episode. Telemetry already falls back to the persisted location.

#### F-04 [HIGH][power][SYSENG] `ICM20948Compass.cpp:318-338` — IMU left in full continuous 9-axis mode; no ODR config, duty-cycling, or sleep path
`beginIMU()` = `begin()` + `sleep(false)` + `lowPower(false)`: accel+gyro continuous with DLPF off, magnetometer at 100 Hz — ~1.3–1.4 mA continuous (gyro ~1.23 mA dominates) while the firmware only samples at 10 Hz. No sleep/suspend API exists for board power policy.
**Fix (SYSENG):** sample-rate dividers to 10–20 Hz, cycled/duty-cycled accel mode, mag at 10/20 Hz; consider gyro-off + accel wake-on-motion when stationary (gyro is only used by the motion classifier). Expose a `sleep()` hook.

#### F-05 [HIGH][performance/power][APPDEV] `UITask.cpp:164,2319` — Compass page: 10 Hz full-frame blocking I2C flush
Compass page returns `UI_COMPASS_REFRESH_MS` (100 ms); every refresh pushes the full 2 KB framebuffer over 400 kHz I2C (~46–55 ms *blocking* per flush) — ~40–50% of main-loop wall time while the page is up (and `AUTO_OFF_MILLIS` is 60 s here, not upstream's 15 s). Delays mesh/BLE servicing by up to ~50 ms per pass.
**Fix:** track last-drawn integer heading/pitch/roll (deadband already exists); when the rounded values are unchanged, skip startFrame/flush entirely. Optionally relax cadence to 150 ms.

### Medium

#### F-06 [MED][power][APPDEV] `target.cpp:642-649` — switch `INPUT_PULLUP`s retained into SYSTEMOFF leak ~250 µA per asserted pin
Pull config survives System OFF; at ON detent P0.12 is held low (VDD/13 kΩ ≈ 250 µA) the whole time the device is "off". Fixed together with F-02.

#### F-07 [MED][reliability][SYSENG] `MuziWorksDuoBoard.cpp:19-24` — no GPIO wake source configured for user shutdown
Only LOW_VOLTAGE/BOOT_PROTECT arm LPCOMP; `SHUTDOWN_REASON_USER` configures no GPIO SENSE. If the slide switch does not physically cycle the 3V3 rail, hibernate in the field = brick until USB. **SYSENG:** confirm the power-path hardware; if needed, configure SENSE on a button/switch pin before SYSTEMOFF.

#### F-08 [MED][bug][APPDEV] `platformio.ini:202` (variant) — Uno BLE env sets `QSPIFLASH=1` but W25Q32JVSS is missing from the CustomLFS chip table
JEDEC probe (EF 40 16) fails → QSPI FS won't mount on Uno BLE builds. `patch_custom_lfs_qspi.py` only injects the Duo's W25Q128JVPQ.
**Fix:** have the patch script inject the W25Q32JVSS entry too (or drop the flag until hardware-verified).

#### F-09 [MED][reliability][APPDEV] `patch_custom_lfs_qspi.py:46-70` — patch can land *after* compilation; marker miss hard-fails every nRF52 env
The `AddPreAction($PROGNAME.elf)` hook runs after objects are compiled — on a clean first build the library can compile unpatched while the log claims success. And `env.Exit(1)` on a marker miss (exact-indentation string match) breaks *all* nRF52 envs (RAK, T1000E…) on a routine CustomLFS update, since the script is registered under `nrf52_base`.
**Fix:** patch at import time only; make the pre-ELF hook a verifier; warn-and-skip instead of exiting for non-Muzi envs.

#### F-10 [MED][reliability][APPDEV] `target.cpp:428-437` — IMU init failure silently disables GPS with no override
If `compass_imu.begin()` fails, Smart GPS never starts the receiver and the `gps=1` setting is intercepted — a single sensor fault kills GPS even at the ON-GPS detent.
**Fix:** when switch = ON-GPS but IMU is unavailable, fall back to plain always-on GPS (the detent is explicit power consent).

#### F-11 [MED][reliability][APPDEV] `EnvironmentSensorManager.cpp:922-924` — blocking 1 s `delay(GPS_START_DELAY_MS)` in the main loop on every GPS start
Every motion-triggered GPS power-up freezes radio/BLE/UI for 1 s, many times a day. The delay also runs *before* `_location->begin()`, so it doesn't even serve as post-power-on settling.
**Fix:** non-blocking deferred start (or at minimum move/shrink the delay). Placement question (pre vs post power-on) → SYSENG.

#### F-12 [MED][bug][APPDEV] `ICM20948Compass.cpp:628-634` — failed re-calibration destroys a previously good calibration until reboot
The failure branch clears `mag_cal_valid`/offsets even when a good calibration was loaded; the cancel path carefully restores state, the failure path doesn't.
**Fix:** snapshot at `startCalibration()` and restore on failure (mirror `cancelCalibration()`), or re-run `loadCompassCalibration()`.

#### F-13 [MED][bug][APPDEV] `ICM20948Compass.cpp:1040` + `target.cpp:223-226` — telemetry can broadcast a bogus 0° heading when uncalibrated
Driver zeroes `heading_deg` when invalid; the telemetry consumer gates only on `reading.flat`, not `heading_validity` — uncalibrated flat units broadcast "due north".
**Fix:** gate `addDirection` on `COMPASS_HEADING_VALID`.

#### F-14 [MED][bug][MANAGER] `EnvironmentSensorManager.cpp:682-684` — location now sent in telemetry even with GPS disabled
Gate changed from `gps_active` to "non-zero coords" (intentional, commented). Consequence: once a location is ever known (incl. persisted-at-boot), turning GPS off no longer stops location disclosure to any requester with `TELEM_PERM_LOCATION`. Affects **all** boards using this shared manager, and diverges from upstream.
**Decision needed:** keep (then gate behind an explicit "share last-known location" pref or a MuziWorks build flag) or revert.

#### F-15 [MED][bug][APPDEV] `UITask.cpp:2458,2517-2527` — `quick_stage` leaks across page navigation; quick-send actions menu silently retargets
`KEY_RIGHT` falls through to generic page-change while `quick_stage == QUICK_ACTIONS`; the menu is keyed on `_page`, so ENTER can send a preset to a recipient the user never chose.
**Fix:** reset `quick_stage = QUICK_TARGETS` on any page change.

#### F-16 [MED][reliability][APPDEV] `UITask.cpp:887,1101,1173` — quick-send target resolved by *ordinal* at send time; concurrent contact-list changes retarget the message
The phone app can add/remove/unfavorite contacts over BLE while the user composes; ordinal N then resolves to a different contact and the DM goes to the wrong person.
**Fix:** capture the target identity (pubkey / channel idx+name) when leaving `QUICK_TARGETS`; re-resolve by pubkey at send; abort if it no longer matches.

#### F-17 [MED][performance][APPDEV] `UITask.cpp:584-618,1036-1097` — GROUP quick-send page: O(N²) contact rescans + advert-table qsort every 500 ms frame
Each frame: full count scan, per-row rescans from ordinal 0 (~500+ `ContactInfo` copies ≈ 100 KB memcpy per frame at 120 favorites), `getRecentlyHeard` qsort per frame, and a per-row compass struct copy.
**Fix:** per-frame ordinal→index array; fetch each row's contact once; hoist `getRecentlyHeard` behind a 2–5 s timer and the compass read out of the row loop (the directional page's `directional_dirty` pattern already does this right).

#### F-18 [MED][reliability/performance][APPDEV] `SH1107Display.cpp:67-76` — `turnOn()` blocks the whole loop for 250 ms although the rail is never cut
`turnOff()` deliberately keeps the panel rail asserted (documented), yet `turnOn()` still runs `delay(SCREEN_ENABLE_SETTLE_MS)` (250 ms) on every wake — every button-press wake stalls mesh/BLE servicing for a quarter second for no reason.
**Fix:** track rail state; only settle-delay when the rail was actually off (i.e., first power-on).

#### F-19 [MED][structure][APPDEV] `variants/muziworks_duo/platformio.ini:403-490` — primary Duo BLE env duplicates ~80 lines of the USB env instead of `extends`
Identical today (verified), so drift *risk*: 40+ tuning defines must be edited twice. The Uno pair already uses `extends` correctly.
**Fix:** `extends = env:muziworks_duo_super_io_companion_radio_usb` + BLE-only additions.

#### F-20 [MED][structure][APPDEV] `UITask.cpp` — HomeScreen `render()`/`handleInput()` grew to ~320/~175 lines with inconsistent extraction; four divergent "time ago" formatters
COMPASS page is a ~90-line inline block while GPS/Profile/Diagnostics got helpers; quick-send/directional input is inlined. Separately, four age formatters disagree (`formatAge` is dead code *and* has a `mins < 24` threshold bug — 30-minute ages would render "1h").
**Fix:** extract `renderCompassPage()` (+ input helpers opportunistically); one `formatAgeSecs`-based formatter for all call sites; delete `formatAge`.

### Low

#### F-21 [LOW][bug][APPDEV] `UITask.cpp:1953-1964,2011` — `renderGpsPage` reads uninitialized `GPSStatus` when `getGPSStatus()` fails → `gs = {}` init.
#### F-22 [LOW][bug][APPDEV] `UITask.cpp:1025` — `getQuickChannelTarget()` return ignored; formats uninitialized struct on failure (contact path does it right) → `continue` on failure.
#### F-23 [LOW][bug][APPDEV] `UITask.cpp:960-965` — keyboard cursor `_` disappears once message exceeds 23 chars (guard tests total, not visible, length) → guard on visible length.
#### F-24 [LOW][reliability][APPDEV] `UITask.cpp:2891-2898` — incoming message yanks user out of the quick-send keyboard; next keystroke lands in MsgPreview and can dismiss the unread queue → skip screen-switch while modal typing.
#### F-25 [LOW][reliability][APPDEV] `UITask.cpp:3018-3019,3192-3198` — long-press actions (Diagnostics, 30 s compass cal, repeater ping TX) fire while the display is off → wake-and-consume like `checkDisplayOn()`.
#### F-26 [LOW][reliability][APPDEV] `UITask.cpp:2913-2937`, `UITask.h:58-60` — LED heartbeat timers are signed `int` millis snapshots; erratic toggling near 2^31 ms (~24.8 days) → `unsigned long` + wrap-safe idiom.
#### F-27 [LOW][performance][APPDEV] `UITask.cpp:2914-2923` — `userLedHandler` walks the sensor settings table (strcmp scan) every loop iteration; answer only needed 1×/s → move inside the timer gate.
#### F-28 [LOW][structure][APPDEV] `UITask.cpp:1430-1472` — battery OCV `#ifndef` defaults defined inside the function body → move to the top-of-file config block.
#### F-29 [LOW][structure][APPDEV] `UITask.cpp:3291-3298` — dead `activeBuzzerPlay()`; `variants/muziworks_duo/platformio.ini` — dead `MUZIWORKS_SUPER_IO_FORCE_BUZZER_ON` flag (3 envs) → delete both.
#### F-30 [LOW][structure][APPDEV] `UITask.cpp` — triplicated truncate-copy loops and quadruplicated scroll-window math across the four list screens → shared helpers (fold into F-20 work, lowest priority).
#### F-31 [LOW][bug][APPDEV] `variant.h:141` — `PIN_GPS_SWITCH` aliases only half the 3-position switch; shared code keying on it (ui-tiny) would misread → remove or comment as not-a-boolean.
#### F-32 [LOW][power][SYSENG] `variant.cpp:65` + `MuziWorksDuoBoard.cpp:38` — `pinMode(BATTERY_PIN, INPUT)` leaves the digital input buffer connected on a mid-rail divider (µA shoot-through, even in SYSTEMOFF); SAADC doesn't need it → drop both calls (SYSENG confirm).
#### F-33 [LOW][reliability][APPDEV] `target.cpp:255-264` — `gps=1` setting returns success but does nothing under Smart GPS → return false or expose "smart" status (client-facing contract).
#### F-34 [LOW][reliability][SYSENG] `ICM20948Compass.cpp:958-961` — no recovery if the IMU stops responding post-init; Smart GPS arming dies until reboot → failure counter + re-`beginIMU()`.
#### F-35 [LOW][bug][APPDEV] `ICM20948Compass.cpp:951-952` — update throttle uses raw `now < next_imu_update` (not wrap-safe; file uses signed-diff idiom everywhere else) → one-line idiom fix.
#### F-36 [LOW][bug][SYSENG] `ICM20948Compass.cpp:786-791,1030,1046-1047` — axis-convention items to verify on hardware: published N/E basis vectors are rotated by the render offset (not true ENU); pitch/roll convention may be swapped vs display orientation. Heading itself self-consistent.
#### F-37 [LOW][reliability][SYSENG] `ICM20948Compass.cpp:528-554` — persisted calibration record doesn't encode axis-sign/rotation config; future frame changes silently misapply old cals → pack frame config into version/reserved.
#### F-38 [LOW][structure][APPDEV] `ICM20948Compass.h:140-142` — Smart-GPS UI state passed through `CompassReading` (driver as mailbox); `publishCompassStatus()` runs 3× per update → move to `GPSStatus`, dedupe (defer: API churn).
#### F-39 [LOW][bug][APPDEV] `LPPDataHelpers.h:139-153` — new readers read up to 6 bytes before the bounds check (matches pre-existing upstream pattern; benign over-read on nRF52 but UB) → check `_pos + n <= _len` first in our three new readers.
#### F-40 [LOW][reliability][APPDEV] `MyMesh.cpp:2417-2447` — `maybePersistLatestLocation()` does a blocking prefs flash write every ≥120 s while moving (~720 writes/day worst case). LittleFS wear-leveling makes this acceptable but it's worth a comment + telemetry watch; also `shouldPersistLocationNow()` bypasses the interval entirely.
#### F-41 [LOW][docs][APPDEV] `git_hash_build_flag.py:8-21` — git runs in process CWD (wrong-repo hash under `pio run -d`); untracked files don't mark dirty → pass `cwd=$PROJECT_DIR`.
#### F-42 [LOW][docs][APPDEV] `UITask.h:95` — `applyGPSSwitchAction()` "returns true = shutdown initiated, caller must abort" contract undocumented at declaration/call sites; `begin()` early-return leaves screens unconstructed → contract comments.
#### F-43 [LOW][docs][APPDEV] No standalone compass design doc in `docs/` (commit message implied one); the .cpp header comment is authoritative and verified accurate, but the cal file format (`/muzi_compass_cal`, MCAL v1) and heading-validity enum that client apps need aren't documented anywhere external → extract or declare in-code doc authoritative.
#### F-44 [LOW][structure][MANAGER] `boards/muzi_base_duo.json:10-16` — Duo and Uno ship Adafruit's VID/PID set identically; upload autodetection can't distinguish units; USB-IF etiquette for production → product decision.

### Verified clean (highlights)
Switch decode + debounce/grace wrap-safety; battery ADC math & LPCOMP threshold; persisted-location validity/wrap handling; splash bitmap bounds; DST math; haversine/bearing math; quick-keyboard navigation bounds; all checked `snprintf` sizings; calibration persistence (magic/version/remove-before-write); discovery protocol consistent with upstream 0x80/0x90 responders (simple_repeater, simple_sensor); NodePrefs append-only migration safe (old files load, default+constrain covers the new byte); `cli_command` 160-byte sizing fits `prv.key` restore; CustomLR1121/wrapper consistent with the LR1110 pattern; SH1107 `drawXbm` convention identical to upstream SSD1306; AutoDiscoverRTC changes (RV3028 weekday fix is a genuine improvement).

---

## Implementation plan

**Phase A — power & hard breaks (this session):**
- [ ] A1 F-01 CLI-rescue gate for USB companion builds
- [ ] A2 F-02+F-06 `initiateShutdown()` rail/pull/buzzer/serial cleanup
- [ ] A3 F-03 Smart GPS hard no-fix stop + motion re-arm
- [ ] A4 F-05 compass page skip-flush deadband

**Phase B — medium bugs (this session):**
- [ ] B1 F-08+F-09 QSPI patch script: Uno chip entry, import-time patch + verifier, no fleet-wide Exit
- [ ] B2 F-10 IMU-failure → plain GPS fallback at ON-GPS
- [ ] B3 F-12 calibration-failure state restore
- [ ] B4 F-13 telemetry heading validity gate
- [ ] B5 F-15/F-16 quick-send stage reset + pubkey-pinned target
- [ ] B6 F-18 SH1107 turnOn settle skip
- [ ] B7 F-19 platformio BLE `extends` + F-29 dead flag/code removal
- [ ] B8 F-21/F-22/F-23/F-24/F-25/F-26/F-27/F-28 UI small fixes
- [ ] B9 F-20 age-formatter consolidation (+ delete `formatAge`)
- [ ] B10 F-33/F-35/F-39/F-41/F-42 small contract/idiom/docs fixes
- [ ] B11 F-11 non-blocking GPS start (needs care; placement Q to SYSENG)

**Phase C — routed to SYSENG (Q posted in `_chat`):** F-04 IMU power config; F-07 wake source after SYSTEMOFF; F-32 battery pin buffer; F-34 IMU I2C recovery; F-36 axis verification; F-37 cal-record frame config; Serial1 park / back-powering confirm (part of F-02).

**Decisions for MANAGER:** F-14 location-telemetry-with-GPS-off policy; F-44 USB VID/PID; (existing: splash licensing, QSPI capacity).

**Deferred (tracked, not this session):** F-17 full ordinal-cache refactor (light version only: hoist compass read + recently-heard timer); F-30 list-screen helper consolidation; F-38 CompassReading API cleanup; F-40 wear-comment only; F-43 docs extraction.
