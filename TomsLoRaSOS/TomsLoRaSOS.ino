/*
  TomsLoRaSOS
  LilyGO T-LoRa Pager outdoor mapper, LoRa messenger and SOS prototype by Tom
  -------------------------------------
  Permanent offline map + GPS follow + LoRa peer positions/messages + SOS mode
  + IMU driven pocket display mode.

  More projects and notes:
  https://steinlaus.de/

  SPDX-License-Identifier: GPL-3.0-or-later
  Commercial licensing available on request.

  Dependencies:
    - Official LilyGoLib and its Pager dependencies
    - PNGdec
    - vendor_factory files imported by prepare_vendor_files.py

  This is an experimental outdoor prototype, not a certified emergency device.
*/

#include <Arduino.h>
#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <SD.h>
#include <PNGdec.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <esp_ota_ops.h>
#include <vector>
#include <string>

#include "app_config.h"
#include "app_types.h"
#include "src/vendor_factory/hal_interface.h"

using std::string;

// -----------------------------------------------------------------------------
// Vendor-HAL UI hooks required by the official factory abstraction.
// This sketch intentionally replaces the large factory UI with a compact map UI.
// -----------------------------------------------------------------------------

static lv_obj_t *g_toast = nullptr;
static uint32_t g_toast_until_ms = 0;

static void earlyBacklightOn() {
#ifdef DISP_BL
  pinMode(DISP_BL, OUTPUT);
  digitalWrite(DISP_BL, HIGH);
#endif
}

static void printBootDiagnostics(const char *stage) {
  Serial.print("[BOOT] ");
  Serial.println(stage);
  Serial.printf("[BOOT] resetReason=%d heap=%u psram=%u\n",
                (int)esp_reset_reason(),
                (unsigned int)ESP.getFreeHeap(),
                (unsigned int)ESP.getFreePsram());

  const esp_partition_t *running = esp_ota_get_running_partition();
  if (running) {
    Serial.printf("[BOOT] running label=%s addr=0x%08X size=0x%X\n",
                  running->label,
                  (unsigned int)running->address,
                  (unsigned int)running->size);
  }
  Serial.flush();
}

void ui_show_wifi_process_bar() {
  // The compact prototype shows WiFi state in the status line.
}

bool isinMenu() {
  return false;
}

void ui_msg_pop_up(const char *title_txt, const char *msg_txt);

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

struct AppSettings {
  String deviceId = "pager";
  String deviceName = "PAGER";
  String wifiSsid;
  String wifiPassword;
  String tileUrl = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
  double fallbackLat = APP_DEFAULT_FALLBACK_LAT;
  double fallbackLon = APP_DEFAULT_FALLBACK_LON;
  uint8_t zoom = APP_DEFAULT_ZOOM;
  uint32_t positionIntervalMs = APP_POS_INTERVAL_MS;
  uint32_t safetyNoMotionMs = APP_DEFAULT_SAFETY_NO_MOTION_SECONDS * 1000UL;
  uint32_t safetyCountdownMs = APP_DEFAULT_SAFETY_COUNTDOWN_SECONDS * 1000UL;
  bool autoDisplay = true;
};

static AppSettings g_cfg;
static Preferences g_preferences;

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

struct ChatMessage {
  String sender;
  String text;
  bool outgoing = false;
  uint32_t timeMs = 0;
};

enum SafetyState {
  SAFETY_OFF,
  SAFETY_ARMED,
  SAFETY_WARNING,
  SAFETY_SOS
};

static OutdoorPeer g_peers[APP_MAX_PEERS];
static ChatMessage g_messages[APP_MAX_MESSAGES];
static uint8_t g_messageCount = 0;
static MapTrack g_tracks[APP_MAX_TRACKS];
static int8_t g_selectedTrackIndex = -1;
static int8_t g_recordingTrackIndex = -1;
static uint8_t g_nextTrackColor = 0;

static gps_params_t g_gps {};
static gps_params_t g_gpsRaw {};
static bool g_hasGpsInfo = false;
static bool g_hasGps = false;
static imu_params_t g_imu {};
static bool g_hasImu = false;

static SafetyState g_safety = SAFETY_OFF;
static uint32_t g_safetyStateSinceMs = 0;
static uint32_t g_lastMotionMs = 0;
static uint32_t g_lastInputMs = 0;
static uint32_t g_lastSosTxMs = 0;
static uint32_t g_lastPosTxMs = 0;
static uint32_t g_lastGpsRefreshMs = 0;
static uint32_t g_lastImuRefreshMs = 0;
static uint32_t g_lastRadioRefreshMs = 0;
static uint32_t g_lastMapRefreshMs = 0;
static uint32_t g_sequence = 1;
static bool g_followGps = true;
static bool g_displayOn = true;

enum MapPanAxis {
  MAP_PAN_HORIZONTAL,
  MAP_PAN_VERTICAL
};

static MapPanAxis g_mapPanAxis = MAP_PAN_HORIZONTAL;
static bool g_manualMapCenterValid = false;
static double g_manualMapLat = APP_DEFAULT_FALLBACK_LAT;
static double g_manualMapLon = APP_DEFAULT_FALLBACK_LON;
static uint32_t g_lastRotaryCenterPressMs = 0;

static uint8_t g_radioRxBuf[APP_MAX_RADIO_PAYLOAD + 2];
static bool g_lastRadioTxOk = false;
static int g_lastTileHttpCode = 0;
static String g_lastTileState = "idle";
static bool g_sdReady = false;
static bool g_wifiEnabled = false;

static uint32_t g_pngDecodeOk = 0;
static uint32_t g_pngDecodeFail = 0;
static int g_lastPngRc = 0;
static String g_lastPngPath = "-";

static float g_prevRoll = NAN;
static float g_prevPitch = NAN;
static float g_prevHeading = NAN;

static int16_t g_sosBeepBuffer[APP_SOS_AUDIO_BEEP_SAMPLES];
static bool g_sosAudioOpen = false;
static uint32_t g_lastSosAudioActionMs = 0;
static uint8_t g_sosAudioBurstCount = 0;
static SafetyState g_sosAudioObservedState = SAFETY_OFF;
static uint32_t g_remoteSosAlarmUntilMs = 0;
static uint32_t g_lastRemoteSosAlarmMs = 0;
static uint8_t g_remoteSosAlarmCount = 0;

// -----------------------------------------------------------------------------
// LVGL objects
// -----------------------------------------------------------------------------

static lv_obj_t *g_canvas = nullptr;
static lv_obj_t *g_peerLabels[APP_MAX_PEERS];
static lv_obj_t *g_peerSosRings[APP_MAX_PEERS];
static int16_t g_peerRingCenterX[APP_MAX_PEERS];
static int16_t g_peerRingCenterY[APP_MAX_PEERS];
static lv_obj_t *g_status = nullptr;
static lv_obj_t *g_footer = nullptr;
static lv_obj_t *g_messagesPopup = nullptr;
static lv_obj_t *g_messagesList = nullptr;
static lv_obj_t *g_textarea = nullptr;
static lv_obj_t *g_messagesEscTop = nullptr;
static lv_obj_t *g_messagesEscBottom = nullptr;
static lv_obj_t *g_messageListItems[APP_MAX_MESSAGE_LIST_ITEMS];
static int16_t g_messageListItemCount = 0;
static int16_t g_selectedMessageListItem = 0;
static lv_obj_t *g_tracksPopup = nullptr;
static lv_obj_t *g_tracksList = nullptr;
static lv_obj_t *g_trackNameArea = nullptr;
static lv_obj_t *g_tracksEscTop = nullptr;
static lv_obj_t *g_tracksEscBottom = nullptr;
static lv_obj_t *g_trackListItems[APP_MAX_TRACK_LIST_ITEMS];
static int8_t g_trackListItemToTrack[APP_MAX_TRACK_LIST_ITEMS];
static int16_t g_trackListItemCount = 0;
static int16_t g_selectedTrackListItem = 0;
static lv_obj_t *g_safetyPopup = nullptr;
static lv_obj_t *g_safetyLabel = nullptr;
static lv_obj_t *g_powerPopup = nullptr;
static lv_obj_t *g_powerLabel = nullptr;

static lv_obj_t *g_wifiPopup = nullptr;
static lv_obj_t *g_wifiNameArea = nullptr;
static lv_obj_t *g_wifiSsidArea = nullptr;
static lv_obj_t *g_wifiPasswordArea = nullptr;
static lv_obj_t *g_wifiToggleButton = nullptr;
static lv_obj_t *g_wifiToggleLabel = nullptr;
static lv_obj_t *g_wifiStateLabel = nullptr;
static lv_obj_t *g_wifiHint = nullptr;
static uint8_t g_wifiActiveField = 0; // 0 = name, 1 = SSID, 2 = password, 3 = on/off

static lv_obj_t *g_bootLabel = nullptr;
static uint16_t *g_mapBuffer = nullptr;

// Forward declarations used by map helpers and the canvas self-test.
static void renderMap();
static void renderMapIfDue();
static void scheduleMapRedraw();
static void resetLookaheadWindow();
static void refreshStatus();
static void uiToast(const String &text, uint32_t durationMs);
static void updateWifiPopupState();
static void disableWifi();
static void connectWifi();
static bool messagesVisible();
static bool tracksVisible();
static bool powerVisible();
static void rebuildMessageList();
static bool hasMapPosition(double lat, double lon);
static void appendPointToRecordingTrack(double lat, double lon, double speedKmph);
static bool saveTracksToSd();
static bool loadTracksFromSd();


// -----------------------------------------------------------------------------
// Keyboard queue
// -----------------------------------------------------------------------------

static portMUX_TYPE g_keyMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t g_keyRead = 0;
static volatile uint8_t g_keyWrite = 0;
static char g_keyQueue[32];

static void enqueueKey(char c) {
  portENTER_CRITICAL(&g_keyMux);
  uint8_t next = (uint8_t)((g_keyWrite + 1) % sizeof(g_keyQueue));
  if (next != g_keyRead) {
    g_keyQueue[g_keyWrite] = c;
    g_keyWrite = next;
  }
  portEXIT_CRITICAL(&g_keyMux);
}

static bool dequeueKey(char &c) {
  bool available = false;
  portENTER_CRITICAL(&g_keyMux);
  if (g_keyRead != g_keyWrite) {
    c = g_keyQueue[g_keyRead];
    g_keyRead = (uint8_t)((g_keyRead + 1) % sizeof(g_keyQueue));
    available = true;
  }
  portEXIT_CRITICAL(&g_keyMux);
  return available;
}

static void keyboardCallback(int state, char &c) {
  if (state == KB_PRESSED) {
    enqueueKey(c);
  }
}

// -----------------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------------

static uint32_t nowMs() {
  return millis();
}

static float angleDistance(float a, float b) {
  float d = fabsf(a - b);
  while (d > 360.0f) d -= 360.0f;
  return d > 180.0f ? 360.0f - d : d;
}

static String safetyName() {
  switch (g_safety) {
    case SAFETY_ARMED: return "SAFE";
    case SAFETY_WARNING: return "WARN";
    case SAFETY_SOS: return "SOS";
    default: return "OFF";
  }
}

static void setDisplay(bool on) {
  if (g_displayOn == on) return;
  g_displayOn = on;
  hw_set_disp_backlight(on ? APP_DISPLAY_BRIGHTNESS : 0);
  if (on) {
    hw_set_kb_backlight(1);
  } else {
    hw_set_kb_backlight(0);
  }
}

static void wakeDisplay() {
  g_lastInputMs = nowMs();
  setDisplay(true);
}

static int batteryMv() {
  return hw_get_battery_voltage();
}

static String replaceToken(String text, const String &token, const String &value) {
  text.replace(token, value);
  return text;
}

static void trimLine(String &s) {
  s.trim();
}

static String valueAfterEquals(const String &line) {
  int at = line.indexOf('=');
  if (at < 0) return "";
  String value = line.substring(at + 1);
  value.trim();
  return value;
}

static String sanitizeRadioField(String value, uint8_t maxLen, const String &fallback) {
  value.trim();
  value.replace("|", "/");
  value.replace("\r", " ");
  value.replace("\n", " ");

  String clean;
  clean.reserve(maxLen);

  for (uint16_t i = 0; i < value.length() && clean.length() < maxLen; i++) {
    char c = value[i];
    if ((uint8_t)c >= 32 && c != 127) clean += c;
  }

  clean.trim();
  return clean.isEmpty() ? fallback : clean;
}

// -----------------------------------------------------------------------------
// Acoustic SOS warning
// -----------------------------------------------------------------------------

static void prepareSosAudioBuffer() {
  for (int i = 0; i < APP_SOS_AUDIO_BEEP_SAMPLES; i++) {
    const float phase = 2.0f * PI * APP_SOS_AUDIO_FREQ_HZ * i /
                        APP_SOS_AUDIO_SAMPLE_RATE;
    g_sosBeepBuffer[i] = (int16_t)(22000.0f * sinf(phase));
  }
}

static bool ensureSosAudioOpen(uint8_t volume) {
#ifdef USING_AUDIO_CODEC
  if (g_sosAudioOpen) {
    instance.codec.setVolume(volume);
    return true;
  }

  instance.powerControl(POWER_SPEAK, true);
  delay(20);

  instance.codec.setVolume(volume);

  if (instance.codec.open(16, 1, APP_SOS_AUDIO_SAMPLE_RATE) < 0) {
    Serial.println("[AUDIO] codec.open failed");
    instance.powerControl(POWER_SPEAK, false);
    return false;
  }

  g_sosAudioOpen = true;
  Serial.println("[AUDIO] speaker ready");
  return true;
#else
  Serial.println("[AUDIO] USING_AUDIO_CODEC not defined; no Pager speaker output");
  return false;
#endif
}

static void playSosBeep(uint8_t volume = APP_SOS_AUDIO_LOCAL_VOLUME) {
#ifdef USING_AUDIO_CODEC
  if (!ensureSosAudioOpen(volume)) return;

  instance.codec.write((uint8_t *)g_sosBeepBuffer, sizeof(g_sosBeepBuffer));
#endif
}

static void stopSosAudio() {
#ifdef USING_AUDIO_CODEC
  if (!g_sosAudioOpen) return;

  instance.codec.close();
  instance.powerControl(POWER_SPEAK, false);
  g_sosAudioOpen = false;
  Serial.println("[AUDIO] speaker off");
#endif

  g_sosAudioBurstCount = 0;
  g_lastSosAudioActionMs = 0;
}

static void triggerRemoteSosAlarm() {
  g_remoteSosAlarmUntilMs = nowMs() + 4500;
  g_lastRemoteSosAlarmMs = 0;
  g_remoteSosAlarmCount = 0;
}

static void pollSosAudio() {
  if (g_sosAudioObservedState != g_safety) {
    g_sosAudioObservedState = g_safety;
    g_sosAudioBurstCount = 0;
    g_lastSosAudioActionMs = 0;

    if (g_safety == SAFETY_OFF || g_safety == SAFETY_ARMED) {
      stopSosAudio();
      return;
    }
  }

  uint32_t now = nowMs();

  if (g_safety != SAFETY_WARNING && g_safety != SAFETY_SOS &&
      now < g_remoteSosAlarmUntilMs) {
    if (g_lastRemoteSosAlarmMs == 0 ||
        now - g_lastRemoteSosAlarmMs >= 280) {
      g_lastRemoteSosAlarmMs = now;
      g_remoteSosAlarmCount++;
      playSosBeep(APP_SOS_AUDIO_REMOTE_VOLUME);
      if (g_remoteSosAlarmCount >= 10) {
        g_remoteSosAlarmUntilMs = 0;
        stopSosAudio();
      }
    }

    return;
  }

  if (g_remoteSosAlarmUntilMs != 0 && now >= g_remoteSosAlarmUntilMs) {
    g_remoteSosAlarmUntilMs = 0;
    g_remoteSosAlarmCount = 0;
    g_lastRemoteSosAlarmMs = 0;
    if (g_safety == SAFETY_OFF || g_safety == SAFETY_ARMED) stopSosAudio();
  }

  if (g_safety == SAFETY_WARNING) {
    if (g_lastSosAudioActionMs == 0 ||
        now - g_lastSosAudioActionMs >= APP_SOS_WARNING_BEEP_PERIOD_MS) {
      g_lastSosAudioActionMs = now;
      playSosBeep(APP_SOS_AUDIO_LOCAL_VOLUME);
    }

    return;
  }

  if (g_safety != SAFETY_SOS) return;

  if (g_sosAudioBurstCount < 3) {
    if (g_lastSosAudioActionMs == 0 ||
        now - g_lastSosAudioActionMs >= APP_SOS_ALARM_BEEP_GAP_MS) {
      g_lastSosAudioActionMs = now;
      g_sosAudioBurstCount++;
      playSosBeep(APP_SOS_AUDIO_LOCAL_VOLUME);
    }

    return;
  }

  if (now - g_lastSosAudioActionMs >= APP_SOS_ALARM_PAUSE_MS) {
    g_sosAudioBurstCount = 0;
    g_lastSosAudioActionMs = 0;
  }
}

// -----------------------------------------------------------------------------
// Persistent WiFi credentials in ESP32 NVS
// -----------------------------------------------------------------------------

static void loadWifiCredentialsFromNvs() {
  g_preferences.begin("pagerwifi", true);

  String savedSsid = g_preferences.getString("ssid", "");
  String savedPassword = g_preferences.getString("password", "");
  String savedName = g_preferences.getString("name", "");

  g_preferences.end();

  // SD configuration remains the preferred explicit source. NVS acts as a
  // fallback when /pager.ini is missing, unreadable or has empty WiFi fields.
  if (g_cfg.wifiSsid.isEmpty() && !savedSsid.isEmpty()) {
    g_cfg.wifiSsid = savedSsid;
    g_cfg.wifiPassword = savedPassword;
    Serial.printf("[WIFI] loaded credentials from NVS, ssid=%s\n",
                  g_cfg.wifiSsid.c_str());
  } else if (!g_cfg.wifiSsid.isEmpty()) {
    Serial.printf("[WIFI] credentials loaded from SD config, ssid=%s\n",
                  g_cfg.wifiSsid.c_str());
  } else {
    Serial.println("[WIFI] no stored credentials found");
  }

  if ((g_cfg.deviceName.isEmpty() || g_cfg.deviceName == "PAGER") &&
      !savedName.isEmpty()) {
    g_cfg.deviceName = savedName;
    Serial.printf("[CONFIG] loaded device_name from NVS: %s\n",
                  g_cfg.deviceName.c_str());
  }
}

static bool saveWifiCredentialsToNvs() {
  g_preferences.begin("pagerwifi", false);

  size_t ssidWritten = g_preferences.putString("ssid", g_cfg.wifiSsid);
  size_t passwordWritten = g_preferences.putString("password", g_cfg.wifiPassword);
  size_t nameWritten = g_preferences.putString("name", g_cfg.deviceName);

  g_preferences.end();

  bool ok = (!g_cfg.wifiSsid.isEmpty() && ssidWritten > 0) ||
            (!g_cfg.deviceName.isEmpty() && nameWritten > 0);
  Serial.printf("[WIFI] save NVS result=%s ssidBytes=%u passwordBytes=%u nameBytes=%u\n",
                ok ? "ok" : "failed",
                (unsigned int)ssidWritten,
                (unsigned int)passwordWritten,
                (unsigned int)nameWritten);
  return ok;
}

// -----------------------------------------------------------------------------
// SD access
// -----------------------------------------------------------------------------

static bool ensureSdReady(bool retryMount) {
  if (instance.isCardReady()) {
    g_sdReady = true;
    Serial.printf("[SD] ready cardType=%u cardMB=%llu sector=%u\n",
                  (unsigned int)SD.cardType(),
                  (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)),
                  (unsigned int)SD.sectorSize());
    return true;
  }

  g_sdReady = false;
  Serial.println("[SD] card not ready");

  if (!retryMount) return false;

  Serial.println("[SD] retry: power on + instance.installSD()");
  instance.powerControl(POWER_SD_CARD, true);
  delay(150);

  bool ok = instance.installSD();
  g_sdReady = ok && instance.isCardReady();

  Serial.printf("[SD] installSD result=%s ready=%s cardType=%u cardMB=%llu sector=%u\n",
                ok ? "ok" : "failed",
                g_sdReady ? "yes" : "no",
                (unsigned int)SD.cardType(),
                (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)),
                (unsigned int)SD.sectorSize());

  return g_sdReady;
}

static bool testSdWrite() {
  if (!ensureSdReady(true)) {
    Serial.println("[SD] WRITE TEST skipped: card not mounted");
    return false;
  }

  const char *path = "/pager_sd_write_test.tmp";

  instance.lockSPI();
  SD.remove(path);
  File f = SD.open(path, FILE_WRITE);

  if (!f) {
    Serial.println("[SD] WRITE TEST failed: cannot open root test file");
    instance.unlockSPI();
    return false;
  }

  size_t written = f.print("pager-sd-write-test\n");
  f.flush();
  f.close();

  bool exists = SD.exists(path);
  size_t size = 0;
  if (exists) {
    File verify = SD.open(path, FILE_READ);
    if (verify) {
      size = verify.size();
      verify.close();
    }
  }

  bool removed = SD.remove(path);
  instance.unlockSPI();

  bool ok = written > 0 && exists && size > 0;
  Serial.printf("[SD] WRITE TEST result=%s written=%u exists=%s size=%u remove=%s\n",
                ok ? "ok" : "failed",
                (unsigned int)written,
                exists ? "yes" : "no",
                (unsigned int)size,
                removed ? "ok" : "failed");

  return ok;
}


static bool sdTileFileUsable(const String &path) {
  instance.lockSPI();
  File file = SD.open(path, FILE_READ);

  if (!file) {
    instance.unlockSPI();
    return false;
  }

  const size_t fileSize = file.size();
  file.close();

  if (fileSize == 0 || fileSize > APP_TILE_MAX_PNG_BYTES) {
    SD.remove(path);
    instance.unlockSPI();

    Serial.printf("[SD] removed invalid tile size=%u path=%s\n",
                  (unsigned int)fileSize, path.c_str());
    return false;
  }

  instance.unlockSPI();
  return true;
}

static bool sdDirectoryExists(const String &path) {
  File dir = SD.open(path, FILE_READ);
  if (!dir) return false;

  bool ok = dir.isDirectory();
  dir.close();
  return ok;
}

static bool zoomAvailable(int zoom) {
  return zoom >= APP_ALLOWED_MIN_ZOOM && zoom <= APP_MAX_ZOOM;
}

static bool setZoomIfAvailable(int zoom) {
  if (!zoomAvailable(zoom)) {
    uiToast("Zoom " + String(zoom) + " nicht erreichbar", 1800);
    return false;
  }

  if (g_cfg.zoom == zoom) return true;

  g_cfg.zoom = zoom;
  resetLookaheadWindow();
  scheduleMapRedraw();
  renderMapIfDue();
  return true;
}

static bool stepZoomToAvailable(int direction) {
  int z = g_cfg.zoom + direction;

  while (z >= APP_ALLOWED_MIN_ZOOM && z <= APP_MAX_ZOOM) {
    if (setZoomIfAvailable(z)) return true;
    z += direction;
  }

  uiToast(direction > 0 ? "Kein hoeherer Zoom erreichbar" :
                          "Kein niedrigerer Zoom erreichbar", 1800);
  return false;
}

static bool ensureSdDirectory(const String &path) {
  if (path.isEmpty() || path == "/") return true;

  if (sdDirectoryExists(path)) {
    return true;
  }

  bool created = SD.mkdir(path);

  // Some ESP32 SD/FAT combinations may return false when a directory already
  // exists. Verify by opening it as a directory before treating that as an
  // actual failure.
  bool ready = sdDirectoryExists(path);

  Serial.printf("[SD] ensure dir %s mkdir=%s ready=%s\n",
                path.c_str(),
                created ? "ok" : "false",
                ready ? "yes" : "no");

  return ready;
}

static bool sdMkdirsForFile(const String &path) {
  int slash = 1;
  while (true) {
    slash = path.indexOf('/', slash);
    if (slash < 0) break;

    String dir = path.substring(0, slash);
    if (!ensureSdDirectory(dir)) {
      Serial.printf("[SD] directory unavailable: %s\n", dir.c_str());
      return false;
    }

    slash++;
  }

  return true;
}

static bool appendLog(const String &line) {
  // Runtime SD access is intentionally restricted to the Core-0 tile I/O task.
  // Keep chat logging on Serial for now to avoid cross-core filesystem access.
  Serial.printf("[CHAT LOG] %s\n", line.c_str());
  return true;
}

static void loadSettings() {
  instance.lockSPI();
  File f = SD.open(APP_CONFIG_PATH, FILE_READ);
  if (!f) {
    instance.unlockSPI();
    Serial.println("No /pager.ini found; using defaults.");
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith("#")) continue;

    int eq = line.indexOf('=');
    if (eq < 1) continue;

    String key = line.substring(0, eq);
    String value = line.substring(eq + 1);
    key.trim();
    value.trim();

    if (key == "device_id") g_cfg.deviceId = value;
    else if (key == "device_name") g_cfg.deviceName = value;
    else if (key == "wifi_ssid") g_cfg.wifiSsid = value;
    else if (key == "wifi_password") g_cfg.wifiPassword = value;
    else if (key == "tile_url") g_cfg.tileUrl = value;
    else if (key == "fallback_lat") g_cfg.fallbackLat = value.toDouble();
    else if (key == "fallback_lon") g_cfg.fallbackLon = value.toDouble();
    else if (key == "zoom") g_cfg.zoom = constrain(value.toInt(), APP_ALLOWED_MIN_ZOOM, APP_MAX_ZOOM);
    else if (key == "position_interval_seconds") g_cfg.positionIntervalMs = max(10L, value.toInt()) * 1000UL;
    else if (key == "safety_no_motion_seconds") g_cfg.safetyNoMotionMs = max(10L, value.toInt()) * 1000UL;
    else if (key == "safety_countdown_seconds") g_cfg.safetyCountdownMs = max(5L, value.toInt()) * 1000UL;
    else if (key == "auto_display") g_cfg.autoDisplay = value.toInt() != 0;
  }

  f.close();
  instance.unlockSPI();
}

static void applyDeviceIdentityDefaults() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t suffix = (uint32_t)(mac & 0xFFFFFF);

  if (g_cfg.deviceId.isEmpty() || g_cfg.deviceId == "pager") {
    char id[16];
    snprintf(id, sizeof(id), "pager%06lX", (unsigned long)suffix);
    g_cfg.deviceId = id;
  }

  if (g_cfg.deviceName.isEmpty() || g_cfg.deviceName == "PAGER") {
    char name[16];
    snprintf(name, sizeof(name), "P%06lX", (unsigned long)suffix);
    g_cfg.deviceName = name;
  }

  g_cfg.deviceId = sanitizeRadioField(g_cfg.deviceId, 24, "pager");
  g_cfg.deviceName = sanitizeRadioField(g_cfg.deviceName, 18, "PAGER");

  Serial.printf("[CONFIG] device_id=%s device_name=%s\n",
                g_cfg.deviceId.c_str(), g_cfg.deviceName.c_str());
}

static bool saveSettings() {
  // Runtime configuration writes to SD are disabled in v0.2.0 so Core 0 is the
  // sole SD owner after startup. WiFi credentials are stored in ESP32 NVS.
  Serial.println("[CONFIG] runtime SD write skipped; NVS remains authoritative");
  return false;
}

// -----------------------------------------------------------------------------
// Map tiles and PNG renderer
// -----------------------------------------------------------------------------

static PNG g_png;
static File g_pngFile;
static uint16_t *g_pngTargetBuffer = nullptr;
static uint16_t g_pngLine[TILE_SIZE];
static uint8_t g_tileIoBuffer[APP_TILE_IO_CHUNK_BYTES];

static QueueHandle_t g_tileRequestQueue = nullptr;
static QueueHandle_t g_tileResultQueue = nullptr;
static TaskHandle_t g_tileDownloadTaskHandle = nullptr;

static portMUX_TYPE g_tilePendingMux = portMUX_INITIALIZER_UNLOCKED;
static TileJob g_pendingTiles[APP_MAX_TILE_QUEUE];
static volatile uint8_t g_tileQueueCount = 0;

static SemaphoreHandle_t g_tileCacheMutex = nullptr;
static TileCacheSlot g_tileCache[APP_TILE_CACHE_SLOTS];
static uint32_t g_tileCacheUseCounter = 1;

static volatile bool g_mapNeedsRedraw = true;
static uint32_t g_lastUiRenderMs = 0;
static uint32_t g_lastDiagnosticsMs = 0;
static int g_lastLookaheadZoom = -1;
static int g_lastLookaheadMinX = 0;
static int g_lastLookaheadMaxX = 0;
static int g_lastLookaheadMinY = 0;
static int g_lastLookaheadMaxY = 0;

static void *pngOpen(const char *filename, int32_t *size) {
  g_pngFile = SD.open(filename, FILE_READ);
  if (!g_pngFile) return nullptr;
  *size = (int32_t)g_pngFile.size();
  return &g_pngFile;
}

static void pngClose(void *handle) {
  File *f = static_cast<File *>(handle);
  if (f) f->close();
}

static int32_t pngRead(PNGFILE *page, uint8_t *buffer, int32_t length) {
  File *f = static_cast<File *>(page->fHandle);
  return f ? (int32_t)f->read(buffer, length) : 0;
}

static int32_t pngSeek(PNGFILE *page, int32_t position) {
  File *f = static_cast<File *>(page->fHandle);
  return f && f->seek(position) ? position : -1;
}

static int pngDraw(PNGDRAW *draw) {
  if (!g_pngTargetBuffer || draw->y < 0 || draw->y >= TILE_SIZE) return 1;

  g_png.getLineAsRGB565(draw, g_pngLine, PNG_RGB565_LITTLE_ENDIAN, 0xFFFFFFFF);

  int width = draw->iWidth;
  if (width > TILE_SIZE) width = TILE_SIZE;

  memcpy(&g_pngTargetBuffer[draw->y * TILE_SIZE],
         g_pngLine,
         width * sizeof(uint16_t));
  return 1;
}

static String tilePath(int z, int x, int y) {
  return String(APP_MAP_ROOT) + "/" + z + "/" + x + "/" + y + ".png";
}

static bool tileQueuedUnsafe(int z, int x, int y, bool lookaheadOnly) {
  for (uint8_t i = 0; i < g_tileQueueCount; i++) {
    if (g_pendingTiles[i].z == z &&
        g_pendingTiles[i].x == x &&
        g_pendingTiles[i].y == y) {
      if (lookaheadOnly || !g_pendingTiles[i].lookaheadOnly) return true;
    }
  }

  return false;
}

static void removePendingTile(const TileJob &job) {
  portENTER_CRITICAL(&g_tilePendingMux);

  for (uint8_t i = 0; i < g_tileQueueCount; i++) {
    if (g_pendingTiles[i].z == job.z &&
        g_pendingTiles[i].x == job.x &&
        g_pendingTiles[i].y == job.y &&
        g_pendingTiles[i].lookaheadOnly == job.lookaheadOnly) {
      for (uint8_t j = i + 1; j < g_tileQueueCount; j++) {
        g_pendingTiles[j - 1] = g_pendingTiles[j];
      }

      g_tileQueueCount--;
      break;
    }
  }

  portEXIT_CRITICAL(&g_tilePendingMux);
}

static bool queueTileJob(int z, int x, int y, bool lookaheadOnly) {
  if (!g_tileRequestQueue) return false;

  TileJob job { z, x, y, lookaheadOnly };
  bool accepted = false;

  portENTER_CRITICAL(&g_tilePendingMux);

  const uint8_t queueLimit = lookaheadOnly ? (APP_MAX_TILE_QUEUE / 2) : APP_MAX_TILE_QUEUE;
  if (g_tileQueueCount < queueLimit && !tileQueuedUnsafe(z, x, y, lookaheadOnly)) {
    g_pendingTiles[g_tileQueueCount++] = job;
    accepted = true;
  }

  portEXIT_CRITICAL(&g_tilePendingMux);

  if (!accepted) return false;

  if (xQueueSend(g_tileRequestQueue, &job, 0) != pdTRUE) {
    removePendingTile(job);
    Serial.printf("[TILE TASK] request queue full for %d/%d/%d\n", z, x, y);
    return false;
  }

  g_lastTileState = "queued";
  return true;
}

static void queueVisibleTile(int z, int x, int y) {
  queueTileJob(z, x, y, false);
}


static int findReadyCacheSlotLocked(int z, int x, int y) {
  for (int i = 0; i < APP_TILE_CACHE_SLOTS; i++) {
    if (g_tileCache[i].allocated &&
        g_tileCache[i].ready &&
        !g_tileCache[i].loading &&
        g_tileCache[i].z == z &&
        g_tileCache[i].x == x &&
        g_tileCache[i].y == y) {
      return i;
    }
  }

  return -1;
}

static int findAnyCacheSlotLocked(int z, int x, int y) {
  for (int i = 0; i < APP_TILE_CACHE_SLOTS; i++) {
    if (g_tileCache[i].allocated &&
        g_tileCache[i].z == z &&
        g_tileCache[i].x == x &&
        g_tileCache[i].y == y) {
      return i;
    }
  }

  return -1;
}

static int chooseCacheSlotLocked() {
  int oldest = -1;
  uint32_t oldestUse = UINT32_MAX;

  for (int i = 0; i < APP_TILE_CACHE_SLOTS; i++) {
    if (!g_tileCache[i].allocated) continue;
    if (g_tileCache[i].loading) continue;

    if (!g_tileCache[i].ready) return i;

    if (g_tileCache[i].lastUsed < oldestUse) {
      oldestUse = g_tileCache[i].lastUsed;
      oldest = i;
    }
  }

  return oldest;
}

static bool initializeTileCache() {
  g_tileCacheMutex = xSemaphoreCreateMutex();
  if (!g_tileCacheMutex) {
    Serial.println("[CACHE] mutex creation failed");
    return false;
  }

  const size_t tileBytes = TILE_SIZE * TILE_SIZE * sizeof(uint16_t);

  for (int i = 0; i < APP_TILE_CACHE_SLOTS; i++) {
    g_tileCache[i].pixels = static_cast<uint16_t *>(ps_malloc(tileBytes));
    g_tileCache[i].allocated = g_tileCache[i].pixels != nullptr;

    Serial.printf("[CACHE] slot=%d allocated=%s bytes=%u\n",
                  i,
                  g_tileCache[i].allocated ? "yes" : "no",
                  (unsigned int)tileBytes);
  }

  return true;
}

static bool cacheHasReadyTile(int z, int x, int y) {
  if (!g_tileCacheMutex) return false;

  bool ready = false;
  if (xSemaphoreTake(g_tileCacheMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    int slot = findReadyCacheSlotLocked(z, x, y);
    ready = slot >= 0;

    if (ready) {
      g_tileCache[slot].lastUsed = g_tileCacheUseCounter++;
    }

    xSemaphoreGive(g_tileCacheMutex);
  }

  return ready;
}

static bool copyCachedTileToMap(int z, int x, int y, int screenX, int screenY) {
  if (!g_tileCacheMutex || !g_mapBuffer) return false;

  bool copied = false;

  if (xSemaphoreTake(g_tileCacheMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    int slot = findReadyCacheSlotLocked(z, x, y);

    if (slot >= 0 && g_tileCache[slot].pixels) {
      g_tileCache[slot].lastUsed = g_tileCacheUseCounter++;

      int srcX = screenX < 0 ? -screenX : 0;
      int srcY = screenY < 0 ? -screenY : 0;
      int dstX = screenX < 0 ? 0 : screenX;
      int dstY = screenY < 0 ? 0 : screenY;

      int width = TILE_SIZE - srcX;
      int height = TILE_SIZE - srcY;

      if (dstX + width > APP_DISPLAY_WIDTH) width = APP_DISPLAY_WIDTH - dstX;
      if (dstY + height > APP_DISPLAY_HEIGHT) height = APP_DISPLAY_HEIGHT - dstY;

      if (width > 0 && height > 0) {
        for (int row = 0; row < height; row++) {
          memcpy(&g_mapBuffer[(dstY + row) * APP_DISPLAY_WIDTH + dstX],
                 &g_tileCache[slot].pixels[(srcY + row) * TILE_SIZE + srcX],
                 width * sizeof(uint16_t));
        }

        copied = true;
      }
    }

    xSemaphoreGive(g_tileCacheMutex);
  }

  return copied;
}

static bool readSdFileIntoPsram(const String &path, uint8_t **dataOut, size_t *sizeOut) {
  if (!dataOut || !sizeOut) return false;

  *dataOut = nullptr;
  *sizeOut = 0;

  instance.lockSPI();
  File file = SD.open(path, FILE_READ);

  if (!file) {
    instance.unlockSPI();
    Serial.printf("[SD READ] open failed: %s\n", path.c_str());
    return false;
  }

  const size_t fileSize = file.size();
  instance.unlockSPI();

  if (fileSize == 0 || fileSize > APP_TILE_MAX_PNG_BYTES) {
    instance.lockSPI();
    file.close();
    instance.unlockSPI();

    Serial.printf("[SD READ] invalid PNG size=%u path=%s\n",
                  (unsigned int)fileSize, path.c_str());
    return false;
  }

  uint8_t *buffer = static_cast<uint8_t *>(ps_malloc(fileSize));
  if (!buffer) {
    instance.lockSPI();
    file.close();
    instance.unlockSPI();

    Serial.printf("[SD READ] PSRAM allocation failed size=%u path=%s\n",
                  (unsigned int)fileSize, path.c_str());
    return false;
  }

  size_t total = 0;

  while (total < fileSize) {
    const size_t wanted = min((size_t)APP_TILE_IO_CHUNK_BYTES, fileSize - total);

    instance.lockSPI();
    const int got = file.read(buffer + total, wanted);
    instance.unlockSPI();

    if (got <= 0) {
      free(buffer);

      instance.lockSPI();
      file.close();
      instance.unlockSPI();

      Serial.printf("[SD READ] read failed offset=%u path=%s\n",
                    (unsigned int)total, path.c_str());
      return false;
    }

    total += (size_t)got;
    vTaskDelay(pdMS_TO_TICKS(APP_TILE_TASK_YIELD_MS));
  }

  instance.lockSPI();
  file.close();
  instance.unlockSPI();

  *dataOut = buffer;
  *sizeOut = total;

  Serial.printf("[SD READ] ok bytes=%u path=%s\n",
                (unsigned int)total, path.c_str());
  return true;
}

static bool decodeTileFileIntoCache(const TileJob &job) {
  if (!g_tileCacheMutex) return false;

  int slot = -1;

  if (xSemaphoreTake(g_tileCacheMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("[CACHE] mutex timeout before decode");
    return false;
  }

  slot = findAnyCacheSlotLocked(job.z, job.x, job.y);
  if (slot >= 0 && g_tileCache[slot].ready && !g_tileCache[slot].loading) {
    g_tileCache[slot].lastUsed = g_tileCacheUseCounter++;
    xSemaphoreGive(g_tileCacheMutex);
    return true;
  }

  if (slot < 0) slot = chooseCacheSlotLocked();

  if (slot < 0 || !g_tileCache[slot].pixels) {
    xSemaphoreGive(g_tileCacheMutex);
    Serial.println("[CACHE] no free PSRAM cache slot");
    return false;
  }

  g_tileCache[slot].z = job.z;
  g_tileCache[slot].x = job.x;
  g_tileCache[slot].y = job.y;
  g_tileCache[slot].loading = true;
  g_tileCache[slot].ready = false;

  uint16_t *target = g_tileCache[slot].pixels;
  xSemaphoreGive(g_tileCacheMutex);

  String path = tilePath(job.z, job.x, job.y);

  uint8_t *compressedPng = nullptr;
  size_t compressedSize = 0;
  int rc = PNG_INVALID_FILE;

  if (readSdFileIntoPsram(path, &compressedPng, &compressedSize)) {
    g_pngTargetBuffer = target;
    g_lastPngPath = path;

    // Decode from PSRAM. The shared display/SD SPI bus remains available while
    // the CPU performs PNG decompression.
    rc = g_png.openRAM(compressedPng, (int)compressedSize, pngDraw);
    if (rc == PNG_SUCCESS) {
      rc = g_png.decode(nullptr, 0);
    }

    g_png.close();
    g_pngTargetBuffer = nullptr;
    free(compressedPng);
  }

  if (xSemaphoreTake(g_tileCacheMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_tileCache[slot].loading = false;
    g_tileCache[slot].ready = rc == PNG_SUCCESS;
    g_tileCache[slot].lastUsed = g_tileCacheUseCounter++;
    xSemaphoreGive(g_tileCacheMutex);
  }

  g_lastPngRc = rc;

  if (rc == PNG_SUCCESS) {
    g_pngDecodeOk++;
    Serial.printf("[PNG core=%d] decode ok path=%s slot=%d\n",
                  xPortGetCoreID(), path.c_str(), slot);
    return true;
  }

  g_pngDecodeFail++;
  Serial.printf("[PNG core=%d] decode failed rc=%d path=%s slot=%d\n",
                xPortGetCoreID(), rc, path.c_str(), slot);
  return false;
}


static void scheduleMapRedraw() {
  g_mapNeedsRedraw = true;
}

static void resetLookaheadWindow() {
  g_lastLookaheadZoom = -1;
}

static void renderMapIfDue() {
  if (!g_mapNeedsRedraw) return;

  uint32_t now = nowMs();
  if (now - g_lastUiRenderMs < APP_UI_RENDER_MIN_INTERVAL_MS) return;

  g_mapNeedsRedraw = false;
  g_lastUiRenderMs = now;
  renderMap();
}


static double clampLat(double lat) {
  if (lat < -85.05112878) return -85.05112878;
  if (lat > 85.05112878) return 85.05112878;
  return lat;
}

static void latLonToWorld(double lat, double lon, int zoom, double &x, double &y) {
  const double n = 256.0 * (1 << zoom);
  lat = clampLat(lat);
  double latRad = lat * PI / 180.0;
  x = (lon + 180.0) / 360.0 * n;
  y = (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / PI) / 2.0 * n;
}

static void worldToLatLon(double x, double y, int zoom, double &lat, double &lon) {
  const double n = 256.0 * (1 << zoom);
  lon = x / n * 360.0 - 180.0;

  const double mercator = PI - 2.0 * PI * y / n;
  lat = atan(sinh(mercator)) * 180.0 / PI;
  lat = clampLat(lat);
}

static double mapCenterLat() {
  if (!g_followGps && g_manualMapCenterValid) return g_manualMapLat;
  return g_hasGps ? g_gps.lat : g_cfg.fallbackLat;
}

static double mapCenterLon() {
  if (!g_followGps && g_manualMapCenterValid) return g_manualMapLon;
  return g_hasGps ? g_gps.lng : g_cfg.fallbackLon;
}

static String mapPanAxisName() {
  return g_mapPanAxis == MAP_PAN_HORIZONTAL ? "X" : "Y";
}

static void centerMapOnGps() {
  g_followGps = true;
  g_manualMapCenterValid = false;
  resetLookaheadWindow();
  scheduleMapRedraw();
  renderMapIfDue();
  refreshStatus();
  uiToast(g_hasGps ? "Karte wieder auf GPS zentriert" :
                     "Noch kein GPS-Fix; Startposition verwendet", 2600);
}

static void toggleMapPanAxis() {
  g_mapPanAxis = g_mapPanAxis == MAP_PAN_HORIZONTAL ?
                 MAP_PAN_VERTICAL : MAP_PAN_HORIZONTAL;

  Serial.printf("[ROTARY] axis switched to %s\n", mapPanAxisName().c_str());
  refreshStatus();
  uiToast("Scrollrad verschiebt Achse " + mapPanAxisName(), 1800);
}

static void panMapByRotaryStep(int direction) {
  double worldX, worldY;
  latLonToWorld(mapCenterLat(), mapCenterLon(), g_cfg.zoom, worldX, worldY);

  const double delta = direction * APP_MAP_PAN_STEP_PX;
  if (g_mapPanAxis == MAP_PAN_HORIZONTAL) {
    worldX += delta;
  } else {
    worldY += delta;
  }

  worldToLatLon(worldX, worldY, g_cfg.zoom, g_manualMapLat, g_manualMapLon);
  g_manualMapCenterValid = true;
  g_followGps = false;
  resetLookaheadWindow();

  scheduleMapRedraw();
  renderMapIfDue();
  refreshStatus();

  Serial.printf("[MAP PAN] axis=%s direction=%d center=%.6f,%.6f\n",
                mapPanAxisName().c_str(), direction,
                g_manualMapLat, g_manualMapLon);
}

static void fillMap(uint16_t color) {
  if (!g_mapBuffer) return;
  for (int i = 0; i < APP_DISPLAY_WIDTH * APP_DISPLAY_HEIGHT; i++) {
    g_mapBuffer[i] = color;
  }
}

static void putPixel(int x, int y, uint16_t color) {
  if (x >= 0 && x < APP_DISPLAY_WIDTH && y >= 0 && y < APP_DISPLAY_HEIGHT) {
    g_mapBuffer[y * APP_DISPLAY_WIDTH + x] = color;
  }
}

static void fillRect(int x, int y, int w, int h, uint16_t color) {
  for (int yy = y; yy < y + h; yy++) {
    for (int xx = x; xx < x + w; xx++) putPixel(xx, yy, color);
  }
}

static bool hasMapPosition(double lat, double lon) {
  if (lat < -85.05112878 || lat > 85.05112878) return false;
  if (lon < -180.0 || lon > 180.0) return false;
  return !(lat == 0.0 && lon == 0.0);
}

static void drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    putPixel(x0, y0, color);
    putPixel(x0 + 1, y0, color);
    putPixel(x0, y0 + 1, color);

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

static void drawTrackLine(int x0, int y0, int x1, int y1, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x0 - 1, y0, x1 - 1, y1, color);
  drawLine(x0, y0 - 1, x1, y1 - 1, color);
}

static bool mapPositionToScreen(double lat,
                                double lon,
                                double centerX,
                                double centerY,
                                int &screenX,
                                int &screenY) {
  if (!hasMapPosition(lat, lon)) return false;

  double markerX, markerY;
  latLonToWorld(lat, lon, g_cfg.zoom, markerX, markerY);
  screenX = APP_DISPLAY_WIDTH / 2 + (int)round(markerX - centerX);
  screenY = APP_DISPLAY_HEIGHT / 2 + (int)round(markerY - centerY);
  return true;
}

static void drawVisibleTracks() {
  double centerX, centerY;
  latLonToWorld(mapCenterLat(), mapCenterLon(), g_cfg.zoom, centerX, centerY);

  for (uint8_t i = 0; i < APP_MAX_TRACKS; i++) {
    if (!g_tracks[i].used || !g_tracks[i].visible ||
        !g_tracks[i].points || g_tracks[i].pointCount < 2) continue;

    uint16_t drawnSegments = 0;

    for (int32_t p = (int32_t)g_tracks[i].pointCount - 1;
         p > 0 && drawnSegments < APP_TRACK_DRAW_MAX_SEGMENTS;
         p--) {
      int x0 = 0;
      int y0 = 0;
      int x1 = 0;
      int y1 = 0;
      const TrackPoint &to = g_tracks[i].points[p];
      const TrackPoint &from = g_tracks[i].points[p - 1];

      const bool fromVisible = mapPositionToScreen(from.lat, from.lon, centerX, centerY, x0, y0) &&
                               x0 >= -8 && x0 <= APP_DISPLAY_WIDTH + 8 &&
                               y0 >= -8 && y0 <= APP_DISPLAY_HEIGHT + 8;
      const bool toVisible = mapPositionToScreen(to.lat, to.lon, centerX, centerY, x1, y1) &&
                             x1 >= -8 && x1 <= APP_DISPLAY_WIDTH + 8 &&
                             y1 >= -8 && y1 <= APP_DISPLAY_HEIGHT + 8;

      if (!fromVisible && !toVisible) {
        continue;
      }

      if (!fromVisible) {
        x0 = constrain(x0, -8, APP_DISPLAY_WIDTH + 8);
        y0 = constrain(y0, -8, APP_DISPLAY_HEIGHT + 8);
      }
      if (!toVisible) {
        x1 = constrain(x1, -8, APP_DISPLAY_WIDTH + 8);
        y1 = constrain(y1, -8, APP_DISPLAY_HEIGHT + 8);
      }

      drawTrackLine(x0, y0, x1, y1, g_tracks[i].color);
      drawnSegments++;
    }
  }
}

static void drawMarker(double lat, double lon, uint16_t color, bool cross) {
  double centerX, centerY, markerX, markerY;
  latLonToWorld(mapCenterLat(), mapCenterLon(), g_cfg.zoom, centerX, centerY);
  latLonToWorld(lat, lon, g_cfg.zoom, markerX, markerY);

  int x = APP_DISPLAY_WIDTH / 2 + (int)round(markerX - centerX);
  int y = APP_DISPLAY_HEIGHT / 2 + (int)round(markerY - centerY);

  for (int dy = -5; dy <= 5; dy++) {
    for (int dx = -5; dx <= 5; dx++) {
      if (dx * dx + dy * dy <= 25) putPixel(x + dx, y + dy, color);
    }
  }

  if (cross) {
    for (int d = -8; d <= 8; d++) {
      putPixel(x + d, y, RGB565(255, 255, 255));
      putPixel(x, y + d, RGB565(255, 255, 255));
    }
  }
}

static void hidePeerLabels() {
  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (g_peerLabels[i]) lv_obj_add_flag(g_peerLabels[i], LV_OBJ_FLAG_HIDDEN);
  }
}

static void hidePeerSosRings() {
  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (g_peerSosRings[i]) lv_obj_add_flag(g_peerSosRings[i], LV_OBJ_FLAG_HIDDEN);
  }
}

static void showPeerSosRing(uint8_t index, int markerX, int markerY) {
  if (index >= APP_MAX_PEERS || !g_peerSosRings[index]) return;

  g_peerRingCenterX[index] = markerX;
  g_peerRingCenterY[index] = markerY;
  lv_obj_clear_flag(g_peerSosRings[index], LV_OBJ_FLAG_HIDDEN);
}

static void updatePeerSosPulse() {
  const uint32_t now = nowMs();
  const uint8_t phase = (now % 1200UL) * 255UL / 1200UL;
  const int size = 18 + ((int)phase * 34) / 255;
  const lv_opa_t opa = (lv_opa_t)(230 - ((int)phase * 175) / 255);

  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (!g_peerSosRings[i] || lv_obj_has_flag(g_peerSosRings[i], LV_OBJ_FLAG_HIDDEN)) continue;

    lv_obj_set_size(g_peerSosRings[i], size, size);
    lv_obj_set_pos(g_peerSosRings[i],
                   g_peerRingCenterX[i] - size / 2,
                   g_peerRingCenterY[i] - size / 2);
    lv_obj_set_style_border_opa(g_peerSosRings[i], opa, 0);
  }
}

static void showPeerLabel(uint8_t index, const String &name, int markerX, int markerY, bool sos) {
  if (index >= APP_MAX_PEERS || !g_peerLabels[index]) return;

  String label = name;
  label.trim();
  if (label.isEmpty()) label = "Pager";
  if (label.length() > 14) label = label.substring(0, 14);

  lv_label_set_text(g_peerLabels[index], label.c_str());
  lv_obj_set_width(g_peerLabels[index], LV_SIZE_CONTENT);
  lv_obj_set_style_text_color(g_peerLabels[index],
                              sos ? lv_color_hex(0xFFCCCC) : lv_color_hex(0xD6ECFF),
                              0);
  lv_obj_set_style_bg_color(g_peerLabels[index], lv_color_black(), 0);
  lv_obj_set_style_bg_opa(g_peerLabels[index], LV_OPA_70, 0);

  int x = markerX + 9;
  int y = markerY - 9;
  int labelWidth = lv_obj_get_width(g_peerLabels[index]);
  if (labelWidth <= 0) labelWidth = 86;
  if (x > APP_DISPLAY_WIDTH - labelWidth - 2) x = markerX - labelWidth - 9;
  if (x < 0) x = 0;
  if (y < 16) y = 16;
  if (y > APP_DISPLAY_HEIGHT - 34) y = APP_DISPLAY_HEIGHT - 34;

  lv_obj_set_pos(g_peerLabels[index], x, y);
  lv_obj_clear_flag(g_peerLabels[index], LV_OBJ_FLAG_HIDDEN);
}

static void drawCanvasTestPattern() {
  if (!g_canvas || !g_mapBuffer) return;

  static uint16_t testLine[APP_DISPLAY_WIDTH];

  for (int y = 0; y < APP_DISPLAY_HEIGHT; y++) {
    for (int x = 0; x < APP_DISPLAY_WIDTH; x++) {
      const bool checker = (((x / 24) + (y / 24)) & 1) != 0;
      if (y < APP_DISPLAY_HEIGHT / 3) {
        testLine[x] = checker ? RGB565(255, 0, 0) : RGB565(80, 0, 0);
      } else if (y < (APP_DISPLAY_HEIGHT * 2) / 3) {
        testLine[x] = checker ? RGB565(0, 255, 0) : RGB565(0, 80, 0);
      } else {
        testLine[x] = checker ? RGB565(0, 0, 255) : RGB565(0, 0, 80);
      }
    }
    memcpy(&g_mapBuffer[y * APP_DISPLAY_WIDTH],
           testLine,
           APP_DISPLAY_WIDTH * sizeof(uint16_t));
  }

  lv_obj_invalidate(g_canvas);
  uiToast("Canvas-Testbild gezeichnet; C laedt Karte neu", 4500);
  Serial.println("[CANVAS] RGB test pattern drawn");
}

static void drawMissingTile(int screenX, int screenY) {
  fillRect(screenX, screenY, TILE_SIZE, TILE_SIZE, RGB565(38, 45, 50));
  for (int d = 0; d < TILE_SIZE; d += 16) {
    for (int x = 0; x < TILE_SIZE; x++) {
      putPixel(screenX + x, screenY + d, RGB565(65, 75, 80));
    }
    for (int y = 0; y < TILE_SIZE; y++) {
      putPixel(screenX + d, screenY + y, RGB565(65, 75, 80));
    }
  }
}

static void queueLookaheadTiles(int minTileX,
                                int maxTileX,
                                int minTileY,
                                int maxTileY,
                                int tileCount) {
  if (APP_LOOKAHEAD_TILE_RADIUS <= 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (g_lastLookaheadZoom == g_cfg.zoom &&
      g_lastLookaheadMinX == minTileX &&
      g_lastLookaheadMaxX == maxTileX &&
      g_lastLookaheadMinY == minTileY &&
      g_lastLookaheadMaxY == maxTileY) {
    return;
  }

  g_lastLookaheadZoom = g_cfg.zoom;
  g_lastLookaheadMinX = minTileX;
  g_lastLookaheadMaxX = maxTileX;
  g_lastLookaheadMinY = minTileY;
  g_lastLookaheadMaxY = maxTileY;

  const int lookMinX = minTileX - APP_LOOKAHEAD_TILE_RADIUS;
  const int lookMaxX = maxTileX + APP_LOOKAHEAD_TILE_RADIUS;
  const int lookMinY = minTileY - APP_LOOKAHEAD_TILE_RADIUS;
  const int lookMaxY = maxTileY + APP_LOOKAHEAD_TILE_RADIUS;

  for (int ty = lookMinY; ty <= lookMaxY; ty++) {
    if (ty < 0 || ty >= tileCount) continue;

    for (int tx = lookMinX; tx <= lookMaxX; tx++) {
      if (tx >= minTileX && tx <= maxTileX &&
          ty >= minTileY && ty <= maxTileY) {
        continue;
      }

      int wrappedX = tx % tileCount;
      if (wrappedX < 0) wrappedX += tileCount;
      queueTileJob(g_cfg.zoom, wrappedX, ty, true);
    }
  }
}

static void renderMap() {
  if (!g_mapBuffer || !g_canvas) return;
  fillMap(RGB565(38, 45, 50));
  hidePeerLabels();
  hidePeerSosRings();

  double centerX, centerY;
  latLonToWorld(mapCenterLat(), mapCenterLon(), g_cfg.zoom, centerX, centerY);

  const double left = centerX - APP_DISPLAY_WIDTH / 2.0;
  const double top = centerY - APP_DISPLAY_HEIGHT / 2.0;

  const int minTileX = (int)floor(left / TILE_SIZE);
  const int maxTileX = (int)floor((left + APP_DISPLAY_WIDTH - 1) / TILE_SIZE);
  const int minTileY = (int)floor(top / TILE_SIZE);
  const int maxTileY = (int)floor((top + APP_DISPLAY_HEIGHT - 1) / TILE_SIZE);
  const int tileCount = 1 << g_cfg.zoom;

  for (int ty = minTileY; ty <= maxTileY; ty++) {
    if (ty < 0 || ty >= tileCount) continue;
    for (int tx = minTileX; tx <= maxTileX; tx++) {
      int wrappedX = tx % tileCount;
      if (wrappedX < 0) wrappedX += tileCount;

      int screenX = (int)round(tx * TILE_SIZE - left);
      int screenY = (int)round(ty * TILE_SIZE - top);
      if (!copyCachedTileToMap(g_cfg.zoom, wrappedX, ty, screenX, screenY)) {
        drawMissingTile(screenX, screenY);
        queueVisibleTile(g_cfg.zoom, wrappedX, ty);
      }
    }
  }

  queueLookaheadTiles(minTileX, maxTileX, minTileY, maxTileY, tileCount);

  drawVisibleTracks();

  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (!g_peers[i].used) continue;
    if (!hasMapPosition(g_peers[i].lat, g_peers[i].lon)) continue;
    drawMarker(g_peers[i].lat, g_peers[i].lon,
               g_peers[i].sos ? RGB565(255, 0, 0) : RGB565(0, 85, 255), false);

    int peerX = 0;
    int peerY = 0;
    if (mapPositionToScreen(g_peers[i].lat, g_peers[i].lon, centerX, centerY, peerX, peerY)) {
      showPeerLabel(i, g_peers[i].name, peerX, peerY, g_peers[i].sos);
      if (g_peers[i].sos) showPeerSosRing(i, peerX, peerY);
    }
  }
  if (g_hasGps && hasMapPosition(g_gps.lat, g_gps.lng)) {
    drawMarker(g_gps.lat, g_gps.lng, RGB565(120, 255, 120), true);
  }

  lv_obj_invalidate(g_canvas);
  Serial.printf("[MAP] redraw center=%.6f,%.6f zoom=%u png=%lu/%lu queue=%u state=%s\n",
                mapCenterLat(), mapCenterLon(), g_cfg.zoom,
                (unsigned long)g_pngDecodeOk, (unsigned long)g_pngDecodeFail,
                g_tileQueueCount, g_lastTileState.c_str());
}

static bool saveHttpResponseToFile(HTTPClient &http, const String &path) {
  if (!ensureSdReady(true)) {
    Serial.printf("[SD] save aborted: card not ready for %s\n", path.c_str());
    return false;
  }

  instance.lockSPI();

  if (!sdMkdirsForFile(path)) {
    Serial.printf("[SD] directory creation failed for %s\n", path.c_str());
    instance.unlockSPI();
    return false;
  }

  if (SD.exists(path)) {
    bool removed = SD.remove(path);
    Serial.printf("[SD] remove old %s -> %s\n",
                  path.c_str(), removed ? "ok" : "failed");
  }

  File out = SD.open(path, FILE_WRITE);
  instance.unlockSPI();

  if (!out) {
    Serial.printf("[SD] open for write failed: %s\n", path.c_str());
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();
  int remaining = http.getSize();
  int totalWritten = 0;
  bool writeOk = stream != nullptr;
  uint32_t lastDataMs = millis();

  while (writeOk && http.connected() && (remaining > 0 || remaining == -1)) {
    const size_t available = stream->available();

    if (available == 0) {
      if (millis() - lastDataMs > APP_TILE_STREAM_TIMEOUT_MS) {
        Serial.printf("[SD] stream timeout path=%s\n", path.c_str());
        writeOk = false;
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(APP_TILE_TASK_YIELD_MS));
      continue;
    }

    size_t wanted = min(available, (size_t)APP_TILE_IO_CHUNK_BYTES);
    if (remaining > 0 && wanted > (size_t)remaining) {
      wanted = (size_t)remaining;
    }

    const int received = stream->readBytes(g_tileIoBuffer, wanted);
    if (received <= 0) {
      Serial.printf("[SD] stream read failed path=%s\n", path.c_str());
      writeOk = false;
      break;
    }

    instance.lockSPI();
    const size_t written = out.write(g_tileIoBuffer, (size_t)received);
    instance.unlockSPI();

    if (written != (size_t)received) {
      Serial.printf("[SD] block write failed received=%d written=%u path=%s\n",
                    received, (unsigned int)written, path.c_str());
      writeOk = false;
      break;
    }

    totalWritten += (int)written;
    if (remaining > 0) remaining -= received;
    lastDataMs = millis();

    // Give the UI, display driver and idle task time between SD blocks.
    vTaskDelay(pdMS_TO_TICKS(APP_TILE_TASK_YIELD_MS));
  }

  instance.lockSPI();
  out.flush();
  out.close();
  instance.unlockSPI();

  if (!writeOk || totalWritten <= 0 || remaining > 0) {
    instance.lockSPI();
    SD.remove(path);
    instance.unlockSPI();

    Serial.printf("[SD] removed incomplete file written=%d remaining=%d path=%s\n",
                  totalWritten, remaining, path.c_str());
    return false;
  }

  instance.lockSPI();
  File verify = SD.open(path, FILE_READ);

  if (!verify) {
    SD.remove(path);
    instance.unlockSPI();

    Serial.printf("[SD] verify open failed: %s\n", path.c_str());
    return false;
  }

  const size_t actualSize = verify.size();
  verify.close();
  instance.unlockSPI();

  Serial.printf("[SD] verify size path=%s size=%u written=%d\n",
                path.c_str(), (unsigned int)actualSize, totalWritten);

  if (actualSize == 0 || actualSize != (size_t)totalWritten) {
    instance.lockSPI();
    SD.remove(path);
    instance.unlockSPI();

    Serial.printf("[SD] invalid file removed: %s\n", path.c_str());
    return false;
  }

  return true;
}


static TileResult downloadTileJob(const TileJob &job) {
  TileResult result {};
  result.job = job;
  result.lookaheadOnly = job.lookaheadOnly;

  if (!job.lookaheadOnly && cacheHasReadyTile(job.z, job.x, job.y)) {
    result.success = true;
    result.cached = true;
    return result;
  }

  String path = tilePath(job.z, job.x, job.y);

  if (sdTileFileUsable(path)) {
    if (job.lookaheadOnly) {
      result.success = true;
      result.cached = true;
      return result;
    }

    result.success = decodeTileFileIntoCache(job);
    result.cached = result.success;

    if (result.success) {
      return result;
    }

    instance.lockSPI();
    SD.remove(path);
    instance.unlockSPI();
    Serial.printf("[SD] removed undecodable tile: %s\n", path.c_str());
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[TILE TASK] offline and tile missing: %s\n", path.c_str());
    return result;
  }

  String url = g_cfg.tileUrl;
  url.replace("{z}", String(job.z));
  url.replace("{x}", String(job.x));
  url.replace("{y}", String(job.y));

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.printf("[TILE TASK] http.begin failed for %s\n", url.c_str());
    return result;
  }

  http.addHeader("User-Agent", APP_TILE_USER_AGENT);

  Serial.printf("[TILE TASK core=%d] GET %s\n", xPortGetCoreID(), url.c_str());
  result.httpCode = http.GET();

  if (result.httpCode == HTTP_CODE_OK) {
    bool saved = saveHttpResponseToFile(http, path);

    Serial.printf("[TILE TASK] HTTP %d save=%s path=%s\n",
                  result.httpCode,
                  saved ? "ok" : "failed",
                  path.c_str());

    if (saved) {
      result.success = job.lookaheadOnly ? true : decodeTileFileIntoCache(job);
    }
  } else {
    Serial.printf("[TILE TASK] HTTP error %d for %s\n",
                  result.httpCode,
                  url.c_str());
  }

  http.end();
  vTaskDelay(pdMS_TO_TICKS(APP_TILE_TASK_YIELD_MS));
  return result;
}


static void tileDownloadTask(void *parameter) {
  (void)parameter;

  Serial.printf("[TILE TASK] started on core %d\n", xPortGetCoreID());

  for (;;) {
    TileJob job {};

    if (xQueueReceive(g_tileRequestQueue, &job, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    TileResult result = downloadTileJob(job);

    if (xQueueSend(g_tileResultQueue, &result, portMAX_DELAY) != pdTRUE) {
      Serial.println("[TILE TASK] result queue send failed");
    }
  }
}

static void processTileResults() {
  if (!g_tileResultQueue) return;

  bool redraw = false;
  TileResult result {};

  while (xQueueReceive(g_tileResultQueue, &result, 0) == pdTRUE) {
    removePendingTile(result.job);

    g_lastTileHttpCode = result.httpCode;

    if (result.lookaheadOnly) {
      g_lastTileState = result.success ? "lookahead" : "lookahead-failed";
    } else if (result.cached) {
      g_lastTileState = "cached";
    } else if (result.success) {
      g_lastTileState = "saved";
    } else {
      g_lastTileState = "save-failed";
    }

    if (!result.lookaheadOnly) redraw = true;
  }

  if (redraw) {
    scheduleMapRedraw();
    refreshStatus();
  }
}



// -----------------------------------------------------------------------------
// UI
// -----------------------------------------------------------------------------

static lv_obj_t *makeLabel(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_pos(label, x, y);
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  return label;
}

static lv_obj_t *makeBoundedLabel(lv_obj_t *parent, const char *text,
                                  lv_coord_t x, lv_coord_t y,
                                  lv_coord_t width,
                                  lv_label_long_mode_t longMode = LV_LABEL_LONG_DOT) {
  lv_obj_t *label = makeLabel(parent, text, x, y);
  lv_obj_set_width(label, width);
  lv_label_set_long_mode(label, longMode);
  return label;
}

static void enableRecolor(lv_obj_t *label) {
  if (label) lv_label_set_recolor(label, true);
}

static void stylePopupShell(lv_obj_t *popup, uint32_t bgColor) {
  lv_obj_set_style_bg_color(popup, lv_color_hex(bgColor), 0);
  lv_obj_set_style_bg_opa(popup, LV_OPA_90, 0);
  lv_obj_set_style_border_color(popup, lv_color_white(), 0);
  lv_obj_set_style_pad_all(popup, 0, 0);
  lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);
}

static void uiToast(const String &text, uint32_t durationMs = 2200) {
  if (!g_toast) {
    g_toast = lv_label_create(lv_layer_top());
    lv_obj_set_style_bg_color(g_toast, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(g_toast, LV_OPA_90, 0);
    lv_obj_set_style_text_color(g_toast, lv_color_white(), 0);
    lv_obj_set_style_pad_all(g_toast, 5, 0);
    lv_obj_align(g_toast, LV_ALIGN_BOTTOM_MID, 0, -21);
  }

  lv_label_set_text(g_toast, text.c_str());
  lv_obj_clear_flag(g_toast, LV_OBJ_FLAG_HIDDEN);
  g_toast_until_ms = nowMs() + durationMs;
}

void ui_msg_pop_up(const char *title_txt, const char *msg_txt) {
  uiToast(String(title_txt) + ": " + msg_txt, 3500);
}

static void refreshStatus() {
  if (!g_status || !g_footer) return;

  String gpsText;
  if (g_hasGps) {
    gpsText = String(g_gps.satellite) + " SAT";
  } else if (g_hasGpsInfo) {
    gpsText = String(g_gpsRaw.satellite) + " SAT";
  } else {
    gpsText = "GPS...";
  }

  String wifiText = WiFi.status() == WL_CONNECTED ? "WIFI" :
                    (g_wifiEnabled ? "W..." : "W-AUS");
  String mapMode = g_followGps ? "GPS" : "PAN-" + mapPanAxisName();
  String text = gpsText + " Z" + String(g_cfg.zoom) + " " + wifiText +
                " " + mapMode;
  if (g_recordingTrackIndex >= 0) text += " REC";
  text += " " + safetyName();
  lv_label_set_text(g_status, text.c_str());
  updateWifiPopupState();

  String footer = "#FFD000 Rad:#Karte #FFD000 Dr:#X/Y #FFD000 C:#GPS "
                  "#FFD000 M:#Msg #FFD000 K:#Track #FFD000 W:#WLAN "
                  "#FFD000 S:#SOS #FFD000 P:#Aus";
  lv_label_set_text(g_footer, footer.c_str());
}

static void addChatMessage(const String &sender, const String &text, bool outgoing) {
  if (g_messageCount >= APP_MAX_MESSAGES) {
    for (uint8_t i = 1; i < APP_MAX_MESSAGES; i++) g_messages[i - 1] = g_messages[i];
    g_messageCount = APP_MAX_MESSAGES - 1;
  }

  g_messages[g_messageCount++] = ChatMessage { sender, text, outgoing, nowMs() };
  appendLog(String(nowMs()) + "|" + sender + "|" + text);
  g_selectedMessageListItem = g_messageCount > 0 ? g_messageCount - 1 : -1;
  if (messagesVisible()) rebuildMessageList();
}

static void styleListItem(lv_obj_t *label, bool selected, bool accent = false) {
  lv_obj_set_width(label, 410);
  lv_obj_set_style_pad_all(label, 2, 0);
  lv_obj_set_style_bg_opa(label, selected ? LV_OPA_60 : LV_OPA_TRANSP, 0);
  lv_obj_set_style_bg_color(label, lv_color_hex(0x2D5D7B), 0);
  lv_obj_set_style_text_color(label,
    selected ? lv_color_hex(0xFFFFFF) :
               (accent ? lv_color_hex(0xB8E986) : lv_color_white()), 0);
}

static void styleEdgeEscLabel(lv_obj_t *label, bool selected) {
  if (!label) return;
  lv_obj_set_size(label, 64, 16);
  lv_obj_set_style_pad_all(label, 1, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_bg_opa(label, selected ? LV_OPA_80 : LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(label, selected ? lv_color_hex(0x2D5D7B) : lv_color_hex(0x15212A), 0);
  lv_obj_set_style_text_color(label,
                              selected ? lv_color_white() : lv_color_hex(0xFFD000),
                              0);
}

static void scrollSelectedListItemIntoView(lv_obj_t **items, int16_t count, int16_t selected) {
  if (selected < 0 || selected >= count || !items[selected]) return;
  lv_obj_scroll_to_view(items[selected], LV_ANIM_OFF);
}

static void rebuildMessageList() {
  if (!g_messagesList) return;
  lv_obj_clean(g_messagesList);
  g_messageListItemCount = 0;

  for (uint8_t i = 0; i < g_messageCount; i++) {
    lv_obj_t *label = lv_label_create(g_messagesList);
    String line = (g_messages[i].outgoing ? "> " : "< ") +
                  g_messages[i].sender + ": " + g_messages[i].text;
    lv_label_set_text(label, line.c_str());
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    g_messageListItems[g_messageListItemCount++] = label;
  }

  if (g_selectedMessageListItem < -1) g_selectedMessageListItem = -1;
  if (g_selectedMessageListItem > g_messageListItemCount) {
    g_selectedMessageListItem = g_messageListItemCount;
  }

  for (int16_t i = 0; i < g_messageListItemCount; i++) {
    const bool outgoing = i < g_messageCount && g_messages[i].outgoing;
    styleListItem(g_messageListItems[i], i == g_selectedMessageListItem, outgoing);
  }

  styleEdgeEscLabel(g_messagesEscTop, g_selectedMessageListItem < 0);
  styleEdgeEscLabel(g_messagesEscBottom, g_selectedMessageListItem == g_messageListItemCount);

  scrollSelectedListItemIntoView(g_messageListItems,
                                 g_messageListItemCount,
                                 g_selectedMessageListItem);
}

static void hideMessagesPopup() {
  if (g_messagesPopup) lv_obj_add_flag(g_messagesPopup, LV_OBJ_FLAG_HIDDEN);
}

static void showMessagesPopup(bool focusTextArea) {
  wakeDisplay();
  g_selectedMessageListItem = g_messageCount > 0 ? g_messageCount - 1 : -1;
  rebuildMessageList();
  lv_obj_clear_flag(g_messagesPopup, LV_OBJ_FLAG_HIDDEN);
  if (focusTextArea) {
    lv_group_focus_obj(g_textarea);
  }
}

static uint16_t trackColorByIndex(uint8_t index) {
  static const uint16_t colors[] = {
    RGB565(255, 70, 70),
    RGB565(70, 170, 255),
    RGB565(80, 230, 120),
    RGB565(255, 210, 70),
    RGB565(210, 100, 255),
    RGB565(255, 140, 60),
    RGB565(80, 240, 230),
    RGB565(255, 110, 180)
  };

  return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

static int8_t newestTrackIndex() {
  for (int8_t i = APP_MAX_TRACKS - 1; i >= 0; i--) {
    if (g_tracks[i].used) return i;
  }

  return -1;
}

static MapTrack *selectedTrack() {
  if (g_selectedTrackIndex < 0 || g_selectedTrackIndex >= APP_MAX_TRACKS) return nullptr;
  if (!g_tracks[g_selectedTrackIndex].used) return nullptr;
  return &g_tracks[g_selectedTrackIndex];
}

static void releaseTrackPoints(MapTrack &track) {
  if (track.points) {
    free(track.points);
    track.points = nullptr;
  }
  track.pointCount = 0;
  track.pointCapacity = 0;
}

static bool allocateTrackPoints(MapTrack &track) {
  if (track.points && track.pointCapacity >= APP_MAX_TRACK_POINTS) return true;

  releaseTrackPoints(track);
  const size_t bytes = sizeof(TrackPoint) * (size_t)APP_MAX_TRACK_POINTS;
  track.points = static_cast<TrackPoint *>(ps_malloc(bytes));

  if (!track.points) {
    Serial.println("[TRACK] PSRAM allocation failed; trying heap");
    track.points = static_cast<TrackPoint *>(malloc(bytes));
  }

  if (!track.points) {
    Serial.printf("[TRACK] allocation failed bytes=%u\n", (unsigned int)bytes);
    return false;
  }

  track.pointCapacity = APP_MAX_TRACK_POINTS;
  track.pointCount = 0;
  Serial.printf("[TRACK] allocated points=%u bytes=%u freePsram=%u\n",
                (unsigned int)track.pointCapacity,
                (unsigned int)bytes,
                (unsigned int)ESP.getFreePsram());
  return true;
}

static void resetTrackSlot(uint8_t slot) {
  if (slot >= APP_MAX_TRACKS) return;
  releaseTrackPoints(g_tracks[slot]);
  g_tracks[slot] = MapTrack {};
}

static int8_t createTrack(const String &requestedName) {
  int8_t slot = -1;
  for (uint8_t i = 0; i < APP_MAX_TRACKS; i++) {
    if (!g_tracks[i].used) {
      slot = i;
      break;
    }
  }

  if (slot < 0) {
    uiToast("Trackliste voll", 2500);
    return -1;
  }

  resetTrackSlot(slot);
  if (!allocateTrackPoints(g_tracks[slot])) {
    uiToast("Track: kein Speicher", 3000);
    return -1;
  }

  String name = requestedName;
  name.trim();
  if (name.isEmpty()) name = "Track " + String(slot + 1);

  g_tracks[slot].used = true;
  g_tracks[slot].visible = true;
  g_tracks[slot].recording = false;
  g_tracks[slot].name = name;
  g_tracks[slot].color = trackColorByIndex(g_nextTrackColor++);
  g_tracks[slot].pointCount = 0;
  g_selectedTrackIndex = slot;

  Serial.printf("[TRACK] created index=%d name=%s\n", slot, name.c_str());
  scheduleMapRedraw();
  return slot;
}

static void rebuildTracksList() {
  if (!g_tracksList) return;
  lv_obj_clean(g_tracksList);
  g_trackListItemCount = 0;

  for (uint8_t i = 0; i < APP_MAX_TRACKS; i++) {
    if (!g_tracks[i].used) continue;

    lv_obj_t *label = lv_label_create(g_tracksList);
    String line = String(g_tracks[i].visible ? "[x] " : "[  ] ") +
                  (g_tracks[i].recording ? "REC " : "    ") +
                  g_tracks[i].name +
                  " (" + String(g_tracks[i].pointCount) + ")";
    lv_label_set_text(label, line.c_str());
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    g_trackListItems[g_trackListItemCount] = label;
    g_trackListItemToTrack[g_trackListItemCount++] = i;
  }

  if (newestTrackIndex() < 0) {
    lv_obj_t *empty = lv_label_create(g_tracksList);
    lv_label_set_text(empty, "Noch keine Tracks. Namen eingeben und Enter.");
    g_trackListItems[g_trackListItemCount] = empty;
    g_trackListItemToTrack[g_trackListItemCount++] = -2;
  }

  if (g_selectedTrackListItem < -1) g_selectedTrackListItem = -1;
  if (g_selectedTrackListItem > g_trackListItemCount) {
    g_selectedTrackListItem = g_trackListItemCount;
  }

  for (int16_t i = 0; i < g_trackListItemCount; i++) {
    const int8_t trackIndex = g_trackListItemToTrack[i];
    const bool accent = trackIndex >= 0 && g_tracks[trackIndex].recording;
    styleListItem(g_trackListItems[i], i == g_selectedTrackListItem, accent);
  }

  styleEdgeEscLabel(g_tracksEscTop, g_selectedTrackListItem < 0);
  styleEdgeEscLabel(g_tracksEscBottom, g_selectedTrackListItem == g_trackListItemCount);

  scrollSelectedListItemIntoView(g_trackListItems,
                                 g_trackListItemCount,
                                 g_selectedTrackListItem);
}

static void selectTrackListItemForTrack(int8_t trackIndex) {
  for (int16_t i = 0; i < g_trackListItemCount; i++) {
    if (g_trackListItemToTrack[i] == trackIndex) {
      g_selectedTrackListItem = i;
      rebuildTracksList();
      return;
    }
  }
}

static void hideTracksPopup() {
  if (g_tracksPopup) lv_obj_add_flag(g_tracksPopup, LV_OBJ_FLAG_HIDDEN);
}

static void showTracksPopup() {
  wakeDisplay();
  int8_t newest = newestTrackIndex();
  g_selectedTrackIndex = newest;
  g_selectedTrackListItem = newest >= 0 ? g_trackListItemCount : -1;
  rebuildTracksList();
  if (newest >= 0) selectTrackListItemForTrack(newest);
  lv_obj_clear_flag(g_tracksPopup, LV_OBJ_FLAG_HIDDEN);
  lv_group_focus_obj(g_trackNameArea);
}

static void selectTrackStep(int direction) {
  if (g_trackListItemCount <= 0) return;

  int16_t next = g_selectedTrackListItem + direction;
  if (next < -1) next = -1;
  if (next > g_trackListItemCount) next = g_trackListItemCount;
  if (next == g_selectedTrackListItem) return;

  g_selectedTrackListItem = next;
  const int8_t trackIndex = (g_selectedTrackListItem >= 0 &&
                             g_selectedTrackListItem < g_trackListItemCount) ?
                            g_trackListItemToTrack[g_selectedTrackListItem] : -1;
  if (trackIndex >= 0) {
    g_selectedTrackIndex = trackIndex;
  }

  rebuildTracksList();
}

static void toggleSelectedTrackVisibility() {
  if (g_selectedTrackListItem < 0 || g_selectedTrackListItem >= g_trackListItemCount) return;

  const int8_t trackIndex = g_trackListItemToTrack[g_selectedTrackListItem];
  if (trackIndex < 0 || trackIndex >= APP_MAX_TRACKS) return;

  MapTrack *track = &g_tracks[trackIndex];
  if (!track->used) return;

  g_selectedTrackIndex = trackIndex;

  track->visible = !track->visible;
  rebuildTracksList();
  scheduleMapRedraw();
  renderMapIfDue();
}

static void stopTrackRecording() {
  if (g_recordingTrackIndex >= 0 && g_recordingTrackIndex < APP_MAX_TRACKS) {
    g_tracks[g_recordingTrackIndex].recording = false;
  }
  g_recordingTrackIndex = -1;
}

static void toggleTrackRecording() {
  if (g_recordingTrackIndex >= 0) {
    stopTrackRecording();
    uiToast("Track-Aufnahme gestoppt", 1800);
    rebuildTracksList();
    saveTracksToSd();
    refreshStatus();
    return;
  }

  int8_t index = newestTrackIndex();
  if (index < 0) index = createTrack("Track 1");
  if (index < 0) return;

  g_recordingTrackIndex = index;
  g_selectedTrackIndex = index;
  g_tracks[index].recording = true;
  if (g_hasGps) appendPointToRecordingTrack(g_gps.lat, g_gps.lng, g_gps.speed);
  uiToast("Track-Aufnahme: " + g_tracks[index].name, 2200);
  rebuildTracksList();
  if (tracksVisible()) selectTrackListItemForTrack(index);
  refreshStatus();
}

static double trackBearingDeg(double fromLat, double fromLon, double toLat, double toLon) {
  const double fromLatRad = fromLat * PI / 180.0;
  const double toLatRad = toLat * PI / 180.0;
  const double deltaLonRad = (toLon - fromLon) * PI / 180.0;
  const double y = sin(deltaLonRad) * cos(toLatRad);
  const double x = cos(fromLatRad) * sin(toLatRad) -
                   sin(fromLatRad) * cos(toLatRad) * cos(deltaLonRad);
  double bearing = atan2(y, x) * 180.0 / PI;
  if (bearing < 0.0) bearing += 360.0;
  return bearing;
}

static double trackAngleDiffDeg(double a, double b) {
  double diff = fabs(a - b);
  while (diff > 360.0) diff -= 360.0;
  return diff > 180.0 ? 360.0 - diff : diff;
}

static double trackSmartAngleLimit(double speedKmph) {
  if (speedKmph < 6.0) return 45.0;
  if (speedKmph < 12.0) return 35.0;
  if (speedKmph < 25.0) return 25.0;
  return 18.0;
}

static bool shouldAppendTrackPoint(MapTrack &track,
                                   double lat,
                                   double lon,
                                   double speedKmph,
                                   double *newDirectionOut) {
  if (!track.points) return false;
  if (track.pointCount == 0) return true;

  const TrackPoint &last = track.points[track.pointCount - 1];
  const bool newPos =
      fabs((double)last.lat - lat) >= APP_TRACK_MIN_POINT_DELTA_DEG ||
      fabs((double)last.lon - lon) >= APP_TRACK_MIN_POINT_DELTA_DEG;

  if (!newPos) return false;

  const uint32_t now = nowMs();
  if (now - last.timeMs >= APP_TRACK_INTERVAL_MS) {
    if (newDirectionOut) {
      *newDirectionOut = trackBearingDeg(last.lat, last.lon, lat, lon);
    }
    return true;
  }

  const double newDirection = trackBearingDeg(last.lat, last.lon, lat, lon);
  if (newDirectionOut) *newDirectionOut = newDirection;

  if (!track.hasDirection) return false;
  if (speedKmph <= APP_TRACK_SMART_SPEED_KMPH) return false;

  return trackAngleDiffDeg(track.lastDirection, newDirection) >
         trackSmartAngleLimit(speedKmph);
}

static void appendPointToRecordingTrack(double lat, double lon, double speedKmph) {
  if (g_recordingTrackIndex < 0 || g_recordingTrackIndex >= APP_MAX_TRACKS) return;
  if (!hasMapPosition(lat, lon)) return;

  MapTrack &track = g_tracks[g_recordingTrackIndex];
  if (!track.used || !track.recording) return;
  if (!track.points && !allocateTrackPoints(track)) return;

  double newDirection = track.lastDirection;
  if (!shouldAppendTrackPoint(track, lat, lon, speedKmph, &newDirection)) return;

  if (track.pointCount >= track.pointCapacity) {
    if (track.pointCapacity == 0) return;
    memmove(track.points,
            track.points + 1,
            sizeof(TrackPoint) * (size_t)(track.pointCapacity - 1));
    track.pointCount = track.pointCapacity - 1;
  }

  TrackPoint &point = track.points[track.pointCount++];
  point.lat = (float)lat;
  point.lon = (float)lon;
  point.timeMs = nowMs();

  if (track.pointCount >= 2) {
    track.lastDirection = newDirection;
    track.hasDirection = true;
  }

  if (tracksVisible()) rebuildTracksList();
  scheduleMapRedraw();
}

static String cleanTrackNameForStorage(String name) {
  name.replace("&", "&amp;");
  name.replace("<", "&lt;");
  name.replace(">", "&gt;");
  name.replace("\"", "&quot;");
  name.replace("'", "&apos;");
  name.replace("\r", " ");
  name.replace("\n", " ");
  name.trim();
  if (name.isEmpty()) name = "Track";
  return name;
}

static String cleanTrackNameForFilename(String name, uint8_t index) {
  name.trim();
  if (name.isEmpty()) name = "Track_" + String(index + 1);

  String out;
  out.reserve(36);
  for (int i = 0; i < name.length() && out.length() < 32; i++) {
    char c = name[i];
    const bool invalid = c == '/' || c == '\\' || c == ':' || c == '*' ||
                         c == '?' || c == '"' || c == '<' || c == '>' ||
                         c == '|' || c == '\r' || c == '\n';
    out += invalid ? '_' : c;
  }

  out.trim();
  while (out.indexOf("__") >= 0) out.replace("__", "_");
  if (out.isEmpty()) out = "Track_" + String(index + 1);
  return out;
}

static String trackGpxPath(uint8_t index) {
  String path = String(APP_TRACKS_DIR) + "/";
  path += cleanTrackNameForFilename(g_tracks[index].name, index);
  path += ".gpx";
  return path;
}

static String trackDirEntryPath(const String &name) {
  if (name.startsWith("/")) return name;
  return String(APP_TRACKS_DIR) + "/" + name;
}

static String trackNameFromPath(String path) {
  int slash = path.lastIndexOf('/');
  if (slash >= 0) path = path.substring(slash + 1);
  int dot = path.lastIndexOf('.');
  if (dot > 0) path = path.substring(0, dot);
  path.trim();
  return path;
}

static String xmlUnescape(String value) {
  value.replace("&lt;", "<");
  value.replace("&gt;", ">");
  value.replace("&quot;", "\"");
  value.replace("&apos;", "'");
  value.replace("&amp;", "&");
  value.trim();
  return value;
}

static String xmlElementText(const String &line, const char *tag) {
  String open = "<";
  open += tag;
  open += ">";
  String close = "</";
  close += tag;
  close += ">";
  int start = line.indexOf(open);
  if (start < 0) return "";
  start += open.length();
  int end = line.indexOf(close, start);
  if (end < 0) return "";
  return xmlUnescape(line.substring(start, end));
}

static String xmlAttributeValue(const String &line, const char *attr) {
  String key = String(attr) + "=\"";
  int start = line.indexOf(key);
  if (start < 0) return "";
  start += key.length();
  int end = line.indexOf('"', start);
  if (end < 0) return "";
  return line.substring(start, end);
}

static String trackFileField(const String &line, int wanted) {
  int field = 0;
  int start = 0;
  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line[i] == '|') {
      if (field == wanted) return line.substring(start, i);
      field++;
      start = i + 1;
    }
  }
  return "";
}

static bool loadLegacyTracksFromOpenFile(File &f,
                                         uint8_t &loadedTracks,
                                         uint32_t &loadedPoints) {
  int8_t current = -1;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line == "TRACKS1") continue;

    String type = trackFileField(line, 0);

    if (type == "T") {
      current = -1;
      if (loadedTracks >= APP_MAX_TRACKS) continue;

      uint8_t slot = loadedTracks++;
      resetTrackSlot(slot);
      if (!allocateTrackPoints(g_tracks[slot])) {
        current = -1;
        continue;
      }

      g_tracks[slot].used = true;
      g_tracks[slot].visible = trackFileField(line, 1).toInt() != 0;
      g_tracks[slot].recording = false;
      g_tracks[slot].color = (uint16_t)trackFileField(line, 2).toInt();
      g_tracks[slot].name = trackFileField(line, 3);
      if (g_tracks[slot].name.isEmpty()) g_tracks[slot].name = "Track " + String(slot + 1);
      current = slot;
      g_selectedTrackIndex = slot;
    } else if (type == "P" && current >= 0) {
      MapTrack &track = g_tracks[current];
      if (!track.points || track.pointCount >= track.pointCapacity) continue;

      TrackPoint &point = track.points[track.pointCount++];
      point.lat = trackFileField(line, 1).toFloat();
      point.lon = trackFileField(line, 2).toFloat();
      point.timeMs = (uint32_t)trackFileField(line, 3).toInt();
      loadedPoints++;
    } else if (type == "E") {
      current = -1;
    }
  }

  return loadedTracks > 0;
}

static bool writeSingleTrackGpx(File &f, const MapTrack &track) {
  f.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  f.println("<gpx version=\"1.1\" creator=\"TomsLoRaSOS\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:pager=\"https://example.invalid/toms-lora-sos\">");
  f.println("  <trk>");
  f.print("    <name>");
  f.print(cleanTrackNameForStorage(track.name));
  f.println("</name>");
  f.println("    <extensions>");
  f.print("      <pager:visible>");
  f.print(track.visible ? 1 : 0);
  f.println("</pager:visible>");
  f.print("      <pager:color>");
  f.print((unsigned int)track.color);
  f.println("</pager:color>");
  f.println("    </extensions>");
  f.println("    <trkseg>");

  for (uint32_t p = 0; p < track.pointCount; p++) {
    const TrackPoint &point = track.points[p];
    f.print("      <trkpt lat=\"");
    f.print(point.lat, 6);
    f.print("\" lon=\"");
    f.print(point.lon, 6);
    f.println("\">");
    f.print("        <extensions><pager:ms>");
    f.print((unsigned long)point.timeMs);
    f.println("</pager:ms></extensions>");
    f.println("      </trkpt>");
  }

  f.println("    </trkseg>");
  f.println("  </trk>");
  f.println("</gpx>");
  return true;
}

static bool loadSingleGpxTrackFromOpenFile(File &f,
                                           uint8_t slot,
                                           const String &fileTrackName,
                                           uint32_t &loadedPoints) {
  if (slot >= APP_MAX_TRACKS) return false;

  resetTrackSlot(slot);
  if (!allocateTrackPoints(g_tracks[slot])) return false;

  MapTrack &track = g_tracks[slot];
  track.used = true;
  track.visible = true;
  track.recording = false;
  track.color = trackColorByIndex(slot);
  track.name = fileTrackName.isEmpty() ? "Track " + String(slot + 1) : fileTrackName;

  bool inTrack = false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    if (line.startsWith("<trk>")) {
      inTrack = true;
    } else if (line.startsWith("</trk>")) {
      inTrack = false;
    } else if (inTrack && line.startsWith("<name>")) {
      if (fileTrackName.isEmpty()) {
        String name = xmlElementText(line, "name");
        if (!name.isEmpty()) track.name = name;
      }
    } else if (inTrack && line.indexOf("<pager:visible>") >= 0) {
      track.visible = xmlElementText(line, "pager:visible").toInt() != 0;
    } else if (inTrack && line.indexOf("<pager:color>") >= 0) {
      track.color = (uint16_t)xmlElementText(line, "pager:color").toInt();
    } else if (inTrack && line.startsWith("<trkpt ")) {
      if (track.pointCount >= track.pointCapacity) continue;

      String lat = xmlAttributeValue(line, "lat");
      String lon = xmlAttributeValue(line, "lon");
      if (lat.isEmpty() || lon.isEmpty()) continue;

      TrackPoint &point = track.points[track.pointCount++];
      point.lat = lat.toFloat();
      point.lon = lon.toFloat();
      point.timeMs = 0;
      loadedPoints++;
    } else if (inTrack && line.indexOf("<pager:ms>") >= 0) {
      if (track.pointCount > 0) {
        track.points[track.pointCount - 1].timeMs =
            (uint32_t)xmlElementText(line, "pager:ms").toInt();
      }
    }
  }

  if (track.pointCount == 0) {
    resetTrackSlot(slot);
    return false;
  }

  g_selectedTrackIndex = slot;
  return true;
}

static bool saveTracksToSd() {
  if (g_tileQueueCount > 0 ||
      (g_tileRequestQueue && uxQueueMessagesWaiting(g_tileRequestQueue) > 0)) {
    uiToast("Tracks: bitte warten, Tiles laufen", 3000);
    return false;
  }

  if (!ensureSdReady(true)) {
    uiToast("Tracks: SD nicht bereit", 2500);
    return false;
  }

  instance.lockSPI();

  if (!ensureSdDirectory(APP_TRACKS_DIR)) {
    instance.unlockSPI();
    uiToast("Tracks: Ordner fehlt", 2500);
    return false;
  }

  uint8_t savedTracks = 0;
  uint32_t savedPoints = 0;

  for (uint8_t i = 0; i < APP_MAX_TRACKS; i++) {
    if (!g_tracks[i].used || !g_tracks[i].points) continue;

    String path = trackGpxPath(i);
    SD.remove(path);
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
      Serial.printf("[TRACK] save failed path=%s\n", path.c_str());
      continue;
    }

    writeSingleTrackGpx(f, g_tracks[i]);
    f.flush();
    f.close();
    savedTracks++;
    savedPoints += g_tracks[i].pointCount;
    Serial.printf("[TRACK] saved path=%s points=%u\n",
                  path.c_str(),
                  (unsigned int)g_tracks[i].pointCount);
  }

  instance.unlockSPI();

  Serial.printf("[TRACK] saved tracks=%u points=%u path=%s\n",
                (unsigned int)savedTracks,
                (unsigned int)savedPoints,
                APP_TRACKS_DIR);
  uiToast("Tracks gespeichert: " + String(savedTracks), 2500);
  return true;
}

static bool loadTracksFromSd() {
  if (!ensureSdReady(false)) {
    Serial.println("[TRACK] load skipped: SD not ready");
    return false;
  }

  instance.lockSPI();

  for (uint8_t i = 0; i < APP_MAX_TRACKS; i++) resetTrackSlot(i);
  g_selectedTrackIndex = -1;
  g_recordingTrackIndex = -1;
  g_nextTrackColor = 0;

  uint8_t loadedTracks = 0;
  uint32_t loadedPoints = 0;

  File dir = SD.open(APP_TRACKS_DIR, FILE_READ);
  if (dir && dir.isDirectory()) {
    File entry = dir.openNextFile();
    while (entry && loadedTracks < APP_MAX_TRACKS) {
      String name = entry.name();
      bool isDir = entry.isDirectory();
      entry.close();

      String check = name;
      check.toLowerCase();
      if (!isDir && check.endsWith(".gpx") &&
          check != "tracks.gpx" && !check.endsWith("/tracks.gpx")) {
        String path = trackDirEntryPath(name);
        File f = SD.open(path, FILE_READ);
        if (f) {
          uint8_t slot = loadedTracks;
          String fileTrackName = trackNameFromPath(name);
          if (loadSingleGpxTrackFromOpenFile(f, slot, fileTrackName, loadedPoints)) {
            loadedTracks++;
            Serial.printf("[TRACK] loaded file=%s points=%u\n",
                          path.c_str(),
                          (unsigned int)g_tracks[slot].pointCount);
          }
          f.close();
        }
      }

      entry = dir.openNextFile();
    }
  }
  if (dir) dir.close();

  if (loadedTracks == 0) {
    File f = SD.open(APP_TRACKS_COMBINED_PATH, FILE_READ);
    bool legacy = false;

    if (!f) {
      f = SD.open(APP_TRACKS_LEGACY_PATH, FILE_READ);
      legacy = true;
    }

    if (f) {
      if (legacy) {
        loadLegacyTracksFromOpenFile(f, loadedTracks, loadedPoints);
      } else {
        int8_t current = -1;
        bool inTrack = false;

        while (f.available()) {
          String line = f.readStringUntil('\n');
          line.trim();
          if (line.isEmpty()) continue;

          if (line == "TRACKS1" || line.startsWith("T|")) {
            f.seek(0);
            loadLegacyTracksFromOpenFile(f, loadedTracks, loadedPoints);
            break;
          }

          if (line.startsWith("<trk>")) {
            current = -1;
            inTrack = true;
            if (loadedTracks >= APP_MAX_TRACKS) continue;

            uint8_t slot = loadedTracks++;
            resetTrackSlot(slot);
            if (!allocateTrackPoints(g_tracks[slot])) continue;

            g_tracks[slot].used = true;
            g_tracks[slot].visible = true;
            g_tracks[slot].recording = false;
            g_tracks[slot].color = trackColorByIndex(slot);
            g_tracks[slot].name = "Track " + String(slot + 1);
            current = slot;
            g_selectedTrackIndex = slot;
          } else if (line.startsWith("</trk>")) {
            current = -1;
            inTrack = false;
          } else if (inTrack && current >= 0 && line.startsWith("<name>")) {
            String name = xmlElementText(line, "name");
            if (!name.isEmpty()) g_tracks[current].name = name;
          } else if (inTrack && current >= 0 && line.indexOf("<pager:visible>") >= 0) {
            g_tracks[current].visible = xmlElementText(line, "pager:visible").toInt() != 0;
          } else if (inTrack && current >= 0 && line.indexOf("<pager:color>") >= 0) {
            g_tracks[current].color = (uint16_t)xmlElementText(line, "pager:color").toInt();
          } else if (inTrack && current >= 0 && line.startsWith("<trkpt ")) {
            MapTrack &track = g_tracks[current];
            if (!track.points || track.pointCount >= track.pointCapacity) continue;

            String lat = xmlAttributeValue(line, "lat");
            String lon = xmlAttributeValue(line, "lon");
            if (lat.isEmpty() || lon.isEmpty()) continue;

            TrackPoint &point = track.points[track.pointCount++];
            point.lat = lat.toFloat();
            point.lon = lon.toFloat();
            point.timeMs = 0;
            loadedPoints++;
          } else if (inTrack && current >= 0 && line.indexOf("<pager:ms>") >= 0) {
            MapTrack &track = g_tracks[current];
            if (track.pointCount > 0) {
              track.points[track.pointCount - 1].timeMs =
                  (uint32_t)xmlElementText(line, "pager:ms").toInt();
            }
          }
        }
      }
      f.close();
    }
  }

  instance.unlockSPI();

  g_nextTrackColor = loadedTracks;
  Serial.printf("[TRACK] loaded tracks=%u points=%u path=%s\n",
                (unsigned int)loadedTracks,
                (unsigned int)loadedPoints,
                APP_TRACKS_DIR);

  scheduleMapRedraw();
  return loadedTracks > 0;
}

static void hideSafetyPopup() {
  if (g_safetyPopup) lv_obj_add_flag(g_safetyPopup, LV_OBJ_FLAG_HIDDEN);
}

static void showSafetyPopup(uint32_t remainingSeconds) {
  wakeDisplay();
  lv_obj_clear_flag(g_safetyPopup, LV_OBJ_FLAG_HIDDEN);
  String text = "KEINE BEWEGUNG ERKANNT\nSOS in " + String(remainingSeconds) +
                " Sekunden\n\n#FFD000 Enter:#Ich bin OK";
  lv_label_set_text(g_safetyLabel, text.c_str());
}

static void hidePowerPopup() {
  if (g_powerPopup) lv_obj_add_flag(g_powerPopup, LV_OBJ_FLAG_HIDDEN);
}

static void showPowerPopup() {
  wakeDisplay();
  lv_obj_clear_flag(g_powerPopup, LV_OBJ_FLAG_HIDDEN);
}

static void shutdownPager() {
  Serial.println("[POWER] shutdown requested");
  if (g_powerLabel) {
    lv_label_set_text(g_powerLabel, "SCHALTE AUS ...\n\nUSB trennen, falls das Geraet\nnicht ausgeht.");
    lv_obj_center(g_powerLabel);
  }
  lv_timer_handler();
  delay(250);

  saveTracksToSd();
  WiFi.disconnect(true);
  hw_set_kb_backlight(0);
  hw_set_disp_backlight(0);
  hw_shutdown();

  while (true) delay(1000);
}

static void setupUi() {
  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

  const size_t mapBytes = APP_DISPLAY_WIDTH * APP_DISPLAY_HEIGHT * sizeof(uint16_t);
  g_mapBuffer = static_cast<uint16_t *>(ps_malloc(mapBytes));
  if (!g_mapBuffer) {
    Serial.println("PSRAM allocation failed; trying normal heap.");
    g_mapBuffer = static_cast<uint16_t *>(malloc(mapBytes));
  }

  if (!g_mapBuffer) {
    Serial.println("FATAL: map canvas allocation failed.");
    lv_obj_t *fatal = lv_label_create(screen);
    lv_label_set_text(fatal, "STARTFEHLER\nKein RAM fuer Kartenpuffer");
    lv_obj_center(fatal);
    lv_obj_set_style_text_color(fatal, lv_color_white(), 0);
    lv_obj_set_style_text_align(fatal, LV_TEXT_ALIGN_CENTER, 0);
    return;
  }

  fillMap(RGB565(38, 45, 50));

  g_canvas = lv_canvas_create(screen);
  lv_canvas_set_buffer(g_canvas, g_mapBuffer, APP_DISPLAY_WIDTH, APP_DISPLAY_HEIGHT, LV_COLOR_FORMAT_RGB565);
  lv_obj_set_pos(g_canvas, 0, 0);

  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    g_peerSosRings[i] = lv_obj_create(screen);
    lv_obj_remove_style_all(g_peerSosRings[i]);
    lv_obj_set_size(g_peerSosRings[i], 22, 22);
    lv_obj_set_style_radius(g_peerSosRings[i], LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(g_peerSosRings[i], LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_peerSosRings[i], 3, 0);
    lv_obj_set_style_border_color(g_peerSosRings[i], lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_border_opa(g_peerSosRings[i], LV_OPA_80, 0);
    lv_obj_add_flag(g_peerSosRings[i], LV_OBJ_FLAG_HIDDEN);

    g_peerLabels[i] = lv_label_create(screen);
    lv_obj_set_width(g_peerLabels[i], LV_SIZE_CONTENT);
    lv_label_set_long_mode(g_peerLabels[i], LV_LABEL_LONG_CLIP);
    lv_obj_set_style_pad_hor(g_peerLabels[i], 2, 0);
    lv_obj_set_style_pad_ver(g_peerLabels[i], 1, 0);
    lv_obj_set_style_text_color(g_peerLabels[i], lv_color_hex(0xD6ECFF), 0);
    lv_obj_set_style_bg_color(g_peerLabels[i], lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_peerLabels[i], LV_OPA_70, 0);
    lv_obj_add_flag(g_peerLabels[i], LV_OBJ_FLAG_HIDDEN);
  }

  g_status = lv_label_create(screen);
  lv_obj_set_pos(g_status, 3, 2);
  lv_obj_set_style_bg_color(g_status, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(g_status, LV_OPA_70, 0);
  lv_obj_set_style_text_color(g_status, lv_color_white(), 0);
  lv_obj_set_style_pad_all(g_status, 2, 0);

  g_footer = lv_label_create(screen);
  lv_obj_align(g_footer, LV_ALIGN_BOTTOM_LEFT, 3, -2);
  lv_obj_set_style_bg_color(g_footer, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(g_footer, LV_OPA_70, 0);
  lv_obj_set_style_text_color(g_footer, lv_color_white(), 0);
  lv_obj_set_style_pad_all(g_footer, 2, 0);
  lv_obj_set_width(g_footer, APP_DISPLAY_WIDTH - 6);
  lv_label_set_long_mode(g_footer, LV_LABEL_LONG_CLIP);
  enableRecolor(g_footer);

  g_messagesPopup = lv_obj_create(lv_layer_top());
  lv_obj_set_size(g_messagesPopup, 448, 194);
  lv_obj_center(g_messagesPopup);
  stylePopupShell(g_messagesPopup, 0x15212A);
  lv_obj_add_flag(g_messagesPopup, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *title = makeBoundedLabel(g_messagesPopup,
                                     "Nachrichten  #FFD000 Rad:#Scroll  #FFD000 Zurueck:#Zu",
                                     14, 1, 420);
  enableRecolor(title);
  lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
  (void)title;

  g_messagesList = lv_obj_create(g_messagesPopup);
  lv_obj_set_pos(g_messagesList, 14, 22);
  lv_obj_set_size(g_messagesList, 420, 102);
  lv_obj_set_flex_flow(g_messagesList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(g_messagesList, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(g_messagesList, LV_SCROLLBAR_MODE_ON);
  lv_obj_set_style_bg_color(g_messagesList, lv_color_hex(0x0D141A), 0);
  lv_obj_set_style_pad_all(g_messagesList, 4, 0);

  g_messagesEscTop = makeLabel(g_messagesPopup, "Zurueck", 192, 15);
  g_messagesEscBottom = makeLabel(g_messagesPopup, "Zurueck", 192, 117);
  styleEdgeEscLabel(g_messagesEscTop, false);
  styleEdgeEscLabel(g_messagesEscBottom, false);

  g_textarea = lv_textarea_create(g_messagesPopup);
  lv_obj_set_pos(g_textarea, 14, 134);
  lv_obj_set_size(g_textarea, 420, 36);
  lv_textarea_set_one_line(g_textarea, true);
  lv_textarea_set_max_length(g_textarea, 180);
  lv_textarea_set_placeholder_text(g_textarea, "Nachricht eingeben; Enter sendet");

  g_tracksPopup = lv_obj_create(lv_layer_top());
  lv_obj_set_size(g_tracksPopup, 448, 194);
  lv_obj_center(g_tracksPopup);
  stylePopupShell(g_tracksPopup, 0x15212A);
  lv_obj_add_flag(g_tracksPopup, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *tracksTitle = makeBoundedLabel(
      g_tracksPopup,
      "Tracks #FFD000 Rad:#Wahl #FFD000 Druck/V:#Sicht #FFD000 R:#Rec #FFD000 S:#SD",
      6, 1, 436);
  enableRecolor(tracksTitle);

  g_tracksList = lv_obj_create(g_tracksPopup);
  lv_obj_set_pos(g_tracksList, 5, 22);
  lv_obj_set_size(g_tracksList, 420, 94);
  lv_obj_set_flex_flow(g_tracksList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(g_tracksList, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(g_tracksList, LV_SCROLLBAR_MODE_ON);
  lv_obj_set_style_bg_color(g_tracksList, lv_color_hex(0x0D141A), 0);
  lv_obj_set_style_pad_all(g_tracksList, 4, 0);

  g_tracksEscTop = makeLabel(g_tracksPopup, "Zurueck", 192, 15);
  g_tracksEscBottom = makeLabel(g_tracksPopup, "Zurueck", 192, 109);
  styleEdgeEscLabel(g_tracksEscTop, false);
  styleEdgeEscLabel(g_tracksEscBottom, false);

  g_trackNameArea = lv_textarea_create(g_tracksPopup);
  lv_obj_set_pos(g_trackNameArea, 5, 126);
  lv_obj_set_size(g_trackNameArea, 420, 36);
  lv_textarea_set_one_line(g_trackNameArea, true);
  lv_textarea_set_max_length(g_trackNameArea, 32);
  lv_textarea_set_placeholder_text(g_trackNameArea, "Neuer Trackname");

  lv_obj_t *tracksHint = makeBoundedLabel(
      g_tracksPopup,
      "Name+#FFD000 Enter:#Neu. Leer: #FFD000 Rad/R/V/S:#Bedienen. #FFD000 Zurueck:#Zu.",
      7, 166, 434);
  enableRecolor(tracksHint);

  g_safetyPopup = lv_obj_create(lv_layer_top());
  lv_obj_set_size(g_safetyPopup, 400, 155);
  lv_obj_center(g_safetyPopup);
  stylePopupShell(g_safetyPopup, 0x8B0000);
  lv_obj_add_flag(g_safetyPopup, LV_OBJ_FLAG_HIDDEN);

  g_safetyLabel = lv_label_create(g_safetyPopup);
  enableRecolor(g_safetyLabel);
  lv_obj_center(g_safetyLabel);
  lv_obj_set_style_text_align(g_safetyLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(g_safetyLabel, lv_color_white(), 0);

  g_powerPopup = lv_obj_create(lv_layer_top());
  lv_obj_set_size(g_powerPopup, 360, 120);
  lv_obj_center(g_powerPopup);
  stylePopupShell(g_powerPopup, 0x202020);
  lv_obj_add_flag(g_powerPopup, LV_OBJ_FLAG_HIDDEN);

  g_powerLabel = lv_label_create(g_powerPopup);
  lv_label_set_text(g_powerLabel, "Ausschalten?\n\n#FFD000 Enter:#Ja\n#FFD000 Zurueck:#Nein");
  enableRecolor(g_powerLabel);
  lv_obj_set_width(g_powerLabel, 340);
  lv_label_set_long_mode(g_powerLabel, LV_LABEL_LONG_WRAP);
  lv_obj_center(g_powerLabel);
  lv_obj_set_style_text_align(g_powerLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(g_powerLabel, lv_color_white(), 0);

  g_wifiPopup = lv_obj_create(lv_layer_top());
  lv_obj_set_size(g_wifiPopup, 448, 214);
  lv_obj_center(g_wifiPopup);
  stylePopupShell(g_wifiPopup, 0x15212A);
  lv_obj_add_flag(g_wifiPopup, LV_OBJ_FLAG_HIDDEN);

  makeBoundedLabel(g_wifiPopup, "WLAN-Einstellungen", 7, 2, 260);
  g_wifiStateLabel = makeBoundedLabel(g_wifiPopup, "WLAN: AUS", 272, 2, 160);
  lv_obj_set_style_text_align(g_wifiStateLabel, LV_TEXT_ALIGN_RIGHT, 0);
  makeLabel(g_wifiPopup, "Name:", 7, 27);
  makeLabel(g_wifiPopup, "SSID:", 7, 67);
  makeLabel(g_wifiPopup, "Passwort:", 7, 107);

  g_wifiNameArea = lv_textarea_create(g_wifiPopup);
  lv_obj_set_pos(g_wifiNameArea, 86, 19);
  lv_obj_set_size(g_wifiNameArea, 337, 32);
  lv_textarea_set_one_line(g_wifiNameArea, true);
  lv_textarea_set_max_length(g_wifiNameArea, 24);

  g_wifiSsidArea = lv_textarea_create(g_wifiPopup);
  lv_obj_set_pos(g_wifiSsidArea, 86, 59);
  lv_obj_set_size(g_wifiSsidArea, 337, 32);
  lv_textarea_set_one_line(g_wifiSsidArea, true);
  lv_textarea_set_max_length(g_wifiSsidArea, 64);

  g_wifiPasswordArea = lv_textarea_create(g_wifiPopup);
  lv_obj_set_pos(g_wifiPasswordArea, 86, 99);
  lv_obj_set_size(g_wifiPasswordArea, 337, 32);
  lv_textarea_set_one_line(g_wifiPasswordArea, true);
  lv_textarea_set_max_length(g_wifiPasswordArea, 64);
  lv_textarea_set_password_mode(g_wifiPasswordArea, true);

  makeLabel(g_wifiPopup, "WLAN:", 7, 143);
  g_wifiToggleButton = lv_obj_create(g_wifiPopup);
  lv_obj_set_pos(g_wifiToggleButton, 86, 135);
  lv_obj_set_size(g_wifiToggleButton, 337, 30);
  lv_obj_clear_flag(g_wifiToggleButton, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(g_wifiToggleButton, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_color(g_wifiToggleButton, lv_color_hex(0x223445), 0);
  lv_obj_set_style_bg_opa(g_wifiToggleButton, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_wifiToggleButton, 1, 0);
  lv_obj_set_style_border_color(g_wifiToggleButton, lv_color_hex(0x5A7C99), 0);
  lv_obj_set_style_border_width(g_wifiToggleButton, 2, LV_STATE_FOCUSED);
  lv_obj_set_style_border_color(g_wifiToggleButton, lv_color_hex(0xFFD000), LV_STATE_FOCUSED);
  lv_obj_set_style_radius(g_wifiToggleButton, 4, 0);
  g_wifiToggleLabel = lv_label_create(g_wifiToggleButton);
  lv_obj_center(g_wifiToggleLabel);
  lv_obj_set_style_text_color(g_wifiToggleLabel, lv_color_white(), 0);

  g_wifiHint = makeLabel(
      g_wifiPopup,
      "#FFD000 Druck:#Feld  #FFD000 Enter:#Weiter/speichern/AN-AUS  #FFD000 Q/Esc:#Schliessen",
      7, 178);
  enableRecolor(g_wifiHint);
  lv_obj_set_width(g_wifiHint, 434);
  lv_label_set_long_mode(g_wifiHint, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_color(g_wifiHint, lv_color_hex(0xD0E8FF), 0);

  g_bootLabel = lv_label_create(screen);
  lv_label_set_text(g_bootLabel, "Outdoor Pager\nStarte GPS, LoRa und Karte ...\n#FFD000 W:#WLAN einrichten");
  enableRecolor(g_bootLabel);
  lv_obj_center(g_bootLabel);
  lv_obj_set_style_text_color(g_bootLabel, lv_color_white(), 0);
  lv_obj_set_style_text_align(g_bootLabel, LV_TEXT_ALIGN_CENTER, 0);

  refreshStatus();
}

// -----------------------------------------------------------------------------
// Radio
// -----------------------------------------------------------------------------

static bool sendRadio(const String &packet) {
  if (packet.length() == 0 || packet.length() > APP_MAX_RADIO_PAYLOAD) return false;

  Serial.printf("[LORA TX] %s\n", packet.c_str());
  g_lastRadioTxOk = radio_transmit(
      reinterpret_cast<const uint8_t *>(packet.c_str()), packet.length());
  Serial.printf("[LORA TX] result=%s\n", g_lastRadioTxOk ? "ok" : "failed");

  hw_set_radio_listening();
  refreshStatus();
  return g_lastRadioTxOk;
}

static OutdoorPeer *findOrCreatePeer(const String &id, const String &name) {
  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (g_peers[i].used && g_peers[i].id == id) return &g_peers[i];
  }
  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (!g_peers[i].used) {
      g_peers[i].used = true;
      g_peers[i].id = id;
      g_peers[i].name = name;
      return &g_peers[i];
    }
  }
  return nullptr;
}

static void clearPeerSlot(uint8_t index) {
  if (index >= APP_MAX_PEERS) return;
  g_peers[index] = OutdoorPeer {};
  if (g_peerLabels[index]) lv_obj_add_flag(g_peerLabels[index], LV_OBJ_FLAG_HIDDEN);
  if (g_peerSosRings[index]) lv_obj_add_flag(g_peerSosRings[index], LV_OBJ_FLAG_HIDDEN);
}

static void removeOwnPeerGhosts(const String &id, const String &name) {
  String ownId = sanitizeRadioField(g_cfg.deviceId, 24, "pager");
  String ownName = sanitizeRadioField(g_cfg.deviceName, 18, "PAGER");

  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (!g_peers[i].used) continue;
    if (g_peers[i].id == ownId || g_peers[i].id == id ||
        (!name.isEmpty() && g_peers[i].sos && g_peers[i].name == name && name == ownName)) {
      clearPeerSlot(i);
    }
  }
}

static void clearPeerSosByName(const String &name) {
  if (name.isEmpty()) return;
  for (uint8_t i = 0; i < APP_MAX_PEERS; i++) {
    if (g_peers[i].used && g_peers[i].name == name) {
      g_peers[i].sos = false;
      if (g_peerSosRings[i]) lv_obj_add_flag(g_peerSosRings[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

static String fieldAt(const String &packet, int wanted) {
  int field = 0;
  int start = 0;
  for (int i = 0; i <= packet.length(); i++) {
    if (i == packet.length() || packet[i] == '|') {
      if (field == wanted) return packet.substring(start, i);
      field++;
      start = i + 1;
    }
  }
  return "";
}

static void sendPosition() {
  if (!g_hasGps) return;
  String packet = "POS|" + sanitizeRadioField(g_cfg.deviceId, 24, "pager") + "|" +
                  sanitizeRadioField(g_cfg.deviceName, 18, "PAGER") + "|" +
                  String(g_gps.lat, 6) + "|" + String(g_gps.lng, 6) + "|" +
                  String(batteryMv()) + "|" + String(g_sequence++);
  sendRadio(packet);
  g_lastPosTxMs = nowMs();
}

static void sendTextMessage(const String &text) {
  String clean = text;
  clean.replace("|", "/");
  clean.trim();
  if (clean.isEmpty()) return;

  String packet = "MSG|" + sanitizeRadioField(g_cfg.deviceId, 24, "pager") + "|" +
                  sanitizeRadioField(g_cfg.deviceName, 18, "PAGER") + "|" + clean;
  sendRadio(packet);
  addChatMessage(sanitizeRadioField(g_cfg.deviceName, 18, "PAGER"), clean, true);
}

static void sendSos() {
  String lat = g_hasGps ? String(g_gps.lat, 6) : "0";
  String lon = g_hasGps ? String(g_gps.lng, 6) : "0";
  String packet = "SOS|" + sanitizeRadioField(g_cfg.deviceId, 24, "pager") + "|" +
                  sanitizeRadioField(g_cfg.deviceName, 18, "PAGER") + "|" +
                  lat + "|" + lon + "|" + String(batteryMv()) +
                  "|NO_MOTION|" + String(g_sequence++);
  sendRadio(packet);
  g_lastSosTxMs = nowMs();
}

static void processRadioPacket(const String &packet) {
  String type = fieldAt(packet, 0);
  String id = fieldAt(packet, 1);
  String name = fieldAt(packet, 2);

  if (id.isEmpty()) return;
  if (id == sanitizeRadioField(g_cfg.deviceId, 24, "pager") ||
      name == sanitizeRadioField(g_cfg.deviceName, 18, "PAGER")) {
    removeOwnPeerGhosts(id, name);
    scheduleMapRedraw();
    return;
  }

  if (type == "POS") {
    OutdoorPeer *peer = findOrCreatePeer(id, name);
    if (!peer) return;
    peer->name = name;
    peer->lat = fieldAt(packet, 3).toDouble();
    peer->lon = fieldAt(packet, 4).toDouble();
    peer->batteryMv = fieldAt(packet, 5).toInt();
    peer->lastSeenMs = nowMs();
    scheduleMapRedraw();
    renderMapIfDue();
  } else if (type == "MSG") {
    String text = fieldAt(packet, 3);
    addChatMessage(name, text, false);
    wakeDisplay();
    uiToast("MSG " + name + ": " + text, 5000);
  } else if (type == "SOS") {
    OutdoorPeer *peer = findOrCreatePeer(id, name);
    if (peer) {
      peer->name = name;
      peer->lat = fieldAt(packet, 3).toDouble();
      peer->lon = fieldAt(packet, 4).toDouble();
      peer->batteryMv = fieldAt(packet, 5).toInt();
      peer->lastSeenMs = nowMs();
      peer->sos = true;
    }
    wakeDisplay();
    ui_msg_pop_up("SOS", (name + " reagiert nicht. Position auf Karte markiert.").c_str());
    hw_feedback();
    triggerRemoteSosAlarm();
    scheduleMapRedraw();

    String ack = "ACK|" + sanitizeRadioField(g_cfg.deviceId, 24, "pager") + "|" +
                 sanitizeRadioField(g_cfg.deviceName, 18, "PAGER") + "|" + fieldAt(packet, 7);
    sendRadio(ack);
  } else if (type == "SAFE") {
    OutdoorPeer *peer = findOrCreatePeer(id, name);
    if (peer) {
      peer->name = name;
      peer->lastSeenMs = nowMs();
      peer->sos = false;
    }
    clearPeerSosByName(name);
    uiToast(name + ": Entwarnung");
    scheduleMapRedraw();
  } else if (type == "ACK") {
    uiToast("SOS empfangen von " + name, 5000);
  }
}

static void pollRadio() {
  if (nowMs() - g_lastRadioRefreshMs < APP_RADIO_REFRESH_MS) return;
  g_lastRadioRefreshMs = nowMs();

  radio_rx_params_t rx {};
  rx.data = g_radioRxBuf;
  rx.length = APP_MAX_RADIO_PAYLOAD;
  hw_get_radio_rx(rx);

  if (rx.state != 0 || rx.length == 0) return;
  if (rx.length > APP_MAX_RADIO_PAYLOAD) rx.length = APP_MAX_RADIO_PAYLOAD;

  g_radioRxBuf[rx.length] = '\0';
  String packet(reinterpret_cast<char *>(g_radioRxBuf));
  Serial.printf("[LORA RX] %s\n", packet.c_str());
  processRadioPacket(packet);
}

// -----------------------------------------------------------------------------
// GPS, IMU, safety and pocket display mode
// -----------------------------------------------------------------------------

static void pollGps() {
  if (nowMs() - g_lastGpsRefreshMs < APP_GPS_REFRESH_MS) return;
  g_lastGpsRefreshMs = nowMs();

  gps_params_t gps {};
  bool gpsFix = hw_get_gps_info(gps);
  if (gpsFix || gps.rx_size > 0 || gps.satellite > 0 || !gps.model.empty()) {
    g_gpsRaw = gps;
    g_hasGpsInfo = true;
    refreshStatus();
  }

  if (gpsFix && gps.lat != 0.0 && gps.lng != 0.0) {
    bool moved = !g_hasGps || fabs(gps.lat - g_gps.lat) > 0.000002 || fabs(gps.lng - g_gps.lng) > 0.000002;
    g_gps = gps;
    g_hasGps = true;
    if (moved) {
      appendPointToRecordingTrack(gps.lat, gps.lng, gps.speed);
      scheduleMapRedraw();
    }
  }
}

static void dismissSafetyWarning() {
  if (g_safety == SAFETY_WARNING || g_safety == SAFETY_SOS) {
    bool hadSos = g_safety == SAFETY_SOS;
    g_safety = SAFETY_ARMED;
    g_safetyStateSinceMs = nowMs();
    g_lastMotionMs = nowMs();
    hideSafetyPopup();
    if (hadSos) {
      sendRadio("SAFE|" + sanitizeRadioField(g_cfg.deviceId, 24, "pager") + "|" +
                sanitizeRadioField(g_cfg.deviceName, 18, "PAGER") + "|" + String(g_sequence++));
      removeOwnPeerGhosts(sanitizeRadioField(g_cfg.deviceId, 24, "pager"),
                          sanitizeRadioField(g_cfg.deviceName, 18, "PAGER"));
      scheduleMapRedraw();
    }
    uiToast("Sicherheitswarnung quittiert");
  }
}

static void toggleSafety() {
  wakeDisplay();
  if (g_safety == SAFETY_OFF) {
    g_safety = SAFETY_ARMED;
    g_lastMotionMs = nowMs();
    g_safetyStateSinceMs = nowMs();
    uiToast("Totmannmodus aktiviert");
  } else {
    bool hadSos = g_safety == SAFETY_SOS;
    g_safety = SAFETY_OFF;
    hideSafetyPopup();
    if (hadSos) {
      sendRadio("SAFE|" + sanitizeRadioField(g_cfg.deviceId, 24, "pager") + "|" +
                sanitizeRadioField(g_cfg.deviceName, 18, "PAGER") + "|" + String(g_sequence++));
      removeOwnPeerGhosts(sanitizeRadioField(g_cfg.deviceId, 24, "pager"),
                          sanitizeRadioField(g_cfg.deviceName, 18, "PAGER"));
      scheduleMapRedraw();
    }
    uiToast("Totmannmodus deaktiviert");
  }
  refreshStatus();
}

static void pollImu() {
  if (nowMs() - g_lastImuRefreshMs < APP_IMU_REFRESH_MS) return;
  g_lastImuRefreshMs = nowMs();

  imu_params_t imu {};
  hw_get_imu_params(imu);
  g_imu = imu;
  g_hasImu = true;

  if (!isnan(g_prevRoll)) {
    float delta = angleDistance(imu.roll, g_prevRoll) +
                  angleDistance(imu.pitch, g_prevPitch) +
                  angleDistance(imu.heading, g_prevHeading);
    if (delta >= APP_IMU_MOTION_DELTA_DEG) {
      g_lastMotionMs = nowMs();
      if (!g_displayOn) setDisplay(true);
    }
  }

  g_prevRoll = imu.roll;
  g_prevPitch = imu.pitch;
  g_prevHeading = imu.heading;

  if (g_cfg.autoDisplay && g_safety != SAFETY_WARNING && g_safety != SAFETY_SOS) {
    bool quiet = nowMs() - g_lastMotionMs > APP_POCKET_QUIET_MS;
    bool pocketLike = fabsf(imu.pitch) > APP_POCKET_PITCH_DEG ||
                      fabsf(imu.roll) > APP_POCKET_ROLL_DEG;
    if (quiet && pocketLike && nowMs() - g_lastInputMs > APP_POCKET_QUIET_MS) {
      setDisplay(false);
    }
  }
}

static void pollSafety() {
  if (g_safety == SAFETY_OFF) return;

  uint32_t now = nowMs();

  if (g_safety == SAFETY_WARNING && g_lastMotionMs > g_safetyStateSinceMs) {
    g_safety = SAFETY_ARMED;
    g_safetyStateSinceMs = now;
    hideSafetyPopup();
    uiToast("Bewegung erkannt - SOS abgebrochen", 2200);
    refreshStatus();
    return;
  }

  if (g_safety == SAFETY_ARMED && now - g_lastMotionMs >= g_cfg.safetyNoMotionMs) {
    g_safety = SAFETY_WARNING;
    g_safetyStateSinceMs = now;
    hw_feedback();
    wakeDisplay();
  }

  if (g_safety == SAFETY_WARNING) {
    uint32_t elapsed = now - g_safetyStateSinceMs;
    if (elapsed >= g_cfg.safetyCountdownMs) {
      g_safety = SAFETY_SOS;
      g_safetyStateSinceMs = now;
      sendSos();
      hw_feedback();
    } else {
      uint32_t remaining = (g_cfg.safetyCountdownMs - elapsed + 999) / 1000;
      showSafetyPopup(remaining);
    }
  }

  if (g_safety == SAFETY_SOS) {
    showSafetyPopup(0);
    if (now - g_lastSosTxMs >= APP_SOS_REPEAT_MS) sendSos();
  }

  refreshStatus();
}

// -----------------------------------------------------------------------------
// Input
// -----------------------------------------------------------------------------

static bool messagesVisible() {
  return g_messagesPopup && !lv_obj_has_flag(g_messagesPopup, LV_OBJ_FLAG_HIDDEN);
}

static bool tracksVisible() {
  return g_tracksPopup && !lv_obj_has_flag(g_tracksPopup, LV_OBJ_FLAG_HIDDEN);
}

static bool powerVisible() {
  return g_powerPopup && !lv_obj_has_flag(g_powerPopup, LV_OBJ_FLAG_HIDDEN);
}

static void selectMessageStep(int direction) {
  if (g_messageListItemCount <= 0) return;

  int16_t next = g_selectedMessageListItem + direction;
  if (next < -1) next = -1;
  if (next > g_messageListItemCount) next = g_messageListItemCount;
  if (next == g_selectedMessageListItem) return;

  g_selectedMessageListItem = next;
  rebuildMessageList();
}

static bool selectedTrackItemIsExit() {
  return g_selectedTrackListItem < 0 ||
         g_selectedTrackListItem >= g_trackListItemCount;
}

static void sendTextarea() {
  const char *text = lv_textarea_get_text(g_textarea);
  if (text && text[0]) {
    sendTextMessage(String(text));
    lv_textarea_set_text(g_textarea, "");
  }
}

static void processKey(char c) {
  Serial.printf("[KEY] code=%u char=%c\n", (unsigned int)(uint8_t)c,
                (c >= 32 && c <= 126) ? c : '.');
  wakeDisplay();

  if (g_safety == SAFETY_WARNING || g_safety == SAFETY_SOS) {
    if (c == '\n' || c == '\r') dismissSafetyWarning();
    return;
  }

  if (powerVisible()) {
    if (c == '\n' || c == '\r') {
      shutdownPager();
    } else if (c == '\b' || c == 127 || c == 27 ||
               c == 'p' || c == 'P') {
      hidePowerPopup();
    }
    return;
  }

  if (wifiVisible()) {
    if (c == 27 || c == 'q' || c == 'Q') {
      hideWifiPopup();
    } else if (c == '\b' || c == 127) {
      lv_obj_t *area = activeWifiArea();
      const char *text = area ? lv_textarea_get_text(area) : "";
      if (area && text && text[0]) {
        lv_textarea_delete_char(area);
      } else if (g_wifiActiveField > 0) {
        focusWifiField(g_wifiActiveField - 1);
        uiToast("Vorheriges Feld", 1500);
      } else {
        hideWifiPopup();
      }
    } else if (c == '\n' || c == '\r') {
      if (g_wifiActiveField < 2) {
        focusWifiField(g_wifiActiveField + 1);
        uiToast(g_wifiActiveField == 1 ? "SSID eingeben" :
                                      "Passwort eingeben; Enter speichert", 2200);
      } else if (g_wifiActiveField == 2) {
        saveWifiAndConnect();
      } else {
        toggleWifiEnabledFromPopup();
        hideWifiPopup();
      }
    } else if (c == 'o' || c == 'O') {
      toggleWifiEnabledFromPopup();
      hideWifiPopup();
    } else if (c >= 32 && c <= 126) {
      lv_obj_t *area = activeWifiArea();
      if (area) lv_textarea_add_char(area, c);
    }
    return;
  }

  if (tracksVisible()) {
    const char *trackName = lv_textarea_get_text(g_trackNameArea);
    bool hasNameText = trackName && trackName[0];

    if (c == '\b' || c == 127 || c == 27) {
      if (hasNameText) {
        lv_textarea_delete_char(g_trackNameArea);
      } else {
        hideTracksPopup();
      }
    } else if (c == '\n' || c == '\r') {
      if (hasNameText) {
        int8_t created = createTrack(String(trackName));
        lv_textarea_set_text(g_trackNameArea, "");
        rebuildTracksList();
        if (created >= 0) selectTrackListItemForTrack(created);
        refreshStatus();
      } else if (selectedTrackItemIsExit()) {
        lv_group_focus_obj(g_trackNameArea);
      }
    } else if (!hasNameText && (c == 'r' || c == 'R')) {
      toggleTrackRecording();
    } else if (!hasNameText && (c == 's' || c == 'S')) {
      saveTracksToSd();
    } else if (!hasNameText && (c == 'v' || c == 'V')) {
      if (selectedTrackItemIsExit()) {
        lv_group_focus_obj(g_trackNameArea);
      } else {
        toggleSelectedTrackVisibility();
      }
    } else if (!hasNameText && c == '+') {
      selectTrackStep(+1);
    } else if (!hasNameText && c == '-') {
      selectTrackStep(-1);
    } else if (c >= 32 && c <= 126) {
      lv_textarea_add_char(g_trackNameArea, c);
    }
    return;
  }

  if (messagesVisible()) {
    if (c == '\b' || c == 127 || c == 27) {
      const char *text = lv_textarea_get_text(g_textarea);
      if (text && text[0]) {
        lv_textarea_delete_char(g_textarea);
      } else {
        hideMessagesPopup();
      }
    } else if (c == '\n' || c == '\r') {
      sendTextarea();
    } else if (c >= 32 && c <= 126) {
      lv_textarea_add_char(g_textarea, c);
    }
    return;
  }

  switch (c) {
    case 'm':
    case 'M':
      showMessagesPopup(false);
      break;
    case 'n':
    case 'N':
      showMessagesPopup(true);
      break;
    case 'k':
    case 'K':
      showTracksPopup();
      break;
    case 'w':
    case 'W':
      showWifiPopup();
      break;
    case 'p':
    case 'P':
      showPowerPopup();
      break;
    case 's':
    case 'S':
      toggleSafety();
      break;
    case 'c':
    case 'C':
      centerMapOnGps();
      break;
    case 'a':
    case 'A':
      toggleMapPanAxis();
      break;
    case 't':
    case 'T':
      drawCanvasTestPattern();
      break;
    case '1':
      setZoomIfAvailable(16);
      break;
    case '2':
      setZoomIfAvailable(17);
      break;
    case '3':
      setZoomIfAvailable(18);
      break;
    case '4':
      setZoomIfAvailable(19);
      break;
    case '+':
      stepZoomToAvailable(+1);
      break;
    case '-':
      stepZoomToAvailable(-1);
      break;
  }

  refreshStatus();
}


static bool wifiVisible() {
  return g_wifiPopup && !lv_obj_has_flag(g_wifiPopup, LV_OBJ_FLAG_HIDDEN);
}

static String wifiStateText() {
  if (!g_wifiEnabled) return "WLAN: AUS";
  if (WiFi.status() == WL_CONNECTED) return "WLAN: AN";
  return "WLAN: VERBINDET";
}

static void updateWifiPopupState() {
  if (g_wifiStateLabel) lv_label_set_text(g_wifiStateLabel, wifiStateText().c_str());
  if (g_wifiToggleLabel) {
    lv_label_set_text(g_wifiToggleLabel, g_wifiEnabled ? "WLAN ausschalten" : "WLAN einschalten");
  }
  if (g_wifiToggleButton) {
    lv_obj_set_style_bg_color(
        g_wifiToggleButton,
        lv_color_hex(g_wifiEnabled ? 0x285A38 : 0x3B2A2A),
        0);
  }
}

static void readWifiFieldsFromPopup() {
  if (!g_wifiPopup) return;

  g_cfg.deviceName = lv_textarea_get_text(g_wifiNameArea);
  g_cfg.deviceName = sanitizeRadioField(g_cfg.deviceName, 18, "PAGER");
  g_cfg.wifiSsid = lv_textarea_get_text(g_wifiSsidArea);
  g_cfg.wifiPassword = lv_textarea_get_text(g_wifiPasswordArea);
}

static lv_obj_t *activeWifiArea() {
  if (g_wifiActiveField == 0) return g_wifiNameArea;
  if (g_wifiActiveField == 1) return g_wifiSsidArea;
  if (g_wifiActiveField == 2) return g_wifiPasswordArea;
  return nullptr;
}

static lv_obj_t *activeWifiObject() {
  if (g_wifiActiveField == 3) return g_wifiToggleButton;
  return activeWifiArea();
}

static void focusWifiField(uint8_t field) {
  g_wifiActiveField = field > 3 ? 3 : field;
  lv_obj_t *obj = activeWifiObject();
  if (obj) lv_group_focus_obj(obj);
}

static void showWifiPopup() {
  wakeDisplay();
  lv_textarea_set_text(g_wifiNameArea, g_cfg.deviceName.c_str());
  lv_textarea_set_text(g_wifiSsidArea, g_cfg.wifiSsid.c_str());
  lv_textarea_set_text(g_wifiPasswordArea, g_cfg.wifiPassword.c_str());
  updateWifiPopupState();
  lv_obj_clear_flag(g_wifiPopup, LV_OBJ_FLAG_HIDDEN);
  focusWifiField(0);
}

static void hideWifiPopup() {
  if (g_wifiPopup) lv_obj_add_flag(g_wifiPopup, LV_OBJ_FLAG_HIDDEN);
}

static void disableWifi() {
  g_wifiEnabled = false;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  resetLookaheadWindow();
  refreshStatus();
  updateWifiPopupState();
}

static void connectWifi() {
  if (!g_wifiEnabled) {
    Serial.println("[WIFI] connect skipped: disabled");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  if (g_cfg.wifiSsid.isEmpty()) {
    Serial.println("[WIFI] connect skipped: SSID empty");
    return;
  }

  Serial.printf("[WIFI] connecting to ssid=%s\n", g_cfg.wifiSsid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.wifiSsid.c_str(), g_cfg.wifiPassword.c_str());
  refreshStatus();
  updateWifiPopupState();
}

static void toggleWifiEnabledFromPopup() {
  readWifiFieldsFromPopup();

  if (g_wifiEnabled) {
    disableWifi();
    uiToast("WLAN ausgeschaltet", 2200);
    return;
  }

  if (g_cfg.wifiSsid.isEmpty()) {
    uiToast("SSID fehlt", 2200);
    focusWifiField(1);
    return;
  }

  g_wifiEnabled = true;
  saveWifiCredentialsToNvs();
  connectWifi();
  uiToast("WLAN eingeschaltet", 2200);
}

static void saveWifiAndConnect() {
  readWifiFieldsFromPopup();

  bool storedNvs = saveWifiCredentialsToNvs();
  hideWifiPopup();

  if (g_wifiEnabled) connectWifi();
  else disableWifi();

  if (storedNvs) {
    uiToast(g_wifiEnabled ? "WLAN gespeichert; Verbindung startet" :
                            "WLAN gespeichert; bleibt aus", 4500);
  } else {
    uiToast(g_wifiEnabled ? "WLAN startet; Speichern fehlgeschlagen" :
                            "WLAN bleibt aus; Speichern fehlgeschlagen", 5000);
  }
}

// -----------------------------------------------------------------------------
// Rotary map control
// -----------------------------------------------------------------------------

static void pollRotary() {
  RotaryMsg_t msg = instance.getRotary();

  if (msg.centerBtnPressed) {
    uint32_t now = nowMs();

    if (now - g_lastRotaryCenterPressMs >= APP_ROTARY_PRESS_DEBOUNCE_MS) {
      g_lastRotaryCenterPressMs = now;
      wakeDisplay();

      if (wifiVisible()) {
        focusWifiField((g_wifiActiveField + 1) % 4);
      } else if (powerVisible()) {
        shutdownPager();
      } else if (tracksVisible()) {
        const char *trackName = lv_textarea_get_text(g_trackNameArea);
        if (trackName && trackName[0]) {
          int8_t created = createTrack(String(trackName));
          lv_textarea_set_text(g_trackNameArea, "");
          rebuildTracksList();
          if (created >= 0) selectTrackListItemForTrack(created);
          refreshStatus();
        } else if (selectedTrackItemIsExit()) {
          lv_group_focus_obj(g_trackNameArea);
        } else {
          toggleSelectedTrackVisibility();
        }
      } else if (messagesVisible()) {
        lv_group_focus_obj(g_textarea);
      } else if (!messagesVisible() &&
                 g_safety != SAFETY_WARNING && g_safety != SAFETY_SOS) {
        toggleMapPanAxis();
      }
    }
  }

  if (msg.dir == ROTARY_DIR_NONE) return;

  wakeDisplay();

  int rotaryStep = msg.dir == ROTARY_DIR_UP ? +1 : -1;

  if (tracksVisible()) {
    selectTrackStep(rotaryStep);
    return;
  }

  if (messagesVisible()) {
    selectMessageStep(rotaryStep);
    return;
  }

  // Keep dialogs stable while entering text or changing settings.
  if (wifiVisible() ||
      powerVisible() ||
      g_safety == SAFETY_WARNING || g_safety == SAFETY_SOS) {
    return;
  }

  panMapByRotaryStep(rotaryStep);
}

// -----------------------------------------------------------------------------
// WiFi
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Arduino lifecycle
// -----------------------------------------------------------------------------

void setup() {
  earlyBacklightOn();
  Serial.begin(115200);
  delay(300);
  printBootDiagnostics("setup entered");

  const uint32_t hwFlags = NO_SCAN_I2C_DEV | NO_HW_SENSOR | NO_INIT_FATFS |
                           NO_HW_RTC | NO_HW_NFC | NO_HW_GPS |
                           NO_HW_LORA | NO_HW_MIC | NO_HW_SD;
  uint32_t probe = instance.begin(hwFlags);
  Serial.printf("[BOOT] instance.begin probe=0x%08X\n", (unsigned int)probe);
  Serial.flush();
  if (!(probe & HW_PSRAM_ONLINE)) {
    Serial.println("LilyGoLib instance.begin() failed.");
    while (true) delay(1000);
  }

  beginLvglHelper(instance);
  instance.setBrightness(APP_DISPLAY_BRIGHTNESS);
  instance.enableRotary();
  instance.clearRotaryMsg();

  g_tileRequestQueue = xQueueCreate(APP_MAX_TILE_QUEUE, sizeof(TileJob));
  g_tileResultQueue = xQueueCreate(APP_TILE_RESULT_QUEUE_LENGTH, sizeof(TileResult));

  if (!initializeTileCache()) {
    Serial.println("[CACHE] initialization incomplete");
  }

  setupUi();
  uiToast("Outdoor Pager startet - Hardware wird initialisiert", 4500);
  lv_timer_handler();
  delay(40);

  Serial.println("[BOOT] starting app hardware");
  hw_init();

  Serial.println("[BOOT] starting GPS");
  bool gpsOk = instance.initGPS();
  Serial.printf("[GPS] init %s\n", gpsOk ? "ok" : "failed");

  Serial.println("[BOOT] starting sensor");
  bool sensorOk = instance.initSensor();
  Serial.printf("[SENSOR] init %s\n", sensorOk ? "ok" : "failed");

  // Verify Pager SD card explicitly and retry through LilyGoLib if needed.
  Serial.println("[BOOT] starting SD");
  ensureSdReady(true);

  loadSettings();
  loadWifiCredentialsFromNvs();
  applyDeviceIdentityDefaults();
  g_cfg.zoom = constrain(g_cfg.zoom, APP_ALLOWED_MIN_ZOOM, APP_MAX_ZOOM);

  instance.lockSPI();
  Serial.printf("[SD] cardType=%u totalMB=%llu usedMB=%llu\n",
                (unsigned int)SD.cardType(),
                (unsigned long long)(SD.totalBytes() / (1024ULL * 1024ULL)),
                (unsigned long long)(SD.usedBytes() / (1024ULL * 1024ULL)));
  instance.unlockSPI();

  Serial.println("[TRACK] startup load begin");
  loadTracksFromSd();

  hw_set_disp_backlight(APP_DISPLAY_BRIGHTNESS);
  hw_set_kb_backlight(1);
  hw_set_keyboard_read_callback(keyboardCallback);
  hw_register_imu_process();

  if (!g_tileRequestQueue || !g_tileResultQueue) {
    Serial.println("[TILE TASK] queue creation failed");
  } else {
    BaseType_t taskOk = xTaskCreatePinnedToCore(
        tileDownloadTask,
        "tile-io",
        APP_TILE_TASK_STACK_SIZE,
        nullptr,
        APP_TILE_TASK_PRIORITY,
        &g_tileDownloadTaskHandle,
        0);

    Serial.printf("[TILE TASK] create result=%s ui-core=%d\n",
                  taskOk == pdPASS ? "ok" : "failed",
                  xPortGetCoreID());
  }

  scheduleMapRedraw();
  renderMapIfDue();

  Serial.println("[BOOT] starting LoRa");
  radio_params_t radioParams {};
  hw_get_radio_params(radioParams);
  radioParams.mode = RADIO_RX;
  int16_t radioState = hw_set_radio_params(radioParams);
  Serial.printf("[LORA] init state=%d freq=%.1f bw=%.1f sf=%u cr=%u sync=0x%02X\n",
                radioState, radioParams.freq, radioParams.bandwidth,
                radioParams.sf, radioParams.cr, radioParams.syncWord);
  hw_set_radio_listening();

  prepareSosAudioBuffer();

  disableWifi();
  Serial.println("[WIFI] startup default: disabled");

  g_lastMotionMs = nowMs();
  g_lastInputMs = nowMs();

  uiToast("Outdoor Pager startet - W fuer WLAN", 4500);
  lv_timer_handler();
  delay(40);
}

void loop() {
  instance.loop();
  lv_timer_handler();

  static bool bootLabelHidden = false;
  static uint32_t bootHideAfterMs = nowMs() + 2200;
  if (!bootLabelHidden && nowMs() >= bootHideAfterMs && g_bootLabel) {
    lv_obj_add_flag(g_bootLabel, LV_OBJ_FLAG_HIDDEN);
    bootLabelHidden = true;
  }

  pollRotary();

  char key;
  while (dequeueKey(key)) processKey(key);

  pollGps();
  pollImu();
  pollRadio();
  pollSafety();
  pollSosAudio();
  updatePeerSosPulse();
  processTileResults();
  renderMapIfDue();

  if (g_hasGps && nowMs() - g_lastPosTxMs >= g_cfg.positionIntervalMs) {
    sendPosition();
  }

  if (nowMs() - g_lastMapRefreshMs >= APP_MAP_REFRESH_MS) {
    g_lastMapRefreshMs = nowMs();
    refreshStatus();

    // Retry missing visible tiles after WiFi reconnects without blocking UI.
    scheduleMapRedraw();
  }

  if (nowMs() - g_lastDiagnosticsMs >= APP_DIAGNOSTICS_PERIOD_MS) {
    g_lastDiagnosticsMs = nowMs();

    Serial.printf("[DIAG] core=%d heap=%u minHeap=%u psram=%u queue=%u tileStack=%u redraw=%s\n",
                  xPortGetCoreID(),
                  (unsigned int)ESP.getFreeHeap(),
                  (unsigned int)ESP.getMinFreeHeap(),
                  (unsigned int)ESP.getFreePsram(),
                  (unsigned int)g_tileQueueCount,
                  g_tileDownloadTaskHandle ?
                    (unsigned int)uxTaskGetStackHighWaterMark(g_tileDownloadTaskHandle) : 0,
                  g_mapNeedsRedraw ? "yes" : "no");
    Serial.printf("[GPS] info=%s fix=%s model=%s rx=%u sat=%u lat=%.6f lon=%.6f\n",
                  g_hasGpsInfo ? "yes" : "no",
                  g_hasGps ? "yes" : "no",
                  g_gpsRaw.model.c_str(),
                  (unsigned int)g_gpsRaw.rx_size,
                  (unsigned int)g_gpsRaw.satellite,
                  g_hasGps ? g_gps.lat : 0.0,
                  g_hasGps ? g_gps.lng : 0.0);
  }

  if (g_toast && g_toast_until_ms && nowMs() > g_toast_until_ms) {
    lv_obj_add_flag(g_toast, LV_OBJ_FLAG_HIDDEN);
    g_toast_until_ms = 0;
  }

  delay(5);
}
