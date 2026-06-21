#pragma once

// LilyGO Pager Outdoor Prototype - central tuning values

#define APP_DISPLAY_WIDTH  480
#define APP_DISPLAY_HEIGHT 222
#define TILE_SIZE          256

#define APP_DEFAULT_ZOOM 18
#define APP_MIN_ZOOM 13
#define APP_MAX_ZOOM 19
#define APP_ALLOWED_MIN_ZOOM 13

#define APP_MAX_PEERS 12
#define APP_MAX_MESSAGES 24
#define APP_MAX_MESSAGE_LIST_ITEMS (APP_MAX_MESSAGES + 2)
#define APP_MAX_RADIO_PAYLOAD 220
#define APP_MAX_TRACKS 8
#define APP_MAX_TRACK_LIST_ITEMS (APP_MAX_TRACKS + 2)
#define APP_MAX_TRACK_POINTS 8192
#define APP_TRACK_DRAW_MAX_SEGMENTS 260
#define APP_TRACK_MIN_POINT_DELTA_DEG 0.00002
#define APP_TRACK_INTERVAL_MS 15000UL
#define APP_TRACK_SMART_SPEED_KMPH 3.0
#define APP_MAX_TILE_QUEUE 16
#define APP_LOOKAHEAD_TILE_RADIUS 1

#define APP_POS_INTERVAL_MS 60000UL
#define APP_GPS_REFRESH_MS 1000UL
#define APP_IMU_REFRESH_MS 100UL
#define APP_RADIO_REFRESH_MS 80UL
#define APP_MAP_REFRESH_MS 1500UL

#define APP_DISPLAY_BRIGHTNESS 14
#define APP_DISPLAY_DIM_BRIGHTNESS 3

// IMU orientation-delta heuristic.
// Tune after practical tests with the installed BHI260AP firmware.
#define APP_IMU_MOTION_DELTA_DEG 2.0f
#define APP_POCKET_QUIET_MS 3500UL
#define APP_POCKET_PITCH_DEG 62.0f
#define APP_POCKET_ROLL_DEG 68.0f

#define APP_DEFAULT_SAFETY_NO_MOTION_SECONDS 30
#define APP_DEFAULT_SAFETY_COUNTDOWN_SECONDS 30
#define APP_SOS_REPEAT_MS 30000UL

#define APP_CONFIG_PATH "/pager.ini"
#define APP_MESSAGE_LOG_PATH "/messages/chat.log"
#define APP_TRACKS_DIR "/tracks"
#define APP_TRACKS_PATH "/tracks"
#define APP_TRACKS_COMBINED_PATH "/tracks/tracks.gpx"
#define APP_TRACKS_LEGACY_PATH "/tracks/tracks.txt"
#define APP_MAP_ROOT "/maps"

#define APP_TILE_USER_AGENT "TomsLoRaSOS/0.1"
#define APP_DEFAULT_FALLBACK_LAT 0.0
#define APP_DEFAULT_FALLBACK_LON 0.0

// RGB565 helper
#define RGB565(r,g,b) (uint16_t)((((uint16_t)(r) & 0xF8) << 8) | (((uint16_t)(g) & 0xFC) << 3) | ((uint16_t)(b) >> 3))

// Map movement per rotary detent while manually exploring the map.
#define APP_MAP_PAN_STEP_PX 64

// Debounce interval for the rotary center button.
#define APP_ROTARY_PRESS_DEBOUNCE_MS 350UL
#define APP_POPUP_ROTARY_SCROLL_PX 18


// Background tile downloader on the second ESP32-S3 CPU core.
#define APP_TILE_TASK_STACK_SIZE       12288
#define APP_TILE_TASK_PRIORITY         1
#define APP_TILE_RESULT_QUEUE_LENGTH   APP_MAX_TILE_QUEUE
#define APP_TILE_WIFI_RETRY_MS         500UL

// SOS acoustic warning. Based on the LilyGoLib SimpleTone example.
#define APP_SOS_AUDIO_SAMPLE_RATE      16000
#define APP_SOS_AUDIO_BEEP_MS          90
#define APP_SOS_AUDIO_FREQ_HZ          1250
#define APP_SOS_AUDIO_LOCAL_VOLUME     35
#define APP_SOS_AUDIO_REMOTE_VOLUME    85
#define APP_SOS_WARNING_BEEP_PERIOD_MS 1000UL
#define APP_SOS_ALARM_BEEP_GAP_MS      240UL
#define APP_SOS_ALARM_PAUSE_MS         1100UL
#define APP_SOS_AUDIO_BEEP_SAMPLES \
  ((APP_SOS_AUDIO_SAMPLE_RATE * APP_SOS_AUDIO_BEEP_MS) / 1000)


// Robust v0.2 architecture: Core 0 owns SD, HTTP and PNG decoding.
// Core 1 draws only from decoded PSRAM tiles.
#define APP_TILE_CACHE_SLOTS           8
#define APP_UI_RENDER_MIN_INTERVAL_MS  45UL
#define APP_TILE_TASK_YIELD_MS         1UL
#define APP_DIAGNOSTICS_PERIOD_MS      10000UL


// v0.2.1 stability: never keep the shared SPI bus locked while waiting for
// network data. Tiles are transferred in short chunks.
#define APP_TILE_IO_CHUNK_BYTES        4096
#define APP_TILE_STREAM_TIMEOUT_MS     15000UL
#define APP_TILE_MAX_PNG_BYTES         (256UL * 1024UL)
