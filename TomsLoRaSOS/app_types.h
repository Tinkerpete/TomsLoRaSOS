#pragma once

#include <Arduino.h>

// Defined in a separate header because the Arduino sketch preprocessor creates
// function prototypes before compiling the .ino file. Custom return types used
// by functions must therefore already be visible at prototype-generation time.
struct OutdoorPeer {
  bool used = false;
  String id;
  String name;
  double lat = 0.0;
  double lon = 0.0;
  int batteryMv = 0;
  uint32_t lastSeenMs = 0;
  bool sos = false;
};

// Tile queue types must live in a header. The Arduino sketch preprocessor
// generates function prototypes before compiling the .ino file. Types used in
// function signatures must therefore already be visible at that stage.
struct TileJob {
  int z = 0;
  int x = 0;
  int y = 0;
  bool lookaheadOnly = false;
};

struct TileResult {
  TileJob job {};
  bool success = false;
  bool cached = false;
  bool lookaheadOnly = false;
  int httpCode = 0;
};

struct TrackPoint {
  float lat = 0.0f;
  float lon = 0.0f;
  uint32_t timeMs = 0;
};

struct MapTrack {
  bool used = false;
  bool visible = true;
  bool recording = false;
  String name;
  uint16_t color = 0xFFFF;
  uint32_t pointCount = 0;
  uint32_t pointCapacity = 0;
  bool hasDirection = false;
  double lastDirection = 0.0;
  TrackPoint *points = nullptr;
};

struct TileCacheSlot {
  bool allocated = false;
  bool loading = false;
  bool ready = false;
  int z = -1;
  int x = -1;
  int y = -1;
  uint32_t lastUsed = 0;
  uint16_t *pixels = nullptr;
};
