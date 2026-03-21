#pragma once

#ifndef MMS_SIM

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <map>
#include <string>
#include <vector>

#include "Mouse.h" // MouseState
#include "Types.h"

class Logger {
  static inline SdFat sd;
  static inline SdFile file;
  static inline bool initialized = false;
  static inline bool headerWritten = false;

  static inline std::map<std::string, std::string> currentRow;
  static inline std::vector<std::string> columns;

  static constexpr size_t BUF_SIZE = 4096;
  static inline char buf[BUF_SIZE];
  static inline size_t bufPos = 0;
  static inline uint32_t lastFlushUs = 0;
  static constexpr uint32_t FLUSH_INTERVAL_US = 100000; // 100ms

  static inline int tickCount = 0;
  static constexpr int WARMUP_TICKS = 10;

  static void bufWrite(const char *s) {
    size_t len = strlen(s);
    if (bufPos + len >= BUF_SIZE - 1) {
      flushBuf();
    }
    memcpy(buf + bufPos, s, len);
    bufPos += len;
  }

  static void flushBuf() {
    if (!initialized || bufPos == 0)
      return;
    file.write(buf, bufPos);
    file.sync();
    bufPos = 0;
    lastFlushUs = micros();
  }

  static void writeHeader() {
    if (headerWritten)
      return;
    bufWrite("timestamp_us");
    for (auto &col : columns) {
      bufWrite(",");
      bufWrite(col.c_str());
    }
    bufWrite("\n");
    headerWritten = true;
  }

  static void addColumn(const std::string &key) {
    for (auto &col : columns) {
      if (col == key)
        return;
    }
    columns.push_back(key);
  }

  static void logRaw(const std::string &key, const std::string &value) {
    if (!initialized)
      return;
    currentRow[key] = value;
    addColumn(key);
  }

  static std::string indexed(const char *key, int i) {
    return std::string(key) + "[" + std::to_string(i) + "]";
  }

  static std::string indexed(const char *key, int i, int j) {
    return std::string(key) + "[" + std::to_string(i) + "][" +
           std::to_string(j) + "]";
  }

public:
  static void init() {
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
      pinMode(LED_BUILTIN, OUTPUT);
      for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
      return;
    }

    char filename[16];
    for (int i = 1; i < 1000; i++) {
      snprintf(filename, sizeof(filename), "log_%03d.csv", i);
      if (!sd.exists(filename)) {
        file.open(filename, O_WRONLY | O_CREAT | O_TRUNC);
        break;
      }
    }

    if (!file.isOpen())
      return;

    initialized = true;
    lastFlushUs = micros();
  }

  static void log(const char *key, double value) {
    char num[24];
    snprintf(num, sizeof(num), "%.6f", value);
    logRaw(key, num);
  }

  static void log(const char *key, float value) { log(key, (double)value); }

  static void log(const char *key, int value) {
    logRaw(key, std::to_string(value));
  }

  static void log(const char *key, unsigned int value) {
    logRaw(key, std::to_string(value));
  }

  static void log(const char *key, long value) {
    logRaw(key, std::to_string(value));
  }

  static void log(const char *key, unsigned long value) {
    logRaw(key, std::to_string(value));
  }

  static void log(const char *key, bool value) {
    logRaw(key, value ? "1" : "0");
  }

  static void log(const char *key, unsigned char value) {
    logRaw(key, std::to_string((int)value));
  }

  static void log(const char *key, const char *value) { logRaw(key, value); }

  static void log(const char *key, const std::string &value) {
    logRaw(key, value);
  }

  template <typename T>
  static typename std::enable_if<std::is_enum<T>::value>::type
  log(const char *key, T value) {
    logRaw(key, std::to_string(static_cast<int>(value)));
  }

  static void log(const char *key, const WorldCoord &c) {
    std::string k(key);
    log((k + "/x").c_str(), c.x);
    log((k + "/y").c_str(), c.y);
    log((k + "/theta").c_str(), c.theta);
  }

  // Convenience: log a WorldCoord under the key "w" (pose shorthand used by
  // TeensyIO)
  static void log(const WorldCoord &c) { log("w", c); }

  static void log(const char *key, const GridCoord &c) {
    std::string k(key);
    log((k + "/x").c_str(), c.x);
    log((k + "/y").c_str(), c.y);
    log((k + "/dir").c_str(), c.dir);
  }

  // Log the full MouseState: pose, grid position, discovered walls, and
  // explored flags. Column names:
  //   state/x, state/y, state/dir          — current grid cell & heading
  //   walls[i][j]                           — wall bitmask for each cell
  //   (TOP=8,RIGHT=4,DOWN=2,LEFT=1) explored[i][j]                        — 1
  //   if that cell has been visited
  static void log(const MouseState &ms) {
    log("state/x", ms.x);
    log("state/y", ms.y);
    log("state/dir", ms.dir);
    log("walls", ms.walls);       // uses the 2-D array overload
    log("explored", ms.explored); // uses the 2-D array overload
  }

  template <typename T>
  static void log(const char *key, const std::vector<T> &vec) {
    for (size_t i = 0; i < vec.size(); i++) {
      log(indexed(key, i).c_str(), vec[i]);
    }
  }

  template <typename T, size_t N>
  static void log(const char *key, const T (&arr)[N]) {
    for (size_t i = 0; i < N; i++) {
      log(indexed(key, i).c_str(), arr[i]);
    }
  }

  template <typename T, size_t R, size_t C>
  static void log(const char *key, const T (&arr)[R][C]) {
    for (size_t i = 0; i < R; i++) {
      for (size_t j = 0; j < C; j++) {
        log(indexed(key, i, j).c_str(), arr[i][j]);
      }
    }
  }

  template <typename T>
  static void log(const char *key, const std::vector<std::vector<T>> &vec) {
    for (size_t i = 0; i < vec.size(); i++) {
      for (size_t j = 0; j < vec[i].size(); j++) {
        log(indexed(key, i, j).c_str(), vec[i][j]);
      }
    }
  }

  static void tick() {
    if (!initialized)
      return;

    tickCount++;

    if (tickCount <= WARMUP_TICKS) {
      currentRow.clear();
      return;
    }

    if (!headerWritten) {
      writeHeader();
    }

    char num[24];
    snprintf(num, sizeof(num), "%lu", (unsigned long)micros());
    bufWrite(num);

    for (auto &col : columns) {
      bufWrite(",");
      auto it = currentRow.find(col);
      if (it != currentRow.end()) {
        bufWrite(it->second.c_str());
      }
    }
    bufWrite("\n");

    currentRow.clear();

    if (micros() - lastFlushUs >= FLUSH_INTERVAL_US) {
      flushBuf();
    }
  }

  static void close() {
    if (!initialized)
      return;
    flushBuf();
    file.close();
  }
};

#else

#include "Mouse.h"

// No-op logger for simulation builds
class Logger {
public:
  static void init() {}
  static void log(const char *, ...) {}
  template <typename T> static void log(const char *, const T &) {}
  template <typename T> static void log(const char *, const std::vector<T> &) {}
  static void log(const WorldCoord &) {}
  static void log(const MouseState &) {}
  static void tick() {}
  static void close() {}
};

#endif
