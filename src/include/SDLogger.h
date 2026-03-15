#pragma once

#ifndef MMS_SIM

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <map>
#include <string>
#include <vector>

class Logger {
  static inline SdFat sd;
  static inline SdFile file;
  static inline bool initialized = false;
  static inline bool headerWritten = false;

  static inline std::map<std::string, double> currentRow;
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

public:
  static void init() {
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {

      // Blink LED to indicate failure
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
    if (!initialized)
      return;
    currentRow[key] = value;
    for (auto &col : columns) {
      if (col == key)
        return;
    }
    columns.push_back(key);
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
        snprintf(num, sizeof(num), "%.6f", it->second);
        bufWrite(num);
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

class Logger {
public:
  static void init() {}
  static void log(const char *, double) {}
  static void tick() {}
  static void close() {}
};

#endif
