#ifndef LOGGER_H
#define LOGGER_H

#include "LogFormat.h"

// Raw sensor logging to a microSD card (SPI). The main loop hands records
// to logRecord(), which only copies them into a RAM buffer; a background
// task on the other core does the slow SD writes, so a card that pauses for
// 100 ms doesn't stall the 100 Hz sampling.
//
// One file per session: /PPM00001.BIN, /PPM00002.BIN, ...

bool initLogger(int csPin);  // false if no card
bool loggerCardPresent();

// Starts a new file whose header is `header`. Returns false if no card or
// a log is already running.
bool startLog(const logfmt::LogHeader &header);
void stopLog();  // flushes and closes; safe to call when not logging
bool logging();

// Never blocks. Drops the record (and counts it) if the buffer is full.
void logRecord(const logfmt::LogRecord &r);

struct LoggerStats {
  bool active;
  char path[16];
  uint32_t records;   // records queued in this file
  uint32_t dropped;   // records lost because the buffer was full
  uint32_t bytesWritten;
  bool writeError;
};
LoggerStats loggerStats();

#endif  // LOGGER_H
