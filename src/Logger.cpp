#include "Logger.h"
#include <Arduino.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

namespace {
// 32 KB ≈ 4 s of records at 100 Hz: rides out long SD write pauses.
constexpr size_t BUFFER_BYTES = 32 * 1024;
constexpr size_t CHUNK_BYTES = 4096;      // SD likes multiples of 512
constexpr uint32_t FLUSH_MS = 1000;       // at most ~1 s lost on power-off

StreamBufferHandle_t buffer = nullptr;
TaskHandle_t writerTask = nullptr;
bool cardOk = false;

// Shared between the loop (requests) and the writer task (actions).
volatile bool startRequested = false;
volatile bool stopRequested = false;
volatile bool active = false;       // loop may queue records
volatile bool writeError = false;
volatile uint32_t records = 0, dropped = 0, bytesWritten = 0;
logfmt::LogHeader pendingHeader;
char path[16] = "";

File file;

bool openNextFile() {
  Preferences p;
  p.begin("paddlelog", false);
  uint32_t n = p.getUInt("next", 1);
  // The counter survives reflashing, but skip any name already on the card
  // (e.g. a card moved between paddles).
  do {
    snprintf(path, sizeof path, "/PPM%05lu.BIN", (unsigned long)(n % 100000));
    n++;
  } while (SD.exists(path) && n < 100000);
  p.putUInt("next", n);
  p.end();
  file = SD.open(path, FILE_WRITE);
  return (bool)file;
}

void writeBytes(const uint8_t *data, size_t len) {
  if (!file) return;
  size_t w = file.write(data, len);
  bytesWritten = bytesWritten + w;
  if (w != len) writeError = true;
}

void writerLoop(void *) {
  static uint8_t chunk[CHUNK_BYTES];
  uint32_t lastFlush = millis();
  for (;;) {
    if (startRequested && stopRequested) {
      // Stopped before the file was even opened.
      startRequested = stopRequested = false;
    } else if (startRequested) {
      startRequested = false;
      writeError = false;
      bytesWritten = 0;
      xStreamBufferReset(buffer);
      if (openNextFile()) {
        writeBytes(reinterpret_cast<const uint8_t *>(&pendingHeader),
                   sizeof pendingHeader);
        file.flush();
        // A stop that arrived while the file was opening wins; the stop
        // branch below then closes the file.
        active = !stopRequested;
      } else {
        writeError = true;
      }
    }

    size_t n = xStreamBufferReceive(buffer, chunk, sizeof chunk,
                                    pdMS_TO_TICKS(100));
    if (n) writeBytes(chunk, n);

    if (file && millis() - lastFlush >= FLUSH_MS) {
      lastFlush = millis();
      file.flush();
    }

    // Stop only once the loop has stopped queuing and the buffer is empty.
    if (stopRequested && !active && xStreamBufferIsEmpty(buffer)) {
      stopRequested = false;
      if (file) {
        file.flush();
        file.close();
      }
    }
  }
}
}  // namespace

bool initLogger(int csPin) {
  // 4 MHz is plenty for ~8 KB/s and tolerates long wires to the card.
  cardOk = SD.begin(csPin, SPI, 4000000);
  if (!cardOk) return false;
  buffer = xStreamBufferCreate(BUFFER_BYTES, 1);
  if (!buffer) {
    cardOk = false;
    return false;
  }
  // Core 0, low priority: the sensor loop runs on core 1.
  xTaskCreatePinnedToCore(writerLoop, "sdlog", 4096, nullptr, 1, &writerTask,
                          0);
  return true;
}

bool loggerCardPresent() { return cardOk; }

bool startLog(const logfmt::LogHeader &header) {
  if (!cardOk || active || startRequested || stopRequested) return false;
  pendingHeader = header;
  records = 0;
  dropped = 0;
  startRequested = true;
  return true;
}

void stopLog() {
  if (!active && !startRequested) return;
  active = false;
  stopRequested = true;
}

bool logging() { return active; }

void logRecord(const logfmt::LogRecord &r) {
  if (!active) return;
  // Only this function sends, so free space can only grow before the send:
  // checking first guarantees a whole record is queued, never a fragment.
  if (xStreamBufferSpacesAvailable(buffer) < sizeof r) {
    dropped = dropped + 1;
    return;
  }
  xStreamBufferSend(buffer, &r, sizeof r, 0);
  records = records + 1;
}

LoggerStats loggerStats() {
  LoggerStats s;
  s.active = active;
  memcpy(s.path, path, sizeof s.path);
  s.records = records;
  s.dropped = dropped;
  s.bytesWritten = bytesWritten;
  s.writeError = writeError;
  return s;
}
