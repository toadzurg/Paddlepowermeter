#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include "Config.h"
#include "Scale.h"
#include "IMU.h"
#include "PowerModel.h"
#include "Battery.h"
#include "BLEManager.h"
#include "BoatLink.h"
#include "Logger.h"

// Serial commands (115200 baud, newline terminated):
//   tare          zero the force sensor (paddle unloaded)
//   cal <N>       calibrate with a known force at the blade centre of
//                 pressure, in newtons (e.g. a 5 kg weight: cal 49.03)
//   debug         toggle 10 Hz telemetry
//   status        print calibration and sensor state
//   imu           print both IMUs once, in paddle axes and raw chip axes
//   boat pair     pair with the next boat node heard (away from other boats)
//   boat cal start   then paddle a measured distance on still water ...
//   boat cal <m>     ... and enter it, e.g. boat cal 500
//   log start | log stop | log   raw sensor log on the microSD card
//   pin [<6 digits>]   show or set the PIN that unlocks app commands
//
// The web app (docs/app) sends the same commands over Bluetooth.

namespace {
PowerModel model;
Preferences prefs;

const uint32_t SAMPLE_US = (uint32_t)(1e6f / cfg::SAMPLE_HZ);
uint32_t nextSampleUs = 0;
uint32_t lastSampleUs = 0;
uint32_t lastNotifyMs = 0;
uint32_t lastBatteryMs = 0;
uint32_t lastDebugMs = 0;
float forceN = 0;
uint8_t imuMask = 0;
bool debug = false;
String line;
// Where command replies go: the serial console, or the app's reply channel
// while an app command runs.
Print *out = &Serial;
char blePin[7] = "";  // unlocks app commands; see the "pin" command

int32_t zeroOffset = 0;
float countsPerN = cfg::DEFAULT_COUNTS_PER_N;
float pulsesPerM = 0;        // impeller calibration, 0 = uncalibrated
bool boatCalRunning = false;
uint32_t boatCalStartCount = 0;

logfmt::LogRecord rec = {};  // filled by readSample(), finished in loop()
bool logRestartPending = false;

logfmt::LogHeader makeLogHeader() {
  logfmt::LogHeader h = {};
  h.magic = logfmt::MAGIC;
  h.version = logfmt::VERSION;
  h.headerSize = sizeof h;
  h.recordSize = sizeof(logfmt::LogRecord);
  h.sampleHz = cfg::SAMPLE_HZ;
  h.imu2PosM = cfg::IMU2_POS_M;
  h.bladeCopPosM = cfg::BLADE_COP_POS_M;
  h.pivotPriorPosM = cfg::PIVOT_PRIOR_POS_M;
  const cfg::AxisMap *maps[2] = {&cfg::IMU1_AXES, &cfg::IMU2_AXES};
  for (int i = 0; i < 2; i++)
    for (int k = 0; k < 3; k++) {
      h.axis[i][k] = maps[i]->axis[k];
      h.sign[i][k] = maps[i]->sign[k];
    }
  getGyroBias(h.gyroBias);
  h.forceZeroOffset = zeroOffset;
  h.forceCountsPerN = countsPerN;
  h.pulsesPerM = pulsesPerM;
  h.startMillis = millis();
  strncpy(h.firmware, __DATE__, sizeof h.firmware - 1);
  return h;
}

// Calibration changed: roll over to a new file so each file's header
// matches its data.
void restartLogIfRunning() {
  if (!logging()) return;
  stopLog();
  logRestartPending = true;
}

void printLogStatus() {
  LoggerStats l = loggerStats();
  if (!loggerCardPresent()) {
    out->println("log: no SD card");
  } else {
    out->printf("log: %s %s, %lu records, %lu dropped, %lu KB%s\n",
                  l.active ? "recording" : "stopped", l.path,
                  (unsigned long)l.records, (unsigned long)l.dropped,
                  (unsigned long)(l.bytesWritten / 1024),
                  l.writeError ? ", WRITE ERROR" : "");
  }
}

void saveCalibration() {
  prefs.putInt("zero", zeroOffset);
  prefs.putFloat("cal", countsPerN);
}

void printStatus() {
  out->printf("IMU1 %s, IMU2 %s\n", imuMask & 1 ? "ok" : "MISSING",
                imuMask & 2 ? "ok" : "MISSING");
  out->printf("zero offset %ld, %.3f counts/N%s\n", (long)zeroOffset,
                countsPerN, prefs.isKey("cal") ? "" : " (UNCALIBRATED default)");
  out->printf("pivot %.3f m, lever arm %.3f m, battery %u%%\n",
                model.pivotPosition(), model.leverArm(), getBatteryLevel());
  BoatSpeed b = readBoatSpeed(pulsesPerM);
  out->printf("boat node %s, %s, %.2f pulses/m%s\n",
                boatLinkPaired() ? "paired" : "NOT PAIRED",
                b.fresh ? "receiving" : "no signal", pulsesPerM,
                pulsesPerM > 0 ? "" : " (UNCALIBRATED)");
  printLogStatus();
  if (out == &Serial) out->printf("app PIN %s\n", blePin);
}

// For checking the axis maps on the bench (docs/BENCH_TEST.md). Gyro bias
// is removed in the paddle line but not in the chip line.
void printImu() {
  ImuReading p[2], raw[2];
  readIMU(p, raw);
  for (int i = 0; i < 2; i++) {
    if (!p[i].valid) {
      out->printf("IMU%d missing\n", i + 1);
      continue;
    }
    out->printf("IMU%d paddle  accel x %6.2f y %6.2f z %6.2f m/s2   "
                  "gyro x %6.3f y %6.3f z %6.3f rad/s\n", i + 1,
                  p[i].accel[0], p[i].accel[1], p[i].accel[2],
                  p[i].gyro[0], p[i].gyro[1], p[i].gyro[2]);
    out->printf("IMU%d chip    accel x %6.2f y %6.2f z %6.2f m/s2   "
                  "gyro x %6.3f y %6.3f z %6.3f rad/s\n", i + 1,
                  raw[i].accel[0], raw[i].accel[1], raw[i].accel[2],
                  raw[i].gyro[0], raw[i].gyro[1], raw[i].gyro[2]);
  }
}

void handleBoatCommand(const String &arg) {
  if (arg == "pair") {
    startBoatPairing();
    out->println("pairing: waiting for a boat node...");
    return;
  }
  BoatSpeed b = readBoatSpeed(pulsesPerM);
  if (arg == "cal start") {
    if (!b.fresh) {
      out->println("no signal from boat node");
      return;
    }
    boatCalRunning = true;
    boatCalStartCount = b.pulseCount;
    out->println("paddle the measured distance, then: boat cal <metres>");
  } else if (arg.startsWith("cal ")) {
    float metres = arg.substring(4).toFloat();
    uint32_t pulses = b.pulseCount - boatCalStartCount;
    if (!boatCalRunning || !b.fresh || metres <= 0 || pulses == 0) {
      out->println("usage: boat cal start, paddle, then boat cal <metres>");
      return;
    }
    boatCalRunning = false;
    pulsesPerM = pulses / metres;
    prefs.putFloat("ppm", pulsesPerM);
    restartLogIfRunning();
    out->printf("impeller: %lu pulses over %.1f m = %.3f pulses/m\n",
                  (unsigned long)pulses, metres, pulsesPerM);
  } else {
    out->println("boat commands: pair | cal start | cal <metres>");
  }
}

void handleCommand(const String &cmd) {
  if (cmd == "tare") {
    zeroOffset = tareScale();
    saveCalibration();
    restartLogIfRunning();
    out->printf("tared: zero offset %ld\n", (long)zeroOffset);
  } else if (cmd.startsWith("cal ")) {
    float n = cmd.substring(4).toFloat();
    if (n <= 0) {
      out->println("usage: cal <newtons>");
      return;
    }
    countsPerN = calibrateScale(n);
    saveCalibration();
    restartLogIfRunning();
    out->printf("calibrated: %.3f counts/N\n", countsPerN);
  } else if (cmd == "debug") {
    debug = !debug;
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "imu") {
    printImu();
  } else if (cmd == "log start") {
    if (logging()) out->println("log: already recording");
    else if (!startLog(makeLogHeader())) out->println("log: no SD card");
    else out->println("log: starting");
  } else if (cmd == "log stop") {
    stopLog();
    logRestartPending = false;
    out->println("log: stopped");
  } else if (cmd == "log") {
    printLogStatus();
  } else if (cmd == "pin") {
    // Replies reach every connected device, so never show it over Bluetooth.
    if (out == &Serial) out->printf("app PIN %s\n", blePin);
    else out->println("pin: shown on the serial console only");
  } else if (cmd.startsWith("pin ")) {
    String p = cmd.substring(4);
    p.trim();
    bool ok = p.length() == 6;
    for (unsigned i = 0; ok && i < p.length(); i++) ok = isDigit(p[i]);
    if (!ok) {
      out->println("usage: pin <6 digits>");
      return;
    }
    strncpy(blePin, p.c_str(), sizeof blePin - 1);
    prefs.putString("blepin", blePin);
    setBlePin(blePin);
    out->println("pin: changed; connected apps must unlock again");
  } else if (cmd.startsWith("boat")) {
    String arg = cmd.substring(4);
    arg.trim();
    handleBoatCommand(arg);
  } else if (cmd.length()) {
    out->println("commands: tare | cal <newtons> | debug | status | imu | boat ... | log ... | pin");
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      handleCommand(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
}

// A random PIN on first boot, kept in flash.
void loadPin() {
  String p = prefs.getString("blepin", "");
  if (p.length() != 6) {
    snprintf(blePin, sizeof blePin, "%06lu", (unsigned long)(esp_random() % 1000000));
    prefs.putString("blepin", blePin);
  } else {
    strncpy(blePin, p.c_str(), sizeof blePin - 1);
  }
}

void runBleCommands() {
  String cmd;
  while (takeBleCommand(cmd)) {
    out = &bleReply();
    handleCommand(cmd);
    out->println("ok");  // marks the end of the reply for the app
    out = &Serial;
  }
}

uint32_t lastLiveMs = 0;
uint16_t liveSeq = 0;
uint16_t lastStrokeSent = 0;

void sendAppPackets(uint32_t nowMs, const BoatSpeed &b) {
  if (nowMs - lastLiveMs >= ble::LIVE_INTERVAL_MS) {
    lastLiveMs = nowMs;
    ble::LivePacket p;
    p.seq = liveSeq++;
    p.tMs = (uint16_t)nowMs;
    p.flags = (model.inDrive() ? ble::LIVE_IN_DRIVE : 0) |
              (b.fresh ? ble::LIVE_BOAT_FRESH : 0) |
              (logging() ? ble::LIVE_LOGGING : 0);
    p.forceDeciN = ble::toI16(forceN * 10);
    p.omegaCrad = ble::toI16(model.angularVelocity() * 100);
    p.instPowerW = ble::toI16(model.instantaneousPower());
    p.boatSpeedCms = ble::toU16(b.speedMps * 100);
    p.pivotMm = ble::toI16(model.pivotPosition() * 1000);
    p.leverMm = ble::toI16(model.leverArm() * 1000);
    sendLive(p);
  }
  if (model.strokeCount() != lastStrokeSent) {
    lastStrokeSent = model.strokeCount();
    ble::StrokePacket p;
    p.count = model.strokeCount();
    p.powerW = ble::toI16(model.strokePower());
    p.rateCentiSpm = ble::toU16(model.strokeRate() * 100);
    p.energyDeciJ = ble::toU16(model.strokeEnergy() * 10);
    bool split = model.hasPowerSplit();
    p.usefulW = split ? ble::toI16(model.usefulPower()) : 0;
    p.slipW = split ? ble::toI16(model.slipPower()) : 0;
    p.efficiencyPermille = split ? ble::toU16(model.efficiency() * 1000) : 0;
    p.flags = split ? ble::STROKE_HAS_SPLIT : 0;
    sendStroke(p);
  }
}

PaddleSample readSample(uint32_t nowUs) {
  PaddleSample s;
  s.timeMs = millis();
  s.dt = lastSampleUs ? (nowUs - lastSampleUs) * 1e-6f : 0;
  lastSampleUs = nowUs;
  // Force holds its last value between the ADC's 80 Hz conversions.
  bool forceFresh = readForce(forceN, &rec.forceRaw);
  s.forceN = forceN;
  ImuReading raw[2];
  readIMU(s.imu, raw);
  BoatSpeed b = readBoatSpeed(pulsesPerM);
  s.boatSpeedValid = b.fresh && pulsesPerM > 0;
  s.boatSpeedMps = b.speedMps;

  rec.tUs = nowUs;
  rec.flags = (raw[0].valid ? logfmt::F_IMU1_VALID : 0) |
              (raw[1].valid ? logfmt::F_IMU2_VALID : 0) |
              (forceFresh ? logfmt::F_FORCE_FRESH : 0) |
              (b.fresh ? logfmt::F_BOAT_FRESH : 0);
  for (int i = 0; i < 2; i++)
    for (int k = 0; k < 3; k++) {
      rec.gyro[i][k] = raw[i].gyro[k];
      rec.accel[i][k] = raw[i].accel[k];
    }
  rec.boatPulseHz = b.pulseHz;
  rec.boatPulseCount = b.pulseCount;
  return s;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // three devices at 100 Hz need fast-mode I²C

  prefs.begin("paddle", false);
  zeroOffset = prefs.getInt("zero", 0);
  countsPerN = prefs.getFloat("cal", cfg::DEFAULT_COUNTS_PER_N);
  pulsesPerM = prefs.getFloat("ppm", 0);
  uint8_t boatMac[6];
  bool haveBoatMac = prefs.getBytes("boatmac", boatMac, 6) == 6;
  loadPin();

  if (!initScale(zeroOffset, countsPerN)) Serial.println("NAU7802 not found");
  imuMask = initIMU();
  if (!(imuMask & 1)) Serial.println("IMU1 not found");
  if (!(imuMask & 2)) Serial.println("IMU2 not found (running on IMU1 only)");
  if (!initBattery()) Serial.println("MAX17048 not found");
  initBLE(blePin);
  if (!initBoatLink(haveBoatMac ? boatMac : nullptr))
    Serial.println("ESP-NOW init failed: no boat speed");

  // Boot with the paddle lying still: remove gyro bias and re-zero the
  // strain gauge (its offset drifts with temperature between sessions).
  if (calibrateGyroBias()) {
    zeroOffset = tareScale();
    saveCalibration();
  } else {
    Serial.println("paddle moving at boot: skipped gyro/force zeroing");
  }
  if (!prefs.isKey("cal"))
    Serial.println("force sensor UNCALIBRATED: hang a known weight and run 'cal <N>'");

  // After zeroing, so the first file's header has this session's offsets.
  if (!initLogger(cfg::LOG_SD_CS_PIN)) Serial.println("no SD card: not logging");
  else if (cfg::LOG_AUTOSTART) startLog(makeLogHeader());

  model.begin(readSample(micros()));
  sendBattery(getBatteryLevel());
  nextSampleUs = micros() + SAMPLE_US;
}

void loop() {
  pollSerial();
  runBleCommands();

  // Fixed-rate sampling: the filters are tuned for SAMPLE_HZ and use the
  // measured dt, so jitter is tolerated but the loop must not run slow.
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_US;
  if ((int32_t)(nowUs - nextSampleUs) > (int32_t)SAMPLE_US)
    nextSampleUs = nowUs + SAMPLE_US;  // fell far behind: don't burst

  model.update(readSample(nowUs));

  rec.forceN = forceN;
  rec.omega = model.angularVelocity();
  rec.pivotPosM = model.pivotPosition();
  rec.instPowerW = model.instantaneousPower();
  logRecord(rec);
  if (logRestartPending && startLog(makeLogHeader())) logRestartPending = false;

  uint8_t mac[6];
  if (takeNewPairing(mac)) {
    prefs.putBytes("boatmac", mac, 6);
    // Pairing may have been started from the app, so tell both.
    for (Print *p : {(Print *)&Serial, &bleReply()})
      p->printf("paired with boat node %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  uint32_t nowMs = millis();
  sendAppPackets(nowMs, readBoatSpeed(pulsesPerM));
  if (nowMs - lastNotifyMs >= cfg::BLE_NOTIFY_MS) {
    lastNotifyMs = nowMs;
    sendPowerMeasurement((int16_t)lroundf(model.strokePower()),
                         model.strokeCount(), model.lastStrokeTimeMs());
  }
  if (nowMs - lastBatteryMs >= cfg::BATTERY_POLL_MS) {
    lastBatteryMs = nowMs;
    sendBattery(getBatteryLevel());
  }
  if (debug && nowMs - lastDebugMs >= 100) {
    lastDebugMs = nowMs;
    BoatSpeed b = readBoatSpeed(pulsesPerM);
    Serial.printf("F %.1f N  w %.2f rad/s  a %.1f rad/s2  L %.3f m  "
                  "P %.0f W  stroke %.0f W  %.1f spm  boat %s %.2f m/s "
                  "(%.1f Hz)\n",
                  forceN, model.angularVelocity(), model.angularAccel(),
                  model.leverArm(), model.instantaneousPower(),
                  model.strokePower(), model.strokeRate(),
                  b.fresh ? "" : "--", b.speedMps, b.pulseHz);
    if (model.hasPowerSplit())
      Serial.printf("  useful %.0f W  slip %.0f W  efficiency %.0f%%\n",
                    model.usefulPower(), model.slipPower(),
                    100 * model.efficiency());
  }
}
