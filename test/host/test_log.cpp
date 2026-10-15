// End-to-end test of the SD log format and the offline replay.
//
// A simulated session is written to a log file exactly as the firmware
// would write it: readings in a deliberately scrambled sensor frame with
// gyro bias added, force as raw ADC counts, timestamps crossing the µs
// wrap, and a torn final record (power cut). Replaying the file must
// reproduce a direct PowerModel run of the same session.
//
// Usage: test_log OUT.BIN   (run.sh then checks OUT.BIN with log2csv.py)

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "LogFormat.h"
#include "LogReplay.h"
#include "StrokeSim.h"

static int failures = 0;

#define CHECK_NEAR(name, got, want, tol)                                      \
  do {                                                                        \
    double g_ = (got), w_ = (want), t_ = (tol);                               \
    bool ok_ = std::fabs(g_ - w_) <= t_;                                      \
    std::printf("  [%s] %-42s got %10.4f  want %10.4f ± %g\n",               \
                ok_ ? " ok " : "FAIL", name, g_, w_, t_);                     \
    if (!ok_) failures++;                                                     \
  } while (0)

int main(int argc, char **argv) {
  const char *path = argc > 1 ? argv[1] : "test_log.bin";
  std::printf("\nSD log round trip (%s)\n", path);

  // Scrambled mounting: IMU1 X/Y swapped with a flipped axis, IMU2 rotated.
  logfmt::LogHeader h = {};
  h.magic = logfmt::MAGIC;
  h.version = logfmt::VERSION;
  h.headerSize = sizeof h;
  h.recordSize = sizeof(logfmt::LogRecord);
  h.sampleHz = cfg::SAMPLE_HZ;
  h.imu2PosM = cfg::IMU2_POS_M;
  h.bladeCopPosM = cfg::BLADE_COP_POS_M;
  h.bladeBendDeg = cfg::BLADE_BEND_DEG;
  h.shaftBendPosM = cfg::SHAFT_BEND_POS_M;
  h.pivotPriorPosM = cfg::PIVOT_PRIOR_POS_M;
  const uint8_t axis[2][3] = {{1, 0, 2}, {2, 0, 1}};
  const int8_t sign[2][3] = {{-1, 1, 1}, {1, -1, 1}};
  const float bias[2][3] = {{0.02f, -0.01f, 0.005f}, {-0.03f, 0.0f, 0.015f}};
  std::memcpy(h.axis, axis, sizeof axis);
  std::memcpy(h.sign, sign, sizeof sign);
  std::memcpy(h.gyroBias, bias, sizeof bias);
  h.forceZeroOffset = 81234;
  h.forceCountsPerN = 1234.5f;
  h.pulsesPerM = 7.5f;
  h.startMillis = 5000;
  std::strncpy(h.firmware, "test", sizeof h.firmware);

  std::FILE *f = std::fopen(path, "wb");
  if (!f) { std::perror(path); return 1; }
  std::fwrite(&h, sizeof h, 1, f);

  SimConfig c;
  c.boatSpeed = 2.5;
  PowerModel direct;
  bool first = true;
  long written = 0;
  uint32_t tUs = 0xFFFFFFFFu - 5000000u;  // wraps 5 s into the session
  uint16_t directStrokes = 0;
  double directPowerSum = 0;
  int directStrokesCounted = 0;

  SimTruth truth = generateStroke(c, 1, [&](const PaddleSample &s, double t) {
    // The firmware sees force quantised to ADC counts; so does the replay.
    PaddleSample q = s;
    int32_t raw = (int32_t)std::lround(s.forceN * h.forceCountsPerN) +
                  h.forceZeroOffset;
    q.forceN = (raw - h.forceZeroOffset) / h.forceCountsPerN;
    // ...and boat speed that came through pulses/m.
    float pulseHz = s.boatSpeedValid ? s.boatSpeedMps * h.pulsesPerM : 0;
    q.boatSpeedMps = s.boatSpeedValid ? pulseHz / h.pulsesPerM : 0;

    if (first) direct.begin(q);
    first = false;
    direct.update(q);
    if (direct.strokeCount() != directStrokes) {
      directStrokes = direct.strokeCount();
      if (t > 10) {  // after the pivot estimate has converged
        directPowerSum += direct.strokePower();
        directStrokesCounted++;
      }
    }

    logfmt::LogRecord r = {};
    if (written) tUs += (uint32_t)std::lround(s.dt * 1e6);
    r.tUs = tUs;
    r.forceRaw = raw;
    r.flags = logfmt::F_FORCE_FRESH |
              (s.imu[0].valid ? logfmt::F_IMU1_VALID : 0) |
              (s.imu[1].valid ? logfmt::F_IMU2_VALID : 0) |
              (s.boatSpeedValid ? logfmt::F_BOAT_FRESH : 0);
    for (int i = 0; i < 2; i++)
      for (int k = 0; k < 3; k++) {
        // Inverse of SampleBuilder: back into the chip's own axes.
        uint8_t ax = axis[i][k];
        r.gyro[i][ax] = sign[i][k] * s.imu[i].gyro[k] + bias[i][ax];
        r.accel[i][ax] = sign[i][k] * s.imu[i].accel[k];
      }
    r.boatPulseHz = pulseHz;
    r.boatPulseCount = 0;
    r.forceN = q.forceN;
    r.omega = direct.angularVelocity();
    r.pivotPosM = direct.pivotPosition();
    r.instPowerW = direct.instantaneousPower();
    std::fwrite(&r, sizeof r, 1, f);
    written++;
  });
  // Torn final record, as left by a power cut mid-write.
  logfmt::LogRecord torn = {};
  std::fwrite(&torn, 40, 1, f);
  std::fclose(f);

  // --- Replay ---------------------------------------------------------------
  LogReader reader;
  std::string err;
  if (!reader.open(path, err)) {
    std::printf("  [FAIL] open: %s\n", err.c_str());
    return 1;
  }
  SampleBuilder builder(reader.header());
  PowerModel replay;
  logfmt::LogRecord r;
  long read = 0;
  bool rfirst = true;
  double maxDOmega = 0, maxDPivot = 0, maxDForce = 0;
  uint16_t replayStrokes = 0;
  double replayPowerSum = 0;
  int replayStrokesCounted = 0;
  bool replaySplit = false;
  while (reader.next(r)) {
    PaddleSample s = builder.next(r);
    if (rfirst) replay.begin(s);
    rfirst = false;
    replay.update(s);
    read++;
    maxDOmega = std::fmax(maxDOmega, std::fabs(replay.angularVelocity() - r.omega));
    maxDPivot = std::fmax(maxDPivot, std::fabs(replay.pivotPosition() - r.pivotPosM));
    maxDForce = std::fmax(maxDForce, std::fabs(s.forceN - r.forceN));
    if (replay.strokeCount() != replayStrokes) {
      replayStrokes = replay.strokeCount();
      if (builder.elapsedSeconds() > 10) {
        replayPowerSum += replay.strokePower();
        replayStrokesCounted++;
      }
    }
    replaySplit = replaySplit || replay.hasPowerSplit();
  }

  double truePower = truth.energy / truth.paddleTime;
  double directMean = directPowerSum / directStrokesCounted;
  double replayMean = replayPowerSum / replayStrokesCounted;

  CHECK_NEAR("records read (torn record ignored)", read, written, 0);
  CHECK_NEAR("duration across µs wrap (s)", builder.elapsedSeconds(),
             c.seconds - 1.0 / cfg::SAMPLE_HZ, 0.011);
  CHECK_NEAR("force rebuilt from raw counts (N)", maxDForce, 0, 1e-3);
  CHECK_NEAR("replay ω matches firmware (rad/s)", maxDOmega, 0, 1e-3);
  CHECK_NEAR("replay pivot matches firmware (m)", maxDPivot, 0, 1e-3);
  CHECK_NEAR("same stroke count", replayStrokes, directStrokes, 0);
  CHECK_NEAR("replay stroke power = direct (W)", replayMean, directMean, 0.1);
  CHECK_NEAR("replay stroke power vs truth (W)", replayMean, truePower,
             0.05 * truePower);
  CHECK_NEAR("power split from logged impeller", replaySplit, 1, 0);

  // --- A version-1 log (before bent shafts) still opens, as straight ----
  {
    std::string v1 = std::string(path) + ".v1";
    logfmt::LogHeader old = h;
    old.version = 1;
    old.headerSize = logfmt::V1_HEADER_SIZE;
    std::FILE *g = std::fopen(v1.c_str(), "wb");
    std::fwrite(&old, logfmt::V1_HEADER_SIZE, 1, g);
    logfmt::LogRecord rr = {};
    rr.flags = logfmt::F_IMU1_VALID;
    for (int i = 0; i < 5; i++) std::fwrite(&rr, sizeof rr, 1, g);
    std::fclose(g);
    LogReader v1reader;
    std::string e;
    bool ok = v1reader.open(v1.c_str(), e);
    CHECK_NEAR("version-1 log opens", ok, 1, 0);
    CHECK_NEAR("version-1 log reads as straight paddle", v1reader.header().bladeBendDeg, 0, 0);
    int nrec = 0;
    while (ok && v1reader.next(rr)) nrec++;
    CHECK_NEAR("version-1 records aligned after shorter header", nrec, 5, 0);
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
