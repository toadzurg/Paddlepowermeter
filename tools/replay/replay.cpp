// Re-runs the firmware's PowerModel over an SD-card log and prints one CSV
// row per stroke.
//
//   ./tools/replay/build.sh
//   ./tools/replay/replay PPM00001.BIN > strokes.csv
//
// Options override the calibration stored in the log header:
//   --counts-per-n X   force calibration
//   --zero N           force zero offset (ADC counts)
//   --ppm X            impeller pulses per metre
//
// Geometry and filter tuning come from src/Config.h as compiled, so edit
// that and rebuild to try different settings on the same recording.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "Config.h"
#include "LogReplay.h"

int main(int argc, char **argv) {
  const char *path = nullptr;
  bool haveCal = false, haveZero = false, havePpm = false;
  float cal = 0, ppm = 0;
  long zero = 0;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--counts-per-n") && i + 1 < argc) {
      cal = std::strtof(argv[++i], nullptr); haveCal = true;
    } else if (!std::strcmp(argv[i], "--zero") && i + 1 < argc) {
      zero = std::strtol(argv[++i], nullptr, 10); haveZero = true;
    } else if (!std::strcmp(argv[i], "--ppm") && i + 1 < argc) {
      ppm = std::strtof(argv[++i], nullptr); havePpm = true;
    } else if (argv[i][0] != '-' && !path) {
      path = argv[i];
    } else {
      std::fprintf(stderr, "usage: replay LOG.BIN [--counts-per-n X] "
                           "[--zero N] [--ppm X]\n");
      return 2;
    }
  }
  if (!path) {
    std::fprintf(stderr, "usage: replay LOG.BIN [options]\n");
    return 2;
  }

  LogReader reader;
  std::string err;
  if (!reader.open(path, err)) {
    std::fprintf(stderr, "%s: %s\n", path, err.c_str());
    return 1;
  }
  const logfmt::LogHeader &h = reader.header();

  // The model's geometry is compiled in; flag recordings made with other
  // settings, since their results won't match the paddle's.
  if (std::fabs(h.imu2PosM - cfg::IMU2_POS_M) > 1e-4f ||
      std::fabs(h.bladeCopPosM - cfg::BLADE_COP_POS_M) > 1e-4f ||
      std::fabs(h.pivotPriorPosM - cfg::PIVOT_PRIOR_POS_M) > 1e-4f ||
      std::fabs(h.bladeBendDeg - cfg::BLADE_BEND_DEG) > 1e-3f ||
      std::fabs(h.shaftBendPosM - cfg::SHAFT_BEND_POS_M) > 1e-4f)
    std::fprintf(stderr, "note: log geometry differs from src/Config.h "
                 "(log: IMU2 %.3f, throat %.3f, blade %.3f, pivot prior %.3f m, "
                 "bend %.1f deg)\n", h.imu2PosM, h.shaftBendPosM, h.bladeCopPosM,
                 h.pivotPriorPosM, h.bladeBendDeg);

  SampleBuilder builder(h);
  builder.setForceCalibration(haveZero ? (int32_t)zero : h.forceZeroOffset,
                              haveCal ? cal : h.forceCountsPerN);
  if (havePpm) builder.setPulsesPerM(ppm);

  PowerModel model;
  logfmt::LogRecord r;
  bool first = true;
  uint16_t lastCount = 0;
  long records = 0, strokes = 0, measured = 0;
  double sumPower = 0, maxDOmega = 0, maxDPivot = 0;

  std::printf("stroke,time_s,power_w,rate_spm,energy_j,pivot_m,lever_m,"
              "useful_w,slip_w,efficiency\n");
  while (reader.next(r)) {
    PaddleSample s = builder.next(r);
    if (first) model.begin(s);
    first = false;
    model.update(s);
    records++;

    // With unchanged settings the replay should track the firmware closely.
    maxDOmega = std::fmax(maxDOmega, std::fabs(model.angularVelocity() - r.omega));
    maxDPivot = std::fmax(maxDPivot, std::fabs(model.pivotPosition() - r.pivotPosM));

    if (model.strokeCount() != lastCount) {
      lastCount = model.strokeCount();
      strokes++;
      // A stroke's power needs the previous stroke to time the cycle, so the
      // first stroke (and the first after a pause) reports 0: skip those.
      if (model.strokePower() > 0) {
        sumPower += model.strokePower();
        measured++;
      }
      std::printf("%u,%.3f,%.1f,%.2f,%.1f,%.4f,%.4f,", model.strokeCount(),
                  builder.elapsedSeconds(), model.strokePower(),
                  model.strokeRate(), model.strokeEnergy(),
                  model.pivotPosition(), model.leverArm());
      if (model.hasPowerSplit())
        std::printf("%.1f,%.1f,%.4f\n", model.usefulPower(), model.slipPower(),
                    model.efficiency());
      else
        std::printf(",,\n");
    }
  }

  std::fprintf(stderr,
               "%ld records (%.1f s), %ld strokes, mean stroke power %.1f W "
               "(over %ld timed strokes)\n"
               "replay vs firmware: max |Δω| %.4f rad/s, max |Δpivot| %.4f m\n",
               records, builder.elapsedSeconds(), strokes,
               measured ? sumPower / measured : 0.0, measured, maxDOmega,
               maxDPivot);
  return 0;
}
