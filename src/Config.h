#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Paddle frame used by every algorithm in this project
//
//   +X  along the shaft, pointing from the grip toward the blade
//   +Y  the direction the power face moves during the drive
//   +Z  X × Y, the stroke rotation axis (+ω_z moves the blade toward +Y)
//
// All positions are metres along the shaft (+X), measured from IMU1.
// ---------------------------------------------------------------------------

namespace cfg {

// Main loop rate. The IMUs run at 200 Hz, so 100 Hz always gets a fresh sample.
constexpr float SAMPLE_HZ = 100.0f;

// --- Geometry (measure these on your paddle) -------------------------------
// Defaults are roughly a 52" (1.32 m) marathon bent-shaft paddle with a 20"
// blade and IMU1 about 22 cm below the grip.
constexpr float IMU2_POS_M          = 0.30f;  // IMU2 position (must be != 0)
constexpr float SHAFT_BEND_POS_M    = 0.60f;  // throat: where the blade leaves the shaft
constexpr float BLADE_COP_FROM_BEND_M = 0.25f;// along the blade, throat to its centre of pressure
// Bent-shaft angle. Positive: blade angled toward the bow when the power
// face faces you (the usual way to hold a bent shaft). 0 = straight paddle.
constexpr float BLADE_BEND_DEG      = 12.0f;
constexpr float PIVOT_PRIOR_POS_M   = -0.10f; // initial guess: about the top hand
constexpr float PIVOT_PRIOR_SIGMA_M = 0.30f;
constexpr float PIVOT_MIN_POS_M     = -0.60f; // physically plausible range
constexpr float PIVOT_MAX_POS_M     = 0.45f;

// Blade centre of pressure measured along the paddle (shaft + blade), for
// logs and display. The lever arm is NOT simply this minus the pivot when
// the shaft is bent; see PowerModel::leverArmFor().
constexpr float BLADE_COP_POS_M = SHAFT_BEND_POS_M + BLADE_COP_FROM_BEND_M;

// --- Sensor axis → paddle axis mapping -------------------------------------
// paddle[i] = sign[i] * sensor[axis[i]]. Identity means the chip's X points
// at the blade and its Y matches the power face direction.
struct AxisMap {
  uint8_t axis[3];
  int8_t  sign[3];
};
constexpr AxisMap IMU1_AXES = {{0, 1, 2}, {1, 1, 1}};
constexpr AxisMap IMU2_AXES = {{0, 1, 2}, {1, 1, 1}};

constexpr uint8_t IMU1_I2C_ADDR = 0x68;
constexpr uint8_t IMU2_I2C_ADDR = 0x69;

// --- Force / stroke detection ----------------------------------------------
// Force is the load at the blade centre of pressure, in newtons, positive
// when the power face pushes on the water. Calibrate with the serial
// "cal <newtons>" command; the factor below is only a fallback.
constexpr float DEFAULT_COUNTS_PER_N = 1000.0f;
constexpr float STROKE_START_N = 25.0f;  // hysteresis: drive starts above this
constexpr float STROKE_END_N   = 10.0f;  // ... and ends below this
constexpr uint32_t MIN_DRIVE_MS = 150;   // shorter force blips are ignored
constexpr uint32_t MIN_CYCLE_MS = 400;   // > 150 strokes/min is not real
constexpr uint32_t MAX_CYCLE_MS = 4000;  // longer gap = paddler stopped

// --- Kalman / AHRS tuning --------------------------------------------------
constexpr float GYRO_NOISE_RAD_S   = 0.01f;  // per-sample σ incl. shaft vibration
constexpr float ACCEL_NOISE_MS2    = 0.5f;   // per-axis σ incl. shaft flex
constexpr float RATE_JERK_PSD      = 1000.0f;// (rad/s³)²/Hz, how fast α may change
constexpr float LIN_ACCEL_NOISE_MS2 = 1.5f;  // boat surge + tilt error seen by pivot fit
constexpr float PIVOT_DRIFT_M2_S   = 0.05f;  // how fast the pivot may wander
constexpr float PIVOT_GATE_SIGMA   = 3.0f;   // reject pivot innovations beyond this
constexpr float AHRS_BETA          = 0.05f;  // Madgwick gain; low = trust gyro

constexpr float GRAVITY_MS2 = 9.80665f;

// --- Raw sensor log (microSD on the default SPI pins: SCK 18, MISO 19,
// MOSI 23) ---------------------------------------------------------------
constexpr int LOG_SD_CS_PIN = 5;
constexpr bool LOG_AUTOSTART = true;  // start logging at boot if a card is in

// --- Output ----------------------------------------------------------------
constexpr uint32_t BLE_NOTIFY_MS    = 1000;  // Cycling Power typical rate
constexpr uint32_t BATTERY_POLL_MS  = 10000;

}  // namespace cfg

#endif  // CONFIG_H
