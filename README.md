# Paddle Power Meter

ESP32 canoe paddle power meter. It measures blade force with a strain gauge
and paddle motion with two IMUs on the shaft, estimates the lever arm
dynamically, and broadcasts power and stroke rate over the standard Bluetooth
Cycling Power profile, so a bike computer or training app can pair with it
like a cycling power meter.

## Hardware

| Part | Role | I²C address |
|------|------|-------------|
| ESP32 | MCU + BLE | |
| NAU7802 | strain-gauge ADC (blade force) | 0x2A |
| BMI270 #1 | IMU near the top of the shaft | 0x68 |
| BMI270 #2 | IMU further down the shaft | 0x69 |
| MAX17048 | LiPo fuel gauge | 0x36 |

## How power is computed

```
P = F · v_blade        v_blade = ω · L        L = x_blade − x_pivot
```

* **F**: force at the blade's centre of pressure, from the strain gauge.
* **ω**: shaft angular velocity about the stroke axis.
* **L**: distance from the pivot (the point on the shaft that stays still
  relative to the boat, usually near the top hand) to the blade.

Per stroke, `∫P dt` over the drive is the stroke energy. **Stroke power** is
that energy divided by the full cycle time (drive + recovery), which is
what gets reported.

### Filtering

All of this lives in `PowerModel`, which is plain C++ with no Arduino
dependencies.

1. **ω and α, from a 2-state Kalman filter** (`RateKalman`, state `[ω, α]`,
   constant-acceleration model). It fuses both gyros and the rigid-body
   identity `(a₂ − a₁)_y = d·(α_z + ω_x·ω_y)`. Gravity is the same at both
   IMUs, so it cancels out of that difference. This gives α without
   numerically differentiating a noisy gyro.
2. **Orientation, from a Madgwick filter** (`Orientation`). It uses real `dt`
   and rad/s. Its accelerometer input is the specific force extrapolated to
   the estimated pivot, where motion terms are ~0, so it sees mostly
   gravity even mid-stroke.
3. **Pivot position, from a scalar Kalman filter** (`Kalman1D`). After gravity
   is removed, each IMU's acceleration is linear in the pivot position:
   `a_t = (α + ω_xω_y)(x − p)` and `a_r = −(ω_y² + ω_z²)(x − p)`. Those are
   linear measurements of `p` whose gain is set by how hard the paddle is
   rotating. Because of that, the filter only moves when the data is
   informative, and it ignores the paddle when it is nearly still. Updates
   only happen while the blade is loaded, and innovations beyond 3σ are
   rejected.

If IMU2 fails, the meter keeps running on IMU1 alone. It loses the
accelerometer-difference α measurement and some pivot accuracy.

## Setup

1. Measure the geometry and set it in `src/Config.h`. Positions are metres
   along the shaft from IMU1, positive toward the blade:
   * `IMU2_POS_M`
   * `BLADE_COP_POS_M` (roughly the middle of the blade)
   * `PIVOT_PRIOR_POS_M` (where your top hand usually is)
2. Check the IMU axes. With the identity map, each chip's X axis points at
   the blade and its Y axis points the way the power face moves. If yours
   are mounted differently, edit `IMU1_AXES` / `IMU2_AXES`.
3. Build and flash: `pio run -t upload`, then `pio device monitor`.
4. Calibrate the force sensor once. The values are stored in flash.
   * Lay the paddle flat and type `tare`.
   * Support the paddle at the pivot. Hang a known mass at the blade's
     centre of pressure and type `cal <newtons>`, e.g. `cal 49.03` for
     5 kg.
5. Each time it boots, lay the paddle still for about 2 s. In that time it
   removes gyro bias and re-zeros the strain gauge, whose offset drifts
   with temperature.

Serial commands: `tare`, `cal <N>`, `debug` (10 Hz telemetry), `status`.

## Tests

The signal processing is tested on a PC against a physics simulation. The
simulation models a rigid paddle rotating about a pivot fixed in the boat,
with gravity, boat surge, sensor noise and gyro bias:

```
./test/host/run.sh
```

Scenarios: nominal, a wrong pivot prior, boat surge, 75 spm race pace, IMU2
failure, and the paddler stopping.

## Layout

```
src/
  PaddlePowerMeter.ino   setup/loop, fixed 100 Hz scheduler, serial commands
  Config.h               geometry, axes, thresholds, filter tuning
  PowerModel.{h,cpp}     power / stroke estimation (hardware independent)
  Kalman.{h,cpp}         Kalman1D (scalar) and RateKalman ([ω, α])
  Orientation.{h,cpp}    Madgwick AHRS
  IMU.{h,cpp}            BMI270 driver → paddle frame, SI units
  Scale.{h,cpp}          NAU7802 driver → newtons
  Battery.{h,cpp}        MAX17048 state of charge
  BLEManager.{h,cpp}     Cycling Power + Battery services
test/host/               PC simulation tests
lib/                     vendored sensor libraries
```
