# Paddle Power Meter

ESP32 canoe paddle power meter. It measures blade force with a strain gauge
and paddle motion with two IMUs on the shaft, estimates the lever arm
dynamically, and broadcasts power and stroke rate over the standard Bluetooth
Cycling Power profile, so a bike computer or training app can pair with it
like a cycling power meter.

**How it fits together:** [the project map](https://toadzurg.github.io/Paddlepowermeter/architecture.html)
(`docs/architecture.html`) draws the whole system, how power is computed, the
web app and the simulator's physics engine, with charts of where the power
goes and how big each part of the code is.

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
P = F · ω · L        L = d + (x_throat − x_pivot) · cos β
```

* **F**: force at the blade's centre of pressure, normal to the blade face,
  from the strain gauge.
* **ω**: shaft angular velocity about the stroke axis.
* **L**: distance from the pivot to the line the blade force acts along.
  The pivot is the point on the shaft that stays still relative to the boat,
  usually near the top hand. On a bent shaft (bend β, blade centre d past the
  throat) that line is turned by β, so L is shorter than the distance to the
  blade itself: about 1.7% at 12°. For a straight paddle (β = 0) it is simply
  pivot to blade centre.

The forward share of the blade force is likewise taken from the blade's
angle, not the shaft's: `cos(θ − β)`. That is the point of a bent shaft: the
blade is vertical while the shaft leans aft by β.

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
   * `SHAFT_BEND_POS_M` (the throat, where the blade leaves the shaft)
   * `BLADE_COP_FROM_BEND_M` (throat to the middle of the blade, along the blade)
   * `BLADE_BEND_DEG` (bent-shaft angle, e.g. 12; 0 for a straight paddle)
   * `PIVOT_PRIOR_POS_M` (where your top hand usually is)
2. Check the IMU axes. With the identity map, each chip's X axis points at
   the blade and its Y axis points the way the power face moves. If yours
   are mounted differently, edit `IMU1_AXES` / `IMU2_AXES`.
3. Build and flash: `pio run -t upload`, then `pio device monitor`. The
   project uses the "minimal SPIFFS" partition (1.9 MB app), because BLE and
   Wi-Fi together don't fit the default 1.2 MB.
4. Calibrate the force sensor once. The values are stored in flash.
   * Lay the paddle flat and type `tare`.
   * Support the paddle at the pivot. Hang a known mass at the blade's
     centre of pressure and type `cal <newtons>`, e.g. `cal 49.03` for
     5 kg.
5. Each time it boots, lay the paddle still for about 2 s. In that time it
   removes gyro bias and re-zeros the strain gauge, whose offset drifts
   with temperature.

Serial commands: `tare`, `cal <N>`, `debug` (10 Hz telemetry), `status`,
`imu` (one reading of both IMUs, for checking axes), and the `boat ...` and
`log ...` commands below.

Before the first outing, work through the
[bench test checklist](docs/BENCH_TEST.md).

## Boat speed (optional impeller)

Boat speed through the water is optional; the paddle measures power without
it. It's needed to split power into useful (force × boat speed) and wasted
(slip). GPS can't stand in for it on moving water, because GPS measures
speed over the ground, not through the water.

The impeller is on the hull and the paddle can't be wired to it, so a
second ESP32, the **boat node**, reads it and broadcasts over ESP-NOW (the
ESP32's built-in peer-to-peer radio, which runs alongside the paddle's
Bluetooth).

```
impeller magnets → Hall switch → boat node ESP32 ──ESP-NOW 20 Hz──► paddle ESP32
```

1. **Wiring.** Put a 3.3 V digital Hall switch (e.g. TI DRV5032) next to the
   impeller's magnets. Connect its output to GPIO 4, VCC to 3V3 and GND to
   GND. The internal pull-up handles open-drain outputs. If you use a reed
   switch instead, raise `MIN_INTERVAL_US` in `src/boat/BoatNode.cpp` to
   about 2000 to absorb contact bounce.
2. **Flash the boat node:** `pio run -e boat -t upload`. Spin the impeller by
   hand and watch the pulse count rise in `pio device monitor`.
3. **Pair**, away from other boats: on the paddle, type `boat pair`. The
   paddle then ignores every other boat node, e.g. at a race start.
4. **Calibrate**, on still water: type `boat cal start`, paddle a measured
   distance, then `boat cal <metres>`. On moving water, do it out and back
   and enter the total distance, so the current mostly cancels.

Impellers add drag and catch weeds, so fit it for training and take it off
for racing. Check your race's rules on speed devices.

### Useful vs slip power

With a calibrated impeller, each stroke's power is split in two. The
`debug` output shows both parts and the efficiency:

```
total  = F · v_blade                (what you put out; sent over BLE)
useful = F · cos(θ − β) · v_boat    (what pushes the boat forward)
slip   = total − useful             (lost churning water)
efficiency = useful / total
```

`cos(θ − β)` is the horizontal share of the blade force, taken from the paddle's
orientation. At the catch and the exit the angled blade partly lifts or
sinks the boat rather than driving it forward. The calculation assumes the
blade moves straight aft, so sweep and steering strokes read slightly high.
A stroke gets no split if boat speed dropped out at any point during its
drive.

Efficiency is not capped at 100%. A reading above 100% means a calibration
(force, lever arm or impeller) is off.

### Impulse per stroke

Every stroke also gets its forward impulse: the push along the boat's
direction of travel, added up over the drive. It needs no impeller:

```
impulse    = ∫ F · cos(θ − β) dt   over the drive, N·s
duty       = drive time / stroke time
mean push  = impulse / stroke time  (= the boat's drag at steady speed)
```

At the same stroke rate, more impulse means more speed. A long, gentle
drive can carry more impulse than a short, hard one at the same power.
`debug`, the app and `replay` all show these.

## Web app

`docs/app/index.html` is a phone and laptop dashboard that connects to the
paddle over Bluetooth. Your bike computer can stay connected at the same
time (up to three devices).

- **Paddle tab:**
  - stroke power, with a 5-stroke average
  - stroke rate, boat speed, efficiency (useful vs slip) and lever arm
  - impulse per stroke, drive time and duty, and mean push
  - a stroke view: an animated side view of the paddle with the live pivot,
    lever arm and blade force, drawn from the shaft angle the paddle sends
  - a force curve: the latest stroke, with the previous four behind it.
    **Live** draws the stroke as you paddle, with a dot and a "now" force
    readout; switch it off to hold the last completed stroke still for
    reading (the choice is remembered)
  - power, force, shaft rate, pivot, battery and SD-log status right now
- **Setup tab:** buttons for tare, force calibration (enter the mass in kg),
  boat pairing and calibration, and SD logging, plus a console for any
  serial command.
- **Try demo** shows the whole interface with simulated strokes, no paddle
  needed. The demo's pivot wanders between strokes the way the paddle's
  estimate does, and power follows the changing lever arm.

**Canoe Race Simulator** (`docs/app/simulator.html`, linked from the app)
compares two strokes, Boat A and Boat B. Pick a boat, start from a preset,
and move the sliders.
- **Boat A vs Boat B:** a table led by efficiency, the share of your power
  that moves you down the course after blade slip, switching, the zigzag
  between switches, sideways drag and the boat surging and bobbing; where
  the power goes; and which way the blade pushes.
- **Force curve:** both boats through the drive. Hover for values, or turn
  on Live to trace it with the animation.
- **Stroke views:** side, end and top, split or overlaid.
- **Blade:** bend, pitch, and lift on or off. With lift, a sweeping blade
  gains a lot; that's untested on the water, so treat it with care.
- **Switching sides:** by heading, or a fixed number of strokes per side.
  Too few loses time switching; too many zigzags.
- **Fit:** seat height, paddle length and hand placement for the most speed
  at what you can hold, within reach, the top hand's height and stability.
  A 3D model of the boat, paddle and paddler turns and zooms; tap a part
  (seat, paddle, blade, hands, arms, torso, legs, stroke) to zoom in on it
  and set it, and compare yours with the suggested fit.
- **Race:** A against B, point to point or there and back, with wind,
  current, shallow water, or a river with bends. It also shows each boat's
  force per stroke at its race effort. Drag the slider under the race to
  replay any moment: a chart of speed along the course, and a table of
  time, distance, speed, power, stroke rate, strokes, distance per stroke,
  switches, energy and the gap. After a run it replays what happened,
  including any changes you made mid-race.

Hull drag comes from a coast-down test of a USCA C-1. The rest follows
standard hydrodynamics. It's a simplified model: use it to compare
strokes, not to predict exact times.

**The body model and its evidence.** With gearing on (Paddler profile),
your arms and your trunk each hold the most power at one speed, from Hill's
force–velocity relation: power peaks at an optimal shortening speed, and
where it peaks depends on fibre type. The arms' speed is the lower hand's
along the pull, against its shoulder; the trunk's is that shoulder's own.
The trunk's curve scales what trunk rotation and a hinge add. Both best
speeds are assumed: calibrated so the default stroke (the trunk's with full
rotation) is at its best at 60 spm, until the paddle measures yours. What
the model draws on, from other paddle and pole sports (no study yet
measures a C1 paddler's trunk on its own):

- The trunk is a large power source: in rowing, legs ~43–45%, trunk
  ~32–36%, arms ~21–24% of power ([Kleshnev, power in rowing](https://ojs.ub.uni-konstanz.de/cpa/article/view/2258);
  [rowing styles](https://www.row2k.com/features/5531/rowing-styles-analysis-and-optimisation-by-dr-valery-kleshnev/));
  another method puts trunk swing at ~49% ([Tachibana](https://coachsci.sdsu.edu/csa/vol111/tachiban.htm)).
  In double poling, trunk and legs give about a third of propulsive power
  on a moderate incline ([Danielsen et al., via Chalmers](https://research.chalmers.se/publication/553157/file/553157_Fulltext.pdf)).
- The trunk works early in the drive, the arms late: legs and trunk move
  mostly in the drive's first two-thirds in rowing; in double poling the
  trunk and hip flexors fire before the shoulder and elbow extensors
  ([Holmberg](https://openarchive.ki.se/xmlui/handle/10616/43443)).
- Trunk rotation adds force in kayaking: a rotating seat raised ergometer
  paddle force from 320 to 465 N (Petrone et al. 2006); power correlates
  with trunk rotation in elite kayakers ([para- and able-bodied kayakers](https://lida.sport-iat.de/ta/Record/4053509?lng=en)),
  though on the water the lower trunk acts as a stable base ([Brown et al. 2024](https://johk.pl/wp-content/uploads/2024/01/jhk-2024-90-01.pdf)).
- Trunk muscles are mostly slow fibres (erector spinae ~55–58% type I;
  abdominals 55–58%: [abdominal fibre types](https://pubmed.ncbi.nlm.nih.gov/161688/)),
  the triceps' lateral head 32–40%; slow fibres reach peak power at a
  lower speed ([Rome et al. 1988](https://www.nature.com/articles/335824a0)),
  so the trunk's best speed needn't be the arms'. Isokinetic trunk rotation
  still gains power up to 120°/s ([isokinetic axial rotation, 2014](https://pmc.ncbi.nlm.nih.gov/articles/PMC3996988/)): its peak is broad.
- Lower rates are more economical below race pace in double poling
  ([Lindinger & Holmberg 2011](https://www.springermedicine.com/how-do-elite-cross-country-skiers-adapt-to-different-double-poli/21068888)),
  and the power-best rate exceeds the efficiency-best one by 1.7–2.4×
  depending on fibre type in cycling (Kohler & Boutellier).

**The arms.** Each elbow goes where that arm is strongest. A joint's torque
is the force times its distance from the force's line
([moment arms](https://isophit.substack.com/p/why-force-directionnot-positiondetermines)),
so as the blade loads, each elbow moves toward the line its hand pushes or
pulls along (the paddle as a lever gives each hand's force). The top hand
is the fulcrum, pushing forward at about head height: its elbow comes up
and the arm lines up from shoulder to grip, near straight, since pushing is
strongest toward full extension (an ascending strength curve, as in a
[press](https://powerliftingtechnique.com/bench-press-lockout/)). The lower
hand pulls aft, low: its elbow drops, bent near 90° by the exit, where the
elbow flexors' torque peaks ([torque–angle data](https://personal.fbw.vu.nl/~jsmeets/2010/Torque_angle-JEK10.pdf)).
Straighter than that it's no weaker: a straight pulling arm is a link the
shoulder's extensors (the lats; extension is the shoulder's strongest
movement, [Int J Sports Med 1994](https://www.thieme-connect.de/products/ejournals/pdf/10.1055/s-2007-1021105.pdf))
and the trunk pull through, as in a sweep or rowing's straight-arm drive.
Only folded past a right angle, or reaching past reach, does it weaken.
With no force (catch, exit, recovery) the top elbow stays up and out, the
lower one hangs.

**The recovery.** No force goes into the water, so the best recovery is the
one that costs least while it carries the blade clear and arrives set for
the catch. The model describes a recovery by a point on the shaft that
travels smoothly from the exit to the catch (the pivot: at the top hand, the
lower hand, or between), how far it rises and comes in mid-carry, and how
much of the shaft's turn is the slice out. It costs each from the energy of
the moving arms and paddle (Winter's internal work), lifting them, and air
drag on the blade, and paddles the cheapest that stays in reach with the top
hand in front of the face. Inputs:

- Each arm is about 5% of body mass: upper arm 2.71%, forearm 1.62%, hand
  0.61% (men; [de Leva 1996](https://ebm.ufabc.edu.br/wp-content/uploads/2013/12/Leva-1996.pdf)),
  with its centre of mass and radius of gyration. A bent-shaft paddle is
  about 240–330 g ([ZRE weights](https://paddling.com/gear/zre-or-light-10-oz-carbon-fiber-canoe-paddle-paddle)).
- Muscle does negative work at 3–5 times positive work's efficiency
  ([walking review](https://pmc.ncbi.nlm.nih.gov/articles/PMC1283673)): it
  costs a quarter as much.
- A flat plate across the flow has a drag coefficient of about 1.2 (Hoerner);
  feathered, the blade goes edge first.
- Moving limbs faster costs more: the reason cyclists' most economical
  cadence is lower than their preferred one ([cadence review](https://researchonline.ljmu.ac.uk/id/eprint/10294/1/Accepted%20PDF.pdf)).

What it finds: keep the top hand steady, a fulcrum in front of the face,
and let the lower hand swing the blade. About a point near the top hand the
lower hand moves about half as far as the blade; about the lower hand, the
top hand moves a quarter further than the blade, and the arms (~4 kg each)
outweigh the paddle about 25 times. A top hand that drops and swings over
the lower hand costs about twice as much and can't stay in reach. The
recovery has barely been studied in any paddle sport (rowing calls it
[a grey area](https://www.row2k.com/features/5722/the-biomechanics-of-the-recovery-phase-by-dr-valery-kleshnev/)),
so this is mechanics, not measurement. Its costs compare styles: the model
starts and stops the hands at the exit and catch, so their size is an upper
bound, and they don't change what you can hold.

**Opening it:**
- iPhone: open the link in the free **Bluefy** browser. Safari, Chrome and
  Edge on iPhone can't use Bluetooth.
- Laptop: Chrome or Edge.

The page is served by GitHub Pages at
`https://toadzurg.github.io/Paddlepowermeter/app/`. To enable that, go to the
repository's Settings → Pages, choose "Deploy from a branch", then `main` and
`/docs`.

**PIN:** viewing data needs no PIN. Changing anything (tare, calibration,
pairing, logging) needs the paddle's 6-digit PIN, so nobody nearby can tare
your paddle at a race start. Type `pin` in the serial console to see it, or
`pin 123456` to change it. The browser remembers it. The PIN is a guard
against accidents and pranks, not strong security: Bluetooth here is
unencrypted, so someone with a radio sniffer could capture it.

The Bluetooth protocol is defined in `src/BleProtocol.h`.

## Raw sensor logging

With a microSD card fitted, the paddle records every sample (100 per second)
to the card. Recordings let you check the power model against real strokes
and different paddlers, and re-run old sessions after improving the code or
fixing a calibration.

**Hardware:** a microSD breakout on the ESP32's default SPI pins:

| SD pin | ESP32 |
|---|---|
| CS | GPIO 5 (`LOG_SD_CS_PIN` in `src/Config.h`) |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |
| VCC / GND | 3V3 / GND |

Use a FAT32-formatted card. Logging starts automatically at boot when a card
is present (`LOG_AUTOSTART`), and each session gets its own file:
`PPM00001.BIN`, `PPM00002.BIN`, ... A file holds about 29 MB per hour. At
most about 1 s of data is lost if the power is cut.

Serial commands: `log start`, `log stop`, and `log` (status). Re-running
`tare`, `cal` or `boat cal` while logging starts a new file, so each file's
header matches the calibration its data was recorded with.

**What's recorded:**
- Every sensor reading as the chips report it: each IMU's own axes, before
  gyro-bias removal, and raw force ADC counts.
- A header with the geometry, axis maps, gyro bias and calibration in force
  at the time.
- A few of the firmware's own outputs, to compare against an offline replay.

Because the readings are raw, a recording made with a wrong axis map or
calibration can still be re-processed correctly. The binary format is
defined in `src/LogFormat.h`.

**Tools (on a PC):**

```
python3 tools/log2csv.py PPM00001.BIN          # → PPM00001.csv for a spreadsheet/plotting
./tools/replay/build.sh                        # once
./tools/replay/replay PPM00001.BIN > strokes.csv
./tools/replay/replay PPM00001.BIN --counts-per-n 1180 --ppm 7.4   # try another calibration
```

`replay` runs the same `PowerModel` the paddle runs and prints one CSV row
per stroke: power, rate, energy, pivot, lever arm, the useful/slip split
when the impeller was logged, and impulse, drive time, duty and mean push. It also reports how closely it matches what
the paddle computed live. To try different geometry or filter tuning on the
same recording, edit `src/Config.h` and rebuild.

## Tests

The signal processing is tested on a PC against a physics simulation. The
simulation models a rigid paddle rotating about a pivot fixed in the boat,
with gravity, boat surge, sensor noise and gyro bias:

```
./test/host/run.sh
./test/web/run.sh     # web app in headless Chromium; needs Node + Playwright
```

Scenarios: nominal, a wrong pivot prior, boat surge, 75 spm race pace, IMU2
failure, and the paddler stopping. The impeller pulse timing has its own
tests, including the microsecond counter wrapping every 71 minutes during
long races. An end-to-end test writes a simulated session as a real log file
and checks that replaying it reproduces the live result exactly. The file
uses a scrambled axis map, gyro bias, raw counts, the timer wrap and a torn
final record.

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
  BLEManager.{h,cpp}     Cycling Power + Battery + app services
  BleProtocol.h          app Bluetooth packet format (shared with tests)
  BoatLink.{h,cpp}       receives boat speed from the boat node (ESP-NOW)
  boat/BoatNode.cpp      boat node firmware (impeller reader)
  Logger.{h,cpp}         microSD raw sensor logging (background writer)
  LogFormat.h            log file format (shared with tools/)
lib/BoatSpeed/           impeller pulse timing + radio packet (shared)
lib/                     vendored sensor libraries
tools/log2csv.py         log → CSV
tools/replay/            re-run the power model over a log
docs/app/index.html      web app (Bluetooth dashboard and setup)
docs/app/simulator.html  canoe race simulator (physics in stroke-sim.js)
docs/architecture.html   project map: system, front end, physics engine
docs/BENCH_TEST.md       hardware bench test checklist
test/host/               PC simulation tests
test/web/                browser test of the web app (simulated paddle)
```
