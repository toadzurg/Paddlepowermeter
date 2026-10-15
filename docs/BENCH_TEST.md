# Bench test checklist

Work through this before the first paddle on the water. Each section builds
on the one before. Tick each box, and note the measured values in the record
sheet at the end, so later sessions can be compared against them.

Commands are typed in the serial monitor (`pio device monitor`, 115200 baud).
Text in `code` is what the firmware prints.

## What you need

- [ ] USB cable and a laptop with PlatformIO
- [ ] Two known weights, e.g. 5 kg and 10 kg (water jugs weighed on a kitchen
      scale are fine), plus a strap to hang them
- [ ] A clamp or vise for the shaft, and a tape measure
- [ ] A phone with **nRF Connect** (free, iOS/Android), and a bike computer or
      training app that pairs with Bluetooth power meters (optional)
- [ ] A FAT32 microSD card (32 GB or smaller)
- [ ] The boat node and impeller, if you're using boat speed

---

## 1. Power-up and sensors

1. [ ] Lay the paddle flat and still. Flash with `pio run -t upload`, then
       open the monitor.
2. [ ] At boot, none of these appear:
       `NAU7802 not found`, `IMU1 not found`, `IMU2 not found`,
       `MAX17048 not found`, `paddle moving at boot`.
       If one does, check that sensor's I²C wiring (SDA 21, SCL 22, 3V3,
       GND) and its address (Config.h: IMUs at 0x68 and 0x69).
3. [ ] Type `status`. Expect `IMU1 ok, IMU2 ok`.
4. [ ] Expect a plausible `battery NN%`. Charge the battery fully once; it
       should then read 95–100%.

## 2. Geometry

Measure along the shaft. Positions are measured from **IMU1**, positive
toward the blade, and must match `src/Config.h`:

5. [ ] `IMU2_POS_M`: IMU1 to IMU2. It must be toward the blade, with at
       least 0.2 m between them; 0.3 m or more is better.
6. [ ] `SHAFT_BEND_POS_M`: IMU1 to the throat, where the blade leaves the
       shaft.
   [ ] `BLADE_COP_FROM_BEND_M`: throat to the middle of the blade (the
       centre of pressure), measured along the blade.
   [ ] `BLADE_BEND_DEG`: the bend angle (e.g. 12 for a 12° bent shaft, 0
       for a straight paddle).
7. [ ] `PIVOT_PRIOR_POS_M`: IMU1 to where your top hand usually grips.
       Negative if your hand is above IMU1.
8. [ ] If any value changed, edit `Config.h` and re-flash.

## 3. IMU axes (the most important check)

The paddle frame is:
- **X** along the shaft toward the blade
- **Y** the direction the power face points
- **Z** = X × Y

Hold each pose still, type `imu`, and check the **paddle** lines of both
IMUs. Readings are specific force, so gravity shows as +9.81 pointing *up*.

| Pose | Expected paddle accel |
|---|---|
| A. Shaft vertical, blade **down** | x ≈ **−9.8**, y ≈ 0, z ≈ 0 |
| B. Shaft horizontal, power face **facing up** | y ≈ **+9.8**, x ≈ 0, z ≈ 0 |
| C. Shaft horizontal, blade pointing to your **right**, power face facing **away** from you | z ≈ **+9.8**, x ≈ 0, y ≈ 0 |

9. [ ] Pose A correct on IMU1 and IMU2
10. [ ] Pose B correct on IMU1 and IMU2
11. [ ] Pose C correct on IMU1 and IMU2

**If a pose is wrong**, fix that IMU's map in `Config.h` (`IMU1_AXES` /
`IMU2_AXES`) using the **chip** line in the same pose:
1. Find the chip axis that reads about ±9.8. That's `axis[i]` for the paddle
   axis this pose tests (x = 0, y = 1, z = 2).
2. Set `sign[i]` = +1 if the chip value has the same sign as the expected
   paddle value, otherwise −1.

For example, if pose A (expect x = −9.8) shows chip `y 9.79`, set
`axis[0] = 1, sign[0] = -1`. Re-flash and repeat all three poses.

12. [ ] **Rotation sign.** Type `debug`. In pose A, sweep the blade quickly
        in the direction the power face points. `w` must go **positive**,
        to at least +2 rad/s. If it goes negative, the Z mapping is wrong:
        re-check pose C. Type `debug` again to stop.
13. [ ] **Gyro at rest.** Lay the paddle still. `imu` paddle gyro values are
        all within ±0.01 rad/s.

## 4. Force sensor

Clamp the shaft at your top-hand position with the power face facing up,
and the blade sticking out free. Tilt the paddle so the **blade face** is
level. On a bent shaft that means the shaft is tilted by the bend angle
(e.g. 12°). That way the weight pushes straight into the blade, the same way
the water does.

14. [ ] Unloaded, type `tare`. Expect `tared: zero offset ...`.
15. [ ] With `debug` on, `F` reads 0 ± 1 N and jitters by less than ±0.5 N.
16. [ ] Hang the 5 kg weight with its strap centred on the blade's
        **centre of pressure**, then type `cal 49.03`. Use 9.807 × the
        actual mass in kg. Expect `calibrated: ... counts/N`.
17. [ ] **Sign:** with the weight on, `F` reads **positive**, about +49 N.
18. [ ] **Linearity:** with the 10 kg weight, `F` is within ±2% of 98.1 N.
19. [ ] **Creep and return:** leave the 10 kg on for 60 s. `F` drifts less
        than 1 N. Remove it and `F` returns to 0 ± 1 N.
20. [ ] **Blade self-weight:** unload, unclamp, and slowly turn the paddle
        through all orientations. `F` stays within ±3 N. The firmware
        re-zeros at each boot in whatever pose the paddle lies in.
21. [ ] Reboot and type `status`: the calibration survived, with no
        `UNCALIBRATED` shown.

## 5. Stroke detection and Bluetooth

22. [ ] Clamp as in section 4. Press firmly on the power face (more than
        25 N, about 3 kg of push) for about half a second, then release
        fully. Repeat every 1.2 s, about 10 times. With `debug` on, the
        stroke rate settles at about 50 spm. `stroke` stays near 0 W,
        because the clamped shaft isn't rotating.
23. [ ] Stop pressing. After 4 s the stroke rate drops to 0.
24. [ ] In **nRF Connect**, scan: `PaddlePower` appears and advertises
        Cycling Power (0x1818) and Battery (0x180F). Connect.
25. [ ] Read Cycling Power Feature (0x2A65): `08-00-00-00`. Read Sensor
        Location (0x2A5D): `00`.
26. [ ] Subscribe to Cycling Power Measurement (0x2A63). One 8-byte
        notification arrives per second, starting `20-00`. Bytes 5–6
        (stroke count) increase as you repeat step 22.
27. [ ] Battery Level (0x2A19) matches the `status` battery %.
28. [ ] Disconnect, then reconnect. It reconnects without a reboot.
29. [ ] (Optional) Pair with a bike computer or app as a power meter. Power
        shows 0 W, and cadence shows about 50 while you repeat step 22.

## 6. Live power sign

30. [ ] Unclamp. Hold the paddle in pose A. Have a helper press on the power
        face while you sweep the blade in the power-face direction. With
        `debug` on, `P` (instantaneous power) goes **positive**. If it goes
        negative, recheck steps 12 and 17: one of the signs is wrong.

## 7. Boat node and impeller (skip if not fitted)

31. [ ] Flash the boat node: `pio run -e boat -t upload`. Open its monitor.
        Expect `boat node <MAC>, ESP-NOW ok on channel 1`.
32. [ ] Turn the impeller exactly 10 revolutions by hand. `pulses` rises by
        a whole multiple of 10. Pulses per revolution must be the same on
        every try; note it in the record sheet.
33. [ ] Spin it fast by hand: `glitches 0` and `overflow 0`. If glitches
        appear, move the Hall sensor closer to the magnets, or raise
        `MIN_INTERVAL_US` for a reed switch.
34. [ ] Stop spinning. `rate` falls to 0 within 1.5 s.
35. [ ] Move away from any other boat node. On the paddle, type `boat pair`,
        then expect `paired with boat node XX:XX:...`.
36. [ ] Paddle `status` shows `boat node paired, receiving`.
37. [ ] Reboot the paddle. It is still paired, without pairing again.
38. [ ] **Range:** with the boat node inside the hull where it will be
        mounted, carry the paddle 3 m away while spinning the impeller.
        `debug` keeps showing the boat Hz, with no `--`.
39. [ ] **Radio sharing:** connect nRF Connect to the paddle (step 24) and
        repeat step 38. Boat speed is still received.
40. [ ] Power the boat node off. Within a second the paddle `debug` shows
        `boat --`.
41. [ ] Impeller calibration (`boat cal start` / `boat cal <m>`) needs water.
        Do it on the first outing, on still water.

## 8. SD logging

42. [ ] Insert the card and reboot. `no SD card` does **not** appear.
43. [ ] Type `log`: `log: recording /PPMnnnnn.BIN`, with the records count
        rising about 100 per second.
44. [ ] **Under load:** run for 10 minutes with nRF Connect subscribed (step
        26) and the boat node sending (step 38). `log` shows `0 dropped`
        and no `WRITE ERROR`.
45. [ ] `tare` while logging moves the log to a new file: the `log` path
        number goes up by one.
46. [ ] **Power cut:** pull the power while logging. The file on the card
        still converts; at most the last second is missing.
47. [ ] `log stop`, then move the card to the PC and check:
        - `python3 tools/log2csv.py PPMnnnnn.BIN` converts without errors.
        - The sample timing is steady:
          ```
          python3 -c "import csv,statistics as s; t=[float(r['time_s']) for r in csv.DictReader(open('PPMnnnnn.csv'))]; d=[b-a for a,b in zip(t,t[1:])]; print(f'median {s.median(d)*1e3:.1f} ms, max {max(d)*1e3:.1f} ms')"
          ```
          Expect a median of 10.0 ms and a maximum under 30 ms.
        - `./tools/replay/replay PPMnnnnn.BIN` reports a `replay vs
          firmware` difference near 0.

## 9. Before the water

48. [ ] Every enclosure is sealed. The battery, SD card and USB port are
        reachable but watertight.
49. [ ] No wire crosses the strain-gauge area, where shaft flex would pull
        on it.
50. [ ] The boat node is mounted and the impeller fin is secure. Plan to
        remove the impeller for races.

---

## Record sheet

| Item | Value | Date |
|---|---|---|
| IMU2 position (m) | | |
| Throat position (m) | | |
| Throat to blade centre (m) | | |
| Bend angle (°) | | |
| Top-hand position (m) | | |
| IMU axis maps changed? (values) | | |
| Force calibration (counts/N, from `status`) | | |
| Force noise, unloaded (± N) | | |
| 10 kg reading (N) / error (%) | | |
| Creep over 60 s (N) | | |
| Blade self-weight swing (± N) | | |
| Impeller pulses per revolution | | |
| Boat node MAC | | |
| Log timing: median / max (ms) | | |
| Records dropped in 10 min | | |
