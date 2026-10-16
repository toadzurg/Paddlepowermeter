// Tests for the stroke simulator: physics (docs/app/stroke-sim.js) in Node,
// then the page (docs/app/simulator.html) in headless Chromium.
//
//   node sim.test.cjs [screenshot dir]

const path = require("path");
const S = require("../../docs/app/stroke-sim.js");
const F = require("../../docs/app/fit-sim.js");
const { chromium } = require("playwright");

let failures = 0;
function check(name, ok, detail = "") {
  console.log(`  [${ok ? " ok " : "FAIL"}] ${name}${detail ? "  " + detail : ""}`);
  if (!ok) failures++;
}
const pct = (x) => `${(x * 100).toFixed(1)}%`;

// The page loads each engine file as <file>?v=<hash of the file>, so a
// changed engine is never served stale from a browser cache.
{
  const fs = require("fs"), crypto = require("crypto");
  const html = fs.readFileSync(path.resolve(__dirname, "../../docs/app/simulator.html"), "utf8");
  console.log("\nEngine cache tag");
  for (const file of ["stroke-sim.js", "fit-sim.js"]) {
    const hash = crypto.createHash("sha1").update(fs.readFileSync(path.resolve(__dirname, `../../docs/app/${file}`))).digest("hex").slice(0, 10);
    const tagged = html.includes(`${file}?v=${hash}`);
    check(`simulator.html loads ${file}?v=${hash}`, tagged, tagged ? "" : "update the ?v= in simulator.html");
  }
}
console.log("\nCanoe Race Simulator physics");
{
  const base = S.simulate({});
  const balance = base.hullPower + base.leewayPower + base.slipPower - base.paddlerPower;
  check("power balance: hull + leeway + slip = paddler", Math.abs(balance) < 1e-3 * base.paddlerPower,
        `residual ${balance.toExponential(1)} W`);
  check("thrust·speed = hull + leeway",
        Math.abs(base.usefulPower - base.hullPower - base.leewayPower) < 1e-3 * base.paddlerPower);
  const sh = base.at(150).shares;
  const total = sh.hull + sh.zigzag + sh.slip + sh.leeway + sh.surge + sh.sink + sh.bounce + sh.swing + sh.drop + sh.pitch;
  check("shares at 150 W add to 1", Math.abs(total - 1) < 1e-4, total.toFixed(6));
  check("efficiency is the hull share at the comparison effort", Math.abs(base.efficiency - sh.hull) < 1e-12);
  // The zigzag: power a straight-running boat would need for the same progress.
  const course = base.at(150).switching.course;
  check("efficiency counts the zigzag: progress down the course", sh.zigzag > 0 &&
        Math.abs(sh.hull / (sh.hull + sh.zigzag) - course ** 3) < 1e-12, `zigzag ${pct(sh.zigzag)} of your power`);
  check("surge costs a little", sh.surge > 0.0005 && sh.surge < 0.02, pct(sh.surge));
  // The Science of Paddling (Part 32) cites about ±10% speed swing.
  const swing = base.at(150).surgeSwing;
  check("speed swings 5–20% over a stroke at 60 spm", swing > 0.05 && swing < 0.2, pct(swing));
  // Switching aside: a switch takes the same time at any rate, so it costs a quick rate more (tested below).
  const slow = S.simulate({ rateSpm: 30, switchLossS: 0 }), quick = S.simulate({ rateSpm: 90, switchLossS: 0 });
  check("same effort, slower rate: more surge", slow.at(150).shares.surge > 3 * quick.at(150).shares.surge,
        `${pct(slow.at(150).shares.surge)} at 30 spm vs ${pct(quick.at(150).shares.surge)} at 90`);
  check("same effort, slower rate: slower boat", slow.speedAt(150) < quick.speedAt(150));
  const longDrive = S.simulate({ driveFraction: 0.65 }), shortDrive = S.simulate({ driveFraction: 0.35 });
  check("longer drive: less slip, better efficiency", longDrive.efficiency > base.efficiency + 0.01 &&
        base.efficiency > shortDrive.efficiency + 0.01,
        `${pct(shortDrive.efficiency)} / ${pct(base.efficiency)} / ${pct(longDrive.efficiency)}`);
  const peakAt = (r) => r.peakForce * Math.pow(150 / r.paddlerPower, 2 / 3);
  check("longer drive: lower peak force at the same effort", peakAt(longDrive) < peakAt(base) && peakAt(base) < peakAt(shortDrive));
  // A faster boat turns more per stroke (moment ∝ V², damping ∝ V), so it switches more often.
  check("harder effort: fewer strokes per side", base.at(400).switching.perSide < base.at(150).switching.perSide - 0.5 &&
        Math.abs(base.at(150).switching.perSide - base.strokesPerSide) < 1e-9,
        `${base.at(150).switching.perSide.toFixed(2)} at 150 W, ${base.at(400).switching.perSide.toFixed(2)} at 400 W`);
  check("hull calibrated near a measured C-1 (k 4.26 at 109 kg)", base.speedAt(150) > 2.5 && base.speedAt(150) < 2.85,
        `${base.speedAt(150).toFixed(2)} m/s at 150 W`);
  check("plausible C1 speed at 150 W", base.speedAt(150) > 2.0 && base.speedAt(150) < 3.2, `${base.speedAt(150).toFixed(2)} m/s`);
  check("blade stays buried during the drive", base.samples.every((s) => s.B.wet > 0.99));

  // The stroke itself doesn't care about rate; switching does: a switch takes
  // about the same time at any rate, so a higher rate loses more to it.
  const fast = S.simulate({ rateSpm: 85, switchLossS: 0 }), base0 = S.simulate({ switchLossS: 0 });
  check("efficiency nearly independent of stroke rate (switching aside)", Math.abs(fast.efficiency - base0.efficiency) < 2e-3,
        `${pct(base0.efficiency)} vs ${pct(fast.efficiency)}`);
  check("speed at equal effort independent of rate (switching aside)", Math.abs(fast.speedAt(150) - base0.speedAt(150)) < 0.01);
  const rate50 = S.simulate({ rateSpm: 50 }), rate80 = S.simulate({ rateSpm: 80 });
  check("a switch takes the same time at any rate, so a higher rate loses more to it",
        1 - rate80.switchDuty > 1 - rate50.switchDuty &&
        Math.abs(base.switchDuty - base.strokesPerSide / (base.strokesPerSide + S.DEFAULTS.switchLossS)) < 1e-9,   // at 60 spm a second is a stroke
        `${pct(1 - rate50.switchDuty)} at 50 spm vs ${pct(1 - rate80.switchDuty)} at 80`);

  // Drag only (the stricter blade model) for the technique comparisons;
  // lift is tested on its own below.
  const clean = S.simulate({ leanDeg: 2, handOutM: 0.38, bladeLift: 0 });
  const lean = S.simulate({ leanDeg: 30, handOutM: 0.55, bladeLift: 0 });
  const sweep = S.simulate({ leanDeg: 40, sweepDeg: 70, handOutM: 0.5, catchDeg: 25, exitDeg: 25, bladeLift: 0 });
  check("clean vertical beats a leaning shaft", clean.efficiency > lean.efficiency,
        `${pct(clean.efficiency)} vs ${pct(lean.efficiency)}`);
  check("leaning shaft beats a wide sweep", lean.efficiency > sweep.efficiency, `${pct(lean.efficiency)} vs ${pct(sweep.efficiency)}`);
  check("sweep pushes sideways", sweep.forceShare.sideways > 0.1 && clean.forceShare.sideways < 0.02,
        `${pct(sweep.forceShare.sideways)} vs ${pct(clean.forceShare.sideways)}`);
  check("off-centre strokes turn the boat more", Math.abs(sweep.yawMoment) > 2 * Math.abs(clean.yawMoment));
  check("paddling on the right turns the bow left", clean.yawMoment > 0);
  check("no J-stroke model left", !("steeringPower" in base) && !("steerLD" in S.DEFAULTS));
  // Paddler engine: what each can hold, and how body use shapes the drive.
  const holdArms = S.capacity({}), holdAll = S.capacity({ rotation: 1, legDrive: 1, hinge: 1 }), holdLong = S.capacity({ rotation: 1, legDrive: 1, hinge: 1 }, 16);
  check("capacity: arms only 70%, whole body 100%, fading over a long race", Math.round(holdArms) === 140 && Math.round(holdAll) === 200 && Math.round(holdLong) === 165,
        `${holdArms} ${holdAll} ${holdLong}`);
  const peakPct = (rr) => { const sm = rr.samples, Td = sm[sm.length - 1].t; let m = 0, t = 0; for (const q of sm) if (q.thrust[0] > m) { m = q.thrust[0]; t = q.t; } return t / Td; };
  const legsSim = S.simulate({ legDrive: 1 }), dropSim = S.simulate({ dropM: 0.03 });
  check("leg drive moves the force peak later, a drop moves it earlier", peakPct(legsSim) > peakPct(base) && peakPct(dropSim) < peakPct(base),
        `${peakPct(base)} ${peakPct(legsSim)} ${peakPct(dropSim)}`);
  const rotSim = S.simulate({ rotation: 1 });
  check("rotation reaches further; settings left as set", rotSim.geom.catchDeg === rotSim.params.catchDeg + 15 && rotSim.params.rotation === 1 &&
        Math.abs(S.simulate({}).efficiency - base.efficiency) < 1e-9);
  const hingeSim = S.simulate({ hinge: 1 });
  check("hinge: reach, hand travel, and a body swing that costs power", hingeSim.geom.catchDeg === base.params.catchDeg + 8 &&
        Math.abs(hingeSim.geom.handTravelM - base.params.handTravelM - 0.15) < 1e-9 && hingeSim.at(150).shares.swing > 0.005 && base.at(150).shares.swing === 0,
        pct(hingeSim.at(150).shares.swing));
  const tHold = (q) => { const rr = S.simulate(q), w = S.capacity(q); return S.race(rr, null, 10000, w, w, {}).tA; };
  check("hinge: the extra muscle outweighs the swing", tHold({ hinge: 1 }) < tHold({}));
  const pitch0 = base.at(150), pitch1 = hingeSim.at(150), hingeCyc = hingeSim.cycleAt(150);
  check("pitch: the body rocks the bow, and it costs a little", pitch1.bowMm > 5 * pitch0.bowMm && pitch1.shares.pitch > pitch0.shares.pitch &&
        pitch1.shares.pitch < 0.01 && Math.abs(Math.max(...hingeCyc.bodyX) - 0.06) < 1e-3 && hingeCyc.pitch.length === hingeCyc.t.length,
        `${pitch0.bowMm} ${pitch1.bowMm} ${pct(pitch1.shares.pitch)}`);
  check("clean stroke: about 8 strokes per side", clean.strokesPerSide > 6 && clean.strokesPerSide < 10,
        clean.strokesPerSide.toFixed(1));
  check("sweep forces more frequent switches", sweep.strokesPerSide < clean.strokesPerSide / 3,
        `${sweep.strokesPerSide.toFixed(1)} vs ${clean.strokesPerSide.toFixed(1)}`);
  check("switching costs the sweep more time", sweep.switchDuty < clean.switchDuty - 0.05,
        `${pct(sweep.switchDuty)} vs ${pct(clean.switchDuty)}`);
  check("strokes per side is capped", S.simulate({ leanDeg: 0, handOutM: 0.36 }).strokesPerSide <= S.MAX_STROKES_PER_SIDE);
  check("sideways push causes leeway drag", sweep.leewayPower > 10 * clean.leewayPower && sweep.leewayPower > 0,
        `${sweep.leewayPower.toFixed(2)} W vs ${clean.leewayPower.toFixed(3)} W`);
  check("leeway drag is a small share", sweep.leewayPower / sweep.paddlerPower < 0.02);
  const slowSwitch = S.simulate({ switchLossS: S.DEFAULTS.switchLossS + 0.5 });
  check("slower switches cost speed", slowSwitch.speedAt(150) < base.speedAt(150));
  const tight = S.simulate({ switchHeadingDeg: 1 });
  check("tighter heading tolerance means more switches", tight.strokesPerSide < base.strokesPerSide);

  const short = S.simulate({ catchDeg: 12, exitDeg: 45 });
  const reach = S.simulate({ catchDeg: 50, exitDeg: 10 });
  check("short reach, pull behind beats a long reach (bent toward the bow)", short.efficiency > reach.efficiency,
        `${pct(short.efficiency)} vs ${pct(reach.efficiency)}`);

  // Vertical push: catch lifts, exit pulls down; the 12° bend shifts the
  // balance toward the exit.
  const shortFlat = S.simulate({ catchDeg: 12, exitDeg: 45, bendDeg: 0 });
  check("flat blade, pull behind: pulls the boat down", shortFlat.at(150).meanLift < -3 && shortFlat.at(150).shares.sink > 0,
        `${shortFlat.at(150).meanLift.toFixed(1)} N`);
  check("bent blade, pull behind: up and down about cancel", Math.abs(short.at(150).meanLift) < 4,
        `${short.at(150).meanLift.toFixed(1)} N`);
  check("long reach lifts the boat (a drag saving)", reach.at(150).meanLift > 10 && reach.at(150).shares.sink < 0,
        `${reach.at(150).meanLift.toFixed(1)} N`);
  check("but the lift doesn't make up for the reach's slip", short.efficiency > reach.efficiency);
  check("vertical effects grow with effort", Math.abs(reach.at(300).shares.sink) > Math.abs(reach.at(100).shares.sink));
  check("bounce is a few mm at most", reach.at(150).bounceMm > 0.1 && reach.at(150).bounceMm < 10,
        `±${reach.at(150).bounceMm.toFixed(1)} mm`);
  const heavy = S.simulate({ massKg: 110 });
  check("heavier boat is slower", heavy.speedAt(150) < base.speedAt(150) - 0.05,
        `${heavy.speedAt(150).toFixed(2)} vs ${base.speedAt(150).toFixed(2)} m/s`);

  const r = S.race(clean, sweep, 10000, 150, 150);
  check("race: clean beats sweep over 10 km", r.timeGap > 10 && r.distanceGap > 20,
        `${r.timeGap.toFixed(1)} s, ${r.distanceGap.toFixed(0)} m`);
  check("race: times match speeds", Math.abs(r.tA * r.vA - 10000) < 1e-6 && Math.abs(r.tB * r.vB - 10000) < 1e-6);
  check("race: same stroke, same effort is a dead heat", S.race(clean, clean, 5000, 150, 150).timeGap === 0);
  // Replays carry each piece's speed through the water (the live shallow-water chart):
  // on a river's inside line the boat goes faster along the course than through the water.
  {
    const rv = S.race(clean, null, 5000, 150, 150, { current: 0, depth: 2, river: { width: 60, bendsDegPerKm: 360, lineA: 1, lineB: 1 } });
    const pieces = rv.profileA.filter((g) => g.kind !== "portage"), w0 = clean.at(150).waterSpeed;
    const bend = pieces.find((g) => g.kind === "bend"), open = S.race(clean, null, 5000, 150, 150).profileA;
    check("race: replays know each piece's speed through the water", pieces.every((g) => g.water > 0.8 * w0 && g.water < 1.1 * w0) &&
          bend && bend.v > bend.water * 1.05 && open.every((g) => Math.abs(g.water - open[0].water) < 1e-12),
          bend ? `bend: ${bend.v.toFixed(2)} along the course, ${bend.water.toFixed(2)} m/s through the water` : "no bend");
  }
  check("race: more effort wins", S.race(sweep, clean, 10000, 200, 150).timeGap > 0);
  check("race: swapping boats flips the gap", Math.abs(S.race(sweep, clean, 10000, 150, 150).timeGap + r.timeGap) < 1e-9);
  const still = S.race(clean, clean, 10000, 150, 150);
  const head = S.race(clean, clean, 10000, 150, 150, { headwind: 5 });
  const tail = S.race(clean, clean, 10000, 150, 150, { headwind: -5 });
  check("race: headwind slows, tailwind helps", head.vA < still.vA - 0.1 && tail.vA > still.vA + 0.05,
        `${head.vA.toFixed(2)} / ${still.vA.toFixed(2)} / ${tail.vA.toFixed(2)} m/s`);
  check("race: calm is the same as no conditions", Math.abs(still.vA - clean.speedAt(150)) < 1e-6);
  const withC = S.race(clean, clean, 10000, 150, 150, { current: 1 });
  check("race: current adds to ground speed", Math.abs(withC.vA - withC.waterA - 1) < 1e-12 &&
        withC.vA > still.vA + 0.95 && withC.waterA < still.waterA, `${withC.vA.toFixed(2)} m/s`);
  const flood = S.race(clean, sweep, 10000, 150, 150, { current: -3 });
  check("race: too much current means nobody finishes", !isFinite(flood.tA) && !isFinite(flood.tB) &&
        flood.timeGap === 0 && flood.distanceGap === 0);
  // There and back: what you lose against the current you don't win back
  // with it (The Science of Paddling, Part 7): T = (2L/V) / (1 − U²/V²).
  const ob0 = S.race(clean, clean, 2000, 150, 150, { course: "outback" });
  const ob = S.race(clean, clean, 2000, 150, 150, { course: "outback", current: 0.5 });
  const V0 = clean.speedAt(150), formula = 1 / (1 - 0.25 / (V0 * V0));
  check("there and back in still water = point to point", Math.abs(ob0.tA - S.race(clean, clean, 2000, 150, 150).tA) < 1e-6);
  check("there and back with current matches (2L/V)/(1 − U²/V²)", Math.abs(ob.tA / ob0.tA - formula) < 0.005,
        `${(ob.tA / ob0.tA).toFixed(4)} vs ${formula.toFixed(4)}`);
  const obw = S.race(clean, clean, 2000, 150, 150, { course: "outback", headwind: 4 });
  const obt = S.race(clean, clean, 2000, 150, 150, { course: "outback", headwind: -4 });
  check("there and back: wind costs either way round", obw.tA > ob0.tA && Math.abs(obw.tA - obt.tA) < 1e-6);
  check("race positions: finish at the distance", Math.abs(ob.posA(ob.tA) - 2000) < 1e-6 && ob.posA(ob.tA / 4) < 1000);
  const strong = S.race(clean, clean, 10000, 300, 100, { current: -2.6 });
  check("race: only the stronger paddler makes headway", isFinite(strong.tA) && !isFinite(strong.tB) && strong.timeGap === Infinity);
  // The hull through one stroke.
  const def = S.simulate({}), cyc = def.cycleAt(150), e150 = def.at(150);
  const span = (a) => Math.max(...a) - Math.min(...a), avg = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  check("hull: the default paddle is a 51 in ZRE (17 × 8 in blade)", Math.abs(S.DEFAULTS.paddleLenM - 1.295) < 1e-9 && Math.abs(S.PADDLE.bladeLen - 0.432) < 1e-9);
  check("hull: speed swing over a stroke = the engine's surge", Math.abs(span(cyc.speed) / e150.waterSpeed - e150.surgeSwing) < 0.005,
        `${(span(cyc.speed) / e150.waterSpeed).toFixed(4)} vs ${e150.surgeSwing.toFixed(4)}`);
  check("hull: bob = the engine's bounce", Math.abs(span(cyc.heave) / 2 * 1000 - e150.bounceMm) < 0.15 * e150.bounceMm, `${(span(cyc.heave) * 500).toFixed(2)} vs ${e150.bounceMm.toFixed(2)} mm`);
  // Without switching: with it, each stroke's push also covers the time lost switching.
  const cyc0 = S.simulate({ switchLossS: 0 }).cycleAt(150);
  check("hull: push balances drag over a stroke; the gap averages zero", Math.abs(avg(cyc0.push) / avg(cyc0.drag) - 1) < 0.1 && Math.abs(avg(cyc.ahead)) < 1e-6,
        `${avg(cyc0.push).toFixed(1)} vs ${avg(cyc0.drag).toFixed(1)} N`);
  check("hull: the blade lifts the paddle side; hips shift toward it to balance", Math.min(...cyc.roll) < -10 && cyc.rollMean < 0 && cyc.hipShift > 0 &&
        Math.abs(avg(cyc.heel)) < 1e-6 && cyc.GM > 0.1);
  check("hull: a wider blade slips less", S.simulate({ bladeWidthM: 0.24 }).at(150).shares.slip < e150.shares.slip);
  // Body motion and the boat score.
  const bodyStill = S.simulate({}), sw10 = S.simulate({ swingM: 0.1 }), sw5 = S.simulate({ swingM: 0.05 });
  check("body: defaults change nothing", Math.abs(bodyStill.efficiency - def.efficiency) < 1e-12 && bodyStill.at(150).shares.swing === 0);
  check("body: a bigger swing costs more (±5 cm ~1%, ±10 cm ~3–5%)", sw5.at(150).shares.swing > 0.005 && sw5.at(150).shares.swing < 0.02 &&
        sw10.at(150).shares.swing > 0.03 && sw10.at(150).shares.swing < 0.05 && sw10.at(150).speed < bodyStill.at(150).speed,
        `${(sw5.at(150).shares.swing * 100).toFixed(2)}%, ${(sw10.at(150).shares.swing * 100).toFixed(2)}%`);
  const dr = S.simulate({ dropM: 0.03, switchLossS: 0 }).at(150);
  check("body: a drop at the catch makes the boat bob more", dr.bounceMm > 2 * dr.bounceStillMm && dr.shares.drop > 0);
  check("body: heeling toward the paddle means fewer switches", S.simulate({ heelDeg: 6 }).strokesPerSide > bodyStill.strokesPerSide + 0.5);
  const hr = S.simulate({ hipRock: 0.8 }).cycleAt(150), heelAmp = (a) => Math.max(...a.map(Math.abs));
  check("body: hip rock cancels the roll", Math.abs(heelAmp(hr.heel) / hr.heelStill - 0.2) < 0.08 && hr.hipMax > 0.02, `${(heelAmp(hr.heel) / hr.heelStill).toFixed(2)}`);
  const sc0 = bodyStill.at(150).boatScore, scW = S.simulate({ leanDeg: 40, sweepDeg: 70, handOutM: 0.5, catchDeg: 25, exitDeg: 25 }).at(150).boatScore;
  const scClean = S.simulate({ switchLossS: 0 }).at(150).boatScore;
  check("boat score: a clean stroke ~100 but for its switching; switching costs the wide sweep more",
        scClean.total > 98 && scClean.total < 102 && Math.abs(sc0.total + sc0.parts.switching - scClean.total) < 1 &&
        scW.total < sc0.total - 8 && scW.parts.switching > sc0.parts.switching && sw10.at(150).boatScore.total < sc0.total - 4,
        `${scClean.total.toFixed(1)} without switching, ${sc0.total.toFixed(1)} with / wide sweep ${scW.total.toFixed(1)}`);
  // A real course, section by section.
  const rivEnv = { current: 0.7, depth: 3, river: { width: 40, bendsDegPerKm: 180, lineA: "best", lineB: 0 } };
  const one = S.race(clean, sweep, 0, 150, 150, { sections: [{ len: 10000, kind: "river", current: 0.7, depth: 3, width: 40, bendsDegPerKm: 180, section: 0 }], lines: { A: "best", B: 0 } });
  const same = S.race(clean, sweep, 10000, 150, 150, rivEnv);
  check("sections: one river section = the river race", Math.abs(one.tA - same.tA) < 1e-3 && Math.abs(one.tB - same.tB) < 1e-6, `${one.tA} vs ${same.tA}`);
  const pond = S.race(clean, clean, 0, 150, 150, { sections: [{ len: 5000, kind: "pond", depth: 2, section: 0 }] });
  check("sections: a pond = open water", Math.abs(pond.tA - S.race(clean, clean, 5000, 150, 150, { depth: 2 }).tA) < 1e-6);
  const carry = S.race(clean, clean, 0, 150, 150, { sections: [{ len: 200, kind: "portage", section: 0 }] });
  check("sections: a 200 m portage is carried (2 m/s + 30 s)", Math.abs(carry.tA - 130) < 1e-9 && carry.profileA[0].rate === 0);
  const into = (h) => S.race(clean, clean, 0, 150, 150, { windSpeed: 5, windFrom: 0, sections: [{ len: 5000, kind: "pond", heading: h, section: 0 }] }).tA;
  // Weather hour by hour: each boat meets the hour it reaches each leg in.
  {
    const ponds = Array.from({ length: 20 }, (_, i) => ({ len: 2000, kind: "pond", heading: 0, section: i }));
    const go = (env) => S.race(clean, clean, 0, 150, 150, Object.assign({ sections: ponds }, env));
    const calm = go({}), still = go({ weather: [{ windSpeed: 0, windFrom: 0, power: 1 }] });
    check("race: weather: one calm, cool hour is the same as none", Math.abs(still.tA - calm.tA) < 1e-6, `${still.tA} vs ${calm.tA}`);
    const hot = go({ weather: [{ windSpeed: 0, windFrom: 0, power: 0.95 }] });
    check("race: weather: the heat takes power, slower", hot.tA > calm.tA && hot.legsA.every((l) => Math.abs(l.watts - 142.5) < 1e-9));
    // A headwind that comes up in the third hour slows only the legs from then on.
    const later = go({ weather: [0, 0, 6].map((w) => ({ windSpeed: w, windFrom: 0, power: 1 })) });
    const firstLate = later.legsA.findIndex((l) => l.hour === 2);
    check("race: weather: a wind that comes up later slows only what's after it", firstLate > 0 &&
          later.legsA.slice(0, firstLate).every((l, i) => Math.abs(l.t - calm.legsA[i].t) < 1e-9) &&
          later.legsA.slice(firstLate).every((l, i) => l.t > calm.legsA[firstLate + i].t) && later.tA > calm.tA,
          `first late leg ${firstLate}, ${later.tA.toFixed(0)} vs ${calm.tA.toFixed(0)} s`);
    const each = go({ weather: [{ windSpeed: 0, windFrom: 0, power: 0.95, powerB: 0.9 }] });
    check("race: weather: each boat its own heat (powerB)", Math.abs(each.tA - hot.tA) < 1e-6 && each.tB > each.tA &&
          each.legsB.every((l) => Math.abs(l.watts - 135) < 1e-9));
    check("race: weather: the replay follows it", later.profileA.some((g) => g.watts === 150) && Math.abs(later.posA(later.tA) - 40000) < 1);
  }
  check("sections: the wind turns with the course", Math.abs(into(0) - S.race(clean, clean, 5000, 150, 150, { windSpeed: 5, windAngle: 0 }).tA) < 1e-6 &&
        into(180) < into(90) && into(90) < into(0) && Math.abs(into(90) - into(270)) < 1e-9);
  const mixed = S.race(clean, clean, 0, 150, 150, { sections: [{ len: 3000, kind: "river", current: 0.8, depth: 1.5, width: 30, bendsDegPerKm: 300, section: 0 },
    { len: 200, kind: "portage", section: 1 }, { len: 2000, kind: "pond", depth: 4, section: 2 }] });
  check("sections: lengths add up; positions reach the finish", mixed.metres === 5200 && Math.abs(mixed.posA(mixed.tA) - 5200) < 1e-6 &&
        JSON.stringify([...new Set(mixed.profileA.map((g) => g.section))]) === "[0,1,2]");

  const out = S.simulate({ directionDeg: 25 });
  check("pulling outward loses forward force", out.forceShare.forward < base.forceShare.forward - 0.05);
  const noTravel = S.simulate({ handTravelM: 0 });
  check("hand travel speeds the blade (more power at the same rate)", base.paddlerPower > noTravel.paddlerPower);

  // Shallow water (The Science of Paddling, Part 4): deep below L/2 changes nothing; under 2 ft drag climbs.
  const cA = S.simulate({});
  const at = (env) => S.race(cA, cA, 10000, 150, 150, env);
  const deep = at({}), d28 = at({ depth: 2.8 }), d14 = at({ depth: 1.4 }), d06 = at({ depth: 0.6 }), d03 = at({ depth: 0.3 });
  check("deep water below L/2 barely matters", Math.abs(d28.vA - deep.vA) / deep.vA < 0.002);
  check("shallower is slower", deep.vA > d14.vA && d14.vA > d06.vA && d06.vA > d03.vA,
        [deep, d14, d06, d03].map((r) => r.vA.toFixed(2)).join(" > "));
  check("2 ft of water costs 3–25% of speed", d06.vA < deep.vA * 0.97 && d06.vA > deep.vA * 0.75,
        `${pct(1 - d06.vA / deep.vA)} slower`);
  check("shallow factor capped", S.shallowFactor(2.7, 0.1) === 5 && S.shallowFactor(2.7, Infinity) === 1);

  // Crosswind: crab, lopsided switching, slower; a gale out-turns the stroke.
  const beam = at({ windSpeed: 20 / 3.6, windAngle: 90 }), calmL = deep.legsA[0], beamL = beam.legsA[0];
  check("crosswind slows the boat a little", beam.vA < deep.vA && beam.vA > deep.vA * 0.9,
        `${pct(1 - beam.vA / deep.vA)} slower`);
  check("crosswind: the hull crabs a few degrees", beamL.leewayDeg > 1 && beamL.leewayDeg < 10, `${beamL.leewayDeg.toFixed(1)}°`);
  check("crosswind: more strokes windward than lee", beamL.windward > beamL.lee + 1 && Math.abs(calmL.windward - calmL.lee) < 1e-9,
        `${beamL.windward.toFixed(1)} / ${beamL.lee.toFixed(1)}`);
  check("calm switching matches the stroke's own figure", Math.abs(calmL.windward - cA.strokesPerSide) < 1e-6);
  const gale = at({ windSpeed: 45 / 3.6, windAngle: 90 }).legsA[0];
  check("a gale out-turns the stroke: steer instead", gale.steer && gale.duty === 1);
  const headOnly = at({ windSpeed: 5, windAngle: 0 }), signed = at({ headwind: 5 });
  check("headwind as speed + angle = signed headwind", Math.abs(headOnly.vA - signed.vA) < 1e-9 && headOnly.legsA[0].leewayDeg === 0);
  check("quartering tailwind beats quartering headwind",
        at({ windSpeed: 5, windAngle: 135 }).vA > at({ windSpeed: 5, windAngle: 45 }).vA);
  const obW = at({ course: "outback", windSpeed: 5, windAngle: 60 });
  check("there and back: wind angle mirrors on the way home", obW.legsA[0].leewayDeg > 0.5 && obW.legsA[1].leewayDeg > 0.5 &&
        obW.legsA[1].water > obW.legsA[0].water);

  // River (The Science of Paddling, Parts 18 and 20): current and depth across the river; corners.
  const riv = (line, current, depth = 3, bends = 180, extra = {}) => at(Object.assign({ current: current / 3.6, depth,
    river: { width: 60, bendsDegPerKm: bends, lineA: line, lineB: line } }, extra));
  check("river, middle line, still water = open water", Math.abs(riv(0, 0, 5).tA - at({ depth: 5 }).tA) < 1e-6);
  const upMid = riv(0, -4), upEdge = riv(1, -4), upBest = riv("best", -4);
  check("upstream: the edges beat the middle", upEdge.tA < upMid.tA * 0.9, `${(upEdge.tA / 60).toFixed(1)} vs ${(upMid.tA / 60).toFixed(1)} min`);
  const dnMid = riv(0, 4), dnEdge = riv(1, 4), dnBest = riv("best", 4);
  check("downstream: the middle beats the edges", dnMid.tA < dnEdge.tA);
  check("best line is at least as fast as any fixed line", upBest.tA <= Math.min(upMid.tA, upEdge.tA, riv(0.5, -4).tA) + 1e-6 &&
        dnBest.tA <= Math.min(dnMid.tA, dnEdge.tA) + 1e-6);
  check("best line: edges upstream, middle downstream", upBest.legsA[0].river.line > 0.7 && dnBest.legsA[0].river.line < 0.3,
        `${upBest.legsA[0].river.line.toFixed(2)} / ${dnBest.legsA[0].river.line.toFixed(2)}`);
  const obR = riv("best", 4, 3, 180, { course: "outback" });
  check("there and back: middle down, edges back", obR.legsA[0].river.line < 0.3 && obR.legsA[1].river.line > 0.7);
  const cut = riv("best", 0, 5, 360), noCut = riv(0, 0, 5, 360);
  check("still water: cutting corners pays", cut.tA < noCut.tA && cut.legsA[0].river.line > 0.3 &&
        cut.legsA[0].river.path < 10000, `line ${cut.legsA[0].river.line.toFixed(2)}, path ${cut.legsA[0].river.path.toFixed(0)} m`);
  const cutShallow = riv("best", 0, 1, 360);
  check("shallower river: cut corners less", cutShallow.legsA[0].river.line < cut.legsA[0].river.line,
        `${cutShallow.legsA[0].river.line.toFixed(2)} vs ${cut.legsA[0].river.line.toFixed(2)}`);
  const segs = S.riverSegments({ width: 60, bendsDegPerKm: 180 }, 1, -1, 3);
  check("river segments add up to a km", Math.abs(segs.reduce((a, g) => a + g.len, 0) - 1000) < 1e-6 &&
        segs.some((g) => g.kind === "crossing") && segs.find((g) => g.kind === "bend").path < segs.find((g) => g.kind === "bend").len);

  // Strokes per side: switching by heading, or a fixed count that sets the wander.
  const autoS = S.simulate({});
  check("auto: wander equals the heading tolerance", Math.abs(autoS.headingWanderDeg - 3) < 1e-6, autoS.headingWanderDeg.toFixed(2));
  const fixedN = (n) => S.simulate({ switchStrokes: n });
  check("fixed count is used", fixedN(9).strokesPerSide === 9);
  // The boat keeps turning through the switch, so wander goes as strokes per side plus the switch's strokes.
  const swStrokes = S.DEFAULTS.switchLossS * S.DEFAULTS.rateSpm / 60;
  check("fixed count: more strokes, more wander", Math.abs(fixedN(12).headingWanderDeg / fixedN(6).headingWanderDeg - (12 + swStrokes) / (6 + swStrokes)) < 0.02,
        `${fixedN(6).headingWanderDeg.toFixed(1)}° / ${fixedN(12).headingWanderDeg.toFixed(1)}°`);
  check("zigzag costs progress: V·(1 − ψ²/6)", Math.abs(autoS.courseFactor - (1 - Math.pow(3 * Math.PI / 180, 2) / 6)) < 1e-12);
  const sp = (n) => fixedN(n).speedAt(150);
  check("strokes per side has a sweet spot", sp(8) > sp(1) && sp(8) > sp(20), `1: ${sp(1).toFixed(3)} 8: ${sp(8).toFixed(3)} 20: ${sp(20).toFixed(3)}`);
  const swp = (n) => S.simulate({ leanDeg: 40, sweepDeg: 70, handOutM: 0.5, catchDeg: 25, exitDeg: 25, switchStrokes: n }).speedAt(150);
  check("a turning stroke wants fewer strokes per side", swp(4) > swp(12));
  check("race in calm matches speedAt with a fixed count", Math.abs(S.race(fixedN(12), fixedN(12), 10000, 150, 150).vA - sp(12)) < 1e-6);

  // Crosswind with a fixed count: average N per side, more windward, course held.
  const f10 = fixedN(10), beamF = S.race(f10, f10, 10000, 150, 150, { windSpeed: 20 / 3.6, windAngle: 90 }).legsA[0];
  check("fixed count in a crosswind: more windward, average kept", beamF.windward > beamF.lee + 2 &&
        Math.abs((beamF.windward + beamF.lee) / 2 - 10) < 1e-9, `${beamF.windward.toFixed(1)} / ${beamF.lee.toFixed(1)}`);
  check("fixed count in a crosswind: wander no wider than calm", beamF.wanderDeg <= f10.headingWanderDeg + 1e-9);
  const calmF = S.race(f10, f10, 10000, 150, 150).legsA[0];
  check("fixed count, calm: even split, the stroke's own wander", calmF.windward === 10 && calmF.lee === 10 &&
        Math.abs(calmF.wanderDeg - f10.headingWanderDeg) < 1e-6);
  const galeF = S.race(f10, f10, 10000, 150, 150, { windSpeed: 45 / 3.6, windAngle: 90 }).legsA[0];
  check("fixed count: a gale still means steering", galeF.steer);

  // Blade lift: a flat plate's normal force follows the sine law.
  const SW = { leanDeg: 40, sweepDeg: 70, handOutM: 0.5, catchDeg: 25, exitDeg: 25 }, CL = { leanDeg: 2, handOutM: 0.38 };
  const sim = (p, extra) => S.simulate(Object.assign({}, p, extra));
  const liftGain = (p) => sim(p, { bladeLift: 1 }).efficiency - sim(p, { bladeLift: 0 }).efficiency;
  check("lift never makes a stroke worse", liftGain(CL) >= 0 && liftGain(SW) >= 0 && liftGain({}) >= 0);
  check("lift helps a sideways-slicing sweep much more than a clean stroke", liftGain(SW) > 5 * liftGain(CL),
        `+${pct(liftGain(SW))} vs +${pct(liftGain(CL))}`);
  check("a sweep's blade force is mostly lift; a clean stroke's mostly not", sim(SW, {}).liftShare > 0.4 && sim(CL, {}).liftShare < 0.25,
        `${pct(sim(SW, {}).liftShare)} vs ${pct(sim(CL, {}).liftShare)}`);
  const lb = sim(SW, { pitchDeg: -15 }), bal = lb.hullPower + lb.leewayPower + lb.slipPower - lb.paddlerPower;
  check("energy still balances with lift and pitch", Math.abs(bal) < 1e-3 * lb.paddlerPower);
  const pe = (d) => sim(CL, { pitchDeg: d }).efficiency;
  check("clean stroke: square blade is best", pe(0) > pe(15) && pe(0) > pe(-15));
  const ps = (d) => sim(SW, { pitchDeg: d }).efficiency;
  check("sweep: turning the face in toward the hull helps", ps(-15) > ps(0) && ps(0) > ps(15));

  // Strength by position (off by default): weak far from vertical.
  const on = (x) => Object.assign({ strengthOn: 1 }, x);
  const LR = { catchDeg: 55, exitDeg: 45 };
  check("strength by position: off by default, changes nothing", !S.DEFAULTS.strengthOn && S.strengthFactor({}) === 1 && S.strengthAt(S.DEFAULTS, 50) === 1);
  check("strength by position: the default stroke keeps what it can hold; a long reach loses it",
        Math.abs(S.strengthFactor(on({})) - 1) < 1e-9 && S.strengthFactor(on(LR)) < 0.85,
        `×${S.strengthFactor(on(LR)).toFixed(2)}`);
  check("strength by position: longer arms keep strength further out",
        S.strengthFactor(on(Object.assign({ upperArmM: 0.34, forearmM: 0.34 }, LR))) > S.strengthFactor(on(Object.assign({ upperArmM: 0.26, forearmM: 0.26 }, LR))));
  const sOff = S.simulate(LR), sOn = S.simulate(on(LR)), pk = (r) => r.peakForce * Math.pow(150 / r.paddlerPower, 2 / 3);
  const sBal = sOn.hullPower + sOn.leewayPower + sOn.slipPower - sOn.paddlerPower;
  check("strength by position: force bunches where you're strong (higher peak), energy still balances",
        pk(sOn) > pk(sOff) * 1.1 && Math.abs(sBal) < 1e-3 * sOn.paddlerPower, `${pk(sOff).toFixed(0)} → ${pk(sOn).toFixed(0)} N`);
  check("strength by position: what you can hold scales with it", Math.abs(S.capacity(on(LR)) / S.capacity(LR) - S.strengthFactor(on(LR))) < 1e-9);

  // The body is fitted to the paddle: within reach for every preset, and it says when it can't be.
  const PRE = { def: {}, long: LR, behind: { catchDeg: 12, exitDeg: 45 }, sweep: { leanDeg: 40, sweepDeg: 70, handOutM: 0.5, catchDeg: 25, exitDeg: 25 },
                rot: { rotation: 1 }, clean: { leanDeg: 2, handOutM: 0.38 } };
  const worst = Object.entries(PRE).map(([k, p]) => { const b = S.simulate(p).body; return [k, Math.max(...b.frames.map((f) => Math.max(f.top, f.low))), b.short]; });
  check("body: both hands within reach all through the drive, for every preset", worst.every(([, m, sh]) => m <= 0.98 && sh === 0),
        worst.map(([k, m]) => `${k} ${(m * 100).toFixed(0)}%`).join(", "));
  const far = S.simulate({ catchDeg: 60, handOutM: 0.85, leanDeg: 40 }).body;
  check("body: a stroke out of reach says by how much", far.short > 0.05 && far.short < 0.3, `${(far.short * 100).toFixed(0)} cm`);
  // The seat: hips where your own stroke's blade leaves the water, and every
  // technique paddled from there.
  const exitOf = (r) => r.samples[r.samples.length - 1].B.cop[0];
  const own = S.simulate({}), behind = S.simulate(PRE.behind);
  check("body: you sit with the blade leaving the water at your hip", Math.abs(own.body.seatX - exitOf(own)) < 0.06,
        `hips ${(100 * (own.body.seatX - exitOf(own))).toFixed(0)} cm ahead of the exit`);
  check("body: one seat for every technique; pulling behind exits behind the hip",
        behind.body.seatX === own.body.seatX && own.body.seatX - exitOf(behind) > 0.2, `${(100 * (own.body.seatX - exitOf(behind))).toFixed(0)} cm behind`);
  const topAhead = (() => { const g = own.geom, P = S.pose(g, 0.5), b = S.fitBody(g, own.body.seatX, P.at(0), P.at(S.lowerHand(g)), S.neutralAt(g, 0.5));
    return P.at(0)[0] - b.off[0]; })();
  check("body: the top hand is ahead of its shoulder mid-drive", topAhead > 0.1 && topAhead < 0.35, `${(topAhead * 100).toFixed(0)} cm`);
  // Seat height: higher is less stable, and changes nothing else.
  const seatAt = (z) => S.simulate({ seatZM: z }), lowSeat = seatAt(0.05), highSeat = seatAt(0.25);
  const gm = (r) => r.cycleAt(150).GM, share = (S.DEFAULTS.massKg - S.BOAT_KG) / S.DEFAULTS.massKg;
  check("seat height: each cm higher costs the paddler's share of a cm of metacentric height",
        Math.abs(gm(lowSeat) - gm(highSeat) - 0.2 * share) < 1e-9 && Math.abs(gm(own) - gm(seatAt(S.DEFAULTS.seatZM))) < 1e-12,
        `${(gm(lowSeat) * 100).toFixed(1)} → ${(gm(highSeat) * 100).toFixed(1)} cm`);
  // The rock each stroke depends on the stroke's timing against the roll
  // period (about 1 s, one stroke at 60 spm), so it isn't simply larger.
  check("seat height: a higher seat rolls more slowly, at the same speed",
        highSeat.cycleAt(150).rollPeriod > lowSeat.cycleAt(150).rollPeriod + 0.1 && highSeat.speedAt(150) === lowSeat.speedAt(150),
        `${lowSeat.cycleAt(150).rollPeriod.toFixed(2)} → ${highSeat.cycleAt(150).rollPeriod.toFixed(2)} s`);
  const hipZ = (r) => { const g = r.geom, P = S.pose(g, 0); return S.fitBody(g, r.body.seatX, P.at(0), P.at(S.lowerHand(g)), S.neutralAt(g, 0)).hip[2]; };
  check("seat height: the hips sit at it", Math.abs(hipZ(highSeat) - 0.25) < 1e-12 && Math.abs(hipZ(own) - S.DEFAULTS.seatZM) < 1e-12);
  const seats = [36, 38, 40].map((c) => S.simulate({ catchDeg: c }).body.seatX);
  check("body: the seat holds steady as the stroke changes", Math.max(...seats) - Math.min(...seats) < 0.05 && seats.every((x) => x > -0.6 && x < 0),
        seats.map((x) => x.toFixed(2)).join(" "));
  // Strength from the arms.
  const arms = (x) => Object.assign({ strengthOn: 2 }, x), fA = (x) => S.strengthFactor(arms(x));
  check("strength from the arms: arms at full stretch tire you (wide sweep, out of reach), comfortable ones don't",
        Math.abs(fA({}) - 1) < 1e-9 && fA(PRE.sweep) < 0.9 && fA({ catchDeg: 60, handOutM: 0.85, leanDeg: 40 }) < 0.6 && fA(PRE.clean) > 0.98,
        `sweep ×${fA(PRE.sweep).toFixed(2)}, clean ×${fA(PRE.clean).toFixed(2)}`);
  // From one seat the body leans to reach the catch, and leaning far forward
  // is weaker: a long reach costs strength again, more with short arms.
  const withArms = (a) => Object.assign({ upperArmM: a, forearmM: a }, LR);
  check("strength from the arms: leaning to a long reach costs strength, more with shorter arms",
        fA(LR) < 0.98 && fA(withArms(0.26)) < fA(LR) - 0.02 && fA(withArms(0.34)) > fA(LR),
        [0.26, 0.30, 0.34].map((a) => `×${fA(withArms(a)).toFixed(3)}`).join(" / "));
  const leans = S.simulate(arms(LR)).body.frames.map((f) => f.lean), ownLean = S.simulate(arms({})).body.frames[0].lean;
  check("body: a long reach leans further forward at the catch", leans[0] > ownLean + 0.1 && leans[leans.length - 1] < leans[0],
        `${(leans[0] * 180 / Math.PI).toFixed(0)}° vs ${(ownLean * 180 / Math.PI).toFixed(0)}°`);
  // Longer arms also reach further: a stroke out of reach for short arms
  // comes within reach.
  const farFor = (a) => S.simulate({ catchDeg: 60, handOutM: 0.85, leanDeg: 40, upperArmM: a, forearmM: a }).body.short;
  check("body: longer arms reach further", farFor(0.34) < farFor(0.30) - 0.05 && farFor(0.30) < farFor(0.26) - 0.05,
        [0.26, 0.30, 0.34].map((a) => `${(farFor(a) * 100).toFixed(0)} cm`).join(" / "));
  // Gearing: the arms hold the most power at one speed of the lower hand
  // against its shoulder.
  const gear = (x) => Object.assign({ gearOn: 1 }, x), gF = (x) => S.gearFactor(gear(x)), armV = (x) => S.simulate(x).body.armSpeed;
  check("gearing: off by default, changes nothing", !S.DEFAULTS.gearOn && S.gearFactor({ rateSpm: 85 }) === 1 &&
        S.capacity({ rateSpm: 85 }) === S.capacity({}));
  check("gearing: the default stroke moves the arms at about their best speed", Math.abs(gF({}) - 1) < 0.005,
        `${armV({}).toFixed(2)} m/s, ×${gF({}).toFixed(3)}`);
  check("gearing: arm speed goes as the stroke rate", Math.abs(armV({ rateSpm: 75 }) / armV({}) - 75 / 60) < 1e-9);
  check("gearing: too fast or too slow a rate holds less", gF({ rateSpm: 80 }) < gF({ rateSpm: 70 }) && gF({ rateSpm: 70 }) < 0.99 && gF({ rateSpm: 45 }) < 0.99,
        `45 spm ×${gF({ rateSpm: 45 }).toFixed(3)}, 70 ×${gF({ rateSpm: 70 }).toFixed(3)}, 80 ×${gF({ rateSpm: 80 }).toFixed(3)}`);
  const lrBest = S.bestRate(gear(LR));
  check("gearing: a longer stroke suits a lower rate, and its best rate puts the arms at their best",
        lrBest < 50 && gF(Object.assign({}, LR, { rateSpm: lrBest })) > 1 - 1e-9, `${lrBest.toFixed(0)} spm`);
  check("gearing: loses gearDrop at 1.5 times the best arm speed", Math.abs(gF({ armBestMs: armV({}) / 1.5 }) - (1 - S.DEFAULTS.gearDrop)) < 1e-9);
  check("gearing: the trunk carries rotation's and a hinge's travel, wider hands move the arms faster",
        Math.abs(armV({ rotation: 1 }) / armV({}) - 1) < 0.05 && Math.abs(armV({ hinge: 1 }) / armV({}) - 1) < 0.05 &&
        armV({ lowerHandM: 0.7 }) > armV({}) * 1.05, `rotation ${armV({ rotation: 1 }).toFixed(2)}, hinge ${armV({ hinge: 1 }).toFixed(2)}, wide ${armV({ lowerHandM: 0.7 }).toFixed(2)} m/s`);
  const capOf = (x) => S.capacity(x), rot80 = { rotation: 1, rateSpm: 80 };
  check("gearing: sets only the arms' share of what you hold",
        Math.abs(capOf(gear(rot80)) - capOf(rot80) + 0.7 * S.DEFAULTS.fitnessW * (1 - gF(rot80))) < 1e-9);
  const armCurve = S.simulate(arms(LR)).strengthCurve;
  let bumpy = 0; for (let i = 2; i < armCurve.length; i++) bumpy = Math.max(bumpy, Math.abs(armCurve[i][1] - 2 * armCurve[i - 1][1] + armCurve[i - 2][1]));
  check("strength from the arms: a smooth curve through the drive (no steps)", bumpy < 0.1, bumpy.toFixed(3));
}

console.log("\nPaddle fit");
{
  const run = (gen) => { let st; do st = gen.next(); while (!st.done); return st.value; };
  const D = S.DEFAULTS, chop = F.CONDITIONS.find((c) => c.id === "chop");
  // A new paddle length keeps the blade path, sweeping less angle when longer.
  const same = F.withBladePath(D, D), long = F.withBladePath(Object.assign({}, D, { paddleLenM: 1.372 }), D);
  check("fit: the same paddle keeps its angles", Math.abs(same.catchDeg - D.catchDeg) < 0.05 && Math.abs(same.exitDeg - D.exitDeg) < 0.05);
  check("fit: a longer paddle on the same blade path sweeps less angle",
        long.catchDeg < D.catchDeg - 1 && long.exitDeg < D.exitDeg - 1 &&
        Math.abs(F.bladeX(long, 0) - F.bladeX(D, 0)) < 1e-3 && Math.abs(F.bladeX(long, 1) - F.bladeX(D, 1)) < 1e-3,
        `${long.catchDeg.toFixed(1)}° / ${long.exitDeg.toFixed(1)}°`);
  check("fit: so the arms move slower: a higher gear", S.simulate(long).body.armSpeed < S.simulate(D).body.armSpeed - 0.01);
  // The search: the seat as high as the water allows, inside every limit.
  const plain = run(F.search(D, chop)), z = plain.best.setup.seatZM;
  const gmAt = (seatZM) => F.measure(Object.assign({}, D, { seatZM }), chop).GM;
  check("fit: the seat as high as the water allows", plain.best.ok && plain.best.GM >= chop.gmMin && gmAt(z + F.RANGES.seatZM[2]) < chop.gmMin,
        `${(z * 100).toFixed(1)} cm, metacentric height ${(plain.best.GM * 100).toFixed(0)} cm`);
  check("fit: with gearing and strength off, setups inside the limits are about as fast",
        plain.best.speed >= plain.current.speed && plain.best.speed < plain.current.speed * 1.01,
        `${((plain.best.speed / plain.current.speed - 1) * 100).toFixed(2)}%`);
  // Gearing ranks them: too fast a rate wants a higher gear, too slow a lower one.
  const fast = run(F.search(Object.assign({}, D, { gearOn: 1, rateSpm: 80 }), chop));
  const slow = run(F.search(Object.assign({}, D, { gearOn: 1, rateSpm: 45 }), chop));
  check("fit: at a high rate, a higher gear: slower arms, more held",
        fast.best.armSpeed < fast.current.armSpeed - 0.05 && fast.best.watts > fast.current.watts && fast.best.speed > fast.current.speed * 1.01,
        `arms ${fast.current.armSpeed.toFixed(2)} → ${fast.best.armSpeed.toFixed(2)} m/s, ${((fast.best.speed / fast.current.speed - 1) * 100).toFixed(1)}%`);
  check("fit: at a low rate, a lower gear: faster arms", slow.best.armSpeed > slow.current.armSpeed + 0.05 && slow.best.speed > slow.current.speed,
        `arms ${slow.current.armSpeed.toFixed(2)} → ${slow.best.armSpeed.toFixed(2)} m/s`);
  const inRange = (m) => Object.entries(F.RANGES).every(([k, [a, b]]) => m.setup[k] >= a - 1e-9 && m.setup[k] <= b + 1e-9);
  check("fit: suggestions stay in the ranges searched", [plain, fast, slow].every((r) => inRange(r.best)));
  // The map: every seat × length, the tippy squares exactly those under the stability needed.
  const grid = run(F.map(fast.best.setup, Object.assign({}, D, { gearOn: 1, rateSpm: 80 }), chop));
  const atBest = grid.cells.find((c) => Math.abs(c.seatZM - 0.15) < 1e-9 && Math.abs(c.paddleLenM - fast.best.setup.paddleLenM) < 1e-6);
  check("fit: map of seat height × paddle length", grid.cells.length === grid.seats.length * grid.lengths.length &&
        grid.cells.every((c) => (c.past.stable > 0) === (c.GM < chop.gmMin)) && (!atBest || atBest.ok),
        `${grid.cells.filter((c) => c.ok).length} of ${grid.cells.length} inside the limits`);
}

(async () => {
  console.log("\nCanoe Race Simulator page");
  const browser = await chromium.launch();
  for (const [scheme, width] of [["light", 390], ["dark", 1200]]) {
    const ctx = await browser.newContext({ viewport: { width, height: 900 }, colorScheme: scheme, deviceScaleFactor: 2 });
    // Regression: an animation frame timestamp earlier than the page's own
    // clock read gave a negative time step and crashed the views. Make every
    // frame lag by 50 ms so that case always happens, instead of by chance.
    await ctx.addInitScript(() => {
      const raf = window.requestAnimationFrame.bind(window);
      window.requestAnimationFrame = (cb) => raf((t) => cb(t - 50));
    });
    const page = await ctx.newPage();
    const errors = [];
    page.on("pageerror", (e) => errors.push(e.message));
    await page.goto("file://" + path.resolve(__dirname, "../../docs/app/simulator.html"));
    await page.waitForTimeout(500);
    const tag = `${scheme} ${width}px`;
    const settle = () => page.waitForTimeout(150);
    const text = (sel) => page.textContent(sel);
    const pressed = (sel) => page.$$eval(`${sel} button[aria-pressed="true"]`, (bs) => bs.map((b) => b.dataset.value || b.dataset.boat));
    const cellVal = async (row, col) => parseFloat((await page.$eval(`#cmp tr[data-row="${row}"] td:nth-child(${col})`, (e) => e.textContent)).replace("−", "-"));
    const slide = (sel, v) => page.$eval(sel, (el, v) => { el.value = v; el.dispatchEvent(new Event("input")); }, v);
    const secs = (t) => t.split(":").reduce((a, x) => a * 60 + +x, 0);
    const labels0 = () => page.$$eval("#race-stats tbody tr td:first-child", (ts) => ts.map((t) => t.firstChild.textContent));
    const goTab = async (t) => { await page.click(`nav.tabs button[data-tab=${t}]`); await page.waitForTimeout(250); };
    const visible = (sel) => page.isVisible(sel);

    // Layout: one tab at a time; phones put the sliders after it, wide screens beside it.
    const box = async (sel) => (await page.$(sel)).boundingBox();
    check(`${tag}: stroke tab first, the others hidden`, (await visible("#eff-card")) && !(await visible("#hull-card")) && !(await visible("#race-card")));
    const [views, ctl] = [await box("#views-card"), await box("#controls")];
    if (width < 900) check(`${tag}: sliders after the tab`, ctl.y > views.y + views.height - 1);
    else check(`${tag}: sliders in a column beside the results`, ctl.x > views.x + views.width - 1);
    await goTab("race");
    check(`${tag}: race tab: race, conditions, score`, (await visible("#race-card")) && (await visible("#conditions-card")) &&
          (await visible("#score-card")) && !(await visible("#eff-card")) && (await visible("#controls")));
    if (width < 560) {
      const rv = await box("#race-view");
      check(`${tag}: phone: the race view runs edge to edge, drawn bigger`, rv.width >= width - 34 && rv.height > 135,
            `${rv.width.toFixed(0)} × ${rv.height.toFixed(0)} px`);
    }
    await page.click(".race-power button.info");
    check(`${tag}: race power explains what each can hold`, /fitness × the muscle used/.test(await text("#tip")) &&
          (await page.$eval("#tip", (e) => !e.hidden)));
    await page.click(".race-power button.info");
    check(`${tag}: race tab: boat picker and race power on top`, (await visible("#boats")) && (await visible("#effA")) &&
          (await visible("#effB")) && !(await visible("#c-effortW")));
    await goTab("boat");
    await slide("#c-effortW", 250); await settle();
    check(`${tag}: boat tab: power set right there`, (await visible("#c-effortW")) && (await visible("#boats")) &&
          /at 250 W/.test(await text("#hull-sub")) && !(await visible("#effA")), await text("#hull-sub"));
    await page.click("#boats [data-boat=b]"); await settle();
    check(`${tag}: boat tab: pick the boat to edit`, (await text("#editing-name")) === "Boat B");
    await page.click("#boats [data-boat=a]");
    await slide("#c-effortW", 150); await settle();
    await goTab("stroke");

    // Resizing the window redraws the charts at their new width, so their text keeps its size.
    {
      const fit = () => page.$eval("#curve", (e) => Math.abs(+e.getAttribute("viewBox").split(" ")[2] - e.getBoundingClientRect().width));
      await page.setViewportSize({ width: width + 300, height: 900 }); await page.waitForTimeout(400);
      const wide = await fit();
      await page.setViewportSize({ width, height: 900 }); await page.waitForTimeout(400);
      check(`${tag}: a resized window redraws the force curve at its width`, wide < 2 && (await fit()) < 2, `${wide.toFixed(0)} px off`);
    }
    // Efficiency comparison
    check(`${tag}: A and B efficiencies`, (await cellVal("eff", 2)) > 50 && (await cellVal("eff", 3)) > 50);
    // With lift, the wide sweep slices sideways and wins; drag only, the defaults win.
    check(`${tag}: lift on by default`, JSON.stringify(await pressed("#lift-mode")) === '["1"]');
    const sweepLift = await cellVal("eff", 3);
    await page.click("#boats [data-boat=b]");
    await page.click("#lift-mode >> text=Drag only"); await settle();
    check(`${tag}: drag only lowers the sweep below the defaults`, (await cellVal("eff", 3)) < sweepLift &&
          (await cellVal("eff", 2)) > (await cellVal("eff", 3)) && /Drag only/.test(await text("#why-b")));
    await page.click("#presets >> text=Long reach"); await settle();
    check(`${tag}: presets keep the blade force model`, JSON.stringify(await pressed("#lift-mode")) === '["0"]');
    await page.click("#presets >> text=Wide sweep"); await settle();
    await page.click("#boats [data-boat=a]"); await settle();
    check(`${tag}: lift share in the details`, /% of the blade force is lift/.test(await text("#why-a")));
    await slide("#c-pitchDeg", -15); await settle();
    check(`${tag}: pitch readout`, (await text("#o-pitchDeg")) === "15° face turned in, toward the hull");
    await slide("#c-pitchDeg", 0); await settle();
    const peakRow = await page.$eval('#cmp tr[data-row="peak"]', (e) => ({ d: e.querySelector("td.d").textContent, wins: e.querySelectorAll("td.win").length }));
    const peakA = await cellVal("peak", 2), peakB = await cellVal("peak", 3);
    check(`${tag}: peak force has no winner, difference adds up`, peakRow.wins === 0 &&
          peakRow.d === `${peakA >= peakB ? "A" : "B"} pulls ${Math.abs(peakA - peakB)} N harder`, peakRow.d);
    check(`${tag}: winner highlighted`, (await page.$$('#cmp tr[data-row="eff"] td.win.a')).length === 1);
    check(`${tag}: two power bars, six parts`, (await page.$$("#bars-power .bar")).length === 2 &&
          (await page.$$("#leg-power tr")).length === 7);
    // Switching's slice: the extra slip against the same boat with no switch time.
    await page.waitForTimeout(300);
    const swRow = await page.$$eval("#leg-power tr", (rs) => { const r = rs.find((q) => /switching/.test(q.textContent));
      return r ? [...r.querySelectorAll("td.n")].map((td) => parseFloat(td.textContent)) : null; });
    check(`${tag}: switching slice: the wide sweep (B) loses more to switching than A`, swRow && swRow[1] > 1 && swRow[1] > 2 * swRow[0] && swRow[0] > 0,
          swRow && `A ${swRow[0]}%, B ${swRow[1]}%`);
    await slide("#c-swingM", 0.1); await settle();
    const legA = await page.$$eval("#leg-power tr td:nth-child(2)", (ts) => ts.slice(1).reduce((a, t) => a + parseFloat(t.textContent), 0));
    check(`${tag}: power shares add to 100%, body swing included`, Math.abs(legA - 100) < 1.5, `${legA.toFixed(1)}%`);
    await slide("#c-swingM", 0); await settle();
    check(`${tag}: details for both boats`, /switch|turning/i.test(await text("#why-a")) && /B:/.test(await text("#why-b")));
    const s150 = await cellVal("speed", 2);
    await slide("#c-effortW", 300); await settle();
    check(`${tag}: compare-at effort raises speed`, (await cellVal("speed", 2)) > s150 && (await text("#o-effortW")) === "300 W");
    await slide("#c-effortW", 150); await settle();

    // Help tips: tap (or click) shows, again or elsewhere hides; mouse hover shows.
    const tipShown = () => page.$eval("#tip", (e) => !e.hidden);
    const effInfo = '#cmp tr[data-row="eff"] button.info';
    check(`${tag}: help buttons on table rows and breakdowns`, (await page.$$("#cmp button.info")).length === 7 &&
          (await page.$$("#leg-power button.info")).length === 6);
    await page.click(effInfo);
    check(`${tag}: tap shows the tip`, (await tipShown()) && /moves you down the course/.test(await text("#tip")) &&
          (await page.$eval(effInfo, (b) => b.getAttribute("aria-expanded"))) === "true");
    const tb = await (await page.$("#tip")).boundingBox();
    check(`${tag}: tip fits on screen`, tb.x >= 0 && tb.x + tb.width <= width + 1, `${tb.x.toFixed(0)}+${tb.width.toFixed(0)} in ${width}`);
    await page.click(effInfo);
    check(`${tag}: tap again hides it`, !(await tipShown()));
    await page.click('#views-card h2 button.info');
    await page.mouse.click(5, 5);
    check(`${tag}: tap elsewhere hides it`, !(await tipShown()));
    await page.hover('#curve-card button.info');
    check(`${tag}: hover shows a tip`, (await tipShown()) && /impulse/.test(await text("#tip")));
    await page.mouse.move(2, 2);
    check(`${tag}: moving away hides it`, !(await tipShown()));

    // Force curve: A and B through the drive
    check(`${tag}: different settings: no blend note`, (await page.$$("#curve-same")).length === 0 &&
          (await page.$$("#curve text.peak")).length === 2);
    check(`${tag}: force curve has both boats`, (await page.$$("#curve path.ln.a")).length === 1 &&
          (await page.$$("#curve path.ln.b")).length === 1);
    check(`${tag}: curve legend: peak, impulse, drive`,
          /^A: peak \d+ N · impulse [\d.]+ N·s · drive 0\.50 s/.test(await text("#curve-legend")), await text("#curve-legend"));
    check(`${tag}: curve peak matches the table's peak force`,
          (await text("#curve-legend")).includes(`peak ${await cellVal("peak", 2)} N`));
    await page.$eval("#curve-card", (e) => e.scrollIntoView());
    const cb = await (await page.$("#curve")).boundingBox();
    await page.mouse.move(cb.x + cb.width * 0.5, cb.y + cb.height / 2);
    check(`${tag}: hover shows both boats' force`, !(await page.$eval("#curve-tip", (e) => e.hidden)) &&
          /A \d+ N/.test(await text("#curve-tip")) && (await page.$$("#curve-hover circle")).length === 2, await text("#curve-tip"));
    await page.mouse.move(cb.x - 20, cb.y - 20);
    await page.click("#curve-mode >> text=Forward push");
    check(`${tag}: forward push mode`, JSON.stringify(await pressed("#curve-mode")) === '["forward"]' &&
          /impulse/.test(await text("#curve-sub")));
    check(`${tag}: curve table rows`, (await page.$$("#curve-table tr")).length === 12);
    // Live trace follows the animation.
    await page.click("#curve-live");
    await page.waitForFunction(() => document.querySelectorAll("#curve path.trace").length >= 1, null, { timeout: 5000 });
    check(`${tag}: live trace on`, (await page.$eval("#curve-live", (e) => e.getAttribute("aria-pressed"))) === "true" &&
          (await page.$eval("#curve", (e) => e.classList.contains("live"))) &&
          /^A (\d+ N|recovery) · B (\d+ N|recovery)$/.test(await text(".live-read")), await text(".live-read"));
    await page.click("#curve-live"); await settle();
    check(`${tag}: live trace off`, (await page.$$("#curve path.trace")).length === 0 &&
          !(await page.$eval("#curve", (e) => e.classList.contains("live"))));
    await page.click("#curve-mode >> text=Blade force");

    // Boats and presets
    check(`${tag}: editing A, names shown`, JSON.stringify(await pressed("#boats")) === '["a"]' &&
          (await text("#nm-a")) === "Defaults" && (await text("#nm-b")) === "Wide sweep");
    check(`${tag}: A's preset highlighted`, JSON.stringify(await pressed("#presets")) === '["Defaults"]');
    await page.click("#boats [data-boat=b]"); await settle();
    check(`${tag}: switch to B`, JSON.stringify(await pressed("#presets")) === '["Wide sweep"]' &&
          (await page.$eval("#c-sweepDeg", (e) => e.value)) === "70" && (await text("#editing-name")) === "Boat B");
    await page.click("#presets >> text=Short reach, pull behind"); await settle();
    check(`${tag}: preset applies to B only`, (await text("#nm-b")) === "Short reach, pull behind" && (await text("#nm-a")) === "Defaults");
    await slide("#c-catchDeg", 14); await settle();
    check(`${tag}: slider makes B custom`, (await text("#nm-b")) === "Custom" && (await pressed("#presets")).length === 0);
    await page.click("#swap"); await settle();
    check(`${tag}: swap`, (await text("#nm-a")) === "Custom" && (await text("#nm-b")) === "Defaults");
    await page.click("#copy"); await settle();   // editing B: copy B to A
    check(`${tag}: copy`, (await text("#nm-a")) === "Defaults" && (await cellVal("eff", 2)) === (await cellVal("eff", 3)));
    await page.click("#view-mode >> text=Overlay"); await settle();
    check(`${tag}: same settings: overlay blends B over A`, (await page.$$("svg.view g.blend-b")).length === 3 &&
          (await page.$$eval("svg.view", (ss) => ss.map((s) => s.querySelector("g.blend-b") === s.querySelectorAll("g")[1]))).every(Boolean) &&
          (await page.$$eval("svg.view", (ss) => ss.every((s) => s.querySelectorAll(".fig-head").length === 1))));
    await page.click("#view-mode >> text=Split");
    check(`${tag}: same settings: curves blend, one label`, (await page.$$("#curve-same")).length === 1 &&
          (await page.$$eval("#curve text.peak", (ts) => ts.map((t) => t.textContent))).join() === "A = B" &&
          (await page.$$eval("#curve path.ln", (ps) => ps.map((p) => p.classList[1]))).join() === "a,b");
    await page.click("#boats [data-boat=a]");
    await page.click("#presets >> text=Clean vertical");
    await page.click("#boats [data-boat=b]");
    await page.click("#presets >> text=Wide sweep");
    await page.click("#boats [data-boat=a]"); await settle();
    check(`${tag}: drive time readout`, (await text("#o-driveFraction")) === "50% of each stroke");
    await slide("#c-driveFraction", 0.8); await settle();
    check(`${tag}: drive time goes to 80%`, (await page.$eval("#c-driveFraction", (e) => e.max)) === "0.8" &&
          (await text("#o-driveFraction")) === "80% of each stroke" && (await cellVal("eff", 2)) > 0);
    await slide("#c-driveFraction", 0.5); await settle();
    check(`${tag}: strokes per side auto by default`, (await text("#o-switchStrokes")) === "auto, by heading" &&
          !(await page.$eval("#c-switchHeadingDeg", (e) => e.disabled)));
    await slide("#c-switchStrokes", 10); await settle();
    check(`${tag}: fixed strokes per side`, (await text("#o-switchStrokes")) === "10 strokes" &&
          (await page.$eval("#c-switchHeadingDeg", (e) => e.disabled)) && (await cellVal("side", 2)) === 10 &&
          (await text("#o-switchHeadingDeg")) === "off (fixed count)");
    check(`${tag}: wander shown`, /^±[\d.]+°$/.test(await page.$eval('#cmp tr[data-row="wander"] td:nth-child(2)', (e) => e.textContent)));
    await slide("#c-switchStrokes", 0); await settle();
    await slide("#c-rateSpm", 30); await settle();
    check(`${tag}: slower rate, longer drive on the curve`, /^A: .*drive 1\.00 s/.test(await text("#curve-legend")), await text("#curve-legend"));
    await slide("#c-rateSpm", 60); await settle();
    await page.$eval("#c-bendDeg", (el) => { el.value = -12; el.dispatchEvent(new Event("input")); });
    check(`${tag}: bend readout shows direction`, (await text("#o-bendDeg")) === "12° toward power face");
    await page.click("#presets >> text=Clean vertical"); await settle();

    // Views: split, overlay, single
    const svgs = () => page.$$eval("svg.view", (ss) => ss.map((s) => s.dataset.boats));
    check(`${tag}: split shows A and B for each view (with lagging frame timestamps)`,
          JSON.stringify(await svgs()) === '["a","b","a","b","a","b"]' && (await page.$$("svg.view line.blade")).length === 6);
    await page.click("#view-mode >> text=Overlay"); await settle();
    check(`${tag}: overlay draws both boats in one view`, JSON.stringify(await svgs()) === '["ba","ba","ba"]' &&
          (await page.$$("svg.view line.blade.a")).length === 3 && (await page.$$("svg.view line.blade.b")).length === 3);
    await page.click("#view-mode >> text=B only"); await settle();
    check(`${tag}: single boat view`, JSON.stringify(await svgs()) === '["b","b","b"]' && (await page.$$("svg.view .path.b")).length === 3);
    check(`${tag}: view mode highlighted`, JSON.stringify(await pressed("#view-mode")) === '["b"]');
    await page.click("#view-mode >> text=Split");

    // Race
    await goTab("race");
    const verdict = () => text("#verdict");
    check(`${tag}: race verdict`, /^Boat [AB] wins by (\d+(\.\d)? s|\d+:\d\d), \d+ m ahead/.test(await verdict()), await verdict());
    check(`${tag}: point to point by default`, JSON.stringify(await pressed("#course")) === '["point"]');
    check(`${tag}: point to point: no turn mentioned`, /the start isn't timed\.$/.test(await text("#race-sub")));
    await page.click("#course >> text=There and back");
    check(`${tag}: there and back: the turn is mentioned`, /the turn at halfway aren't timed\.$/.test(await text("#race-sub")));
    await page.click("#course >> text=Point to point");
    check(`${tag}: no chip label is cut off`, (await page.$$eval(".chips button:not(.info)", (bs) =>
      bs.filter((b) => b.offsetParent && b.scrollWidth > b.clientWidth + 1).map((b) => b.textContent))).length === 0);
    const t0 = await text("#t-a");
    await slide("#effB", 250);
    check(`${tag}: more effort for B flips the race`, /^Boat B wins/.test(await verdict()), await verdict());
    await slide("#effB", 150);
    await slide("#wind", 20); await slide("#windDir", 90);
    check(`${tag}: wind readouts`, (await text("#o-wind")) === "20 km/h" && (await text("#o-windDir")) === "the side");
    check(`${tag}: crosswind slows and makes switching lopsided`, secs(await text("#t-a")) > secs(t0) &&
          /crab; switching [\d.]+ strokes windward, [\d.]+ lee/.test(await text("#c-a")), await text("#c-a"));
    await page.click("#boats [data-boat=a]");
    await slide("#c-switchStrokes", 10); await settle();
    check(`${tag}: fixed count in a crosswind holds course`, /switching [\d.]+ strokes windward, [\d.]+ lee to hold course, wandering ±[\d.]+°/.test(await text("#c-a")),
          await text("#c-a"));
    await slide("#c-switchStrokes", 0); await settle();
    // Heat: WBGT from temperature and humidity; power falls 0.2% per °C over 10 °C.
    await slide("#wind", 0);
    await slide("#temp", 5); await slide("#humid", 50); await page.waitForTimeout(150);
    const tCool = secs(await text("#t-a"));
    check(`${tag}: heat: a cool day costs nothing`, /^Heat index \d+ °C: no loss\.$/.test(await text("#heat-note")) &&
          (await text("#o-heatLoss")) === "0.20% per °C", await text("#heat-note"));
    // 30 °C, 70%: e = 29.7 hPa, WBGT = 17.0 + 11.7 + 3.9 = 32.6 °C, 22.6 °C over: 4.5% less power.
    await slide("#temp", 30); await slide("#humid", 70); await page.waitForTimeout(150);
    const tHot = secs(await text("#t-a"));
    check(`${tag}: heat: hot and humid, less power, slower`, /^Heat index 33 °C: 4\.5% less power than on a cool day\.$/.test(await text("#heat-note")) &&
          tHot > tCool && (await text("#o-effA")) === "150 W", `${await text("#heat-note")} | ${tCool} → ${tHot}`);
    await slide("#humid", 20); await page.waitForTimeout(150);
    check(`${tag}: heat: drier is easier`, secs(await text("#t-a")) < tHot, await text("#heat-note"));
    await slide("#heatLoss", 0); await page.waitForTimeout(150);
    check(`${tag}: heat: left out`, secs(await text("#t-a")) === tCool && (await text("#o-heatLoss")) === "left out");
    await slide("#heatLoss", 0.2); await slide("#temp", 10); await slide("#humid", 50); await slide("#wind", 20);
    await slide("#windDir", 0);
    check(`${tag}: headwind readout`, (await text("#o-windDir")) === "dead ahead" && (await text("#c-a")) === "");
    await slide("#wind", 0);
    await slide("#depth", 0.6);
    check(`${tag}: depth readout and shallow note`, (await text("#o-depth")) === "0.6 m (2.0 ft)" &&
          /wave drag ×[\d.]+ in the shallows/.test(await text("#c-a")) && secs(await text("#t-a")) > secs(t0), await text("#c-a"));
    await slide("#depth", 5);
    check(`${tag}: deep`, (await text("#o-depth")) === "deep" && (await text("#t-a")) === t0);
    await slide("#current", 4);
    const tWith = await text("#t-a");
    await page.click("#course >> text=There and back");
    check(`${tag}: current helps one way, costs there and back`, secs(tWith) < secs(t0) && secs(await text("#t-a")) > secs(t0) &&
          (await text("#o-current")).endsWith("going out"), `${t0} / ${tWith} / ${await text("#t-a")}`);
    await slide("#current", -10);
    for (const id of ["#effA", "#effB"]) await slide(id, 50);
    check(`${tag}: current too strong`, (await verdict()).startsWith("Neither boat") && (await text("#t-a")) === "never");
    await slide("#current", 0);
    for (const id of ["#effA", "#effB"]) await slide(id, 150);
    await page.click("#course >> text=Point to point");

    // River: lines per boat, map, notes
    const riverShown = () => page.$eval("#river-map", (e) => !e.hasAttribute("hidden"));
    check(`${tag}: open water hides river settings`, !(await riverShown()) && !(await page.isVisible("#width")));
    await page.click("#water >> text=River");
    check(`${tag}: river settings and map`, (await riverShown()) && (await page.isVisible("#width")) &&
          (await text("#l-current")) === "Current mid-channel");
    check(`${tag}: line choices in order, A best, B middle`,
          JSON.stringify(await page.$$eval("#line-a button", (bs) => bs.map((b) => b.textContent))) === '["Middle","Between","Edges","Best line"]' &&
          JSON.stringify([await pressed("#line-a"), await pressed("#line-b")]) === '[["best"],["0"]]');
    await slide("#current", -4); await slide("#depth", 3);
    check(`${tag}: upstream, best line beats the middle`, /^Boat A wins/.test(await verdict()) &&
          /^edges: current [\d.]+ km\/h against you/.test(await text("#c-a")) && /^middle:/.test(await text("#c-b")),
          `${await text("#c-a")} | ${await text("#c-b")}`);
    // A big gap squeezes the bends into the zoomed-out band; zoom in to see the lines.
    const lines = () => page.$$("#river-map .line-a, #river-map .line-b").then((l) => l.length);
    for (let i = 0; i < 6 && (await lines()) === 0; i++) await page.click("#zoom-in");
    check(`${tag}: river map draws both lines`, (await lines()) === 2);
    await page.click("#zoom-fit");
    await page.click("#line-b >> text=Edges");
    await slide("#current", 4);
    check(`${tag}: downstream, edges lose`, /^Boat A wins/.test(await verdict()) && /^middle:/.test(await text("#c-a")), await text("#c-a"));
    await page.click("#course >> text=There and back");
    check(`${tag}: there and back notes per leg`, /^out: middle: .*; back: edges: /.test(await text("#c-a")), await text("#c-a"));
    await slide("#bends", 0);
    check(`${tag}: straight river readout`, (await text("#o-bends")) === "straight");
    await slide("#bends", 180); await slide("#current", 0); await slide("#depth", 5);
    await page.click("#course >> text=Point to point");
    await page.click("#water >> text=Open water");
    check(`${tag}: back to open water`, !(await riverShown()) && (await text("#l-current")) === "Current");

    // Replay: scrub through the race; stats and the speed chart follow.
    check(`${tag}: stats rows`, JSON.stringify(await labels0()) ===
          '["Time","Distance","Speed now","Avg speed","Power","Stroke rate","Strokes","Distance per stroke","Avg distance per stroke","Switches","Energy","Gap"]',
          JSON.stringify(await labels0()));
    check(`${tag}: replay starts at the winner's finish`, /✓/.test(await page.$eval("#race-stats tbody tr:first-child", (r) => r.textContent)));
    await page.$eval("#scrub", (el) => { el.value = 500; el.dispatchEvent(new Event("input")); });
    const dist = () => page.$$eval('#race-stats tbody tr:nth-child(2) td.v', (ts) => ts.map((t) => parseFloat(t.textContent)));
    const [dA5, dB5] = await dist();
    check(`${tag}: scrub halfway`, dA5 > 0.5 && dA5 < 9.5 && dB5 > 0.5 && dB5 < 9.5 && /^\d+:\d\d(:\d\d)? · [\d.]+ of 10.00 km$/.test(await text("#scrub-time")),
          `${dA5} / ${dB5} km, ${await text("#scrub-time")}`);
    await page.click("#scrub-fwd");
    check(`${tag}: step forward`, (await page.$eval("#scrub", (e) => +e.value)) === 510 && (await dist())[0] > dA5);
    await page.click("#scrub-back"); await page.click("#scrub-back");
    check(`${tag}: step back`, (await page.$eval("#scrub", (e) => +e.value)) === 490);
    // One scale for the whole race: the boats' on-screen gap grows as the real gap does.
    const pxGap = async (v, sel = "#race-view") => {
      await page.$eval("#scrub", (el, v) => { el.value = v; el.dispatchEvent(new Event("input")); }, v);
      return page.$$eval(`${sel} path.boat-a, ${sel} path.boat-b`, (ps) =>
        Math.abs(ps.map((p) => parseFloat(/translate\(([-\d.]+)/.exec(p.getAttribute("transform"))[1])).reduce((x, y) => x - y)));
    };
    const [g2, g6, g9] = [await pxGap(200), await pxGap(600), await pxGap(900)];
    check(`${tag}: lanes gap grows through the race`, g2 < g6 && g6 < g9 && g9 > 20, `${g2} < ${g6} < ${g9} px`);
    // Zoom: Fit by default; zoom out to the whole course; zoom in hides the trailing boat behind a marker.
    const dis = (id) => page.$eval(id, (e) => e.disabled);
    const ticks = () => page.$$eval("#race-view text", (ts) => ts.map((t) => t.textContent).join(" "));
    check(`${tag}: zoom starts at Fit`, (await dis("#zoom-fit")) && !(await dis("#zoom-in")) && !(await dis("#zoom-out")));
    for (let i = 0; i < 12 && !(await dis("#zoom-out")); i++) await page.click("#zoom-out");
    check(`${tag}: zoomed out shows the whole course`, /^0 m .*10 km A B$/.test(await ticks()) && !(await dis("#zoom-fit")), await ticks());
    await page.click("#zoom-fit");
    for (let i = 0; i < 3; i++) await page.click("#zoom-in");
    check(`${tag}: zoomed in: a boat behind the view shows how far back`, /◀ \d+ m/.test(await ticks()) &&
          (await page.$$("#race-view path.boat-a, #race-view path.boat-b")).length === 1, await ticks());
    await page.click("#zoom-fit");
    const zbox = await page.$eval("#race-view", (e) => { e.scrollIntoView({ block: "center" }); const r = e.getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; });
    await page.mouse.move(...zbox); await page.keyboard.down("Control"); await page.mouse.wheel(0, -300); await page.keyboard.up("Control");
    check(`${tag}: Ctrl + scroll zooms`, !(await dis("#zoom-fit")));
    await page.click("#zoom-fit");
    await page.$eval("#scrub", (el) => { el.value = 490; el.dispatchEvent(new Event("input")); });
    check(`${tag}: speed chart: two boats with markers`, (await page.$$("#race-speed path.ln")).length === 2 &&
          (await page.$$("#race-speed circle.dot")).length === 2);
    // River with bends: the chart shades bends and the replay says where each boat is.
    await page.click("#water >> text=River");
    await slide("#current", -3); await slide("#depth", 2.5);
    check(`${tag}: bends shaded`, (await page.$$("#race-speed rect.bendband")).length > 5);
    check(`${tag}: river view replaces the lanes, with both boats on it`, (await page.$eval("#race-view", (e) => e.hasAttribute("hidden"))) &&
          !(await page.$eval("#river-map", (e) => e.hasAttribute("hidden"))) && (await page.$$("#river-map path.boat-a, #river-map path.boat-b")).length === 2);
    const [r2, r9] = [await pxGap(200, "#river-map"), await pxGap(900, "#river-map")];
    const rw = await page.$eval("#river-map", (e) => e.viewBox.baseVal.width);
    check(`${tag}: river gap grows to a clear share of the view`, r2 < r9 && r9 > rw * 0.2, `${r2} → ${r9} of ${rw} px`);
    // Scrubbing never shifts the page below the replay text.
    const wheres = new Set(), below = new Set();
    for (let v = 0; v <= 1000; v += 7) {
      await page.$eval("#scrub", (el, v) => { el.value = v; el.dispatchEvent(new Event("input")); }, v);
      wheres.add(await text("#scrub-where"));
      below.add(await page.$eval("#race-start", (e) => Math.round(e.getBoundingClientRect().top + scrollY)));
    }
    check(`${tag}: scrubbing doesn't move the page`, below.size === 1, [...below].join(", "));
    const all = [...wheres].join(" | ");
    for (let i = 0; i < 12 && !(await dis("#zoom-out")); i++) await page.click("#zoom-out");
    check(`${tag}: river zoomed out: a plain band, both boats`, (await page.$$("#river-map .line-a")).length === 0 &&
          (await page.$$("#river-map path.boat-a, #river-map path.boat-b")).length === 2 && /bends not drawn/.test(await text("#river-map")));
    await page.click("#zoom-fit");
    check(`${tag}: replay passes straights, crossings and bends`, /along the bank/.test(all) && /crossing the river/.test(all) &&
          /inside of a bend/.test(all), all.slice(0, 160));
    await slide("#current", 0); await slide("#depth", 5);
    await page.click("#water >> text=Open water");

    // Live race, open water: the gap keeps growing on screen (the view doesn't zoom out with it).
    await page.evaluate(() => { window.sim.raceWallS = 4; });
    const liveGap = () => page.$$eval("#race-view path.boat-a, #race-view path.boat-b", (ps) =>
      Math.abs(ps.map((p) => parseFloat(/translate\(([-\d.]+)/.exec(p.getAttribute("transform"))[1])).reduce((x, y) => x - y)));
    await page.click("#race-start");
    await page.waitForFunction(() => window.sim.run && window.sim.run.A.d > 4000, null, { timeout: 8000 });
    const lg1 = await liveGap();
    await page.waitForFunction(() => window.sim.run && window.sim.run.A.d > 8000, null, { timeout: 8000 });
    const lg2 = await liveGap();
    check(`${tag}: live race: lanes gap keeps growing`, lg2 > lg1 * 1.5, `${lg1} → ${lg2} px`);
    await page.click("#race-stop");
    // Race controls: pause, resume, slow motion, stop; a change mid-race applies from then on.
    await page.evaluate(() => { window.sim.raceWallS = 1.5; });
    const run = () => page.evaluate(() => window.sim.run && { t: window.sim.run.t, dA: window.sim.run.A.d, paused: window.sim.run.paused });
    // The slider runs from the start line during a race; scrubbing pauses it.
    await page.click("#race-start");
    await page.waitForTimeout(250);
    const sv = () => page.$eval("#scrub", (e) => +e.value);
    check(`${tag}: slider starts from 0 on a new race`, (await sv()) > 0 && (await sv()) < 600, `${await sv()}`);
    await page.$eval("#scrub", (el) => { el.value = 300; el.dispatchEvent(new Event("input")); });
    check(`${tag}: scrubbing a run pauses it`, (await run()).paused && (await text("#race-pause")) === "Resume");
    check(`${tag}: can't scrub past what's been raced`, (await run()).t >= (await page.evaluate(() => window.sim.run.t)) &&
          (await page.$eval("#scrub-time", (e) => e.textContent)).length > 0);
    await page.click("#race-stop");
    check(`${tag}: pause and stop start disabled`, await page.$eval("#race-pause", (b) => b.disabled) && await page.$eval("#race-stop", (b) => b.disabled));
    await page.click("#race-start");
    await page.waitForTimeout(300);
    await page.click("#race-pause");
    const p1 = await run(); await page.waitForTimeout(250); const p2 = await run();
    check(`${tag}: pause holds the race`, p1.paused && p2.t === p1.t && p2.dA === p1.dA && /^Paused at /.test(await text("#race-status")));
    await page.click("#race-pause");
    await page.waitForTimeout(150);
    check(`${tag}: resume`, (await run()).t > p1.t && (await text("#race-pause")) === "Pause");
    await page.click("#race-stop");
    check(`${tag}: stop`, !(await run()) && (await text("#race-status")) === "Stopped." && await page.$eval("#race-pause", (b) => b.disabled));
    check(`${tag}: stop puts the slider back at the start`, (await page.$eval("#scrub", (e) => +e.value)) === 0 &&
          /^0:00 · 0.00 of/.test(await text("#scrub-time")), await text("#scrub-time"));
    await page.click("#race-slow");
    check(`${tag}: slow motion toggle`, (await page.$eval("#race-slow", (b) => b.getAttribute("aria-pressed"))) === "true");
    await page.click("#race-slow");
    // Mid-race change: B goes to 400 W halfway, so B finishes between its 150 W and 400 W times.
    const tB150 = await page.evaluate(() => window.sim.race.tB);
    await page.click("#race-start");
    await page.waitForFunction(() => window.sim.run && window.sim.run.B.d > window.sim.race.metres / 2, null, { timeout: 5000 });
    await slide("#effB", 400);
    const tB400 = await page.evaluate(() => window.sim.race.tB);
    await page.waitForFunction(() => !window.sim.run, null, { timeout: 8000 });
    const raced = await text("#race-status");
    const finB = secs(raced.match(/B (\d+:\d\d(:\d\d)?)/)[1]);
    check(`${tag}: mid-race change applies from then on`, finB < tB150 - 1 && finB > tB400 + 1 && /changed mid-race/.test(raced),
          `${raced} (150 W ${tB150.toFixed(0)} s, 400 W ${tB400.toFixed(0)} s)`);
    const labels = () => page.$$eval("#race-stats tbody tr td:first-child", (ts) => ts.map((t) => t.textContent));
    check(`${tag}: replay after a run with a power change shows avg power`, (await labels()).includes("Avg power"));
    await slide("#effB", 150);
    check(`${tag}: changing settings after a run goes back to the prediction`, !(await labels()).includes("Avg power"));
    // A whole race: it stops at the last boat's finish, both times listed, matching the prediction.
    await page.click("#race-start");
    await page.waitForFunction(() => !window.sim.run, null, { timeout: 8000 });
    const fin = await text("#race-status"), timeCells = await page.$$eval("#race-stats tbody tr:first-child td.v", (ts) => ts.map((t) => t.textContent));
    const gapCells = await page.$$eval("#race-stats tbody tr:last-child td.v", (ts) => ts.map((t) => t.textContent));
    check(`${tag}: race stops at the last finish, both times listed and as predicted`,
          fin === `Finished: A ${await text("#t-a")}, B ${await text("#t-b")}.` + (/ by /.test(fin) ? fin.slice(fin.indexOf(". ") + 1) : "") &&
          timeCells.every((c) => / ✓$/.test(c)) && (await page.$eval("#scrub", (e) => +e.value)) === 1000 &&
          (gapCells.every((c) => c === "level") || gapCells.some((c) => /^\+\d+:\d\d$/.test(c))), `${fin} | ${timeCells} | ${gapCells}`);
    await page.evaluate(() => { window.sim.raceWallS = 12; });

    await page.click("#race-start");
    await page.waitForTimeout(600);
    check(`${tag}: race animates two boats`, (await page.$$("#race-view .boat-a, #race-view .boat-b")).length === 2);
    check(`${tag}: no J-stroke wording`, !/J-stroke|steering stroke/i.test(await text("body")));

    if (process.argv[2]) {
      await page.waitForTimeout(400);
      await page.screenshot({ path: path.join(process.argv[2], `sim-${scheme}.png`), fullPage: true });
    }
    // Open-Meteo replies (mocked): now, or every hour from start_date to end_date. 60 km/h at 21:00
    // on 2023-07-29, 5 km/h otherwise; 18 °C, 60%; the course's clock 4 h behind UTC.
    const meteo = [];
    const meteoReply = (route) => {
      const url = new URL(route.request().url()); meteo.push(url.href);
      const q = (k) => url.searchParams.get(k);
      let body;
      if (q("current")) body = { utc_offset_seconds: -14400, current: { time: "2026-10-16T12:00", wind_speed_10m: 25, wind_direction_10m: 222, temperature_2m: 28.4, relative_humidity_2m: 71 } };
      else {
        const time = [];
        for (let t = Date.parse(q("start_date")); t <= Date.parse(q("end_date")) + 23 * 3600e3; t += 3600e3) time.push(new Date(t).toISOString().slice(0, 16));
        body = { utc_offset_seconds: -14400, hourly: { time, wind_speed_10m: time.map((h) => (h === "2023-07-29T21:00" ? 60 : 5)), wind_direction_10m: time.map(() => 95),
          temperature_2m: time.map(() => 18), relative_humidity_2m: time.map(() => 60) } };
      }
      route.fulfill({ status: 200, contentType: "application/json", headers: { "access-control-allow-origin": "*" }, body: JSON.stringify(body) });
    };
    const noGauges = (route) => route.fulfill({ status: 200, contentType: "application/json", headers: { "access-control-allow-origin": "*" }, body: '{"value":{"timeSeries":[]}}' });
    await page.route("https://*.open-meteo.com/**", meteoReply);
    await page.route("https://waterservices.usgs.gov/**", noGauges);
    // Real course: the AuSable preset, sections to edit, a GPX of your own.
    await page.click("#water >> text=AuSable / GPX");
    await page.waitForFunction(() => document.querySelectorAll("#cb-secs tbody tr").length > 10, null, { timeout: 10000 });
    // It loads with the weather and flow of the 2023 race its track is from, hour by hour from the start.
    const race = "2023-07-29";
    await page.waitForFunction(() => /^\d{4} race start/.test(document.getElementById("cb-weather-info").textContent) &&
      /No USGS gauges/.test(document.getElementById("cb-gauge-info").textContent), null, { timeout: 5000 });
    check(`${tag}: real course: the AuSable loads with its race-day weather and flow`,
          (await page.$eval("#cb-weather-time", (e) => e.value)) === `${race}T21:00` &&
          (await text("#cb-weather-info")).startsWith(`${race.slice(0, 4)} race start, ${race} 21:00: 18 °C, 60% humidity`) &&
          /The race meets it hour by hour \(48 h\): 18–18 °C, wind 4–48 km\/h at the paddler\.$/.test(await text("#cb-weather-info")) &&
          (await page.$eval("#wind", (e) => e.value)) === "40" &&
          /^Hour by hour: heat index \d+–\d+ °C/.test(await text("#heat-note")) &&
          (await page.$eval("#temp", (e) => e.value)) === "18" && (await page.$eval("#humid", (e) => e.value)) === "60" &&
          (await page.$eval("#cb-gauge-date", (e) => e.value)) === race,
          `${await page.$eval("#cb-weather-time", (e) => e.value)} | ${await text("#cb-weather-info")} | ${meteo.slice(-1)}`);
    await page.unroute("https://*.open-meteo.com/**");
    await page.unroute("https://waterservices.usgs.gov/**");
    await page.fill("#cb-weather-time", ""); await page.fill("#cb-gauge-date", "");
    const rows = () => page.$$eval("#cb-secs tbody tr", (r) => r.length);
    check(`${tag}: real course: the AuSable loads in sections`, (await rows()) >= 20 &&
          /^AuSable River Canoe Marathon · 180\.\d km · 6 portages/.test(await text("#cb-info")) && !(await page.isVisible("#dist")), await text("#cb-info"));
    check(`${tag}: real course: raced on the map`, (await page.isVisible("#course-view")) && !(await page.isVisible("#race-view")) &&
          (await page.$$("#course-view path.boat-a, #course-view path.boat-b")).length === 2 && /^\d+:\d\d:\d\d$/.test(await text("#t-a")) &&
          /^river \d+ km \(.+\) · ponds \d+ km · 6 portages, \d+:\d\d carrying$/.test(await text("#c-a")), await text("#c-a"));
    await page.click("#cb-secs tbody tr:nth-child(2)");
    check(`${tag}: real course: a row opens its section`, (await text("#cb-sec-name")).startsWith("2. River to Mio Pond") &&
          / bends per km$/.test(await text("#cb-sec-geo")), await text("#cb-sec-geo"));
    const tCb = secs(await text("#t-a"));
    await slide("#cb-current", 6); await page.waitForTimeout(250);
    check(`${tag}: real course: more current, faster`, secs(await text("#t-a")) < tCb && /^6\.0$/.test(await page.$eval("#cb-secs tbody tr:nth-child(2) td:nth-child(4)", (e) => e.textContent)));
    const n0 = await rows();
    await page.click("#cb-split"); check(`${tag}: real course: split`, (await rows()) === n0 + 1 && / a$/.test(await text("#cb-sec-name")));
    await page.click("#cb-merge"); check(`${tag}: real course: merge`, (await rows()) === n0);
    await page.click("#cb-kind >> text=Pond");
    check(`${tag}: real course: make it a pond`, !/bends/.test(await text("#cb-sec-geo")) && !(await page.isVisible("#cb-width")) &&
          (await page.$eval("#cb-secs tbody tr:nth-child(2) td:nth-child(4)", (e) => e.textContent)) === "—");
    await page.$eval("#scrub", (el) => { el.value = 500; el.dispatchEvent(new Event("input")); });
    check(`${tag}: real course: replay names the section`, /^A (River to|.+ Pond)/.test(await text("#scrub-where")), await text("#scrub-where"));
    for (let i = 0; i < 12 && !(await page.$eval("#zoom-out", (e) => e.disabled)); i++) await page.click("#zoom-out");
    check(`${tag}: real course: zoomed out, the whole course`, /Whole course/.test(await text("#course-view")));
    await page.click("#zoom-fit");
    // Current from USGS gauges (replies mocked): two on the course, one up a tributary.
    const pts = await page.evaluate(() => [window.COURSES.ausable.points[200], window.COURSES.ausable.points[2000]]);
    const series = (id, name, p, cfs, stat = "00000") => ({ name: `USGS:${id}:00060:${stat}`, sourceInfo: { siteName: name, siteCode: [{ value: id }],
      geoLocation: { geogLocation: { latitude: p[0], longitude: p[1] } } }, values: [{ value: [{ value: String(cfs), dateTime: "2023-07-29T00:00:00.000" }] }] });
    const asked = [];
    await page.route("https://waterservices.usgs.gov/**", (route) => {
      const url = route.request().url(); asked.push(url);
      const past = url.includes("/dv/");
      const ts = past ? [series("0001", "UP RIVER AT ONE, MI", pts[0], 200, "00003"), series("0002", "DOWN RIVER AT TWO, MI", pts[1], 900, "00003")]
        : [series("0001", "UP RIVER AT ONE, MI", pts[0], 300), series("0002", "DOWN RIVER AT TWO, MI", pts[1], 1000),
           series("0003", "SIDE CREEK, MI", [pts[0][0] + 0.05, pts[0][1]], 50)];
      route.fulfill({ status: 200, contentType: "application/json", headers: { "access-control-allow-origin": "*" }, body: JSON.stringify({ value: { timeSeries: ts } }) });
    });
    const row1 = () => page.$eval("#cb-secs tbody tr:nth-child(1) td:nth-child(4)", (e) => e.textContent);
    const gaugeInfo = () => text("#cb-gauge-info");
    await page.click("#cb-secs tbody tr:nth-child(1)");
    await page.click("#cb-gauges");
    await page.waitForFunction(() => /gauge/.test(document.getElementById("cb-gauge-info").textContent) && !/Asking/.test(document.getElementById("cb-gauge-info").textContent));
    // 300 ft³/s in 30 m × 1.2 m: u = 15Q / (8WD) = 0.44 m/s = 1.6 km/h.
    check(`${tag}: gauges: two on the course, the creek left out`, /^2 gauges on the course, latest reading\./.test(await gaugeInfo()) &&
          (await page.$$("#cb-gauge-list tbody tr")).length === 2 && (await row1()) === "1.6" &&
          /^Current from Up River at One, MI: 300 ft³\/s/.test(await text("#cb-sec-gauge")), `${await gaugeInfo()} | ${await row1()} | ${await text("#cb-sec-gauge")}`);
    await page.click('#cb-gauge-list input[data-g="0"]');
    check(`${tag}: gauges: untick one, the next nearest feeds it`, (await row1()) === "5.3", await row1());
    await page.click('#cb-gauge-list input[data-g="0"]');
    await slide("#cb-current", 2);
    check(`${tag}: gauges: a hand-set current stays`, (await row1()) === "2.0" && (await text("#cb-sec-gauge")) === "Current set by hand");
    await page.fill("#cb-gauge-date", "2023-07-29");
    await page.click("#cb-gauges");
    await page.waitForFunction(() => /daily mean/.test(document.getElementById("cb-gauge-info").textContent));
    check(`${tag}: gauges: a past day uses its daily mean`, (await row1()) === "1.1" && asked.some((u) => u.includes("/dv/") && u.includes("startDT=2023-07-29")),
          `${await row1()} | ${asked.slice(-1)}`);
    await page.unroute("https://waterservices.usgs.gov/**");
    await page.route("https://waterservices.usgs.gov/**", (route) => route.abort());
    await page.click("#cb-gauges");
    await page.waitForFunction(() => /Couldn't reach USGS/.test(document.getElementById("cb-gauge-info").textContent));
    check(`${tag}: gauges: no connection says so`, true);
    await page.unroute("https://waterservices.usgs.gov/**");
    // Wind from Open-Meteo (replies mocked): now, a past hour from the archive, and no connection.
    await page.route("https://*.open-meteo.com/**", meteoReply);
    const weatherInfo = () => text("#cb-weather-info");
    const tCalm = secs(await text("#t-a"));
    await page.click("#cb-weather");
    await page.waitForFunction(() => /at the paddler/.test(document.getElementById("cb-weather-info").textContent));
    await page.waitForTimeout(250);
    // 25 km/h at 10 m is 20 km/h at the paddler; 222° rounds to 225°, the SW.
    check(`${tag}: weather: now sets wind, temperature and humidity`, (await page.$eval("#wind", (e) => e.value)) === "20" &&
          (await page.$eval("#temp", (e) => e.value)) === "28" && (await page.$eval("#humid", (e) => e.value)) === "70" &&
          (await page.$eval("#windDir", (e) => e.value)) === "225" && (await text("#o-windDir")) === "SW" &&
          /^Now: 28 °C, 71% humidity; wind 25 km\/h from the SW at 10 m, 20 km\/h at the paddler\.$/.test(await weatherInfo()) &&
          meteo.some((u) => u.startsWith("https://api.open-meteo.com/v1/forecast") && /latitude=4\d\.\d{4}/.test(u)) &&
          secs(await text("#t-a")) !== tCalm, `${await weatherInfo()} | ${meteo.slice(-1)}`);
    await page.fill("#cb-weather-time", "2023-07-29T21:00");
    await page.click("#cb-weather");
    await page.waitForFunction(() => /^2023/.test(document.getElementById("cb-weather-info").textContent));
    check(`${tag}: weather: a past hour comes from the archive`, (await page.$eval("#wind", (e) => e.value)) === "40" &&
          (await page.$eval("#windDir", (e) => e.value)) === "90" && (await page.$eval("#temp", (e) => e.value)) === "18" &&
          /hour by hour \(48 h\): 18–18 °C, wind 4–48 km\/h at the paddler\.$/.test(await weatherInfo()) &&
          meteo.some((u) => u.startsWith("https://archive-api.open-meteo.com/v1/archive") && u.includes("start_date=2023-07-28") && u.includes("end_date=2023-07-31")),
          `${await weatherInfo()} | ${meteo.slice(-1)}`);
    await slide("#wind", 0); await page.waitForTimeout(150);
    check(`${tag}: weather: moving the wind takes it off the weather`, (await weatherInfo()) === "Weather is set by hand." &&
          /^Heat index/.test(await text("#heat-note")));
    await page.unroute("https://*.open-meteo.com/**");
    await page.route("https://*.open-meteo.com/**", (route) => route.abort());
    await page.fill("#cb-weather-time", "");
    await page.click("#cb-weather");
    await page.waitForFunction(() => /Couldn't get the weather/.test(document.getElementById("cb-weather-info").textContent));
    check(`${tag}: weather: no connection says so`, (await page.$eval("#wind", (e) => e.value)) === "0");
    await page.unroute("https://*.open-meteo.com/**");
    // An AuSable gauge the course knows: current scales from its race-week median as (Q/median)^0.55.
    for (const [cfs, want, label] of [[788, "2.7", "Average"], [1576, "4.0", "Whoa"]]) {
      await page.route("https://waterservices.usgs.gov/**", (route) => route.fulfill({ status: 200, contentType: "application/json",
        headers: { "access-control-allow-origin": "*" }, body: JSON.stringify({ value: { timeSeries: [series("04136500", "AU SABLE RIVER AT MIO, MI", pts[0], cfs)] } }) }));
      await page.fill("#cb-gauge-date", "");
      await page.click("#cb-gauges");
      await page.waitForFunction(() => /1 gauge on the course/.test(document.getElementById("cb-gauge-info").textContent));
      await page.click("#cb-secs tbody tr:nth-child(1)");
      check(`${tag}: gauges: Mio at ${cfs} ft³/s is ${label.toLowerCase()} for race week, current ${want} km/h`, (await row1()) === want &&
            new RegExp(`${label} for race week`).test(await text("#cb-gauge-list")) &&
            new RegExp(`${label.toLowerCase()} for race week \\(2\\.7 km/h in a typical year\\)`).test(await text("#cb-sec-gauge")),
            `${await row1()} | ${await text("#cb-sec-gauge")}`);
      await page.unroute("https://waterservices.usgs.gov/**");
    }
    // Your own GPX: a 3 km straight with a dam halfway.
    const trk = Array.from({ length: 61 }, (_, i) => `<trkpt lat="44.0" lon="${(-84 + (i / 60) * 0.03745).toFixed(6)}">` +
      `<time>${new Date(Date.parse("2023-07-30T01:00:00Z") + i * 60e3).toISOString()}</time></trkpt>`).join("");
    await page.route("https://*.open-meteo.com/**", meteoReply);
    await page.route("https://waterservices.usgs.gov/**", noGauges);
    // Saved months later (its metadata time); the race's own time is on the track.
    const gpx = `<?xml version="1.0"?><gpx version="1.1" xmlns="http://www.topografix.com/GPX/1/1"><metadata><time>2024-01-05T12:00:00Z</time></metadata><wpt lat="44.0003" lon="-83.98127"><name>TEST DAM</name></wpt>` +
      `<wpt lat="44.0" lon="-83.99"><name>FEED: Test Dam</name></wpt><trk><name>t</name><trkseg>${trk}</trkseg></trk></gpx>`;
    await page.setInputFiles("#cb-file", { name: "mine.gpx", mimeType: "application/gpx+xml", buffer: Buffer.from(gpx) });
    await page.waitForFunction(() => /^mine\.gpx/.test(document.getElementById("cb-info").textContent), null, { timeout: 5000 });
    check(`${tag}: real course: import a GPX`, /^mine\.gpx · 3\.0 km · 1 portage from waypoints$/.test(await text("#cb-info")) &&
          JSON.stringify(await page.$$eval("#cb-secs tbody tr td:nth-child(3)", (ts) => ts.map((t) => t.textContent))) ===
          '["River to Test Dam","Test Dam portage","River to the finish"]', await text("#cb-info"));
    // Recorded 01:00 UTC on the 30th: 21:00 on the 29th there, the hour of the 60 km/h wind.
    await page.waitForFunction(() => /^Race start from the file/.test(document.getElementById("cb-weather-info").textContent) &&
      /No USGS gauges/.test(document.getElementById("cb-gauge-info").textContent), null, { timeout: 5000 });
    check(`${tag}: real course: a GPX loads with the weather from its race start, not when it was saved`,
          (await page.$eval("#cb-weather-time", (e) => e.value)) === "2023-07-29T21:00" && (await page.$eval("#wind", (e) => e.value)) === "40" &&
          /^Race start from the file, 2023-07-29 21:00: 18 °C/.test(await text("#cb-weather-info")) &&
          (await page.$eval("#cb-gauge-date", (e) => e.value)) === "2023-07-29", await text("#cb-weather-info"));
    await page.unroute("https://*.open-meteo.com/**");
    await page.unroute("https://waterservices.usgs.gov/**");
    await slide("#wind", 0); await slide("#windDir", 0); await slide("#temp", 10); await slide("#humid", 50);
    await page.setInputFiles("#cb-file", { name: "nope.gpx", mimeType: "text/plain", buffer: Buffer.from("hello") });
    await page.waitForFunction(() => /nope\.gpx/.test(document.getElementById("cb-info").textContent), null, { timeout: 5000 });
    check(`${tag}: real course: a bad file says so`, /^Couldn't read nope\.gpx: /.test(await text("#cb-info")), await text("#cb-info"));
    await page.click("#water >> text=Open water");
    check(`${tag}: back from the real course`, (await page.isVisible("#dist")) && (await page.isVisible("#race-view")) && !(await page.isVisible("#builder")));
    // On the hull: Overlay by default (both boats in each picture), Split, one boat; power, shallow water.
    await goTab("boat");
    check(`${tag}: hull: one part at a time`, (await visible("#hp-surge")) && !(await visible("#hp-roll")) &&
          JSON.stringify(await pressed("#hull-part")) === '["surge"]');
    await page.click("#hull-part >> text=Bob, pitch, roll"); await settle();
    check(`${tag}: hull: pick a part`, (await visible("#hp-roll")) && !(await visible("#hp-surge")));
    await page.click("#hull-part >> text=Through a stroke"); await settle();
    check(`${tag}: hull: overlay by default, both boats in each picture`, JSON.stringify(await pressed("#hull-mode")) === '["overlay"]' &&
          (await page.$$("#hp-chart svg")).length === 1 && (await page.$$("#hp-chart path.ln")).length === 6 &&
          (await page.$$("#hp-surge path.hull")).length === 2 && (await page.$$("#hp-roll path.hull")).length === 2 &&
          /^Boat Amoves the boat: \d+%Boat Bmoves the boat: \d+%/.test(await text("#hull-power")) &&
          /^A Speed swings [\d.]+–[\d.]+ km\/h, costing [\d.]+%.*B Speed swings/.test(await text("#hull-surge-read")), await text("#hull-surge-read"));
    // The power chart's note sits under its legend, however many rows, inside the drawing.
    const powerFit = await page.evaluate(() => {
      const svg = document.getElementById("hull-power"), ts = [...svg.querySelectorAll("text")], note = ts[ts.length - 1];
      const legendY = Math.max(...ts.filter((t) => t !== note && +t.getAttribute("y") > 100).map((t) => +t.getAttribute("y")));
      return { gap: +note.getAttribute("y") - legendY, inside: +note.getAttribute("y") <= svg.viewBox.baseVal.height };
    });
    check(`${tag}: hull: the power chart's note clears its legend`, powerFit.gap >= 14 && powerFit.inside, JSON.stringify(powerFit));
    // Bob, pitch and roll each draw the paddler with the paddle in hand.
    const figs = await page.evaluate(() => ["hp-surge", "hp-bob", "hp-pitch", "hp-roll"].map((id) => {
      const el = document.getElementById(id); return [el.querySelectorAll(".fig-head").length, el.querySelectorAll(".paddle-hv").length];
    }));
    check(`${tag}: hull: surge, bob, pitch and roll draw the paddler and paddle`, figs.every(([h, p]) => h >= 1 && p >= 1), JSON.stringify(figs));
    // Pitch shows the push and the drag; the power chart's legend gives both boats.
    const pitchArrows = await page.$$eval("#hp-pitch line[marker-end]", (ls) => ls.length);
    const legendAB = /Hull: friction\s*[\d.]+%\s*[\d.]+%/.test(await text("#hull-power")) && /A saves|A costs/.test(await text("#hull-power"));
    check(`${tag}: hull: pitch draws push and drag; power legend has A and B`, pitchArrows >= 2 && legendAB, `${pitchArrows} arrows`);
    // Arrowheads: one definition each, outside every picture that can be hidden.
    const arrowheads = await page.evaluate(() => ["ha-a", "ha-b", "ha3"].map((id) => {
      const ms = document.querySelectorAll(`marker#${id}`);
      return ms.length === 1 && !ms[0].closest(".hpair, .hull-part, [hidden]");
    }));
    check(`${tag}: hull: arrowheads defined once, where they always draw`, arrowheads.every(Boolean), JSON.stringify(arrowheads));
    check(`${tag}: hull: roll and its balance`, /^A Lifts the paddle side up to \d+ N·m; balance with hips \d+ mm toward the paddle side\. Held still: ±[\d.]+°/.test(await text("#hull-roll-read")), await text("#hull-roll-read"));
    await page.$eval("#hull-card", (e) => e.scrollIntoView());   // it only redraws while on screen
    const surgeSvg = () => page.$eval("#hp-surge", (e) => e.innerHTML);
    const surge1 = await surgeSvg(); await page.waitForTimeout(300);
    check(`${tag}: hull: the drawings move with the stroke`, (await surgeSvg()) !== surge1);
    await page.click("#hull-mode >> text=Split");
    check(`${tag}: hull: split, a picture per boat`, (await page.$$("#hp-chart svg")).length === 2 && (await page.$$("#hp-surge svg")).length === 2 &&
          (await page.$$("#hp-chart path.ln")).length === 6 && (await page.$$('#hp-surge svg[data-boats="b"] path.hull.boat-b')).length === 1);
    await page.click("#hull-mode >> text=B only");
    check(`${tag}: hull: B only`, JSON.stringify(await pressed("#hull-mode")) === '["b"]' && (await page.$$("#hp-chart path.ln")).length === 3 &&
          !/^A /.test(await text("#hull-heading-read")) && (await page.$$("#hp-surge path.hull.boat-a")).length === 0);
    await page.click("#hull-mode >> text=Overlay");
    // Race score: the race's own times, then what each loss costs.
    const scoreReady = () => page.waitForFunction(() => document.getElementById("score").dataset.ready === "1", null, { timeout: 60000 });
    await scoreReady();
    const heads = await page.$$eval("#score-heads .big", (bs) => bs.map((b) => b.textContent));
    check(`${tag}: race score: the race's own times`, heads[0] === (await text("#t-a")) && heads[1] === (await text("#t-b")) &&
          /^\d+(\.\d)? km (open water|river).*, (\d+ W|A \d+ W, B \d+ W)(, [\d.]+% lost to heat)?\.$/.test(await text("#score-status")) &&
          /Blade slip/.test(await text("#score-bars")) && (await page.$$eval("h2", (hs) => hs.map((h) => h.textContent))).some((h) => h.startsWith("Boat A vs Boat B")),
          `${heads} | ${await text("#score-status")}`);
    // Boat score for both boats; body settings show what they do.
    check(`${tag}: boat score for both boats`, /^Boat A\d+ \/ 100.*Boat B\d+ \/ 100/.test(await text("#boat-score")), await text("#boat-score"));
    await page.$eval("#ctl-body", (e) => e.closest("details").open = true);
    const score0 = await text("#boat-score .tile.a .big");
    await slide("#c-swingM", 0.1); await page.waitForTimeout(400);
    await scoreReady();
    check(`${tag}: race score: body swing costs A time`, /Body swing/.test(await text("#score-bars")) &&
          /Boat A: body swing costs \d+:\d\d\. Keep your body quieter\./.test(await text("#score-find")), await text("#score-find"));
    check(`${tag}: body swing: its cost, a lower score, the still-body curve`, /^Hull speed swings [\d.]+–[\d.]+ km\/h: costs [\d.]+% of your power$/.test(await text("#e-swingM")) &&
          (await text("#boat-score .tile.a .big")) !== score0 && /Body swing −/.test(await text("#boat-score .tile.a")) && (await page.$$("#hp-chart path.ln.still")).length === 1,
          await text("#e-swingM"));
    await page.click("#swing-timing >> text=Forward in the drive");
    check(`${tag}: body swing timing`, JSON.stringify(await pressed("#swing-timing")) === '["-1"]');
    await slide("#c-heelDeg", 6); await page.waitForTimeout(400);
    check(`${tag}: heel: fewer switches`, /^Switch every [\d.]+ strokes \(from [\d.]+\), [\d.]+% less time lost switching$/.test(await text("#e-heelDeg")), await text("#e-heelDeg"));
    await slide("#c-hipRock", 0.8); await slide("#c-dropM", 0.03); await page.waitForTimeout(400);
    check(`${tag}: hip rock and drop`, /^Heel ±[\d.]+° instead of ±[\d.]+°, hips moving up to \d+ mm$/.test(await text("#e-hipRock")) &&
          /^Loads the blade early: force peaks at \d+% of the drive \(from \d+%\)\. Bob ±[\d.]+ mm \(from ±[\d.]+\), costing [\d.]+%$/.test(await text("#e-dropM")) && /Hip rock 80%/.test(await text("#hull-roll-read")));
    for (const k of ["swingM", "heelDeg", "hipRock", "dropM"]) await slide(`#c-${k}`, 0);
    await page.click("#swing-timing >> text=Back in the drive");
    await page.waitForTimeout(300);
    check(`${tag}: body back to still`, (await text("#boat-score .tile.a .big")) === score0 && (await page.isHidden("#e-swingM")));
    // Paddler engine: rotation shows its reach, the engine box, and "what each can hold".
    await page.$eval("#ctl-paddler", (e) => e.closest("details").open = true);
    check(`${tag}: engine box: arms only by default`, /What Boat A can hold140 W for an hour/.test(await text("#engine-box")), await text("#engine-box"));
    await slide("#c-rotation", 0.8); await page.waitForTimeout(400);
    check(`${tag}: rotation: reach and muscle`, /^Reach \+12\.0°, hand travel \+9\.6 cm\. Uses more muscle: \+19 W you can hold$/.test(await text("#e-rotation")) &&
          /159 W/.test(await text("#engine-box")), await text("#e-rotation"));
    await slide("#c-hinge", 0.5); await page.waitForTimeout(400);
    check(`${tag}: hinge: reach, swing and muscle`, /^Reach \+4\.0°, hand travel \+7\.5 cm\. Your body swings ±3\.0 cm, costing [\d.]+%\. Hips and back: \+8 W you can hold$/.test(await text("#e-hinge")) &&
          (await page.$$("#hp-chart path.ln.still")).length >= 1, await text("#e-hinge"));
    check(`${tag}: pitch readout and the paddler`, /^A Pitches ±[\d.]+°, the bow moving ±[\d.]+ mm, costing [\d.]+%/.test(await text("#hull-pitch-read")) &&
          (await page.$$("#hp-pitch .fig-head")).length >= 1 && (await page.$$("svg.view .fig-head")).length >= 3, await text("#hull-pitch-read"));
    await goTab("stroke");
    await page.click("#view-mode >> text=Overlay"); await page.waitForTimeout(200);
    check(`${tag}: overlay: both paddlers, each in its colour`, await page.$$eval("svg.view", (ss) => ss.every((s) =>
          s.querySelectorAll(".fig-head").length === 2 && /series-1/.test(s.querySelector(".fig-head.fig-a").getAttribute("style")))) &&
          (await page.$$("#hp-pitch .fig-head")).length === 2 && (await page.$$("#hp-roll .fig-head")).length === 2 && (await page.$$("#hp-roll .paddler")).length === 0);
    await page.click("#view-mode >> text=Split");
    await slide("#c-hinge", 0);
    await goTab("race");
    await page.click("#effort-mode >> text=What each can hold"); await page.waitForTimeout(300);
    await scoreReady();
    check(`${tag}: effort: what each can hold`, (await page.isDisabled("#effA")) && (await page.isDisabled("#effB")) &&
          /^\d+ W · what A can hold$/.test(await text("#o-effA")) && /Muscle not used/.test(await text("#score-bars")) &&
          /^Boat A is faster by [\d:]+\. .+ costs Boat [AB] [\d:]+ more/.test(await text("#score-verdict")), `${await text("#o-effA")} | ${await text("#score-verdict")}`);
    const rowsGap = await page.evaluate(() => {
      const o = window.sim.score, sum = (x) => o[x].rows.reduce((a, q) => a + (q[0] === "Portages" ? 0 : q[1]), 0);
      return [sum("a") - sum("b"), o.a.t - o.b.t];
    });
    check(`${tag}: race score rows add up to the gap, at different efforts`, Math.abs(rowsGap[0] - rowsGap[1]) < 0.5,
          `${rowsGap[0].toFixed(1)} vs ${rowsGap[1].toFixed(1)} s`);
    await page.click("#effort-mode >> text=Set by hand"); await slide("#c-rotation", 0); await page.waitForTimeout(300);
    check(`${tag}: effort: back to set by hand`, !(await page.isDisabled("#effA")) && !/what A can hold/.test(await text("#o-effA")));
    // Switching sides in the views: n strokes right, a switch lasting switchLossS, n strokes left.
    const sw = await page.evaluate(() => {
      const r = window.sim.results.a, Tc = r.cycle, n = Math.max(1, Math.round(r.strokesPerSide)), Ts = r.params.switchLossS;
      const tl = window.sim.timeline;
      return { on: document.querySelector("#switch-anim").getAttribute("aria-pressed"), n,
        first: tl("a", 0.5 * Tc), last: tl("a", (n - 0.5) * Tc), mid: tl("a", n * Tc + Ts / 2),
        left: tl("a", n * Tc + Ts + 0.5 * Tc), back: tl("a", 2 * n * Tc + 2 * Ts + 0.5 * Tc),
        readout: document.querySelector(".pair[data-view=top] .switch-read").textContent };
    });
    check(`${tag}: switching sides on by default, timed by strokes per side and switch time`, sw.on === "true" &&
          sw.first.m === 1 && sw.first.stroke === 1 && sw.last.stroke === sw.n && sw.last.m === 1 &&
          sw.mid.stroke === 0 && Math.abs(sw.mid.m) < 0.1 && sw.mid.lift.over > 0.1 &&
          sw.left.m === -1 && sw.left.stroke === 1 && sw.back.m === 1 && /^(\d+\/\d+ [RL]|switch)/.test(sw.readout),
          `${sw.n} per side, now "${sw.readout}"`);
    // The recovery: the top hand travels to the catch and the shaft tilts
    // about it to carry the blade clear, without jumps where the drive ends.
    const rec = await page.evaluate(() => {
      const r = window.sim.results.a, df = r.params.driveFraction, st = (ph) => window.sim.stateAt(r, ph);
      const low = (s) => Math.min(s.B.tip[2], s.B.throat[2]), mids = [], grips = [];
      for (let u = 0.25; u <= 0.75; u += 0.05) { const s = st(df + u * (1 - df)); mids.push(low(s)); grips.push(s.P.at(0)[2]); }
      const a = st(df - 1e-4).P.at(0.6), b = st(df + 1e-4).P.at(0.6);
      return { clear: Math.min(...mids), grip: Math.max(...grips), step: Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) };
    });
    check(`${tag}: recovery carries the blade clear, top hand within reach, no jump at the exit`,
          rec.clear > 0.03 && rec.grip < 1.0 && rec.step < 0.02, `blade ${(rec.clear * 100).toFixed(0)} cm up, top hand ${rec.grip.toFixed(2)} m, step ${(rec.step * 1000).toFixed(0)} mm`);
    await goTab("stroke");
    await page.click("#switch-anim"); await settle();
    const off = await page.evaluate(() => [0.3, 5, 9.7, 31].every((t) => window.sim.timeline("a", t).m === 1) &&
      !document.querySelector(".switch-read"));
    check(`${tag}: switching sides off: always the right side`, off);
    await page.click("#switch-anim"); await settle();
    // Paddler profile: height scales the body, presets keep the profile, profiles save and load.
    await goTab("stroke");
    await page.click("#profile summary");
    const torso0 = await page.evaluate(() => window.sim.boats.a.torsoM);
    await slide("#c-heightM", 1.95); await settle();
    const torso1 = await page.evaluate(() => window.sim.boats.a.torsoM);
    check(`${tag}: profile: height scales the body`, Math.abs(torso1 / torso0 - 1.95 / 1.78) < 0.01 && /195 cm/.test(await text("#o-heightM")),
          `${torso0} → ${torso1}`);
    await page.click("#strength-mode >> text=By paddle angle"); await slide("#c-fitnessW", 260); await settle();
    check(`${tag}: profile: strength by position shows its curve`, (await visible("#strength-curve")) &&
          (await page.$$("#strength-curve path.cur")).length === 1 && /×[\d.]+ against the default stroke/.test(await text("#strength-read")));
    await page.click("#presets >> text=Long reach"); await settle();
    const kept = await page.evaluate(() => [window.sim.boats.a.heightM, window.sim.boats.a.fitnessW, window.sim.boats.a.strengthOn]);
    check(`${tag}: profile: presets keep it`, JSON.stringify(kept) === "[1.95,260,1]" && /×0\.\d\d/.test(await text("#strength-read")),
          `${kept} | ${await text("#strength-read")}`);
    page.once("dialog", (d) => d.accept("Tall"));
    await page.click("#profile-save"); await settle();
    await page.click("#profile-copy"); await settle();
    const copied = await page.evaluate(() => [window.sim.boats.b.heightM, window.sim.boats.b.fitnessW, window.sim.boats.b.catchDeg]);
    check(`${tag}: profile: copy to the other boat (profile only)`, copied[0] === 1.95 && copied[1] === 260 && copied[2] === 25, `${copied}`);
    await page.click("#profile-fit"); await slide("#c-heightM", 1.78); await slide("#c-fitnessW", 200);
    await page.click("#strength-mode >> text=Off"); await settle();
    await page.selectOption("#profile-pick", "Tall"); await settle();
    const loaded = await page.evaluate(() => [window.sim.boats.a.heightM, window.sim.boats.a.fitnessW, window.sim.boats.a.strengthOn]);
    check(`${tag}: profile: saved and loaded`, JSON.stringify(loaded) === "[1.95,260,1]" && (await page.$$("#profile-pick option")).length === 2, `${loaded}`);
    page.once("dialog", (d) => d.accept());
    await page.click("#profile-delete"); await settle();
    check(`${tag}: profile: deleted`, (await page.$$("#profile-pick option")).length === 1);
    // Age and sex: female reshapes the shoulders and forearms; lengths from height follow it.
    await slide("#c-heightM", 1.78); await page.click("#profile-fit"); await settle();
    const body = () => page.evaluate(() => ["shoulderM", "forearmM", "torsoM"].map((k) => window.sim.boats.a[k]));
    const male = await body();
    await page.click("#sex >> text=Female"); await settle();
    const fem = await body();
    check(`${tag}: profile: female, narrower shoulders and shorter forearms for the height`,
          JSON.stringify(male) === "[0.4,0.3,0.55]" && JSON.stringify(fem) === "[0.386,0.29,0.55]" &&
          (await page.$eval('#sex [data-value="1"]', (e) => e.getAttribute("aria-pressed"))) === "true", `${male} → ${fem}`);
    await slide("#c-heightM", 1.6); await page.click("#profile-fit"); await settle();
    check(`${tag}: profile: lengths from height for a woman`, JSON.stringify(await body()) === "[0.347,0.26,0.494]", `${await body()}`);
    // Typical fitness: 200 W × (kg / 79)^⅔ × 0.8 for a woman × 0.85 at 60 (0.6% a year from 35).
    await slide("#c-ageY", 60); await settle();
    const kg = await page.evaluate(() => window.sim.boats.a.massKg - 11);
    const want = Math.round((200 * Math.pow(kg / 79, 2 / 3) * 0.8 * 0.85) / 5) * 5;
    check(`${tag}: profile: typical fitness for age, sex and weight`,
          (await text("#fitness-typical")) === `Typical for a woman of 60, ${kg} kg: ${want} W`, await text("#fitness-typical"));
    await page.click("#fitness-typical"); await settle();
    check(`${tag}: profile: typical fitness sets it`, (await page.evaluate(() => window.sim.boats.a.fitnessW)) === want);
    // At 60, A loses 20% more to the heat: 30 °C, 70% is 4.5% for B (35), 5.4% for A.
    await slide("#temp", 30); await slide("#humid", 70); await page.waitForTimeout(200);
    check(`${tag}: heat: an older paddler loses more`, /^Heat index 33 °C: A 5\.4%, B 4\.5% less power than on a cool day\.$/.test(await text("#heat-note")),
          await text("#heat-note"));
    await slide("#temp", 10); await slide("#humid", 50);
    await page.click("#sex >> text=Male"); await slide("#c-ageY", 35); await slide("#c-heightM", 1.95); await slide("#c-fitnessW", 260); await settle();
    const back = await body();
    check(`${tag}: profile: back to male restores the lengths`, Math.abs(back[0] / (0.4 * 1.95 / 1.78) - 1) < 0.01 &&
          Math.abs(back[1] / (0.3 * 1.95 / 1.78) - 1) < 0.01, `${back}`);
    // A stroke the arms can't reach is flagged; strength from the arms shows its curve.
    await page.click("#boats [data-boat=a]");
    await slide("#c-handOutM", 0.85); await slide("#c-catchDeg", 60); await settle();
    check(`${tag}: reach warning for a stroke out of reach`, (await visible("#reach-warn")) && /Boat A's stroke needs \d+ cm more reach/.test(await text("#reach-warn")),
          await text("#reach-warn"));
    await page.click("#strength-mode >> text=From the arms"); await settle();
    check(`${tag}: strength from the arms: curve through the drive`, (await page.$$("#strength-curve path.cur")).length === 1 &&
          /Arms at up to \d+% \(lower\)/.test(await text("#strength-read")) && !(await visible("#c-strengthPeakDeg")));
    await page.click("#strength-mode >> text=Off");
    await page.click("#presets >> text=Defaults"); await settle();
    check(`${tag}: gearing: off by default, its settings hidden`, !(await visible("#c-armBestMs")));
    await page.click("#gear-mode >> text=On"); await slide("#c-rateSpm", 80); await settle();
    check(`${tag}: gearing: arm speed, what the arms hold and the best rate`, (await visible("#c-armBestMs")) &&
          /moves [\d.]+ m\/s against its shoulder.*×0\.\d\d.*best at \d+ spm/.test(await text("#gear-read")) && /Gearing: your arms hold ×0\.\d\d/.test(await text("#engine-box")),
          await text("#gear-read"));
    // The fit tab: a search for Boat A (gearing on, 80 spm), tried in Boat B.
    const bWas = await page.evaluate(() => JSON.stringify(window.sim.boats.b));
    await page.click("nav.tabs >> text=Fit");
    await page.click("#fit-go");
    await page.waitForFunction(() => !document.getElementById("fit-result").hidden, null, { timeout: 60000 });
    const fitShown = await page.evaluate(() => ({ rows: document.querySelectorAll("#fit-table tbody tr").length, why: document.querySelectorAll("#fit-why li").length,
      cells: document.querySelectorAll("#fit-map .cell").length, table: document.getElementById("fit-table").textContent }));
    check(`${tag}: fit: a suggestion with reasons and a map`, fitShown.rows >= 10 && fitShown.why >= 3 && fitShown.cells === 45 && /\+\d+\.\d%/.test(fitShown.table),
          `${fitShown.rows} rows, ${fitShown.why} reasons, ${fitShown.cells} squares`);
    const aPaddle = await page.evaluate(() => window.sim.boats.a.paddleLenM);
    await page.click("#fit-try"); await settle();
    const tried = await page.evaluate(() => ({ tab: window.sim.tab, b: window.sim.boats.b, a: window.sim.boats.a.paddleLenM }));
    check(`${tag}: fit: try it in Boat B`, tried.tab === "stroke" && tried.b.paddleLenM !== aPaddle && tried.b.gearOn === 1 && tried.a === aPaddle,
          `B paddle ${(tried.b.paddleLenM / 0.0254).toFixed(0)} in`);
    await page.evaluate((w) => Object.assign(window.sim.boats.b, JSON.parse(w)), bWas);
    await page.click("#boats [data-boat=b]"); await slide("#c-rateSpm", JSON.parse(bWas).rateSpm); await page.click("#boats [data-boat=a]");
    await page.click("#gear-mode >> text=Off"); await slide("#c-rateSpm", 60); await settle();
    check(`${tag}: fit: says when the boat has changed since the search`, /has changed since this search/.test(await text("#fit-status")));
    check(`${tag}: reach warning gone when in reach`, !(await visible("#reach-warn")));
    // Back to the defaults for what follows.
    await page.evaluate(() => { const S = window.StrokeSim; for (const x of ["a", "b"]) for (const k of ["heightM", "torsoM", "shoulderM", "upperArmM", "forearmM", "thighM", "shinM", "fitnessW", "strengthOn"]) window.sim.boats[x][k] = S.DEFAULTS[k]; });
    await page.click("#presets >> text=Clean vertical"); await settle();
    await page.click("#profile summary");
    const sh = await page.evaluate(() => [window.sim.shoulders([0, 0, 0], 1, 1), window.sim.shoulders([0, 0, 0], 1, -1)]);
    check(`${tag}: trunk rotation: paddle-side shoulder forward at the catch, back at the exit`,
          sh[0].pad[0] > sh[0].off[0] && sh[1].pad[0] < sh[1].off[0] && sh[0].pad[1] < 0);
    // The C1 shape in the stroke views: a gunwale line inside the hull from above.
    check(`${tag}: stroke views draw the C1 (hull and gunwale from above)`, (await page.$$('.pair[data-view="top"] svg path.hull-in')).length >= 1);
    await slide("#depth", 0.9);
    check(`${tag}: hull: shallow race water shows on the heat map`, /0\.9 m deep: \+\d+%/.test(await text("#hull-shallow")) && (await page.$$("#hull-shallow circle")).length === 2);
    await slide("#depth", 5);
    check(`${tag}: no page errors`, errors.length === 0, errors.join("; "));
    // A stale, cached engine: the page asks for a reload instead of breaking.
    if (scheme === "light") {
      const old = await ctx.newPage();
      await old.route("**/stroke-sim.js*", (r) => r.fulfill({ contentType: "text/javascript",
        body: "window.StrokeSim = { simulate() {}, DEFAULTS: {} };" }));
      await old.goto("file://" + path.resolve(__dirname, "../../docs/app/simulator.html"));
      await old.waitForTimeout(300);
      check(`${tag}: stale engine shows a reload prompt`, /kept an old copy/.test(await old.textContent("main")) &&
            (await old.$$("main > .card button.primary")).length === 1);
      await old.close();
      // Each boat's settings are remembered, and can be reset.
      {
        const url = "file://" + path.resolve(__dirname, "../../docs/app/simulator.html");
        const mem = await ctx.newPage();
        await mem.goto(url); await mem.evaluate(() => localStorage.removeItem("ppm.sim.boats")); await mem.reload(); await mem.waitForTimeout(500);
        const val = (sel) => mem.$eval(sel, (e) => e.value);
        await mem.$eval("#c-catchDeg", (el) => { el.value = 50; el.dispatchEvent(new Event("input")); });
        await mem.click("#boats [data-boat=b]");
        await mem.$eval("#c-rateSpm", (el) => { el.value = 72; el.dispatchEvent(new Event("input")); });
        await mem.waitForTimeout(300); await mem.reload(); await mem.waitForTimeout(500);
        const kept = [await val("#c-catchDeg"), await mem.evaluate(() => window.sim.boats.b.rateSpm)];
        check(`${tag}: each boat's settings are remembered`, kept[0] === "50" && kept[1] === 72, `${kept}`);
        mem.once("dialog", (d) => d.accept());
        await mem.click("#reset"); await mem.waitForTimeout(300);
        check(`${tag}: reset the boat being edited`, (await val("#c-catchDeg")) === "35" && (await mem.textContent("#reset")) === "Reset Boat A" &&
              (await mem.evaluate(() => window.sim.boats.b.rateSpm)) === 72);
        mem.once("dialog", (d) => d.accept());
        await mem.click("#reset-both"); await mem.waitForTimeout(300);
        check(`${tag}: reset both: back to the starting boats`, (await mem.evaluate(() => window.sim.boats.b.rateSpm)) === 60 &&
              (await mem.textContent("#nm-a")) === "Defaults" && (await mem.textContent("#nm-b")) === "Wide sweep");
        await mem.evaluate(() => localStorage.setItem("ppm.sim.boats", '{"a":{"catchDeg":"x","rateSpm":66},"b":7}'));
        await mem.reload(); await mem.waitForTimeout(500);
        check(`${tag}: a damaged save keeps what it can`, (await val("#c-catchDeg")) === "35" && (await mem.evaluate(() => window.sim.boats.a.rateSpm)) === 66 &&
              (await mem.textContent("#nm-b")) === "Wide sweep");
        // A save from before the switch time's default went to 1 s: its 0.3 s
        // (the old default) follows the new one; a 0.3 s chosen since is kept.
        await mem.evaluate(() => localStorage.setItem("ppm.sim.boats", '{"a":{"switchLossS":0.3,"rateSpm":66},"b":{"switchLossS":0.5}}'));
        await mem.reload(); await mem.waitForTimeout(500);
        const old = await mem.evaluate(() => [window.sim.boats.a.switchLossS, window.sim.boats.a.rateSpm, window.sim.boats.b.switchLossS]);
        await mem.evaluate(() => localStorage.setItem("ppm.sim.boats", '{"version":2,"a":{"switchLossS":0.3},"b":{}}'));
        await mem.reload(); await mem.waitForTimeout(500);
        const chosen = await mem.evaluate(() => window.sim.boats.a.switchLossS);
        check(`${tag}: an old save takes the new switch time default; one chosen since stays`,
              old[0] === 1 && old[1] === 66 && old[2] === 0.5 && chosen === 0.3, `${old} / ${chosen}`);
        await mem.evaluate(() => localStorage.removeItem("ppm.sim.boats"));
        await mem.close();
      }
      // A phone: taps act once each, a paused race stops redrawing, and the
      // followed map widens so it doesn't lurch kilometres a frame.
      const { devices } = require("playwright");
      const phone = await browser.newContext({ ...devices["iPhone 13"] });
      const mob = await phone.newPage();
      await mob.goto("file://" + path.resolve(__dirname, "../../docs/app/simulator.html"));
      await mob.waitForTimeout(500);
      await mob.tap("nav.tabs button[data-tab=race]"); await mob.waitForTimeout(300);
      await mob.tap("#water >> text=AuSable / GPX"); await mob.waitForTimeout(800);
      await mob.$eval("#race-start", (e) => e.scrollIntoView({ block: "center" }));
      const barKm = async () => { const m = (await mob.textContent("#course-view")).match(/([\d.]+) (k?m)N/); return m ? +m[1] * (m[2] === "km" ? 1 : 0.001) : 0; };
      const still = await barKm();
      await mob.tap("#race-start"); await mob.waitForTimeout(700);
      const moving = await barKm();
      // Zooming in during the race works, and slows the replay to match; Fit goes back to full speed.
      for (let i = 0; i < 4; i++) await mob.tap("#zoom-in");
      await mob.waitForTimeout(200);
      const zoomed = await barKm(), slowMsg = await mob.textContent("#race-status");
      await mob.tap("#zoom-fit"); await mob.waitForTimeout(200);
      check(`${tag}: phone: zoom works mid-race and slows the replay`, zoomed < moving / 8 && /^Zoomed in, so slowed: 1 s here is /.test(slowMsg) &&
            (await barKm()) === moving && (await mob.textContent("#race-status")) === "", `${moving} → ${zoomed} km | ${slowMsg}`);
      await mob.tap("#race-pause"); await mob.waitForTimeout(150);
      const pausedSvg = await mob.$eval("#course-view", (e) => e.innerHTML);
      await mob.waitForTimeout(300);
      check(`${tag}: phone: a tap pauses (once), and a paused race holds still`, (await mob.textContent("#race-pause")) === "Resume" &&
            /^Paused at /.test(await mob.textContent("#race-status")) && pausedSvg === (await mob.$eval("#course-view", (e) => e.innerHTML)));
      check(`${tag}: phone: the map widens while the race runs`, moving > still * 4, `${still} km → ${moving} km`);
      await mob.tap("#race-pause"); await mob.waitForTimeout(150);
      check(`${tag}: phone: resume and stop`, (await mob.textContent("#race-pause")) === "Pause" &&
            (await mob.tap("#race-stop"), await mob.waitForTimeout(150), (await mob.textContent("#race-status")) === "Stopped."));
      await phone.close();
      // Racing lines: open water weaves (hit and switch), the AuSable shows each
      // boat's line across the channel, and the river view holds one scale.
      const lp = await ctx.newPage();
      await lp.goto("file://" + path.resolve(__dirname, "../../docs/app/simulator.html"));
      await lp.waitForTimeout(500);
      await lp.click("nav.tabs button[data-tab=race]"); await lp.waitForTimeout(300);
      await lp.$eval("#scrub", (el) => { el.value = 500; el.dispatchEvent(new Event("input")); });
      await lp.click("#zoom-out"); await lp.waitForTimeout(100);
      check(`${tag}: open water: each boat's weave`, (await lp.$$("#race-view .line-a")).length === 1 && (await lp.$$("#race-view .line-b")).length === 1);
      check(`${tag}: open water: the weave in words, each boat's sideways swing`,
            /^Sideways swing, drawn ×\d+: A ±\d+ cm, switching every [\d.]+ strokes · B ±\d+ cm, every [\d.]+ strokes\. Switching by heading/.test(await lp.textContent("#race-weave")),
            await lp.textContent("#race-weave"));
      // Live with the replay: each boat's heading over its last strokes, and its place on the shallow-water map.
      const live = await lp.evaluate(() => ({ heading: document.querySelectorAll("#race-heading path").length, dots: document.querySelectorAll("#race-shallow-now circle").length,
        hRead: document.getElementById("race-heading-read").textContent, sRead: document.getElementById("race-shallow-read").textContent,
        two: document.getElementById("conditions-cols").classList.contains("two") }));
      check(`${tag}: race: live heading and shallow water; no empty lines column on open water`, live.heading === 2 && live.dots === 2 &&
            /A [\d.]+° off straight/.test(live.hRead) && /A [\d.]+ km\/h through the water, deep water/.test(live.sRead) && live.two,
            `${live.hRead} | ${live.sRead.split("Colour")[0]}`);
      await lp.click("#water >> text=River"); await lp.waitForTimeout(300);
      check(`${tag}: river: the lines column is back`, !(await lp.$eval("#conditions-cols", (e) => e.classList.contains("two"))) && (await lp.isVisible("#line-a")));
      await lp.click("#race-start"); await lp.waitForTimeout(300);
      const spacings = await lp.evaluate(() => new Promise((res) => {
        const seen = new Set(); let n = 0;
        const f = () => { const xs = [...document.querySelectorAll("#river-map text")].filter((t) => / k?m$/.test(t.textContent)).map((t) => +t.getAttribute("x"));
          if (xs.length > 1) seen.add(Math.round((xs[1] - xs[0]) * 10)); if (++n < 60) requestAnimationFrame(f); else res(seen.size); };
        requestAnimationFrame(f);
      }));
      await lp.click("#race-stop");
      check(`${tag}: river view keeps one scale through a run (no flicker)`, spacings === 1, `${spacings} scales`);
      await lp.click("#water >> text=AuSable / GPX"); await lp.waitForTimeout(800);
      await lp.click("#line-b >> text=Edges"); await lp.waitForTimeout(300);
      await lp.$eval("#scrub", (el) => { el.value = 8; el.dispatchEvent(new Event("input")); });
      for (let i = 0; i < 5; i++) await lp.click("#zoom-in");
      const ld = await lp.$$eval("#course-view path.line-a, #course-view path.line-b", (ps) => ps.map((q) => q.getAttribute("d")));
      await lp.click("#race-start"); await lp.waitForTimeout(300);
      const wideHint = /Close-up shows each boat's line/.test(await lp.textContent("#course-view"));
      await lp.click("#zoom-close"); await lp.waitForTimeout(200);
      check(`${tag}: AuSable mid-race: Close-up shows the lines in one tap`, wideHint && (await lp.$$("#course-view path.line-a")).length === 1 &&
            /^Zoomed in, so slowed/.test(await lp.textContent("#race-status")));
      await lp.click("#race-stop");
      await lp.$eval("#scrub", (el) => { el.value = 8; el.dispatchEvent(new Event("input")); });
      for (let i = 0; i < 5; i++) await lp.click("#zoom-in");
      const ld2 = await lp.$$eval("#course-view path.line-a, #course-view path.line-b", (ps) => ps.map((q) => q.getAttribute("d")));
      check(`${tag}: Close-up only on a real course`, ld2.length === 2 && (await lp.click("#water >> text=Open water"), await lp.isHidden("#zoom-close")));
      check(`${tag}: AuSable: each boat's line across the channel`, ld.length === 2 && ld[0] !== ld[1] && /A solid · B dashed/.test(await lp.textContent("#course-view")));
      await lp.close();
    }
    await ctx.close();
  }
  await browser.close();
  console.log(`\n${failures ? "FAILED" : "PASSED"} (${failures} failure${failures === 1 ? "" : "s"})`);
  process.exit(failures ? 1 : 0);
})().catch((e) => { console.error(e); process.exit(1); });
