// Tests for the stroke simulator: physics (docs/app/stroke-sim.js) in Node,
// then the page (docs/app/simulator.html) in headless Chromium.
//
//   node sim.test.cjs [screenshot dir]

const path = require("path");
const S = require("../../docs/app/stroke-sim.js");
const { chromium } = require("playwright");

let failures = 0;
function check(name, ok, detail = "") {
  console.log(`  [${ok ? " ok " : "FAIL"}] ${name}${detail ? "  " + detail : ""}`);
  if (!ok) failures++;
}
const pct = (x) => `${(x * 100).toFixed(1)}%`;

// The page loads the engine as stroke-sim.js?v=<hash of the file>, so a
// changed engine is never served stale from a browser cache.
{
  const fs = require("fs"), crypto = require("crypto");
  const hash = crypto.createHash("sha1").update(fs.readFileSync(path.resolve(__dirname, "../../docs/app/stroke-sim.js"))).digest("hex").slice(0, 10);
  const html = fs.readFileSync(path.resolve(__dirname, "../../docs/app/simulator.html"), "utf8");
  console.log("\nEngine cache tag");
  const tagged = html.includes(`stroke-sim.js?v=${hash}`);
  check(`simulator.html loads stroke-sim.js?v=${hash}`, tagged, tagged ? "" : "update the ?v= in simulator.html");
}
console.log("\nStroke simulator physics");
{
  const base = S.simulate({});
  const balance = base.hullPower + base.leewayPower + base.slipPower - base.paddlerPower;
  check("power balance: hull + leeway + slip = paddler", Math.abs(balance) < 1e-3 * base.paddlerPower,
        `residual ${balance.toExponential(1)} W`);
  check("thrust·speed = hull + leeway",
        Math.abs(base.usefulPower - base.hullPower - base.leewayPower) < 1e-3 * base.paddlerPower);
  const sh = base.at(150).shares;
  const total = sh.hull + sh.slip + sh.leeway + sh.surge + sh.sink + sh.bounce;
  check("shares at 150 W add to 1", Math.abs(total - 1) < 1e-4, total.toFixed(6));
  check("efficiency is the hull share at the comparison effort", Math.abs(base.efficiency - sh.hull) < 1e-12);
  check("surge costs a little", sh.surge > 0.0005 && sh.surge < 0.02, pct(sh.surge));
  // The Science of Paddling (Part 32) cites about ±10% speed swing.
  const swing = base.at(150).surgeSwing;
  check("speed swings 5–20% over a stroke at 60 spm", swing > 0.05 && swing < 0.2, pct(swing));
  const slow = S.simulate({ rateSpm: 30 }), quick = S.simulate({ rateSpm: 90 });
  check("same effort, slower rate: more surge", slow.at(150).shares.surge > 3 * quick.at(150).shares.surge,
        `${pct(slow.at(150).shares.surge)} at 30 spm vs ${pct(quick.at(150).shares.surge)} at 90`);
  check("same effort, slower rate: slower boat", slow.speedAt(150) < quick.speedAt(150));
  const longDrive = S.simulate({ driveFraction: 0.65 }), shortDrive = S.simulate({ driveFraction: 0.35 });
  check("longer drive: less slip, better efficiency", longDrive.efficiency > base.efficiency + 0.01 &&
        base.efficiency > shortDrive.efficiency + 0.01,
        `${pct(shortDrive.efficiency)} / ${pct(base.efficiency)} / ${pct(longDrive.efficiency)}`);
  const peakAt = (r) => r.peakForce * Math.pow(150 / r.paddlerPower, 2 / 3);
  check("longer drive: lower peak force at the same effort", peakAt(longDrive) < peakAt(base) && peakAt(base) < peakAt(shortDrive));
  check("hull calibrated near a measured C-1 (k 4.26 at 109 kg)", base.speedAt(150) > 2.5 && base.speedAt(150) < 2.85,
        `${base.speedAt(150).toFixed(2)} m/s at 150 W`);
  check("plausible C1 speed at 150 W", base.speedAt(150) > 2.0 && base.speedAt(150) < 3.2, `${base.speedAt(150).toFixed(2)} m/s`);
  check("blade stays buried during the drive", base.samples.every((s) => s.B.wet > 0.99));

  const fast = S.simulate({ rateSpm: 85 });
  check("efficiency nearly independent of stroke rate", Math.abs(fast.efficiency - base.efficiency) < 2e-3,
        `${pct(base.efficiency)} vs ${pct(fast.efficiency)}`);
  check("speed at equal effort independent of rate", Math.abs(fast.speedAt(150) - base.speedAt(150)) < 0.01);

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
  const slowSwitch = S.simulate({ switchLossStrokes: 1 });
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
  check("fixed count: more strokes, more wander", fixedN(12).headingWanderDeg > fixedN(6).headingWanderDeg * 1.9,
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
}

(async () => {
  console.log("\nStroke simulator page");
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

    // Layout: phones put the sliders after the views, wide screens beside them.
    const box = async (sel) => (await page.$(sel)).boundingBox();
    const [views, ctl, raceBox] = [await box("#views-card"), await box("#controls"), await box("#race-card")];
    if (width < 900) check(`${tag}: sliders between views and race`, ctl.y > views.y && ctl.y < raceBox.y);
    else check(`${tag}: sliders in a column beside the results`, ctl.x > views.x + views.width - 1);

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
    check(`${tag}: peak force has no winner, difference adds up`, peakRow.wins === 0 &&
          peakRow.d === `A pulls ${(await cellVal("peak", 2)) - (await cellVal("peak", 3))} N harder`, peakRow.d);
    check(`${tag}: winner highlighted`, (await page.$$('#cmp tr[data-row="eff"] td.win.a')).length === 1);
    check(`${tag}: two power bars, four parts`, (await page.$$("#bars-power .bar")).length === 2 &&
          (await page.$$("#leg-power tr")).length === 5);
    check(`${tag}: details for both boats`, /switch|turning/i.test(await text("#why-a")) && /B:/.test(await text("#why-b")));
    const s150 = await cellVal("speed", 2);
    await slide("#c-effortW", 300); await settle();
    check(`${tag}: compare-at effort raises speed`, (await cellVal("speed", 2)) > s150 && (await text("#o-effortW")) === "300 W");
    await slide("#c-effortW", 150); await settle();

    // Help tips: tap (or click) shows, again or elsewhere hides; mouse hover shows.
    const tipShown = () => page.$eval("#tip", (e) => !e.hidden);
    const effInfo = '#cmp tr[data-row="eff"] button.info';
    check(`${tag}: help buttons on table rows and breakdowns`, (await page.$$("#cmp button.info")).length === 7 &&
          (await page.$$("#leg-power button.info")).length === 4);
    await page.click(effInfo);
    check(`${tag}: tap shows the tip`, (await tipShown()) && /actually moves the boat/.test(await text("#tip")) &&
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
          (await page.$$eval("svg.view", (ss) => ss.map((s) => s.querySelector("g.blend-b") === s.querySelectorAll("g")[1]))).every(Boolean));
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
    const verdict = () => text("#verdict");
    check(`${tag}: race verdict`, /^Boat [AB] wins by (\d+(\.\d)? s|\d+:\d\d), \d+ m ahead/.test(await verdict()), await verdict());
    // Force per stroke at race effort: folded away, summary shows the peaks.
    const sumTxt = () => text("#race-curve-sum");
    check(`${tag}: race force curve open by default`, (await page.$eval("#race-curve-d", (d) => d.open)) &&
          (await sumTxt()) === "Hide force per stroke" && (await page.$$("#race-curve path.ln")).length === 2, await sumTxt());
    await page.click("#race-curve-sum"); await settle();
    check(`${tag}: race force curve folds away`, (await sumTxt()) === "Show force per stroke");
    await page.click("#race-curve-sum"); await settle();
    await page.click("#race-curve-live");
    await page.waitForFunction(() => document.querySelectorAll("#race-curve path.trace").length >= 1, null, { timeout: 5000 });
    check(`${tag}: race curve live trace`, /^A (\d+ N|recovery) · B (\d+ N|recovery)$/.test(await text("#race-curve .live-read")));
    await page.click("#race-curve-live");
    const cap = () => text("#race-curve-cap");
    check(`${tag}: race curve caption: each boat's effort and peak`, /^A at 150 W: peak \d+ NB at 150 W: peak \d+ N$/.test(await cap()), await cap());
    const before = await cap();
    await slide("#effB", 250);
    check(`${tag}: race curve follows each boat's effort`, (await cap()) !== before && (await cap()).startsWith(before.split("B at")[0]) &&
          (await cap()).includes("B at 250 W"), `${before} → ${await cap()}`);
    await slide("#effB", 150);
    await page.click("#race-curve-sum");
    check(`${tag}: point to point by default`, JSON.stringify(await pressed("#course")) === '["point"]');
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
    check(`${tag}: river map draws both lines`, (await page.$$("#river-map .line-a, #river-map .line-b")).length === 2);
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
          '["Time","Distance","Speed now","Avg speed","Power","Stroke rate","Strokes","Distance per stroke","Switches","Energy","Gap"]',
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
    check(`${tag}: speed chart: two boats with markers`, (await page.$$("#race-speed path.ln")).length === 2 &&
          (await page.$$("#race-speed circle.dot")).length === 2);
    // River with bends: the chart shades bends and the replay says where each boat is.
    await page.click("#water >> text=River");
    await slide("#current", -3); await slide("#depth", 2.5);
    check(`${tag}: bends shaded`, (await page.$$("#race-speed rect.bendband")).length > 5);
    check(`${tag}: river view replaces the lanes, with both boats on it`, (await page.$eval("#race-view", (e) => e.hasAttribute("hidden"))) &&
          !(await page.$eval("#river-map", (e) => e.hasAttribute("hidden"))) && (await page.$$("#river-map circle.boatdot")).length === 2);
    const wheres = new Set();
    for (let v = 0; v <= 1000; v += 7) {
      await page.$eval("#scrub", (el, v) => { el.value = v; el.dispatchEvent(new Event("input")); }, v);
      wheres.add(await text("#scrub-where"));
    }
    const all = [...wheres].join(" | ");
    check(`${tag}: replay passes straights, crossings and bends`, /along the bank/.test(all) && /crossing the river/.test(all) &&
          /inside of a bend/.test(all), all.slice(0, 160));
    await slide("#current", 0); await slide("#depth", 5);
    await page.click("#water >> text=Open water");

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
    await page.evaluate(() => { window.sim.raceWallS = 12; });

    await page.click("#race-start");
    await page.waitForTimeout(600);
    check(`${tag}: race animates two boats`, (await page.$$("#race-view .boat-a, #race-view .boat-b")).length === 2);
    check(`${tag}: no J-stroke wording`, !/J-stroke|steering stroke/i.test(await text("body")));

    if (process.argv[2]) {
      await page.waitForTimeout(400);
      await page.screenshot({ path: path.join(process.argv[2], `sim-${scheme}.png`), fullPage: true });
    }
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
    }
    await ctx.close();
  }
  await browser.close();
  console.log(`\n${failures ? "FAILED" : "PASSED"} (${failures} failure${failures === 1 ? "" : "s"})`);
  process.exit(failures ? 1 : 0);
})().catch((e) => { console.error(e); process.exit(1); });
