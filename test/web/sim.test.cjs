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
  // Burke (The Science of Paddling, Part 32) cites about ±10% speed swing.
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
  check("hull calibrated near Burke's C-1 (k 4.26 at 109 kg)", base.speedAt(150) > 2.5 && base.speedAt(150) < 2.85,
        `${base.speedAt(150).toFixed(2)} m/s at 150 W`);
  check("plausible C1 speed at 150 W", base.speedAt(150) > 2.0 && base.speedAt(150) < 3.2, `${base.speedAt(150).toFixed(2)} m/s`);
  check("blade stays buried during the drive", base.samples.every((s) => s.B.wet > 0.99));

  const fast = S.simulate({ rateSpm: 85 });
  check("efficiency nearly independent of stroke rate", Math.abs(fast.efficiency - base.efficiency) < 2e-3,
        `${pct(base.efficiency)} vs ${pct(fast.efficiency)}`);
  check("speed at equal effort independent of rate", Math.abs(fast.speedAt(150) - base.speedAt(150)) < 0.01);

  const clean = S.simulate({ leanDeg: 2, handOutM: 0.38 });
  const lean = S.simulate({ leanDeg: 30, handOutM: 0.55 });
  const sweep = S.simulate({ leanDeg: 40, sweepDeg: 70, handOutM: 0.5, catchDeg: 25, exitDeg: 25 });
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
  // with it (Burke, Part 7): T = (2L/V) / (1 − U²/V²).
  const ob0 = S.race(clean, clean, 2000, 150, 150, { course: "outback" });
  const ob = S.race(clean, clean, 2000, 150, 150, { course: "outback", current: 0.5 });
  const V0 = clean.speedAt(150), burke = 1 / (1 - 0.25 / (V0 * V0));
  check("there and back in still water = point to point", Math.abs(ob0.tA - S.race(clean, clean, 2000, 150, 150).tA) < 1e-6);
  check("there and back with current matches Burke's formula", Math.abs(ob.tA / ob0.tA - burke) < 0.005,
        `${(ob.tA / ob0.tA).toFixed(4)} vs ${burke.toFixed(4)}`);
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
}

(async () => {
  console.log("\nStroke simulator page");
  const browser = await chromium.launch();
  for (const scheme of ["light", "dark"]) {
    const ctx = await browser.newContext({ viewport: { width: 390, height: 844 }, colorScheme: scheme, deviceScaleFactor: 2 });
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
    const eff = () => page.textContent("#r-eff");
    const num = async (id) => parseFloat((await page.textContent(id)).replace("−", "-"));
    check(`${scheme}: shows efficiency`, /^\d+\.\d%$/.test(await eff()), await eff());
    check(`${scheme}: all three views drawn (with lagging frame timestamps)`,
          (await page.$$("svg.view line.blade")).length === 3 && (await page.$$("svg.view path.path")).length === 3);

    await page.click("#presets >> text=Clean vertical");
    await page.waitForTimeout(150);
    const cleanEff = await num("#r-eff");
    await page.click("#save");
    await page.click("#presets >> text=Wide sweep");
    await page.waitForTimeout(150);
    check(`${scheme}: sweep scores lower than clean vertical`, (await num("#r-eff")) < cleanEff);
    check(`${scheme}: comparison shows the difference`, (await page.textContent("#d-eff")).includes("worse"),
          await page.textContent("#d-eff"));
    check(`${scheme}: power split has four parts`, (await page.$$("#bar-power span")).length === 4);
    check(`${scheme}: switch-sides tile`, /\d/.test(await page.textContent("#r-side")), await page.textContent("#r-side"));
    check(`${scheme}: no J-stroke wording`, !/J-stroke|steering/i.test(await page.textContent("body")));

    check(`${scheme}: vertical note`, /lifts|pulls down|cancels/.test(await page.textContent("#vert-note")),
          await page.textContent("#vert-note"));

    // Presets highlight while the sliders match them.
    const pressed = (sel) => page.$$eval(`${sel} button[aria-pressed="true"]`, (bs) => bs.map((b) => b.dataset.value));
    check(`${scheme}: selected preset highlighted`, JSON.stringify(await pressed("#presets")) === '["Wide sweep"]');
    await page.$eval("#c-catchDeg", (el) => { el.value = 26; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: no preset highlighted after a slider change`, (await pressed("#presets")).length === 0);

    // Race: each boat picks a preset, the live sliders or its own save.
    await page.click("#presets >> text=Clean vertical");
    await page.waitForTimeout(150);
    check(`${scheme}: race defaults (A sliders, B wide sweep)`,
          JSON.stringify([await pressed("#chips-a"), await pressed("#chips-b")]) === '[["live"],["Wide sweep"]]');
    const verdict = () => page.textContent("#verdict");
    check(`${scheme}: race verdict A wins`, /^Boat A wins by (\d+(\.\d)? s|\d+:\d\d), \d+ m ahead/.test(await verdict()), await verdict());
    check(`${scheme}: finishing times shown`, /\d+:\d\d/.test(await page.textContent("#t-a")) && /\d+:\d\d/.test(await page.textContent("#t-b")),
          `${await page.textContent("#t-a")} / ${await page.textContent("#t-b")}`);
    await page.click("#chips-b >> text=Clean vertical");
    await page.waitForTimeout(50);
    check(`${scheme}: B preset chip selects and highlights`, JSON.stringify(await pressed("#chips-b")) === '["Clean vertical"]');
    check(`${scheme}: same stroke is a dead heat`, (await verdict()) === "Dead heat.");
    await page.click("#chips-a >> text=Wide sweep");
    await page.waitForTimeout(50);
    check(`${scheme}: A preset chip`, /^Boat B wins/.test(await verdict()), await verdict());

    // Save sliders for each boat, then move the sliders: saves stay put.
    await page.click("#save-a");
    check(`${scheme}: save A selects it`, JSON.stringify(await pressed("#chips-a")) === '["saved"]' &&
          (await page.textContent("#saved-a")).includes("Clean vertical"), await page.textContent("#saved-a"));
    await page.click("#presets >> text=Long reach");
    await page.waitForTimeout(150);
    await page.click("#save-b");
    check(`${scheme}: save B selects it`, JSON.stringify(await pressed("#chips-b")) === '["saved"]');
    await page.click("#presets >> text=Wide sweep");
    await page.waitForTimeout(150);
    check(`${scheme}: saved clean beats saved long reach`, /^Boat A wins/.test(await verdict()), await verdict());

    await page.$eval("#c-effB", (el) => { el.value = 250; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: more effort for B flips the race`, /^Boat B wins/.test(await verdict()), await verdict());
    await page.$eval("#c-effB", (el) => { el.value = 150; el.dispatchEvent(new Event("input")); });

    // Conditions
    const timeA = () => page.textContent("#t-a");
    const secs = (t) => t.split(":").reduce((a, x) => a * 60 + +x, 0);
    const calm = await timeA();
    await page.$eval("#c-wind", (el) => { el.value = 20; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: wind readout`, (await page.textContent("#o-wind")) === "20 km/h headwind");
    check(`${scheme}: headwind slows the race`, secs(await timeA()) > secs(calm), `${calm} → ${await timeA()}`);
    await page.$eval("#c-wind", (el) => { el.value = 0; el.dispatchEvent(new Event("input")); });
    await page.$eval("#c-current", (el) => { el.value = -10; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: current readout`, (await page.textContent("#o-current")) === "10.0 km/h against you");
    for (const id of ["#c-effA", "#c-effB"]) await page.$eval(id, (el) => { el.value = 50; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: current too strong`, (await verdict()).startsWith("Neither boat") && (await timeA()) === "never",
          await verdict());
    await page.$eval("#c-current", (el) => { el.value = 0; el.dispatchEvent(new Event("input")); });
    for (const id of ["#c-effA", "#c-effB"]) await page.$eval(id, (el) => { el.value = 150; el.dispatchEvent(new Event("input")); });
    await page.click("#chips-a >> text=Current sliders");
    await page.click("#chips-b >> text=Wide sweep");

    // Course choice
    check(`${scheme}: point to point by default`, JSON.stringify(await pressed("#chips-course")) === '["point"]');
    const t1 = await timeA();
    await page.$eval("#c-current", (el) => { el.value = 4; el.dispatchEvent(new Event("input")); });
    const tWith = await timeA();
    await page.click("#chips-course >> text=There and back");
    check(`${scheme}: there and back selected`, JSON.stringify(await pressed("#chips-course")) === '["outback"]' &&
          (await page.textContent("#o-current")).endsWith("going out"), await page.textContent("#o-current"));
    check(`${scheme}: current helps one way, costs there and back`, secs(tWith) < secs(t1) && secs(await timeA()) > secs(t1),
          `${t1} still, ${tWith} with current, ${await timeA()} there and back`);
    await page.$eval("#c-current", (el) => { el.value = 0; el.dispatchEvent(new Event("input")); });
    await page.click("#chips-course >> text=Point to point");

    await page.click("#race-start");
    await page.waitForTimeout(600);
    check(`${scheme}: race animates two boats`, (await page.$$("#race-view .boat-a, #race-view .boat-b")).length >= 2);
    await page.click("#presets >> text=Short reach, pull behind");
    await page.waitForTimeout(150);
    check(`${scheme}: short-reach preset`, (await page.$eval("#c-catchDeg", (e) => e.value)) === "12");
    await page.$eval("#c-massKg", (el) => { el.value = 120; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: weight slider keeps the preset highlighted`, JSON.stringify(await pressed("#presets")) === '["Short reach, pull behind"]');

    // Slider moves the model: more lean, more turning.
    await page.click("#presets >> text=Defaults");
    await page.waitForTimeout(150);
    const yaw = async () => parseFloat((await page.textContent("#r-yaw-sub")).replace(/[^\d.−-]/g, "").replace("−", "-"));
    const yaw0 = await yaw();
    await page.$eval("#c-leanDeg", (el) => { el.value = 35; el.dispatchEvent(new Event("input")); });
    await page.waitForTimeout(150);
    check(`${scheme}: lean slider increases turning`, (await yaw()) > yaw0);
    check(`${scheme}: slider readout`, (await page.textContent("#o-leanDeg")) === "35°");
    check(`${scheme}: drive time readout`, (await page.textContent("#o-driveFraction")) === "50% of each stroke");
    await page.$eval("#c-bendDeg", (el) => { el.value = -12; el.dispatchEvent(new Event("input")); });
    check(`${scheme}: bend readout shows direction`, (await page.textContent("#o-bendDeg")) === "12° toward power face");

    if (process.argv[2]) {
      await page.click("#presets >> text=Defaults");
      await page.waitForTimeout(400);
      await page.screenshot({ path: path.join(process.argv[2], `sim-${scheme}.png`), fullPage: true });
    }
    check(`${scheme}: no page errors`, errors.length === 0, errors.join("; "));
    await ctx.close();
  }
  await browser.close();
  console.log(`\n${failures ? "FAILED" : "PASSED"} (${failures} failure${failures === 1 ? "" : "s"})`);
  process.exit(failures ? 1 : 0);
})().catch((e) => { console.error(e); process.exit(1); });
