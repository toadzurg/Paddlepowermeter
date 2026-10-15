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
  check("thrust·speed = hull + leeway", Math.abs(base.usefulPower - base.hullPower - base.leewayPower) < 1e-3 * base.paddlerPower);
  check("plausible C1 speed at 150 W", base.speedAt(150) > 2.0 && base.speedAt(150) < 3.2, `${base.speedAt(150).toFixed(2)} m/s`);
  check("blade stays buried during the drive", base.samples.every((s) => s.B.wet > 0.99));

  const fast = S.simulate({ rateSpm: 85 });
  check("efficiency independent of stroke rate", Math.abs(fast.efficiency - base.efficiency) < 1e-3,
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

  const r = S.race(clean, sweep, 10000, 150, 150);
  check("race: clean beats sweep over 10 km", r.timeGap > 10 && r.distanceGap > 20,
        `${r.timeGap.toFixed(1)} s, ${r.distanceGap.toFixed(0)} m`);
  check("race: times match speeds", Math.abs(r.tA * r.vA - 10000) < 1e-6 && Math.abs(r.tB * r.vB - 10000) < 1e-6);
  check("race: same stroke, same effort is a dead heat", S.race(clean, clean, 5000, 150, 150).timeGap === 0);
  check("race: more effort wins", S.race(sweep, clean, 10000, 200, 150).timeGap > 0);
  check("race: swapping boats flips the gap", Math.abs(S.race(sweep, clean, 10000, 150, 150).timeGap + r.timeGap) < 1e-9);

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
    check(`${scheme}: power split has three parts`, (await page.$$("#bar-power span")).length === 3);
    check(`${scheme}: switch-sides tile`, /\d/.test(await page.textContent("#r-side")), await page.textContent("#r-side"));
    check(`${scheme}: no J-stroke wording`, !/J-stroke|steering/i.test(await page.textContent("body")));

    // Race: Boat A (clean, sliders) against Boat B (wide sweep by default).
    await page.click("#presets >> text=Clean vertical");
    await page.waitForTimeout(150);
    check(`${scheme}: saving makes the saved stroke Boat B`, (await page.$eval("#race-b", (e) => e.value)) === "saved");
    await page.selectOption("#race-b", "Wide sweep");
    check(`${scheme}: race verdict A wins`, /^Boat A wins by [\d.]+ s, \d+ m ahead/.test(await page.textContent("#verdict")),
          await page.textContent("#verdict"));
    check(`${scheme}: finishing times shown`, /\d+:\d\d/.test(await page.textContent("#t-a")) && /\d+:\d\d/.test(await page.textContent("#t-b")),
          `${await page.textContent("#t-a")} / ${await page.textContent("#t-b")}`);
    await page.$eval("#c-effB", (el) => { el.value = 250; el.dispatchEvent(new Event("input")); });
    await page.waitForTimeout(100);
    check(`${scheme}: more effort for B flips the race`, /^Boat B wins/.test(await page.textContent("#verdict")),
          await page.textContent("#verdict"));
    await page.$eval("#c-effB", (el) => { el.value = 150; el.dispatchEvent(new Event("input")); });
    await page.selectOption("#race-b", "Clean vertical");
    await page.waitForTimeout(100);
    check(`${scheme}: same stroke is a dead heat`, (await page.textContent("#verdict")) === "Dead heat.");
    await page.selectOption("#race-b", "Wide sweep");
    await page.click("#race-start");
    await page.waitForTimeout(600);
    check(`${scheme}: race animates two boats`, (await page.$$("#race-view .boat-a, #race-view .boat-b")).length >= 2);
    await page.click("#presets >> text=Short reach, pull behind");
    await page.waitForTimeout(150);
    check(`${scheme}: short-reach preset`, (await page.$eval("#c-catchDeg", (e) => e.value)) === "12");

    // Slider moves the model: more lean, more turning.
    await page.click("#presets >> text=Defaults");
    await page.waitForTimeout(150);
    const yaw = async () => parseFloat((await page.textContent("#r-yaw-sub")).replace(/[^\d.−-]/g, "").replace("−", "-"));
    const yaw0 = await yaw();
    await page.$eval("#c-leanDeg", (el) => { el.value = 35; el.dispatchEvent(new Event("input")); });
    await page.waitForTimeout(150);
    check(`${scheme}: lean slider increases turning`, (await yaw()) > yaw0);
    check(`${scheme}: slider readout`, (await page.textContent("#o-leanDeg")) === "35°");
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
