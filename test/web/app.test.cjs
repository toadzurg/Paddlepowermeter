// Browser test for the web app (docs/app/index.html) against a simulated
// paddle. Needs Node and Playwright (Chromium):
//
//   ./test/web/run.sh
//
// The simulated paddle replaces navigator.bluetooth. Its packets are the
// bytes the C++ firmware structs produce (test/host/test_ble_protocol), and
// its replies arrive in 20-byte chunks, as from a phone with the minimum
// Bluetooth packet size.

const { chromium } = require("playwright");
const path = require("path");
const fs = require("fs");

const root = path.resolve(__dirname, "../..");
const pageUrl = "file://" + path.join(root, "docs/app/index.html");
const samples = JSON.parse(fs.readFileSync(process.argv[2], "utf8"));
const shotDir = process.argv[3];
let failures = 0;

function check(name, ok, detail = "") {
  console.log(`  [${ok ? " ok " : "FAIL"}] ${name}${detail ? "  " + detail : ""}`);
  if (!ok) failures++;
}

// Runs in the page before its scripts: a fake paddle.
function installMockPaddle() {
  const SERVICE = "7e1a0001-5c3b-4c2e-9a77-0f5d3e2b9c10";
  const ids = { live: "7e1a0002", stroke: "7e1a0003", command: "7e1a0004", reply: "7e1a0005" };
  class Char {
    constructor() { this.listeners = []; this.value = null; }
    addEventListener(_, f) { this.listeners.push(f); }
    async startNotifications() { return this; }
    async readValue() { return this.value; }
    emit(dv) { this.value = dv; for (const f of this.listeners) f({ target: this }); }
  }
  const chars = {};
  for (const [k, short] of Object.entries(ids)) chars[k] = new Char();
  const battery = new Char();
  battery.value = new DataView(new Uint8Array([87]).buffer);

  const mock = {
    writes: [], pin: "123456", unlocked: false, connects: 0,
    sendReply(text) {
      const bytes = new TextEncoder().encode(text);
      for (let i = 0; i < bytes.length; i += 20)
        chars.reply.emit(new DataView(bytes.slice(i, i + 20).buffer));
    },
    onWrite(text) {
      this.writes.push(text);
      setTimeout(() => {
        if (text.startsWith("auth ")) {
          this.unlocked = text.slice(5) === this.pin;
          this.sendReply(this.unlocked ? "ok: unlocked\n" : "error: wrong PIN\n");
          return;
        }
        if (!this.unlocked) { this.sendReply("error: locked, send auth <pin>\n"); return; }
        const answers = {
          tare: "tared: zero offset 81234\n",
          status: "IMU1 ok, IMU2 ok\nzero offset 81234, 1234.500 counts/N\npivot -0.142 m, lever arm 1.092 m, battery 87%\n",
        };
        this.sendReply((answers[text] || `echo ${text}\n`) + "ok\n");
      }, 20);
    },
    hex(h) { return new DataView(new Uint8Array(h.match(/../g).map((x) => parseInt(x, 16))).buffer); },
    pushLive(h) { chars.live.emit(this.hex(h)); },
    pushStroke(h) { chars.stroke.emit(this.hex(h)); },
  };
  chars.command.writeValueWithResponse = async (data) => mock.onWrite(new TextDecoder().decode(data));

  const services = {
    [SERVICE]: { getCharacteristic: async (u) => chars[Object.keys(ids).find((k) => u.startsWith(ids[k]))] },
    battery_service: { getCharacteristic: async () => battery },
  };
  const listeners = [];
  const device = {
    name: "PaddlePower",
    addEventListener: (_, f) => listeners.push(f),
    gatt: {
      connect: async () => { mock.connects++; return { getPrimaryService: async (u) => services[u] }; },
      disconnect: () => listeners.forEach((f) => f()),
    },
  };
  mock.dropConnection = () => listeners.forEach((f) => f());
  Object.defineProperty(navigator, "bluetooth", {
    configurable: true,
    value: { requestDevice: async (opts) => { mock.requestOpts = opts; return device; } },
  });
  window.__mock = mock;
}

// Live packet bytes in the firmware layout, built here independently of
// the page's own encoder.
function liveHex(seq, tMs, inDrive, force) {
  const b = Buffer.alloc(17);
  b.writeUInt16LE(seq, 0); b.writeUInt16LE(tMs & 0xffff, 2); b.writeUInt8(inDrive ? 1 : 0, 4);
  b.writeInt16LE(Math.round(force * 10), 5); b.writeInt16LE(250, 7); b.writeInt16LE(300, 9);
  b.writeUInt16LE(0, 11); b.writeInt16LE(-140, 13); b.writeInt16LE(1090, 15);
  return b.toString("hex");
}

(async () => {
  const browser = await chromium.launch();

  // ---------------------------------------------------- unsupported browser
  console.log("\nWeb app: browser without Bluetooth");
  {
    const ctx = await browser.newContext();
    await ctx.addInitScript(() => Object.defineProperty(navigator, "bluetooth", { value: undefined }));
    const page = await ctx.newPage();
    await page.goto(pageUrl);
    check("explains Bluefy / Chrome", await page.isVisible("#unsupported"));
    await ctx.close();
  }

  for (const scheme of ["light", "dark"]) {
    console.log(`\nWeb app: simulated paddle (${scheme})`);
    const ctx = await browser.newContext({ viewport: { width: 390, height: 844 }, colorScheme: scheme,
                                           deviceScaleFactor: 2 });
    await ctx.addInitScript(installMockPaddle);
    const page = await ctx.newPage();
    const errors = [];
    page.on("pageerror", (e) => errors.push(e.message));
    page.on("dialog", (d) => d.accept());
    await page.goto(pageUrl);

    if (scheme === "light") {
      // -------------------------------- decoders vs firmware-produced bytes
      for (const [kind, fn] of [["live", "decodeLive"], ["stroke", "decodeStroke"]]) {
        for (const s of samples[kind]) {
          const got = await page.evaluate(([fn, h]) => {
            const dv = new DataView(new Uint8Array(h.match(/../g).map((x) => parseInt(x, 16))).buffer);
            return window.ppm[fn](dv);
          }, [fn, s.hex]);
          const bad = Object.entries(s.expect).filter(([k, v]) =>
            typeof v === "number" ? Math.abs(got[k] - v) > 1e-6 : got[k] !== v);
          check(`${fn} matches firmware bytes ${s.hex.slice(0, 8)}…`, bad.length === 0,
                bad.map(([k, v]) => `${k}: got ${got[k]} want ${v}`).join(", "));
        }
      }
    }

    // ----------------------------------------------------------- connect
    await page.click("#connect");
    await page.waitForFunction(() => window.__mock.connects === 1);
    const opts = await page.evaluate(() => window.__mock.requestOpts);
    check("asks for the paddle by name", opts.filters[0].namePrefix === "PaddlePower");
    check("requests the app service", opts.optionalServices.includes("7e1a0001-5c3b-4c2e-9a77-0f5d3e2b9c10"));

    await page.evaluate((h) => window.__mock.pushLive(h), samples.live[0].hex);
    await page.evaluate((h) => window.__mock.pushStroke(h), samples.stroke[0].hex);
    await page.waitForTimeout(600);
    check("pill shows connected", (await page.textContent("#pill-text")).startsWith("Connected"));
    check("stroke power", (await page.textContent("#v-power")) === "173W");
    check("stroke rate", (await page.textContent("#v-rate")) === "50spm");
    check("efficiency above 100% shown, not capped", (await page.textContent("#v-eff")) === "107%");
    check("useful / slip", (await page.textContent("#v-eff-sub")) === "useful 149\u00a0W · slip 25\u00a0W");
    check("impulse per stroke", (await page.textContent("#v-impulse")) === "78.5N·s", await page.textContent("#v-impulse"));
    check("drive time and duty", (await page.textContent("#v-impulse-sub")) === "drive 0.57\u00a0s · 47% of the stroke",
          await page.textContent("#v-impulse-sub"));
    check("mean push = impulse × rate / 60", (await page.textContent("#v-push")) === "65N", await page.textContent("#v-push"));
    check("force", (await page.textContent("#v-force")) === "187N");
    check("boat speed km/h", (await page.textContent("#v-speed")) === "8.9km/h",
          await page.textContent("#v-speed"));
    check("battery", (await page.textContent("#v-batt")) === "87%");

    // ------------------------------------------------------ force curve
    let seq = 100, t = 1000;
    for (let i = 0; i < 4; i++) {            // 4 strokes, peak 200 N on the last
      const peak = 150 + i * 50 / 3;
      for (let k = 0; k <= 12; k++) await page.evaluate((h) => window.__mock.pushLive(h),
        liveHex(seq++, (t += 50), true, peak * Math.sin(Math.PI * k / 12)));
      for (let k = 0; k < 10; k++) await page.evaluate((h) => window.__mock.pushLive(h),
        liveHex(seq++, (t += 50), false, 1));
    }
    await page.waitForTimeout(100);
    check("force curve drawn", await page.isVisible("#chart"));
    const widest = await page.evaluate(() => Math.max(...window.ppm.state.curves.map((c) => c[c.length - 1][0])));
    check("dropout mid-drive discarded (no stretched curve)", widest < 0.7, `widest ${widest.toFixed(2)} s`);
    check("history strokes drawn", (await page.$$("#chart path.history")).length === 3);
    check("17-byte packets from older firmware still decode", await page.isVisible("#stroke-view"));
    check("stroke view asks for newer firmware when there's no angle",
          (await page.textContent("#stroke-view")).includes("update the paddle firmware"));
    check("peak labelled", (await page.textContent("#chart text.peak-label")) === "peak 200 N",
          await page.textContent("#chart text.peak-label"));
    await page.locator("#chart").scrollIntoViewIfNeeded();
    const box = await page.locator("#chart").boundingBox();
    await page.mouse.move(box.x + box.width * 0.45, box.y + box.height * 0.5);
    const tipBox = await page.locator("#tooltip").boundingBox();
    check("hover readout stays inside the plot", tipBox.y >= box.y, `tip ${tipBox.y.toFixed(0)} vs plot ${box.y.toFixed(0)}`);
    check("hover readout", /s · \d+ N/.test(await page.textContent("#tooltip")),
          await page.textContent("#tooltip"));
    if (shotDir) await page.screenshot({ path: path.join(shotDir, `dashboard-${scheme}.png`), fullPage: true });
    await page.mouse.move(0, 0);

    // ------------------------------------------------ live / held curve
    const push = (inDrive, f) => page.evaluate((h) => window.__mock.pushLive(h), liveHex(seq++, (t += 50), inDrive, f));
    const pressedLive = () => page.$eval("#curve-live", (e) => e.getAttribute("aria-pressed"));
    check("force curve is live by default", (await pressedLive()) === "true" &&
          (await page.textContent("#curve-sub")).startsWith("Live"));
    for (let k = 0; k <= 5; k++) await push(true, 240 * Math.sin(Math.PI * k / 12));
    await page.waitForTimeout(100);
    check("live: the stroke in progress draws, with a now dot", (await page.$$("#now-dot")).length === 1 &&
          (await page.textContent("#now-label")) === `now ${Math.round(240 * Math.sin(Math.PI * 5 / 12))} N`,
          await page.textContent("#now-label"));
    await page.click("#curve-live");
    await page.waitForTimeout(50);
    check("held: shows the last completed stroke, no now dot", (await pressedLive()) === "false" &&
          (await page.$$("#now-dot")).length === 0 && (await page.textContent("#chart text.peak-label")) === "peak 200 N" &&
          (await page.textContent("#curve-sub")).startsWith("Held"));
    for (let k = 6; k <= 12; k++) await push(true, 240 * Math.sin(Math.PI * k / 12));
    await page.waitForTimeout(100);
    check("held: doesn't move while you paddle", (await page.textContent("#chart text.peak-label")) === "peak 200 N");
    await push(false, 1);
    await page.waitForTimeout(100);
    check("held: updates when the stroke completes", (await page.textContent("#chart text.peak-label")) === "peak 240 N",
          await page.textContent("#chart text.peak-label"));
    check("held choice remembered", (await page.evaluate(() => localStorage.getItem("ppm.curveLive"))) === "0");
    await page.click("#curve-live");
    check("back to live", (await pressedLive()) === "true" && (await page.evaluate(() => localStorage.getItem("ppm.curveLive"))) === "1");

    // ------------------------------------------------------ setup / commands
    await page.click('nav button[data-tab="setup"]');
    await page.click('button[data-cmd="tare"]');
    await page.waitForSelector("#console .err");
    check("locked paddle refuses tare", (await page.textContent("#console")).includes("error: locked"));

    await page.fill("#pin", "000000");
    await page.click("#unlock");
    await page.waitForFunction(() => document.querySelector("#console").textContent.includes("wrong PIN"));
    check("wrong PIN rejected", (await page.textContent("#lock-state")) === "Locked");
    check("PIN not echoed in console", !(await page.textContent("#console")).includes("000000"));

    await page.fill("#pin", "123456");
    await page.click("#unlock");
    await page.waitForFunction(() => document.querySelector("#lock-state").textContent === "Unlocked");
    check("right PIN unlocks", true);

    await page.click('button[data-cmd="tare"]');
    await page.waitForFunction(() => document.querySelector("#console").textContent.includes("tared:"));
    check("tare sent after confirm", (await page.evaluate(() => window.__mock.writes)).includes("tare"));

    await page.click('button[data-cmd="status"]');
    await page.waitForFunction(() => document.querySelector("#console").textContent.includes("lever arm 1.092"));
    const consoleText = await page.textContent("#console");
    check("multi-line reply reassembled from 20-byte chunks",
          consoleText.includes("IMU1 ok, IMU2 ok") && consoleText.includes("zero offset 81234, 1234.500 counts/N"));
    check("end-of-reply marker hidden", !/\nok\n|^ok$/m.test(await page.innerText("#console")));

    await page.fill("#cal-kg", "5");
    await page.click("#calibrate");
    await page.waitForTimeout(100);
    check("calibrate sends newtons", (await page.evaluate(() => window.__mock.writes)).includes("cal 49.03"));
    if (shotDir) await page.screenshot({ path: path.join(shotDir, `setup-${scheme}.png`), fullPage: true });

    // ----------------------------------------------- reconnect re-unlocks
    await page.evaluate(() => { window.__mock.unlocked = false; window.__mock.dropConnection(); });
    await page.waitForFunction(() => window.__mock.connects === 2);
    await page.waitForFunction(() => window.__mock.writes.filter((w) => w === "auth 123456").length === 2);
    check("reconnects and unlocks with the saved PIN", true);

    check("no page errors", errors.length === 0, errors.join("; "));
    await ctx.close();
  }

  // ------------------------------------------------- startup diagnostics
  console.log("\nWeb app: startup diagnostics");
  {
    const ctx = await browser.newContext();
    const page = await ctx.newPage();
    await page.goto(pageUrl);
    check("'Starting the app' note hidden once the script runs", !(await page.isVisible("#js-status")));
    check("version shown", /App version \d{4}-\d\d-\d\d/.test(await page.textContent("#version")));

    // A copy of the page whose main script can't parse, as an engine
    // missing some syntax would see it.
    const broken = path.resolve(shotDir || "/tmp", "broken.html");
    fs.writeFileSync(broken, fs.readFileSync(path.join(root, "docs/app/index.html"), "utf8")
      .replace('const APP_VERSION', 'const APP_VERSION = = '));
    await page.goto("file://" + broken);
    const msg = await page.textContent("#js-status-text");
    check("parse error reported on the page", msg.startsWith("App error") && /line \d+/.test(msg), msg);
    await ctx.close();
  }

  // ------------------------------------------- browser-injected scripts
  // Bluefy (and similar) inject their own scripts into the page. These were
  // the cause of "demo does nothing" in Bluefy on iPhone.
  console.log("\nWeb app: alongside browser-injected scripts");
  {
    const src = fs.readFileSync(path.join(root, "docs/app/index.html"), "utf8");
    const variant = (name, html) => {
      const f = path.resolve(shotDir || "/tmp", name);
      fs.writeFileSync(f, html);
      return "file://" + f;
    };
    const cases = [
      ["another script uses the same global names",
       variant("collide.html", src.replace("<main>", "<script>let state = {}; function log() {} var $ = 1;</script><main>")), null],
      ["Bluetooth add-on throws when read", pageUrl,
       () => Object.defineProperty(navigator, "bluetooth", { configurable: true,
         get() { throw new Error("polyfill not ready"); } })],
      ["outside script reports 'Script error.'", pageUrl, null],
    ];
    for (const [name, url, init] of cases) {
      const ctx = await browser.newContext();
      if (init) await ctx.addInitScript(init);
      const page = await ctx.newPage();
      const errors = [];
      page.on("pageerror", (e) => errors.push(e.message));
      await page.goto(url + "?demo");
      if (name.startsWith("outside"))  // as WebKit reports an injected script's error
        await page.evaluate(() => window.dispatchEvent(new ErrorEvent("error", { message: "Script error." })));
      await page.waitForTimeout(1200);
      const ok = (await page.textContent("#pill-text")) === "Demo" && !(await page.isVisible("#js-status"));
      check(`${name}: app still starts, no error box`, ok,
            (await page.textContent("#js-status-text")).slice(0, 90));
      if (name.startsWith("outside")) {
        await page.click('nav button[data-tab="setup"]');
        check("outside error noted in the console instead",
              (await page.textContent("#console")).includes("not this app"));
      }
      await ctx.close();
    }
  }

  // ----------------------------------------------------------------- demo
  console.log("\nWeb app: demo mode");
  {
    const ctx = await browser.newContext({ viewport: { width: 390, height: 844 } });
    await ctx.addInitScript(() => Object.defineProperty(navigator, "bluetooth", { value: undefined }));
    const page = await ctx.newPage();
    const errors = [];
    page.on("pageerror", (e) => errors.push(e.message));
    await page.goto(pageUrl);
    await page.click("#demo-2");
    await page.waitForTimeout(4000);
    check("demo button shows it's running", (await page.textContent("#demo")) === "Stop demo");
    check("demo shows stroke power", /^\d+W$/.test(await page.textContent("#v-power")),
          await page.textContent("#v-power"));
    check("demo draws force curves", await page.isVisible("#chart"));
    check("demo shows impulse per stroke", /^\d+\.\dN·s$/.test(await page.textContent("#v-impulse")),
          await page.textContent("#v-impulse"));
    // Older firmware sends a 15-byte stroke packet without impulse.
    const old = await page.evaluate(() => {
      const dv = new DataView(new ArrayBuffer(15));
      dv.setUint16(0, 3, true); dv.setInt16(2, 150, true); dv.setUint16(4, 5000, true);
      return window.ppm.decodeStroke(dv);
    });
    check("15-byte stroke packet (old firmware) decodes without impulse", old.power === 150 && old.impulse === null && old.duty === null);
    check("links to the stroke simulator", (await page.getAttribute("#sim-link", "href")) === "simulator.html");
    check("stroke view drawn", await page.isVisible("#stroke-view") &&
          (await page.$$("#stroke-view line.blade")).length === 1);
    const pivots = [];
    for (let i = 0; i < 6; i++) {
      pivots.push(parseFloat((await page.textContent("#v-pivot")).replace("\u2212", "-")));
      await page.waitForTimeout(1000);
    }
    const spread = Math.max(...pivots) - Math.min(...pivots);
    check("demo pivot is dynamic", spread > 0.03, `moved ${(spread * 100).toFixed(0)} cm in 6 s`);
    const lever = parseFloat(await page.textContent("#v-lever"));
    const pivotNow = parseFloat((await page.textContent("#v-pivot")).replace("\u2212", "-"));
    // Bent shaft (12°): L = 0.25 + (0.60 − pivot)·cos 12°, as in the firmware.
    const expectL = 0.25 + (0.60 - pivotNow) * Math.cos(12 * Math.PI / 180);
    check("lever arm uses the bent-shaft formula", Math.abs(lever - expectL) < 0.015, `${lever} vs ${expectL.toFixed(3)}`);
    if (shotDir) {
      await page.waitForFunction(() => document.querySelector("#stroke-view line.force"), null, { timeout: 3000 });
      await page.locator("#stroke-view").screenshot({ path: path.join(shotDir, "stroke-drive.png") });
      await page.waitForFunction(() => !document.querySelector("#stroke-view line.force"), null, { timeout: 3000 });
      await page.waitForTimeout(250);
      await page.locator("#stroke-view").screenshot({ path: path.join(shotDir, "stroke-recovery.png") });
    }
    await page.click("#demo");
    check("demo button stops it", (await page.textContent("#demo")) === "Try demo" &&
          (await page.textContent("#pill-text")) === "Not connected");
    check("no page errors", errors.length === 0, errors.join("; "));

    await page.goto(pageUrl + "?demo");
    await page.waitForTimeout(1500);
    check("?demo link starts the demo", (await page.textContent("#pill-text")) === "Demo");
    check("?demo removed from the address", !new URL(page.url()).search, page.url());
    await page.reload();
    await page.waitForTimeout(1200);
    check("reload opens the normal app", (await page.textContent("#pill-text")) === "Not connected");
    await ctx.close();
  }

  await browser.close();
  console.log(`\n${failures ? "FAILED" : "PASSED"} (${failures} failure${failures === 1 ? "" : "s"})`);
  process.exit(failures ? 1 : 0);
})().catch((e) => { console.error(e); process.exit(1); });
