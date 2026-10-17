// Paddle fit (used by simulator.html; also loadable in Node for tests): the
// seat height, paddle length and hand placement that suit a paddler, on the
// stroke engine (stroke-sim.js).
//
//  - The seat goes as high as the water allows: higher gives leverage the
//    engine doesn't count yet, and costs stability, which it does.
//  - Then paddle length, the lower hand and the top hand are searched for
//    the most speed at the power the paddler can hold, keeping both hands in
//    reach and the top hand between chin and forehead.
//  - A new paddle length keeps the stroke's blade path: the blade goes in
//    and comes out where it did, so a longer paddle sweeps a smaller angle
//    and the arms move slower (a higher gear).
//
// With gearing and strength by position off, setups inside the limits are
// about equally fast; those curves (assumed until measured) rank them.
(function (root) {
  "use strict";
  const S = typeof module !== "undefined" && module.exports ? require("./stroke-sim.js") : root.StrokeSim;

  // How stable the boat must be for the water raced in: the metacentric
  // height to keep (m). Assumed: the default 10 cm seat for rough water.
  const CONDITIONS = [
    { id: "flat", label: "Flat", gmMin: 0.33, help: "Flat water: the seat can go as high as about 25 cm." },
    { id: "chop", label: "Some chop", gmMin: 0.40, help: "Some chop: keeps 40 cm of metacentric height, about a 17 cm seat." },
    { id: "rough", label: "Rough", gmMin: 0.46, help: "Rough water: keeps 46 cm, about the usual 10 cm seat." },
  ];
  // Search ranges [low, high, step] (m).
  const RANGES = {
    seatZM: [0.05, 0.25, 0.025],
    paddleLenM: [47 * 0.0254, 55 * 0.0254, 0.0254],   // 47″ to 55″
    lowerHandM: [0.5, 0.72, 0.02],
    handOutM: [0.25, 0.45, 0.025],
  };
  const MAP_SEATS = [0.05, 0.1, 0.15, 0.2, 0.25];
  // The top hand's height above the shoulder line, chin to forehead, for a
  // 1.78 m paddler (scaled by height).
  const TOP_LOW = 0.10, TOP_HIGH = 0.30, REF_HEIGHT = 1.78;
  // A setup must beat the one it would replace by this share of speed: where
  // the ranking is flat, keep what the paddler has.
  const BETTER = 2e-4;
  const steps = ([a, b, d]) => { const out = []; for (let i = 0; a + i * d <= b + d / 1000; i++) out.push(+(a + i * d).toFixed(4)); return out; };

  // Where the blade centre is fore and aft at drive progress u: the
  // shaft-axis point level with it, as the engine places the blade.
  function bladeX(p, u) {
    const g = S.paddled(Object.assign({}, S.DEFAULTS, p)), P = S.paddleOf(g);
    return S.pose(g, u).at(P.throat + P.copFromThroat)[0];
  }
  // The angle (catchDeg at u = 0, exitDeg at u = 1) that puts the blade at x.
  function angleFor(p, key, u, x) {
    let lo = 0, hi = 85;
    for (let i = 0; i < 40; i++) {
      const m = (lo + hi) / 2, past = u === 0 ? bladeX(Object.assign({}, p, { [key]: m }), u) > x : bladeX(Object.assign({}, p, { [key]: m }), u) < x;
      if (past) hi = m; else lo = m;
    }
    return (lo + hi) / 2;
  }
  // p with its catch and exit angles set so the blade goes in and comes out
  // where it does in ref's stroke.
  function withBladePath(p, ref) {
    const q = Object.assign({}, p);
    q.catchDeg = angleFor(q, "catchDeg", 0, bladeX(ref, 0));
    q.exitDeg = angleFor(q, "exitDeg", 1, bladeX(ref, 1));
    return q;
  }

  // One setup: speed at what the paddler can hold, and the limits.
  function measure(p, needs) {
    const q = Object.assign({}, S.DEFAULTS, p), r = S.simulate(q), body = r.body, watts = S.capacity(q);
    const above = body.frames.map((f) => f.above), k = q.heightM / REF_HEIGHT;
    const top = [Math.min(...above), Math.max(...above)], GM = r.cycleAt(q.effortW).GM;
    // How far past each limit (m): 0 inside it.
    const past = {
      reach: body.short,
      topHand: Math.max(0, TOP_LOW * k - top[0]) + Math.max(0, top[1] - TOP_HIGH * k),
      stable: Math.max(0, needs.gmMin - GM),
    };
    return { setup: p, speed: r.speedAt(watts), watts, GM, top, armSpeed: body.armSpeed, trunkSpeed: body.trunkSpeed, trunkShare: body.trunkShare, strokesPerSide: r.strokesPerSide, past,
             ok: past.reach <= 0.01 && past.topHand === 0 && past.stable === 0 };
  }
  // Inside the limits beats outside them; then the faster, or the nearer the limits.
  const beats = (a, b) => (a.ok !== b.ok ? a.ok : a.ok ? a.speed > b.speed * (1 + BETTER) :
    a.past.reach + a.past.topHand + a.past.stable < b.past.reach + b.past.topHand + b.past.stable - 1e-4);

  // The highest seat in range that keeps the stability needed (stability
  // depends on the seat and weight, not the paddle or hands).
  function* seatFor(p, needs) {
    const seats = steps(RANGES.seatZM).reverse();
    for (const z of seats) {
      const m = measure(Object.assign({}, p, { seatZM: z }), needs);
      yield;
      if (m.GM >= needs.gmMin) return z;
    }
    return seats[seats.length - 1];
  }
  // The search, a step at a time: yields its progress (0–1), returns
  // { current, best } measured.
  function* search(p, needs) {
    const total = 1 + 2 * (steps(RANGES.paddleLenM).length + steps(RANGES.lowerHandM).length + steps(RANGES.handOutM).length);
    let n = 0;
    const current = measure(p, needs);
    const seatZM = yield* seatFor(p, needs);
    let best = measure(Object.assign({}, p, { seatZM }), needs);
    yield ++n / total;
    for (let pass = 0; pass < 2; pass++) {
      let moved = false;
      for (const key of ["paddleLenM", "lowerHandM", "handOutM"]) for (const v of steps(RANGES[key])) {
        yield ++n / total;
        if (Math.abs(v - best.setup[key]) < 1e-6) continue;
        const trial = Object.assign({}, best.setup, { [key]: v });
        const m = measure(key === "paddleLenM" ? withBladePath(trial, p) : trial, needs);
        if (beats(m, best)) { best = m; moved = true; }
      }
      if (!moved) break;
    }
    return { current, best };
  }
  // Seat height × paddle length with the hands of `p`, each length keeping
  // its blade path. `also`: setups whose seat and length get a row and a
  // column of their own (the paddler's and the suggestion), so they're
  // marked where they are, not on the nearest square. Yields progress;
  // returns { seats, lengths, cells }.
  function* map(p, ref, needs, also = []) {
    const merged = (base, vs) => [...base, ...vs.filter((v) => base.every((b) => Math.abs(b - v) > 1e-3))].sort((a, b) => a - b)
      .filter((v, i, a) => i === 0 || v - a[i - 1] > 1e-3);
    const seats = merged(MAP_SEATS, also.map((q) => q.seatZM)), lengths = merged(steps(RANGES.paddleLenM), also.map((q) => q.paddleLenM)), cells = [];
    for (const seatZM of seats) for (const paddleLenM of lengths) {
      cells.push(Object.assign({ seatZM, paddleLenM }, measure(withBladePath(Object.assign({}, p, { seatZM, paddleLenM }), ref), needs)));
      yield cells.length / (seats.length * lengths.length);
    }
    return { seats, lengths, cells };
  }

  const api = { CONDITIONS, RANGES, TOP_LOW, TOP_HIGH, REF_HEIGHT, bladeX, withBladePath, measure, search, map };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.FitSim = api;
})(typeof window !== "undefined" ? window : globalThis);
