// Stroke simulator physics (used by simulator.html; also loadable in Node
// for tests). A simplified, quasi-static model meant for comparing
// techniques, not for predicting absolute speed:
//
//  - Boat frame: X toward the bow, Y to port (left), Z up, waterline Z = 0.
//    The paddle is on the starboard (right, −Y) side.
//  - The shaft is set by its swing angle θ in the stroke plane (catch → exit),
//    its lean φ outward from vertical, and the stroke direction ψ relative
//    to the keel (a sweep turns ψ during the drive). The pivot is a point
//    on the shaft; it may travel aft during the drive, and it rises or
//    drops so the blade stays buried to the throat, as a good paddler does.
//    Strokes then differ by direction and placement, not by how deep the
//    blade happens to be.
//  - The blade face is kept square to its motion (best case), bent by β
//    toward the bow side like PowerModel. Water pushes on the face with drag
//    ½ρ·Cd·A·(normal speed)², using the blade's speed through the water
//    (its motion in the boat plus the boat's own speed) and the immersed
//    part of the blade only.
//  - Pushing off the boat's centreline turns it (paddling on the right
//    turns the bow left). The paddler corrects by switching sides
//    (hit and switch): the hull resists turning in proportion to its speed,
//    so the boat drifts off course at a steady rate, and the paddler
//    switches when the heading is off by switchHeadingDeg. Each switch
//    loses switchLossStrokes of a stroke with no propulsion. A stroke that
//    turns the boat more forces more switches, so more lost time.
//  - Sideways push slows the boat too. The boat (with paddler and the water
//    it carries along, SWAY_MASS) sways under the stroke's sideways force,
//    and the hull resists by crabbing slightly through the water, like a
//    long shallow keel. The force it carries, Y, costs induced drag
//    Y² / (½ρV²·π·(2·draft)²). Brief pushes mostly just sway the boat;
//    steady ones are carried by the hull and cost the most.
//  - The push comes in pulses (drive, then recovery), so the boat surges:
//    faster during the drive, slower in the recovery. Drag grows with V², so
//    an uneven speed costs more than a steady one at the same average
//    (mean of V³ > V̄³). Only the paddle's push is modelled, not your body
//    moving fore and aft in the boat.
//  - Boat speed is where mean thrust (counting the switches) balances hull
//    drag k·V² plus leeway and surge drag. Then paddler power = moving the
//    boat (k·V³) + leeway + surge + slip (power into the water), exactly.
//    With quadratic drag on blade and hull, these shares do not depend on
//    stroke rate or effort: every speed and force scales together.
//  - Vertical push. A blade angled forward at the catch pushes water down
//    and lifts the boat; angled back at the exit it lifts water and pulls
//    the boat down. The average over the stroke changes how deep the boat
//    sits: drag scales with wetted area, about displacement^(2/3). The
//    swing in vertical force bounces the boat on its waterplane (a spring,
//    ρ·g·area) and the bouncing makes waves, which cost power. These grow
//    with effort relative to the boat's weight, so they are worked out at
//    the comparison effort and your stroke rate.
//  - Efficiency = moving the boat / paddler power, where moving the boat is
//    k·V³ for the boat at its own weight with no vertical push. A stroke
//    that lifts the boat can earn a little back.

(function (root) {
  "use strict";
  const RHO = 1000, CD = 1.6, BLADE_AREA = 0.095;  // ~20"×8.5" marathon blade
  const HULL_K = 5.5;            // N/(m/s)², C1 marathon canoe
  const DRIVE_FRACTION = 0.5;    // of the stroke cycle
  // Turning resistance of the hull: c = C_TRACK·ρ·V·L³·d / 12 (N·m·s/rad),
  // for a hull of length L and draft d. C_TRACK is set so a clean vertical
  // stroke allows about 8 strokes per side at ±3°, typical of hit and
  // switch; the comparison between strokes is what matters.
  const HULL_L = 5.6, HULL_DRAFT = 0.12, C_TRACK = 0.20;
  const MAX_STROKES_PER_SIDE = 20;  // you'd switch by then anyway (fatigue)
  const SWAY_MASS = 140;            // kg: boat + paddler + entrained water
  // Hull as a slender lifting surface: area L·d, aspect ratio (2d)²/(L·d),
  // lift slope π·AR/2 per radian (slender-body theory).
  const HULL_AR = Math.pow(2 * HULL_DRAFT, 2) / (HULL_L * HULL_DRAFT);
  const HULL_LIFT_SLOPE = Math.PI * HULL_AR / 2;
  const BOAT_HALF_BEAM = 0.36;   // m, at the gunwale
  const G = 9.81;
  const MASS_REF = 90;           // kg, paddler + boat that HULL_K is for
  const SURGE_ADDED = 0.05;      // entrained water moving with the hull, fore-aft
  // Heave: waterplane area of a C1 marathon hull (5.6 m, ~0.6 m wide at the
  // waterline), added mass of the water moving with it, and damping (wave
  // making) as a fraction of critical.
  const WATERPLANE = 2.3, HEAVE_ADDED = 1.0, HEAVE_ZETA = 0.3;
  // Air drag of paddler and boat (CdA, m²), used for wind in race().
  const RHO_AIR = 1.2, CDA_AIR = 0.45;

  // Bent-shaft marathon paddle, distances from the grip (m).
  const PADDLE = { throat: 0.82, bladeLen: 0.50, copFromThroat: 0.25 };

  const DEFAULTS = {
    pivotFromGrip: 0.10,  // m: the point on the shaft that stays put
    leanDeg: 10,          // outward from vertical (end view)
    catchDeg: 35,         // forward reach at the catch
    exitDeg: 20,          // angle behind vertical at the exit
    directionDeg: 0,      // stroke direction vs the keel; + = aft and outward
    sweepDeg: 0,          // how much the direction turns during the drive
    handOutM: 0.40,       // top hand from the centreline
    handTravelM: 0.15,    // pivot moves aft this far during the drive
    bendDeg: 12,
    rateSpm: 60,
    massKg: 90,                // paddler + boat
    effortW: 150,              // effort the effort-dependent results are for
    switchHeadingDeg: 3,       // switch sides when this far off course
    switchLossStrokes: 0.3,    // time lost per switch, in strokes
  };

  const rad = (d) => (d * Math.PI) / 180;
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => Math.hypot(a[0], a[1], a[2]);
  const unit = (a) => { const n = norm(a) || 1; return mul(a, 1 / n); };

  // Smooth 0→1 progress, zero speed at both ends of the drive.
  const ease = (u) => (1 - Math.cos(Math.PI * u)) / 2;

  // Paddle pose at drive progress u ∈ [0, 1].
  function pose(p, u) {
    const e = ease(u);
    const th = rad(-p.catchDeg + (p.catchDeg + p.exitDeg) * e);
    const psi = rad(p.directionDeg + p.sweepDeg * (0.5 - e));
    const phi = rad(p.leanDeg);
    // Shaft direction (grip → blade): start pointing down, swing by θ about
    // the lateral axis (+θ moves the blade aft), lean outboard by φ, then
    // turn the stroke plane by ψ about the vertical.
    let d = [-Math.sin(th), 0, -Math.cos(th)];
    d = [d[0], d[1] * Math.cos(-phi) - d[2] * Math.sin(-phi), d[1] * Math.sin(-phi) + d[2] * Math.cos(-phi)];
    d = [d[0] * Math.cos(psi) - d[1] * Math.sin(psi), d[0] * Math.sin(psi) + d[1] * Math.cos(psi), d[2]];
    // The pivot stays put in the boat (plus hand travel aft), at the height
    // that puts the throat at the waterline.
    const pivot = [-p.handTravelM * e, -p.handOutM, -d[2] * (PADDLE.throat - p.pivotFromGrip)];
    const at = (s) => add(pivot, mul(d, s - p.pivotFromGrip));  // point s m from the grip
    return { d, pivot, at, th, psi };
  }

  // Blade geometry given the shaft direction and the power-face direction.
  function blade(p, P, faceDir) {
    const b = rad(p.bendDeg);
    const ey = unit(sub(faceDir, mul(P.d, dot(faceDir, P.d))));  // ⊥ shaft
    const along = unit(add(mul(P.d, Math.cos(b)), mul(ey, -Math.sin(b))));
    const n = unit(add(mul(P.d, Math.sin(b)), mul(ey, Math.cos(b))));  // power face normal
    const throat = P.at(PADDLE.throat);
    const cop = add(throat, mul(along, PADDLE.copFromThroat));
    const tip = add(throat, mul(along, PADDLE.bladeLen));
    // Immersed fraction of the blade, along its length.
    const z0 = throat[2], z1 = tip[2];
    let wet = 0;
    if (z0 <= 0 && z1 <= 0) wet = 1;
    else if (z0 > 0 && z1 < 0) wet = -z1 / (z0 - z1);
    else if (z1 > 0 && z0 < 0) wet = -z0 / (z1 - z0);
    return { n, cop, throat, tip, wet };
  }

  // Samples one drive at a given boat speed V. Velocities in the boat frame.
  function drive(p, V, steps = 240) {
    const Tc = 60 / p.rateSpm, Td = Tc * DRIVE_FRACTION, dt = Td / steps;
    const k = 0.5 * RHO * CD * BLADE_AREA;
    const out = [];
    for (let i = 0; i <= steps; i++) {
      const u = i / steps, du = 1e-3;
      const P = pose(p, u);
      const Pa = pose(p, Math.max(0, u - du)), Pb = pose(p, Math.min(1, u + du));
      const span = (Math.min(1, u + du) - Math.max(0, u - du)) * Td;
      // Velocity of the shaft-axis point level with the blade centre.
      const sC = PADDLE.throat + PADDLE.copFromThroat;
      let vAxis = mul(sub(Pb.at(sC), Pa.at(sC)), 1 / span);
      // At the very ends the blade is momentarily still; use the swing
      // direction there so the face orientation stays defined.
      let face = vAxis;
      if (norm(face) < 1e-6) face = sub(pose(p, Math.min(1, u + 0.02)).at(sC), pose(p, Math.max(0, u - 0.02)).at(sC));
      const B = blade(p, P, face);
      const Ba = blade(p, Pa, face), Bb = blade(p, Pb, face);
      const vBlade = mul(sub(Bb.cop, Ba.cop), 1 / span);       // relative to the boat
      const vWater = add(vBlade, [V, 0, 0]);                   // through the water
      const vn = dot(vWater, B.n);
      const F = vn > 0 ? k * B.wet * vn * vn : 0;              // on the power face
      const thrust = mul(B.n, -F);                             // on the boat
      out.push({
        u, t: u * Td, dt: i === 0 || i === steps ? dt / 2 : dt,
        P, B, F, thrust,
        paddler: F * dot(B.n, vBlade),
        useful: thrust[0] * V,
        slip: F * vn,
        yaw: cross(B.cop, thrust)[2],  // about the paddler, bow-left positive
      });
    }
    return { samples: out, Tc };
  }

  // Leeway drag: sway the boat under the sideways force through a few
  // stroke cycles (to settle), then average the hull's induced drag over
  // the last one. The hull resists sway with Y = b·v, b = ½ρV·A·lift slope.
  function leewayDrag(samples, Tc, V, mass = MASS_REF) {
    const swayMass = SWAY_MASS + (mass - MASS_REF);
    if (V <= 0) return 0;
    const b = 0.5 * RHO * V * HULL_L * HULL_DRAFT * HULL_LIFT_SLOPE;
    const qpb2 = 0.5 * RHO * V * V * Math.PI * Math.pow(2 * HULL_DRAFT, 2);
    const n = 400, dt = Tc / n, Td = samples[samples.length - 1].t;
    const fy = (t) => pulse(samples, Td, t, 1);   // sideways push at cycle time t
    let v = 0, sumD = 0;
    const cycles = 6;
    for (let c = 0; c < cycles; c++)
      for (let i = 0; i < n; i++) {
        const t = i * dt, Y = b * v;
        v += ((fy(t) - Y) / swayMass) * dt;   // explicit Euler; τ = m/b ≫ dt
        if (c === cycles - 1) sumD += (Y * Y) / qpb2;
      }
    return sumD / n;
  }

  // Thrust at cycle time t (zero in the recovery), component k.
  function pulse(samples, Td, t, k) {
    if (t > Td) return 0;
    const i = Math.min(samples.length - 1, Math.round((t / Td) * (samples.length - 1)));
    return samples[i].thrust[k];
  }

  // Surge drag: the boat's speed V + δ(t) under the pulsed push, linearised
  // about V (m·δ' = Fx − F̄ − 2kV·δ). The extra hull drag power averages
  // k·((V+δ)³ − V³); returned as the equivalent force at V.
  function surgeDrag(samples, Tc, V, mass, k) {
    if (V <= 0) return 0;
    const m = mass * (1 + SURGE_ADDED), Td = samples[samples.length - 1].t;
    const n = 400, dt = Tc / n;
    let mean = 0;
    for (let i = 0; i < n; i++) mean += pulse(samples, Td, i * dt, 0) / n;
    let d = 0;
    const cycles = 8, last = [];
    for (let c = 0; c < cycles; c++)
      for (let i = 0; i < n; i++) {
        d += ((pulse(samples, Td, i * dt, 0) - mean - 2 * k * V * d) / m) * dt;
        if (c === cycles - 1) last.push(d);
      }
    const avg = last.reduce((a, x) => a + x, 0) / n;
    let extra = 0;
    for (const x of last) { const v = V + x - avg; extra += k * (v * v * v - V * V * V) / n; }
    return Math.max(0, extra / V);
  }

  // Heave under the vertical push, at stroke period Tc (s), with the force
  // scaled by `scale`. Spring ρ·g·waterplane, mass with added water,
  // damping HEAVE_ZETA of critical. Returns the wave power (W), the bounce
  // amplitude (m) and the mean vertical push (N, + lifts the boat).
  function heave(samples, Td0, Tc, scale, mass) {
    const K = RHO * G * WATERPLANE, m = mass * (1 + HEAVE_ADDED);
    const c = 2 * HEAVE_ZETA * Math.sqrt(K * m);
    const Td = Tc * DRIVE_FRACTION, n = 1000, dt = Tc / n;
    const fz = (t) => (t > Td ? 0 : scale * pulse(samples, Td0, (t / Td) * Td0, 2));
    let mean = 0;
    for (let i = 0; i < n; i++) mean += fz(i * dt) / n;
    let z = 0, v = 0, power = 0, lo = Infinity, hi = -Infinity;
    const cycles = 12;
    for (let k = 0; k < cycles; k++)
      for (let i = 0; i < n; i++) {
        v += ((fz(i * dt) - mean - c * v - K * z) / m) * dt;   // semi-implicit Euler
        z += v * dt;
        if (k === cycles - 1) { power += c * v * v / n; lo = Math.min(lo, z); hi = Math.max(hi, z); }
      }
    return { power, amplitude: (hi - lo) / 2, meanLift: mean };
  }

  // Mean forward thrust, mean yaw moment, leeway and surge drag at speed V.
  function means(p, V, k, steps = 120) {
    const { samples, Tc } = drive(p, V, steps);
    let thrust = 0, yaw = 0;
    for (const s of samples) { thrust += s.thrust[0] * s.dt; yaw += s.yaw * s.dt; }
    return { thrust: thrust / Tc, yaw: yaw / Tc, leeway: leewayDrag(samples, Tc, V, p.massKg),
             surge: surgeDrag(samples, Tc, V, p.massKg, k) };
  }
  const yawDamping = (V) => C_TRACK * RHO * V * Math.pow(HULL_L, 3) * HULL_DRAFT / 12;

  // Hit and switch. A mean yaw moment M turns the boat at M / c rad/s; the
  // heading swings from +tol to −tol between switches, so a side lasts
  // 2·tol·c / (M·Tc) strokes. Returns strokes per side and the share of
  // time spent paddling (the rest is switching).
  function switching(p, V, yaw, Tc) {
    const tol = rad(p.switchHeadingDeg);
    const perSide = Math.abs(yaw) > 1e-9
      ? Math.min(MAX_STROKES_PER_SIDE, Math.max(1, (2 * tol * yawDamping(V)) / (Math.abs(yaw) * Tc)))
      : MAX_STROKES_PER_SIDE;
    return { perSide, duty: perSide / (perSide + p.switchLossStrokes) };
  }

  function simulate(params) {
    const p = Object.assign({}, DEFAULTS, params);
    const k = HULL_K * Math.pow(p.massKg / MASS_REF, 2 / 3);  // drag ∝ wetted area
    // Boat speed: mean thrust falls as the boat speeds up (less slip), so
    // bisect for thrust (less the time lost switching) = hull drag.
    const Tc0 = 60 / p.rateSpm;
    let lo = 0, hi = 8;
    for (let i = 0; i < 40; i++) {
      const V = (lo + hi) / 2;
      const m = means(p, V, k);
      const sw = switching(p, V, m.yaw, Tc0);
      if (sw.duty * m.thrust > k * V * V + m.leeway + m.surge) lo = V; else hi = V;
    }
    const V = (lo + hi) / 2;
    const { samples, Tc } = drive(p, V);
    const sum = (f) => samples.reduce((a, s) => a + f(s) * s.dt, 0);
    const yawMean = sum((s) => s.yaw) / Tc;
    const sw = switching(p, V, yawMean, Tc);
    // Averages over a whole side, switches included.
    const paddler = sw.duty * sum((s) => s.paddler) / Tc;
    const useful = sw.duty * sum((s) => s.useful) / Tc;
    const slip = sw.duty * sum((s) => s.slip) / Tc;
    const imp2 = sum((s) => dot(s.thrust, s.thrust)) || 1;
    const share = (i) => sum((s) => s.thrust[i] * s.thrust[i]) / imp2;
    const hull = k * V * V * V;
    const leeway = leewayDrag(samples, Tc, V, p.massKg) * V;
    const surge = surgeDrag(samples, Tc, V, p.massKg, k) * V;
    const peakForce = Math.max(...samples.map((s) => s.F));
    const Td0 = samples[samples.length - 1].t;
    const pd = Math.max(1e-9, paddler);
    const base = hull / pd;  // efficiency before the vertical push

    // At an effort of `watts` (and your stroke rate): forces scale as
    // (watts / paddler)^(2/3). The vertical push changes hull drag by
    // eps = sinking + bounce (a fraction of k·V³), which the same propulsion
    // pays for, so speed ∝ (1 + eps)^(-1/3).
    const cache = new Map();
    function at(watts) {
      const key = Math.round(watts * 100);
      if (cache.has(key)) return cache.get(key);
      const scale = Math.pow(watts / pd, 2 / 3);
      const Vw = V * Math.cbrt(watts / pd);
      const h = heave(samples, Td0, 60 / p.rateSpm, scale, p.massKg);
      const weight = p.massKg * G;
      const sink = Math.pow(Math.max(0.05, 1 - h.meanLift / weight), 2 / 3) - 1;
      const bounce = h.power / (k * Vw * Vw * Vw);
      const eps = sink + bounce, f = 1 / (1 + eps);
      const r = {
        speed: Vw * Math.cbrt(f),
        efficiency: base * f,
        // Shares of your power. They add to 1; `sink` is negative (a saving)
        // when the stroke lifts the boat.
        shares: { hull: base * f, slip: slip / pd, leeway: leeway / pd, surge: surge / pd,
                  sink: base * sink * f, bounce: base * bounce * f },
        meanLift: h.meanLift,            // N, + lifts the boat
        sinkMm: -h.meanLift / (RHO * G * WATERPLANE) * 1000,  // + deeper
        bounceMm: h.amplitude * 1000,
        hullK: k * (1 + eps),            // effective drag constant at this effort
      };
      cache.set(key, r);
      return r;
    }
    const e = at(p.effortW);
    return {
      params: p, speed: V, cycle: Tc,
      paddlerPower: paddler, usefulPower: useful, slipPower: slip, hullPower: hull,
      leewayPower: leeway, surgePower: surge,
      // Share of your power that moves the boat at the comparison effort,
      // after slip, sideways, surge and vertical effects, including time
      // lost switching sides.
      efficiency: e.efficiency,
      baseEfficiency: base,
      at,
      strokesPerSide: sw.perSide,
      switchDuty: sw.duty,
      forceShare: { forward: share(0), sideways: share(1), vertical: share(2) },
      yawMoment: yawMean,  // N·m; > 0 turns the bow away from the paddle side
      peakForce,
      speedAt: (watts) => at(watts).speed,
      // Turning moment scales with speed², i.e. with effort^(2/3).
      yawAt: (watts) => yawMean * Math.pow(watts / pd, 2 / 3),
      samples,
    };
  }

  // Speed through the water at `watts` with wind and current. Air drag
  // ½ρ·CdA·u² on paddler and boat, at the air speed past them u = ground
  // speed + headwind; still-air drag is already part of HULL_K, so only the
  // change counts. The paddle drives the same share of power into overcoming
  // resistance as in still conditions.
  function waterSpeed(r, watts, headwind, current) {
    const e = r.at(watts), k = e.hullK, budget = k * Math.pow(e.speed, 3);
    const air = (V) => { const u = V + current + headwind; return 0.5 * RHO_AIR * CDA_AIR * (u * Math.abs(u) - V * V); };
    let lo = 0, hi = 3 * e.speed + 10;
    for (let i = 0; i < 60; i++) {
      const V = (lo + hi) / 2;
      if (k * V * V * V + air(V) * V < budget) lo = V; else hi = V;
    }
    return (lo + hi) / 2;
  }

  // Steady race between two strokes: each boat holds its average speed at
  // its own effort over `metres`. env: headwind (m/s, + against you,
  // − tailwind) and current (m/s, + with you). Returns ground speeds, times
  // (Infinity if a boat can't make headway) and the finishing gap.
  function race(a, b, metres, wattsA, wattsB, env = {}) {
    const hw = env.headwind || 0, cur = env.current || 0;
    const wA = waterSpeed(a, wattsA, hw, cur), wB = waterSpeed(b, wattsB, hw, cur);
    const vA = wA + cur, vB = wB + cur;
    const tA = vA > 1e-3 ? metres / vA : Infinity, tB = vB > 1e-3 ? metres / vB : Infinity;
    const first = Math.min(tA, tB);
    return {
      vA, vB, waterA: wA, waterB: wB, tA, tB, metres,
      // > 0: A wins by this many seconds (0 if neither finishes)
      timeGap: isFinite(tA) || isFinite(tB) ? tB - tA : 0,
      distanceGap: isFinite(first) ? (Math.max(0, vA) - Math.max(0, vB)) * first : 0,  // m apart as the winner finishes
    };
  }

  const api = { simulate, race, pose, blade, DEFAULTS, PADDLE, BOAT_HALF_BEAM, DRIVE_FRACTION, MAX_STROKES_PER_SIDE };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.StrokeSim = api;
})(typeof window !== "undefined" ? window : globalThis);
