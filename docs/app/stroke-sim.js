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
//  - The blade face is square to its motion in the boat, bent by β toward
//    the bow side like PowerModel, and can be pitched (turned about the
//    shaft) by pitchDeg. Through the water the flow meets it at an angle α,
//    because the boat is moving too. A flat plate's force stays normal to
//    its face; at an angle it is ½ρ·Cd·A·|v|²·sin α (the plate's "sine law",
//    Hoerner, Fluid-Dynamic Drag): the part across the flow is lift. With
//    bladeLift off, only the flow's normal component counts, |v|²·sin²α
//    (drag only), as a stricter bound. |v| is the blade's speed through the
//    water (its motion in the boat plus the boat's own) and only the
//    immersed part of the blade pushes.
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
//    drag k·V² plus leeway drag. Then paddler power = moving the boat
//    (k·V³) + leeway + slip (power into the water), exactly. With quadratic
//    drag on blade and hull, these shares do not depend on effort: every
//    speed and force scales together.
//  - Vertical push. A blade angled forward at the catch pushes water down
//    and lifts the boat; angled back at the exit it lifts water and pulls
//    the boat down. The average over the stroke changes how deep the boat
//    sits: drag scales with wetted area, about displacement^(2/3). The
//    swing in vertical force bounces the boat on its waterplane (a spring,
//    ρ·g·area) and the bouncing makes waves, which cost power.
//  - Surge, sinking and bounce depend on the size of each pulse against the
//    boat's mass and weight, so they are worked out at the comparison
//    effort and your actual stroke rate: the drive's force shape, scaled to
//    that effort, played over 60/rate seconds. A slower rate at the same
//    power means bigger, further-apart pulses, so more surge (The Science
//    of Paddling, Parts 25 and 33).
//  - Efficiency = moving the boat / paddler power, where moving the boat is
//    k·V³ for the boat at its own weight with no vertical push. A stroke
//    that lifts the boat can earn a little back.

(function (root) {
  "use strict";
  const RHO = 1000, CD = 1.6, BLADE_AREA = 0.095;  // ~20"×8.5" marathon blade
  // Hull drag k·V² (still air included). A coast-down test measured k = 4.26 by
  // coasting a USCA C-1 (18'6", ~109 kg aboard) at mostly under 2.3 m/s
  // (The Science of Paddling, Part 14). Scaled to 90 kg by wetted area, that
  // is 3.75; wave drag grows quickly toward hull speed (~3 m/s for 5.6 m,
  // Part 4), adding roughly 1 more at race pace (2.5 m/s).
  const HULL_K = 4.9;            // N/(m/s)², C1 marathon canoe at 90 kg
  const DRIVE_FRACTION = 0.5;    // default share of the stroke cycle in the drive
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
  // Side-on air drag area (paddler + freeboard, m²), and how far the wind's
  // push sits aft of the hull's pivot underway. The pivot moves forward when
  // a hull is moving (The Science of Paddling, Part 37), so the stern
  // blows downwind and the bow turns into the wind.
  const CDA_SIDE = 1.1, WIND_ARM = 0.3;
  // Wave-making share of hull drag at race pace (2.5 m/s), rising as V⁶
  // toward hull speed; only it changes in shallow water.
  const WAVE_SHARE = 0.23, WAVE_VREF = 2.5, SHALLOW_MAX = 5;

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
    pitchDeg: 0,          // blade turned about the shaft; + = face turned out, away from the hull
    bladeLift: 1,         // 1: flat-plate force with lift; 0: drag only
    rateSpm: 60,
    driveFraction: DRIVE_FRACTION,  // share of each stroke spent in the drive
    massKg: 90,                // paddler + boat
    effortW: 150,              // effort the effort-dependent results are for
    switchHeadingDeg: 3,       // switch sides when this far off course
    switchLossStrokes: 0.3,    // time lost per switch, in strokes
    switchStrokes: 0,          // fixed strokes per side; 0 = switch by heading
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
    let ey = unit(sub(faceDir, mul(P.d, dot(faceDir, P.d))));  // ⊥ shaft
    if (p.pitchDeg) {   // turn the face about the shaft; + turns it away from the hull
      const q = rad(p.pitchDeg), side = cross(P.d, ey);
      ey = unit(sub(mul(ey, Math.cos(q)), mul(side, Math.sin(q))));
    }
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
    const Tc = 60 / p.rateSpm, Td = Tc * p.driveFraction, dt = Td / steps;
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
      const vn = dot(vWater, B.n), vmag = norm(vWater);
      // Normal force on the power face: sine law (with lift) or drag only.
      const F = vn > 0 ? k * B.wet * vn * (p.bladeLift ? vmag : vn) : 0;
      // Of that force, the part across the flow is lift: cos α of it.
      const lift = vn > 0 && vmag > 0 ? F * Math.sqrt(Math.max(0, 1 - (vn / vmag) * (vn / vmag))) : 0;
      const thrust = mul(B.n, -F);                             // on the boat
      out.push({
        u, t: u * Td, dt: i === 0 || i === steps ? dt / 2 : dt,
        P, B, F, thrust, lift,
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

  // Forward or vertical push (component k) at time t of a stroke lasting
  // Tc with a drive of Td: the sampled drive (Td0 long) stretched to Td,
  // scaled by `scale`.
  const replay = (samples, Td0, Td, scale, k) => (t) => (t > Td ? 0 : scale * pulse(samples, Td0, (t / Td) * Td0, k));

  // Surge: the boat's speed V + δ(t) under the pulsed push f(t), linearised
  // about V (m·δ' = f − f̄ − 2kV·δ), periodic over Tc. The extra hull drag
  // power averages k·((V+δ)³ − V³). Returns that power (W) and the speed
  // swing (m/s, peak to peak).
  function surge(f, Tc, V, mass, k) {
    if (V <= 0) return { power: 0, swing: 0 };
    const m = mass * (1 + SURGE_ADDED), n = 1000, dt = Tc / n, b = 2 * k * V;
    let mean = 0;
    for (let i = 0; i < n; i++) mean += f(i * dt) / n;
    const cycle = (d0) => {
      const out = [];
      let d = d0;
      for (let i = 0; i < n; i++) { d += ((f(i * dt) - mean - b * d) / m) * dt; out.push(d); }
      return out;
    };
    // Linear: the periodic start d0 satisfies d0 = d0·decay + d(Tc | d0 = 0).
    const decay = Math.pow(1 - (b / m) * dt, n);
    const last = cycle(cycle(0)[n - 1] / (1 - decay));
    const avg = last.reduce((a, x) => a + x, 0) / n;
    let extra = 0;
    for (const x of last) { const v = V + x - avg; extra += k * (v * v * v - V * V * V) / n; }
    return { power: Math.max(0, extra), swing: Math.max(...last) - Math.min(...last) };
  }

  // Heave under the vertical push, at stroke period Tc (s), with the force
  // scaled by `scale`. Spring ρ·g·waterplane, mass with added water,
  // damping HEAVE_ZETA of critical. Returns the wave power (W), the bounce
  // amplitude (m) and the mean vertical push (N, + lifts the boat).
  function heave(fz, Tc, mass) {
    const K = RHO * G * WATERPLANE, m = mass * (1 + HEAVE_ADDED);
    const c = 2 * HEAVE_ZETA * Math.sqrt(K * m);
    const n = 1000, dt = Tc / n;
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

  // Mean forward thrust, mean yaw moment and leeway drag at speed V.
  function means(p, V, k, steps = 120) {
    const { samples, Tc } = drive(p, V, steps);
    let thrust = 0, yaw = 0;
    for (const s of samples) { thrust += s.thrust[0] * s.dt; yaw += s.yaw * s.dt; }
    return { thrust: thrust / Tc, yaw: yaw / Tc, leeway: leewayDrag(samples, Tc, V, p.massKg) };
  }
  const yawDamping = (V) => C_TRACK * RHO * V * Math.pow(HULL_L, 3) * HULL_DRAFT / 12;

  // Hit and switch. A mean yaw moment M turns the boat at M / c rad/s; the
  // heading swings from +tol to −tol between switches, so a side lasts
  // 2·tol·c / (M·Tc) strokes. Returns strokes per side and the share of
  // time spent paddling (the rest is switching).
  //
  // Or the paddler takes a fixed switchStrokes per side, and the heading
  // wanders as far as it must: ±N·Tc·M / (2c). Either way the boat zigzags
  // about the course, and its progress along it is V·cos ψ, on average
  // V·(1 − ψ²/6) for a heading that swings evenly through ±ψ.
  function switching(p, V, yaw, Tc) {
    const turnPerStroke = Math.abs(yaw) * Tc / yawDamping(V);  // rad per stroke
    let perSide;
    if (p.switchStrokes > 0) perSide = Math.min(MAX_STROKES_PER_SIDE, Math.max(1, Math.round(p.switchStrokes)));
    else perSide = turnPerStroke > 1e-12
      ? Math.min(MAX_STROKES_PER_SIDE, Math.max(1, (2 * rad(p.switchHeadingDeg)) / turnPerStroke))
      : MAX_STROKES_PER_SIDE;
    const wander = Math.min(Math.PI / 2, (perSide * turnPerStroke) / 2);
    return { perSide, duty: perSide / (perSide + p.switchLossStrokes), wander, course: 1 - (wander * wander) / 6, turnPerStroke };
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
      if (sw.duty * m.thrust > k * V * V + m.leeway) lo = V; else hi = V;
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
    const peakForce = Math.max(...samples.map((s) => s.F));
    const Td0 = samples[samples.length - 1].t;
    const pd = Math.max(1e-9, paddler);
    const base = hull / pd;  // efficiency before the vertical push

    // At an effort of `watts` (and your stroke rate): forces scale as
    // (watts / paddler)^(2/3). Surge, sinking and bounce add to hull drag by
    // eps (a fraction of k·V³), which the same propulsion pays for, so
    // speed ∝ (1 + eps)^(-1/3).
    const cache = new Map();
    function at(watts) {
      const key = Math.round(watts * 100);
      if (cache.has(key)) return cache.get(key);
      const scale = Math.pow(watts / pd, 2 / 3);
      const Vw = V * Math.cbrt(watts / pd);
      const Tc1 = 60 / p.rateSpm, Td1 = Tc1 * p.driveFraction;
      const h = heave(replay(samples, Td0, Td1, scale, 2), Tc1, p.massKg);
      const sg = surge(replay(samples, Td0, Td1, scale * sw.duty, 0), Tc1, Vw, p.massKg, k);
      const weight = p.massKg * G;
      const sink = Math.pow(Math.max(0.05, 1 - h.meanLift / weight), 2 / 3) - 1;
      const bounce = h.power / (k * Vw * Vw * Vw);
      const surgeFrac = sg.power / (k * Vw * Vw * Vw);
      const eps = sink + bounce + surgeFrac, f = 1 / (1 + eps);
      const r = {
        speed: Vw * Math.cbrt(f) * sw.course,     // made good along the course
        waterSpeed: Vw * Math.cbrt(f),            // through the water
        efficiency: base * f,
        // Shares of your power. They add to 1; `sink` is negative (a saving)
        // when the stroke lifts the boat.
        shares: { hull: base * f, slip: slip / pd, leeway: leeway / pd, surge: base * surgeFrac * f,
                  sink: base * sink * f, bounce: base * bounce * f },
        meanLift: h.meanLift,            // N, + lifts the boat
        sinkMm: -h.meanLift / (RHO * G * WATERPLANE) * 1000,  // + deeper
        bounceMm: h.amplitude * 1000,
        surgeSwing: sg.swing / Vw,        // speed swing over a stroke, peak to peak / mean
        hullK: k * (1 + eps),            // effective drag constant at this effort
      };
      cache.set(key, r);
      return r;
    }
    const e = at(p.effortW);
    return {
      params: p, speed: V, cycle: Tc,
      paddlerPower: paddler, usefulPower: useful, slipPower: slip, hullPower: hull,
      leewayPower: leeway,
      // Share of your power that moves the boat at the comparison effort,
      // after slip, sideways, surge and vertical effects, including time
      // lost switching sides.
      efficiency: e.efficiency,
      baseEfficiency: base,
      at,
      strokesPerSide: sw.perSide,
      headingWanderDeg: sw.wander * 180 / Math.PI,   // ± about the course
      courseFactor: sw.course,
      turnPerStroke: sw.turnPerStroke,               // rad of heading per stroke
      switchDuty: sw.duty,
      forceShare: { forward: share(0), sideways: share(1), vertical: share(2) },
      // Share of the blade force that is lift (across the flow), over the drive.
      liftShare: sum((s) => s.lift) / Math.max(1e-9, sum((s) => s.F)),
      yawMoment: yawMean,  // N·m; > 0 turns the bow away from the paddle side
      peakForce,
      speedAt: (watts) => at(watts).speed,
      // Turning moment scales with speed², i.e. with effort^(2/3).
      yawAt: (watts) => yawMean * Math.pow(watts / pd, 2 / 3),
      samples,
    };
  }

  // Shallow water (The Science of Paddling, Part 4): the boat's own wave, wavelength
  // λ = 2πV²/g, runs slower when the depth D is under about λ/2, as
  // c = √(gλ/2π · tanh(2πD/λ)). The hull then makes the wave it would make
  // in deep water at V / √tanh(gD/V²) (Schlichting's intermediate speed), so
  // the wave drag (∝ V⁶ near hull speed) grows by tanh(gD/V²)^-3, capped at
  // SHALLOW_MAX. Under 2 ft at race pace drag "skyrockets".
  function shallowFactor(V, depth) {
    if (!(depth > 0) || !isFinite(depth) || V <= 0) return 1;
    return Math.min(SHALLOW_MAX, Math.pow(Math.tanh((G * depth) / (V * V)), -3));
  }

  // Speed through the water at `watts` in the conditions of one leg:
  //  - air drag ½ρ·CdA·|u|·u on paddler and boat, u = air velocity past the
  //    boat (ground speed + headwind ahead, crosswind across); still-air
  //    drag is already part of HULL_K, so only the change counts;
  //  - the crosswind's side push Y is carried by the hull crabbing (leeway),
  //    costing induced drag Y²/(½ρV²π(2d)²) as for the stroke's own push;
  //  - its turning moment Y·WIND_ARM adds to the stroke's on one side and
  //    takes from it on the other, so you take more strokes on the windward
  //    side and fewer on the lee side. If the wind out-turns your stroke,
  //    you stay on the windward side and steer out the rest, which the
  //    hull pays for as more side force at the stern;
  //  - shallow water raises the wave drag (shallowFactor).
  // The paddle drives the same share of power into overcoming resistance as
  // in still conditions; time lost switching changes with the wind.
  function legSpeed(r, watts, c) {
    const e = r.at(watts), k = e.hullK, budget = k * Math.pow(e.waterSpeed, 3);
    const loss = r.params.switchLossStrokes, fixed = r.params.switchStrokes > 0;
    const M = Math.abs(r.yawAt(watts));
    const tol = rad(r.params.switchHeadingDeg);
    const head = c.windSpeed * Math.cos(rad(c.windAngle)), cross = Math.abs(c.windSpeed * Math.sin(rad(c.windAngle)));
    const qPlan = (V) => 0.5 * RHO * V * V * Math.PI * Math.pow(2 * HULL_DRAFT, 2);
    // Heading turned per stroke under a moment m at speed V: the stroke's own
    // figure (scale-free) scaled by m/M, and by 1/V for the hull's damping.
    const turn = (m, V) => r.turnPerStroke * (M > 1e-9 ? m / M : 0) * (e.waterSpeed / Math.max(1e-6, V));
    const clampN = (n) => Math.min(MAX_STROKES_PER_SIDE, Math.max(1, n));
    // Switching with the wind's turning moment W (bow into the wind) added to
    // the stroke's on the lee side and taken from it on the windward side.
    //  - by heading: each side swings ±tol, so it lasts 2·tol / turn strokes;
    //  - fixed count N: to hold the course over a windward + lee pair the
    //    turns must cancel, Nw·(M − W) = Nl·(M + W), with Nw + Nl = 2N. The
    //    heading then wanders ±Nw·turn(M − W)/2.
    // Returns strokes each side, the share of time paddling, the wander (rad)
    // and the progress factor 1 − ψ²/6.
    const switchIn = (V, W) => {
      let windward, lee, wander;
      if (fixed) {
        const N = r.strokesPerSide;
        windward = (N * (M + W)) / Math.max(1e-9, M); lee = (N * (M - W)) / Math.max(1e-9, M);
        wander = (windward * turn(M - W, V)) / 2;
      } else {
        const per = (m) => { const t = turn(m, V); return t > 1e-12 ? clampN((2 * tol) / t) : MAX_STROKES_PER_SIDE; };
        windward = per(M - W); lee = per(M + W);
        wander = Math.max(windward * turn(M - W, V), lee * turn(M + W, V)) / 2;
      }
      wander = Math.min(Math.PI / 2, wander);
      return { windward, lee, duty: (windward + lee) / (windward + lee + 2 * loss), wander, course: 1 - (wander * wander) / 6 };
    };
    const calm = switchIn(e.waterSpeed, 0), calmDuty = calm.duty;
    const state = (V) => {
      const ux = V + c.current + head, uy = cross, u = Math.hypot(ux, uy);
      const airX = 0.5 * RHO_AIR * CDA_AIR * (u * ux - V * V);
      const side = 0.5 * RHO_AIR * CDA_SIDE * u * uy;
      const W = side * WIND_ARM;
      let duty, windward, lee, wander, course, steer = 0;
      if (W < M) ({ windward, lee, duty, wander, course } = switchIn(V, W));
      else {           // can't hold course by switching alone
        windward = Infinity; lee = 0; duty = 1; wander = 0; course = 1;
        steer = (W - M) / (HULL_L / 2);
      }
      const q = qPlan(V);
      // Induced drag can't sensibly exceed the side force itself (a 45° crab).
      const induced = q > 0 ? Math.min(side + steer, (side * side + steer * steer) / q) : side + steer;
      const wave = WAVE_SHARE * k * WAVE_VREF * WAVE_VREF * Math.pow(V / WAVE_VREF, 6);
      const shallow = wave * (shallowFactor(V, c.depth) - 1);
      const leeway = V > 0 ? side / (0.5 * RHO * V * V * HULL_L * HULL_DRAFT * HULL_LIFT_SLOPE) : 0;
      return { R: k * V * V + airX + induced + shallow, duty, windward, lee, steer: steer > 0,
               wanderDeg: wander * 180 / Math.PI, course,
               leewayDeg: Math.min(45, leeway * 180 / Math.PI), shallow: shallowFactor(V, c.depth), sideForce: side };
    };
    let lo = 0, hi = 3 * e.waterSpeed + 10;
    for (let i = 0; i < 60; i++) {
      const V = (lo + hi) / 2, st = state(V);
      if (st.R * V < budget * st.duty / calmDuty) lo = V; else hi = V;
    }
    const V = (lo + hi) / 2;
    // `water` is progress along the course through the water: the zigzag
    // about the course (switching) takes its share.
    const st = state(V);
    return Object.assign({ water: V * st.course, waterRaw: V }, st);
  }
  // Back-compatible: speed through the water with a headwind and current.
  function waterSpeed(r, watts, headwind, current) {
    return legSpeed(r, watts, { windSpeed: Math.abs(headwind), windAngle: headwind < 0 ? 180 : 0, current, depth: Infinity }).water;
  }

  // ---------------------------------------------------------------- river
  // A river (The Science of Paddling, Parts 18 and 20): current and
  // depth are greatest mid-channel and fall to the banks,
  //   straight: shape(y) = 1 − (1 − 2y/W)², y from the nearer bank;
  //   bend: deep and fast on the outside, a shallow slack bar on the inside,
  //     shape(y) = (y/W)(2 − y/W) / 0.75, y from the inside bank
  //     (the same as a straight in mid-channel).
  // Current = mid-channel current × shape; depth = mid-channel depth × shape.
  // Bends turn BEND_ANGLE each, with centreline radius BEND_RADIUS × width;
  // a line closer to the inside is shorter (radius − offset). A line `u`
  // runs from mid-channel (0) to RIVER_EDGE m off the bank (1): along the
  // nearer bank on straights and the inside of each bend. Bends alternate,
  // so a line off the middle crosses the river between bends (at 45°,
  // through the conditions halfway to mid-channel).
  const RIVER_EDGE = 3, MIN_DEPTH = 0.25, BEND_ANGLE = 90, BEND_RADIUS = 1.5;
  // Portage: carrying the boat at a jog, plus getting out and back in.
  const CARRY_SPEED = 2, PORTAGE_SETUP = 30;   // m/s, s
  const shapeStraight = (y, W) => 1 - Math.pow(1 - (2 * y) / W, 2);
  const shapeBend = (y, W) => ((y / W) * (2 - y / W)) / 0.75;
  // Distance from the bank for line u.
  const lineY = (u, W) => W / 2 - u * (W / 2 - RIVER_EDGE);
  // Segments (along-course length, path length, current, depth) for one km
  // of river on line u, given mid-channel current and depth.
  function riverSegments(river, u, current, depth) {
    const W = river.width, y = lineY(u, W), per = 1000;
    const turn = rad(Math.max(0, river.bendsDegPerKm || 0));
    const Rc = BEND_RADIUS * W, bendLen = Math.min(0.9 * per, turn * Rc);
    const bends = bendLen > 0 ? (turn * 180 / Math.PI) / BEND_ANGLE : 0;
    const d = (shape) => Math.max(MIN_DEPTH, depth * shape);
    const segs = [];
    const straightLen = per - bendLen;
    // Crossing between bends: across W − 2y of river, at 45°.
    const across = bends > 0 ? Math.max(0, W - 2 * y) : 0;
    const perStraight = bends > 0 ? straightLen / bends : straightLen;
    const along = Math.min(perStraight, across);
    if (across > 0 && along > 0) {
      const sh = shapeStraight((y + W / 2) / 2, W);
      segs.push({ kind: "crossing", len: along * bends, path: Math.hypot(along, across) * bends, current: current * sh, depth: d(sh) });
    }
    const rest = straightLen - along * bends;
    if (rest > 0) {
      const sh = shapeStraight(y, W);
      segs.push({ kind: "straight", len: rest, path: rest, current: current * sh, depth: d(sh) });
    }
    if (bendLen > 0) {
      const sh = shapeBend(y, W);
      segs.push({ kind: "bend", len: bendLen, path: bendLen * (Rc - (W / 2 - y)) / Rc, current: current * sh, depth: d(sh) });
    }
    return segs;
  }
  // Time for `len` m of a leg (c: wind and mid-channel current/depth) on
  // line u, and what the boat met there.
  function riverLeg(r, watts, c, river, u) {
    const segs = riverSegments(river, u, c.current, c.depth);
    let t = 0, path = 0;
    const parts = segs.map((sg) => {
      const st = legSpeed(r, watts, Object.assign({}, c, { current: sg.current, depth: sg.depth }));
      // Current runs along the river; the boat's path may be shorter (inside
      // of a bend) or longer (crossing), and moves along at path/len of it.
      const ground = st.water + sg.current * (sg.len / sg.path);
      const tt = ground > 1e-3 ? sg.path / ground : Infinity;
      t += tt; path += sg.path;
      return Object.assign({}, sg, { t: tt, water: st.water });
    });
    const k = c.len / 1000;
    return { t: t * k, path: path * k, parts, line: u };
  }
  // The fastest line for a leg, by a coarse scan then refinement.
  // n scan steps, then `refine` thirds (a long course uses fewer).
  function bestLine(r, watts, c, river, n = 20, refine = 20) {
    let best = 0, bt = Infinity;
    for (let i = 0; i <= n; i++) {
      const u = i / n, t = riverLeg(r, watts, c, river, u).t;
      if (t < bt) { bt = t; best = u; }
    }
    let lo = Math.max(0, best - 1 / n), hi = Math.min(1, best + 1 / n);
    for (let i = 0; i < refine; i++) {
      const m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
      if (riverLeg(r, watts, c, river, m1).t < riverLeg(r, watts, c, river, m2).t) hi = m2; else lo = m1;
    }
    const u = (lo + hi) / 2;
    return riverLeg(r, watts, c, river, u).t <= bt ? u : best;
  }

  // The course laid out along its length, for replays: segments {d0, d1,
  // v (speed made good), kind, leg, line, current, depth, watts, rate,
  // perSide, loss}. Open water is one piece per leg. A river repeats, once
  // per bend: a straight, the crossing to the next bend's inside, the bend
  // (riverSegments' per-km mix). Bends sit at the same places for every
  // line. Each leg's speeds are scaled so its time matches the leg's own.
  function courseProfile(r, watts, legs, river) {
    const out = [];
    let d = 0;
    legs.forEach((l, li) => {
      const base = { leg: li, watts, rate: r.params.rateSpm, loss: r.params.switchLossStrokes,
                     perSide: isFinite(l.windward) ? (l.windward + l.lee) / 2 : MAX_STROKES_PER_SIDE };
      if (l.section !== undefined) Object.assign(base, { leg: 0, section: l.section, name: l.name });
      const start = d, end = d + l.len;
      if (l.kind === "portage") {
        out.push(Object.assign({ d0: start, d1: end, v: l.v, kind: "portage", line: 0, current: 0, depth: Infinity }, base, { rate: 0, watts: 0 }));
        d = end;
        return;
      }
      if (l.kind === "pond") {
        out.push(Object.assign({ d0: start, d1: end, v: l.v, kind: "pond", line: 0, current: l.current, depth: l.depth }, base));
        d = end;
        return;
      }
      if (!l.river) {
        out.push(Object.assign({ d0: start, d1: end, v: l.v, kind: "open", line: 0, current: 0, depth: Infinity }, base));
        d = end;
        return;
      }
      const parts = l.river.parts, by = (k) => parts.find((p) => p.kind === k);
      const turnDeg = Math.max(0, (l.cfg || river).bendsDegPerKm || 0), bends = turnDeg / BEND_ANGLE;
      const units = [];
      for (const k of ["straight", "crossing", "bend"]) {
        const p = by(k);
        if (p && p.len > 0) units.push({ kind: k, len: bends > 0 ? p.len / bends : p.len, v: p.len / p.t, current: p.current, depth: p.depth });
      }
      const first = out.length;
      while (d < end - 1e-9 && units.length) for (const u of units) {
        if (d >= end - 1e-9) break;
        const d1 = Math.min(end, d + u.len);
        out.push(Object.assign({ d0: d, d1, v: u.v, kind: u.kind, line: l.river.line, current: u.current, depth: u.depth }, base));
        d = d1;
      }
      // Match the leg's own time, so replays and results agree.
      const tp = out.slice(first).reduce((a, g) => a + (g.v > 0 ? (g.d1 - g.d0) / g.v : Infinity), 0);
      if (isFinite(tp) && isFinite(l.t) && l.t > 0) for (let i = first; i < out.length; i++) out[i].v *= tp / l.t;
      d = end;
    });
    return out;
  }

  // Steady race between two strokes: each boat holds its average speed at
  // its own effort. env: headwind (m/s, + against you, − tailwind), current
  // (m/s, + with you) and course: "point" (one way) or "outback" (out to a
  // turn at half distance and back, so the wind and current reverse on the
  // way home; the turn itself isn't timed). Time lost against them is never
  // fully won back with them (The Science of Paddling, Part 7).
  // Returns times (Infinity if a boat can't make headway), average ground
  // speeds, the finishing gap and each boat's distance at time t.
  function race(a, b, metres, wattsA, wattsB, env = {}) {
    // Wind: windSpeed (m/s) from windAngle (deg: 0 = dead ahead, 90 = from
    // the side, 180 = from behind), or a signed headwind (m/s).
    let ws = env.windSpeed || 0, wa = env.windAngle || 0;
    if (env.headwind) { ws = Math.abs(env.headwind); wa = env.headwind > 0 ? 0 : 180; }
    const cur = env.current || 0, depth = env.depth === undefined ? Infinity : env.depth;
    const leg = (len, back) => ({ len, windSpeed: ws, windAngle: back ? 180 - wa : wa, current: back ? -cur : cur, depth });
    let legs = env.course === "outback" ? [leg(metres / 2, false), leg(metres / 2, true)] : [leg(metres, false)];
    // env.sections: a real course, piece by piece: { len (m), kind: "river" |
    // "pond" | "portage", heading (deg, the way you're going: 0 = north),
    // current (m/s, + with you), depth (m), width (m), bendsDegPerKm, section
    // (which editable section it belongs to), name }. env.windFrom (deg,
    // compass) and windSpeed: the wind's angle to the boat changes as the
    // course turns. A portage is carried, not paddled.
    if (env.sections) {
      const from = env.windFrom || 0;
      legs = env.sections.map((sc) => ({
        len: sc.len, kind: sc.kind || "river", section: sc.section, name: sc.name,
        windSpeed: ws, windAngle: Math.abs((((from - (sc.heading || 0)) % 360) + 540) % 360 - 180),
        current: sc.current || 0, depth: sc.depth === undefined ? Infinity : sc.depth,
        cfg: (sc.kind || "river") === "river" ? { width: sc.width, bendsDegPerKm: sc.bendsDegPerKm || 0 } : null,
      }));
      metres = legs.reduce((a, l) => a + l.len, 0);
    }
    // env.river: { width (m), bendsDegPerKm, lineA, lineB } with current and
    // depth taken as mid-channel; each line is 0 (middle) … 1 (edges) or
    // "best" (fastest for each leg).
    const river = env.river && env.river.width > 2 * RIVER_EDGE ? env.river : null;
    const boat = (r, watts, line) => {
      // Best line once per editable section, on its average bends.
      const best = new Map();
      const bestFor = (l) => {
        if (!best.has(l.section)) {
          const mine = legs.filter((m) => m.section === l.section && m.cfg);
          const len = mine.reduce((a, m) => a + m.len, 0) || 1;
          const bends = mine.reduce((a, m) => a + m.cfg.bendsDegPerKm * m.len, 0) / len;
          best.set(l.section, bestLine(r, watts, l, Object.assign({}, l.cfg, { bendsDegPerKm: bends }), 10, 10));
        }
        return best.get(l.section);
      };
      const ls = legs.map((l) => {
        if (l.kind === "portage") {
          const t = l.len / CARRY_SPEED + PORTAGE_SETUP;
          return { len: l.len, v: l.len / t, t, kind: "portage", section: l.section, name: l.name, water: 0 };
        }
        const st = legSpeed(r, watts, l);
        const tag = { kind: l.kind, section: l.section, name: l.name, cfg: l.cfg, current: l.current, depth: l.depth };
        if (env.sections && !l.cfg) {
          const v = st.water + l.current;
          return Object.assign(st, tag, { len: l.len, v, t: v > 1e-3 ? l.len / v : Infinity });
        }
        if (env.sections) {
          const u = line === "best" ? bestFor(l) : Math.min(1, Math.max(0, +line || 0));
          const rl = riverLeg(r, watts, l, l.cfg, u);
          return Object.assign(st, tag, { len: l.len, v: isFinite(rl.t) ? l.len / rl.t : 0, t: rl.t, river: rl });
        }
        if (!river) {
          const v = st.water + l.current;
          return Object.assign(st, { len: l.len, v, t: v > 1e-3 ? l.len / v : Infinity });
        }
        const u = line === "best" ? bestLine(r, watts, l, river) : Math.min(1, Math.max(0, +line || 0));
        const rl = riverLeg(r, watts, l, river, u);
        return Object.assign(st, { len: l.len, v: isFinite(rl.t) ? l.len / rl.t : 0, t: rl.t, river: rl });
      });
      const t = ls.reduce((s, l) => s + l.t, 0);
      const profile = courseProfile(r, watts, ls, river);
      const pos = (time) => {        // distance covered by `time`
        let d = 0, left = time;
        for (const g of profile) {
          if (left <= 0 || !(g.v > 0)) break;
          const dt = Math.min(left, (g.d1 - g.d0) / g.v);
          d += g.v * dt; left -= dt;
        }
        return Math.min(metres, d);
      };
      return { legs: ls, t, v: isFinite(t) ? metres / t : 0, water: (ls.find((l) => l.kind !== "portage") || ls[0]).water, pos, profile };
    };
    const lineOf = (x) => (env.sections ? (env.lines || {})[x] : river && river[`line${x}`]);
    const A = boat(a, wattsA, lineOf("A")), B = boat(b, wattsB, lineOf("B"));
    const first = Math.min(A.t, B.t);
    return {
      vA: A.v, vB: B.v, waterA: A.water, waterB: B.water, tA: A.t, tB: B.t, metres,
      legsA: A.legs, legsB: B.legs, posA: A.pos, posB: B.pos, profileA: A.profile, profileB: B.profile,
      // > 0: A wins by this many seconds (0 if neither finishes)
      timeGap: isFinite(A.t) || isFinite(B.t) ? B.t - A.t : 0,
      // m apart as the winner finishes
      distanceGap: isFinite(first) ? A.pos(first) - B.pos(first) : 0,
    };
  }

  // Bumped when the page needs something new from the engine, so a page
  // that meets an old, cached engine can say so instead of breaking.
  const API_LEVEL = 26;
  const api = { API_LEVEL, simulate, race, shallowFactor, riverSegments, pose, blade, DEFAULTS, PADDLE, BOAT_HALF_BEAM, DRIVE_FRACTION, MAX_STROKES_PER_SIDE };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.StrokeSim = api;
})(typeof window !== "undefined" ? window : globalThis);
