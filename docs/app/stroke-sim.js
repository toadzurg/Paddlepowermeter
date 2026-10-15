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
//  - Boat speed is where mean thrust (counting the switches) balances hull
//    drag k·V² plus that leeway drag. Then paddler power = moving the boat
//    (k·V³) + leeway drag + slip (power into the water), exactly;
//    efficiency = moving the boat / paddler power.
//  - With quadratic drag on blade and hull, efficiency does not depend on
//    stroke rate or effort: every speed and force scales together. It is a
//    measure of technique and geometry alone.

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
  function leewayDrag(samples, Tc, V) {
    if (V <= 0) return 0;
    const b = 0.5 * RHO * V * HULL_L * HULL_DRAFT * HULL_LIFT_SLOPE;
    const qpb2 = 0.5 * RHO * V * V * Math.PI * Math.pow(2 * HULL_DRAFT, 2);
    const n = 400, dt = Tc / n, Td = samples[samples.length - 1].t;
    const fy = (t) => {                       // sideways push at cycle time t
      if (t > Td) return 0;
      const i = Math.min(samples.length - 1, Math.round((t / Td) * (samples.length - 1)));
      return samples[i].thrust[1];
    };
    let v = 0, sumD = 0;
    const cycles = 6;
    for (let c = 0; c < cycles; c++)
      for (let i = 0; i < n; i++) {
        const t = i * dt, Y = b * v;
        v += ((fy(t) - Y) / SWAY_MASS) * dt;   // explicit Euler; τ = m/b ≫ dt
        if (c === cycles - 1) sumD += (Y * Y) / qpb2;
      }
    return sumD / n;
  }

  // Mean forward thrust, mean yaw moment and leeway drag at boat speed V.
  function means(p, V, steps = 120) {
    const { samples, Tc } = drive(p, V, steps);
    let thrust = 0, yaw = 0;
    for (const s of samples) { thrust += s.thrust[0] * s.dt; yaw += s.yaw * s.dt; }
    return { thrust: thrust / Tc, yaw: yaw / Tc, leeway: leewayDrag(samples, Tc, V) };
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
    // Boat speed: mean thrust falls as the boat speeds up (less slip), so
    // bisect for thrust (less the time lost switching) = hull drag.
    const Tc0 = 60 / p.rateSpm;
    let lo = 0, hi = 8;
    for (let i = 0; i < 40; i++) {
      const V = (lo + hi) / 2;
      const m = means(p, V);
      const sw = switching(p, V, m.yaw, Tc0);
      if (sw.duty * m.thrust > HULL_K * V * V + m.leeway) lo = V; else hi = V;
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
    const share = (k) => sum((s) => s.thrust[k] * s.thrust[k]) / imp2;
    const hull = HULL_K * V * V * V;
    const leeway = leewayDrag(samples, Tc, V) * V;
    const peakForce = Math.max(...samples.map((s) => s.F));
    const atEffort = (watts) => V * Math.cbrt(watts / Math.max(1e-9, paddler));
    return {
      params: p, speed: V, cycle: Tc,
      paddlerPower: paddler, usefulPower: useful, slipPower: slip, hullPower: hull, leewayPower: leeway,
      // Share of your power that moves the boat, after slip, including the
      // effect of time lost switching sides.
      efficiency: paddler > 0 ? hull / paddler : 0,
      strokesPerSide: sw.perSide,
      switchDuty: sw.duty,
      forceShare: { forward: share(0), sideways: share(1), vertical: share(2) },
      yawMoment: yawMean,  // N·m; > 0 turns the bow away from the paddle side
      peakForce,
      speedAt: atEffort,
      // Turning moment scales with speed², i.e. with effort^(2/3).
      yawAt: (watts) => yawMean * Math.pow(watts / Math.max(1e-9, paddler), 2 / 3),
      samples,
    };
  }

  // Steady race between two strokes: each boat holds its average speed at
  // its own effort over `metres`. Returns times and the finishing gap.
  function race(a, b, metres, wattsA, wattsB) {
    const vA = a.speedAt(wattsA), vB = b.speedAt(wattsB);
    const tA = metres / vA, tB = metres / vB;
    const first = Math.min(tA, tB);
    return {
      vA, vB, tA, tB, metres,
      timeGap: tB - tA,                       // > 0: A wins by this many seconds
      distanceGap: (vA - vB) * first,         // m between them as the winner finishes
    };
  }

  const api = { simulate, race, pose, blade, DEFAULTS, PADDLE, BOAT_HALF_BEAM, DRIVE_FRACTION, MAX_STROKES_PER_SIDE };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.StrokeSim = api;
})(typeof window !== "undefined" ? window : globalThis);
