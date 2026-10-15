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
//  - Pushing off the boat's centreline turns it. The turn is cancelled by
//    a J-stroke behind the paddler (STEER_ARM aft), using the blade as a
//    rudder with lift/drag STEER_LD: correcting a mean yaw moment M costs
//    extra drag M / (STEER_ARM · STEER_LD).
//  - Boat speed is where the stroke's mean forward thrust balances hull
//    drag k·V² plus that steering drag. Then paddler power = moving the
//    boat (k·V³) + steering + slip (power into the water), exactly, and
//    efficiency = moving the boat / paddler power.
//  - With quadratic drag on blade and hull, efficiency does not depend on
//    stroke rate or effort: every speed and force scales together. It is a
//    measure of technique and geometry alone.

(function (root) {
  "use strict";
  const RHO = 1000, CD = 1.6, BLADE_AREA = 0.095;  // ~20"×8.5" marathon blade
  const HULL_K = 5.5;            // N/(m/s)², C1 marathon canoe
  const DRIVE_FRACTION = 0.5;    // of the stroke cycle
  // Steering defaults; adjustable as steerArmM / steerLD.
  const STEER_ARM = 0.6;         // m aft of the boat's centre, J-stroke
  const STEER_LD = 4;            // paddle blade used as a rudder
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
    steerArmM: STEER_ARM,
    steerLD: STEER_LD,
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

  // Mean forward thrust and mean yaw moment at boat speed V.
  function means(p, V, steps = 120) {
    const { samples, Tc } = drive(p, V, steps);
    let thrust = 0, yaw = 0;
    for (const s of samples) { thrust += s.thrust[0] * s.dt; yaw += s.yaw * s.dt; }
    return { thrust: thrust / Tc, yaw: yaw / Tc };
  }
  const steeringDrag = (p, yaw) => Math.abs(yaw) / (p.steerArmM * p.steerLD);

  function simulate(params) {
    const p = Object.assign({}, DEFAULTS, params);
    // Boat speed: mean thrust falls as the boat speeds up (less slip), so
    // bisect for thrust = hull drag + steering drag.
    let lo = 0, hi = 8;
    for (let i = 0; i < 40; i++) {
      const V = (lo + hi) / 2;
      const m = means(p, V);
      if (m.thrust > HULL_K * V * V + steeringDrag(p, m.yaw)) lo = V; else hi = V;
    }
    const V = (lo + hi) / 2;
    const { samples, Tc } = drive(p, V);
    const sum = (f) => samples.reduce((a, s) => a + f(s) * s.dt, 0);
    const paddler = sum((s) => s.paddler) / Tc;
    const useful = sum((s) => s.useful) / Tc;
    const slip = sum((s) => s.slip) / Tc;
    const imp2 = sum((s) => dot(s.thrust, s.thrust)) || 1;
    const share = (k) => sum((s) => s.thrust[k] * s.thrust[k]) / imp2;
    const yawMean = sum((s) => s.yaw) / Tc;
    const hull = HULL_K * V * V * V;
    const steering = steeringDrag(p, yawMean) * V;
    const peakForce = Math.max(...samples.map((s) => s.F));
    const atEffort = (watts) => V * Math.cbrt(watts / Math.max(1e-9, paddler));
    return {
      params: p, speed: V, cycle: Tc,
      paddlerPower: paddler, usefulPower: useful, slipPower: slip,
      hullPower: hull, steeringPower: steering,
      // Share of your power that moves the boat, after slip and steering.
      efficiency: paddler > 0 ? hull / paddler : 0,
      // Before the steering correction: useful (thrust · speed) / paddler.
      propulsiveEfficiency: paddler > 0 ? useful / paddler : 0,
      forceShare: { forward: share(0), sideways: share(1), vertical: share(2) },
      yawMoment: yawMean,  // N·m; > 0 turns the bow away from the paddle side
      peakForce,
      speedAt: atEffort,
      // Turning moment scales with speed², i.e. with effort^(2/3).
      yawAt: (watts) => yawMean * Math.pow(watts / Math.max(1e-9, paddler), 2 / 3),
      samples,
    };
  }

  const api = { simulate, pose, blade, DEFAULTS, PADDLE, BOAT_HALF_BEAM, DRIVE_FRACTION, STEER_ARM, STEER_LD };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.StrokeSim = api;
})(typeof window !== "undefined" ? window : globalThis);
