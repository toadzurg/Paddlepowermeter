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
//    loses switchLossS seconds with no propulsion: about the same time at any
//    stroke rate, so it costs more strokes at a higher rate. A stroke that
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
//    (mean of V³ > V̄³). The body's own motion (swing, drop, hinge) adds to
//    the surge and bob: see swingVel and bodyDropAcc.
//  - Boat speed is where mean thrust (counting the switches) balances hull
//    drag k·V² plus leeway drag. Then paddler power = moving the boat
//    (k·V³) + leeway + slip (power into the water), exactly. With quadratic
//    drag on blade and hull, these shares barely depend on effort: every
//    speed and force scales together. Switching does: a faster boat turns
//    more per stroke, so it switches more often.
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
  const RHO = 1000, CD = 1.6;
  // A marathon blade is a rounded teardrop: about 0.78 of its length × width.
  const BLADE_FILL = 0.78;
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
  // Pitch: the waterplane's fore-aft second moment (a slim, pointed shape:
  // about area·L²/20), the inertia of boat, paddler and the water moving
  // with the hull, and damping as for heave. The body's weight moving
  // fore-aft, the hips swinging the torso (its centre about BODY_H above
  // the waterline) and the push on the hull, about THRUST_ARM above the
  // drag, pitch it.
  const PITCH_IL = WATERPLANE * 5.6 * 5.6 / 20, PITCH_ZETA = 0.3, BODY_H = 0.4, THRUST_ARM = 0.1;
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

  // Roll. A USCA C1 is 31–32" wide at its 4" waterline; at race weight it
  // floats a little lower and narrower. Transverse waterplane inertia
  // C_IT·L·B³/12 over the displaced volume is the metacentric radius; the
  // paddler kneeling or sitting high puts the centre of gravity KG up: KG is
  // for the default seat, and a seat Δz higher lifts the paddler's share of
  // the weight by Δz. The roll inertia is mostly the paddler's (radius of
  // gyration ~0.35 m).
  const ROLL_BEAM = 0.72, ROLL_CIT = 0.45, ROLL_KG = 0.45, ROLL_KB = 0.04;
  const ROLL_GYR = 0.35, BOAT_KG = 11, ROLL_ZETA = 0.2;
  // Bent-shaft marathon paddle, distances from the grip (m): throat, blade
  // length and the blade's centre of pressure, from its length and blade
  // (a ZRE Revolution: 51" overall, 17" × 8" blade, by default).
  const paddleOf = (p) => ({ throat: p.paddleLenM - p.bladeLenM, bladeLen: p.bladeLenM, copFromThroat: 0.5 * p.bladeLenM,
                             area: BLADE_FILL * p.bladeLenM * p.bladeWidthM });

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
    paddleLenM: 1.295,         // 51"
    bladeLenM: 0.432,          // 17"
    bladeWidthM: 0.203,        // 8"
    rateSpm: 60,
    driveFraction: DRIVE_FRACTION,  // share of each stroke spent in the drive
    massKg: 90,                // paddler + boat
    seatZM: 0.10,              // hips above the waterline, sitting in the boat: higher is less stable
    effortW: 150,              // effort the effort-dependent results are for
    switchHeadingDeg: 3,       // switch sides when this far off course
    switchLossS: 0.3,          // time lost per switch, s (an estimate, not measured)
    switchStrokes: 0,          // fixed strokes per side; 0 = switch by heading
    // Body motion, relative to the boat.
    swingM: 0,                 // torso fore-aft, ± m (half the travel)
    swingTiming: 1,            // 1: moving back (aft) through the drive; -1: forward
    dropM: 0,                  // drop onto the paddle at the catch, m
    heelDeg: 0,                // boat heeled toward the paddle side
    hipRock: 0,                // share of the blade's roll the hips cancel, 0–1
    // The paddler's engine.
    fitnessW: 200,             // power you could hold for an hour using your whole body
    rotation: 0,               // trunk rotation, 0–1: more reach and hand travel, more muscle
    legDrive: 0,               // leg drive, 0–1: force held later in the drive, more muscle
    hinge: 0,                  // hinge from the waist, 0–1: lean to the catch, sit up through the drive
    // Strength by position (off by default). 1: by paddle angle, weakest far
    // from strengthPeakDeg, losing strengthDrop of your strength 45° away
    // (with arms of ARM_REF). 2: from the arms, weakest with an arm folded
    // or at full stretch, losing strengthDrop at full stretch.
    strengthOn: 0,
    strengthPeakDeg: 0,        // shaft angle you pull hardest at; + = behind vertical
    strengthDrop: 0.4,
    // Gearing (off by default): the arms hold the most power at one speed.
    gearOn: 0,
    armBestMs: 0.9,            // the lower hand's speed against its shoulder you hold the most power at, m/s
    gearDrop: 0.15,            // share of the arms' power lost at half or 1.5 times it
    // The paddler's body, m, fitted to the paddle each moment (see fitBody).
    // Results change through strength by position: by angle, the arm length;
    // from the arms, every length. Otherwise it's for the drawings and the
    // reach check.
    lowerHandM: 0.60,          // grip to the lower hand, along the shaft
    heightM: 1.78,
    torsoM: 0.55,              // hip to shoulder
    shoulderM: 0.40,           // shoulder width
    upperArmM: 0.30,
    forearmM: 0.30,            // elbow to the middle of the grip
    thighM: 0.45,
    shinM: 0.45,
  };

  // Who's paddling, as opposed to how.
  const PROFILE_KEYS = ["heightM", "torsoM", "shoulderM", "upperArmM", "forearmM", "thighM", "shinM", "massKg", "fitnessW",
                        "strengthOn", "strengthPeakDeg", "strengthDrop", "gearOn", "armBestMs", "gearDrop"];
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

  // Shaft direction (grip → blade): start pointing down, swing by θ about
  // the lateral axis (+θ moves the blade aft), lean outboard by φ, then
  // turn the stroke plane by ψ about the vertical.
  function shaftDir(th, phi, psi) {
    let d = [-Math.sin(th), 0, -Math.cos(th)];
    d = [d[0], d[1] * Math.cos(-phi) - d[2] * Math.sin(-phi), d[1] * Math.sin(-phi) + d[2] * Math.cos(-phi)];
    return [d[0] * Math.cos(psi) - d[1] * Math.sin(psi), d[0] * Math.sin(psi) + d[1] * Math.cos(psi), d[2]];
  }
  // Paddle pose at drive progress u ∈ [0, 1].
  // When in the drive the force comes: leg drive holds it later, a drop
  // onto the paddle loads the blade early (u^w: w > 1 later, < 1 earlier).
  const driveWarp = (p) => (1 + 0.5 * (p.legDrive || 0)) * (1 - 0.35 * Math.min(1, (p.dropM || 0) / 0.1));
  function pose(p, u) {
    const e = ease(Math.pow(u, driveWarp(p)));
    const th = rad(-p.catchDeg + (p.catchDeg + p.exitDeg) * e);
    const psi = rad(p.directionDeg + p.sweepDeg * (0.5 - e));
    const d = shaftDir(th, rad(p.leanDeg), psi);
    // The pivot stays put in the boat (plus hand travel aft), at the height
    // that puts the throat at the waterline.
    const pivot = [-p.handTravelM * e, -p.handOutM, -d[2] * (paddleOf(p).throat - p.pivotFromGrip)];
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
    const PADDLE = paddleOf(p);
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

  // Strength by position: the share of your strength at shaft angle thDeg
  // (from vertical, + behind). Full at strengthPeakDeg, dropping as the
  // square of the angle away from it; longer arms keep it further out. An
  // assumption, not measured: your paddle can log the real curve.
  const ARM_REF = 0.6, STRENGTH_MIN = 0.25;
  function strengthAt(p, thDeg) {
    if (p.strengthOn !== 1) return 1;
    const arm = (p.upperArmM || 0.3) + (p.forearmM || 0.3);
    const x = ((thDeg - p.strengthPeakDeg) * (ARM_REF / arm)) / 45;
    return Math.max(STRENGTH_MIN, 1 - p.strengthDrop * x * x);
  }
  // ------------------------------------------------------------ the body
  // The paddler sits on the centreline at a seat seatX ahead of the paddle's
  // pivot (behind it, so negative), hips seatZM above the water. The torso
  // pitches forward, rolls toward the paddle side and twists (paddle-side
  // shoulder forward) about the hips. Each moment it takes the posture
  // nearest the technique's own (its lean and rotation) that keeps both
  // hands within REACH of their shoulders: top hand on the grip, lower hand
  // lowerHandM down the shaft.
  const LEAN0 = rad(15), REACH = 0.97;
  const POSE_SIGMA = [rad(25), rad(15), rad(25)];
  const POSE_LIMITS = [[rad(-20), rad(60)], [rad(-10), rad(35)], [rad(-50), rad(50)]];
  const dist = (a, b) => norm(sub(a, b));
  const armOf = (p) => p.upperArmM + p.forearmM;
  const lowerHand = (p) => Math.max(p.pivotFromGrip + 0.1, p.lowerHandM);
  // Shoulders and hips for posture v = [pitch, roll, twist] (rad).
  function torso(p, seatX, v, dz = 0) {
    const [pt, rl, tw] = v, hip = [seatX, 0, p.seatZM + dz];
    const t = [Math.sin(pt) * Math.cos(rl), -Math.sin(rl), Math.cos(pt) * Math.cos(rl)];
    const centre = add(hip, mul(t, p.torsoM));
    const across = [-Math.sin(tw), Math.cos(tw) * Math.cos(rl), Math.cos(tw) * Math.sin(rl)];   // toward the off side
    const h = p.shoulderM / 2;
    return { hip, centre, up: t, off: add(centre, mul(across, h)), pad: sub(centre, mul(across, h)) };
  }
  // How far past its reach an arm is (m), rounded over REACH_ROUND: a sharp
  // corner where the arm meets its reach stalls the Newton search below,
  // leaving the posture where it started along that limit.
  const REACH_ROUND = 0.002;
  const beyond = (x) => 0.5 * (x + Math.sqrt(x * x + REACH_ROUND * REACH_ROUND));
  // The posture nearest `neutral` that reaches grip and low. Returns the
  // body and how stretched each arm is (shoulder to hand / arm length).
  // quick: a coarse search, good enough to choose the seat.
  function fitBody(p, seatX, grip, low, neutral, dz = 0, start = neutral, quick = false) {
    const arm = armOf(p), max = REACH * arm;
    const cost = (v) => {
      const b = torso(p, seatX, v, dz);
      let e = 0;
      for (let i = 0; i < 3; i++) e += ((v[i] - neutral[i]) / POSE_SIGMA[i]) ** 2;
      const d1 = dist(b.off, grip), d2 = dist(b.pad, low);
      const o1 = beyond(d1 - max), o2 = beyond(d2 - max);
      // A mild liking for comfortably bent arms, as a paddler sits to have.
      const comfy = ((d1 / arm - ARM_BEST) / 0.3) ** 2 + ((d2 / arm - ARM_BEST) / 0.3) ** 2;
      return e + 0.3 * comfy + 1e5 * (o1 * o1 + o2 * o2);
    };
    // Damped Newton on a smooth cost (finite-difference gradient and
    // Hessian), kept inside the posture limits: converges exactly, so the
    // posture changes smoothly from one moment to the next.
    const clamp = (v) => v.map((x, i) => Math.min(POSE_LIMITS[i][1], Math.max(POSE_LIMITS[i][0], x)));
    let v = clamp(start.slice()), e = cost(v), lam = 1e-3;
    if (quick) {
      for (let step = rad(8); step > rad(0.5); ) {
        let moved = false;
        for (let i = 0; i < 3; i++) for (const sgn of [1, -1]) {
          const w = v.slice(); w[i] += sgn * step;
          const c = clamp(w), ew = cost(c);
          if (ew < e - 1e-12) { v = c; e = ew; moved = true; }
        }
        if (!moved) step /= 2;
      }
      const b = torso(p, seatX, v, dz);
      return Object.assign(b, { pose: v, cost: e, top: dist(b.off, grip) / armOf(p), low: dist(b.pad, low) / armOf(p) });
    }
    const h = 1e-4;
    for (let it = 0; it < 40; it++) {
      const g = [0, 0, 0], H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      const at = (di, dj, si, sj) => { const w = v.slice(); w[di] += si * h; if (dj >= 0) w[dj] += sj * h; return cost(w); };
      for (let i = 0; i < 3; i++) {
        const ep = at(i, -1, 1), em = at(i, -1, -1);
        g[i] = (ep - em) / (2 * h);
        H[i][i] = (ep - 2 * e + em) / (h * h);
        for (let j = 0; j < i; j++)
          H[i][j] = H[j][i] = (at(i, j, 1, 1) - at(i, j, 1, -1) - at(i, j, -1, 1) + at(i, j, -1, -1)) / (4 * h * h);
      }
      let improved = false;
      for (let tries = 0; tries < 8 && !improved; tries++) {
        const A = H.map((row, i) => row.map((x, j) => x + (i === j ? lam * (1 + Math.abs(H[i][i])) : 0)));
        const d = solve3(A, g.map((x) => -x));
        const w = clamp(v.map((x, i) => x + d[i])), ew = cost(w);
        if (ew < e) { v = w; improved = Math.abs(e - ew) > 1e-14; e = ew; lam = Math.max(1e-6, lam / 4); if (!improved) break; }
        else lam *= 8;
      }
      if (!improved) break;
    }
    const b = torso(p, seatX, v, dz);
    return Object.assign(b, { pose: v, cost: e, top: dist(b.off, grip) / arm, low: dist(b.pad, low) / arm });
  }
  // Solves A·x = b for a 3×3 A (Cramer's rule).
  function solve3(A, b) {
    const det = (m) => m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    const D = det(A) || 1e-12;
    return [0, 1, 2].map((k) => det(A.map((row, i) => row.map((x, j) => (j === k ? b[i] : x)))) / D);
  }
  // The technique's own posture at drive progress u: leaning LEAN0, further
  // at the catch and less at the exit with a hinge (HINGE_TILT), and the
  // shoulders wound up at the catch by the rotation, unwinding to the exit.
  const neutralAt = (p, u) => [LEAN0 + HINGE_TILT * (p.hinge || 0) * Math.cos(Math.PI * u), 0,
                               rad(25 * (p.rotation || 0) * Math.cos(Math.PI * u))];
  // Where the blade leaves the water, fore and aft: the shaft-axis point level
  // with the blade centre at the end of the drive (as in drive()).
  const exitX = (pg) => pose(pg, 1).at(paddleOf(pg).throat + paddleOf(pg).copFromThroat)[0];
  // Where the paddler sits is set by who they are and what's in the boat,
  // not by the stroke: their own stroke (the default technique, with their
  // body, paddle and seat height) leaves the water at the hip, as a C1
  // paddler's does. That puts the top hand ahead of its shoulder. Every
  // technique is paddled from that seat, so one that pulls behind exits
  // behind the hip. If the whole of their own stroke can't be reached from
  // there, the nearest seat that reaches it.
  const SEAT_KEYS = [...PROFILE_KEYS, "paddleLenM", "bladeLenM", "bendDeg", "seatZM"];
  const seatCache = new Map();
  function seatFor(pg) {
    const own = paddled(Object.assign({}, DEFAULTS, Object.fromEntries(SEAT_KEYS.map((k) => [k, pg[k]]))));
    const key = JSON.stringify(own);
    if (seatCache.has(key)) return seatCache.get(key);
    const frames = [0, 0.25, 0.5, 0.75, 1].map((u) => {
      const P = pose(own, u);
      return { grip: P.at(0), low: P.at(lowerHand(own)), n: neutralAt(own, u) };
    });
    const stretchPast = (b) => Math.max(0, b.top - REACH) + Math.max(0, b.low - REACH);   // arm stretch past reach
    const x0 = exitX(own);
    const total = (x) => 1e4 * frames.reduce((a, f) => a + stretchPast(fitBody(own, x, f.grip, f.low, f.n, 0, f.n, true)) ** 2, 0) +
      ((x - x0) / 0.2) ** 2;
    let lo = -1.0, hi = 0.3;
    for (let i = 0; i < 14; i++) {   // to about 2 mm
      const m1 = lo + (hi - lo) * 0.382, m2 = lo + (hi - lo) * 0.618;
      if (total(m1) < total(m2)) hi = m2; else lo = m1;
    }
    if (seatCache.size > 60) seatCache.clear();
    seatCache.set(key, (lo + hi) / 2);
    return (lo + hi) / 2;
  }
  // The body through the drive: how stretched each arm is, how far forward
  // the trunk leans (rad), how high the top hand is above the shoulders (m),
  // and how far short of the hands the arms fall where they can't reach (m). armSpeed: the lower hand's average speed
  // against its shoulder through the drive (m/s), what the arm's muscles
  // move at; the trunk carries the rest of the hand's travel.
  const bodyCache = new Map();
  function bodyThroughDrive(pg, n = 20) {
    const key = JSON.stringify(pg);
    if (bodyCache.has(key)) return bodyCache.get(key);
    const seatX = seatFor(pg), arm = armOf(pg), out = [];
    let prev = null, rel = null, path = 0;
    for (let i = 0; i <= n; i++) {
      const u = i / n, P = pose(pg, u), nn = neutralAt(pg, u), low = P.at(lowerHand(pg));
      const b = fitBody(pg, seatX, P.at(0), low, nn, 0, prev || nn);
      prev = b.pose;
      const r = sub(low, b.pad), grip = P.at(0);
      if (rel) path += dist(r, rel);
      rel = r;
      out.push({ u, top: b.top, low: b.low, lean: b.pose[0], above: grip[2] - b.centre[2],
                 short: Math.max(0, (Math.max(b.top, b.low) - 1) * arm) });
    }
    const body = { seatX, frames: out, short: Math.max(...out.map((f) => f.short)), armSpeed: path / (60 / pg.rateSpm * pg.driveFraction) };
    if (bodyCache.size > 60) bodyCache.clear();
    bodyCache.set(key, body);
    return body;
  }
  // From the arms: strongest with the arm about ARM_BEST of full stretch,
  // weaker folded up or stretched out; the lower (pulling) arm counts most.
  const ARM_BEST = 0.72, ARM_SPAN = 0.25;
  const armStrength = (p, e) => Math.max(STRENGTH_MIN, 1 - p.strengthDrop * ((Math.min(1.05, e) - ARM_BEST) / ARM_SPAN) ** 2);
  // Leaning far forward to reach the catch is weaker too: the trunk can't
  // brace the pull. Lean past the usual LEAN0 costs strength as the square
  // of the excess, losing strengthDrop LEAN_SPAN past it. Assumed, not measured.
  const LEAN_SPAN = rad(40);
  const leanStrength = (p, pitch) => Math.max(STRENGTH_MIN, 1 - p.strengthDrop * (Math.max(0, pitch - LEAN0) / LEAN_SPAN) ** 2);

  // Drive progress u at each moment. Where you're weaker the paddle moves
  // slower (force ∝ speed², so speed ∝ √strength puts the force in step
  // with your strength), and faster where you're strong: the drive takes
  // the same time. Returns u(τ) for τ = time / drive time, and the average
  // strength over the drive's time.
  const timingCache = new Map();
  function driveTiming(p) {
    if (!p.strengthOn) return { at: (tau) => tau, mean: 1, strength: null };
    const key = JSON.stringify(p);
    if (timingCache.has(key)) return timingCache.get(key);
    const n = p.strengthOn === 2 ? 80 : 400, cum = [0], strength = [];
    let sumS = 0;
    // From the arms: fit the body through the drive, warm-starting each
    // moment from the last.
    const seatX = p.strengthOn === 2 ? seatFor(p) : 0;
    let prev = null;
    const sAt = (u) => {
      if (p.strengthOn === 1) return strengthAt(p, (pose(p, u).th * 180) / Math.PI);
      const P = pose(p, u), nn = neutralAt(p, u);
      const b = fitBody(p, seatX, P.at(0), P.at(lowerHand(p)), nn, 0, prev || nn);
      prev = b.pose;
      return (0.75 * armStrength(p, b.low) + 0.25 * armStrength(p, b.top)) * leanStrength(p, b.pose[0]);
    };
    for (let i = 1; i <= n; i++) {
      const u = (i - 0.5) / n, s = sAt(u);
      strength.push([u, s]);
      cum.push(cum[i - 1] + 1 / Math.sqrt(s));
      sumS += Math.sqrt(s);
    }
    const T = cum[n];
    const out = {
      strength,
      at: (tau) => {
        const target = Math.max(0, Math.min(1, tau)) * T;
        let lo = 0, hi = n;
        while (hi - lo > 1) { const m = (lo + hi) >> 1; if (cum[m] < target) lo = m; else hi = m; }
        const f = (target - cum[lo]) / Math.max(1e-12, cum[hi] - cum[lo]);
        return (lo + f) / n;
      },
      mean: sumS / T,   // ∫s dt / Td, with dt ∝ du / √s
    };
    if (timingCache.size > 60) timingCache.clear();
    timingCache.set(key, out);
    return out;
  }

  // Samples one drive at a given boat speed V. Velocities in the boat frame.
  function drive(p, V, steps = 240) {
    const Tc = 60 / p.rateSpm, Td = Tc * p.driveFraction, dt = Td / steps;
    const PADDLE = paddleOf(p), k = 0.5 * RHO * CD * PADDLE.area;
    const timing = driveTiming(p);
    const out = [];
    for (let i = 0; i <= steps; i++) {
      const tau = i / steps, dtau = 1e-3;
      const ta = Math.max(0, tau - dtau), tb = Math.min(1, tau + dtau);
      const u = timing.at(tau);
      const P = pose(p, u);
      const Pa = pose(p, timing.at(ta)), Pb = pose(p, timing.at(tb));
      const span = (tb - ta) * Td;
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
        u, t: tau * Td, dt: i === 0 || i === steps ? dt / 2 : dt,
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
    return { power: Math.max(0, extra), swing: Math.max(...last) - Math.min(...last), series: last.map((x) => x - avg), dt };
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
  function pitch(M, Tc, mass) {
    const K = RHO * G * PITCH_IL, I = pitchInertia(mass), c = 2 * PITCH_ZETA * Math.sqrt(K * I);
    const n = 1000, dt = Tc / n;
    let mean = 0;
    for (let i = 0; i < n; i++) mean += M(i * dt) / n;
    let q = 0, w = 0, power = 0, lo = Infinity, hi = -Infinity;
    for (let k = 0; k < 12; k++)
      for (let i = 0; i < n; i++) {
        w += ((M(i * dt) - mean - c * w - K * q) / I) * dt;
        q += w * dt;
        if (k === 11) { power += c * w * w / n; lo = Math.min(lo, q); hi = Math.max(hi, q); }
      }
    return { power, amplitude: (hi - lo) / 2 };
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
    return { perSide, duty: perSide / (perSide + switchLossStrokes(p)), wander, course: 1 - (wander * wander) / 6, turnPerStroke };
  }

  // Body motion relative to the boat over one stroke (period Tc, drive Td).
  // Swing: the torso moves aft through the drive and forward through the
  // recovery (timing 1), or the other way, ± A. Drop: down by d over the
  // first 40% of the drive, held, and back up through the recovery.
  // Velocity (fore-aft) and vertical acceleration, as functions of time.
  function bodySwingVel(A, timing, Tc, Td) {
    return (t) => timing * (t < Td ? (-A * Math.PI / Td) * Math.sin(Math.PI * t / Td) : (A * Math.PI / (Tc - Td)) * Math.sin(Math.PI * (t - Td) / (Tc - Td)));
  }
  // A full hinge swings the torso about ±20° from the hips: head, arms and
  // trunk (about 60% of you) move about ±10 cm, ±6 cm for the whole body,
  // aft through the drive. It adds to any body swing set by hand.
  const HINGE_SWING_M = 0.06, HINGE_TILT = rad(20);
  function swingVel(p, Tc, Td) {
    const a = bodySwingVel(p.swingM || 0, p.swingTiming, Tc, Td), b = bodySwingVel(HINGE_SWING_M * (p.hinge || 0), 1, Tc, Td);
    return (t) => a(t) + b(t);
  }
  // The body's fore-aft place (m, + forward, forward at the catch when moving
  // back in the drive) and its acceleration, from the same motion.
  function swingPosAcc(p, Tc, Td) {
    const one = (A, s) => ({
      x: (t) => s * (t < Td ? A * Math.cos(Math.PI * t / Td) : -A * Math.cos(Math.PI * (t - Td) / (Tc - Td))),
      a: (t) => s * (t < Td ? -A * Math.pow(Math.PI / Td, 2) * Math.cos(Math.PI * t / Td) : A * Math.pow(Math.PI / (Tc - Td), 2) * Math.cos(Math.PI * (t - Td) / (Tc - Td))),
    });
    const u = one(p.swingM || 0, p.swingTiming), v = one(HINGE_SWING_M * (p.hinge || 0), 1);
    return { x: (t) => u.x(t) + v.x(t), a: (t) => u.a(t) + v.a(t) };
  }
  // The body's height from the drop (m, 0 sitting up, −d after the catch).
  function bodyDropPos(d, Tc, Td) {
    const t1 = 0.4 * Td;
    return (t) => (t < t1 ? (d / 2) * (Math.cos(Math.PI * t / t1) - 1) : t < Td ? -d : -(d / 2) * (1 + Math.cos(Math.PI * (t - Td) / (Tc - Td))));
  }
  // Bow-up pitch moment (N·m) over a stroke.
  function pitchMoment(p, fx, Tc, Td) {
    const mp = p.massKg - BOAT_KG, b = swingPosAcc(p, Tc, Td);
    return (t) => -mp * G * b.x(t) + mp * BODY_H * b.a(t) - THRUST_ARM * fx(t);
  }
  const pitchInertia = (mass) => BOAT_KG * Math.pow(0.28 * 5.6, 2) + (mass - BOAT_KG) * 0.09 + mass * 5.6 * 5.6 / 20;
  function bodyDropAcc(d, Tc, Td) {
    const t1 = 0.4 * Td;
    return (t) => (t < t1 ? (-d / 2) * Math.pow(Math.PI / t1, 2) * Math.cos(Math.PI * t / t1) :
      t < Td ? 0 : (d / 2) * Math.pow(Math.PI / (Tc - Td), 2) * Math.cos(Math.PI * (t - Td) / (Tc - Td)));
  }
  // Extra hull drag power (W) when the hull's speed is V + δ(t) − (m_p/m)·u(t),
  // u the body's speed relative to the boat: momentum is shared, drag isn't.
  function hullCubic(k, V, delta, dt, u, share) {
    const n = delta.length;
    let extra = 0;
    for (let i = 0; i < n; i++) { const v = V + delta[i] - share * u(i * dt); extra += k * (v * v * v - V * V * V) / n; }
    return Math.max(0, extra);
  }

  // A switch's lost time, in strokes at this rate.
  const switchLossStrokes = (p) => (p.switchLossS * p.rateSpm) / 60;
  // The stroke as paddled: heel brings the hands in, rotation and a hinge
  // reach further and travel further.
  const paddled = (p) => Object.assign({}, p, {
    handOutM: Math.max(0.1, p.handOutM - 0.9 * Math.sin(rad(p.heelDeg || 0))),
    catchDeg: p.catchDeg + 15 * (p.rotation || 0) + 8 * (p.hinge || 0),
    handTravelM: p.handTravelM + 0.12 * (p.rotation || 0) + 0.15 * (p.hinge || 0),
  });
  function simulate(params) {
    const p = Object.assign({}, DEFAULTS, params);
    // Heeling the boat toward the paddle brings the hands (about 0.9 m up)
    // nearer the keel line: the stroke works as if the top hand were further in.
    // Trunk rotation and a hinge from the waist reach further at the catch and
    // carry the hands further.
    const pg = paddled(p);
    const k = HULL_K * Math.pow(p.massKg / MASS_REF, 2 / 3);  // drag ∝ wetted area
    // Boat speed: mean thrust falls as the boat speeds up (less slip), so
    // bisect for thrust (less the time lost switching) = hull drag.
    const Tc0 = 60 / p.rateSpm;
    let lo = 0, hi = 8;
    for (let i = 0; i < 40; i++) {
      const V = (lo + hi) / 2;
      const m = means(pg, V, k);
      const sw = switching(pg, V, m.yaw, Tc0);
      if (sw.duty * m.thrust > k * V * V + m.leeway) lo = V; else hi = V;
    }
    const V = (lo + hi) / 2;
    const { samples, Tc } = drive(pg, V);
    const sum = (f) => samples.reduce((a, s) => a + f(s) * s.dt, 0);
    const yawMean = sum((s) => s.yaw) / Tc;
    const sw = switching(pg, V, yawMean, Tc);
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
      const fz = replay(samples, Td0, Td1, scale, 2), mp = p.massKg - BOAT_KG;
      const h = heave(fz, Tc1, p.massKg);
      // Between switches the blade pulls at full strength: the switches
      // thin out the strokes, not each stroke's surge.
      const fx = replay(samples, Td0, Td1, scale, 0);
      const sg = surge(fx, Tc1, Vw, p.massKg, k);
      const pt = pitch(pitchMoment(p, fx, Tc1, Td1), Tc1, p.massKg);
      // The body's motion: the swing shakes the hull's speed, the drop pushes it up and down.
      const swingPower = p.swingM || p.hinge ? hullCubic(k, Vw, sg.series, sg.dt, swingVel(p, Tc1, Td1), mp / p.massKg) - sg.power : 0;
      const dropAcc = bodyDropAcc(p.dropM, Tc1, Td1);
      const hd = p.dropM ? heave((t) => fz(t) - mp * dropAcc(t), Tc1, p.massKg) : h;
      const weight = p.massKg * G;
      const meanLift = sw.duty * h.meanLift;   // averaged over a side, switches included
      const sink = Math.pow(Math.max(0.05, 1 - meanLift / weight), 2 / 3) - 1;
      const bounce = h.power / (k * Vw * Vw * Vw), drop = (hd.power - h.power) / (k * Vw * Vw * Vw);
      const surgeFrac = sg.power / (k * Vw * Vw * Vw), swing = Math.max(0, swingPower) / (k * Vw * Vw * Vw);
      const pitchFrac = pt.power / (k * Vw * Vw * Vw);
      const eps = sink + bounce + surgeFrac + swing + drop + pitchFrac, f = 1 / (1 + eps);
      // Turning per stroke grows with speed (moment ∝ V², damping ∝ V), so
      // switching is worked out at this effort's speed. (Speed itself is
      // solved with the duty at the stroke's own speed: within ~0.1%.)
      const swW = switching(pg, Vw * Math.cbrt(f), yawMean * scale, Tc1);
      const r = {
        speed: Vw * Math.cbrt(f) * swW.course,    // made good along the course
        waterSpeed: Vw * Math.cbrt(f),            // through the water
        efficiency: base * f,
        // Shares of your power. They add to 1; `sink` is negative (a saving)
        // when the stroke lifts the boat.
        shares: { hull: base * f, slip: slip / pd, leeway: leeway / pd, surge: base * surgeFrac * f,
                  sink: base * sink * f, bounce: base * bounce * f, swing: base * swing * f, drop: base * drop * f,
                  pitch: base * pitchFrac * f },
        meanLift,                        // N, + lifts the boat
        sinkMm: -meanLift / (RHO * G * WATERPLANE) * 1000,  // + deeper
        bounceMm: hd.amplitude * 1000,      // with the drop, if any
        bounceStillMm: h.amplitude * 1000,  // the stroke alone
        pitchDeg: pt.amplitude * 180 / Math.PI,      // ± bow up and down
        bowMm: pt.amplitude * 5.6 / 2 * 1000,         // ± at the bow
        surgeSwing: sg.swing / Vw,        // speed swing over a stroke, peak to peak / mean
        hullK: k * (1 + eps),            // effective drag constant at this effort
        switching: swW,                  // strokes per side, wander, duty at this effort
      };
      // Boat score (0–100): how much of the push on the hull becomes steady
      // forward speed. Points off for time lost switching, the zigzag, and
      // the power that goes into surging, bobbing, crabbing and the body's
      // motion; riding higher earns some back. Blade slip isn't counted.
      const sh = r.shares, over = (x) => (x / Math.max(1e-9, sh.hull)) * 100;
      const parts = { switching: (1 - swW.duty) * 100, zigzag: (1 - swW.course) * 100, swing: over(sh.swing), drop: over(sh.drop),
                      surge: over(sh.surge), bob: over(sh.bounce), pitch: over(sh.pitch), crab: over(sh.leeway), riding: over(sh.sink) };
      r.boatScore = { total: Math.max(0, 100 - Object.values(parts).reduce((a, x) => a + x, 0)), parts };
      cache.set(key, r);
      return r;
    }
    // One stroke at `watts`, n steps: what the stroke and the water do to the
    // hull. Forward, sideways and vertical push (N); speed (m/s) and hull drag
    // k·V² (N) from the surge; distance ahead of a boat at the same average
    // speed (m); heave (m, + up); the roll moment about the keel (N·m, + puts
    // the paddle side down) and the heel it would cause if the paddler held
    // still (rad, about the balanced mean), with the hip shift that balances
    // the mean (m, toward the paddle side when +).
    function cycleAt(watts, n = 200) {
      const e = at(watts), scale = Math.pow(watts / pd, 2 / 3);
      const Tc1 = 60 / p.rateSpm, Td1 = Tc1 * p.driveFraction, dt = Tc1 / n;
      const sampleAt = (t) => (t > Td1 ? null : samples[Math.min(samples.length - 1, Math.round((t / Td1) * (samples.length - 1)))]);
      const t = [], push = [], side = [], up = [], roll = [];
      for (let i = 0; i < n; i++) {
        const s = sampleAt(i * dt), f = s ? mul(s.thrust, scale) : [0, 0, 0];
        t.push(i * dt); push.push(f[0]); side.push(f[1]); up.push(f[2]);
        roll.push(s ? s.B.cop[1] * f[2] - s.B.cop[2] * f[1] : 0);
      }
      // A periodic linear response to a forcing f: m·x'' + c·x' + K·x = f − mean (K = 0: surge).
      const periodic = (f, m, c, K) => {
        const mean = f.reduce((a, x) => a + x, 0) / n;
        let x = 0, v = 0, xs = [], vs = [];
        for (let k = 0; k < 40; k++) {
          xs = []; vs = [];
          for (let i = 0; i < n; i++) { v += ((f[i] - mean - c * v - K * x) / m) * dt; x += v * dt; xs.push(x); vs.push(v); }
        }
        return { xs, vs, mean };
      };
      const V = e.waterSpeed, ms = p.massKg * (1 + SURGE_ADDED), mp = p.massKg - BOAT_KG;
      const sg = periodic(push, ms, 2 * k * V, 0);
      const vAvg = sg.vs.reduce((a, x) => a + x, 0) / n;
      const still = sg.vs.map((x) => V + x - vAvg);
      const uB = swingVel(p, Tc1, Td1), aB = bodyDropAcc(p.dropM, Tc1, Td1);
      const speed = still.map((v, i) => v - (mp / p.massKg) * uB(i * dt));
      let ahead = 0;
      const gap = speed.map((v) => (ahead += (v - V) * dt));
      const gAvg = gap.reduce((a, x) => a + x, 0) / n;
      const Kh = RHO * G * WATERPLANE, mh = p.massKg * (1 + HEAVE_ADDED);
      const hv = periodic(up.map((f, i) => f - mp * aB(i * dt)), mh, 2 * HEAVE_ZETA * Math.sqrt(Kh * mh), Kh);
      const hAvg = hv.xs.reduce((a, x) => a + x, 0) / n;
      // Roll stiffness Δ·g·GM; inertia mostly the paddler's.
      const vol = p.massKg / RHO, BM = (ROLL_CIT * HULL_L * Math.pow(ROLL_BEAM, 3)) / 12 / vol;
      const KG = ROLL_KG + ((p.massKg - BOAT_KG) / p.massKg) * (p.seatZM - DEFAULTS.seatZM);
      const GM = ROLL_KB + BM - KG, Kr = p.massKg * G * GM;
      const Ir = (p.massKg - BOAT_KG) * ROLL_GYR * ROLL_GYR + BOAT_KG * 0.25 * 0.25;
      const rl = periodic(roll.map((m) => m * (1 - p.hipRock)), Ir, 2 * ROLL_ZETA * Math.sqrt(Kr * Ir), Kr);
      const rAvg = rl.xs.reduce((a, x) => a + x, 0) / n;
      const r0 = p.hipRock ? periodic(roll, Ir, 2 * ROLL_ZETA * Math.sqrt(Kr * Ir), Kr) : rl;
      const span = (a) => (Math.max(...a) - Math.min(...a)) / 2;
      // Pitch (rad, + bow up), and the body's place for drawing the paddler.
      const bp = swingPosAcc(p, Tc1, Td1), dz = bodyDropPos(p.dropM || 0, Tc1, Td1);
      const Kp = RHO * G * PITCH_IL, Ip = pitchInertia(p.massKg);
      const pm = t.map((ti, i) => -mp * G * bp.x(ti) + mp * BODY_H * bp.a(ti) - THRUST_ARM * push[i]);
      const pc = periodic(pm, Ip, 2 * PITCH_ZETA * Math.sqrt(Kp * Ip), Kp);
      const pAvg = pc.xs.reduce((a, x) => a + x, 0) / n;
      return {
        pitch: pc.xs.map((x) => x - pAvg), pitchMoment: pm, bodyX: t.map(bp.x), bodyZ: t.map(dz),
        t, push, side, up, speed, drag: speed.map((v) => k * v * v), ahead: gap.map((x) => x - gAvg),
        heave: hv.xs.map((x) => x - hAvg), roll, heel: rl.xs.map((x) => x - rAvg), speedStill: still,
        heelStill: span(r0.xs), hipMax: p.hipRock * Math.max(...roll.map(Math.abs)) / (mp * G),
        rollMean: rl.mean, hipShift: -rl.mean / ((p.massKg - BOAT_KG) * G),
        rollPeriod: 2 * Math.PI * Math.sqrt(Ir / Kr), GM, drive: Td1, cycle: Tc1,
      };
    }
    const e = at(p.effortW);
    let bodyMemo = null;   // worked out when first asked for: most runs never need it
    return {
      params: p, speed: V, cycle: Tc,
      // The body fitted to this stroke: the seat, how stretched each arm is
      // through the drive, and how far short of the hands they fall (m).
      get body() { return bodyMemo || (bodyMemo = bodyThroughDrive(pg)); },
      strengthCurve: driveTiming(pg).strength,   // [drive progress, strength] (null when off)
      geom: pg,   // the stroke as paddled: heel and rotation applied (for drawing)
      paddlerPower: paddler, usefulPower: useful, slipPower: slip, hullPower: hull,
      leewayPower: leeway,
      // Share of your power that moves the boat at the comparison effort,
      // after slip, sideways, surge and vertical effects, including time
      // lost switching sides.
      efficiency: e.efficiency,
      baseEfficiency: base,
      at,
      // Switching at the comparison effort.
      strokesPerSide: e.switching.perSide,
      headingWanderDeg: e.switching.wander * 180 / Math.PI,   // ± about the course
      courseFactor: e.switching.course,
      turnPerStroke: e.switching.turnPerStroke,               // rad of heading per stroke
      switchDuty: e.switching.duty,
      forceShare: { forward: share(0), sideways: share(1), vertical: share(2) },
      // Share of the blade force that is lift (across the flow), over the drive.
      liftShare: sum((s) => s.lift) / Math.max(1e-9, sum((s) => s.F)),
      yawMoment: yawMean,  // N·m; > 0 turns the bow away from the paddle side
      peakForce,
      speedAt: (watts) => at(watts).speed,
      cycleAt,
      // Turning moment scales with speed², i.e. with effort^(2/3).
      yawAt: (watts) => yawMean * Math.pow(watts / pd, 2 / 3),
      samples,
    };
  }

  // The power a paddler can hold: fitness is whole-body power for an hour.
  // Arms alone manage about 70% of it; trunk rotation adds about 12%, leg
  // drive 10% (kayak studies: blocking the legs costs ~10–16% of power) and
  // a hinge from the waist, the hips and back, 8%.
  // Over a longer race it fades as hours^-0.07 (82% for 16 h).
  // With strength by position on, a stroke that spends more of its drive
  // where you're weak tires you sooner: what you can hold scales with your
  // average strength through the drive, against the default stroke's.
  function strengthFactor(p) {
    const q = Object.assign({}, DEFAULTS, p);
    if (!q.strengthOn) return 1;
    // The default technique, paddled by this paddler.
    const ref = Object.assign({}, DEFAULTS, Object.fromEntries(PROFILE_KEYS.map((k) => [k, q[k]])));
    return driveTiming(paddled(q)).mean / driveTiming(paddled(ref)).mean;
  }
  // Gearing (off by default): muscles make the most power at one speed. A
  // linear force–velocity line makes power a parabola in speed, so the arms
  // hold less the further their speed (armSpeed, set by the stroke rate,
  // the drive's share, the stroke's length and the hands' spacing) is from
  // armBestMs, losing gearDrop at half or 1.5 times it. It sets the arms'
  // share of what you hold; trunk and legs move at their own speeds.
  // Assumed until measured: the meter's force and shaft speed at a few
  // rates give the paddler's own line.
  function gearFactor(p) {
    const q = Object.assign({}, DEFAULTS, p);
    if (!q.gearOn) return 1;
    const x = bodyThroughDrive(paddled(q)).armSpeed / q.armBestMs - 1;
    return Math.max(STRENGTH_MIN, 1 - 4 * q.gearDrop * x * x);
  }
  // The stroke rate that puts the arms at their best speed: arm speed is the
  // arm's path over the drive's time, so it goes as the rate.
  const bestRate = (p) => {
    const q = Object.assign({}, DEFAULTS, p);
    return q.rateSpm * q.armBestMs / bodyThroughDrive(paddled(q)).armSpeed;
  };
  function capacity(p, hours = 1) {
    const q = Object.assign({}, DEFAULTS, p);
    return q.fitnessW * (0.7 * gearFactor(q) + 0.12 * q.rotation + 0.1 * q.legDrive + 0.08 * q.hinge) * strengthFactor(q) *
      Math.pow(Math.max(1, hours), -0.07);
  }

  // Shallow water (The Science of Paddling, Part 4): the boat's own wave, wavelength
  // λ = 2πV²/g, runs slower when the depth D is under about λ/2, as
  // c = √(gλ/2π · tanh(2πD/λ)). The hull then makes the wave it would make
  // in deep water at about V / √tanh(gD/V²), so the wave drag (∝ V⁶ near
  // hull speed) grows by tanh(gD/V²)^-3, capped at SHALLOW_MAX. This is one
  // explicit step of Schlichting's relation (which puts the deep-water speed
  // inside the tanh): close below a depth Froude number of ~0.7, and lower
  // than it nearer the wave speed, where the relation itself breaks down.
  // Under 2 ft at race pace drag "skyrockets".
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
    const loss = switchLossStrokes(r.params), fixed = r.params.switchStrokes > 0;
    const M = Math.abs(r.yawAt(watts));
    const tol = rad(r.params.switchHeadingDeg);
    const head = c.windSpeed * Math.cos(rad(c.windAngle)), cross = Math.abs(c.windSpeed * Math.sin(rad(c.windAngle)));
    const qPlan = (V) => 0.5 * RHO * V * V * Math.PI * Math.pow(2 * HULL_DRAFT, 2);
    // Heading turned per stroke under a moment m at speed V: m over the
    // hull's yaw damping, over one stroke cycle.
    const turn = (m, V) => (m * r.cycle) / yawDamping(Math.max(1e-6, V));
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
      // Current runs along the river. Round a bend it follows the boat's own
      // arc (sg.current is already the current on that line); on a crossing
      // only its share along the path, len/path, helps.
      const ground = st.water + sg.current * (sg.kind === "crossing" ? sg.len / sg.path : 1);
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
      const base = { leg: li, watts, rate: r.params.rateSpm, loss: switchLossStrokes(r.params),
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
    // b can be null to race A alone (B then mirrors A).
    const A = boat(a, wattsA, lineOf("A")), B = b ? boat(b, wattsB, lineOf("B")) : A;
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
  const API_LEVEL = 40;
  const PADDLE = paddleOf(DEFAULTS);
  const api = { API_LEVEL, shaftDir, paddled, paddleOf, gearFactor, bestRate, fitBody, lowerHand, neutralAt, simulate, race, capacity, strengthAt, strengthFactor, BOAT_KG, shallowFactor, riverSegments, pose, blade, DEFAULTS, PADDLE, BOAT_HALF_BEAM, DRIVE_FRACTION, MAX_STROKES_PER_SIDE };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.StrokeSim = api;
})(typeof window !== "undefined" ? window : globalThis);
