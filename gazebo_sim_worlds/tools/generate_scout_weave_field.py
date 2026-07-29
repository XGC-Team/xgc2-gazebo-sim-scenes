#!/usr/bin/env python3
"""Slot-sweep obstacle-field placement and verification for the Scout weave scene.

This is the durable home of the field tool that produced
``worlds/ugv4_figure_eight_scout_obstacles/source_manifest.json``.  The original
lived under /tmp and did not survive a reboot; this rebuild is derived from the
shipped semantics, which are pinned by three persistent artifacts:

  * the routed formation schedule and cosine blend in
    ``config/platforms/scout_mini_planning_geometry.yaml`` of the
    xgc2-formation-dmpc ``formation_generator`` package,
  * the rigid spin and the Gerono leader in
    ``config/scenarios/ugv4_figure_eight_scout/scenario.yaml``,
  * the reference implementation of the composed slot offset in
    ``pysim/scenario_ugv_scout_eight.py`` of the paper repo
    (``schedule_offset`` and ``FIELD``).

What the field has to satisfy
-----------------------------
Every body is scored by its MINIMUM SLOT CLEARANCE: the smallest planar
centre-to-surface distance between the body and any of the four composed slot
references

    p_i(t) = leader(t) + R_z(spin * t) @ h_i(t),

swept over ``t`` in ``[0, laps * period)`` at ``dt``.  ``h_i`` is the routed
schedule blended with the C1 cosine ramp, exactly as in ``schedule_offset``.
The certified requirement is

    REQ = 0.43 (Scout envelope) + 0.05 (paper rho) + 0.001 (rho margin)
        + 0.27 (provisional static-obstacle margin) = 0.751 m,

so a body scoring below REQ is a deliberate BITE that the closed-loop DMPC must
deform around, and the amount below REQ is its deformation demand.  Bodies are
placed in three role bands:

    strong  [0.45, 0.60] m   ~0.25 m deformation demand, the interesting cases
    medium  [0.75, 0.95] m   just clear of REQ, shape pressure without a bite
    tip     [0.90, 1.60] m   lobe-extreme markers, no deformation demand

Global checks (all of them are contract, not preference):

    pair surface gap    >= 2.2 m   no body pair may form a slot-width gap
    crossing keep-out   |c| - r >= 4.2 m   the self-intersection stays open
    fence               |cx| + r <= 12.5 and |cy| + r <= 12.5 m
    opposite-side gate  two bodies on opposite sides of the leader path may not
                        sit within 12 s of leader path time of each other
    leader corridor     leader path clearance >= 0.43 + 0.05 + 0.10 = 0.58 m

Modes
-----
``--verify`` scores an existing manifest and reports every band and global check
WITHOUT moving anything.  This is the acceptance path and the contract.

``--generate`` re-places the bodies by an incremental anchored scan: the four
strong bodies first, then the four medium bodies, then the two tips last with an
outward fan scan around the lobe extremes.  Anchored bodies take the lateral
offset whose clearance lands nearest the middle of their band; tips take the
largest in-band clearance the fan can reach.  Each candidate is filtered against
the already-placed bodies, so THE RESULT IS PLACEMENT-ORDER DEPENDENT: a new run
may legally land bodies at positions different from the shipped manifest, and it
may land them at a slid anchor when the requested anchor admits no candidate in
band.  That is expected and is not drift.  Bit-reproduction of the shipped
positions is explicitly NOT the contract -- passing ``--verify`` is.  (Concretely,
the shipped ``w_post_e`` realises leader-path anchor 105.1 s rather than the 74 s
seed recorded in the role table below; ``--verify`` prints the realised anchor of
every body so this stays visible.)

Acceptance, against the shipped manifest::

    ./generate_scout_weave_field.py --verify

must report ALL CHECKS PASS with min slot clearances

    w_post_a 0.5313  w_hex_c  0.5417  w_post_e 0.5405  w_slab_g 0.5216  (strong)
    w_slab_b 0.8508  w_ball_d 0.8607  w_hex_f  0.8471  w_ball_h 0.8609  (medium)
    w_tip_r  1.5018  w_tip_l  1.5891                                    (tip)

The scene README's older 0.50-0.51 / 0.83-0.85 / 1.56-1.59 figures predate the
2026-07-30 blend/spin retune (26 s / 0.02 rad/s); the bands recorded in
``source_manifest.json`` are the current ones and are what this tool enforces.

Requires numpy.  Never writes to the manifest it was pointed at; ``--generate``
needs an explicit ``--out``.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    PACKAGE_ROOT / "worlds" / "ugv4_figure_eight_scout_obstacles" / "source_manifest.json"
)

# ---------------------------------------------------------------- requirement
SCOUT_RADIUS = 0.43
PAPER_RHO = 0.05
RHO_MARGIN = 0.001
STATIC_MARGIN = 0.27
REQ = SCOUT_RADIUS + PAPER_RHO + RHO_MARGIN + STATIC_MARGIN  # 0.751 m
LEADER_CORRIDOR = SCOUT_RADIUS + PAPER_RHO + 0.10  # 0.58 m, the pysim C3 audit

# ------------------------------------------------------------------- geometry
HEX_CIRCUMRADIUS = 0.27  # ugv_hexagon_1 template vertex radius, before scale

BANDS = {
    "strong": (0.45, 0.60),
    "medium": (0.75, 0.95),
    "tip": (0.90, 1.60),
}

MIN_PAIR_SURFACE_GAP = 2.2
MIN_CROSSING_KEEPOUT = 4.2
FENCE_HALF_EXTENT = 12.5
MIN_OPPOSITE_SIDE_GATE_SEC = 12.0

# Routed x1.5 schedule: the platform overlay's endpoint slot permutations, i.e.
# scout_mini_planning_geometry.yaml exploration/formation_schedule/entries and
# pysim scenario_ugv_scout_eight.SCHEDULE.  Keep the two byte-identical.
ROUTED_SCHEDULE = (
    (0.0, ((1.8, 0.0), (0.0, 1.8), (-1.8, 0.0), (0.0, -1.8))),
    (42.0, ((0.0, 0.9), (0.0, 2.7), (0.0, -0.9), (0.0, -2.7))),
    (84.0, ((1.8, 0.0), (0.0, 1.8), (-1.8, 0.0), (0.0, -1.8))),
    (126.0, ((2.7, 0.0), (-0.9, 0.0), (-2.7, 0.0), (0.9, 0.0))),
    (168.0, ((1.275, 1.275), (-1.275, 1.275), (-1.275, -1.275), (1.275, -1.275))),
    (210.0, ((1.8, 0.0), (0.0, 1.8), (-1.8, 0.0), (0.0, -1.8))),
)

# Placement order IS this order: strong anchors, then medium anchors, then the
# two tips.  ``anchor`` is a leader path time in seconds; ``side`` is the sign of
# cross(leader_velocity, body_centre - leader_position), i.e. +1 = left of the
# direction of travel.  ``fan`` applies to tips only and biases the first scanned
# side of the outward fan.
ROLES = (
    ("w_post_a", "strong", 12.0, +1, None),
    ("w_hex_c", "strong", 36.0, +1, None),
    ("w_post_e", "strong", 74.0, -1, None),
    ("w_slab_g", "strong", 98.0, -1, None),
    ("w_slab_b", "medium", 24.0, +1, None),
    ("w_ball_d", "medium", 50.0, +1, None),
    ("w_hex_f", "medium", 88.0, -1, None),
    ("w_ball_h", "medium", 112.0, -1, None),
    ("w_tip_r", "tip", None, +1, +1),
    ("w_tip_l", "tip", None, -1, -1),
)
ROLE_OF = {name: role for name, role, _, _, _ in ROLES}


class Config:
    """Leader, formation composition and sweep discretisation."""

    def __init__(self, amplitude, half_height, rate, blend, spin, dt, laps):
        self.amplitude = float(amplitude)
        self.half_height = float(half_height)
        self.rate = float(rate)
        self.blend = float(blend)
        self.spin = float(spin)
        self.dt = float(dt)
        self.laps = float(laps)

    @property
    def period(self):
        return 2.0 * math.pi / self.rate

    def leader(self, t):
        return np.array(
            [
                self.amplitude * math.sin(self.rate * t),
                self.half_height * math.sin(2.0 * self.rate * t),
            ]
        )

    def leader_velocity(self, t):
        return np.array(
            [
                self.amplitude * self.rate * math.cos(self.rate * t),
                2.0 * self.half_height * self.rate * math.cos(2.0 * self.rate * t),
            ]
        )

    def schedule_offset(self, index, t):
        """Cosine-blended routed template, then the rigid spin.

        Byte-for-byte the composition of pysim scenario_ugv_scout_eight
        .schedule_offset, which itself mirrors the C++ offsetOfAgent.
        """
        times = [entry[0] for entry in ROUTED_SCHEDULE]
        active = 0
        for position, switch in enumerate(times):
            if switch <= t:
                active = position
        current = np.asarray(ROUTED_SCHEDULE[active][1][index], float)
        if active > 0:
            gap = times[active] - times[active - 1]
            window = min(self.blend, gap)
            elapsed = t - times[active]
            if elapsed < window:
                previous = np.asarray(ROUTED_SCHEDULE[active - 1][1][index], float)
                alpha = 0.5 - 0.5 * math.cos(math.pi * max(0.0, elapsed / window))
                current = previous + (current - previous) * alpha
        angle = self.spin * t
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array(
            [
                cosine * current[0] - sine * current[1],
                sine * current[0] + cosine * current[1],
            ]
        )


def slot_sweep(config):
    """(points, times, agent_index) of every sampled slot reference."""
    times = np.arange(0.0, config.laps * config.period, config.dt)
    leader = np.stack(
        [
            config.amplitude * np.sin(config.rate * times),
            config.half_height * np.sin(2.0 * config.rate * times),
        ],
        axis=1,
    )
    blocks, stamps, agents = [], [], []
    for index in range(4):
        offsets = np.array([config.schedule_offset(index, t) for t in times])
        blocks.append(leader + offsets)
        stamps.append(times)
        agents.append(np.full(times.shape, index + 1))
    return (
        np.concatenate(blocks, axis=0),
        np.concatenate(stamps),
        np.concatenate(agents),
    )


def leader_path(config, step=0.001):
    times = np.arange(0.0, config.period, step)
    points = np.stack(
        [
            config.amplitude * np.sin(config.rate * times),
            config.half_height * np.sin(2.0 * config.rate * times),
        ],
        axis=1,
    )
    velocities = np.stack(
        [
            config.amplitude * config.rate * np.cos(config.rate * times),
            2.0 * config.half_height * config.rate * np.cos(2.0 * config.rate * times),
        ],
        axis=1,
    )
    return times, points, velocities


def planar_radius(obstacle):
    """Radius of the smallest planar disc containing the body's cross-section."""
    kind = obstacle["type"]
    scale = obstacle["scale"]
    if kind in ("cylinder", "sphere"):
        return float(scale[0])
    if kind == "cube":
        return math.hypot(float(scale[0]) / 2.0, float(scale[1]) / 2.0)
    if kind == "v_polytope":
        if obstacle.get("v_polytope_type") != "ugv_hexagon_1":
            raise ValueError(
                "%s: unknown v_polytope template %r"
                % (obstacle["name"], obstacle.get("v_polytope_type"))
            )
        return HEX_CIRCUMRADIUS * float(scale[0])
    raise ValueError("%s: unsupported type %s" % (obstacle["name"], kind))


class NearestSlot:
    """Nearest sampled slot reference for a batch of candidate centres.

    Uses scipy's KD-tree when it is importable (the placement scan issues
    hundreds of thousands of queries against a 50k-point cloud) and falls back
    to a chunked numpy scan so the tool still runs on a bare ROS python.
    """

    def __init__(self, points):
        self.points = points
        self.tree = None
        try:
            from scipy.spatial import cKDTree
        except ImportError:
            return
        self.tree = cKDTree(points)

    def query(self, centres):
        centres = np.atleast_2d(np.asarray(centres, float))
        if self.tree is not None:
            distances, indices = self.tree.query(centres, k=1)
            return np.asarray(distances, float), np.asarray(indices, int)
        distances = np.empty(len(centres))
        indices = np.empty(len(centres), dtype=int)
        for row, centre in enumerate(centres):
            gaps = np.hypot(self.points[:, 0] - centre[0], self.points[:, 1] - centre[1])
            index = int(gaps.argmin())
            distances[row], indices[row] = gaps[index], index
        return distances, indices


def min_slot_clearance(nearest, centre, radius):
    distances, indices = nearest.query([centre])
    return float(distances[0]) - radius, int(indices[0])


def path_anchor(centre, times, points, velocities):
    """Nearest leader-path time and the side the body sits on."""
    gaps = np.hypot(points[:, 0] - centre[0], points[:, 1] - centre[1])
    index = int(gaps.argmin())
    delta = centre - points[index]
    turn = velocities[index, 0] * delta[1] - velocities[index, 1] * delta[0]
    return times[index], (1 if turn > 0.0 else -1), gaps[index]


def circular_gap(first, second, period):
    delta = abs(first - second)
    return min(delta, period - delta)


class Body:
    __slots__ = ("name", "role", "kind", "centre", "radius", "clearance",
                 "anchor", "side", "worst")

    def __init__(self, name, role, centre, radius):
        self.name = name
        self.role = role
        self.kind = "?"
        self.centre = np.asarray(centre, float)
        self.radius = float(radius)
        self.clearance = None
        self.anchor = None
        self.side = None
        self.worst = None


# ------------------------------------------------------------------ reporting
class Report:
    def __init__(self):
        self.failures = []

    def check(self, ok, label, detail):
        print("  %-4s %-26s %s" % ("PASS" if ok else "FAIL", label, detail))
        if not ok:
            self.failures.append("%s: %s" % (label, detail))
        return ok


def evaluate(bodies, config, nearest, stamps, agents, report):
    times, path, velocities = leader_path(config)

    print("slot sweep: %d samples over t in [0, %.2f) s at dt %.3f, 4 agents"
          % (len(nearest.points), config.laps * config.period, config.dt))
    print("leader A=%.3f B/2=%.3f w=%.4f period=%.4f s; blend %.1f s, spin %.4f rad/s"
          % (config.amplitude, config.half_height, config.rate, config.period,
             config.blend, config.spin))
    print("certified requirement REQ = %.3f + %.3f + %.3f + %.3f = %.4f m"
          % (SCOUT_RADIUS, PAPER_RHO, RHO_MARGIN, STATIC_MARGIN, REQ))
    print()
    print("%-9s %-6s %-10s %9s %9s %9s   %-24s %s"
          % ("body", "role", "type", "radius", "clear", "demand", "band", "worst approach"))

    for body in bodies:
        clearance, index = min_slot_clearance(nearest, body.centre, body.radius)
        body.clearance = clearance
        body.worst = (agents[index], stamps[index])
        body.anchor, body.side, _ = path_anchor(body.centre, times, path, velocities)
        low, high = BANDS[body.role]
        demand = REQ - clearance
        print("%-9s %-6s %-10s %9.4f %9.4f %+9.4f   [%.2f, %.2f] %-9s agent %d @ t=%.2f s"
              % (body.name, body.role, body.kind, body.radius, clearance, demand,
                 low, high, "ok" if low <= clearance <= high else "OUT",
                 body.worst[0], body.worst[1]))
    print()

    print("band checks")
    for body in bodies:
        low, high = BANDS[body.role]
        report.check(
            low <= body.clearance <= high,
            "%s band" % body.name,
            "%.4f m in [%.2f, %.2f]" % (body.clearance, low, high),
        )
    print()

    print("global checks")
    worst_pair = (math.inf, None, None)
    for first in range(len(bodies)):
        for second in range(first + 1, len(bodies)):
            gap = (
                float(np.linalg.norm(bodies[first].centre - bodies[second].centre))
                - bodies[first].radius
                - bodies[second].radius
            )
            if gap < worst_pair[0]:
                worst_pair = (gap, bodies[first].name, bodies[second].name)
    report.check(
        worst_pair[0] >= MIN_PAIR_SURFACE_GAP,
        "pair surface gap",
        "worst %.4f m (%s/%s) >= %.2f" % (worst_pair[0], worst_pair[1], worst_pair[2],
                                          MIN_PAIR_SURFACE_GAP),
    )

    worst_cross = min(
        (float(np.linalg.norm(body.centre)) - body.radius, body.name) for body in bodies
    )
    report.check(
        worst_cross[0] >= MIN_CROSSING_KEEPOUT,
        "crossing keep-out",
        "worst |c|-r %.4f m (%s) >= %.2f" % (worst_cross[0], worst_cross[1],
                                             MIN_CROSSING_KEEPOUT),
    )

    worst_fence = max(
        (max(abs(body.centre[0]), abs(body.centre[1])) + body.radius, body.name)
        for body in bodies
    )
    report.check(
        worst_fence[0] <= FENCE_HALF_EXTENT,
        "fence",
        "worst axis extent %.4f m (%s) <= %.2f" % (worst_fence[0], worst_fence[1],
                                                   FENCE_HALF_EXTENT),
    )

    gates = []
    for first in range(len(bodies)):
        for second in range(first + 1, len(bodies)):
            if bodies[first].side == bodies[second].side:
                continue
            gap = circular_gap(bodies[first].anchor, bodies[second].anchor, config.period)
            if gap < MIN_OPPOSITE_SIDE_GATE_SEC:
                gates.append("%s/%s %.2f s" % (bodies[first].name, bodies[second].name, gap))
    report.check(
        not gates,
        "opposite-side gate",
        "no pair within %.1f s" % MIN_OPPOSITE_SIDE_GATE_SEC if not gates
        else "; ".join(gates),
    )

    corridor = min(
        float(np.hypot(path[:, 0] - body.centre[0], path[:, 1] - body.centre[1]).min())
        - body.radius
        for body in bodies
    )
    report.check(
        corridor >= LEADER_CORRIDOR,
        "leader corridor",
        "min leader clearance %.4f m >= %.4f" % (corridor, LEADER_CORRIDOR),
    )
    print()

    print("realised leader-path anchors (seed anchors are placement hints only)")
    seeds = {name: anchor for name, _, anchor, _, _ in ROLES}
    for body in bodies:
        seed = seeds.get(body.name)
        print("  %-9s t*=%7.2f s  side=%+d  seed=%s"
              % (body.name, body.anchor, body.side,
                 "%.1f s" % seed if seed is not None else "lobe extreme"))
    print()


def load_bodies(manifest):
    bodies = []
    for obstacle in manifest["obstacles"]:
        name = obstacle["name"]
        if name not in ROLE_OF:
            raise ValueError("%s: no role recorded for this body" % name)
        body = Body(name, ROLE_OF[name], obstacle["position"][:2], planar_radius(obstacle))
        body.kind = obstacle["type"]
        bodies.append(body)
    return bodies


# ------------------------------------------------------------------ placement
def unit(vector):
    norm = float(np.linalg.norm(vector))
    if norm <= 0.0:
        raise ValueError("degenerate direction")
    return np.asarray(vector, float) / norm


def slide_sequence(span, step):
    yield 0.0
    offset = step
    while offset <= span + 1e-9:
        yield offset
        yield -offset
        offset += step


def fan_sequence(span, step, first_sign):
    yield 0.0
    angle = step
    while angle <= span + 1e-9:
        yield first_sign * angle
        yield -first_sign * angle
        angle += step


def candidate_ok(centre, radius, placed, config, times, path, velocities):
    for body in placed:
        gap = float(np.linalg.norm(centre - body.centre)) - radius - body.radius
        if gap < MIN_PAIR_SURFACE_GAP:
            return False
    if float(np.linalg.norm(centre)) - radius < MIN_CROSSING_KEEPOUT:
        return False
    if max(abs(centre[0]), abs(centre[1])) + radius > FENCE_HALF_EXTENT:
        return False
    anchor, side, corridor = path_anchor(centre, times, path, velocities)
    if corridor - radius < LEADER_CORRIDOR:
        return False
    for body in placed:
        if body.side == side:
            continue
        if circular_gap(anchor, body.anchor, config.period) < MIN_OPPOSITE_SIDE_GATE_SEC:
            return False
    return True


def best_in_band(nearest, centres, radius, band, mode, placed, config, geometry):
    """Best candidate that is in band and passes every filter, or None.

    ``mode`` picks the selection rule the shipped field was placed with:
    ``"centre"`` takes the candidate whose min slot clearance sits closest to
    the middle of its band -- the anchored strong/medium bodies land at 0.525
    and 0.85 m, which is what reproduces the shipped centres to a few mm.
    ``"first"`` keeps the caller's scan order and takes the first candidate that
    is in band and legal, which is how the tips end up pushed outward by the
    2.2 m pair-gap filter rather than by their own band.
    """
    low, high = band
    times, path, velocities = geometry
    distances, _ = nearest.query(centres)
    clearances = distances - radius
    rows = np.nonzero((clearances >= low) & (clearances <= high))[0]
    if rows.size == 0:
        return None
    if mode == "first":
        order = rows
    else:
        target = 0.5 * (low + high)
        order = rows[np.argsort(np.abs(clearances[rows] - target), kind="stable")]
    for row in order:
        centre = centres[row]
        if candidate_ok(centre, radius, placed, config, times, path, velocities):
            return int(row), centre, float(clearances[row])
    return None


def place_anchored(spec, radius, placed, config, nearest, geometry, options):
    """Lateral scan off the leader path at the seed anchor, requested side first."""
    _, role, anchor, side, _ = spec
    band = BANDS[role]
    offsets = np.arange(options.scan_min, options.scan_max + 1e-9, options.scan_step)
    for slide in slide_sequence(options.anchor_slide, options.anchor_slide_step):
        stamp = anchor + slide
        origin = config.leader(stamp)
        velocity = config.leader_velocity(stamp)
        normal = unit(np.array([-velocity[1], velocity[0]]))
        for sign in (side, -side):
            centres = origin + np.outer(sign * offsets, normal)
            found = best_in_band(nearest, centres, radius, band, "centre",
                                 placed, config, geometry)
            if found is not None:
                _, centre, clearance = found
                return centre, clearance, stamp, sign
    return None


def place_tip(spec, radius, placed, config, nearest, geometry, options):
    """Outward fan scan around a lobe extreme: nearest legal ring, smallest fan."""
    _, role, _, side, fan = spec
    band = BANDS[role]
    # Lobe extremes of the Gerono eight: |x| = A at w t = pi/2 and 3 pi / 2.
    extreme_time = (math.pi / 2.0 if side > 0 else 3.0 * math.pi / 2.0) / config.rate
    origin = config.leader(extreme_time)
    base = 0.0 if side > 0 else math.pi
    angles = np.array(list(fan_sequence(options.fan_span, options.fan_step, fan)))
    headings = base + np.radians(angles)
    rays = np.stack([np.cos(headings), np.sin(headings)], axis=1)
    for distance in np.arange(options.scan_min, options.tip_scan_max + 1e-9,
                              options.scan_step):
        centres = origin + distance * rays
        found = best_in_band(nearest, centres, radius, band, "first",
                             placed, config, geometry)
        if found is not None:
            row, centre, clearance = found
            return centre, clearance, extreme_time, float(angles[row])
    return None


def generate(catalog, config, nearest, options):
    """Re-place every body; returns a new manifest dict.

    Incremental and therefore placement-order dependent: each body is filtered
    against the bodies already placed, so the outcome depends on the order in
    ROLES and on the scan discretisation.  See the module docstring.
    """
    geometry = leader_path(config)
    times, path, velocities = geometry
    catalogue = {obstacle["name"]: obstacle for obstacle in catalog["obstacles"]}
    placed, results = [], {}
    for spec in ROLES:
        name, role = spec[0], spec[1]
        obstacle = catalogue[name]
        radius = planar_radius(obstacle)
        placer = place_tip if role == "tip" else place_anchored
        found = placer(spec, radius, placed, config, nearest, geometry, options)
        if found is None:
            raise SystemExit(
                "generate: no in-band candidate for %s (role %s); widen --scan-max "
                "or --anchor-slide" % (name, role)
            )
        centre, clearance, stamp, detail = found
        anchor, body_side, _ = path_anchor(centre, times, path, velocities)
        body = Body(name, role, centre, radius)
        body.kind = obstacle["type"]
        body.clearance = clearance
        body.anchor = anchor
        body.side = body_side
        placed.append(body)
        results[name] = centre
        print("placed %-9s %-6s at (%+9.6f, %+9.6f) clearance %.4f  seed %s -> t*=%.2f s"
              % (name, role, centre[0], centre[1], clearance,
                 "%.1f s" % stamp if role != "tip" else "extreme %.1f s/%+.1f deg"
                 % (stamp, detail), anchor))

    manifest = json.loads(json.dumps(catalog))
    for obstacle in manifest["obstacles"]:
        centre = results[obstacle["name"]]
        obstacle["position"] = [
            float(round(centre[0], 6)),
            float(round(centre[1], 6)),
            float(obstacle["position"][2]),
        ]
    return manifest


# ----------------------------------------------------------------------- main
def build_config(arguments):
    return Config(
        arguments.amplitude,
        arguments.half_height,
        arguments.rate,
        arguments.blend,
        arguments.spin,
        arguments.dt,
        arguments.laps,
    )


def cross_check_overlay(path):
    """Assert the embedded routed schedule still equals the deployed overlay."""
    import yaml  # optional: only needed for --overlay

    overlay = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    schedule = overlay["exploration"]["formation_schedule"]
    entries = tuple(
        (float(entry["t"]), tuple(tuple(float(v) for v in slot) for slot in entry["offsets"]))
        for entry in schedule["entries"]
    )
    if entries != ROUTED_SCHEDULE:
        raise SystemExit("overlay cross-check: routed schedule differs from the embedded table")
    if float(schedule["blend_s"]) != 26.0:
        raise SystemExit("overlay cross-check: blend_s is %s, expected 26.0" % schedule["blend_s"])
    print("overlay cross-check: routed schedule and blend_s match the embedded table")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--verify", action="store_true", default=True,
                      help="score an existing manifest, move nothing (default)")
    mode.add_argument("--generate", action="store_true",
                      help="re-place every body and write a new manifest")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST,
                        help="manifest to verify, or the body catalogue for --generate")
    parser.add_argument("--out", type=Path, help="output manifest for --generate")
    parser.add_argument("--overlay", type=Path,
                        help="scout_mini_planning_geometry.yaml to cross-check the schedule")
    parser.add_argument("--amplitude", type=float, default=6.0, help="leader A")
    parser.add_argument("--half-height", type=float, default=3.0, help="leader B/2")
    parser.add_argument("--rate", type=float, default=0.05, help="leader w")
    parser.add_argument("--blend", type=float, default=26.0, help="cosine morph ramp, s")
    parser.add_argument("--spin", type=float, default=0.02, help="rigid formation spin, rad/s")
    parser.add_argument("--dt", type=float, default=0.05, help="slot sweep step, s")
    parser.add_argument("--laps", type=float, default=5.0, help="laps of slot sweep")
    parser.add_argument("--scan-min", type=float, default=0.6)
    parser.add_argument("--scan-max", type=float, default=9.0)
    parser.add_argument("--scan-step", type=float, default=0.005)
    parser.add_argument("--tip-scan-max", type=float, default=10.0)
    parser.add_argument("--fan-span", type=float, default=88.0, help="tip fan half-span, deg")
    parser.add_argument("--fan-step", type=float, default=1.0, help="tip fan step, deg")
    parser.add_argument("--anchor-slide", type=float, default=40.0,
                        help="how far the anchor may slide along the path, s")
    parser.add_argument("--anchor-slide-step", type=float, default=0.5)
    arguments = parser.parse_args(argv)

    if arguments.overlay:
        cross_check_overlay(arguments.overlay)

    config = build_config(arguments)
    catalog = json.loads(arguments.manifest.read_text(encoding="utf-8"))
    points, stamps, agents = slot_sweep(config)
    nearest = NearestSlot(points)

    if arguments.generate:
        if arguments.out is None:
            raise SystemExit("--generate requires --out (the shipped manifest is never touched)")
        if arguments.out.resolve() == arguments.manifest.resolve():
            raise SystemExit("--out must differ from --manifest")
        print("generating placement from catalogue %s" % arguments.manifest)
        manifest = generate(catalog, config, nearest, arguments)
        arguments.out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print("wrote %s" % arguments.out)
        print()
        catalog = manifest

    print("verifying %s" % (arguments.out if arguments.generate else arguments.manifest))
    print("scene %s, %d bodies" % (catalog["scene"], len(catalog["obstacles"])))
    print()
    bodies = load_bodies(catalog)
    report = Report()
    evaluate(bodies, config, nearest, stamps, agents, report)
    if report.failures:
        print("RESULT: %d CHECK(S) FAILED" % len(report.failures))
        for failure in report.failures:
            print("  - %s" % failure)
        return 1
    print("RESULT: ALL CHECKS PASS (%d bodies, %d band checks, 5 global checks)"
          % (len(bodies), len(bodies)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
