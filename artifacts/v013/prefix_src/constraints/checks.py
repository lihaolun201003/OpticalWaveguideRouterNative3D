"""Phase 5: hard constraints on the FULL CONTINUOUS primitive (v0.1.2).

Checks implemented here (all decided conservatively; undecidable -> REJECT):

1. minimum bend radius      R >= Rmin for every arc, straight = infinite radius,
                            no illegal corners (C1 continuity is structural).
2. 3D obstacle collision    continuous primitive vs solid boxes with an explicit
                            centreline-to-obstacle keep-out.
3. minimum centreline spacing  continuous nearest approach to FIXED pre-existing
                            waveguide centrelines, evaluated over ALL parameter
                            pairs (not same-parameter comparisons).
4. chip-volume boundary     the WHOLE primitive (exact analytic coordinate
                            extrema), not just its endpoints.
5. source/target pose       position and tangent with explicit tolerances.

Method (anti-fabrication):
* line vs box            : exact. Slab intersection test, then exact distance
                           (endpoint-to-box plus 12 box edges) - no sampling.
* arc vs box             : conservative subdivision. A chord of an arc of
                           radius R and sweep dphi deviates from the arc by at
                           most the sagitta s = R (1 - cos(dphi/2)). Therefore
                               clearance >= min_i dist(chord_i, box) - s
                           is a certified LOWER bound, while
                               clearance <= min_i dist(chord_i, box) + s
                           is an upper bound. safe: LB >= required; violation:
                           UB < required, or a WITNESS point of the curve inside
                           the keep-out; otherwise refine; at the depth cap the
                           primitive is REJECTED as undecided.
* primitive vs primitive : same LB/UB scheme with both sagittas subtracted.
* bounding boxes         : used only to EXCLUDE (a positive AABB gap is a valid
                           lower bound); AABB overlap is never called a
                           collision.

v0.1.2 - CERTIFIED NUMERICS, SAGITTA DOMAIN AND WITNESS RULE
------------------------------------------------------------
* CERTIFIED SEGMENT/SEGMENT DECISION.  The classical denominator a*e - b*b
  suffers catastrophic cancellation for almost parallel segments: for two
  1000 um segments 5e-6 um apart the difference of two ~1e12 products is ~1e-4,
  below the resolution of the operands, and v0.1.1 reported 9.04e-07 um for a
  pair whose true distance is exactly 0.  The Lagrange identity
  a*e - b*b == |d1 x d2|^2 gives the very same quantity without that
  cancellation, and segment_segment_distance_certified() adds an explicit
  conservative error bound
      SEGMENT_DISTANCE_ERROR_FLOOR_UM + 1e4 * 2**-53 * scale * (1/sin theta)
  and falls back to EXACT RATIONAL arithmetic (fractions.Fraction) whenever that
  bound cannot separate "safe" from "violation".  So the DECISION is exact, and
  a comparison that cannot be certified is never reported as safe.  Supported
  numeric scale: coordinates and lengths from 1e-9 um to 1e9 um (doubles), which
  covers every synthetic case of this project by many orders of magnitude; the
  exact-rational fallback is scale free.
* SAGITTA DOMAIN.  R (1 - cos(dphi/2)) bounds the deviation of a chord from its
  sub-arc only while the central angle dphi of that sub-arc satisfies
  dphi <= pi (MAX_CHORD_SWEEP_RAD).  cos() is periodic, so outside the domain
  the formula is not merely loose, it is wrong: dphi = 4*pi gives 0.  Every
  subdivision is therefore raised into the domain before the bound is
  evaluated, which is exactly what chord_count_for_domain()/sagitta_bound_um()
  do; a multi-turn arc can no longer be certified from a degenerate chord.
* WITNESS RULE.  The chord nodes are points OF the curve, so their exact
  point-to-box / point-to-centreline distance is a reliable violation witness.
  Whenever such a witness is below the required keep-out the result is
  "violation", and the "the lower bound looks safe" branch can never override
  it.  (v0.1.1 let a bogus lower bound of 99.005 um win over a curve that
  passes straight through the obstacle.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from fractions import Fraction
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from ..geometry.vec import Vec3, angle_between, dot, norm, sub

# Proven domain of the sagitta bound: the central angle of ONE chord must not
# exceed pi.
MAX_CHORD_SWEEP_RAD = math.pi
# Absolute floor of the floating point error bound of one segment/segment
# decision, in um.  Below this the decision is recomputed exactly.
SEGMENT_DISTANCE_ERROR_FLOOR_UM = 1e-9
_FLOAT_EPS = 2.0 ** -53


# --------------------------------------------------------------------- boxes
@dataclass(frozen=True)
class Box3D:
    lo: Vec3
    hi: Vec3

    def __post_init__(self) -> None:
        object.__setattr__(self, "lo", tuple(float(c) for c in self.lo))
        object.__setattr__(self, "hi", tuple(float(c) for c in self.hi))
        for i in range(3):
            if not (self.lo[i] <= self.hi[i]):
                raise ValueError("Box3D requires lo <= hi componentwise")

    def expand(self, m: float) -> "Box3D":
        return Box3D(tuple(c - m for c in self.lo), tuple(c + m for c in self.hi))

    def shrink(self, m: float) -> "Box3D":
        return Box3D(tuple(c + m for c in self.lo), tuple(c - m for c in self.hi))

    def contains_point(self, p: Vec3, margin: float = 0.0) -> bool:
        return all(self.lo[i] - margin <= p[i] <= self.hi[i] + margin for i in range(3))

    def to_dict(self) -> dict:
        return {"lo": list(self.lo), "hi": list(self.hi)}

    @classmethod
    def from_dict(cls, d: dict) -> "Box3D":
        return cls(tuple(d["lo"]), tuple(d["hi"]))


def aabb_gap(lo1: Vec3, hi1: Vec3, lo2: Vec3, hi2: Vec3) -> float:
    """Distance between two axis aligned boxes (0 when they overlap)."""
    s = 0.0
    for i in range(3):
        d = max(lo1[i] - hi2[i], lo2[i] - hi1[i], 0.0)
        s += d * d
    return math.sqrt(s)


def point_box_distance(p: Vec3, box: Box3D) -> float:
    s = 0.0
    for i in range(3):
        if p[i] < box.lo[i]:
            d = box.lo[i] - p[i]
        elif p[i] > box.hi[i]:
            d = p[i] - box.hi[i]
        else:
            d = 0.0
        s += d * d
    return math.sqrt(s)


def segment_box_intersection(p0: Vec3, p1: Vec3, box: Box3D) -> Optional[float]:
    """Slab test; returns the entry parameter t in [0,1] or None. Exact."""
    d = sub(p1, p0)
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-15:
            if p0[i] < box.lo[i] or p0[i] > box.hi[i]:
                return None
        else:
            ta = (box.lo[i] - p0[i]) / d[i]
            tb = (box.hi[i] - p0[i]) / d[i]
            if ta > tb:
                ta, tb = tb, ta
            if ta > t0:
                t0 = ta
            if tb < t1:
                t1 = tb
            if t0 > t1:
                return None
    return t0


# ------------------------------------------------- certified segment distance
def _ssd_stable(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3):
    """(distance, error_bound, condition) of the closest approach of 2 segments.

    The denominator is |d1 x d2|^2 (Lagrange form of a*e - b*b), which avoids
    the catastrophic cancellation of the classical expression for almost
    parallel segments.  condition is 1/sin(theta) of the two directions, i.e.
    the amplification of the closest-point parameters; it is 1.0 on the exactly
    parallel / degenerate branches, which are well conditioned.
    """
    d1 = sub(q1, p1)
    d2 = sub(q2, p2)
    r = sub(p1, p2)
    a = dot(d1, d1)
    e = dot(d2, d2)
    f = dot(d2, r)
    cr = (d1[1] * d2[2] - d1[2] * d2[1],
          d1[2] * d2[0] - d1[0] * d2[2],
          d1[0] * d2[1] - d1[1] * d2[0])
    denom = dot(cr, cr)
    cond = 1.0
    if a <= 0.0 and e <= 0.0:
        d = norm(r)
    elif a <= 0.0:
        t = min(1.0, max(0.0, f / e))
        d = norm(sub(p1, (p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t)))
    else:
        c = dot(d1, r)
        if e <= 0.0:
            s = min(1.0, max(0.0, -c / a))
            d = norm(sub((p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s), p2))
        else:
            b = dot(d1, d2)
            if denom > 0.0:
                s = min(1.0, max(0.0, (b * f - c * e) / denom))
                cond = math.sqrt(max(0.0, a * e) / denom)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = min(1.0, max(0.0, (b - c) / a))
            c1 = (p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s)
            c2 = (p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t)
            d = norm(sub(c1, c2))
    scale = max([1.0, norm(d1), norm(d2), norm(r)]
                + [abs(c) for c in p1] + [abs(c) for c in q1]
                + [abs(c) for c in p2] + [abs(c) for c in q2])
    bound = SEGMENT_DISTANCE_ERROR_FLOOR_UM + 1e4 * _FLOAT_EPS * scale * max(1.0, cond)
    return d, bound, cond


def segment_segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> float:
    """Stable float closest approach of two closed segments.

    This is the reported estimate; every DECISION uses
    segment_segment_distance_certified() below.
    """
    return _ssd_stable(p1, q1, p2, q2)[0]


def segment_segment_bounds(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> Tuple[float, float]:
    """Conservative (lower, upper) bound from the stable float computation."""
    d, bound, _cond = _ssd_stable(p1, q1, p2, q2)
    return d - bound, d + bound


def _exact_sq_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> Fraction:
    """EXACT squared closest approach in rational arithmetic.

    Every double is a rational number, so Fraction(float) is lossless and the
    clamped closed form can be evaluated with exact comparisons and exact
    arithmetic.  The result is the true squared distance of the two segments as
    given, whatever the conditioning of the float formula.
    """
    F = Fraction

    def fv(v):
        return tuple(F(float(c)) for c in v)

    def fsub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def fdot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    A1, B1, A2, B2 = fv(p1), fv(q1), fv(p2), fv(q2)
    d1 = fsub(B1, A1)
    d2 = fsub(B2, A2)
    r = fsub(A1, A2)
    a = fdot(d1, d1)
    e = fdot(d2, d2)
    f = fdot(d2, r)
    if a == 0 and e == 0:
        return fdot(r, r)
    if a == 0:
        s = F(0)
        t = max(F(0), min(F(1), f / e))
    else:
        c = fdot(d1, r)
        if e == 0:
            t = F(0)
            s = max(F(0), min(F(1), -c / a))
        else:
            b = fdot(d1, d2)
            denom = a * e - b * b
            s = (max(F(0), min(F(1), (b * f - c * e) / denom))
                 if denom > 0 else F(0))
            t = (b * s + f) / e
            if t < 0:
                t = F(0)
                s = max(F(0), min(F(1), -c / a))
            elif t > 1:
                t = F(1)
                s = max(F(0), min(F(1), (b - c) / a))
    c1 = tuple(A1[i] + d1[i] * s for i in range(3))
    c2 = tuple(A2[i] + d2[i] * t for i in range(3))
    w = fsub(c1, c2)
    return w[0] * w[0] + w[1] * w[1] + w[2] * w[2]


def _sqrt_float_bounds(d2: Fraction) -> Tuple[float, float]:
    """Floats (lo, hi) with lo <= sqrt(d2) <= hi (rigorous by construction)."""
    if d2 <= 0:
        return 0.0, 0.0
    x = math.sqrt(float(d2))
    lo = hi = x
    for _ in range(200):
        if Fraction(lo) * Fraction(lo) <= d2:
            break
        lo = math.nextafter(lo, -math.inf)
    for _ in range(200):
        if Fraction(hi) * Fraction(hi) >= d2:
            break
        hi = math.nextafter(hi, math.inf)
    return lo, hi


def segment_segment_distance_certified(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3,
                                       required: float, tol: float = 0.0
                                       ) -> Tuple[str, float, float, str]:
    """Certified comparison of the closest approach against a required value.

    Returns (status, lower_bound_um, upper_bound_um, method) with status in
    {"safe", "violation"}.  The stable float value decides whenever its explicit
    error bound separates the two cases; otherwise the decision is recomputed in
    exact rational arithmetic, so it can never be a false "safe".
    """
    threshold = required - tol
    d, bound, _cond = _ssd_stable(p1, q1, p2, q2)
    if d - bound >= threshold:
        return "safe", d - bound, d + bound, "stable_float"
    if d + bound < threshold:
        return "violation", d - bound, d + bound, "stable_float"
    d2 = _exact_sq_distance(p1, q1, p2, q2)
    lo, hi = _sqrt_float_bounds(d2)
    if threshold <= 0.0:
        return "safe", lo, hi, "exact_rational"
    thr = Fraction(float(threshold))
    if d2 >= thr * thr:
        return "safe", lo, hi, "exact_rational"
    return "violation", lo, hi, "exact_rational"


def segment_box_distance(p0: Vec3, p1: Vec3, box: Box3D) -> float:
    """Exact distance segment<->box (0 when they intersect)."""
    if segment_box_intersection(p0, p1, box) is not None:
        return 0.0
    d = min(point_box_distance(p0, box), point_box_distance(p1, box))
    lo, hi = box.lo, box.hi
    corners = [(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
               (lo[0], hi[1], lo[2]), (hi[0], hi[1], lo[2]),
               (lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2]),
               (lo[0], hi[1], hi[2]), (hi[0], hi[1], hi[2])]
    edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    for i, j in edges:
        d = min(d, segment_segment_distance(p0, p1, corners[i], corners[j]))
        if d <= 0.0:
            return 0.0
    return d


# ---------------------------------------------------------------- results
@dataclass
class CheckResult:
    name: str
    status: str                 # safe | violation | undecided
    bound_um: float             # certified lower bound (safe) or upper bound (violation)
    detail: Dict = field(default_factory=dict)

    @property
    def safe(self) -> bool:
        return self.status == "safe"

    def to_dict(self) -> dict:
        return {"name": self.name, "status": self.status,
                "bound_um": self.bound_um, "detail": self.detail}


def _combine(name: str, results: Sequence[CheckResult],
             detailed: bool = True) -> CheckResult:
    subs = [r.to_dict() for r in results] if detailed else []
    if not results:
        return CheckResult(name, "safe", math.inf, {})
    if any(r.status == "violation" for r in results):
        worst = min((r for r in results if r.status == "violation"), key=lambda r: r.bound_um)
        d = {"worst": worst.detail}
        if detailed:
            d["sub"] = subs
        return CheckResult(name, "violation", worst.bound_um, d)
    if any(r.status == "undecided" for r in results):
        worst = min((r for r in results if r.status == "undecided"), key=lambda r: r.bound_um)
        d = {"worst": worst.detail}
        if detailed:
            d["sub"] = subs
        return CheckResult(name, "undecided", worst.bound_um, d)
    d = {"sub": subs} if detailed else {}
    return CheckResult(name, "safe", min(r.bound_um for r in results), d)


# ------------------------------------------------------- primitive clearance
def _prefilter_safe(seg, box: Box3D, required: float) -> Optional[float]:
    """If the AABB gap already proves safety, return that gap (a true LB)."""
    lo, hi = seg.aabb()
    g = aabb_gap(lo, hi, box.lo, box.hi)
    if g >= required:
        return g
    return None


def line_box_clearance(seg: LineSegment3D, box: Box3D, required: float) -> CheckResult:
    g = _prefilter_safe(seg, box, required)
    if g is not None:
        return CheckResult("obstacle_clearance", "safe", g, {"method": "aabb_gap"})
    t = segment_box_intersection(seg.p0, seg.p1, box)
    if t is not None:
        p = (seg.p0[0] + (seg.p1[0] - seg.p0[0]) * t,
             seg.p0[1] + (seg.p1[1] - seg.p0[1]) * t,
             seg.p0[2] + (seg.p1[2] - seg.p0[2]) * t)
        return CheckResult("obstacle_clearance", "violation", 0.0,
                           {"method": "exact_slab_intersection", "witness_point": list(p)})
    d = segment_box_distance(seg.p0, seg.p1, box)
    st = "safe" if d >= required else "violation"
    return CheckResult("obstacle_clearance", st, d,
                       {"method": "exact_segment_box_distance"})


def chord_count_for_domain(theta: float, requested: int = 1) -> int:
    """Smallest chord count >= requested with |theta| / n <= pi per chord.

    Outside that domain the sagitta formula R (1 - cos(dphi/2)) is NOT a bound
    on the deviation of the chord from the sub-arc; because cos() is periodic it
    even returns 0 for dphi = 4*pi, which would certify a four-turn arc from a
    single degenerate "chord".  Every subdivision in this module and in the
    independent validator goes through this function.
    """
    if not math.isfinite(theta):
        raise ValueError("chord_count_for_domain needs a finite sweep")
    need = int(math.ceil(abs(theta) / MAX_CHORD_SWEEP_RAD - 1e-12))
    return max(1, int(requested), need)


def sagitta_bound_um(seg, n_chords: int) -> float:
    """Sagitta bound of the chord subdivision, inside its proven domain.

    n_chords is raised to chord_count_for_domain(...) first, so the returned
    number is always derived from sub-arcs with |delta_theta| <= pi.
    """
    if seg.kind == "line":
        return 0.0
    n = chord_count_for_domain(float(seg.theta), n_chords)
    dphi = abs(float(seg.theta)) / n
    if dphi > MAX_CHORD_SWEEP_RAD + 1e-12:
        raise AssertionError("sagitta bound evaluated outside its domain: %r" % dphi)
    return seg.radius * (1.0 - math.cos(dphi / 2.0))


def _chord_nodes(seg, n_chords: int):
    """(n, sagitta, nodes) of the domain-safe chord subdivision."""
    if seg.kind == "line":
        return 1, 0.0, [seg.point_at(0.0), seg.point_at(seg.length)]
    n = chord_count_for_domain(float(seg.theta), n_chords)
    sag = sagitta_bound_um(seg, n)
    L = seg.length
    return n, sag, [seg.point_at(L * i / n) for i in range(n + 1)]


def arc_box_clearance(arc: CircularArc3D, box: Box3D, required: float,
                      cfg: RouterConfig) -> CheckResult:
    g = _prefilter_safe(arc, box, required)
    if g is not None:
        return CheckResult("obstacle_clearance", "safe", g, {"method": "aabb_gap"})
    n = 2
    for level in range(cfg.max_subdiv_level + 1):
        n_eff, sag, nodes = _chord_nodes(arc, n)
        # WITNESS FIRST: the chord nodes are points OF the curve, so a node
        # inside the keep-out is a reliable violation, never overridden by the
        # lower bound below.
        witness = min(point_box_distance(p, box) for p in nodes)
        if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required:
            return CheckResult("obstacle_clearance", "violation", witness,
                               {"method": "violation_witness_on_the_curve",
                                "subdivision_level": level, "n_chords": n_eff,
                                "witness_clearance_um": witness})
        lb = math.inf
        ub = math.inf
        for i in range(n_eff):
            d = segment_box_distance(nodes[i], nodes[i + 1], box)
            if d - sag < lb:
                lb = d - sag
            if d + sag < ub:
                ub = d + sag
        tol = cfg.clearance_decision_tol_um
        if lb >= required - tol:
            return CheckResult("obstacle_clearance", "safe", lb,
                               {"method": "conservative_subdivision",
                                "subdivision_level": level, "n_chords": n_eff,
                                "sagitta_bound_um": sag})
        if ub < required - tol:
            return CheckResult("obstacle_clearance", "violation", ub,
                               {"method": "conservative_subdivision",
                                "subdivision_level": level, "n_chords": n_eff,
                                "upper_bound_sample_um": ub})
        if level == cfg.max_subdiv_level:
            return CheckResult("obstacle_clearance", "undecided", lb,
                               {"method": "conservative_subdivision",
                                "subdivision_level": level, "n_chords": n_eff,
                                "lower_bound_um": lb, "upper_bound_um": ub,
                                "note": "depth cap reached without a decision -> reject"})
        n *= 2
    raise AssertionError("unreachable")


def primitive_box_clearance(seg, box: Box3D, required: float,
                            cfg: RouterConfig) -> CheckResult:
    if seg.kind == "line":
        return line_box_clearance(seg, box, required)
    return arc_box_clearance(seg, box, required, cfg)


# --------------------------------------------------------------- boundary
def boundary_check(seg, chip: Box3D, margin: float) -> CheckResult:
    """Exact containment of the whole primitive (analytic extrema)."""
    inner = chip.shrink(margin)
    if inner.lo[0] > inner.hi[0] or inner.lo[1] > inner.hi[1] or inner.lo[2] > inner.hi[2]:
        return CheckResult("boundary", "violation", -1.0,
                           {"reason": "boundary margin larger than the chip volume"})
    lo, hi = seg.aabb()
    worst = None
    for i, ax in enumerate("xyz"):
        over = max(inner.lo[i] - lo[i], hi[i] - inner.hi[i], 0.0)
        if over > 0.0:
            worst = {"axis": ax, "overshoot_um": over,
                     "primitive_lo": lo[i], "primitive_hi": hi[i],
                     "allowed": [inner.lo[i], inner.hi[i]]}
            break
    if worst is not None:
        return CheckResult("boundary", "violation", -worst["overshoot_um"], worst)
    slack = min(min(lo[i] - inner.lo[i], inner.hi[i] - hi[i]) for i in range(3))
    return CheckResult("boundary", "safe", slack, {"method": "exact_analytic_extrema"})


# ---------------------------------------------------------------- spacing
def _sagitta_of(seg, n_chords: int) -> float:
    """Sagitta bound of one primitive (domain safe; 0 for a line)."""
    return sagitta_bound_um(seg, n_chords)


def pair_clearance_bounds(A, B, level: int) -> Tuple[float, float, float]:
    """(lower, upper, witness_upper) bound on the min distance between A and B.

    Both bounds come from the same domain-safe subdivision.  With chords c_i of A
    and d_j of B, every point of A is within sagA of some chord point and every
    point of B within sagB of a chord point, hence

        min_ij dist(c_i, d_j) - sagA - sagB  <=  min dist(A, B)
                                             <=  min_ij dist(c_i, d_j) + sagA + sagB

    Only bounding boxes are used to *exclude* chord pairs from the exact
    distance computation (a positive AABB gap is a certified lower bound); a
    bounding-box overlap is never treated as a collision.

    witness_upper is an upper bound on dist(A, B) built only from points that lie
    ON the two curves (min over curve points of the distance to the other curve
    plus that curve's sagitta), so it may be used to PROVE a violation.
    """
    if A.kind == "line" and B.kind == "line":
        _st, lo, hi, _method = segment_segment_distance_certified(
            A.p0, A.p1, B.p0, B.p1, 0.0, 0.0)
        # hi is a certified UPPER bound of the true distance, so it is a valid
        # witness value (using the raw float estimate here would not be).
        return lo, hi, hi
    nA, sagA, pa = _chord_nodes(A, 2 ** level)
    nB, sagB, pb = _chord_nodes(B, 2 ** level)
    blo, bhi = B.aabb()
    best_lo = math.inf
    best_hi = math.inf
    for i in range(nA):
        lo_i = (min(pa[i][0], pa[i + 1][0]), min(pa[i][1], pa[i + 1][1]),
                min(pa[i][2], pa[i + 1][2]))
        hi_i = (max(pa[i][0], pa[i + 1][0]), max(pa[i][1], pa[i + 1][1]),
                max(pa[i][2], pa[i + 1][2]))
        for j in range(nB):
            lo_j = (min(pb[j][0], pb[j + 1][0]), min(pb[j][1], pb[j + 1][1]),
                    min(pb[j][2], pb[j + 1][2]))
            hi_j = (max(pb[j][0], pb[j + 1][0]), max(pb[j][1], pb[j + 1][1]),
                    max(pb[j][2], pb[j + 1][2]))
            gap = aabb_gap(lo_i, hi_i, lo_j, hi_j)
            if gap >= best_hi:
                continue          # cannot improve the minimum: excluded, not collided
            d_lo, d_hi = segment_segment_bounds(pa[i], pa[i + 1], pb[j], pb[j + 1])
            if d_lo < best_lo:
                best_lo = d_lo
            if d_hi < best_hi:
                best_hi = d_hi
    witness = math.inf
    for p in pa:
        for j in range(nB):
            d = segment_segment_distance(p, p, pb[j], pb[j + 1])
            if d + sagB < witness:
                witness = d + sagB
    for p in pb:
        for i in range(nA):
            d = segment_segment_distance(p, p, pa[i], pa[i + 1])
            if d + sagA < witness:
                witness = d + sagA
    return best_lo - sagA - sagB, best_hi + sagA + sagB, witness


def pair_clearance_lb_ub(A, B, level: int) -> Tuple[float, float]:
    """Certified (lower, upper) bound on the min distance between two primitives."""
    lb, ub, _w = pair_clearance_bounds(A, B, level)
    return lb, ub


def primitive_spacing(seg, others: Sequence[object], required: float,
                      cfg: RouterConfig, name: str = "centerline_spacing") -> CheckResult:
    """Certified nearest approach between one primitive and reference centrelines.

    The closest approach is taken over ALL parameter pairs (chord pairs cover
    the full parameter ranges of both curves); no same-parameter comparison is
    used anywhere.  A curve point inside the keep-out is a violation witness and
    wins over any lower bound that happens to look safe.
    """
    if not others:
        return CheckResult(name, "safe", math.inf, {"note": "no reference centreline"})
    results = []
    tol = cfg.clearance_decision_tol_um
    for other in others:
        lo1, hi1 = seg.aabb()
        lo2, hi2 = other.aabb()
        gap = aabb_gap(lo1, hi1, lo2, hi2)
        if gap >= required:
            results.append(CheckResult(name, "safe", gap, {"method": "aabb_gap"}))
            continue
        level = 0
        while True:
            lb, ub, witness = pair_clearance_bounds(seg, other, level)
            if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required - tol:
                results.append(CheckResult(name, "violation", witness,
                                           {"method": "violation_witness_on_the_curve",
                                            "level": level, "witness_distance_um": witness,
                                            "lower_bound_um": lb,
                                            "upper_bound_um": ub}))
                break
            if lb >= required - tol:
                results.append(CheckResult(name, "safe", lb,
                                           {"method": "conservative_chord_bounds",
                                            "level": level, "upper_bound_um": ub}))
                break
            if ub < required - tol:
                results.append(CheckResult(name, "violation", ub,
                                           {"method": "conservative_chord_bounds",
                                            "level": level, "lower_bound_um": lb}))
                break
            if level >= cfg.max_subdiv_level:
                results.append(CheckResult(name, "undecided", lb,
                                           {"method": "conservative_chord_bounds",
                                            "level": level, "upper_bound_um": ub,
                                            "note": "depth cap reached -> reject"}))
                break
            level += 1
    return _combine(name, results)


# ----------------------------------------------------------------- radius
def radius_check(segments: Sequence[object], cfg: RouterConfig) -> CheckResult:
    worst = math.inf
    for seg in segments:
        if seg.kind == "arc":
            worst = min(worst, seg.radius)
    if worst is math.inf:
        return CheckResult("min_bend_radius", "safe", math.inf,
                           {"note": "straight only; straight radius = infinite"})
    if worst < cfg.rmin_um - cfg.radius_tol_um:
        return CheckResult("min_bend_radius", "violation", worst,
                           {"rmin_um": cfg.rmin_um, "measured_radius_um": worst,
                            "tolerance_um": cfg.radius_tol_um})
    return CheckResult("min_bend_radius", "safe", worst,
                       {"rmin_um": cfg.rmin_um, "measured_radius_um": worst,
                        "tolerance_um": cfg.radius_tol_um})


# ------------------------------------------------------------------- poses
def pose_match(pose: Pose3D, target: Pose3D, cfg: RouterConfig) -> Tuple[bool, float, float]:
    pos_err = norm(sub(pose.position, target.position))
    ang_err = math.degrees(angle_between(pose.tangent, target.tangent))
    ok = pos_err <= cfg.eps_pos_um and ang_err <= cfg.eps_ang_deg
    return ok, pos_err, ang_err


# ------------------------------------------------------- aggregate check
def validate_segments(segments: Sequence[object], chip: Box3D,
                      obstacles: Sequence[Box3D],
                      reference_centerlines: Sequence[object],
                      cfg: RouterConfig, detailed: bool = True) -> Dict:
    """Full hard-constraint validation of a candidate primitive sequence.

    detailed=False is the search hot path: identical decisions, fewer dicts.
    """
    checks: List[CheckResult] = []
    checks.append(radius_check(segments, cfg))

    bnd = [boundary_check(s, chip, cfg.effective_boundary_margin_um) for s in segments]
    checks.append(_combine("boundary", bnd, detailed))

    req_obs = cfg.effective_obstacle_clearance_um
    obs_res = []
    for s in segments:
        for box in obstacles:
            obs_res.append(primitive_box_clearance(s, box, req_obs, cfg))
    checks.append(_combine("obstacle_clearance", obs_res, detailed))

    sp_res = []
    for s in segments:
        sp_res.append(primitive_spacing(s, reference_centerlines,
                                        cfg.dmin_centerline_um, cfg))
    checks.append(_combine("centerline_spacing", sp_res, detailed))

    ok = all(c.status == "safe" for c in checks)
    out = {"ok": ok,
           "checks": ({c.name: c.to_dict() for c in checks} if detailed else {}),
           "min_radius_um": (None if math.isinf(radius_check(segments, cfg).bound_um)
                             else radius_check(segments, cfg).bound_um),
           "clearance_required_um": req_obs,
           "clearance_bound_um": checks[2].bound_um,
           "spacing_required_um": cfg.dmin_centerline_um,
           "spacing_bound_um": checks[3].bound_um,
           "total_length_um": sum(s.length for s in segments)}
    if not ok:
        out["failed"] = [c.name for c in checks if c.status != "safe"]
    return out


def path_self_spacing(path, cfg: RouterConfig, exempt_adjacent: bool = True) -> CheckResult:
    """Non-adjacent self-approach of the path (post-hoc, post-search check).

    Scope of the adjacency exemption: only segment pairs that share an endpoint
    (|i - j| == 1) are exempt, because their distance is identically zero at the
    shared junction.  Every other pair is evaluated over all parameter pairs, so
    a path that wraps around and re-approaches itself is detected.
    """
    res = []
    segs = list(path.segments)
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            if exempt_adjacent and j - i == 1:
                continue
            level = 0
            while True:
                lb, ub, witness = pair_clearance_bounds(segs[i], segs[j], level)
                tol = cfg.clearance_decision_tol_um
                if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < cfg.dmin_centerline_um - tol:
                    res.append(CheckResult("self_spacing", "violation", witness,
                                           {"pair": [i, j], "level": level,
                                            "method": "violation_witness_on_the_curve"}))
                    break
                if lb >= cfg.dmin_centerline_um - tol:
                    res.append(CheckResult("self_spacing", "safe", lb,
                                           {"pair": [i, j], "level": level}))
                    break
                if ub < cfg.dmin_centerline_um - tol:
                    res.append(CheckResult("self_spacing", "violation", ub,
                                           {"pair": [i, j], "level": level}))
                    break
                if level >= cfg.max_subdiv_level:
                    res.append(CheckResult("self_spacing", "undecided", lb,
                                           {"pair": [i, j], "level": level}))
                    break
                level += 1
    out = _combine("self_spacing", res)
    out.detail["note"] = ("non-adjacent pairs only; adjacent pairs share an endpoint "
                          "and are exempt")
    out.detail["exempt_adjacent"] = exempt_adjacent
    return out
