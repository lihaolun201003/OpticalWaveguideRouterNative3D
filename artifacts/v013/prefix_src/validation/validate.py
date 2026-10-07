"""Independent validation entry point (v0.1.2).

This module is deliberately self-contained: it re-implements curve evaluation,
distance queries and conservative subdivision from the SERIALISED JSON of the
raw searched path.  It imports nothing from native3d.geometry, -search or
-constraints, so it is an independent second implementation that can be run by
anyone holding only an artifact file:

    python -m native3d.validation.validate artifacts/cases/<file>.json

CERTIFICATION (v0.1.2)
----------------------
Four kinds of number are reported and they are never conflated:

* SAMPLED estimate        - a value measured at finitely many points.  A sampled
  MINIMUM is an UPPER bound on the true minimum distance (it can only be too
  large), while a sampled point outside the chip, inside an obstacle or closer
  than the required keep-out to a reference centreline is a reliable VIOLATION
  witness.
* ANALYTIC distance       - the exact value where a closed form exists
  (segment/segment and segment/AABB distance, and the exact coordinate extrema
  of an arc), computed here without any sampling.
* CONSERVATIVE LOWER BOUND - a certified lower bound on the true minimum, i.e. a
  number that can never exceed it: exact for line/line and line/AABB, and
  min(chord distance) - sagitta for curves approximated by chords.
* CERTIFICATION STATE     - safe (bound >= required), violation (a witness
  exists, or a bound proves the requirement is missed), undecided (neither could
  be decided inside the refinement cap).  An undecided check is NEVER counted as
  a pass: the verdict is then "uncertified", never "pass".

Hard rules added in v0.1.2 (each one has a regression in tests/):

1. SOURCE POSE.  The path must start at the source pose.  The start position and
   the start tangent have their own tolerances (config source_pos_tol_um and
   source_ang_tol_deg) and both enter the verdict; the target acceptance ball
   (eps_pos_um / eps_ang_deg) is NOT reused for the source.
2. REFERENCE CURVES.  Reference centrelines may be lines OR arcs.  All four
   combinations line/line, line/arc, arc/line and arc/arc are decided, and when
   both sides are curves the certified lower bound subtracts the geometric
   approximation error of BOTH sides.  An unsupported or malformed reference or
   segment kind is REJECTED (ValueError), never silently dropped.
3. SAGITTA DOMAIN.  R (1 - cos(dphi/2)) is only a bound on the deviation of a
   chord from its sub-arc when the central angle of that sub-arc satisfies
   dphi <= pi (MAX_CHORD_SWEEP_RAD).  Every subdivision is raised into that
   domain before the bound is evaluated, so a multi-turn arc (a sweep of 4*pi,
   or 8*pi, or a full turn whose endpoints coincide) can never produce a
   sagitta of 0 through the periodicity of cos().
4. WITNESS FIRST.  A reliable violation witness (a point of the curve inside the
   keep-out) decides "violation" and is never overridden by a lower bound that
   happens to look safe.
5. SELF-CHECK.  A certified lower bound above the sampled upper bound is
   mathematically impossible; if it happens the certification self check fails,
   the verdict stops being "pass" and the JSON says so.
6. NUMERIC DECISION.  Segment/segment decisions use |d1 x d2|^2 (the Lagrange
   form of a*e - b*b, without the catastrophic cancellation) plus an explicit
   conservative error bound; when that bound cannot decide the comparison the
   decision is recomputed in EXACT RATIONAL arithmetic.  A comparison that
   cannot be certified is never reported as safe.

v0.1 printed the sampled minimum of an arc against a box as a "certified lower
bound", which produced lower bounds ABOVE the sampled minimum (mathematically
impossible) and let a path cross a reference centreline and still be reported as
a pass.  See tests/test_repair_c_validator_bounds.py and the v0.1.2 regressions.
"""

from __future__ import annotations

import json
import math
import sys
from fractions import Fraction
from typing import Dict, List, Optional, Sequence, Tuple

Vec = Tuple[float, float, float]

# Refinement cap for chord based bounds (at most 2**MAX_CHORD_LEVEL chords).
MAX_CHORD_LEVEL = 12
# Sampling step of the (upper bound / witness) estimates, in um.
SAMPLE_STEP_UM = 0.25
# Proven domain of the sagitta bound R (1 - cos(dphi/2)): the central angle of
# ONE chord must not exceed pi.
MAX_CHORD_SWEEP_RAD = math.pi
# Segment kinds this validator understands.  Anything else is rejected.
SUPPORTED_KINDS = ("line", "arc")
# Absolute floor of the floating point error bound of one segment/segment
# decision, in um.  Below this the decision is recomputed exactly.
SEGMENT_DISTANCE_ERROR_FLOOR_UM = 1e-9
_FLOAT_EPS = 2.0 ** -53


# --------------------------------------------------------------- vector math
def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


# ------------------------------------------------------ input contract checks
def _require_finite_vec(value, what: str) -> None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("%s must be three numbers, got %r" % (what, value))
    for c in value:
        try:
            f = float(c)
        except (TypeError, ValueError):
            raise ValueError("%s must be three numbers, got %r" % (what, value))
        if not math.isfinite(f):
            raise ValueError("%s must be finite numbers, got %r" % (what, value))


def check_segment_dict(seg: dict, what: str) -> None:
    """Explicitly reject an unsupported or malformed serialised primitive."""
    if not isinstance(seg, dict):
        raise ValueError("%s must be a JSON object, got %r" % (what, type(seg)))
    kind = seg.get("kind")
    if kind not in SUPPORTED_KINDS:
        raise ValueError("%s has unsupported kind %r; supported kinds are %r"
                         % (what, kind, list(SUPPORTED_KINDS)))
    if kind == "line":
        _require_finite_vec(seg.get("p0"), what + ".p0")
        _require_finite_vec(seg.get("p1"), what + ".p1")
        if _norm(_sub(tuple(float(c) for c in seg["p1"]),
                      tuple(float(c) for c in seg["p0"]))) <= 1e-9:
            raise ValueError("%s is a degenerate zero-length line" % what)
    else:
        _require_finite_vec(seg.get("center"), what + ".center")
        _require_finite_vec(seg.get("u"), what + ".u")
        _require_finite_vec(seg.get("v"), what + ".v")
        if "radius_um" not in seg or "theta_rad" not in seg:
            raise ValueError("%s is missing radius_um / theta_rad" % what)
        try:
            radius = float(seg["radius_um"])
        except (TypeError, ValueError):
            raise ValueError("%s.radius_um is not a number" % what)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("%s.radius_um must be finite and > 0, got %r"
                             % (what, seg["radius_um"]))
        try:
            theta = float(seg["theta_rad"])
        except (TypeError, ValueError):
            raise ValueError("%s.theta_rad is not a number" % what)
        if not math.isfinite(theta) or abs(theta) <= 1e-12:
            raise ValueError("%s.theta_rad must be finite and non-zero" % what)
        u = tuple(float(c) for c in seg["u"])
        v = tuple(float(c) for c in seg["v"])
        if abs(_norm(u) - 1.0) > 1e-9 or abs(_norm(v) - 1.0) > 1e-9:
            raise ValueError("%s.u / %s.v must be unit vectors" % (what, what))
        if abs(_dot(u, v)) > 1e-9:
            raise ValueError("%s.u and %s.v must be orthogonal" % (what, what))


# ---------------------------------------------------------- curve evaluation
def seg_length(seg: dict) -> float:
    if seg["kind"] == "line":
        return _norm(_sub(seg["p1"], seg["p0"]))
    return float(seg["radius_um"]) * abs(float(seg["theta_rad"]))


def eval_seg(seg: dict, s: float) -> Vec:
    """Own evaluation of the serialised primitive at arclength s."""
    if seg["kind"] == "line":
        p0, p1 = seg["p0"], seg["p1"]
        L = _norm(_sub(p1, p0))
        t = 0.0 if L == 0 else s / L
        return (p0[0] + (p1[0] - p0[0]) * t,
                p0[1] + (p1[1] - p0[1]) * t,
                p0[2] + (p1[2] - p0[2]) * t)
    c, u, v = seg["center"], seg["u"], seg["v"]
    R, th = float(seg["radius_um"]), float(seg["theta_rad"])
    phi = (s / R) * (1.0 if th > 0 else -1.0)
    cs, sn = math.cos(phi), math.sin(phi)
    return (c[0] + R * (cs * u[0] + sn * v[0]),
            c[1] + R * (cs * u[1] + sn * v[1]),
            c[2] + R * (cs * u[2] + sn * v[2]))


def eval_tangent_exact(seg: dict, s: float) -> Vec:
    """Own analytic unit tangent at arclength s (recomputed from parameters)."""
    if seg["kind"] == "line":
        d = _sub(seg["p1"], seg["p0"])
        n = _norm(d)
        return (d[0] / n, d[1] / n, d[2] / n)
    u, v = seg["u"], seg["v"]
    R, th = float(seg["radius_um"]), float(seg["theta_rad"])
    phi = (s / R) * (1.0 if th > 0 else -1.0)
    cs, sn = math.cos(phi), math.sin(phi)
    sgn = 1.0 if th > 0 else -1.0
    return (sgn * (-sn * u[0] + cs * v[0]),
            sgn * (-sn * u[1] + cs * v[1]),
            sgn * (-sn * u[2] + cs * v[2]))


def eval_tangent_fd(seg: dict, s: float, h: float = 1e-4) -> Vec:
    """Tangent by central finite difference (independent of stored tangents)."""
    L = seg_length(seg)
    a = max(0.0, s - h)
    b = min(L, s + h)
    d = _sub(eval_seg(seg, b), eval_seg(seg, a))
    n = _norm(d)
    return (0.0, 0.0, 1.0) if n == 0 else (d[0] / n, d[1] / n, d[2] / n)


# -------------------------------------------------------------- distance ops
def point_box_distance(p, lo, hi) -> float:
    s = 0.0
    for i in range(3):
        d = lo[i] - p[i] if p[i] < lo[i] else (p[i] - hi[i] if p[i] > hi[i] else 0.0)
        s += d * d
    return math.sqrt(s)


def point_seg_distance(p, a, b) -> float:
    ab = _sub(b, a)
    L2 = _dot(ab, ab)
    if L2 <= 1e-18:
        return _norm(_sub(p, a))
    t = max(0.0, min(1.0, _dot(_sub(p, a), ab) / L2))
    return _norm(_sub(p, _add(a, _mul(ab, t))))


def _ssd_stable(p1, q1, p2, q2):
    """Stable float closest approach of two closed segments.

    The classical denominator a*e - b*b suffers catastrophic cancellation for
    almost parallel segments (two 1000 um segments 5e-6 um apart of direction
    give a*e ~ b*b ~ 1e12 and a difference ~1e-4, below the resolution of the
    operands).  The Lagrange identity a*e - b*b == |d1 x d2|^2 lets the very
    same quantity be computed without that cancellation.

    Returns (distance, error_bound, condition) where condition is the
    amplification factor 1/sin(theta) of the closest-point parameters (1.0 for
    the exactly parallel / degenerate branches, which are well conditioned).
    """
    d1 = _sub(q1, p1)
    d2 = _sub(q2, p2)
    r = _sub(p1, p2)
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    eps = 0.0
    cr = _cross(d1, d2)
    denom = _dot(cr, cr)                 # == a*e - b*b, computed stably
    cond = 1.0
    if a <= eps and e <= eps:
        d = _norm(r)
    elif a <= eps:
        s = 0.0
        t = max(0.0, min(1.0, f / e))
        d = _norm(_sub(p1, _add(p2, _mul(d2, t))))
    else:
        c = _dot(d1, r)
        if e <= eps:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
            d = _norm(_sub(_add(p1, _mul(d1, s)), p2))
        else:
            b = _dot(d1, d2)
            if denom > 0.0:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
                cond = math.sqrt(max(0.0, a * e) / denom)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))
            c1 = _add(p1, _mul(d1, s))
            c2 = _add(p2, _mul(d2, t))
            d = _norm(_sub(c1, c2))
    scale = max([1.0, _norm(d1), _norm(d2), _norm(r)]
                + [abs(c) for c in p1] + [abs(c) for c in q1]
                + [abs(c) for c in p2] + [abs(c) for c in q2])
    bound = SEGMENT_DISTANCE_ERROR_FLOOR_UM + 1e4 * _FLOAT_EPS * scale * max(1.0, cond)
    return d, bound, cond


def seg_seg_distance(p1, q1, p2, q2) -> float:
    """Closest approach of two closed segments (stable float estimate).

    The value is the stable float computation; the CERTIFIED decision (with an
    explicit error bound and an exact-rational fallback) is
    seg_seg_distance_certified below.  Every place in this file that turns a
    distance into a safe/violation statement uses the certified form.
    """
    return _ssd_stable(p1, q1, p2, q2)[0]


def _exact_sq_distance(p1, q1, p2, q2) -> Fraction:
    """EXACT squared closest approach, in rational arithmetic.

    The inputs are doubles and every double is a rational number, so converting
    with Fraction(float) is lossless.  The classical clamped closed form is then
    evaluated with exact comparisons and exact arithmetic, i.e. the returned
    value is the true squared distance of the two segments as given.
    """
    F = Fraction

    def fv(v):
        return tuple(F(float(c)) for c in v)

    d1 = _sub(fv(q1), fv(p1))
    d2 = _sub(fv(q2), fv(p2))
    r = _sub(fv(p1), fv(p2))

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    a = dot(d1, d1)
    e = dot(d2, d2)
    f = dot(d2, r)
    if a == 0 and e == 0:
        return dot(r, r)
    if a == 0:
        s = F(0)
        t = max(F(0), min(F(1), f / e))
    else:
        c = dot(d1, r)
        if e == 0:
            t = F(0)
            s = max(F(0), min(F(1), -c / a))
        else:
            b = dot(d1, d2)
            denom = a * e - b * b
            s = max(F(0), min(F(1), (b * f - c * e) / denom)) if denom > 0 else F(0)
            t = (b * s + f) / e
            if t < 0:
                t = F(0)
                s = max(F(0), min(F(1), -c / a))
            elif t > 1:
                t = F(1)
                s = max(F(0), min(F(1), (b - c) / a))
    c1 = tuple(fv(p1)[i] + d1[i] * s for i in range(3))
    c2 = tuple(fv(p2)[i] + d2[i] * t for i in range(3))
    w = _sub(c1, c2)
    return w[0] * w[0] + w[1] * w[1] + w[2] * w[2]


def _sqrt_float_bounds(d2: Fraction) -> Tuple[float, float]:
    """Floats (lo, hi) with lo <= sqrt(d2) <= hi (rigorous, by construction)."""
    if d2 <= 0:
        return 0.0, 0.0
    x = math.sqrt(float(d2))
    lo = x
    hi = x
    for _ in range(200):
        if Fraction(lo) * Fraction(lo) <= d2:
            break
        lo = math.nextafter(lo, -math.inf)
    for _ in range(200):
        if Fraction(hi) * Fraction(hi) >= d2:
            break
        hi = math.nextafter(hi, math.inf)
    return lo, hi


def seg_seg_distance_certified(p1, q1, p2, q2, required: float,
                               tol: float = 0.0) -> Tuple[str, float, float, str]:
    """Certified comparison of the closest approach against a required value.

    Returns (status, lower_bound_um, upper_bound_um, method) with status in
    safe / violation.  The decision is exact whenever the stable float value and
    its conservative error bound cannot separate the two cases.
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


BOX_EDGES = ((0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6),
             (5, 7), (0, 4), (1, 5), (2, 6), (3, 7))


def box_corners(lo, hi):
    return [(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
            (lo[0], hi[1], lo[2]), (hi[0], hi[1], lo[2]),
            (lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2]),
            (lo[0], hi[1], hi[2]), (hi[0], hi[1], hi[2])]


def seg_box_intersection(p0, p1, lo, hi) -> Optional[float]:
    """Exact slab test: entry parameter t in [0, 1], or None (faces included)."""
    d = _sub(p1, p0)
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-15:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return None
        else:
            ta = (lo[i] - p0[i]) / d[i]
            tb = (hi[i] - p0[i]) / d[i]
            if ta > tb:
                ta, tb = tb, ta
            if ta > t0:
                t0 = ta
            if tb < t1:
                t1 = tb
            if t0 > t1:
                return None
    return t0


def seg_box_distance(p0, p1, lo, hi) -> float:
    """EXACT distance segment <-> AABB (0 when they intersect).

    A crossing through the interior of a face is caught by the exact slab test.
    Otherwise the closest pair is either an endpoint of the segment against the
    box (the distance from a fixed point of the segment to a face is affine, so
    the minimum over the segment is attained at an endpoint) or the segment
    against one of the 12 box edges.
    """
    if seg_box_intersection(p0, p1, lo, hi) is not None:
        return 0.0
    d = min(point_box_distance(p0, lo, hi), point_box_distance(p1, lo, hi))
    corners = box_corners(lo, hi)
    for i, j in BOX_EDGES:
        v = seg_seg_distance(p0, p1, corners[i], corners[j])
        if v < d:
            d = v
            if d <= 0.0:
                return 0.0
    return d


# ------------------------------------------------------------- arc geometry
def arc_phase_interval(seg: dict) -> Tuple[float, float]:
    th = float(seg["theta_rad"])
    return (0.0, th) if th > 0 else (th, 0.0)


def _phase_reached(phase: float, lo: float, hi: float) -> bool:
    """True when some phase + 2*k*pi lies inside [lo, hi]."""
    two_pi = 2.0 * math.pi
    k = math.ceil((lo - phase) / two_pi - 1e-12)
    return phase + k * two_pi <= hi + 1e-12


def seg_aabb(seg: dict):
    """EXACT axis aligned bounding box of one serialised segment.

    A coordinate of an arc is c + R (cos(phi) u + sin(phi) v) = c + A cos(phi - p)
    with A = R hypot(u_i, v_i) and p = atan2(v_i, u_i), so its extrema on the
    sweep are attained exactly at the phases p (maximum) and p + pi (minimum)
    when those phases fall inside the sweep.  There is no sampling here and no
    "add the other side just in case": both the maximum and the minimum are only
    added when they are actually reached.
    """
    if seg["kind"] == "line":
        p0, p1 = seg["p0"], seg["p1"]
        return (tuple(min(p0[i], p1[i]) for i in range(3)),
                tuple(max(p0[i], p1[i]) for i in range(3)))
    c, u, v = seg["center"], seg["u"], seg["v"]
    R, th = float(seg["radius_um"]), float(seg["theta_rad"])
    lo_phi, hi_phi = arc_phase_interval(seg)
    span = hi_phi - lo_phi
    p0 = eval_seg(seg, 0.0)
    p1 = eval_seg(seg, seg_length(seg))
    lo: List[float] = []
    hi: List[float] = []
    for i in range(3):
        vals = [p0[i], p1[i]]
        amp = R * math.hypot(u[i], v[i])
        if amp > 0.0:
            if span >= 2.0 * math.pi - 1e-12:
                vals.append(c[i] + amp)
                vals.append(c[i] - amp)
            else:
                phi_max = math.atan2(v[i], u[i])
                if _phase_reached(phi_max, lo_phi, hi_phi):
                    vals.append(c[i] + amp)
                if _phase_reached(phi_max + math.pi, lo_phi, hi_phi):
                    vals.append(c[i] - amp)
        lo.append(min(vals))
        hi.append(max(vals))
    return (tuple(lo), tuple(hi))


def chord_count_for_domain(seg: dict, requested: int) -> int:
    """Smallest chord count >= requested with |delta_theta| <= pi per chord.

    Outside that domain R (1 - cos(dphi/2)) is NOT a bound on the deviation of
    the chord from the sub-arc, and because cos() is periodic it even returns 0
    for dphi = 4*pi (a sweep of 8*pi subdivided into 2 chords), i.e. it would
    claim that a four-turn circle equals a single point.
    """
    if seg["kind"] == "line":
        return max(1, int(requested))
    theta = abs(float(seg["theta_rad"]))
    need = int(math.ceil(theta / MAX_CHORD_SWEEP_RAD - 1e-12))
    return max(1, int(requested), need)


def _phase_in_interval(phase: float, lo: float, hi: float) -> bool:
    """True when some phase + 2*k*pi lies inside [lo, hi] (lo <= hi)."""
    two_pi = 2.0 * math.pi
    k = math.floor((lo - phase) / two_pi)
    for kk in (k, k + 1, k + 2):
        p = phase + kk * two_pi
        if lo - 1e-12 <= p <= hi + 1e-12:
            return True
    return False


def _sagitta(seg: dict, n_chords: int) -> float:
    """Max deviation of an arc from its chord polygon (0 for a line).

    n_chords is raised into the proven domain first (|delta_theta| <= pi per
    chord); the raise is what stops cos() periodicity from returning a sagitta
    of 0 for a multi-turn arc.
    """
    if seg["kind"] == "line":
        return 0.0
    n = chord_count_for_domain(seg, n_chords)
    dphi = abs(float(seg["theta_rad"])) / n
    if dphi > MAX_CHORD_SWEEP_RAD + 1e-12:
        raise AssertionError("sagitta bound evaluated outside its domain: %r" % dphi)
    return float(seg["radius_um"]) * (1.0 - math.cos(dphi / 2.0))


def _chord_plan(seg: dict, requested: int):
    """(n, sagitta_um, chord nodes) with the domain rule applied."""
    if seg["kind"] == "line":
        # a line is ONE chord whatever the requested depth is
        L = seg_length(seg)
        return 1, 0.0, [eval_seg(seg, 0.0), eval_seg(seg, L)]
    n = chord_count_for_domain(seg, requested)
    L = seg_length(seg)
    return n, _sagitta(seg, n), [eval_seg(seg, L * i / n) for i in range(n + 1)]


def sample_points(seg: dict):
    L = seg_length(seg)
    npt = max(3, int(math.ceil(L / SAMPLE_STEP_UM)) + 1)
    return [eval_seg(seg, L * i / (npt - 1)) for i in range(npt)]


def point_arc_distance(p, seg: dict) -> float:
    """EXACT distance from a point to a serialised arc.

    The closest point of the full circle through the arc is the radial
    projection of p; if its phase falls inside the sweep the distance is
    | |p - C| - R |, otherwise the closest point of the arc is one of its two
    endpoints (the distance to a circular arc is unimodal along the sweep, so
    the endpoint check is exhaustive).
    """
    c, u, v = seg["center"], seg["u"], seg["v"]
    R = float(seg["radius_um"])
    d = _sub(p, c)
    n = _norm(d)
    if n <= 1e-15:
        return R
    phi = math.atan2(_dot(d, v), _dot(d, u))
    lo, hi = arc_phase_interval(seg)
    if _phase_in_interval(phi, lo, hi):
        return abs(n - R)
    L = seg_length(seg)
    return min(_norm(_sub(p, eval_seg(seg, 0.0))),
               _norm(_sub(p, eval_seg(seg, L))))


def point_curve_distance(p, ref: dict) -> float:
    """Exact distance from a point to a reference centreline (line or arc).

    This is an ACHIEVED distance between a point of the path and a point of the
    reference curve, so it is a rigorous upper bound on the true minimum
    distance and a valid violation witness.  Used for the sampled estimate and
    for the witness checks, never as a certified lower bound.
    """
    if ref["kind"] == "line":
        return point_seg_distance(p, ref["p0"], ref["p1"])
    return point_arc_distance(p, ref)


# ------------------------------------------------------- certified clearance
def _certified_clearance(seg: dict, boxes, required: float, tol: float) -> Dict:
    """Certified lower bound of dist(seg, all boxes) plus sampling witnesses."""
    if not boxes:
        return {"lower_bound_um": math.inf, "status": "safe", "sampled_min_um": None,
                "analytic_min_um": None, "levels": [], "samples": 0,
                "witness_inside": 0, "witness_keepout_um": None}
    pts = sample_points(seg)
    sampled_min = math.inf
    inside = 0
    for p in pts:
        for box in boxes:
            d = point_box_distance(p, box["lo"], box["hi"])
            if d < sampled_min:
                sampled_min = d
            if d <= 1e-12:
                inside += 1
    empty = {"levels": [], "samples": len(pts), "witness_inside": inside,
             "witness_keepout_um": sampled_min}
    if seg["kind"] == "line":
        analytic_min = min(seg_box_distance(seg["p0"], seg["p1"], b["lo"], b["hi"])
                           for b in boxes)
        lb = analytic_min
        status = "safe" if lb >= required - tol else "violation"
        if inside:
            status = "violation"
        return dict(empty, lower_bound_um=lb, status=status,
                    sampled_min_um=sampled_min, analytic_min_um=analytic_min)
    n = 2
    best_lb = -math.inf
    best_ub = math.inf
    levels = []
    witness = math.inf
    for _level in range(MAX_CHORD_LEVEL + 1):
        n_eff, sag, nodes = _chord_plan(seg, n)
        for p in nodes:                        # these points ARE on the curve
            for box in boxes:
                d = point_box_distance(p, box["lo"], box["hi"])
                if d < witness:
                    witness = d
        if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required - tol:
            break                              # reliable violation witness
        dmin = math.inf
        for i in range(n_eff):
            for box in boxes:
                d = seg_box_distance(nodes[i], nodes[i + 1], box["lo"], box["hi"])
                if d < dmin:
                    dmin = d
        lb = dmin - sag
        ub = dmin + sag
        best_lb = max(best_lb, lb)
        best_ub = min(best_ub, ub)
        levels.append({"n_chords": n_eff, "lower_bound_um": lb,
                       "upper_bound_um": ub, "sagitta_um": sag})
        if lb >= required - tol:
            break
        n *= 2
    if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required - tol or inside:
        status = "violation"
    elif best_lb >= required - tol:
        status = "safe"
    elif sampled_min < required - tol:
        status = "violation"                   # reliable sampling evidence
    else:
        status = "undecided"
    return {"lower_bound_um": best_lb, "status": status, "sampled_min_um": sampled_min,
            "analytic_min_um": None, "levels": levels, "samples": len(pts),
            "witness_inside": inside,
            "witness_keepout_um": (None if math.isinf(witness) else witness)}


def _pair_chord_bounds(A: dict, B: dict, level: int):
    """(lb, ub, witness_ub, nA, nB) chord bounds of two serialised primitives.

    Every point of A is within sagA of a chord point of A and every point of B
    within sagB of a chord point of B, hence

        min_ij dist(c_i, d_j) - sagA - sagB  <=  dist(A, B)
                                             <=  min_ij dist(c_i, d_j) + sagA + sagB

    Both sagittas obey the proven domain rule (see chord_count_for_domain).  The
    min over chord pairs is itself decided with the certified segment/segment
    decision, so a near-parallel chord pair cannot silently produce a bogus
    "safe" bound.  witness_ub is an upper bound built ONLY from points that lie
    on the two curves, so it is a valid violation witness.
    """
    nA, sagA, nodesA = _chord_plan(A, 2 ** level)
    nB, sagB, nodesB = _chord_plan(B, 2 ** level)
    dmin_lo = math.inf
    dmin_hi = math.inf
    for i in range(nA):
        for j in range(nB):
            _st, lo, hi, _m = seg_seg_distance_certified(
                nodesA[i], nodesA[i + 1], nodesB[j], nodesB[j + 1], 0.0, 0.0)
            if lo < dmin_lo:
                dmin_lo = lo
            if hi < dmin_hi:
                dmin_hi = hi
    witness = math.inf
    for p in nodesA:
        for i in range(nB):
            d = point_seg_distance(p, nodesB[i], nodesB[i + 1])
            if d < witness:
                witness = d
    witness += sagB
    for p in nodesB:
        for i in range(nA):
            d = point_seg_distance(p, nodesA[i], nodesA[i + 1])
            if d + sagA < witness:
                witness = d + sagA
    return dmin_lo - sagA - sagB, dmin_hi + sagA + sagB, witness, nA, nB


def _certified_spacing(seg: dict, refs, required: float, tol: float) -> Dict:
    """Certified lower bound of dist(seg, every reference centreline).

    Reference centrelines may be lines AND arcs; the v0.1.1 filter
    kind == "line" silently dropped every arc reference, so a path crossing a
    reference arc was certified as a pass.  All four kind combinations are
    decided here and both sagittas are subtracted when both sides are curves.
    """
    kinds = sorted({r["kind"] for r in refs})
    if not refs:
        return {"lower_bound_um": math.inf, "status": "safe", "sampled_min_um": None,
                "analytic_min_um": None, "levels": [], "samples": 0,
                "n_reference_curves": 0, "reference_kinds": [],
                "witness_keepout_um": None}
    pts = sample_points(seg)
    sampled_min = math.inf
    for p in pts:
        for r in refs:
            d = point_curve_distance(p, r)
            if d < sampled_min:
                sampled_min = d
    base = {"levels": [], "samples": len(pts), "n_reference_curves": len(refs),
            "reference_kinds": kinds, "sampled_min_um": sampled_min,
            "witness_keepout_um": sampled_min}
    if seg["kind"] == "line" and all(r["kind"] == "line" for r in refs):
        statuses = []
        worst_lb = math.inf
        worst_ub = math.inf
        analytic = math.inf
        methods = []
        for r in refs:
            st, lo, hi, method = seg_seg_distance_certified(
                seg["p0"], seg["p1"], r["p0"], r["p1"], required, tol)
            statuses.append(st)
            worst_lb = min(worst_lb, lo)
            worst_ub = min(worst_ub, hi)
            analytic = min(analytic, _ssd_stable(seg["p0"], seg["p1"],
                                                 r["p0"], r["p1"])[0])
            methods.append(method)
        status = ("violation" if any(s == "violation" for s in statuses) else "safe")
        return dict(base, lower_bound_um=worst_lb, status=status,
                    analytic_min_um=analytic, upper_bound_um=worst_ub,
                    method=(("exact segment/segment distance (" +
                             ", ".join(sorted(set(methods))) + ") for line/line "
                             "pairs; sampled minima are upper bounds only")))
    n = 2
    best_lb = -math.inf
    best_ub = math.inf
    levels = []
    witness = math.inf
    for _level in range(MAX_CHORD_LEVEL + 1):
        lb = math.inf
        ub = math.inf
        for r in refs:
            p_lb, p_ub, w, nA, nB = _pair_chord_bounds(seg, r, _level)
            lb = min(lb, p_lb)
            ub = min(ub, p_ub)
            witness = min(witness, w)
            levels.append({"level": _level, "reference_kind": r["kind"],
                           "n_chords_self": nA, "n_chords_reference": nB,
                           "lower_bound_um": p_lb, "upper_bound_um": p_ub})
        best_lb = max(best_lb, lb)
        best_ub = min(best_ub, ub)
        if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required - tol:
            break                              # reliable violation witness
        if lb >= required - tol:
            break
        n *= 2
    if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required - tol:
        status = "violation"
    elif best_lb >= required - tol:
        status = "safe"
    elif sampled_min < required - tol:
        status = "violation"
    else:
        status = "undecided"
    return dict(base, lower_bound_um=best_lb, status=status, analytic_min_um=None,
                upper_bound_um=best_ub, witness_keepout_um=witness,
                method=("certified chord bounds over all parameter pairs for every "
                        "kind combination (line/line, line/arc, arc/line, arc/arc); "
                        "both sagittas subtracted; sagitta domain dphi <= pi; "
                        "sampled minima are upper bounds only"))


# ---------------------------------------------------------------- validation
def validate_path_dict(path: dict, scene: dict, cfg: dict) -> Dict:
    segs = path.get("segments")
    if not isinstance(segs, list) or not segs:
        raise ValueError("path.segments must be a non-empty list, got %r" % (segs,))
    for i, s in enumerate(segs):
        check_segment_dict(s, "path.segments[%d]" % i)
    refs = scene.get("reference_centerlines", []) or []
    for i, r in enumerate(refs):
        check_segment_dict(r, "reference_centerlines[%d]" % i)

    chip = scene["chip_volume"]
    inner_lo = tuple(chip["lo"][i] + cfg["effective_boundary_margin_um"] for i in range(3))
    inner_hi = tuple(chip["hi"][i] - cfg["effective_boundary_margin_um"] for i in range(3))
    obstacles = scene.get("obstacles", [])
    req_obs = cfg["effective_obstacle_clearance_um"]
    dmin = cfg["dmin_centerline_um"]
    tol = cfg.get("clearance_decision_tol_um", 1e-9)
    pos_tol = cfg.get("pos_tol_um", 1e-6)
    source_pos_tol = cfg.get("source_pos_tol_um", 1e-6)
    source_ang_tol = cfg.get("source_ang_tol_deg", 1e-6)
    target_pos_tol = cfg["eps_pos_um"]
    target_ang_tol = cfg["eps_ang_deg"]

    out: Dict = {"validator": "native3d.validation.validate (independent, JSON only)",
                 "units": "um", "n_segments": len(segs),
                 "segment_kinds": [s["kind"] for s in segs]}

    # ---- length, radius (refit from 3 sampled points), continuity
    L_ana = sum(seg_length(s) for s in segs)
    L_ind = 0.0
    radii_stored, radii_fit = [], []
    for s in segs:
        L = seg_length(s)
        if s["kind"] == "line":
            L_ind += _norm(_sub(s["p1"], s["p0"]))
            continue
        R = float(s["radius_um"])
        radii_stored.append(R)
        A, B, C = eval_seg(s, 0.0), eval_seg(s, L / 3.0), eval_seg(s, 2.0 * L / 3.0)
        area2 = _norm(_cross(_sub(B, A), _sub(C, A)))
        R_fit = (R if area2 < 1e-12 else
                 (_norm(_sub(B, C)) * _norm(_sub(A, C)) * _norm(_sub(A, B)) / (2.0 * area2)))
        radii_fit.append(R_fit)
        n_ch = 64
        tot = 0.0
        for i in range(n_ch):
            a = eval_seg(s, L * i / n_ch)
            b = eval_seg(s, L * (i + 1) / n_ch)
            chord = _norm(_sub(b, a))
            tot += 2.0 * R_fit * math.asin(max(-1.0, min(1.0, chord / (2.0 * R_fit))))
        L_ind += tot
    out["total_length_um_analytic"] = L_ana
    out["total_length_um_independent"] = L_ind
    out["length_relative_error"] = (abs(L_ind - L_ana) / L_ana) if L_ana else 0.0
    out["min_radius_um_stored"] = (min(radii_stored) if radii_stored else None)
    out["min_radius_um_refit"] = (min(radii_fit) if radii_fit else None)
    out["radius_refit_max_rel_error"] = (
        max(abs(a - b) / a for a, b in zip(radii_stored, radii_fit)) if radii_stored else 0.0)
    out["radius_violations"] = [
        {"segment": i, "radius_um": s["radius_um"]}
        for i, s in enumerate(segs)
        if s["kind"] == "arc" and float(s["radius_um"]) < cfg["rmin_um"] - cfg["radius_tol_um"]]

    pos_errs, ang_errs = [], []
    for i in range(len(segs) - 1):
        a, b = segs[i], segs[i + 1]
        La, Lb = seg_length(a), seg_length(b)
        pa, pb = eval_seg(a, La), eval_seg(b, 0.0)
        pos_errs.append(_norm(_sub(pa, pb)))
        ta = eval_tangent_exact(a, La)
        tb = eval_tangent_exact(b, 0.0)
        ang_errs.append(math.degrees(math.acos(max(-1.0, min(1.0, _dot(ta, tb))))))
    out["junction_position_errors_um"] = pos_errs
    out["junction_tangent_errors_deg"] = ang_errs
    out["max_junction_position_error_um"] = max(pos_errs) if pos_errs else 0.0
    out["max_junction_tangent_error_deg"] = max(ang_errs) if ang_errs else 0.0
    out["c1_continuous"] = (out["max_junction_position_error_um"] <= cfg["pos_tol_um"]
                            and out["max_junction_tangent_error_deg"] <= cfg["ang_tol_deg"])

    # ---- boundary: exact analytic extrema of every segment
    n_samp = 0
    bnd_viol = 0
    min_margin = math.inf
    boxes = [seg_aabb(s) for s in segs]
    analytic_margin = math.inf
    analytic_overshoot = 0.0
    worst_axis = None
    for i, (lo, hi) in enumerate(boxes):
        for k, ax in enumerate("xyz"):
            over = max(inner_lo[k] - lo[k], hi[k] - inner_hi[k], 0.0)
            if over > analytic_overshoot:
                analytic_overshoot = over
                worst_axis = {"segment": i, "axis": ax, "overshoot_um": over,
                              "segment_lo": lo[k], "segment_hi": hi[k],
                              "allowed": [inner_lo[k], inner_hi[k]]}
            margin = min(lo[k] - inner_lo[k], inner_hi[k] - hi[k])
            if margin < analytic_margin:
                analytic_margin = margin
    for s in segs:
        for p in sample_points(s):
            n_samp += 1
            m = min(min(p[k] - inner_lo[k], inner_hi[k] - p[k]) for k in range(3))
            if m < min_margin:
                min_margin = m
            if m < -tol:
                bnd_viol += 1
    boundary_certified = analytic_overshoot <= tol
    out["boundary"] = {
        "samples": n_samp,
        "violation_samples": bnd_viol,
        "min_margin_um_sampled": min_margin,
        "analytic_min_margin_um": analytic_margin,
        "analytic_overshoot_um": analytic_overshoot,
        "worst_extreme": worst_axis,
        "certified_contained": boundary_certified,
        "conservative_margin_lower_bound_um": analytic_margin,
        "status": ("safe" if boundary_certified else "violation"),
        "method": ("exact analytic coordinate extrema per segment (no chord error, "
                   "no sampling)")}

    # ---- obstacle clearance
    obs = _certified_clearance_multi(segs, obstacles, req_obs, tol)
    out["obstacle_clearance"] = {
        "required_um": req_obs,
        "min_sampled_clearance_um": obs["sampled_min_um"],
        "analytic_min_clearance_um": obs["analytic_min_um"],
        "conservative_clearance_lower_bound_um": obs["lower_bound_um"],
        "sampled_points_inside_obstacle": obs["witness_inside"],
        "sampled_below_required": (None if obs["sampled_min_um"] is None else
                                   obs["sampled_min_um"] < req_obs - tol),
        "certified_below_required": obs["lower_bound_um"] < req_obs - tol,
        "witness_keepout_um": obs.get("witness_keepout_um"),
        "status": obs["status"],
        "method": obs["method"],
        "per_segment": obs["per_segment"],
    }

    # ---- spacing against fixed reference centrelines (lines AND arcs)
    sp = _certified_spacing_multi(segs, refs, dmin, tol)
    out["centerline_spacing"] = {
        "required_um": dmin,
        "min_sampled_um": sp["sampled_min_um"],
        "analytic_min_um": sp["analytic_min_um"],
        "conservative_lower_bound_um": sp["lower_bound_um"],
        "sampled_below_required": (None if sp["sampled_min_um"] is None else
                                   sp["sampled_min_um"] < dmin - tol),
        "certified_below_required": (None if sp["lower_bound_um"] is None else
                                     sp["lower_bound_um"] < dmin - tol),
        "n_reference_curves": sp.get("n_reference_curves", 0),
        "reference_kinds": sp.get("reference_kinds", []),
        "witness_keepout_um": sp.get("witness_keepout_um"),
        "status": sp["status"],
        "method": sp["method"],
        "per_segment": sp["per_segment"],
    }

    # ---- endpoints: SOURCE pose and TARGET pose, each with its own tolerance
    src, tgt = scene["source_pose"], scene["target_pose"]
    p_start = eval_seg(segs[0], 0.0)
    p_end = eval_seg(segs[-1], seg_length(segs[-1]))
    t_start = eval_tangent_exact(segs[0], 0.0)
    t_end = eval_tangent_exact(segs[-1], seg_length(segs[-1]))
    fd_start = eval_tangent_fd(segs[0], 1e-3)
    fd_end = eval_tangent_fd(segs[-1], max(0.0, seg_length(segs[-1]) - 1e-3))

    def ang(u, v):
        return math.degrees(math.acos(max(-1.0, min(1.0, _dot(u, v)))))
    start_pos_err = _norm(_sub(p_start, src["position"]))
    start_ang_err = ang(t_start, src["tangent"])
    end_pos_err = _norm(_sub(p_end, tgt["position"]))
    end_ang_err = ang(t_end, tgt["tangent"])
    out["endpoint_errors"] = {
        "start_position_error_um": start_pos_err,
        "start_tangent_error_deg": start_ang_err,
        "end_position_error_um": end_pos_err,
        "end_tangent_error_deg": end_ang_err,
        "finite_difference_cross_check": {
            "start_tangent_error_deg": ang(fd_start, src["tangent"]),
            "end_tangent_error_deg": ang(fd_end, tgt["tangent"]),
            "note": "central difference at +-1e-3 um; agreement within the chord "
                    "error 1e-3/R rad confirms the analytic tangents"},
        "tolerances": {"source_pos_tol_um": source_pos_tol,
                       "source_ang_tol_deg": source_ang_tol,
                       "target_eps_pos_um": target_pos_tol,
                       "target_eps_ang_deg": target_ang_tol}}
    source_pos_bad = start_pos_err > source_pos_tol
    source_ang_bad = start_ang_err > source_ang_tol
    target_pos_bad = end_pos_err > target_pos_tol
    target_ang_bad = end_ang_err > target_ang_tol
    out["source_pose_check"] = {
        "start_position_error_um": start_pos_err,
        "start_tangent_error_deg": start_ang_err,
        "source_pos_tol_um": source_pos_tol,
        "source_ang_tol_deg": source_ang_tol,
        "position_ok": not source_pos_bad,
        "tangent_ok": not source_ang_bad,
        "status": ("safe" if not (source_pos_bad or source_ang_bad) else "violation"),
        "method": ("the path start is compared against the SOURCE pose with the "
                   "source tolerances; the target acceptance ball is not reused")}
    out["target_pose_check"] = {
        "end_position_error_um": end_pos_err,
        "end_tangent_error_deg": end_ang_err,
        "eps_pos_um": target_pos_tol,
        "eps_ang_deg": target_ang_tol,
        "position_ok": not target_pos_bad,
        "tangent_ok": not target_ang_bad,
        "status": ("safe" if not (target_pos_bad or target_ang_bad) else "violation"),
        "method": "the path end is compared against the TARGET pose acceptance ball"}

    # ---- z profile (analytic extrema)
    zs = []
    n_out_of_plane = 0
    z0 = src["position"][2]
    for s in segs:
        zs.append(eval_seg(s, 0.0)[2])
        lo, hi = seg_aabb(s)
        if abs(lo[2] - z0) > 1e-9 or abs(hi[2] - z0) > 1e-9:
            n_out_of_plane += 1
    zs.append(eval_seg(segs[-1], seg_length(segs[-1]))[2])
    out["z_profile"] = {"source_z_um": z0, "target_z_um": tgt["position"][2],
                        "junction_z_um": zs, "z_min_um": min(zs), "z_max_um": max(zs),
                        "n_segments_leaving_z0": n_out_of_plane,
                        "z_varies": (max(zs) - min(zs)) > 1e-9}

    counts = {
        "radius_violations": len(out["radius_violations"]),
        "boundary_violation_samples": bnd_viol,
        "boundary_analytic_violation": int(not boundary_certified),
        "obstacle_points_inside": obs["witness_inside"],
        "obstacle_sampled_below_required": int(bool(out["obstacle_clearance"]["sampled_below_required"])),
        "obstacle_certified_below_required": int(bool(out["obstacle_clearance"]["certified_below_required"])),
        "spacing_below_required": (0 if sp["sampled_min_um"] is None else
                                   int(sp["sampled_min_um"] < dmin - tol)),
        "spacing_certified_below_required": (0 if sp["lower_bound_um"] is None else
                                             int(sp["lower_bound_um"] < dmin - tol)),
        "non_c1_junctions": sum(1 for e in ang_errs if e > cfg["ang_tol_deg"]),
        "source_position_out_of_tolerance": int(source_pos_bad),
        "source_tangent_out_of_tolerance": int(source_ang_bad),
        "endpoint_position_out_of_tolerance": int(target_pos_bad),
        "endpoint_tangent_out_of_tolerance": int(target_ang_bad),
        "undecided_checks": (int(obs["status"] == "undecided")
                             + int(sp["status"] == "undecided")
                             + int(out["boundary"]["status"] == "undecided")),
    }
    out["violation_counts"] = counts

    # ---- certification self check: a lower bound above the sampled upper bound
    #      is mathematically impossible and must never be reported as a pass.
    self_check_failed = False
    self_check_notes = []
    if (out["obstacle_clearance"]["min_sampled_clearance_um"] is not None
            and out["obstacle_clearance"]["conservative_clearance_lower_bound_um"]
            > out["obstacle_clearance"]["min_sampled_clearance_um"] + 1e-9):
        self_check_failed = True
        self_check_notes.append("obstacle clearance lower bound above the sampled "
                                "upper bound")
    if (out["centerline_spacing"]["min_sampled_um"] is not None
            and out["centerline_spacing"]["conservative_lower_bound_um"]
            > out["centerline_spacing"]["min_sampled_um"] + 1e-9):
        self_check_failed = True
        self_check_notes.append("spacing lower bound above the sampled upper bound")

    witnessed = (counts["radius_violations"] > 0
                 or counts["boundary_violation_samples"] > 0
                 or counts["boundary_analytic_violation"] > 0
                 or counts["obstacle_points_inside"] > 0
                 or counts["obstacle_sampled_below_required"] > 0
                 or counts["spacing_below_required"] > 0
                 or obs["status"] == "violation"
                 or sp["status"] == "violation"
                 or counts["non_c1_junctions"] > 0
                 or counts["source_position_out_of_tolerance"] > 0
                 or counts["source_tangent_out_of_tolerance"] > 0
                 or counts["endpoint_position_out_of_tolerance"] > 0
                 or counts["endpoint_tangent_out_of_tolerance"] > 0
                 or out["max_junction_position_error_um"] > pos_tol)
    undecided = counts["undecided_checks"] > 0 or self_check_failed
    if witnessed:
        verdict = "fail"
    elif undecided:
        verdict = "uncertified"
    else:
        verdict = "pass"
    out["verdict"] = verdict
    out["certification"] = {
        "all_checks_certified": (verdict == "pass"),
        "n_undecided": counts["undecided_checks"],
        "self_check_failed": self_check_failed,
        "self_check_notes": self_check_notes,
        "sampling_is_only_an_estimate": True,
        "certified_lower_bounds_are_never_above_the_sampled_upper_bounds": (
            not self_check_failed),
        "verdict_is_a_proof_of_pass": (verdict == "pass"),
        "notes": ("pass = every check was certified safe; fail = a violation witness "
                  "or a bound that proves the requirement is missed; uncertified = a "
                  "check could not be decided inside the refinement cap, or the "
                  "certification self check failed (never reported as pass)"),
    }
    return out


def _certified_clearance_multi(segs, obstacles, required, tol) -> Dict:
    per = []
    lower = math.inf
    sampled = math.inf
    inside = 0
    witness = math.inf
    statuses = []
    for i, s in enumerate(segs):
        r = _certified_clearance(s, obstacles, required, tol)
        per.append({"segment": i, "kind": s["kind"], "lower_bound_um": r["lower_bound_um"],
                    "sampled_min_um": r["sampled_min_um"],
                    "analytic_min_um": r["analytic_min_um"],
                    "status": r["status"], "n_chords": (r["levels"][-1]["n_chords"]
                                                        if r["levels"] else None)})
        if not obstacles:
            continue
        lower = min(lower, r["lower_bound_um"])
        sampled = min(sampled, r["sampled_min_um"])
        if r.get("witness_keepout_um") is not None:
            witness = min(witness, r["witness_keepout_um"])
        inside += r["witness_inside"]
        statuses.append(r["status"])
    if not obstacles:
        return {"lower_bound_um": math.inf, "sampled_min_um": None, "analytic_min_um": None,
                "witness_inside": 0, "status": "safe", "per_segment": [],
                "witness_keepout_um": None,
                "method": "no obstacle in the scene"}
    if inside:
        status = "violation"
    elif any(x == "violation" for x in statuses):
        status = "violation"
    elif any(x == "undecided" for x in statuses):
        status = "undecided"
    else:
        status = "safe"
    all_lines = all(s["kind"] == "line" for s in segs)
    return {"lower_bound_um": lower,
            "sampled_min_um": sampled,
            "analytic_min_um": (min(p["analytic_min_um"] for p in per)
                                if all_lines else None),
            "witness_inside": inside, "status": status, "per_segment": per,
            "witness_keepout_um": (None if math.isinf(witness) else witness),
            "method": ("exact segment/segment and segment/AABB distance for lines; "
                       "adaptive chord subdivision with an explicit sagitta bound "
                       "inside its proven domain for arcs; a curve point inside the "
                       "keep-out is a violation witness; sampled minima are upper "
                       "bounds only")}


def _certified_spacing_multi(segs, refs, required, tol) -> Dict:
    if not refs:
        return {"lower_bound_um": None, "sampled_min_um": None, "analytic_min_um": None,
                "status": "safe", "per_segment": [], "n_reference_curves": 0,
                "reference_kinds": [], "witness_keepout_um": None,
                "method": "no reference centreline in the scene"}
    per = []
    lower = math.inf
    sampled = math.inf
    witness = math.inf
    statuses = []
    for i, s in enumerate(segs):
        r = _certified_spacing(s, refs, required, tol)
        per.append({"segment": i, "kind": s["kind"], "lower_bound_um": r["lower_bound_um"],
                    "sampled_min_um": r["sampled_min_um"],
                    "analytic_min_um": r["analytic_min_um"], "status": r["status"]})
        lower = min(lower, r["lower_bound_um"])
        sampled = min(sampled, r["sampled_min_um"])
        if r.get("witness_keepout_um") is not None:
            witness = min(witness, r["witness_keepout_um"])
        statuses.append(r["status"])
    if any(x == "violation" for x in statuses):
        status = "violation"
    elif any(x == "undecided" for x in statuses):
        status = "undecided"
    else:
        status = "safe"
    return {"lower_bound_um": lower, "sampled_min_um": sampled,
            "analytic_min_um": (min(r["analytic_min_um"] for r in per)
                                if all(r["analytic_min_um"] is not None for r in per)
                                else None),
            "status": status, "per_segment": per,
            "n_reference_curves": len(refs),
            "reference_kinds": sorted({r["kind"] for r in refs}),
            "witness_keepout_um": (None if math.isinf(witness) else witness),
            "method": ("certified chord bounds for every kind combination "
                       "(line/line, line/arc, arc/line, arc/arc); exact "
                       "segment/segment distance with an exact-rational fallback for "
                       "line/line pairs; both sagittas subtracted and the sagitta "
                       "domain dphi <= pi enforced; sampled minima are upper bounds "
                       "only")}


def validate_artifact(artifact: dict) -> Dict:
    return validate_path_dict(artifact["search"]["path"], artifact["input"],
                              artifact["input"]["config"])


def main(argv: Sequence[str]) -> int:  # pragma: no cover - CLI
    if len(argv) < 2:
        print("usage: python -m native3d.validation.validate <artifact.json>")
        return 2
    with open(argv[1], "r", encoding="utf-8") as fh:
        art = json.load(fh)
    res = validate_artifact(art)
    print(json.dumps(res, indent=2))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv))
