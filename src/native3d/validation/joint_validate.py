"""INDEPENDENT joint validation of a v0.2 multi-channel artifact (JSON only).

This module is deliberately self-contained: it reads ONE JSON file and imports
nothing from ``native3d.geometry``, ``native3d.search``, ``native3d.constraints``
or ``native3d.multinet`` (only the standard library: json, math, sys, fractions).
It re-implements curve evaluation, distance queries and conservative subdivision
from the SERIALISED geometry, so anyone holding a single artifact can re-derive
every number:

    python -m native3d.validation.joint_validate artifacts/v020/cases/<file>.json

WHAT IT RE-DOES FROM THE JSON (never from the scheduler's verdicts)
------------------------------------------------------------------
* the pose contract of every declared input pose (three finite numbers and a
  non-zero direction; the direction is normalised here exactly as the production
  ``Pose3D`` contract does);
* the PORT checks of every net: the returned path must start on its source pose
  inside the SOURCE tolerances and end on its target pose inside the TARGET
  acceptance ball;
* C1 continuity at every junction, the minimum bend radius, the chip boundary and
  the obstacle keep-out of every returned primitive;
* the spacing to the STATIC reference centrelines and to the PUBLISHED occupancy;
* the post-hoc GLOBAL-ARCLENGTH self-spacing check (``|s - t| >= ell_local`` over
  the assembled-path cumulative arclength), INCLUDING the segment compared with
  itself and multi-turn re-entry;
* every curve pair between two DIFFERENT nets (line/line, line/arc, arc/line,
  arc/arc) with NO local exemption;
* the occupancy bookkeeping: every published curve must belong to a committed net,
  carry valid ``net_id``/``segment_id``/``occupancy_revision``, and match THAT
  net's returned segment content (so "the occupancy really is the net's own
  actual geometry" is re-verified from the file alone).

Numbers are reported in the three separate classes the project uses everywhere:

    SAMPLED estimate          - measured at finitely many points; a sampled
                                MINIMUM can only be too LARGE, never a certificate;
    CERTIFIED lower bound     - can never exceed the true minimum;
    VIOLATION witness         - a point pair that really lies on the curves and is
                                closer than required.

A check that cannot be decided inside its refinement cap is ``undecided`` and is
NEVER counted as a pass; the net verdict is then ``uncertified``.

GEOMETRY OF THIS SECOND IMPLEMENTATION
--------------------------------------
The restricted minimum of the self-spacing check is computed WITHOUT polygon
clipping, by a different route: the check domain inside an arclength rectangle is
the rectangle intersected with one of the two half-planes ``t <= s - ell`` /
``t >= s + ell``; the minimum of the convex quadratic is attained at the
unconstrained minimiser when that point is inside, and otherwise on the boundary
of that set, which consists of parts of the four rectangle edges plus the part of
the band line inside the rectangle.  Each of those pieces is a straight line
parametrised by one parameter, along which the squared distance is a 1-D quadratic
solved in closed form.  This is a genuinely different algorithm from the polygon
clipping of the production implementation, and it decides the same quantity.
"""

from __future__ import annotations

import json
import math
import sys
from fractions import Fraction
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------- contract
UNITS = "um"
VALIDATOR_MARGIN_UM = 1e-9          # numeric margin of this implementation
MAX_SUBDIV_LEVEL = 6                # refinement cap of this implementation
STATUS_SAFE = "safe"
STATUS_VIOLATION = "violation"
STATUS_UNDECIDED = "undecided"


class ContractError(ValueError):
    """The artifact itself is malformed: rejected, never silently skipped."""


# =====================================================================
# vectors
# =====================================================================
def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a, what: str):
    n = _norm(a)
    if not (n > 0.0) or not math.isfinite(n):
        raise ContractError("%s must be a non-zero finite direction: %r" % (what, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _dist(a, b) -> float:
    return _norm(_sub(a, b))


def _angle_deg(u, v) -> float:
    """Angle between two directions in DEGREES, atan2 form.

    ``acos(dot)`` is NOT used here on purpose: for a genuinely continuous junction
    the dot product is 1 - theta^2/2, so at theta ~ 1e-8 rad the double closest to
    it is only ~2.2e-16 away from 1 and acos() returns ~1.2e-6 deg for an angle
    that is really ~0.  That precision artefact would report a false C1 corner,
    so the angle is measured from |u x v| and dot instead, exactly as the
    production endpoint check does.
    """
    n1, n2 = _norm(u), _norm(v)
    if n1 <= 0.0 or n2 <= 0.0:
        raise ContractError("a tangent of the returned path is not a non-zero vector")
    c = _dot(u, v) / (n1 * n2)
    s = _norm(_cross(u, v)) / (n1 * n2)
    return math.degrees(math.atan2(s, c))


def _num(x, what: str) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise ContractError("%s must be a number: %r" % (what, x))
    v = float(x)
    if not math.isfinite(v):
        raise ContractError("%s must be finite: %r" % (what, x))
    return v


def _vec(x, what: str) -> Tuple[float, float, float]:
    if not isinstance(x, (list, tuple)) or len(x) != 3:
        raise ContractError("%s must be a list of 3 numbers: %r" % (what, x))
    return (_num(x[0], what + "[0]"), _num(x[1], what + "[1]"), _num(x[2], what + "[2]"))


def parse_pose(d, what: str) -> Dict:
    if not isinstance(d, dict):
        raise ContractError("%s must be an object: %r" % (what, d))
    pos = _vec(d.get("position"), what + ".position")
    tan_raw = _vec(d.get("tangent"), what + ".tangent")
    tan = _unit(tan_raw, what + ".tangent")
    return {"position": pos, "tangent": tan, "tangent_was_normalised": tan != tan_raw}


# =====================================================================
# primitives
# =====================================================================
class Primitive:
    kind = "?"

    def point_at(self, s: float):  # pragma: no cover - interface
        raise NotImplementedError

    def tangent_at(self, s: float):  # pragma: no cover - interface
        raise NotImplementedError

    @property
    def length(self) -> float:  # pragma: no cover - interface
        raise NotImplementedError

    def aabb(self):  # pragma: no cover - interface
        raise NotImplementedError

    def to_dict(self) -> dict:  # pragma: no cover - interface
        raise NotImplementedError


class Line(Primitive):
    kind = "line"

    def __init__(self, d: dict) -> None:
        self.p0 = _vec(d.get("p0"), "line.p0")
        self.p1 = _vec(d.get("p1"), "line.p1")
        self._len = _dist(self.p1, self.p0)
        if self._len <= 0.0:
            raise ContractError("degenerate line segment (zero length): %r" % (d,))
        declared = d.get("length_um")
        if declared is not None and abs(_num(declared, "line.length_um") - self._len) > 1e-6:
            raise ContractError("line length_um %r does not match |p1 - p0| = %.12f"
                                % (declared, self._len))
        self._t = _scale(_sub(self.p1, self.p0), 1.0 / self._len)

    @property
    def length(self) -> float:
        return self._len

    def point_at(self, s: float):
        return _add(self.p0, _scale(self._t, s))

    def tangent_at(self, s: float):
        return self._t

    def aabb(self):
        return (tuple(min(self.p0[i], self.p1[i]) for i in range(3)),
                tuple(max(self.p0[i], self.p1[i]) for i in range(3)))


class Arc(Primitive):
    kind = "arc"

    def __init__(self, d: dict) -> None:
        self.center = _vec(d.get("center"), "arc.center")
        u = _vec(d.get("u"), "arc.u")
        v = _vec(d.get("v"), "arc.v")
        nu, nv = _norm(u), _norm(v)
        if nu <= 0.0 or nv <= 0.0:
            raise ContractError("arc basis must be non-zero: %r" % (d,))
        self.u = _scale(u, 1.0 / nu)
        self.v = _scale(v, 1.0 / nv)
        if abs(_dot(self.u, self.v)) > 1e-9:
            raise ContractError("arc basis is not orthogonal: |dot(u,v)| = %.3e"
                                % abs(_dot(self.u, self.v)))
        if abs(nu - 1.0) > 1e-9 or abs(nv - 1.0) > 1e-9:
            raise ContractError("arc basis is not unit: |u| = %.15f, |v| = %.15f"
                                % (nu, nv))
        self.radius = _num(d.get("radius_um"), "arc.radius_um")
        self.theta = _num(d.get("theta_rad"), "arc.theta_rad")
        if self.radius <= 0.0:
            raise ContractError("arc radius must be > 0")
        if abs(self.theta) <= 1e-12:
            raise ContractError("degenerate arc sweep %r" % (self.theta,))
        self._len = self.radius * abs(self.theta)
        declared = d.get("length_um")
        if declared is not None and abs(_num(declared, "arc.length_um") - self._len) > 1e-6:
            raise ContractError("arc length_um %r does not match R|theta| = %.12f"
                                % (declared, self._len))
        self._aabb = self._compute_aabb()

    @property
    def length(self) -> float:
        return self._len

    def point_at_phi(self, phi: float):
        c, s = math.cos(phi), math.sin(phi)
        return (self.center[0] + self.radius * (c * self.u[0] + s * self.v[0]),
                self.center[1] + self.radius * (c * self.u[1] + s * self.v[1]),
                self.center[2] + self.radius * (c * self.u[2] + s * self.v[2]))

    def tangent_at_phi(self, phi: float):
        c, s = math.cos(phi), math.sin(phi)
        sgn = 1.0 if self.theta > 0 else -1.0
        return (sgn * (-s * self.u[0] + c * self.v[0]),
                sgn * (-s * self.u[1] + c * self.v[1]),
                sgn * (-s * self.u[2] + c * self.v[2]))

    def phi_of_arclength(self, s: float) -> float:
        return (s / self.radius) * (1.0 if self.theta > 0 else -1.0)

    def point_at(self, s: float):
        return self.point_at_phi(self.phi_of_arclength(s))

    def tangent_at(self, s: float):
        return self.tangent_at_phi(self.phi_of_arclength(s))

    def _phase_in_sweep(self, phi: float) -> bool:
        lo, hi = (0.0, self.theta) if self.theta > 0 else (self.theta, 0.0)
        k = phi - lo
        k -= 2.0 * math.pi * math.floor(k / (2.0 * math.pi))
        cands = [k, k - 2.0 * math.pi, k + 2.0 * math.pi, phi, phi - 2.0 * math.pi,
                 phi + 2.0 * math.pi]
        for p in cands:
            if lo - 1e-12 <= p <= hi + 1e-12:
                return True
        return False

    def _compute_aabb(self):
        lo_phi = 0.0 if self.theta > 0 else self.theta
        hi_phi = self.theta if self.theta > 0 else 0.0
        p0 = self.point_at_phi(0.0)
        p1 = self.point_at_phi(self.theta)
        lows, highs = [], []
        span = abs(self.theta)
        for i in range(3):
            # amp is the TRUE amplitude of that coordinate around the centre:
            # radius * hypot(u_i, v_i), which is 0 when the coordinate axis is the
            # arc-plane NORMAL.  Using `radius` there would invent an extremum on
            # the normal axis and make the AABB (and the boundary check) wrong.
            amp = self.radius * math.hypot(self.u[i], self.v[i])
            vals = [p0[i], p1[i]]
            if span >= 2.0 * math.pi - 1e-12:
                vals += [self.center[i] + amp, self.center[i] - amp]
            elif amp > 0.0:
                # The two stationary phases of this coordinate are phi* and
                # phi* + pi.  Each candidate is only used when its UNWRAPPED phase
                # really lies in the sweep, and the coordinate is then EVALUATED at
                # that phase - never "centre +/- amp" by assumption.  Assuming the
                # sign is wrong at the wrapped representative (phi* + pi = 2*pi is
                # the sweep start 0, where the value is the MINIMUM, not the
                # maximum) and would invent an extremum outside the real arc.
                phi_max = math.atan2(self.v[i], self.u[i])
                for base in (phi_max, phi_max + math.pi):
                    for turn in (-2, -1, 0, 1, 2):
                        cand = base + 2.0 * math.pi * turn
                        if lo_phi - 1e-12 <= cand <= hi_phi + 1e-12:
                            vals.append(self.center[i] + self.radius * (
                                math.cos(cand) * self.u[i] + math.sin(cand) * self.v[i]))
            lows.append(min(vals))
            highs.append(max(vals))
        return (tuple(lows), tuple(highs))

    def aabb(self):
        return self._aabb


def parse_primitive(d) -> Primitive:
    if not isinstance(d, dict):
        raise ContractError("primitive must be an object: %r" % (d,))
    kind = d.get("kind")
    if kind == "line":
        return Line(d)
    if kind == "arc":
        return Arc(d)
    raise ContractError("unsupported primitive kind %r (line/arc only)" % (kind,))


def parse_segments(seq) -> List[Primitive]:
    if seq is None:
        return []
    if not isinstance(seq, list):
        raise ContractError("segments must be a list")
    return [parse_primitive(s) for s in seq]


# =====================================================================
# distances
# =====================================================================
def point_segment_distance(p, a, b) -> float:
    """Exact 3-D point/segment distance (clamped projection)."""
    ab = _sub(b, a)
    denom = _dot(ab, ab)
    if denom <= 0.0:
        return _dist(p, a)
    t = _dot(_sub(p, a), ab) / denom
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return _dist(p, _add(a, _scale(ab, t)))


def segment_segment_distance(p1, q1, p2, q2) -> float:
    """Closest approach of two segments: 4 endpoint cases + interior candidate."""
    best = min(point_segment_distance(p1, p2, q2), point_segment_distance(q1, p2, q2),
               point_segment_distance(p2, p1, q1), point_segment_distance(q2, p1, q1))
    u = _sub(q1, p1)
    v = _sub(q2, p2)
    w = _sub(p1, p2)
    a, b, c = _dot(u, u), _dot(u, v), _dot(v, v)
    d, e = _dot(u, w), _dot(v, w)
    det = a * c - b * b
    if abs(det) > 1e-14 * max(1.0, a * c):
        s = (b * e - c * d) / det
        t = (a * e - b * d) / det
        if -1e-12 <= s <= 1 + 1e-12 and -1e-12 <= t <= 1 + 1e-12:
            sc = 0.0 if s < 0.0 else (1.0 if s > 1.0 else s)
            tc = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            best = min(best, _dist(_add(p1, _scale(u, sc)), _add(p2, _scale(v, tc))))
    return best


def _fr(v: float) -> Fraction:
    return Fraction(v)


def point_segment_d2_exact(p, a, b) -> Fraction:
    ab = (_fr(b[0]) - _fr(a[0]), _fr(b[1]) - _fr(a[1]), _fr(b[2]) - _fr(a[2]))
    denom = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    pa = (_fr(p[0]) - _fr(a[0]), _fr(p[1]) - _fr(a[1]), _fr(p[2]) - _fr(a[2]))
    if denom == 0:
        return pa[0] * pa[0] + pa[1] * pa[1] + pa[2] * pa[2]
    t = (pa[0] * ab[0] + pa[1] * ab[1] + pa[2] * ab[2]) / denom
    if t < 0:
        t = Fraction(0)
    elif t > 1:
        t = Fraction(1)
    r = (pa[0] - t * ab[0], pa[1] - t * ab[1], pa[2] - t * ab[2])
    return r[0] * r[0] + r[1] * r[1] + r[2] * r[2]


def segment_segment_d2_exact(p1, q1, p2, q2) -> Fraction:
    """EXACT rational squared distance of two segments (standard case analysis)."""
    best = min(point_segment_d2_exact(p1, p2, q2), point_segment_d2_exact(q1, p2, q2),
               point_segment_d2_exact(p2, p1, q1), point_segment_d2_exact(q2, p1, q1))
    u = tuple(_fr(q1[i]) - _fr(p1[i]) for i in range(3))
    v = tuple(_fr(q2[i]) - _fr(p2[i]) for i in range(3))
    w = tuple(_fr(p1[i]) - _fr(p2[i]) for i in range(3))
    a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]
    b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
    c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
    d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]
    e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]
    det = a * c - b * b
    if det != 0:
        s = (b * e - c * d) / det
        t = (a * e - b * d) / det
        if 0 <= s <= 1 and 0 <= t <= 1:
            r = (w[0] + s * u[0] - t * v[0], w[1] + s * u[1] - t * v[1],
                 w[2] + s * u[2] - t * v[2])
            val = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
            if val < best:
                best = val
    return best


def sqrt_bounds(d2: Fraction) -> Tuple[float, float]:
    """Sound float (lower, upper) bounds of sqrt(d2) from an exact rational."""
    if d2 <= 0:
        return 0.0, 0.0
    est = math.sqrt(float(d2))
    if est == 0.0:
        return 0.0, float(d2) ** 0.5
    lo, hi = est, est
    for _ in range(4):
        lo = math.nextafter(lo, 0.0)
        hi = math.nextafter(hi, math.inf)
    return lo, hi


def point_arc_distance_with_witness(p, arc: Arc):
    """3-D distance to the PLANAR arc (not to the sphere through it)."""
    q = _sub(p, arc.center)
    a = _dot(q, arc.u)
    b = _dot(q, arc.v)
    h = _dot(q, _cross(arc.u, arc.v))
    rho = math.hypot(a, b)
    if rho == 0.0:
        return math.sqrt(arc.radius ** 2 + h * h), arc.point_at_phi(0.0)
    phi = math.atan2(b, a)
    lo, hi = (0.0, arc.theta) if arc.theta > 0 else (arc.theta, 0.0)
    reps = [phi + 2.0 * math.pi * k for k in range(-3, 4)]
    reach = [r for r in reps if lo - 1e-12 <= r <= hi + 1e-12]
    if reach:
        best = None
        for r in reach:
            pt = arc.point_at_phi(r)
            d = _dist(p, pt)
            if best is None or d < best[0]:
                best = (d, pt)
        return best
    p0, p1 = arc.point_at_phi(0.0), arc.point_at_phi(arc.theta)
    d0, d1 = _dist(p, p0), _dist(p, p1)
    return (d0, p0) if d0 <= d1 else (d1, p1)


def aabb_gap(lo1, hi1, lo2, hi2) -> float:
    s = 0.0
    for i in range(3):
        if hi1[i] < lo2[i]:
            s += (lo2[i] - hi1[i]) ** 2
        elif hi2[i] < lo1[i]:
            s += (lo1[i] - hi2[i]) ** 2
    return math.sqrt(s)


def point_box_distance_sq(p, lo, hi) -> float:
    s = 0.0
    for i in range(3):
        d = max(lo[i] - p[i], 0.0, p[i] - hi[i])
        s += d * d
    return s


def point_box_distance(p, lo, hi) -> float:
    return math.sqrt(point_box_distance_sq(p, lo, hi))


def segment_box_distance(p0, p1, lo, hi) -> float:
    """EXACT closest distance of a straight segment to an AABB.

    ``dist^2(p(t), box)`` is the sum of six convex pieces ``max(0, a + t b)^2``,
    so the breakpoints ``a + t b = 0`` cut [0, 1] into intervals on which it is a
    plain quadratic.  Each interval is minimised in closed form, which makes the
    answer exact instead of a 64-point sample.
    """
    d = _sub(p1, p0)
    terms = []
    for i in range(3):
        terms.append((lo[i] - p0[i], -d[i]))
        terms.append((p0[i] - hi[i], d[i]))
    breaks = {0.0, 1.0}
    for a, b in terms:
        if b != 0.0:
            t = -a / b
            if 0.0 < t < 1.0:
                breaks.add(t)
    pts = sorted(breaks)
    best = math.inf
    for k in range(len(pts) - 1):
        t_lo, t_hi = pts[k], pts[k + 1]
        # the sign of every term is CONSTANT on the OPEN interval, so the active
        # set must be sampled in its interior: at t_lo a term whose breakpoint is
        # exactly t_lo reads 0 and would be dropped even though it is active for
        # every t > t_lo.
        t_mid = 0.5 * (t_lo + t_hi)
        a2 = a1 = a0 = 0.0
        for a, b in terms:
            if a + t_mid * b > 0.0:
                a2 += b * b
                a1 += 2.0 * a * b
                a0 += a * a
        if a2 <= 0.0:
            t = t_mid
        else:
            t = min(t_hi, max(t_lo, -a1 / (2.0 * a2)))
        val = a0 + a1 * t + a2 * t * t
        if val < best:
            best = val
    return math.sqrt(max(0.0, best))


def primitive_box_clearance(seg: Primitive, box_lo, box_hi, required: float,
                            tol: float):
    """Certified (status, lower, upper, witness) clearance of one primitive."""
    lo, hi = seg.aabb()
    if aabb_gap(lo, hi, box_lo, box_hi) >= required:
        return STATUS_SAFE, aabb_gap(lo, hi, box_lo, box_hi), None, None
    if seg.kind == "line":
        d = segment_box_distance(seg.p0, seg.p1, box_lo, box_hi)
        if d >= required - tol:
            return STATUS_SAFE, d, d, None
        return STATUS_VIOLATION, d, d, d
    level = 0
    while True:
        n, sag, nodes = chord_elements(seg, 2 ** level)
        # witness first: a subdivision node is a point OF the curve
        witness = min(point_box_distance(p, box_lo, box_hi) for p in nodes)
        if witness + VALIDATOR_MARGIN_UM < required - tol:
            return STATUS_VIOLATION, max(0.0, witness - sag), witness, witness
        lb = math.inf
        ub = math.inf
        for k in range(n):
            d = segment_box_distance(nodes[k], nodes[k + 1], box_lo, box_hi)
            lb = min(lb, d - sag)
            ub = min(ub, d + sag)
        if lb >= required - tol:
            return STATUS_SAFE, lb, ub, None
        if ub < required - tol:
            return STATUS_VIOLATION, lb, ub, ub
        if level >= MAX_SUBDIV_LEVEL:
            return STATUS_UNDECIDED, lb, ub, None
        level += 1


def chord_elements(seg: Primitive, n_requested: int):
    """(chord_count, sagitta_bound, nodes) with |sweep| <= pi per chord."""
    if seg.kind == "line":
        return 1, 0.0, [seg.point_at(0.0), seg.point_at(seg.length)]
    need = max(1, int(math.ceil(abs(seg.theta) / math.pi - 1e-12)), int(n_requested))
    dphi = abs(seg.theta) / need
    sag = seg.radius * (1.0 - math.cos(dphi / 2.0))
    return need, sag, [seg.point_at(seg.length * i / need) for i in range(need + 1)]


def pair_bounds(A: Primitive, B: Primitive, level: int):
    """(lower, upper, witness_distance) certified bounds on the min distance."""
    if A.kind == "line" and B.kind == "line":
        d2 = segment_segment_d2_exact(A.p0, A.p1, B.p0, B.p1)
        lo, hi = sqrt_bounds(d2)
        return lo, hi, hi
    na, saga, pa = chord_elements(A, 2 ** level)
    nb, sagb, pb = chord_elements(B, 2 ** level)
    lo = math.inf
    hi = math.inf
    for i in range(na):
        for j in range(nb):
            d = segment_segment_distance(pa[i], pa[i + 1], pb[j], pb[j + 1])
            if d - saga - sagb < lo:
                lo = d - saga - sagb
            if d + saga + sagb < hi:
                hi = d + saga + sagb
    witness = math.inf
    for p in pa:
        for j in range(nb):
            d = point_segment_distance(p, pb[j], pb[j + 1])
            if d + sagb < witness:
                witness = d + sagb
    for p in pb:
        for i in range(na):
            d = point_segment_distance(p, pa[i], pa[i + 1])
            if d + saga < witness:
                witness = d + saga
    return max(0.0, lo), hi, witness


def pair_spacing_verdict(A: Primitive, B: Primitive, required: float,
                         tol: float) -> Dict:
    lo1, hi1 = A.aabb()
    lo2, hi2 = B.aabb()
    gap = aabb_gap(lo1, hi1, lo2, hi2)
    if gap >= required:
        return {"status": STATUS_SAFE, "kind": "%s-%s" % (A.kind, B.kind),
                "certified_lower_bound_um": gap, "level": None,
                "method": "aabb_gap", "sampled_estimate_um": None}
    n = 48
    sampled = math.inf
    for i in range(n + 1):
        p = A.point_at(A.length * i / n)
        for j in range(n + 1):
            q = B.point_at(B.length * j / n)
            d = _dist(p, q)
            if d < sampled:
                sampled = d
    level = 0
    while True:
        lb, ub, witness = pair_bounds(A, B, level)
        if witness + VALIDATOR_MARGIN_UM < required - tol:
            return {"status": STATUS_VIOLATION, "kind": "%s-%s" % (A.kind, B.kind),
                    "certified_lower_bound_um": lb, "certified_upper_bound_um": ub,
                    "level": level, "method": "violation_witness_on_the_curve",
                    "sampled_estimate_um": sampled, "witness_distance_um": witness}
        if lb >= required - tol:
            return {"status": STATUS_SAFE, "kind": "%s-%s" % (A.kind, B.kind),
                    "certified_lower_bound_um": lb, "certified_upper_bound_um": ub,
                    "level": level, "method": "conservative_chord_bounds",
                    "sampled_estimate_um": sampled}
        if ub < required - tol:
            return {"status": STATUS_VIOLATION, "kind": "%s-%s" % (A.kind, B.kind),
                    "certified_lower_bound_um": lb, "certified_upper_bound_um": ub,
                    "level": level, "method": "conservative_chord_bounds",
                    "sampled_estimate_um": sampled, "witness_distance_um": ub}
        if level >= MAX_SUBDIV_LEVEL:
            return {"status": STATUS_UNDECIDED, "kind": "%s-%s" % (A.kind, B.kind),
                    "certified_lower_bound_um": lb, "certified_upper_bound_um": ub,
                    "level": level, "method": "conservative_chord_bounds",
                    "sampled_estimate_um": sampled,
                    "note": "refinement cap reached without a decision"}
        level += 1


# =====================================================================
# global-arclength self-spacing (second implementation)
# =====================================================================
class _Elem:
    __slots__ = ("s0", "s1", "start", "vel", "delta", "seg_index")

    def __init__(self, seg_index, s0, s1, start, vel, delta):
        self.seg_index = seg_index
        self.s0 = s0
        self.s1 = s1
        self.start = start
        self.vel = vel
        self.delta = delta


def _elements(seg: Primitive, seg_index: int, off: float, n_chords: int) -> List[_Elem]:
    L = seg.length
    if seg.kind == "line":
        u = seg.tangent_at(0.0)
        return [_Elem(seg_index, off, off + L, seg.point_at(0.0), u, 0.0)]
    n, sag, pts = chord_elements(seg, n_chords)
    step = L / n
    out = []
    for i in range(n):
        a, b = pts[i], pts[i + 1]
        d = _sub(b, a)
        cl = _norm(d)
        if cl <= 0.0 or step <= 0.0:
            continue
        out.append(_Elem(seg_index, off + step * i, off + step * (i + 1), a,
                         _scale(d, 1.0 / step), sag))
    return out


class _Quad:
    """Squared distance between two chord lines, solved in closed form."""

    def __init__(self, A: _Elem, B: _Elem):
        self.va, self.vb = A.vel, B.vel
        self.W = (A.start[0] - B.start[0] - A.s0 * A.vel[0] + B.s0 * B.vel[0],
                  A.start[1] - B.start[1] - A.s0 * A.vel[1] + B.s0 * B.vel[1],
                  A.start[2] - B.start[2] - A.s0 * A.vel[2] + B.s0 * B.vel[2])
        cr = _cross(self.va, self.vb)
        self.cross_sq = _dot(cr, cr)
        self.crit = None
        if self.cross_sq > 1e-18:
            a = _dot(self.va, self.va)
            b = _dot(self.vb, self.vb)
            c = _dot(self.va, self.vb)
            d1 = _dot(self.va, self.W)
            d2 = _dot(self.vb, self.W)
            det = c * c - a * b
            if det != 0.0:
                self.crit = ((b * d1 - c * d2) / det, (c * d1 - a * d2) / det)

    def f(self, s: float, t: float) -> float:
        r = (self.W[0] + s * self.va[0] - t * self.vb[0],
             self.W[1] + s * self.va[1] - t * self.vb[1],
             self.W[2] + s * self.va[2] - t * self.vb[2])
        return r[0] * r[0] + r[1] * r[1] + r[2] * r[2]

    def min_along(self, base, direction, p_lo: float, p_hi: float):
        """min |Q1 - Q2| over a straight piece (s,t) = base + P * direction."""
        bs, bt = base
        ds, dt = direction
        A0 = (self.W[0] + bs * self.va[0] - bt * self.vb[0],
              self.W[1] + bs * self.va[1] - bt * self.vb[1],
              self.W[2] + bs * self.va[2] - bt * self.vb[2])
        A1 = (ds * self.va[0] - dt * self.vb[0], ds * self.va[1] - dt * self.vb[1],
              ds * self.va[2] - dt * self.vb[2])
        aa = _dot(A1, A1)
        ab = _dot(A0, A1)
        p = p_lo if aa <= 0.0 else min(p_hi, max(p_lo, -ab / aa))
        s = bs + p * ds
        t = bt + p * dt
        return math.sqrt(max(0.0, self.f(s, t))), s, t


def _clip_interval(p_lo: float, p_hi: float, c0: float, c1: float):
    """[p_lo, p_hi] intersected with ``c0 + P * c1 <= 0``."""
    if c1 == 0.0:
        return (p_lo, p_hi) if c0 <= 0.0 else None
    bound = -c0 / c1
    if c1 > 0.0:
        hi = min(p_hi, bound)
        return (p_lo, hi) if hi >= p_lo else None
    lo = max(p_lo, bound)
    return (lo, p_hi) if lo <= p_hi else None


def _elem_pair_restricted(A: _Elem, B: _Elem, ell: float):
    """(lower, upper|None, witness|None) of one element pair over the domain.

    The check domain inside the arclength rectangle ``[a0,a1] x [b0,b1]`` is the
    union of the two convex pieces

        D-  = rectangle n { s - t >= ell }      (band line t = s - ell)
        D+  = rectangle n { t - s >= ell }      (band line t = s + ell)

    and the squared distance between the two chord points is a convex quadratic
    in (s, t).  For each piece the minimum is attained at the unconstrained
    minimiser when that point is inside the piece, and otherwise on the piece's
    boundary, which consists of parts of the four rectangle edges plus the part of
    the band line inside the rectangle.  Every one of those pieces is a straight
    line in (s, t) parametrised by a single parameter, along which the squared
    distance is a 1-D quadratic solved in closed form.
    """
    a0, a1, b0, b1 = A.s0, A.s1, B.s0, B.s1
    if max(abs(a1 - b0), abs(a0 - b1)) < ell:
        return None
    q = _Quad(A, B)
    delta = A.delta + B.delta
    best_lo = math.inf
    best_ub = math.inf
    best_pt = None

    def in_box(s, t):
        return (a0 - 1e-12 <= s <= a1 + 1e-12) and (b0 - 1e-12 <= t <= b1 + 1e-12)

    # (base, direction, parameter length): the parameter starts at 0 on the corner
    edges = [((a0, b0), (0.0, 1.0), b1 - b0),
             ((a1, b0), (0.0, 1.0), b1 - b0),
             ((a0, b0), (1.0, 0.0), a1 - a0),
             ((a0, b1), (1.0, 0.0), a1 - a0)]

    for sign in (+1.0, -1.0):
        # piece: sign * (t - s) >= ell, i.e. g(s, t) = ell - sign * (t - s) <= 0
        def g(s, t, sign=sign):
            return ell - sign * (t - s)

        cands = []
        if q.crit is not None:
            cs, ct = q.crit
            if in_box(cs, ct) and g(cs, ct) <= 1e-12:
                cands.append((math.sqrt(max(0.0, q.f(cs, ct))), cs, ct))
        for base, direction, plen in edges:
            if plen <= 0.0:
                continue
            c0 = g(base[0], base[1])
            c1 = -sign * (direction[1] - direction[0])
            iv = _clip_interval(0.0, plen, c0, c1)
            if iv is None:
                continue
            cands.append(q.min_along(base, direction, iv[0], iv[1]))
        # the band line t = s + sign * ell, clipped to the rectangle
        s_lo = max(a0, b0 - sign * ell)
        s_hi = min(a1, b1 - sign * ell)
        if s_hi >= s_lo:
            cands.append(q.min_along((0.0, sign * ell), (1.0, 1.0), s_lo, s_hi))
        for val, s, t in cands:
            if val < best_lo:
                best_lo = val
            if in_box(s, t) and abs(s - t) >= ell and val < best_ub:
                best_ub = val
                best_pt = (s, t)
    if math.isinf(best_lo):
        return None
    # the rectangle is a superset of the restricted region, so the rectangle
    # minimum is also a valid lower bound of the restricted minimum
    rect_lo = segment_segment_distance(
        A.start, _add(A.start, _scale(A.vel, a1 - a0)),
        B.start, _add(B.start, _scale(B.vel, b1 - b0)))
    if rect_lo > best_lo:
        best_lo = rect_lo
    lower = max(0.0, best_lo - delta - VALIDATOR_MARGIN_UM)
    if math.isinf(best_ub):
        return lower, None, None
    return lower, max(0.0, best_ub + delta), best_pt


def self_spacing_global(segments: Sequence[Primitive], required: float, ell: float,
                        tol: float) -> Dict:
    spans = []
    off = 0.0
    for seg in segments:
        spans.append((off, off + seg.length))
        off += seg.length
    total = off
    out = {"name": "self_spacing_global_arclength", "check_domain":
           "|s - t| >= ell_local over the assembled-path cumulative arclength",
           "required_um": required, "ell_local_um": ell,
           "total_length_um": total, "n_segments": len(segments),
           "validator_margin_um": VALIDATOR_MARGIN_UM,
           "max_refinement_level": MAX_SUBDIV_LEVEL}
    if not segments:
        out.update({"status": STATUS_SAFE, "certified_lower_bound_um": None,
                    "witness": None})
        return out

    def point_at(s):
        if s <= spans[0][0]:
            return segments[0].point_at(0.0)
        for k, (a, b) in enumerate(spans):
            if a <= s < b:
                return segments[k].point_at(s - a)
        return segments[-1].point_at(segments[-1].length)

    lb = math.inf
    witness = None
    violation = False
    n_undecided = 0
    n_exempt = 0
    n_pairs = 0
    per_pair = []
    for i in range(len(segments)):
        for j in range(i, len(segments)):
            n_pairs += 1
            si, sj = segments[i], segments[j]
            ai0, ai1 = spans[i]
            bj0, bj1 = spans[j]
            if max(abs(ai1 - bj0), abs(ai0 - bj1)) < ell:
                n_exempt += 1
                continue
            lo_i, hi_i = si.aabb()
            lo_j, hi_j = sj.aabb()
            gap = aabb_gap(lo_i, hi_i, lo_j, hi_j)
            if gap >= required:
                lb = min(lb, gap)
                per_pair.append({"segment_pair": [i, j], "status": STATUS_SAFE,
                                 "method": "aabb_gap",
                                 "certified_lower_bound_um": gap})
                continue
            level_cap = MAX_SUBDIV_LEVEL if (si.kind == "arc" or sj.kind == "arc") else 0
            status = STATUS_UNDECIDED
            pair_lo = math.inf
            pair_ub = math.inf
            pair_pt = None
            used_level = 0
            for level in range(level_cap + 1):
                n_i = min(64, 2 ** level)
                ei = _elements(si, i, ai0, n_i)
                ej = ei if j == i else _elements(sj, j, bj0, n_i)
                pair_lo = math.inf
                pair_ub = math.inf
                pair_pt = None
                for A in ei:
                    for B in ej:
                        if j == i and B.s0 < A.s0:
                            continue
                        r = _elem_pair_restricted(A, B, ell)
                        if r is None:
                            continue
                        lo_e, ub_e, pt = r
                        if lo_e < pair_lo:
                            pair_lo = lo_e
                        if ub_e is not None and ub_e < pair_ub:
                            pair_ub = ub_e
                            pair_pt = pt
                used_level = level
                if pair_ub != math.inf and pair_ub + tol < required:
                    status = STATUS_VIOLATION
                    break
                if pair_lo >= required - tol:
                    status = STATUS_SAFE
                    break
            lb = min(lb, pair_lo)
            if status == STATUS_VIOLATION:
                violation = True
                ps, pt = pair_pt
                witness = {"segment_pair": [i, j], "global_arclength_s_um": ps,
                           "global_arclength_t_um": pt,
                           "arclength_separation_um": abs(ps - pt),
                           "in_check_domain": bool(abs(ps - pt) >= ell),
                           "point_s_um": list(point_at(ps)),
                           "point_t_um": list(point_at(pt)),
                           "distance_sampled_um": _dist(point_at(ps), point_at(pt)),
                           "distance_upper_bound_um": pair_ub}
            elif status == STATUS_UNDECIDED:
                n_undecided += 1
            per_pair.append({"segment_pair": [i, j], "status": status,
                             "certified_lower_bound_um": pair_lo,
                             "refinement_level": used_level})
    status = STATUS_VIOLATION if violation else (
        STATUS_SAFE if lb >= required - tol else STATUS_UNDECIDED)
    out.update({"status": status,
                "certified_lower_bound_um": (lb if math.isfinite(lb) else None),
                "witness": witness, "n_segment_pairs": n_pairs,
                "n_pairs_fully_exempt": n_exempt,
                "n_pairs_undecided": n_undecided, "pairs": per_pair,
                "verdict_rule": ("violation only through a point pair with "
                                 "|s - t| >= ell_local; undecided is never a pass")})
    return out


# =====================================================================
# per-net checks
# =====================================================================
def check_segments(segments: Sequence[Primitive], chip_lo, chip_hi,
                   obstacles: Sequence[Tuple], required_clearance: float,
                   rmin: float, radius_tol: float) -> Dict:
    checks: Dict[str, Dict] = {}
    if not segments:
        return {"ok": True, "checks": checks, "n_segments": 0}
    worst_r = min((s.radius for s in segments if s.kind == "arc"), default=math.inf)
    checks["min_bend_radius"] = {
        "status": STATUS_SAFE if worst_r >= rmin - radius_tol else STATUS_VIOLATION,
        "measured_radius_um": (None if math.isinf(worst_r) else worst_r),
        "required_um": rmin}
    worst_over = 0.0
    for s in segments:
        lo, hi = s.aabb()
        for i in range(3):
            worst_over = max(worst_over, chip_lo[i] - lo[i], hi[i] - chip_hi[i])
    checks["boundary"] = {
        "status": STATUS_SAFE if worst_over <= 1e-9 else STATUS_VIOLATION,
        "max_overshoot_um": max(0.0, worst_over),
        "note": ("the AABB of every primitive is compared with the chip volume shrunk "
                 "by the boundary margin; an arc AABB uses the true amplitude "
                 "radius * hypot(u_i, v_i) per axis")}
    worst_lb = math.inf
    worst_ub = math.inf
    n_viol = 0
    n_undec = 0
    witness = None
    for s in segments:
        for (blo, bhi) in obstacles:
            st, lb, ub, wit = primitive_box_clearance(s, blo, bhi, required_clearance,
                                                      VALIDATOR_MARGIN_UM)
            if lb is not None and lb < worst_lb:
                worst_lb = lb
            if ub is not None and ub < worst_ub:
                worst_ub = ub
            if st == STATUS_VIOLATION:
                n_viol += 1
                if witness is None:
                    witness = {"kind": s.kind, "clearance_um": wit}
            elif st == STATUS_UNDECIDED:
                n_undec += 1
    status = (STATUS_VIOLATION if n_viol else
              (STATUS_UNDECIDED if n_undec else STATUS_SAFE))
    checks["obstacle_clearance"] = {
        "status": status,
        "certified_lower_bound_um": (None if math.isinf(worst_lb) else worst_lb),
        "certified_upper_bound_um": (None if math.isinf(worst_ub) else worst_ub),
        "required_um": required_clearance,
        "n_violation_pairs": n_viol, "n_undecided_pairs": n_undec,
        "witness": witness}
    ok = all(c["status"] == STATUS_SAFE for c in checks.values())
    return {"ok": ok, "checks": checks, "n_segments": len(segments)}


def check_ports(segments: Sequence[Primitive], source: Dict, target: Dict,
                src_pos_tol: float, src_ang_tol_deg: float,
                eps_pos: float, eps_ang_deg: float) -> Dict:
    if not segments:
        return {"ok": False, "reason": "no segment to check the ports against"}
    p_first = segments[0].point_at(0.0)
    t_first = segments[0].tangent_at(0.0)
    p_last = segments[-1].point_at(segments[-1].length)
    t_last = segments[-1].tangent_at(segments[-1].length)
    sp = _dist(p_first, source["position"])
    sa = _angle_deg(t_first, source["tangent"])
    tp = _dist(p_last, target["position"])
    ta = _angle_deg(t_last, target["tangent"])
    ok = (sp <= src_pos_tol and sa <= src_ang_tol_deg
          and tp <= eps_pos and ta <= eps_ang_deg)
    return {"ok": ok,
            "source_position_error_um": sp, "source_position_tol_um": src_pos_tol,
            "source_tangent_error_deg": sa, "source_tangent_tol_deg": src_ang_tol_deg,
            "target_position_error_um": tp, "target_position_tol_um": eps_pos,
            "target_tangent_error_deg": ta, "target_tangent_tol_deg": eps_ang_deg,
            "uses_source_tolerances_for_the_start": True,
            "uses_the_target_acceptance_ball_for_the_end": True}


def check_c1(segments: Sequence[Primitive], pos_tol: float, ang_tol_deg: float) -> Dict:
    worst_p = 0.0
    worst_a = 0.0
    corners = []
    for k in range(len(segments) - 1):
        a, b = segments[k], segments[k + 1]
        pa = a.point_at(a.length)
        pb = b.point_at(0.0)
        ta = a.tangent_at(a.length)
        tb = b.tangent_at(0.0)
        dp = _dist(pa, pb)
        da = _angle_deg(ta, tb)
        worst_p = max(worst_p, dp)
        worst_a = max(worst_a, da)
        if dp > pos_tol or da > ang_tol_deg:
            corners.append({"junction": k, "position_error_um": dp,
                            "tangent_error_deg": da})
    return {"ok": not corners, "n_junctions": max(0, len(segments) - 1),
            "max_position_error_um": worst_p, "position_tol_um": pos_tol,
            "max_tangent_error_deg": worst_a, "tangent_tol_deg": ang_tol_deg,
            "corners": corners}


def check_reference_spacing(segments, references, required, tol) -> Dict:
    worst = math.inf
    n_undecided = 0
    n_violation = 0
    witness = None
    for s in segments:
        for ref in references:
            v = pair_spacing_verdict(s, ref, required, tol)
            if v["status"] == STATUS_UNDECIDED:
                n_undecided += 1
            if v["status"] == STATUS_VIOLATION:
                n_violation += 1
                if witness is None:
                    witness = {"segment_kind": s.kind, "reference_kind": ref.kind,
                               "method": v["method"],
                               "distance_um": v.get("witness_distance_um")}
            b = v.get("certified_lower_bound_um")
            if b is not None and b < worst:
                worst = b
    if n_violation:
        status = STATUS_VIOLATION
    elif n_undecided:
        status = STATUS_UNDECIDED
    elif not references:
        status = STATUS_SAFE
    else:
        status = STATUS_SAFE
    return {"status": status, "required_um": required,
            "min_certified_lower_bound_um": (None if math.isinf(worst) else worst),
            "n_pairs": len(segments) * len(references),
            "n_violations": n_violation, "n_undecided": n_undecided,
            "witness": witness}


def check_inter_net(segments_a: Sequence[Primitive], net_id_a,
                    segments_b: Sequence[Primitive], net_id_b,
                    required: float, tol: float) -> Dict:
    kinds = {}
    worst = math.inf
    violation = None
    n_undecided = 0
    for i, sa in enumerate(segments_a):
        for j, sb in enumerate(segments_b):
            v = pair_spacing_verdict(sa, sb, required, tol)
            k = v["kind"]
            kinds[k] = kinds.get(k, 0) + 1
            if v["status"] == STATUS_UNDECIDED:
                n_undecided += 1
            if v["status"] == STATUS_VIOLATION and violation is None:
                violation = {"candidate_segment": i, "other_segment": j,
                             "other_net": net_id_b, "kind": k,
                             "method": v["method"],
                             "distance_um": v.get("witness_distance_um"),
                             "certified_upper_bound_um": v.get("certified_upper_bound_um")}
            b = v.get("certified_lower_bound_um")
            if b is not None and b < worst:
                worst = b
    status = (STATUS_VIOLATION if violation is not None
              else (STATUS_UNDECIDED if n_undecided else STATUS_SAFE))
    return {"net_pair": [net_id_a, net_id_b], "status": status,
            "required_um": required,
            "n_pairs": len(segments_a) * len(segments_b),
            "n_pairs_by_kind": kinds,
            "min_certified_lower_bound_um": (None if math.isinf(worst) else worst),
            "n_undecided": n_undecided, "violation": violation,
            "no_local_exemption_between_nets": True}


# =====================================================================
# the whole artifact
# =====================================================================
def validate_joint_artifact(payload: Dict) -> Dict:
    if not isinstance(payload, dict):
        raise ContractError("the artifact must be a JSON object")
    units = payload.get("units")
    if units != UNITS:
        raise ContractError("units must be %r, got %r" % (UNITS, units))
    chip = payload.get("chip_volume")
    if not isinstance(chip, dict):
        raise ContractError("chip_volume missing")
    chip_lo = _vec(chip.get("lo"), "chip_volume.lo")
    chip_hi = _vec(chip.get("hi"), "chip_volume.hi")
    cfg = payload.get("config") or {}
    rmin = _num(cfg.get("rmin_um"), "config.rmin_um")
    dmin = _num(cfg.get("dmin_centerline_um"), "config.dmin_centerline_um")
    radius_tol = _num(cfg.get("radius_tol_um", 1e-6), "config.radius_tol_um")
    pos_tol = _num(cfg.get("pos_tol_um", 1e-6), "config.pos_tol_um")
    ang_tol = _num(cfg.get("ang_tol_deg", 1e-6), "config.ang_tol_deg")
    eps_pos = _num(cfg.get("eps_pos_um", 2.0), "config.eps_pos_um")
    eps_ang = _num(cfg.get("eps_ang_deg", 3.0), "config.eps_ang_deg")
    src_pos_tol = _num(cfg.get("source_pos_tol_um", 1e-6), "config.source_pos_tol_um")
    src_ang_tol = _num(cfg.get("source_ang_tol_deg", 1e-6),
                       "config.source_ang_tol_deg")
    tol = _num(cfg.get("clearance_decision_tol_um", 1e-9),
               "config.clearance_decision_tol_um")
    clearance = _num(cfg.get("effective_obstacle_clearance_um",
                             cfg.get("obstacle_clearance_um", 5.0)),
                     "config.effective_obstacle_clearance_um")
    boundary_margin = _num(cfg.get("effective_boundary_margin_um",
                                   cfg.get("wg_envelope_radius_um", 2.0)),
                           "config.effective_boundary_margin_um")
    policy = payload.get("self_spacing_policy") or {}
    ell = _num(policy.get("ell_local_um"), "self_spacing_policy.ell_local_um")

    obstacles = []
    for k, b in enumerate(payload.get("obstacles") or []):
        obstacles.append((_vec(b.get("lo"), "obstacles[%d].lo" % k),
                          _vec(b.get("hi"), "obstacles[%d].hi" % k)))
    static_refs = [parse_primitive(s) for s in
                   (payload.get("static_reference_centerlines") or [])]

    inner_lo = tuple(chip_lo[i] + boundary_margin for i in range(3))
    inner_hi = tuple(chip_hi[i] - boundary_margin for i in range(3))

    nets = payload.get("nets") or []
    if not isinstance(nets, list) or not nets:
        raise ContractError("the artifact carries no net")

    # ---- occupancy bookkeeping -------------------------------------------
    occ_entries = ((payload.get("occupancy") or {}).get("entries")) or []
    published = []
    bookkeeping = {"ok": True, "problems": [], "n_entries": len(occ_entries)}
    for e in occ_entries:
        origin = e.get("origin")
        if origin not in ("static_reference_centerline", "committed_net_path"):
            bookkeeping["problems"].append("unknown occupancy origin %r" % (origin,))
            continue
        if origin == "committed_net_path":
            for field in ("net_id", "segment_id", "occupancy_revision"):
                if e.get(field) is None:
                    bookkeeping["problems"].append(
                        "published curve without %s: %r" % (field, e.get("fingerprint")))
            published.append(e)
    if bookkeeping["problems"]:
        bookkeeping["ok"] = False

    net_reports = []
    seg_by_net: Dict[str, List[Primitive]] = {}
    for net in nets:
        net_id = net.get("net_id")
        declared = net.get("status")
        report: Dict = {"net_id": net_id, "declared_status": declared,
                        "declared_accepted": bool(net.get("accepted")),
                        "index": net.get("index")}
        # A malformed pose or primitive REJECTS the whole artifact (ContractError
        # propagates): it is never silently turned into a per-net "fail", because a
        # reader must not be able to mistake "the file is broken" for "this net is
        # geometrically bad".
        source = parse_pose((net.get("input") or {}).get("source_pose"),
                            "%s.source_pose" % net_id)
        target = parse_pose((net.get("input") or {}).get("target_pose"),
                            "%s.target_pose" % net_id)
        segments = parse_segments(net.get("segments"))
        seg_by_net[net_id] = segments
        report["n_segments"] = len(segments)
        report["pose_contract_ok"] = True
        if not segments:
            report["verdict"] = ("no_path" if not net.get("accepted") else "fail")
            report["note"] = ("this net carries no returned segment; its declared "
                              "status is %r" % (declared,))
            net_reports.append(report)
            continue
        report["ports"] = check_ports(segments, source, target, src_pos_tol,
                                      src_ang_tol, eps_pos, eps_ang)
        report["c1"] = check_c1(segments, pos_tol, ang_tol)
        report["geometry"] = check_segments(segments, inner_lo, inner_hi, obstacles,
                                           clearance, rmin, radius_tol)
        report["self_spacing"] = self_spacing_global(segments, dmin, ell, tol)
        # The occupancy a net was REALLY searched against is the one published
        # BEFORE it: entry.occupancy_revision < this net's own publication
        # revision.  Curves published later are covered by the pairwise inter-net
        # check over the committed set (see below), not here.
        if net.get("committed"):
            limit = net.get("occupancy_revision_after")
        else:
            limit = net.get("occupancy_revision_before")
        earlier = [e for e in published
                   if e.get("net_id") != net_id
                   and (limit is None
                        or int(e.get("occupancy_revision", 0)) <= int(limit))]
        report["occupancy_revision_window"] = {
            "this_net_published_at": limit,
            "n_curves_of_earlier_nets": len(earlier),
            "rule": ("the candidate was checked against the curves published before "
                     "it; later curves are covered by the committed-set pairwise "
                     "inter-net check")}
        own_refs = [parse_primitive(e["geometry"]) for e in earlier]
        report["static_reference_spacing"] = check_reference_spacing(
            segments, static_refs, dmin, tol)
        report["published_occupancy_spacing"] = check_reference_spacing(
            segments, own_refs, dmin, tol)
        # occupancy identity: a committed net's occupancy must BE its own path
        if net.get("committed") or net.get("committed_segment_ids"):
            mine = [e for e in published if e.get("net_id") == net_id]
            raw_segments = net.get("segments") or []
            ids = sorted(int(e.get("segment_id")) for e in mine)
            expected = list(range(len(segments)))
            identical = (ids == expected)
            if identical:
                for e in mine:
                    k = int(e["segment_id"])
                    if e["geometry"] != raw_segments[k]:
                        identical = False
                        break
            report["occupancy_is_this_net_path"] = {
                "ok": identical, "n_published_curves": len(mine),
                "declared_committed_segment_ids": net.get("committed_segment_ids"),
                "expected_segment_ids": expected,
                "rule": ("a published curve must be exactly one primitive of the "
                         "returned path of the net it is attributed to, compared as "
                         "stored JSON geometry with no re-derivation")}
        ok = (report["ports"]["ok"] and report["c1"]["ok"]
              and report["geometry"]["ok"]
              and report["self_spacing"]["status"] == STATUS_SAFE
              and report["static_reference_spacing"]["status"] == STATUS_SAFE
              and report["published_occupancy_spacing"]["status"] == STATUS_SAFE)
        undecided = any(x["status"] == STATUS_UNDECIDED for x in
                        (report["self_spacing"], report["static_reference_spacing"],
                         report["published_occupancy_spacing"]))
        if report["geometry"]["checks"]["obstacle_clearance"]["status"] == \
                STATUS_UNDECIDED or \
                report["geometry"]["checks"]["boundary"]["status"] == STATUS_UNDECIDED:
            undecided = True
        declared_accepted = bool(net.get("accepted"))
        occupancy_ok = report.get("occupancy_is_this_net_path", {}).get("ok", True)
        report["independently_accepted"] = bool(ok and occupancy_ok)
        if undecided:
            verdict = "uncertified"
        elif declared_accepted:
            verdict = "pass" if (ok and occupancy_ok) else "fail"
        else:
            # the net was rejected by the scheduler: the rejection is CORROBORATED
            # only when this second implementation also finds a real failure
            verdict = "rejected_ok" if not ok else "fail"
        report["verdict"] = verdict
        report["agrees_with_declared_status"] = bool(
            report["independently_accepted"] == declared_accepted)
        net_reports.append(report)

    # ---- inter-net: every pair of COMMITTED nets ---------------------------
    # The invariant this round really establishes is that the PUBLISHED set is
    # mutually at or above dmin, because the scheduler checked each candidate
    # against the already published curves before publishing it.  A rejected
    # candidate is deliberately NOT part of that invariant (it was never
    # published); its spacing to the committed set is reported separately as
    # evidence, not as a run failure.
    committed_reports = [n for n in net_reports
                         if n.get("n_segments") and n.get("verdict") == "pass"]
    rejected_reports = [n for n in net_reports
                        if n.get("n_segments") and n.get("verdict") == "rejected_ok"]
    inter = []
    for x in range(len(committed_reports)):
        for y in range(x + 1, len(committed_reports)):
            ida = committed_reports[x]["net_id"]
            idb = committed_reports[y]["net_id"]
            inter.append(check_inter_net(seg_by_net[ida], ida, seg_by_net[idb], idb,
                                         dmin, tol))
    evidence = []
    for r in rejected_reports:
        for c in committed_reports:
            evidence.append(check_inter_net(seg_by_net[r["net_id"]], r["net_id"],
                                            seg_by_net[c["net_id"]], c["net_id"],
                                            dmin, tol))

    counts = {"n_nets": len(nets),
              "n_nets_with_a_path": sum(1 for r in net_reports if r.get("n_segments")),
              "n_pass": sum(1 for r in net_reports if r.get("verdict") == "pass"),
              "n_rejected_ok": sum(1 for r in net_reports
                                   if r.get("verdict") == "rejected_ok"),
              "n_fail": sum(1 for r in net_reports if r.get("verdict") == "fail"),
              "n_no_path": sum(1 for r in net_reports
                               if r.get("verdict") == "no_path"),
              "n_uncertified": sum(1 for r in net_reports
                                   if r.get("verdict") == "uncertified"),
              "n_committed_pairs_checked": len(inter),
              "n_inter_net_pairs": len(inter),
              "n_inter_net_violations": sum(1 for r in inter
                                            if r["status"] == STATUS_VIOLATION),
              "n_inter_net_undecided": sum(1 for r in inter
                                           if r["status"] == STATUS_UNDECIDED),
              "n_rejected_candidate_evidence_pairs": len(evidence),
              "n_rejected_candidate_evidence_violations":
                  sum(1 for r in evidence if r["status"] == STATUS_VIOLATION),
              "declared_verified": sum(1 for n in nets if n.get("accepted")),
              "declared_committed": sum(1 for n in nets if n.get("committed")),
              "n_disagreements": sum(1 for r in net_reports
                                     if r.get("agrees_with_declared_status") is False)}
    overall = ("fail" if (counts["n_fail"] or counts["n_inter_net_violations"]
                          or not bookkeeping["ok"])
               else ("uncertified" if (counts["n_uncertified"]
                                       or counts["n_inter_net_undecided"])
                     else "pass"))
    return {
        "artifact": "native3d_v020_joint_validation",
        "validator": ("native3d.validation.joint_validate (independent second "
                      "implementation, standard library only)"),
        "case": payload.get("case"),
        "units": UNITS,
        "channel_count": payload.get("channel_count"),
        "order": payload.get("order"),
        "overall_verdict": overall,
        "verdict_semantics": {
            "pass": ("the net is declared accepted and this second implementation "
                     "reproduces every acceptance check, including the occupancy "
                     "identity"),
            "rejected_ok": ("the net is declared NOT accepted and this second "
                            "implementation also finds a real geometric failure (or "
                            "the net carries no path at all, which this validator "
                            "cannot and does not re-derive)"),
            "no_path": "the net carries no returned segment",
            "fail": "a contradiction between the declared status and the geometry",
            "overall": ("fail only for a contradiction or a violation inside the "
                        "PUBLISHED set; a rejected candidate is not part of the "
                        "published-set invariant and is reported as evidence")},
        "counts": counts,
        "occupancy_bookkeeping": bookkeeping,
        "tolerances": {"dmin_centerline_um": dmin, "rmin_um": rmin,
                       "radius_tol_um": radius_tol, "pos_tol_um": pos_tol,
                       "ang_tol_deg": ang_tol, "eps_pos_um": eps_pos,
                       "eps_ang_deg": eps_ang,
                       "source_pos_tol_um": src_pos_tol,
                       "source_ang_tol_deg": src_ang_tol,
                       "clearance_decision_tol_um": tol,
                       "effective_obstacle_clearance_um": clearance,
                       "effective_boundary_margin_um": boundary_margin},
        "self_spacing_policy": policy,
        "nets": net_reports,
        "inter_net": inter,
        "rejected_candidate_spacing_evidence": evidence,
        "certification_classes": {
            "sampled_estimate": "measured at finitely many points; a sampled MINIMUM can only be too large",
            "certified_lower_bound": "can never exceed the true minimum",
            "violation_witness": "a point pair ON the curves, closer than required"},
        "no_optical_loss_model": ("geometry only: no dB, no insertion loss and no "
                                  "crosstalk number is produced here"),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        print("usage: python -m native3d.validation.joint_validate <artifact.json>")
        return 2
    rc = 0
    for path in argv:
        with open(path, "r", encoding="utf-8") as fh:
            payload = json.load(fh)
        try:
            res = validate_joint_artifact(payload)
        except ContractError as exc:
            print("%s: CONTRACT ERROR: %s" % (path, exc))
            rc = 1
            continue
        c = res["counts"]
        print("%s\n  case=%s channels=%s overall=%s\n"
              "  declared verified/committed = %d/%d, independent pass/fail/"
              "uncertified = %d/%d/%d, inter-net pairs=%d violations=%d "
              "undecided=%d, disagreements=%d"
              % (path, res["case"], res["channel_count"], res["overall_verdict"],
                 c["declared_verified"], c["declared_committed"], c["n_pass"],
                 c["n_fail"], c["n_uncertified"], c["n_inter_net_pairs"],
                 c["n_inter_net_violations"], c["n_inter_net_undecided"],
                 c["n_disagreements"]))
        if res["overall_verdict"] != "pass":
            rc = 1
    return rc


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
