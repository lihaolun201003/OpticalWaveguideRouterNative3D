"""v0.1.2 repair A5 regression: near-parallel segments, floating point cancellation.

v0.1.1 computed the segment/segment closest approach with the classic
"denom = a*e - b*b" formula.  For two almost parallel segments of length 1000 um
whose true distance is exactly 0 the products a*e and b*b are both ~1e12 and their
difference is ~1e-4, i.e. below the resolution of the operands: the computed
denominator (and therefore the closest-point parameter) is dominated by rounding
error and the reported distance becomes ~9.04e-07 um.

Counterexample actually run:

    A = (0,0,0)      -> (1000,0,0)
    B = (0,-5e-6,0)  -> (1000,5e-6,0)          true distance 0 (they cross)
    dmin_centerline_um = 5e-7

    v0.1.1 production spacing   : safe,   bound 9.039999999999993e-07 um
    v0.1.1 independent validator: pass,   analytic minimum 9.039999999999993e-07,
                                  while the sampled minimum is exactly 0

v0.1.2 computes the denominator as |d1 x d2|^2 (Lagrange identity:
a*e - b*b == |d1 x d2|^2, but without the catastrophic cancellation), and a
certified decision subtracts an explicit conservative floating point error bound;
when that bound cannot decide the comparison the decision is recomputed in exact
rational arithmetic, so an uncertifiable comparison can never be reported as safe.
"""

import math

import pytest

from native3d.config import RouterConfig
from native3d.constraints import checks as checks_mod
from native3d.constraints.checks import (segment_segment_distance,
                                         segment_segment_distance_certified)
from native3d.validation.validate import seg_seg_distance, validate_path_dict

A0, A1 = (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)
B0, B1 = (0.0, -5e-6, 0.0), (1000.0, 5e-6, 0.0)
DMIN = 5e-7


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def rot(p, axis, angle):
    """Rodrigues rotation, written out in the test (no router helper)."""
    n = math.sqrt(sum(c * c for c in axis))
    k = tuple(c / n for c in axis)
    c_, s_ = math.cos(angle), math.sin(angle)
    kxv = (k[1] * p[2] - k[2] * p[1], k[2] * p[0] - k[0] * p[2],
           k[0] * p[1] - k[1] * p[0])
    kdp = sum(k[i] * p[i] for i in range(3))
    return tuple(p[i] * c_ + kxv[i] * s_ + k[i] * kdp * (1.0 - c_) for i in range(3))


def shift(p, d):
    return tuple(p[i] + d[i] for i in range(3))


# ------------------------------------------------------- distance function
def test_the_near_parallel_crossing_pair_has_distance_zero():
    d = segment_segment_distance(A0, A1, B0, B1)
    assert d <= 1e-12, "the segments really cross at (500,0,0); got %r" % d


def test_the_independent_validator_agrees():
    assert seg_seg_distance(A0, A1, B0, B1) <= 1e-12
    assert seg_seg_distance(B0, B1, A0, A1) <= 1e-12


@pytest.mark.parametrize("scale", [1e-3, 1e-1, 1.0, 1e2, 1e3])
def test_scaled_versions_of_the_counterexample(scale):
    a0, a1 = tuple(c * scale for c in A0), tuple(c * scale for c in A1)
    b0, b1 = tuple(c * scale for c in B0), tuple(c * scale for c in B1)
    d = segment_segment_distance(a0, a1, b0, b1)
    assert d <= 1e-9 * max(1.0, scale), (scale, d)
    assert seg_seg_distance(a0, a1, b0, b1) <= 1e-9 * max(1.0, scale)


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (1e5, -2e5, 3e5),
                                    (-12345.6, 7.5, 1e4)])
def test_translated_versions_of_the_counterexample(offset):
    a0, a1 = shift(A0, offset), shift(A1, offset)
    b0, b1 = shift(B0, offset), shift(B1, offset)
    assert segment_segment_distance(a0, a1, b0, b1) <= 1e-9
    assert seg_seg_distance(a0, a1, b0, b1) <= 1e-9


@pytest.mark.parametrize("axis,angle", [((1, 1, 1), 0.7), ((0, 1, 0), 1.3),
                                        ((1, -2, 0.5), -2.1)])
def test_rotated_versions_of_the_counterexample(axis, angle):
    a0, a1 = rot(A0, axis, angle), rot(A1, axis, angle)
    b0, b1 = rot(B0, axis, angle), rot(B1, axis, angle)
    assert segment_segment_distance(a0, a1, b0, b1) <= 1e-9
    assert seg_seg_distance(a0, a1, b0, b1) <= 1e-9


# --------------------------------------------------------------- production
def test_production_spacing_is_no_longer_safe_for_the_counterexample():
    from native3d.constraints.checks import primitive_spacing
    from native3d.geometry import LineSegment3D
    cfg = RouterConfig(dmin_centerline_um=DMIN)
    res = primitive_spacing(LineSegment3D(A0, A1), [LineSegment3D(B0, B1)],
                            DMIN, cfg)
    assert res.status == "violation", (res.status, res.bound_um)
    assert res.bound_um <= 1e-12


def test_the_certified_decision_is_never_a_false_safe():
    status, lb, ub, method = segment_segment_distance_certified(
        A0, A1, B0, B1, DMIN, 0.0)
    assert status == "violation", (status, lb, ub, method)
    assert lb <= ub
    assert lb <= 1e-12


def test_the_certified_decision_uses_exact_arithmetic_when_it_cannot_decide():
    status, lb, ub, method = segment_segment_distance_certified(
        A0, A1, B0, B1, DMIN, 0.0)
    assert method == "exact_rational", method
    # a well conditioned pair is decided by the fast path
    status2, _lb2, _ub2, method2 = segment_segment_distance_certified(
        (0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (0.0, 5.0, 0.0), (100.0, 5.0, 0.0),
        1.0, 0.0)
    assert status2 == "safe"
    assert method2 == "stable_float"


# -------------------------------------------------------- independent check
def test_the_validator_is_not_a_pass_for_the_counterexample():
    path = {"segments": [{"kind": "line", "p0": list(A0), "p1": list(A1)}]}
    scene = {
        "chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [],
        "reference_centerlines": [{"kind": "line", "p0": list(B0), "p1": list(B1)}],
        "source_pose": {"position": list(A0), "tangent": [1.0, 0.0, 0.0]},
        "target_pose": {"position": list(A1), "tangent": [1.0, 0.0, 0.0]},
    }
    v = validate_path_dict(path, scene, cfg_dict(dmin_centerline_um=DMIN))
    sp = v["centerline_spacing"]
    assert sp["analytic_min_um"] <= 1e-12, "the v0.1.1 value was 9.04e-07"
    assert sp["min_sampled_um"] <= 1e-12
    assert sp["status"] == "violation"
    assert v["verdict"] == "fail"


def test_a_sampled_violation_witness_alone_must_not_be_overridden_by_a_bound():
    """Even if a lower bound looked safe, the sampled witness decides."""
    from native3d.validation import validate as V
    path = {"segments": [{"kind": "line", "p0": list(A0), "p1": list(A1)}]}
    scene = {
        "chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [],
        "reference_centerlines": [{"kind": "line", "p0": list(B0), "p1": list(B1)}],
        "source_pose": {"position": list(A0), "tangent": [1.0, 0.0, 0.0]},
        "target_pose": {"position": list(A1), "tangent": [1.0, 0.0, 0.0]},
    }
    saved = V.seg_seg_distance_certified
    try:
        # a bogus "certified safe at 1.0 um" analytic decision
        V.seg_seg_distance_certified = lambda *a, **k: ("safe", 1.0, 1.0, "stub")
        v = V.validate_path_dict(path, scene, cfg_dict(dmin_centerline_um=DMIN))
    finally:
        V.seg_seg_distance_certified = saved
    assert v["centerline_spacing"]["min_sampled_um"] <= 1e-12
    assert v["verdict"] == "fail", v["violation_counts"]


# -------------------------------------------- regression of the v0.1.1 cases
@pytest.mark.parametrize("p1,q1,p2,q2,exact", [
    ((0, 0, 0), (100, 0, 0), (0, 5, 0), (100, 5, 0), 5.0),
    ((0, 0, 0), (100, 0, 0), (50, 0, 0), (150, 0, 0), 0.0),
    ((0, 0, 0), (100, 0, 0), (160, 0, 0), (200, 0, 0), 60.0),
    ((50, 3, 0), (50, 3, 0), (0, 0, 0), (100, 0, 0), 3.0),
    ((1, 2, 3), (1, 2, 3), (4, 6, 3), (4, 6, 3), 5.0),
    ((0, 0, 0), (1, 0, 0), (5, 1, 0), (5, 9, 0), math.sqrt(17.0)),
    ((0, 0, 0), (1, 0, 0), (0, 1, 1), (1, 1, 1), math.sqrt(2.0)),
    ((0, 0, 0), (10, 0, 0), (5, 0, 7), (5, 10, 7), 7.0),
    ((0, 0, 0), (1e6, 0, 0), (0, 1e-3, 0), (1e6, 1e-3, 0), 1e-3),
])
def test_well_conditioned_cases_are_unchanged(p1, q1, p2, q2, exact):
    d = segment_segment_distance(p1, q1, p2, q2)
    assert d == pytest.approx(exact, abs=1e-9 * max(1.0, exact))
    assert seg_seg_distance(p1, q1, p2, q2) == pytest.approx(
        exact, abs=1e-9 * max(1.0, exact))


def _exact_squared_distance(p1, q1, p2, q2):
    """Exact squared distance, enumerated as feature pairs (Fractions).

    Independent formulation: the squared distance is convex on [0,1]^2, so its
    minimum is either at the interior stationary point or on the boundary; the
    boundary is s in {0,1} or t in {0,1}, which are exactly point/segment
    distances.  Everything is exact rational arithmetic.
    """
    from fractions import Fraction as F

    def f(v):
        return tuple(F(x) for x in v)

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    def sub(a, b):
        return tuple(a[i] - b[i] for i in range(3))

    def point_seg2(p, a, b):
        ab = sub(b, a)
        L2 = dot(ab, ab)
        if L2 == 0:
            r = sub(p, a)
            return dot(r, r)
        t = dot(sub(p, a), ab) / L2
        t = max(F(0), min(F(1), t))
        r = sub(p, tuple(a[i] + ab[i] * t for i in range(3)))
        return dot(r, r)

    P1, Q1, P2, Q2 = f(p1), f(q1), f(p2), f(q2)
    best = min(point_seg2(P1, P2, Q2), point_seg2(Q1, P2, Q2),
               point_seg2(P2, P1, Q1), point_seg2(Q2, P1, Q1))
    d1 = sub(Q1, P1)
    d2 = sub(Q2, P2)
    r = sub(P1, P2)
    a, e, b, c, fv = dot(d1, d1), dot(d2, d2), dot(d1, d2), dot(d1, r), dot(d2, r)
    denom = a * e - b * b
    if denom > 0:
        s = (b * fv - c * e) / denom
        t = (b * s + fv) / e
        if 0 <= s <= 1 and 0 <= t <= 1:
            w = sub(tuple(P1[i] + d1[i] * s for i in range(3)),
                    tuple(P2[i] + d2[i] * t for i in range(3)))
            best = min(best, dot(w, w))
    return best


@pytest.mark.parametrize("p1,q1,p2,q2", [
    ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (0.0, 3.0, 0.0), (100.0, 3.0, 0.0)),
    ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (30.0, -10.0, 5.0), (30.0, 10.0, 5.0)),
    ((0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (0.0, 5.0, 0.0), (4.0, 5.0, 0.0)),
    ((0.0, 0.0, 0.0), (1000.0, 0.0, 0.0), (0.0, -5e-6, 0.0), (1000.0, 5e-6, 0.0)),
    ((0.0, 0.0, 0.0), (1000.0, 0.0, 0.0), (0.0, 5e-6, 0.0), (1000.0, 5e-6, 0.0)),
    ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (160.0, 0.0, 0.0), (200.0, 0.0, 0.0)),
])
def test_the_distance_matches_an_independent_exact_formulation(p1, q1, p2, q2):
    exact2 = _exact_squared_distance(p1, q1, p2, q2)
    exact = math.sqrt(float(exact2))
    d = segment_segment_distance(p1, q1, p2, q2)
    assert d == pytest.approx(exact, abs=1e-9 * max(1.0, exact)), (d, exact)
    assert seg_seg_distance(p1, q1, p2, q2) == pytest.approx(
        exact, abs=1e-9 * max(1.0, exact))
    # a required distance ABOVE the true distance must be a violation ...
    status, lb, ub, _method = segment_segment_distance_certified(
        p1, q1, p2, q2, exact + 0.5, 0.0)
    assert status == "violation", (status, d, exact)
    # ... and one BELOW it must be safe
    status2, lb2, ub2, _m2 = segment_segment_distance_certified(
        p1, q1, p2, q2, max(0.0, exact - 0.5), 0.0)
    assert status2 == "safe", (status2, d, exact)
    assert lb <= ub and lb2 <= ub2


def test_the_numeric_scale_of_the_certified_decision_is_documented():
    doc = checks_mod.__doc__ or ""
    assert "certified" in doc.lower()
    assert "exact rational" in doc.lower() or "exact-rational" in doc.lower()
    assert checks_mod.SEGMENT_DISTANCE_ERROR_FLOOR_UM > 0.0
