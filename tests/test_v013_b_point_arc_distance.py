"""v0.1.3 PART B1 regression: 3D point to PLANAR ARC distance.

THE DEFECT (v0.1.2, reproduced for real before the repair)

``validate.point_arc_distance`` returned ``abs(|p - C| - R)``, the radial
difference to the SPHERE through the arc, projected on the in-plane phase of p.
For the XY reference circle C = (0,0,0), u = +x, v = +y, R = 50, theta = 2*pi and
p = (0, 0, 50) - a point on the circle-plane NORMAL through the centre, i.e.
70.710678118654 um away from the curve - it returned exactly 0.0.  The frozen
v0.1.2 snapshot is frozen under artifacts/v013/prefix_src/ and the value is
checked from a test below through tests/v013_helpers.prefix_module, so the
red -> green transition is executable evidence and not prose.

THE CORRECT 3D DISTANCE (implemented in point_arc_distance_witness)

With q = p - C, a = q.u, b = q.v, h = q.(u x v) and rho = hypot(a, b):

    |P(phi) - p|^2 = R^2 + rho^2 + h^2 - 2 R rho cos(phi - phi*),
    phi* = atan2(b, a)                        (nearest FULL-CIRCLE phase)

so the distance is sqrt((rho - R)^2 + h^2) whenever some phi* + 2 k pi is inside
the phase interval the signed sweep really visits ([0, theta] for theta > 0 and
[theta, 0] for theta < 0 - the arc is traversed BACKWARDS for theta < 0, which is
why atan2() may not simply be compared against (0, theta)).  When no such phase
is reachable, the distance along the circle has an interior MAXIMUM and no
interior minimum, so the minimum is attained at one of the two arc endpoints.
rho == 0 (the point is on the centre axis) is sqrt(R^2 + h^2) and is never 0.

THE FULL-PATH COUNTEREXAMPLE (B1 acceptance)

    path      : (-10,0,50) -> (10,0,50)          (a legal straight segment)
    reference : the XY full circle R = 50 above
    chip      : [-2000,2000]^3, no obstacle, source/target = the path's poses

true minimum spacing = sqrt(40^2 + 50^2) = 64.031242374328 um > dmin = 5 um, the
production constraint stack says SAFE, and v0.1.2 reported sampled minimum 0.0,
verdict "fail" and a broken certification self check.  Both are checked here.

EVERY reported distance is accompanied by a WITNESS (point, phase, arclength)
that really lies on the curve; the regressions recompute |witness - p| and require
it to agree with the reported distance to 1e-9 um.  Real crossings still fail.
"""

from __future__ import annotations

import math

import pytest

from native3d.config import RouterConfig
from native3d.validation.validate import (eval_seg, point_arc_distance,
                                          point_arc_distance_witness, seg_length,
                                          validate_path_dict)

from v013_helpers import prefix_module

XY_FULL_CIRCLE = {"kind": "arc", "center": [0.0, 0.0, 0.0], "u": [1.0, 0.0, 0.0],
                  "v": [0.0, 1.0, 0.0], "radius_um": 50.0,
                  "theta_rad": 2.0 * math.pi}
FLYOVER_TRUE_MIN_UM = math.sqrt(40.0 ** 2 + 50.0 ** 2)
WITNESS_TOL_UM = 1e-9


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def arc(center, u, v, R, theta):
    return {"kind": "arc", "center": list(center), "u": list(u), "v": list(v),
            "radius_um": float(R), "theta_rad": float(theta)}


def scene_dict(refs, source, target, obstacles=()):
    return {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
            "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
            "reference_centerlines": list(refs),
            "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
            "target_pose": {"position": list(target[0]), "tangent": list(target[1])}}


def brute_min_distance(p, seg, n=20000):
    """Independent oracle: minimum over n + 1 points sampled ON the curve."""
    L = seg_length(seg)
    return min(math.dist(p, eval_seg(seg, L * i / n)) for i in range(n + 1))


def assert_witness_is_on_the_curve(p, seg, w):
    """Recompute the reported distance from the witness itself."""
    assert abs(math.dist(p, w["witness_point"]) - w["distance_um"]) <= WITNESS_TOL_UM
    assert w["witness_residual_um"] <= WITNESS_TOL_UM
    R = float(seg["radius_um"])
    c = seg["center"]
    assert math.dist(w["witness_point"], c) == pytest.approx(R, abs=1e-9)
    L = seg_length(seg)
    assert -1e-9 <= w["witness_arclength_um"] <= L + 1e-9
    assert eval_seg(seg, w["witness_arclength_um"]) == pytest.approx(
        tuple(w["witness_point"]), abs=1e-9)


# ===========================================================================
# the function-level counterexample
# ===========================================================================
def test_the_point_counterexample_is_the_true_3d_distance():
    """p = (0,0,50) on the circle-plane normal: 70.710678118654 um, never 0."""
    p = (0.0, 0.0, 50.0)
    got = point_arc_distance(p, XY_FULL_CIRCLE)
    true = math.sqrt(50.0 ** 2 + 50.0 ** 2)
    assert true == pytest.approx(70.710678118654, abs=1e-9)
    assert got == pytest.approx(true, abs=1e-12)
    assert got > 0.0
    w = point_arc_distance_witness(p, XY_FULL_CIRCLE)
    assert w["branch"] == "centre_axis"
    assert w["rho_um"] == 0.0
    assert w["h_um"] == pytest.approx(50.0)
    assert w["distance_um"] == pytest.approx(math.sqrt(50.0 ** 2 + 50.0 ** 2))
    assert_witness_is_on_the_curve(p, XY_FULL_CIRCLE, w)


def test_the_frozen_v012_snapshot_really_returned_zero_for_that_point():
    """Red -> green in ONE executable test, from the frozen v0.1.2 source."""
    frozen = prefix_module("validation.validate")
    p = (0.0, 0.0, 50.0)
    assert frozen.point_arc_distance(p, XY_FULL_CIRCLE) == 0.0
    assert point_arc_distance(p, XY_FULL_CIRCLE) == pytest.approx(
        math.sqrt(50.0 ** 2 + 50.0 ** 2), abs=1e-12)


# ===========================================================================
# the full-path counterexample (the acceptance of PART B1)
# ===========================================================================
def flyover_path(z=50.0):
    return {"segments": [{"kind": "line", "p0": [-10.0, 0.0, z],
                          "p1": [10.0, 0.0, z]}]}


def flyover_scene(z=50.0):
    return scene_dict([XY_FULL_CIRCLE], ((-10.0, 0.0, z), (1.0, 0.0, 0.0)),
                      ((10.0, 0.0, z), (1.0, 0.0, 0.0)))


def test_the_full_path_counterexample_is_now_accepted():
    z = 50.0
    v = validate_path_dict(flyover_path(z), flyover_scene(z), cfg_dict())
    sp = v["centerline_spacing"]
    assert FLYOVER_TRUE_MIN_UM == pytest.approx(64.031242374328, abs=1e-9)
    assert v["verdict"] == "pass", (v["verdict"], sp["status"])
    assert sp["status"] == "safe"
    assert sp["min_sampled_um"] == pytest.approx(FLYOVER_TRUE_MIN_UM, abs=1e-9)
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9
    assert v["certification"]["self_check_failed"] is False
    assert v["certification"]["all_checks_certified"] is True
    assert v["obstacle_clearance"]["status"] == "safe"


def test_the_frozen_v012_snapshot_really_rejected_the_same_legal_path():
    """The exact v0.1.2 result of the counterexample, from the frozen snapshot."""
    z = 50.0
    frozen = prefix_module("validation.validate")
    v0 = frozen.validate_path_dict(flyover_path(z), flyover_scene(z), cfg_dict())
    assert v0["verdict"] == "fail"
    assert v0["centerline_spacing"]["min_sampled_um"] == 0.0
    assert v0["certification"]["self_check_failed"] is True
    # ... and the repaired validator accepts it
    v1 = validate_path_dict(flyover_path(z), flyover_scene(z), cfg_dict())
    assert v1["verdict"] == "pass"
    assert v1["certification"]["self_check_failed"] is False


@pytest.mark.parametrize("z", [1.0, 10.0, 25.0, 50.0, 90.0])
def test_the_flyover_spacing_is_the_exact_3d_distance_for_every_height(z):
    v = validate_path_dict(flyover_path(z), flyover_scene(z), cfg_dict())
    true_min = math.sqrt(40.0 ** 2 + z ** 2)
    assert v["verdict"] == "pass"
    sp = v["centerline_spacing"]
    assert sp["min_sampled_um"] == pytest.approx(true_min, abs=1e-9)
    assert sp["min_sampled_um"] >= true_min - 1e-9          # upper bound only
    assert sp["conservative_lower_bound_um"] <= true_min    # lower bound only
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9


# ===========================================================================
# the individual geometry cases the formula must handle
# ===========================================================================
@pytest.mark.parametrize("h", [-50.0, -7.5, 0.0, 5.0, 50.0, 123.0])
@pytest.mark.parametrize("theta", [math.pi / 2.0, -math.pi / 2.0, 2.0 * math.pi,
                                   -2.0 * math.pi, 4.0 * math.pi, 8.0 * math.pi])
def test_points_on_the_centre_axis_are_never_reported_as_zero(h, theta):
    seg = arc((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, theta)
    p = (0.0, 0.0, h)
    expected = math.sqrt(50.0 ** 2 + h ** 2)
    w = point_arc_distance_witness(p, seg)
    assert w["distance_um"] == pytest.approx(expected, abs=1e-12)
    assert w["distance_um"] >= 50.0
    assert w["distance_um"] != 0.0
    assert w["branch"] == "centre_axis"
    assert_witness_is_on_the_curve(p, seg, w)


def test_the_sweep_can_miss_the_nearest_full_circle_phase():
    """A 60 deg arc: the nearest circle phase is outside it, so endpoints decide."""
    seg = arc((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, math.pi / 3.0)
    R, th = 50.0, math.pi / 3.0
    # p = (0, -100, 0): phi* = -pi/2 is NOT in [0, pi/3]; the START endpoint wins
    p = (0.0, -100.0, 0.0)
    start = (R, 0.0, 0.0)
    end = (R * math.cos(th), R * math.sin(th), 0.0)
    expected = min(math.dist(p, start), math.dist(p, end))
    assert expected == pytest.approx(math.dist(p, start))
    w = point_arc_distance_witness(p, seg)
    assert w["branch"] == "start_endpoint"
    assert w["distance_um"] == pytest.approx(expected, abs=1e-12)
    assert w["witness_point"] == pytest.approx(start, abs=1e-12)
    # the full circle would be much closer: the arc distance must be LARGER
    assert w["full_circle_distance_um"] == pytest.approx(50.0)
    assert w["distance_um"] > w["full_circle_distance_um"]
    assert_witness_is_on_the_curve(p, seg, w)

    # p = (0, +100, 0): phi* = +pi/2 is NOT in [0, pi/3] either; END wins now
    p2 = (0.0, 100.0, 0.0)
    expected2 = min(math.dist(p2, start), math.dist(p2, end))
    assert expected2 == pytest.approx(math.dist(p2, end))
    w2 = point_arc_distance_witness(p2, seg)
    assert w2["branch"] == "end_endpoint"
    assert w2["distance_um"] == pytest.approx(expected2, abs=1e-12)
    assert w2["witness_point"] == pytest.approx(end, abs=1e-12)
    assert_witness_is_on_the_curve(p2, seg, w2)


@pytest.mark.parametrize("theta", [2.0 * math.pi, 5.0 * math.pi / 2.0,
                                   4.0 * math.pi, 8.0 * math.pi,
                                   -2.0 * math.pi, -4.0 * math.pi,
                                   -8.0 * math.pi])
def test_every_multi_turn_sweep_reaches_the_nearest_phase(theta):
    """Anything spanning >= 2 pi must contain every phase, so the formula holds."""
    seg = arc((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, theta)
    points = [(70.0, 0.0, 0.0), (0.0, -70.0, 20.0), (-30.0, 40.0, -5.0),
              (0.0, 0.0, 17.0)]
    for p in points:
        rho = math.hypot(p[0], p[1])
        h = p[2]
        w = point_arc_distance_witness(p, seg)
        if rho == 0.0:
            # on the centre axis: every phase attains sqrt(R^2 + h^2)
            assert w["branch"] == "centre_axis"
            assert w["distance_um"] == pytest.approx(math.sqrt(50.0 ** 2 + h ** 2))
        else:
            assert w["branch"] == "nearest_full_circle_phase"
            assert w["distance_um"] == pytest.approx(
                math.sqrt((rho - 50.0) ** 2 + h ** 2), abs=1e-12)
        assert_witness_is_on_the_curve(p, seg, w)


def test_a_full_turn_whose_endpoints_coincide_is_still_a_full_turn():
    """cos() periodicity must not collapse the seam of a 2 pi sweep."""
    seg = arc((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, 2.0 * math.pi)
    L = seg_length(seg)
    assert math.dist(eval_seg(seg, 0.0), eval_seg(seg, L)) < 1e-12
    # a query point whose nearest phase is exactly the seam
    p = (70.0, 0.0, 0.0)
    assert point_arc_distance(p, seg) == pytest.approx(20.0, abs=1e-12)
    # and one whose representative needs the +2 pi wrap
    p2 = (0.0, -70.0, 0.0)
    w = point_arc_distance_witness(p2, seg)
    assert w["distance_um"] == pytest.approx(20.0, abs=1e-12)
    assert w["witness_phase_rad"] == pytest.approx(3.0 * math.pi / 2.0, abs=1e-12)


def test_a_sweep_that_crosses_the_periodic_cut():
    """u = -x, v = -y: the covered phases cross +-pi when the sweep turns negative."""
    neg = arc((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), 50.0,
              -math.pi / 2.0)
    pos = arc((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), 50.0,
              math.pi / 2.0)
    # p = (0, 50, 0) has phi* = -pi/2 (== 3 pi / 2): reachable for the NEGATIVE
    # sweep [-pi/2, 0], NOT reachable for the positive one [0, pi/2].
    p = (0.0, 50.0, 0.0)
    assert p == pytest.approx(eval_seg(neg, seg_length(neg)), abs=1e-12)
    w_neg = point_arc_distance_witness(p, neg)
    assert w_neg["branch"] == "nearest_full_circle_phase"
    assert w_neg["distance_um"] == pytest.approx(0.0, abs=1e-12)
    assert_witness_is_on_the_curve(p, neg, w_neg)

    w_pos = point_arc_distance_witness(p, pos)
    assert w_pos["branch"] == "start_endpoint"
    expected = math.dist(p, eval_seg(pos, 0.0))     # (-50, 0, 0) is nearest
    assert w_pos["distance_um"] == pytest.approx(expected, abs=1e-12)
    assert w_pos["distance_um"] == pytest.approx(math.sqrt(50.0 ** 2 + 50.0 ** 2))
    assert_witness_is_on_the_curve(p, pos, w_pos)


def test_a_negative_sweep_is_the_same_point_set_as_its_mirror():
    """(C, u, v, R, -theta) and (C, u, -v, R, +theta) are the SAME curve.

    A negative sweep traverses [theta, 0] backwards; flipping the sign of v turns
    that very traversal into [0, |theta|] with the same start point, the same
    tangent and the same points.  Distances to the two descriptions must agree,
    which is exactly what comparing atan2() with (0, theta) gets wrong.
    """
    R = 37.5
    th = 2.0 * math.pi + 0.7
    neg = arc((3.0, -4.0, 5.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), R, -th)
    mir = arc((3.0, -4.0, 5.0), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0), R, th)
    L = seg_length(neg)
    for i in range(41):
        s = L * i / 40.0
        assert math.dist(eval_seg(neg, s), eval_seg(mir, s)) < 1e-9
    for p in ((0.0, 0.0, 5.0), (40.0, -60.0, 5.0), (3.0, -4.0, 55.0),
              (-25.0, 11.0, -4.0)):
        a = point_arc_distance(p, neg)
        b = point_arc_distance(p, mir)
        assert a == pytest.approx(b, abs=1e-9), p
        # every sampled point of the curve itself is at distance 0
        for i in range(0, 11):
            q = eval_seg(neg, L * i / 10.0)
            assert point_arc_distance(q, neg) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("normal", [(0.0, 0.0, 1.0), (1.0, 1.0, 1.0),
                                    (1.0, -2.0, 0.5), (0.0, 1.0, 0.0)])
@pytest.mark.parametrize("theta", [1.1, 2.0 * math.pi + 0.4, -2.3,
                                   -4.0 * math.pi - 0.3])
def test_arbitrary_tilted_planes_agree_with_an_independent_brute_force(normal, theta):
    """An orthonormal basis in an arbitrary plane, checked against sampling."""
    n = list(normal)
    nn = math.sqrt(sum(c * c for c in n))
    n = [c / nn for c in n]
    ref = (1.0, 0.0, 0.0)
    if abs(sum(ref[i] * n[i] for i in range(3))) > 0.9:
        ref = (0.0, 1.0, 0.0)
    d = sum(ref[i] * n[i] for i in range(3))
    u = [ref[i] - d * n[i] for i in range(3)]
    nu = math.sqrt(sum(c * c for c in u))
    u = [c / nu for c in u]
    v = [n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2],
         n[0] * u[1] - n[1] * u[0]]
    seg = arc((10.0, -4.0, 7.0), u, v, 37.5, theta)
    L = seg_length(seg)
    n_samp = 4000
    pts = [(60.0, 0.0, 0.0), (2.0, 0.0, 45.0), (-30.0, -30.0, -30.0),
           eval_seg(seg, L * 0.37), eval_seg(seg, L * 0.999),
           eval_seg(seg, L * 1.001)]
    for p in pts:
        w = point_arc_distance_witness(p, seg)
        brute = brute_min_distance(p, seg, n=n_samp)
        assert w["distance_um"] <= brute + 1e-9, (p, w, brute)
        assert brute - w["distance_um"] <= L / (2.0 * n_samp) + 1e-9, (p, brute)
        assert_witness_is_on_the_curve(p, seg, w)


@pytest.mark.parametrize("seg_kwargs", [
    dict(center=(0.0, 0.0, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0), R=50.0,
         theta=math.pi / 3.0),
    dict(center=(5.0, 5.0, 5.0), u=(0.0, 0.0, 1.0), v=(1.0, 0.0, 0.0), R=12.0,
         theta=-math.pi / 2.0),
    dict(center=(0.0, 0.0, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 0.0, 1.0), R=50.0,
         theta=2.0 * math.pi),
    dict(center=(0.0, 0.0, 0.0), u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0), R=50.0,
         theta=8.0 * math.pi),
])
def test_every_reported_distance_comes_with_a_real_witness(seg_kwargs):
    seg = arc(seg_kwargs["center"], seg_kwargs["u"], seg_kwargs["v"],
              seg_kwargs["R"], seg_kwargs["theta"])
    R = seg_kwargs["R"]
    c = seg_kwargs["center"]
    for p in ((0.0, 0.0, 0.0), (R + 3.0, 0.0, 0.0), (0.0, -R, 11.0),
              (c[0] + R, c[1], c[2]), tuple(c), (c[0], c[1], c[2] + 2.0 * R)):
        w = point_arc_distance_witness(p, seg)
        assert w["witness_residual_um"] <= WITNESS_TOL_UM
        assert_witness_is_on_the_curve(p, seg, w)
        brute = brute_min_distance(p, seg)
        L = seg_length(seg)
        assert w["distance_um"] <= brute + 1e-9
        assert brute - w["distance_um"] <= L / (2.0 * 20000) + 1e-9


# ===========================================================================
# real violations must still fail
# ===========================================================================
def test_a_path_that_actually_crosses_the_reference_arc_still_fails():
    """The path really passes through the circle: (-100,0,0) -> (100,0,0)."""
    path = {"segments": [{"kind": "line", "p0": [-100.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    scene = scene_dict([XY_FULL_CIRCLE], ((-100.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                       ((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    v = validate_path_dict(path, scene, cfg_dict())
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(0.0, abs=1e-9)
    assert v["verdict"] == "fail"


def test_a_path_that_crosses_a_reference_LINE_still_fails():
    ref = {"kind": "line", "p0": [50.0, -50.0, 0.0], "p1": [50.0, 50.0, 0.0]}
    path = {"segments": [{"kind": "line", "p0": [-100.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    v = validate_path_dict(path, scene_dict([ref], ((-100.0, 0.0, 0.0),
                                                   (1.0, 0.0, 0.0)),
                                            ((100.0, 0.0, 0.0),
                                             (1.0, 0.0, 0.0))), cfg_dict())
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["verdict"] == "fail"


def test_an_arc_that_reaches_into_the_keep_out_is_still_a_violation():
    """The arc itself is the path; a reference line crosses it (-50, 0, 0)."""
    big = arc((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0,
              8.0 * math.pi)
    ref = {"kind": "line", "p0": [-51.0, -1.0, 0.0], "p1": [-49.0, 1.0, 0.0]}
    L = seg_length(big)
    v = validate_path_dict(
        {"segments": [big]},
        scene_dict([ref], (eval_seg(big, 0.0),
                           (0.0, 1.0, 0.0)),
                   (eval_seg(big, L), (0.0, 1.0, 0.0))), cfg_dict())
    assert v["centerline_spacing"]["status"] == "violation"
    assert v["centerline_spacing"]["min_sampled_um"] < 5.0
    assert v["verdict"] == "fail"
    assert v["certification"]["self_check_failed"] is False
