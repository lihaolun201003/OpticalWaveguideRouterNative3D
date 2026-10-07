"""Repair C regression: the independent validator must not sell a sampled minimum
as a certified lower bound, and must decide line geometry analytically.

v0.1 used a coarse-scan + coordinate-descent search for segment/segment distance
(a lower bound on the TRUE minimum, i.e. an UPPER bound on the distance) and then
printed it as "conservative_clearance_lower_bound_um".  That produced the
impossible pair

    obstacle sampled minimum = 5.773917590493442 um
    claimed certified lower bound = 5.820272826928609 um

A lower bound above an upper bound is a mathematical contradiction, and the same
defect let a path cross a pre-existing centreline and still be reported "pass".
"""

import math

import pytest

from native3d.cases import base_config
from native3d.constraints.checks import Box3D, validate_segments
from native3d.geometry import CircularArc3D, Pose3D
from native3d.geometry.path3d import Path3D
from native3d.io.artifacts import build_artifact, finalise_artifact
from native3d.io.caseio import Scene
from native3d.search.router import route
from native3d.validation.validate import (point_box_distance, point_seg_distance,
                                          seg_seg_distance, validate_artifact,
                                          validate_path_dict)

BASE_CFG = base_config()


def cfg_dict(**over):
    d = {"effective_boundary_margin_um": 0.0, "effective_obstacle_clearance_um": 5.0,
         "dmin_centerline_um": 5.0, "rmin_um": 50.0, "radius_tol_um": 1e-6,
         "pos_tol_um": 1e-6, "ang_tol_deg": 1e-6, "clearance_decision_tol_um": 1e-9,
         "eps_pos_um": 2.0, "eps_ang_deg": 3.0}
    d.update(over)
    return d


def line_path(p0, p1):
    return {"segments": [{"kind": "line", "p0": list(p0), "p1": list(p1)}]}


def scene_dict(lo, hi, obstacles=(), refs=(), source=((0, 0, 0), (1, 0, 0)),
               target=((100, 0, 0), (1, 0, 0))):
    return {
        "chip_volume": {"lo": list(lo), "hi": list(hi)},
        "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
        "reference_centerlines": [dict(kind="line", p0=list(a), p1=list(b))
                                  for a, b in refs],
        "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
        "target_pose": {"position": list(target[0]), "tangent": list(target[1])},
    }


# --------------------------------------------------- distance-function level
def test_crossing_segments_have_distance_zero():
    """segment A: (0,0,0)->(100,0,0);  segment B: (3.125,-75,0)->(3.125,25,0)"""
    d = seg_seg_distance((0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                         (3.125, -75.0, 0.0), (3.125, 25.0, 0.0))
    assert d == pytest.approx(0.0, abs=1e-9), "the two centre lines really intersect"


@pytest.mark.parametrize("p1,q1,p2,q2,exact", [
    # parallel, 5 um apart
    ((0, 0, 0), (100, 0, 0), (0, 5, 0), (100, 5, 0), 5.0),
    # collinear and overlapping -> 0
    ((0, 0, 0), (100, 0, 0), (50, 0, 0), (150, 0, 0), 0.0),
    # collinear, disjoint
    ((0, 0, 0), (100, 0, 0), (160, 0, 0), (200, 0, 0), 60.0),
    # degenerate: a point against a segment
    ((50, 3, 0), (50, 3, 0), (0, 0, 0), (100, 0, 0), 3.0),
    # both degenerate
    ((1, 2, 3), (1, 2, 3), (4, 6, 3), (4, 6, 3), 5.0),
    # minimum at an endpoint
    ((0, 0, 0), (1, 0, 0), (5, 1, 0), (5, 9, 0), math.sqrt(17.0)),
    # skew lines
    ((0, 0, 0), (1, 0, 0), (0, 1, 1), (1, 1, 1), math.sqrt(2.0)),
    # touching at a shared endpoint
    ((0, 0, 0), (10, 0, 0), (0, 0, 0), (0, 10, 0), 0.0),
    # one segment entirely "above" the other, closest pair interior/interior
    ((0, 0, 0), (10, 0, 0), (5, 0, 7), (5, 10, 7), 7.0),
])
def test_analytic_segment_segment_cases(p1, q1, p2, q2, exact):
    d = seg_seg_distance(p1, q1, p2, q2)
    assert d == pytest.approx(exact, abs=1e-9)
    assert d == pytest.approx(seg_seg_distance(p2, q2, p1, q1), abs=1e-12)  # symmetry


def test_analytic_distance_is_exact_not_a_sampled_estimate():
    """A crossing pair must be exactly 0, not 'as small as the scan could see'."""
    for x in (0.390625, 3.125, 50.0, 99.999):
        d = seg_seg_distance((0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                             (x, -75.0, 0.0), (x, 25.0, 0.0))
        assert d < 1e-12, "x=%r" % x


# ------------------------------------------------- full-validator level
def test_validator_fails_a_path_crossing_a_reference_centreline():
    """Exact counterexample: sampled spacing 0.109375 < 0.3 but old verdict pass."""
    path = line_path((0, 0, 0), (100, 0, 0))
    scene = scene_dict((-200, -200, -200), (200, 200, 200),
                       refs=[((0.390625, -75, 0), (0.390625, 25, 0))])
    v = validate_path_dict(path, scene, cfg_dict(dmin_centerline_um=0.3))
    sp = v["centerline_spacing"]
    assert sp["min_sampled_um"] < 0.3
    assert sp["conservative_lower_bound_um"] <= sp["min_sampled_um"] + 1e-9, (
        "a lower bound may never exceed the sampled upper bound")
    assert v["verdict"] == "fail"
    assert v["violation_counts"]["spacing_below_required"] >= 1


def test_validator_fails_a_segment_passing_through_a_box_face_interior():
    path = line_path((0, 0, 0), (100, 0, 0))
    scene = scene_dict((-200, -200, -200), (200, 200, 200),
                       obstacles=[((40, -10, -10), (60, 10, 10))])
    v = validate_path_dict(path, scene, cfg_dict())
    assert v["obstacle_clearance"]["min_sampled_clearance_um"] == pytest.approx(0.0)
    assert v["verdict"] == "fail"
    assert v["violation_counts"]["obstacle_points_inside"] >= 1


def test_validator_exact_tangency_is_legal_and_epsilon_closer_is_not():
    box = [((0, 10, -10), (10, 20, 10))]
    def scene_at(y):
        return scene_dict((-100, -100, -100), (100, 100, 100), obstacles=box,
                          source=((-50, y, 0), (1, 0, 0)),
                          target=((50, y, 0), (1, 0, 0)))
    v = validate_path_dict(line_path((-50, 5, 0), (50, 5, 0)), scene_at(5), cfg_dict())
    assert v["obstacle_clearance"]["min_sampled_clearance_um"] == pytest.approx(5.0)
    assert v["obstacle_clearance"]["analytic_min_clearance_um"] == pytest.approx(5.0)
    assert v["verdict"] == "pass", v["violation_counts"]

    # one nanometre closer to the box than the required 5 um: must fail
    v2 = validate_path_dict(line_path((-50, 5.001, 0), (50, 5.001, 0)),
                            scene_at(5.001), cfg_dict())
    assert v2["obstacle_clearance"]["analytic_min_clearance_um"] == pytest.approx(4.999,
                                                                                  abs=1e-9)
    assert v2["verdict"] == "fail"


def test_validator_spacing_boundary_is_exact():
    ref = [((0, 5, 0), (100, 5, 0))]
    for y, expect in ((0.0, "pass"), (-0.001, "pass"), (0.001, "fail")):
        scene = scene_dict((-100, -100, -100), (100, 100, 100), refs=ref,
                           source=((0, y, 0), (1, 0, 0)),
                           target=((100, y, 0), (1, 0, 0)))
        v = validate_path_dict(line_path((0, y, 0), (100, y, 0)), scene, cfg_dict())
        assert v["centerline_spacing"]["min_sampled_um"] == pytest.approx(abs(y - 5.0))
        assert v["verdict"] == expect, (y, v["violation_counts"])


def test_arc_certified_lower_bound_is_below_an_independently_known_true_distance():
    """Right half circle R=50 about the origin against the half space x >= 60.

    The true minimum distance is exactly 10.0 um (attained at (50, 0, 0)), so the
    certified lower bound must stay below it and the sampled estimate must reach
    it: this is the ordering lower_bound <= true_distance <= sampled_minimum on a
    case whose exact answer is known independently of the validator.
    """
    seg = {"kind": "arc", "center": [0.0, 0.0, 0.0], "u": [1.0, 0.0, 0.0],
           "v": [0.0, 1.0, 0.0], "radius_um": 50.0, "theta_rad": math.pi}
    scene = scene_dict((-100, -100, -100), (100, 100, 100),
                       obstacles=[((60, -100, -100), (100, 100, 100))],
                       source=((50.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                       target=((-50.0, 0.0, 0.0), (0.0, -1.0, 0.0)))
    v = validate_path_dict({"segments": [seg]}, scene, cfg_dict())
    oc = v["obstacle_clearance"]
    true_distance = 10.0
    assert oc["min_sampled_clearance_um"] == pytest.approx(true_distance, abs=1e-6)
    assert oc["conservative_clearance_lower_bound_um"] <= true_distance + 1e-9, \
        "a certified lower bound may never exceed the true distance"
    assert oc["conservative_clearance_lower_bound_um"] <= \
        oc["min_sampled_clearance_um"] + 1e-9
    assert oc["status"] == "safe"
    assert v["verdict"] == "pass", v["violation_counts"]


def test_lower_bound_never_exceeds_the_sampled_upper_bound_on_a_real_path():
    cfg = base_config(max_expansions=5000, max_runtime_s=60.0,
                      straight_lengths_um=(20.0, 40.0), turn_angles_deg=(45.0, 90.0),
                      turn_radii_um=(50.0,), skew_axis_tilt_deg=(45.0,),
                      connector_radii_um=(50.0,))
    chip = Box3D((-100.0, -100.0, -100.0), (300.0, 300.0, 200.0))
    from native3d.geometry import LineSegment3D
    sc = Scene("validator_bounds", chip,
               [Box3D((60.0, -100.0, -100.0), (70.0, 100.0, 20.0))],
               [LineSegment3D((0.0, 150.0, -100.0), (0.0, 150.0, 150.0))],
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((200.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = route(sc, mode="native3d")
    assert res.status == "success"
    art = finalise_artifact(build_artifact(sc, res, "native3d", "pytest"))
    v = validate_artifact(art)
    for key, samp, lb in (("obstacle_clearance", "min_sampled_clearance_um",
                           "conservative_clearance_lower_bound_um"),
                          ("centerline_spacing", "min_sampled_um",
                           "conservative_lower_bound_um")):
        assert v[key][lb] <= v[key][samp] + 1e-9, key
        assert v[key]["status"] in ("safe", "violation", "undecided"), key
    assert v["obstacle_clearance"]["status"] == "safe"
    assert v["verdict"] == "pass"


# ------------------------------------------------------- continuous boundary
def _poking_arc(overshoot_um=1e-4, radius=50.0):
    """Arc whose x maximum exceeds the chip face by 'overshoot' BETWEEN samples.

    Sampling is 0.25 um of arclength (max(3, ceil(L/0.25)+1) points).  The
    extremum is placed exactly halfway between two samples, so every sample is
    inside the box while the arc itself is not.
    """
    L = 10.0
    s_ext = 5.125                     # position of the extremum along the arc
    alpha = -s_ext / radius           # global start angle
    theta = L / radius
    center = (200.0 + overshoot_um - radius, 0.0, 0.0)
    u = (math.cos(alpha), math.sin(alpha), 0.0)
    v = (-math.sin(alpha), math.cos(alpha), 0.0)
    return {"kind": "arc", "center": list(center), "u": list(u), "v": list(v),
            "radius_um": radius, "theta_rad": theta, "length_um": L}


def test_validator_detects_an_arc_that_leaves_the_box_between_two_samples():
    from native3d.validation.validate import eval_seg, seg_length
    seg = _poking_arc()
    L = seg_length(seg)
    npt = max(3, int(math.ceil(L / 0.25)) + 1)
    xs = [eval_seg(seg, L * i / (npt - 1))[0] for i in range(npt)]
    assert max(xs) < 200.0, "every sample must be inside the box (else no counterexample)"
    assert max(xs) > 199.99, "... but close to the face, so the test is meaningful"
    from native3d.validation.validate import eval_tangent_exact
    start = eval_seg(seg, 0.0)
    end = eval_seg(seg, L)
    scene = scene_dict((-200, -200, -200), (200, 200, 200),
                       source=(start, eval_tangent_exact(seg, 0.0)),
                       target=(end, eval_tangent_exact(seg, L)))
    v = validate_path_dict({"segments": [seg]}, scene, cfg_dict())
    assert v["verdict"] == "fail", "an arc outside the chip must never be a pass"
    assert v["boundary"]["violation_samples"] == 0, \
        "the counterexample is only sharp if the SAMPLES all look legal"
    assert v["boundary"]["certified_contained"] is False
    assert v["boundary"]["analytic_overshoot_um"] > 0.0


def test_production_arc_aabb_is_the_exact_analytic_box():
    """u=(-1,0,0), v=(0,-1,0), R=50, theta=pi/2 -> x in [-50, 0] (max is 0)."""
    arc = CircularArc3D((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0),
                        50.0, math.pi / 2.0)
    lo, hi = arc.aabb()
    pts = [arc.point_at(arc.length * i / 40000.0) for i in range(40001)]
    for i in range(3):
        assert lo[i] == pytest.approx(min(p[i] for p in pts), abs=1e-6)
        assert hi[i] == pytest.approx(max(p[i] for p in pts), abs=1e-6)
    assert hi[0] == pytest.approx(0.0, abs=1e-9)
    assert lo[0] == pytest.approx(-50.0, abs=1e-9)


def test_inflated_arc_aabb_does_not_falsely_reject_a_legal_arc():
    cfg = base_config()
    arc = CircularArc3D((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0),
                        50.0, math.pi / 2.0)
    chip = Box3D((-100.0, -100.0, -100.0), (25.0, 25.0, 100.0))
    assert validate_segments((arc,), chip, [], [], cfg)["ok"] is True, \
        "the true box is [-50,0]x[-50,0]; the inflated one reaches +50 in x"


def test_arc_aabb_matches_sampling_for_many_planes_and_both_sweep_signs():
    from native3d.cases import tilted_plane_arc
    cases = []
    for normal in ((0, 0, 1), (1, 1, 1), (1, -2, 0.5), (0, 1, 0)):
        for theta in (math.radians(40.0), math.radians(200.0),
                      -math.radians(75.0), -math.radians(300.0)):
            cases.append(tilted_plane_arc((3.0, -4.0, 5.0), normal, 30.0, theta))
    for arc in cases:
        lo, hi = arc.aabb()
        pts = [arc.point_at(arc.length * i / 8000.0) for i in range(8001)]
        for i in range(3):
            assert lo[i] == pytest.approx(min(p[i] for p in pts), abs=1e-5)
            assert hi[i] == pytest.approx(max(p[i] for p in pts), abs=1e-5)


def test_validator_reports_uncertified_instead_of_pass_when_it_cannot_decide():
    """A violation witness may never be overridden by a weak lower bound."""
    path = line_path((0, 0, 0), (100, 0, 0))
    scene = scene_dict((-200, -200, -200), (200, 200, 200),
                       refs=[((0.390625, -75, 0), (0.390625, 25, 0))],
                       obstacles=[((40, -10, -10), (60, 10, 10))])
    v = validate_path_dict(path, scene, cfg_dict(dmin_centerline_um=0.3))
    assert v["verdict"] == "fail"
    assert v["certification"]["verdict_is_a_proof_of_pass"] is False or \
        v["verdict"] != "pass"
    assert set(v["certification"]) >= {"all_checks_certified", "n_undecided",
                                       "sampling_is_only_an_estimate"}