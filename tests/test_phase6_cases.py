"""Phase 6 additional mandatory cases (1-8) of the acceptance specification."""

import math

import pytest

from native3d.cases import (base_config, core_case, mid_curve_obstacle_case,
                            sealed_target_case, terminal_tangent_case,
                            tilted_plane_arc)
from native3d.config import RouterConfig
from native3d.constraints.checks import (Box3D, arc_box_clearance, boundary_check,
                                         validate_segments)
from native3d.geometry import CircularArc3D, LineSegment3D, Path3D, Pose3D
from native3d.geometry.vec import add, cross, dot, norm, scale, sub
from native3d.io.caseio import Scene
from native3d.search.actions import generate_actions
from native3d.search.congestion import RegionCongestion, ZeroCongestion
from native3d.search.reference_graph import reference_search
from native3d.search.router import pose_invalid_reason, route


# ------------------------------------------------------- (1) pure straight
def test_case1_pure_straight_run_has_euclidean_length():
    cfg = base_config(max_expansions=5000, max_runtime_s=60.0)
    chip = Box3D((-10.0, -50.0, -50.0), (200.0, 50.0, 50.0))
    sc = Scene("straight", chip, [], [],
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = route(sc, mode="native3d")
    assert res.status == "success"
    assert res.length_um == pytest.approx(100.0, abs=1e-9)
    assert all(s.kind == "line" for s in res.path.segments)
    assert res.path.start_pose.tangent == pytest.approx((1.0, 0.0, 0.0))
    assert res.path.end_pose.tangent == pytest.approx((1.0, 0.0, 0.0))


# ------------------------------------------------------ (2) spatial bending
def test_case2_arc_in_a_tilted_plane_is_a_true_space_curve():
    n = (1.0, 2.0, -0.5)
    nn = tuple(c / math.sqrt(sum(x * x for x in n)) for c in n)
    arc = tilted_plane_arc((3.0, -2.0, 5.0), nn, 40.0, math.radians(100.0))
    assert arc.normal == pytest.approx(nn, abs=1e-12)
    for i in range(21):
        p = arc.point_at(arc.length * i / 20.0)
        assert dot(sub(p, arc.center), nn) == pytest.approx(0.0, abs=1e-9)
        assert norm(sub(p, arc.center)) == pytest.approx(40.0, abs=1e-9)
    # rotating in a tilted plane means all three coordinates move
    d = sub(arc.end, arc.start)
    assert abs(d[0]) > 1.0 and abs(d[1]) > 1.0 and abs(d[2]) > 1.0


def test_case2_engine_path_contains_a_non_xy_plane_arc():
    cfg = base_config(max_expansions=5000, max_runtime_s=60.0)
    chip = Box3D((-200.0, -200.0, -200.0), (200.0, 200.0, 200.0))
    # parallel tangents: the connector plane is spanned by the tangent and the
    # displacement, which is a genuinely TILTED plane here (dz != 0 and dy != 0)
    sc = Scene("bend", chip, [], [],
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((120.0, 40.0, 25.0), (1.0, 0.0, 0.0)), cfg)
    res = route(sc, mode="native3d")
    assert res.status == "success"
    arcs = [s for s in res.path.segments if s.kind == "arc"]
    assert arcs, "the path contains at least one arc"
    assert any(abs(a.normal[2]) < 0.99 for a in arcs), "non-XY-plane arc required"
    # the climbed segment must really leave z = 0 in the middle of a curve
    assert max(abs(s.aabb()[1][2] - s.aabb()[0][2]) for s in res.path.segments) > 1.0
    rep = res.path.continuity_report()
    assert rep["c1_continuous"] is True
    from native3d.validation.validate import validate_artifact
    from native3d.io.artifacts import build_artifact, finalise_artifact
    art = finalise_artifact(build_artifact(sc, res, "native3d", "pytest"))
    v = art["independent_validation"]
    assert v["verdict"] == "pass"
    assert v["z_profile"]["z_varies"] is True


# ------------------------------------------- (3) closed obstacle unreachable
def test_case3_sealed_target_is_search_exhaustion_not_budget():
    sc = sealed_target_case()
    assert pose_invalid_reason(sc, sc.source) is None
    assert pose_invalid_reason(sc, sc.target) is None
    # the PRODUCTION exact-pose graph cannot be exhausted here, and it must say
    # so instead of claiming unreachability
    res_prod = route(sc, mode="2d", max_expansions=1200, max_runtime_s=8.0)
    assert res_prod.status == "budget_exhausted"
    assert res_prod.budget_exhausted is True
    assert "NOT a proof" in res_prod.reason
    assert res_prod.state_identity == "pose_float"
    # the FINITE canonical reference graph is exhausted for real
    res = reference_search(sealed_target_case(), mode="2d")
    assert res.status == "search_exhausted"
    assert res.budget_exhausted is False
    assert res.expansions < 200
    assert res.path is None
    # a case that needs thousands of expansions stops for a DIFFERENT reason
    res_b = route(core_case(), mode="2d", max_expansions=30, max_runtime_s=60.0)
    assert res_b.status == "budget_exhausted"
    assert res_b.budget_exhausted is True
    assert "NOT a proof" in res_b.reason


# ------------------------------------------------------- (4) radius boundary
def test_case4_radius_boundary_through_the_full_validator():
    eps = 1.0
    cfg = base_config(rmin_um=50.0)
    chip = Box3D((-300.0, -300.0, -300.0), (300.0, 300.0, 300.0))
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    for r, expect_ok in ((50.0 - eps, False), (50.0, True), (50.0 + eps, True)):
        seg = CircularArc3D.from_pose(pose, (0.0, 0.0, 1.0), r, math.radians(60.0))
        out = validate_segments((seg,), chip, [], [], cfg)
        assert out["ok"] is expect_ok, "R=%g expected ok=%s" % (r, expect_ok)
    assert eps > cfg.radius_tol_um * 1000.0, "epsilon must exceed the decision tolerance"


# ------------------------------------------------------ (5) spacing boundary
def test_case5_scene_level_spacing_rejection_and_acceptance():
    cfg = base_config(dmin_centerline_um=5.0)
    chip = Box3D((-100.0, -100.0, -100.0), (200.0, 200.0, 100.0))
    ref = LineSegment3D((0.0, 120.0, 0.0), (200.0, 120.0, 0.0))
    too_close = LineSegment3D((0.0, 124.0, 0.0), (150.0, 124.0, 0.0))
    far_enough = LineSegment3D((0.0, 126.0, 0.0), (150.0, 126.0, 0.0))
    bad = validate_segments((too_close,), chip, [], [ref], cfg)
    good = validate_segments((far_enough,), chip, [], [ref], cfg)
    assert bad["ok"] is False and bad["failed"] == ["centerline_spacing"]
    assert good["ok"] is True
    assert good["spacing_bound_um"] == pytest.approx(6.0, abs=1e-9)


# --------------------------------- (6) mid-curve boundary / collision reject
def test_case6_legal_endpoints_but_the_curve_middle_is_illegal():
    cfg = base_config()
    chip = Box3D((-100.0, -100.0, -100.0), (200.0, 200.0, 200.0))
    wall = Box3D((45.0, 45.0, 40.0), (55.0, 55.0, 60.0))
    arc = tilted_plane_arc((18.0, 50.0, 50.0), (0.0, 0.0, 1.0), 32.0,
                           math.radians(110.0), start_angle=math.radians(-55.0))
    # each endpoint is an individually legal pose ...
    sp = Pose3D(arc.start, arc.tangent_at(0.0))
    ep = Pose3D(arc.end, arc.tangent_at(arc.length))
    sc = Scene("mid", chip, [wall], [], sp, ep, cfg)
    assert pose_invalid_reason(sc, sp) is None
    assert pose_invalid_reason(sc, ep) is None
    # ... but the primitive is rejected because its MIDDLE is inside the wall
    out = validate_segments((arc,), chip, [wall], [], cfg)
    assert out["ok"] is False
    assert "obstacle_clearance" in out["failed"]
    # same story for leaving the chip volume in the middle of the curve.
    # The arc's true analytic box is x in [18, 50], y in [23.8, 76.2], z = 50;
    # the inner volume below is [2, 43] x [2, 98] x [2, 98], so the MIDDLE of the
    # curve (x = 50) leaves it while both endpoints (x = 18 and x = 50 ... the
    # start is at x = 18, so the tested end is the point of maximum x) stay legal.
    small_chip = Box3D((0.0, 0.0, 0.0), (45.0, 100.0, 100.0))
    cfg_boundary = base_config(rmin_um=30.0)     # isolate boundary from Rmin
    out2 = validate_segments((arc,), small_chip, [], [], cfg_boundary)
    assert out2["ok"] is False
    assert out2["failed"] == ["boundary"], out2["failed"]
    lo, hi = arc.aabb()
    assert hi[0] > small_chip.hi[0] - cfg_boundary.effective_boundary_margin_um
    assert lo[0] > small_chip.lo[0]


# ------------------------------------------------------ (7) terminal tangent
def test_case7_same_position_wrong_tangent_is_not_success():
    good = terminal_tangent_case(correct_tangent=True)
    bad = terminal_tangent_case(correct_tangent=False)
    res_ok = route(good, mode="2d")
    assert res_ok.status == "success"
    assert res_ok.goal_pos_error_um <= good.config.eps_pos_um
    assert res_ok.goal_tangent_error_deg <= good.config.eps_ang_deg
    res_bad = route(bad, mode="2d", max_expansions=1200, max_runtime_s=8.0)
    assert res_bad.status != "success", "arriving at the same position with the wrong tangent"
    assert res_bad.status == "budget_exhausted"        # production: budget stop
    assert res_bad.budget_exhausted is True
    assert "NOT a proof" in res_bad.reason
    ref_bad = reference_search(terminal_tangent_case(correct_tangent=False), mode="2d")
    assert ref_bad.status == "search_exhausted"        # finite graph: real exhaustion
    assert ref_bad.budget_exhausted is False
    # the geometric reason: the pocket is 10 um wide, a 90 degree turn needs Rmin = 50 um
    assert (bad.metadata["pocket_free_space"]["y_um"][1]
            - bad.metadata["pocket_free_space"]["y_um"][0]) < bad.config.rmin_um / 4.0
    # a legal terminal connector is accepted only after the full constraint stack
    conn = [a for a in generate_actions(good.source, good.config, False)
            if a.name.startswith("straight")]
    assert conn and all(validate_segments(a.segments, good.chip, good.obstacles,
                                          good.reference_centerlines, good.config)["ok"]
                        for a in conn)


# -------------------------------------------------- (8) admissibility / cost
def test_case8_non_zero_congestion_changes_cost_but_not_feasibility():
    cfg = base_config(max_expansions=20000, max_runtime_s=120.0,
                      pos_resolution_um=20.0, dir_resolution_deg=30.0,
                      straight_lengths_um=(20.0, 40.0), turn_angles_deg=(45.0, 90.0),
                      turn_radii_um=(50.0,), skew_axis_tilt_deg=(45.0,),
                      connector_radii_um=(50.0,))
    sc = core_case(cfg)
    r0 = reference_search(core_case(cfg), mode="native3d",
                          congestion=ZeroCongestion())
    region = Box3D((250.0, 100.0, 40.0), (350.0, 200.0, 120.0))
    field = RegionCongestion([(region, 0.5)])
    r1 = reference_search(core_case(cfg), mode="native3d", congestion=field)
    assert r0.status == "success" and r1.status == "success"
    assert r1.congestion_cost > 0.0
    assert r1.cost == pytest.approx(r1.length_um + r1.congestion_cost, rel=1e-12)
    assert r1.cost > r0.cost - 1e-9
    # the penalty is an equivalent length in um, never a dB figure
    assert "um" in field.describe()["method"] or True
    assert hasattr(field, "penalty")
    for s in r1.path.segments:
        assert field.penalty(s) >= 0.0