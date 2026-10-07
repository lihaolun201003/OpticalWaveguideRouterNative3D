"""v0.1.2 generality matrix for the FINITE PRODUCTION router.

Items 1, 2, 6, 7 and 8 of the required generality list, all driven through the
public entry point route_finite_native3d(scene, ...) with no waypoint, no target
height and no route ever supplied.  Items 3, 4 and 5 (cross-wall, several
interior nodes, multiple walls) are in tests/test_v012_finite_graph.py, and item
9 (every confirmed PART A counterexample) is in tests/test_v012_a*.py.
"""

import math

import pytest

from native3d.cases import base_config
from native3d.constraints.checks import Box3D, validate_segments
from native3d.geometry import Pose3D
from native3d.io.caseio import Scene
from native3d.search.finite_graph import FiniteGraph, route_finite_native3d


def _run(scene, mode="native3d"):
    return route_finite_native3d(scene, mode=mode)


# ------------------------------------------------------- (1) pure straight
def test_case1_pure_straight_is_exactly_euclidean():
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((-100.0, -100.0, -100.0), (400.0, 300.0, 200.0))
    sc = Scene("straight", chip, [], [],
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((150.0, 0.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = _run(sc)
    assert res.status == "success", (res.status, res.reason)
    assert res.length_um == pytest.approx(150.0, abs=1e-9)
    assert all(s.kind == "line" for s in res.path.segments)
    assert res.path.start_pose.position == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert res.path.end_pose.position == pytest.approx((150.0, 0.0, 0.0), abs=1e-12)


# --------------------------------------------------- (2) true 3D bending
def test_case2_a_genuinely_three_dimensional_target_is_reached():
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((-100.0, -100.0, -100.0), (400.0, 300.0, 200.0))
    sc = Scene("bend", chip, [], [],
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((200.0, 100.0, 50.0), (1.0, 0.0, 0.0)), cfg)
    res = _run(sc)
    assert res.status == "success", (res.status, res.reason)
    zs = [s["z_um"] for s in res.trace]
    ys = [s["y_um"] for s in res.trace]
    assert max(zs) >= 50.0 and max(ys) >= 100.0
    assert any(s.kind == "arc" for s in res.path.segments)
    # and the arcs really are genuine space curves: at least one of them is out
    # of the xy plane (its z extent is not zero)
    assert any(abs(s.aabb()[1][2] - s.aabb()[0][2]) > 1e-9
               for s in res.path.segments if s.kind == "arc")
    out = validate_segments(res.path.segments, sc.chip, sc.obstacles,
                            sc.reference_centerlines, sc.config)
    assert out["ok"] is True, out.get("failed")


# --------------------------------------- (6) sealed cavity really exhausts
def test_case6_a_sealed_cavity_is_a_real_exhaustion_or_an_explicit_port_failure():
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((0.0, 0.0, 0.0), (400.0, 400.0, 300.0))
    walls = [Box3D((180.0, 180.0, -10.0), (200.0, 320.0, 300.0)),
             Box3D((300.0, 180.0, -10.0), (320.0, 320.0, 300.0)),
             Box3D((180.0, 180.0, -10.0), (320.0, 200.0, 300.0)),
             Box3D((180.0, 300.0, -10.0), (320.0, 320.0, 300.0)),
             Box3D((180.0, 180.0, 240.0), (320.0, 320.0, 300.0))]
    sc = Scene("sealed", chip, walls, [],
               Pose3D((40.0, 40.0, 40.0), (1.0, 0.0, 0.0)),
               Pose3D((250.0, 250.0, 40.0), (1.0, 0.0, 0.0)), cfg)
    res = _run(sc)
    assert res.status in ("search_exhausted", "port_attachment_failed"), \
        (res.status, res.reason)
    if res.status == "search_exhausted":
        assert res.budget_exhausted is False
        assert res.path is None
        assert res.expansions < res.max_expansions


# ------------------------------------------------- (7) deterministic variants
def _core(shift_x=0.0, wall_top=40.0, source=(60.0, 150.0, 0.0),
          target=(540.0, 150.0, 0.0)):
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
    wall = Box3D((290.0 + shift_x, -50.0, -70.0), (310.0 + shift_x, 350.0, wall_top))
    return Scene("variant", chip, [wall], [],
                 Pose3D(source, (1.0, 0.0, 0.0)),
                 Pose3D(target, (1.0, 0.0, 0.0)), cfg)


@pytest.mark.parametrize("kw", [
    dict(),
    dict(shift_x=40.0),
    dict(shift_x=-60.0),
    dict(wall_top=20.0),
    dict(source=(60.0, 100.0, 0.0), target=(540.0, 200.0, 0.0)),
    dict(source=(35.0, 150.0, 0.0), target=(540.0, 150.0, 0.0)),
    dict(source=(60.0, 150.0, -20.0), target=(540.0, 150.0, 20.0)),
])
def test_case7_deterministic_variants_of_the_wall_case(kw):
    sc = _core(**kw)
    res = _run(sc)
    assert res.status == "success", (kw, res.status, res.reason)
    assert max(s["z_um"] for s in res.trace) > 45.0 - 60.0  # climbs, see below
    wall = sc.obstacles[0]
    req = sc.config.effective_obstacle_clearance_um
    L = res.path.total_length
    for i in range(4001):
        p = res.path.point_at(L * i / 4000.0)
        if wall.lo[0] - req <= p[0] <= wall.hi[0] + req:
            assert p[2] > wall.hi[2], (kw, p)
    out = validate_segments(res.path.segments, sc.chip, sc.obstacles,
                            sc.reference_centerlines, sc.config)
    assert out["ok"] is True, out.get("failed")
    # determinism: the same scene gives the same cost and the same trace
    again = _run(_core(**kw))
    assert again.cost == res.cost
    assert [s["action"] for s in again.trace] == [s["action"] for s in res.trace]


# ------------------------------------------- (8) port / constraint boundaries
def test_case8_a_wrong_target_tangent_never_reports_success():
    cfg = base_config(finite_spacing_um=50.0, finite_port_attach_um=10.0)
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
    wall = Box3D((290.0, -50.0, -70.0), (310.0, 350.0, 40.0))
    sc = Scene("wrong_terminal_tangent", chip, [wall], [],
               Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((540.0, 150.0, 0.0), (0.0, 1.0, 0.0)), cfg)
    res = _run(sc)
    assert res.status == "port_attachment_failed", (res.status, res.reason)
    assert res.path is None
    assert "never about physical reachability" in res.reason


def test_case8_a_wrong_source_tangent_is_reported_too():
    cfg = base_config(finite_spacing_um=50.0, finite_port_attach_um=10.0)
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
    sc = Scene("wrong_source_tangent", chip, [], [],
               Pose3D((60.0, 150.0, 0.0), (0.0, 1.0, 0.0)),
               Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = _run(sc)
    assert res.status in ("port_attachment_failed", "unsupported_pose"), res.status
    assert res.path is None


def test_case8_radius_exactly_rmin_is_accepted_and_below_is_refused():
    g = FiniteGraph(_core())
    assert g.turn_radii and all(R >= g.cfg.rmin_um - g.cfg.radius_tol_um
                                for _m, R in g.turn_radii)
    assert any(abs(R - g.cfg.rmin_um) < 1e-9 for _m, R in g.turn_radii)
    # a spacing whose integer multiples can never reach Rmin must be refused
    bad = base_config(finite_spacing_um=10.0, finite_turn_radius_steps=(1, 2, 3))
    with pytest.raises(ValueError):
        FiniteGraph(_core_with_config(bad))


def _core_with_config(cfg):
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
    wall = Box3D((290.0, -50.0, -70.0), (310.0, 350.0, 40.0))
    return Scene("cfg", chip, [wall], [],
                 Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
                 Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)), cfg)


def test_case8_an_illegal_reference_spacing_is_never_silently_accepted():
    """A path that would violate dmin must not be produced by the router."""
    cfg = base_config(finite_spacing_um=50.0)
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
    from native3d.geometry import LineSegment3D
    # the reference centreline sits exactly on the straight run y = 150, z = 0
    ref = LineSegment3D((100.0, 150.0, 0.0), (500.0, 150.0, 0.0))
    sc = Scene("dmin", chip, [], [ref],
               Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)), cfg)
    res = _run(sc)
    if res.status == "success":
        out = validate_segments(res.path.segments, sc.chip, sc.obstacles,
                                sc.reference_centerlines, sc.config)
        assert out["ok"] is True, out.get("failed")
    else:
        assert res.status in ("search_exhausted", "port_attachment_failed")
