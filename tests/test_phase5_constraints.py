"""Phase 5 tests: hard constraints on the full continuous primitive."""

import math

import pytest

from native3d.cases import base_config, tilted_plane_arc
from native3d.config import RouterConfig
from native3d.constraints.checks import (Box3D, arc_box_clearance, boundary_check,
                                         line_box_clearance, pair_clearance_lb_ub,
                                         path_self_spacing, point_box_distance,
                                         primitive_spacing, radius_check,
                                         segment_box_distance,
                                         segment_box_intersection,
                                         segment_segment_distance)
from native3d.geometry import CircularArc3D, LineSegment3D, Path3D, Pose3D
from native3d.geometry.vec import add, norm, scale, sub

EPS_RADIUS = 1.0     # test epsilon, far larger than the 1e-6 decision tolerance


def arc_from(pose, normal, R, sweep_deg):
    return CircularArc3D.from_pose(pose, normal, R, math.radians(sweep_deg))


# ------------------------------------------------------------------- radius
def test_radius_boundary_just_below_at_and_above_rmin():
    cfg = RouterConfig(rmin_um=50.0, radius_tol_um=1e-6)
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    r_minus = radius_check((arc_from(pose, (0, 0, 1), 50.0 - EPS_RADIUS, 90.0),), cfg)
    r_at = radius_check((arc_from(pose, (0, 0, 1), 50.0, 90.0),), cfg)
    r_plus = radius_check((arc_from(pose, (0, 0, 1), 50.0 + EPS_RADIUS, 90.0),), cfg)
    assert r_minus.status == "violation" and r_minus.bound_um == pytest.approx(49.0)
    assert r_at.status == "safe"
    assert r_plus.status == "safe"
    # straight segments have infinite radius and can never violate Rmin
    straight = LineSegment3D((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    assert radius_check((straight,), cfg).status == "safe"
    assert radius_check((straight,), cfg).bound_um == math.inf
    # documented in-tolerance behaviour: Rmin - 1e-9 is accepted
    r_tol = radius_check((arc_from(pose, (0, 0, 1), 50.0 - 1e-9, 90.0),), cfg)
    assert r_tol.status == "safe"


# ----------------------------------------------------------------- boundary
def test_boundary_checks_the_whole_primitive_not_only_endpoints():
    cfg = base_config()
    chip = Box3D((0.0, 0.0, 0.0), (100.0, 100.0, 100.0))
    margin = cfg.effective_boundary_margin_um
    good = LineSegment3D((10.0, 10.0, 10.0), (90.0, 90.0, 90.0))
    assert boundary_check(good, chip, margin).status == "safe"
    out = LineSegment3D((50.0, 50.0, 50.0), (110.0, 50.0, 50.0))
    assert boundary_check(out, chip, margin).status == "violation"
    # arc: legal endpoints, but the MIDDLE of the curve leaves the volume
    arc = tilted_plane_arc((50.0, 50.0, 50.0), (0.0, 0.0, 1.0), 54.0,
                           math.radians(120.0), start_angle=math.radians(120.0))
    assert chip.contains_point(arc.start) and chip.contains_point(arc.end)
    res = boundary_check(arc, chip, margin)
    assert res.status == "violation"
    assert res.detail["overshoot_um"] > 1.0
    # the sampling-free bound really is the analytic extremum
    lo, hi = arc.aabb()
    assert lo[0] == pytest.approx(50.0 - 54.0, abs=1e-9)


# ---------------------------------------------------------------- collision
def test_collision_detected_in_the_middle_of_a_curve_with_legal_endpoints():
    cfg = base_config()
    wall = Box3D((45.0, 45.0, 40.0), (55.0, 55.0, 60.0))
    arc = tilted_plane_arc((18.0, 50.0, 50.0), (0.0, 0.0, 1.0), 32.0,
                           math.radians(110.0), start_angle=math.radians(-55.0))
    assert not wall.expand(cfg.effective_obstacle_clearance_um).contains_point(arc.start)
    assert not wall.expand(cfg.effective_obstacle_clearance_um).contains_point(arc.end)
    mid = arc.point_at(arc.length / 2.0)
    assert wall.contains_point(mid), "test construction: the middle is inside the wall"
    res = arc_box_clearance(arc, wall, cfg.effective_obstacle_clearance_um, cfg)
    assert res.status == "violation"
    # a legal line far from the wall is safe and reports a certified bound
    line = LineSegment3D((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    box = Box3D((40.0, 20.0, -5.0), (60.0, 30.0, 5.0))
    ok = line_box_clearance(line, box, 5.0)
    assert ok.status == "safe"
    assert ok.bound_um == pytest.approx(20.0, abs=1e-9)


def test_aabb_overlap_alone_is_not_a_collision():
    """Bounding boxes may overlap while the primitive stays far from the solid."""
    cfg = base_config()
    box = Box3D((10.0, 10.0, 10.0), (20.0, 20.0, 20.0))
    # diagonal segment passing near the corner (30, 0, 0): AABB overlaps in x/y
    seg = LineSegment3D((-30.0, 30.0, 0.0), (30.0, -30.0, 0.0))
    lo, hi = seg.aabb()
    assert hi[0] > box.lo[0] and lo[1] < box.hi[1]
    res = line_box_clearance(seg, box, 1.0)
    assert res.status == "safe"
    assert res.bound_um > 1.0


def test_exact_segment_box_distance_matches_a_dense_sampling_oracle():
    box = Box3D((10.0, 10.0, 10.0), (20.0, 20.0, 20.0))
    seg = LineSegment3D((0.0, 0.0, 0.0), (25.0, 30.0, 5.0))
    exact = segment_box_distance(seg.p0, seg.p1, box)
    n = 20000

    def oracle(p):
        return math.sqrt(sum(max(box.lo[i] - p[i], p[i] - box.hi[i], 0.0) ** 2
                             for i in range(3)))

    sampled = min(oracle(seg.point_at(seg.length * k / n)) for k in range(n + 1))
    assert exact <= sampled + 1e-9
    assert exact == pytest.approx(sampled, abs=0.02)
    # an intersecting segment has distance exactly zero
    hit = LineSegment3D((0.0, 15.0, 15.0), (30.0, 15.0, 15.0))
    assert segment_box_intersection(hit.p0, hit.p1, box) is not None
    assert segment_box_distance(hit.p0, hit.p1, box) == 0.0


# ----------------------------------------------------------------- spacing
def test_spacing_boundary_just_above_and_below_dmin():
    cfg = base_config(dmin_centerline_um=5.0)
    a = LineSegment3D((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    above = LineSegment3D((0.0, 5.0 + 0.01, 0.0), (100.0, 5.01, 0.0))
    below = LineSegment3D((0.0, 5.0 - 0.01, 0.0), (100.0, 4.99, 0.0))
    ok = primitive_spacing(a, [above], cfg.dmin_centerline_um, cfg)
    bad = primitive_spacing(a, [below], cfg.dmin_centerline_um, cfg)
    # v0.1.2: a certified bound carries the documented floating point error
    # floor (SEGMENT_DISTANCE_ERROR_FLOOR_UM = 1e-9 um), so a certified lower
    # bound is at most that far below the exact distance and a certified upper
    # bound at most that far above it.  Both directions are still decided.
    assert ok.status == "safe"
    assert ok.bound_um == pytest.approx(5.01, abs=2e-9)
    assert ok.bound_um <= 5.01
    assert bad.status == "violation"
    assert bad.bound_um == pytest.approx(4.99, abs=2e-9)
    assert bad.bound_um >= 4.99


def test_spacing_closest_points_between_sampling_nodes_are_found():
    """The closest pair is at interior parameters, not at any sample node."""
    cfg = base_config(dmin_centerline_um=30.05)
    a = LineSegment3D((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    b = LineSegment3D((53.0, 30.0, -20.0), (53.0, 30.0, 20.0))
    lb, ub = pair_clearance_lb_ub(a, b, 0)
    # the certified interval brackets 30.0 within the documented error floor
    assert lb == pytest.approx(30.0, abs=2e-9)
    assert ub == pytest.approx(30.0, abs=2e-9)
    assert lb <= 30.0 <= ub
    # naive node sampling every 10 um on a would report the wrong (larger) value
    nodes = [a.point_at(a.length * k / 10.0) for k in range(11)]
    node_min = min(norm(sub(p, b.point_at(b.length / 2.0))) for p in nodes)
    assert node_min > 30.1, "a node-only comparison misses the true closest approach"
    res_ok = primitive_spacing(a, [b], 29.95, cfg)
    res_bad = primitive_spacing(a, [b], 30.05, cfg)
    assert res_ok.status == "safe"
    assert res_bad.status == "violation"


def test_arc_to_line_spacing_is_certified_over_all_parameter_pairs():
    cfg = base_config(dmin_centerline_um=5.0)
    arc = arc_from(Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), (0, 0, 1), 50.0, 90.0)
    ref = LineSegment3D((0.0, 80.0, 0.0), (200.0, 80.0, 0.0))
    res = primitive_spacing(arc, [ref], cfg.dmin_centerline_um, cfg)
    assert res.status == "safe"
    # certified bound must not exceed the true distance (50 top of the arc to y=80)
    assert res.bound_um <= 30.0 + 1e-6
    assert res.bound_um > 5.0


def test_adjacent_segments_are_exempt_but_wrap_around_re_approach_is_flagged():
    cfg = base_config(dmin_centerline_um=5.0)
    s0 = LineSegment3D((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    s1 = LineSegment3D((100.0, 0.0, 0.0), (100.0, 3.0, 0.0))
    s2 = LineSegment3D((100.0, 3.0, 0.0), (0.0, 3.0, 0.0))
    res = path_self_spacing(Path3D((s0, s1, s2)), cfg)
    assert res.status == "violation", "non-adjacent self re-approach must be caught"
    # the shared-endpoint junctions themselves are not reported as violations
    assert any(d.get("pair") == [0, 2] for d in [res.detail.get("worst", {})])
    far = Path3D((s0, LineSegment3D((100.0, 0.0, 0.0), (100.0, 9.0, 0.0)),
                  LineSegment3D((100.0, 9.0, 0.0), (0.0, 9.0, 0.0))))
    assert path_self_spacing(far, cfg).status == "safe"


def test_shallow_subdivision_depth_reports_undecided_and_rejects():
    """A conservative check that cannot decide must reject, never accept.

    v0.1.2 note: the fixture was re-designed.  The v0.1.1 fixture put the box so
    that a CHORD NODE of the quarter arc was 3.5 um away while the requirement
    was 4.0 um; since v0.1.2 treats a point of the curve inside the keep-out as a
    reliable violation witness, that fixture is now decided immediately as
    "violation" (correctly - the true clearance 3.5 < 4.0).  To keep testing the
    UNDECIDED path the box is now placed between two chord nodes: the true
    clearance is exactly 3.0 um, every node of the level-0/1 subdivision is
    10.18 um away, and only a finer subdivision finds the witness.
    """
    deep = base_config(obstacle_clearance_um=4.0, max_subdiv_level=9)
    shallow = base_config(obstacle_clearance_um=4.0, max_subdiv_level=1)
    arc = arc_from(Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), (0, 1, 0), 50.0, 90.0)
    assert arc.point_at(arc.length) == pytest.approx((50.0, 0.0, -50.0), abs=1e-9)
    # the box sits 3.0 um above the arc point at 33.75 deg of the quarter turn
    mid = arc.point_at(arc.length * (33.75 / 90.0))
    box = Box3D((mid[0] - 0.05, 3.0, mid[2] - 0.05),
                (mid[0] + 0.05, 3.1, mid[2] + 0.05))
    dense = min(point_box_distance(arc.point_at(arc.length * i / 20000.0), box)
                for i in range(20001))
    assert dense == pytest.approx(3.0, abs=1e-6)
    for n in (2, 4):
        nodes = [arc.point_at(arc.length * i / n) for i in range(n + 1)]
        assert min(point_box_distance(p, box) for p in nodes) > 10.0
    inner = arc.point_at(arc.length * (2.0 / 3.0))
    assert point_box_distance(inner, box) > 20.0
    assert segment_box_distance(inner, inner, box) > 20.0
    d = arc_box_clearance(arc, box, 4.0, deep)
    s = arc_box_clearance(arc, box, 4.0, shallow)
    assert d.status == "violation", "the exact clearance 3.0 < 4.0 must be decided"
    assert d.detail["method"] == "violation_witness_on_the_curve"
    assert s.status == "undecided"
    assert "reject" in s.detail["note"]
    # an undecided check must not be accepted by the aggregate validator
    from native3d.constraints.checks import validate_segments
    chip = Box3D((-200.0, -200.0, -200.0), (200.0, 200.0, 200.0))
    out = validate_segments((arc,), chip, [box], [], shallow)
    assert out["ok"] is False
    assert out["failed"] == ["obstacle_clearance"]
    assert validate_segments((arc,), chip, [box], [], deep)["ok"] is False
