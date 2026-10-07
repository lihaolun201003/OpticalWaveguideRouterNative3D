"""Phase 1 tests: Pose3D, LineSegment3D, CircularArc3D (arbitrary plane), Path3D."""

import math

import pytest

from native3d.geometry import CircularArc3D, LineSegment3D, Path3D, Pose3D
from native3d.geometry.vec import add, cross, dot, norm, rotate_about, scale, sub


def test_pose_normalises_and_stores_true_3d_tangent():
    p = Pose3D((1.0, 2.0, 3.0), (0.0, 0.0, 5.0))
    assert p.tangent == pytest.approx((0.0, 0.0, 1.0))
    p2 = Pose3D((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    assert norm(p2.tangent) == pytest.approx(1.0)
    assert p2.elevation_rad == pytest.approx(math.asin(1.0 / math.sqrt(3.0)))


@pytest.mark.parametrize("bad", [(0.0, 0.0, 0.0), (1e-18, 0.0, 0.0)])
def test_pose_rejects_zero_tangent(bad):
    with pytest.raises(ValueError):
        Pose3D((0.0, 0.0, 0.0), bad)


def test_pose_rejects_non_finite():
    with pytest.raises(ValueError):
        Pose3D((0.0, 0.0, float("nan")), (1.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        Pose3D((0.0, 0.0, 0.0), (float("inf"), 0.0, 0.0))


def test_line_segment_length_is_euclidean_and_degenerate_rejected():
    seg = LineSegment3D((0.0, 0.0, 0.0), (3.0, 4.0, 12.0))
    assert seg.length == pytest.approx(13.0)
    assert seg.point_at(6.5) == pytest.approx((1.5, 2.0, 6.0))
    assert seg.tangent_at(0.0) == pytest.approx(seg.tangent_at(seg.length))
    with pytest.raises(ValueError):
        LineSegment3D((1.0, 1.0, 1.0), (1.0, 1.0, 1.0))


def test_arc_in_arbitrary_tilted_plane_geometry():
    """An arc in a TILTED plane (normal (1,1,1)/sqrt(3)), not an XY arc."""
    s2 = 1.0 / math.sqrt(2.0)
    center = (10.0, -4.0, 7.0)
    R = 37.5
    theta = math.radians(115.0)
    # tilted plane with the orthonormal in-plane basis (u, v), normal u x v
    u, v = (1.0, 0.0, 0.0), (0.0, s2, -s2)
    arc = CircularArc3D(center, u, v, R, theta)
    nn = arc.normal
    assert norm(nn) == pytest.approx(1.0)
    assert abs(dot(nn, (1.0, 0.0, 0.0))) < 0.9, "normal must not be axis aligned"
    for i in range(41):
        phi = theta * i / 40.0
        p = arc.point_at_phi(phi)
        assert norm(sub(p, arc.center)) == pytest.approx(R, abs=1e-9)
        assert dot(sub(p, arc.center), nn) == pytest.approx(0.0, abs=1e-9)
        # tangent is perpendicular to the radius and lies in the plane
        assert abs(dot(arc.tangent_at_phi(phi), sub(p, arc.center))) < 1e-9
        assert abs(dot(arc.tangent_at_phi(phi), nn)) < 1e-9
    assert arc.length == pytest.approx(R * abs(theta), abs=1e-9)
    # z really varies: this is not a plane of constant height
    zs = [arc.point_at_phi(theta * i / 40.0)[2] for i in range(41)]
    assert max(zs) - min(zs) > 1.0
    # tangent at the end matches the analytic rotation of the start tangent
    t0 = arc.start_pose.tangent
    expect_end = rotate_about(t0, nn, theta)
    assert norm(sub(arc.end_pose.tangent, expect_end)) < 1e-9


def test_arc_from_pose_sign_conventions_about_the_normal():
    p = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    n = (0.0, 0.0, 1.0)
    R = 20.0
    a = CircularArc3D.from_pose(p, n, R, math.radians(90.0))
    assert a.start_pose.position == pytest.approx(p.position)
    assert a.start_pose.tangent == pytest.approx(p.tangent)
    # positive sweep turns the tangent towards n x t = +y
    assert a.end_pose.tangent == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert a.end_pose.position == pytest.approx((R, R, 0.0), abs=1e-12)
    b = CircularArc3D.from_pose(p, n, R, -math.radians(90.0))
    assert b.start_pose.tangent == pytest.approx(p.tangent)
    assert b.end_pose.tangent == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)
    assert b.end_pose.position == pytest.approx((R, -R, 0.0), abs=1e-12)
    assert b.length == pytest.approx(a.length)


def test_arc_rejects_illegal_inputs():
    with pytest.raises(ValueError):
        CircularArc3D((0, 0, 0), (1, 0, 0), (0, 1, 0), 0.0, 1.0)      # R = 0
    with pytest.raises(ValueError):
        CircularArc3D((0, 0, 0), (1, 0, 0), (0, 1, 0), -5.0, 1.0)     # R < 0
    with pytest.raises(ValueError):
        CircularArc3D((0, 0, 0), (1, 0, 0), (0, 1, 0), float("inf"), 1.0)
    with pytest.raises(ValueError):
        CircularArc3D((0, 0, 0), (1, 0, 0), (0, 1, 0), 10.0, 0.0)     # sweep = 0
    with pytest.raises(ValueError):
        CircularArc3D((0, 0, 0), (2, 0, 0), (0, 1, 0), 10.0, 1.0)     # not unit
    with pytest.raises(ValueError):
        CircularArc3D((0, 0, 0), (1, 0, 0), (1, 0, 0), 10.0, 1.0)     # not orthogonal
    # normal not perpendicular to the tangent
    with pytest.raises(ValueError):
        CircularArc3D.from_pose(Pose3D((0, 0, 0), (1, 0, 0)), (1, 0, 0), 10.0, 1.0)


def test_path_cumulative_length_and_c1_report():
    p0 = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    arc = CircularArc3D.from_pose(p0, (0.0, 1.0, 0.0), 30.0, math.radians(60.0))
    line = LineSegment3D(arc.end, add(arc.end, scale(arc.end_pose.tangent, 40.0)))
    path = Path3D((arc, line))
    assert path.total_length == pytest.approx(arc.length + 40.0)
    assert path.cumulative_lengths[1] == pytest.approx(arc.length)
    rep = path.continuity_report()
    assert rep["c1_continuous"] is True
    assert rep["max_position_error_um"] < 1e-9
    assert rep["max_tangent_error_deg"] < 1e-9
    # line -> arc junction has a curvature jump but is NOT a non-C1 corner
    assert len(rep["curvature_jumps"]) == 1
    assert rep["corners"] == []
    assert rep["min_radius_um"] == pytest.approx(30.0)


def test_path_reports_non_c1_corner_separately_from_curvature_jump():
    a = LineSegment3D((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    b = LineSegment3D((10.0, 0.0, 0.0), (10.0, 10.0, 0.0))   # 90 degree corner
    path = Path3D((a, b))
    rep = path.continuity_report()
    assert rep["c1_continuous"] is False
    assert any(c["type"] == "tangent_discontinuity" for c in rep["corners"])
    assert rep["max_tangent_error_deg"] == pytest.approx(90.0, abs=1e-9)
    assert rep["curvature_jumps"] == []          # both radii are infinite


def test_pure_straight_path_length_equals_euclidean_distance():
    """Phase 6 case 1: known 3D endpoints, length is the Euclidean distance."""
    p0, p1 = (1.0, 2.0, 3.0), (5.0, 5.0, 15.0)
    path = Path3D((LineSegment3D(p0, p1),))
    d = math.sqrt(sum((p1[i] - p0[i]) ** 2 for i in range(3)))
    assert path.total_length == pytest.approx(d)
    assert path.start_pose.tangent == pytest.approx(path.end_pose.tangent)
    assert path.end_pose.position == pytest.approx(p1)
