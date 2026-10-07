"""Repair D regression: LSL/RSR second-arc sweep sign.

Re-derived from the coordinate / sweep / tangent conventions of
CircularArc3D.from_pose (a positive sweep rotates the tangent towards
normal x tangent, right-hand rule):

    left  arc centre  = p + R * left(t),   left(t)  = (-t_y, t_x) in the plane
    right arc centre  = p + R * right(t),  right(t) = ( t_y, -t_x)
    first  arc sweep  = mod2pi(+/- u_ang)                         (unchanged)
    LSL second arc    = +mod2pi(theta_e - u_ang)     (v0.1: mod2pi(u_ang - theta_e))
    RSR second arc    = -mod2pi(u_ang - theta_e)     (v0.1: mod2pi(theta_e - u_ang))
    LSR / RSL are correct in v0.1 and must NOT be flipped.

Counterexample (R = 10 um):
    source (0,0,0) tangent +x ; target (100,100,0) tangent +y
    LSL: arc +pi/4, straight sqrt(90^2+90^2), arc +pi/4, total 142.987184 um

The v0.1 implementation generated only LSR/RSL (about 204.374715 um) and dropped
the legal LSL in its endpoint check.
"""

import math

import pytest

from native3d.config import RouterConfig
from native3d.geometry import Pose3D
from native3d.geometry.vec import (add, cross, dot, norm, rotate_about, scale, sub)
from native3d.search.connector import connector_actions, planar_connectors

R = 10.0
EXPECTED_SAME_TURN = math.sqrt(90.0 ** 2 + 90.0 ** 2) + 2.0 * R * math.pi / 4.0
EXPECTED_MIXED_TURN = (R * (math.pi / 2.0 + math.asin(0.2))
                       + math.sqrt(100.0 ** 2 - 4.0 * R * R)
                       + R * (2.0 * math.pi - (math.pi - (math.pi / 2.0 + math.asin(0.2)))))
Z_HAT = (0.0, 0.0, 1.0)


def connect(pose_a, pose_b, radius=R, tol=1e-6):
    return planar_connectors(pose_a, pose_b, radius, coplanar_tol_um=tol)


def turn_signs(segs):
    """Signed sweeps about +z for each arc of a connector in the xy plane."""
    out = []
    for s in segs:
        if s.kind == "arc":
            sgn = 1.0 if dot(s.normal, Z_HAT) > 0 else -1.0
            out.append(sgn * s.theta)
    return out


def check_connector(segs, pose_a, pose_b, radius=R):
    assert segs, "connector must have segments"
    assert segs[0].start_pose.position == pytest.approx(pose_a.position, abs=1e-12)
    assert segs[0].start_pose.tangent == pytest.approx(pose_a.tangent, abs=1e-12)
    assert segs[-1].end_pose.position == pytest.approx(pose_b.position, abs=1e-9)
    assert segs[-1].end_pose.tangent == pytest.approx(pose_b.tangent, abs=1e-9)
    from native3d.geometry import Path3D
    rep = Path3D(tuple(segs)).continuity_report()
    assert rep["c1_continuous"] is True, rep["corners"]
    for seg in segs:
        if seg.kind == "arc":
            assert seg.radius == pytest.approx(radius, abs=1e-12)


def by_total(conns, total, tol=1e-6):
    return [c for c in conns if abs(sum(s.length for s in c) - total) <= tol]


# --------------------------------------------------------------- LSL / RSR
def test_lsl_exists_for_parallel_but_rotated_tangents():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, 100.0, 0.0), (0.0, 1.0, 0.0))
    conns = connect(a, b)
    hits = by_total(conns, EXPECTED_SAME_TURN)
    assert hits, ("no LSL connector; totals = %r" %
                  sorted(round(sum(s.length for s in c), 6) for c in conns))
    for segs in hits:
        check_connector(segs, a, b)
        signs = turn_signs(segs)
        assert signs == pytest.approx([math.pi / 4.0, math.pi / 4.0], abs=1e-9), signs
        assert [s.kind for s in segs] == ["arc", "line", "arc"]
        assert segs[1].length == pytest.approx(math.sqrt(90.0 ** 2 + 90.0 ** 2), abs=1e-9)
        assert sum(s.length for s in segs) == pytest.approx(EXPECTED_SAME_TURN, abs=1e-9)
    assert EXPECTED_SAME_TURN == pytest.approx(142.987184, abs=1e-6)


def test_rsr_exists_for_the_mirrored_configuration():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, -100.0, 0.0), (0.0, -1.0, 0.0))
    conns = connect(a, b)
    hits = by_total(conns, EXPECTED_SAME_TURN)
    assert hits, ("no RSR connector; totals = %r" %
                  sorted(round(sum(s.length for s in c), 6) for c in conns))
    for segs in hits:
        check_connector(segs, a, b)
        signs = turn_signs(segs)
        assert signs == pytest.approx([-math.pi / 4.0, -math.pi / 4.0], abs=1e-9), signs


def test_v01_connector_set_for_the_counterexample_is_incomplete():
    """Documented v0.1 behaviour: the short same-turn connector was missing."""
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, 100.0, 0.0), (0.0, 1.0, 0.0))
    conns = connect(a, b)
    for segs in conns:
        check_connector(segs, a, b)
    totals = sorted(round(sum(s.length for s in c), 6) for c in conns)
    assert min(totals) == pytest.approx(EXPECTED_SAME_TURN, abs=1e-6), totals


# ---------------------------------------------------------- LSR / RSL intact
# Parallel tangents with a lateral offset of 80 um: the same-turn families need
# a near-full circle, the mixed families give the short S curve.  Closed form:
#   C1 = (0, +R) [left of +x], C2 = (0, 80) + (0, -R) [right of +x]  ->  dc = (0, 80-2R)
#   L  = sqrt((80-2R)^2 - 4R^2)     phi = atan2(2R, L)     th = pi/2 + phi
#   LSR: first arc +th (left), straight L, second arc -th (right)
#   RSL: the mirror image (first arc -th, second arc +th)
OFFSET = 80.0
L_MIX = math.sqrt((OFFSET - 2.0 * R) ** 2 - 4.0 * R * R)
TH_MIX = math.pi / 2.0 + math.atan2(2.0 * R, L_MIX)
TOTAL_MIX = R * 2.0 * TH_MIX + L_MIX


def test_lsr_keeps_its_v01_signs():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((0.0, OFFSET, 0.0), (1.0, 0.0, 0.0))
    conns = connect(a, b)
    hits = by_total(conns, TOTAL_MIX, tol=1e-6)
    assert hits, ("no LSR connector; totals = %r" %
                  sorted(round(sum(s.length for s in c), 6) for c in conns))
    for segs in hits:
        check_connector(segs, a, b)
        signs = turn_signs(segs)
        assert signs == pytest.approx([TH_MIX, -TH_MIX], abs=1e-9), signs
        assert segs[1].length == pytest.approx(L_MIX, abs=1e-9)


def test_rsl_keeps_its_v01_signs():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((0.0, -OFFSET, 0.0), (1.0, 0.0, 0.0))
    conns = connect(a, b)
    hits = by_total(conns, TOTAL_MIX, tol=1e-6)
    assert hits, ("no RSL connector; totals = %r" %
                  sorted(round(sum(s.length for s in c), 6) for c in conns))
    for segs in hits:
        check_connector(segs, a, b)
        signs = turn_signs(segs)
        assert signs == pytest.approx([-TH_MIX, TH_MIX], abs=1e-9), signs


def test_sweep_larger_than_two_pi_is_handled():
    """RSR of a 180 degree heading reversal needs 270 + 270 degrees of arc."""
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((0.0, 100.0, 0.0), (-1.0, 0.0, 0.0))
    conns = connect(a, b)
    totals = sorted(round(sum(s.length for s in c), 6) for c in conns)
    rsr_total = R * 3.0 * math.pi + 120.0
    hits = by_total(conns, rsr_total)
    assert hits, totals
    for segs in hits:
        check_connector(segs, a, b)
        signs = turn_signs(segs)
        assert signs == pytest.approx([-3.0 * math.pi / 2.0, -3.0 * math.pi / 2.0],
                                      abs=1e-9)
        assert abs(signs[0]) + abs(signs[1]) > 2.0 * math.pi


# ------------------------------------------- arbitrary planes / rigid motions
def _rotated(pose, angles):
    ax, ay, az = angles
    p = pose.position
    t = pose.tangent
    for axis, ang in (ax, ay, az):
        p = tuple(rotate_about(p, axis, ang))
        t = tuple(rotate_about(t, axis, ang))
    return Pose3D(p, t)


AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def test_connector_lengths_are_rigid_motion_invariant():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, 100.0, 0.0), (0.0, 1.0, 0.0))
    base = sorted(round(sum(s.length for s in c), 6) for c in connect(a, b))
    for rot in (0.7, 1.9, -2.4):
        for axis in AXES:
            a2 = Pose3D(tuple(rotate_about(a.position, axis, rot)),
                        tuple(rotate_about(a.tangent, axis, rot)))
            b2 = Pose3D(tuple(rotate_about(b.position, axis, rot)),
                        tuple(rotate_about(b.tangent, axis, rot)))
            got = sorted(round(sum(s.length for s in c), 6) for c in connect(a2, b2))
            assert got == pytest.approx(base, abs=1e-6), (axis, rot)
            for c in connect(a2, b2):
                check_connector(c, a2, b2)
    # arbitrary rotation + translation: a genuinely tilted connector plane
    ax = (1.0 / math.sqrt(3.0),) * 3
    a3 = Pose3D((11.0, -7.0, 3.0), (1.0, 0.0, 0.0))
    b3 = Pose3D((111.0, 93.0, 3.0), (0.0, 1.0, 0.0))
    a3 = Pose3D(tuple(rotate_about(a3.position, ax, 1.1)), tuple(rotate_about(a3.tangent, ax, 1.1)))
    b3 = Pose3D(tuple(rotate_about(b3.position, ax, 1.1)), tuple(rotate_about(b3.tangent, ax, 1.1)))
    got3 = sorted(round(sum(s.length for s in c), 6) for c in connect(a3, b3))
    assert got3 == pytest.approx(base, abs=1e-6)
    normals = [s.normal for c in connect(a3, b3) for s in c if s.kind == "arc"]
    assert any(abs(abs(n[2]) - 1.0) > 1e-6 for n in normals), \
        "the connector arcs must live in a non-XY plane here"


def test_collinear_degenerate_straight_connector():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((50.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    conns = connect(a, b)
    assert any(len(c) == 1 and c[0].kind == "line" and
               c[0].length == pytest.approx(50.0) for c in conns), conns
    # identical pose -> no connector, never a degenerate zero-length segment
    assert connect(a, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))) == []


def test_non_coplanar_poses_have_no_connector_in_this_family():
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, 50.0, 0.0), (0.0, 0.0, 1.0))
    assert connect(a, b) == []
    # ... but the same two tangents ARE coplanar when the offset leaves the plane
    c = Pose3D((100.0, 0.0, 70.0), (0.0, 0.0, 1.0))
    assert connect(a, c), "the xz-plane pose pair must have connectors"


# ------------------------------------------------------- engine integration
def test_connector_actions_offer_the_lsl_connector_to_the_search():
    cfg = RouterConfig(connector_radii_um=(10.0,), rmin_um=10.0,
                       connector_max_length_um=3000.0)
    a = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    b = Pose3D((100.0, 100.0, 0.0), (0.0, 1.0, 0.0))
    acts = connector_actions(a, b, cfg)
    assert acts, "the connector action generator must offer the LSL connector"
    lens = sorted(round(x.length, 6) for x in acts)
    assert min(lens) == pytest.approx(EXPECTED_SAME_TURN, abs=1e-6)
    for act in acts:
        check_connector(act.segments, a, b, act.radius_um)