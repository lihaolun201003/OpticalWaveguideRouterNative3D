"""Phase 3 tests: 3D motion primitives, their axes, signs and mode discipline."""

import math

import pytest

from native3d.config import RouterConfig
from native3d.geometry import Pose3D
from native3d.geometry.vec import dot, norm, sub
from native3d.search.actions import (Action, generate_actions, local_frame,
                                     primitive_is_in_plane)


def actions_by_prefix(acts, prefix):
    return [a for a in acts if a.name.startswith(prefix)]


def test_action_families_exist_including_oblique_skew():
    cfg = RouterConfig()
    acts = generate_actions(Pose3D((0, 0, 0), (1, 0, 0)), cfg, True)
    names = [a.name for a in acts]
    assert any(n.startswith("straight") for n in names)
    assert any(n.startswith("yaw_left") for n in names)
    assert any(n.startswith("yaw_right") for n in names)
    assert any(n.startswith("pitch_up") for n in names)
    assert any(n.startswith("pitch_down") for n in names)
    assert any(n.startswith("skew_tilt") for n in names)
    skew = actions_by_prefix(acts, "skew")[0]
    # the skew arc plane is genuinely tilted: not horizontal, not vertical
    n = skew.segments[0].normal
    assert abs(n[2]) > 0.1 and abs(n[2]) < 0.99


def test_sign_conventions_from_a_known_pose():
    """+x heading, z up: yaw left -> +y, pitch up -> +z, right/down mirrored."""
    cfg = RouterConfig()
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))

    def end(action):
        return action.end_pose

    acts = {(a.name.split("_")[0] + "_" + a.name.split("_")[1] if a.name.startswith(("yaw", "pitch"))
             else a.name.split("_")[0]): a
            for a in generate_actions(pose, cfg, True)}

    yl = [a for a in generate_actions(pose, cfg, True)
          if a.name.startswith("yaw_left_90")][0]
    yr = [a for a in generate_actions(pose, cfg, True)
          if a.name.startswith("yaw_right_90")][0]
    pu = [a for a in generate_actions(pose, cfg, True)
          if a.name.startswith("pitch_up_90")][0]
    pd = [a for a in generate_actions(pose, cfg, True)
          if a.name.startswith("pitch_down_90")][0]
    assert end(yl).tangent == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert end(yr).tangent == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)
    assert end(pu).tangent == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)
    assert end(pd).tangent == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)
    # yaw keeps z, pitch changes z
    assert end(yl).position[2] == pytest.approx(0.0, abs=1e-9)
    assert end(pu).position[2] > 0.0
    assert end(pd).position[2] < 0.0


def test_skew_action_changes_azimuth_and_elevation_at_once():
    cfg = RouterConfig()
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    skew = [a for a in generate_actions(pose, cfg, True)
            if a.name.startswith("skew_tilt45_plus_45")][0]
    t = skew.end_pose.tangent
    assert t[1] > 0.05 and t[2] > 0.05, "oblique action must move both angles"
    assert skew.end_pose.position[2] > 0.0
    # its arc plane normal is tilted
    n = skew.segments[0].normal
    assert abs(n[2]) > 0.1 and abs(n[2]) < 0.99


def test_every_primitive_is_a_real_3d_motion_with_analytic_radius():
    cfg = RouterConfig()
    pose = Pose3D((5.0, -3.0, 2.0), (0.6, 0.8, 0.0))
    for a in generate_actions(pose, cfg, True):
        assert a.length > 0.0
        end = a.end_pose
        assert norm(sub(end.position, pose.position)) > 0.0
        for seg in a.segments:
            if seg.kind == "arc":
                assert seg.radius >= cfg.rmin_um
                assert seg.length == pytest.approx(seg.radius * abs(seg.theta))


def test_two_d_mode_generates_only_in_plane_primitives():
    cfg = RouterConfig()
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    acts = generate_actions(pose, cfg, allow_out_of_plane=False)
    assert acts, "2D mode still has straight + yaw candidates"
    assert not any(a.name.startswith(("pitch", "skew")) for a in acts)
    for a in acts:
        assert primitive_is_in_plane(a, 0.0)
        assert a.z_change_um == pytest.approx(0.0, abs=1e-12)
    # and the 3D mode has strictly more candidates
    acts3 = generate_actions(pose, cfg, allow_out_of_plane=True)
    assert len(acts3) > len(acts)


def test_local_frame_is_orthonormal_and_global_z_anchored():
    for t in [(1, 0, 0), (0, 1, 0), (0.6, 0.8, 0.0), (0.5, 0.0, math.sqrt(0.75))]:
        t = tuple(float(c) for c in t)
        n = math.sqrt(sum(c * c for c in t))
        t = tuple(c / n for c in t)
        left, n_yaw, a_up = local_frame(t)
        for v in (left, n_yaw, a_up):
            assert norm(v) == pytest.approx(1.0)
            assert abs(dot(v, t)) < 1e-9
        assert norm(sub(a_up, tuple(-c for c in left))) < 1e-12
    # horizontal tangent: yaw axis is exactly the global z axis
    left, n_yaw, a_up = local_frame((1.0, 0.0, 0.0))
    assert n_yaw == pytest.approx((0.0, 0.0, 1.0))
    assert left == pytest.approx((0.0, 1.0, 0.0))


def test_yaw_axis_is_perpendicular_even_for_an_inclined_tangent():
    """Yaw axis = local up-in-plane axis; documented behaviour for inclined t."""
    t = (0.5, 0.0, math.sqrt(0.75))
    left, n_yaw, a_up = local_frame(t)
    assert abs(dot(n_yaw, t)) < 1e-12
    assert norm(n_yaw) == pytest.approx(1.0)
    # for a horizontal tangent the yaw axis IS the global z axis and z is kept
    cfg = RouterConfig()
    yl_h = [a for a in generate_actions(Pose3D((0, 0, 0), (1, 0, 0)), cfg, True)
            if a.name.startswith("yaw_left_90")][0]
    assert yl_h.axis == pytest.approx((0.0, 0.0, 1.0))
    assert yl_h.end_pose.position[2] == pytest.approx(0.0, abs=1e-12)
    # for an inclined tangent the documented arc turns about the local up axis:
    # the arc lies in the tilted plane perpendicular to that axis, so the
    # elevation changes with it (a circular arc about the global z axis cannot
    # even start on an inclined tangent, because the axis must be perpendicular
    # to the tangent)
    acts = generate_actions(Pose3D((0, 0, 0), t), cfg, True)
    yl = [a for a in acts if a.name.startswith("yaw_left_90")][0]
    arc = yl.segments[0]
    assert abs(dot(arc.normal, n_yaw) - 1.0) < 1e-12
    for i in range(9):
        p = arc.point_at(arc.length * i / 8.0)
        assert abs(dot((p[0] - 0.0, p[1] - 0.0, p[2] - 0.0), n_yaw)) < 1e-9, \
            "the whole arc lies in the plane perpendicular to the yaw axis"
    assert yl.end_pose.elevation_rad != pytest.approx(math.asin(t[2]), abs=1e-3)
