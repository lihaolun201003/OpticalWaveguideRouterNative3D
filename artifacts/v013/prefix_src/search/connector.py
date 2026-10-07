"""Terminal connector: exact pose-to-pose 3D primitive sequences.

The connector is generated DURING search from the current real 3D pose to the
target 3D pose.  It is an ordinary candidate motion primitive of the engine: it
is checked by exactly the same full constraint stack (radius / boundary /
collision / spacing) before it may enter the open set, and it is accepted only
when its end pose matches the target pose within the tight tolerances.

Family implemented: planar Dubins arc-straight-arc (LSL, RSR, LSR, RSL) with a
common radius R >= Rmin, for pose pairs that are coplanar, plus the degenerate
straight connector.  The plane is the plane spanned by the start tangent and
the end tangent (or by the tangent and the displacement when the tangents are
parallel); it is a genuinely 3D plane, so the arcs of a connector are ordinary
arbitrary-plane space arcs and can change z.

Sweep convention (v0.1.1, re-derived - see the LSL/RSR comment below): with
e1 = start tangent, n = plane normal, e2 = n x e1 and theta_e the signed angle
from e1 to the target tangent,
    LEFT  arc sweep = mod2pi(angle reached - angle left)   (positive)
    RIGHT arc sweep = mod2pi(angle left - angle reached)   (negative)
A sweep is allowed to exceed 2*pi (a full extra turn), which is what the
same-turn families require when the two tangents are parallel.

Supported range: this is NOT a complete Dubins implementation.  Only the four
arc-straight-arc families with a common radius are generated, only for
COPLANAR pose pairs, and only one common radius per call.  No
arc-arc-arc (LRL/RLR) family and no non-coplanar family exist here.  Therefore a
missing connector means "this action set found no arc-straight-arc connector for
this radius", never "the pose pair is physically unreachable".
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from ..geometry.vec import Vec3, add, cross, dot, norm, scale, sub, unit
from .actions import Action

TWO_PI = 2.0 * math.pi


def _mod2pi(a: float) -> float:
    return a % TWO_PI


def _rot90_2d(v: Tuple[float, float]) -> Tuple[float, float]:
    return (-v[1], v[0])


def planar_connectors(pose_a: Pose3D, pose_b: Pose3D, radius: float,
                      coplanar_tol_um: float = 1e-6,
                      parallel_tol: float = 1e-9) -> List[List[object]]:
    """All valid planar Dubins arc-straight-arc connectors for this radius."""
    p, t = pose_a.position, pose_a.tangent
    q, d = pose_b.position, pose_b.tangent
    dp = sub(q, p)
    if norm(dp) <= 1e-9 and abs(dot(t, d) - 1.0) <= 1e-12:
        return []
    td = cross(t, d)
    ntd = norm(td)
    if ntd > parallel_tol:
        n = scale(td, 1.0 / ntd)
        if abs(dot(dp, n)) > coplanar_tol_um:
            return []
    else:
        tp = cross(t, dp)
        ntp = norm(tp)
        if ntp <= 1e-9:
            # collinear: only a straight run can connect
            if dot(t, d) > 0.0 and dot(dp, t) > 1e-9:
                return [[LineSegment3D(p, q)]]
            return []
        n = scale(tp, 1.0 / ntp)
    if abs(dot(t, n)) > 1e-7 or abs(dot(d, n)) > 1e-7:
        return []

    e1 = t
    e2 = cross(n, e1)

    def to2d(v: Vec3) -> Tuple[float, float]:
        return (dot(v, e1), dot(v, e2))

    def to3d(a: float, b: float) -> Vec3:
        return add(p, add(scale(e1, a), scale(e2, b)))

    Q = to2d(dp)
    theta_e = math.atan2(dot(d, e2), dot(d, e1))
    R = float(radius)
    cs_l = (0.0, R)
    cs_r = (0.0, -R)
    ce_l = (Q[0] + R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[0],
            Q[1] + R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[1])
    ce_r = (Q[0] - R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[0],
            Q[1] - R * _rot90_2d((math.cos(theta_e), math.sin(theta_e)))[1])

    specs = []
    # LSL / RSR: both circles lie on the SAME side of the straight, so the two
    # sweeps have the same sign as the family (s1 == s2).  Sweep derivation
    # (R = radius, p = start position, t = start tangent, d = target tangent,
    #  e1 = t, e2 = n x e1, theta_e = signed angle from e1 to d):
    #   first  arc: the tangent must become the straight direction u_ang, so a
    #               LEFT arc sweeps mod2pi(u_ang) and a RIGHT arc mod2pi(-u_ang);
    #   second arc: the tangent must become theta_e, so a LEFT arc sweeps
    #               mod2pi(theta_e - u_ang) and a RIGHT arc mod2pi(u_ang - theta_e).
    # v0.1 had these two the wrong way round, which made the second arc turn the
    # opposite way and the endpoint check then threw the whole family away
    # (tests/test_repair_d_connector_signs.py).
    for tag, cs, ce, s1, s2 in (("LSL", cs_l, ce_l, +1, +1),
                                ("RSR", cs_r, ce_r, -1, -1)):
        dc = (ce[0] - cs[0], ce[1] - cs[1])
        L = math.hypot(dc[0], dc[1])
        if L <= 1e-9:
            continue
        u_ang = math.atan2(dc[1], dc[0])
        if s1 > 0:
            th1 = _mod2pi(u_ang)
            th2 = _mod2pi(theta_e - u_ang)
        else:
            th1 = _mod2pi(-u_ang)
            th2 = _mod2pi(u_ang - theta_e)
        specs.append((tag, s1 * th1, L, u_ang, s2 * th2))
    # LSR / RSL
    for tag, cs, ce, s1, s2 in (("LSR", cs_l, ce_r, +1, -1),
                                ("RSL", cs_r, ce_l, -1, +1)):
        dc = (ce[0] - cs[0], ce[1] - cs[1])
        D = math.hypot(dc[0], dc[1])
        if D < 2.0 * R - 1e-9:
            continue
        L = math.sqrt(max(0.0, D * D - 4.0 * R * R))
        phi = math.atan2(2.0 * R, L) if L > 1e-12 else math.pi / 2.0
        base = math.atan2(dc[1], dc[0])
        u_ang = base + phi if s1 > 0 else base - phi
        # LSR / RSL: the two circles are on OPPOSITE sides (s1 == -s2).  The
        # first arc must still reach the straight direction u_ang and the second
        # arc must still reach theta_e, which is exactly the pair below; v0.1 had
        # this correct and v0.1.1 leaves it unchanged (regression-guarded by
        # test_lsr_keeps_its_v01_signs / test_rsl_keeps_its_v01_signs).
        if s1 > 0:
            th1 = _mod2pi(u_ang)
            th2 = _mod2pi(u_ang - theta_e)
        else:
            th1 = _mod2pi(-u_ang)
            th2 = _mod2pi(theta_e - u_ang)
        specs.append((tag, s1 * th1, L, u_ang, s2 * th2))

    out: List[List[object]] = []
    for tag, sw1, L, u_ang, sw2 in specs:
        segs: List[object] = []
        try:
            if abs(sw1) > 1e-12:
                arc1 = CircularArc3D.from_pose(Pose3D(p, t), n, R, sw1)
                segs.append(arc1)
                cur_pose = arc1.end_pose
            else:
                cur_pose = Pose3D(p, t)
            u3 = to3d(math.cos(u_ang), math.sin(u_ang))
            u_dir = unit(sub(u3, to3d(0.0, 0.0)), what="connector straight dir")
            if L > 1e-9:
                st = LineSegment3D(cur_pose.position, add(cur_pose.position, scale(u_dir, L)))
                segs.append(st)
                cur_pose = st.end_pose
            if abs(sw2) > 1e-12:
                arc2 = CircularArc3D.from_pose(Pose3D(cur_pose.position, u_dir), n, R, sw2)
                segs.append(arc2)
                cur_pose = arc2.end_pose
        except ValueError:
            continue
        if not segs:
            continue
        if norm(sub(cur_pose.position, q)) > coplanar_tol_um:
            continue
        if abs(dot(cur_pose.tangent, d) - 1.0) > 1e-9:
            continue
        out.append(segs)
    return out


def connector_actions(pose_a: Pose3D, pose_b: Pose3D, cfg: RouterConfig,
                      allow_out_of_plane: bool = True) -> List[Action]:
    """Validated-candidate connector actions from pose_a to the target pose_b."""
    actions: List[Action] = []
    radii = sorted({float(r) for r in cfg.connector_radii_um} | {cfg.rmin_um})
    for R in radii:
        for segs in planar_connectors(pose_a, pose_b, R):
            if not allow_out_of_plane:
                zs = [seg.aabb() for seg in segs]
                if any(abs(lo[2] - pose_a.position[2]) > 1e-9 or
                       abs(hi[2] - pose_a.position[2]) > 1e-9 for lo, hi in zs):
                    continue
            total = sum(s.length for s in segs)
            if total > cfg.connector_max_length_um:
                continue
            actions.append(Action(name="connector_R%g" % R, segments=tuple(segs),
                                  kind="connector", radius_um=R))
    return actions