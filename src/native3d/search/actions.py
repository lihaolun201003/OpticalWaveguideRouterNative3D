"""Phase 3: native 3D motion primitives.

Families (all generated FROM a real 3D pose and expanded during search):

straight(L)          straight run along the current unit 3D tangent.
yaw_left/right(th)   circular arc about the local up-in-plane axis
                     n_yaw = normalize(z_hat - (z_hat . t) t) = t x left.
                     For a HORIZONTAL tangent n_yaw is exactly the global z axis
                     and the arc is a pure horizontal-plane turn that keeps z.
                     For an inclined tangent no circular arc can start on the
                     global z axis at all (the axis must be perpendicular to the
                     tangent), so the arc plane is the tilted plane
                     perpendicular to the local up axis: the turn is the natural
                     generalisation of a yaw and it changes the elevation as
                     well.  This is the honest definition; it is NOT claimed to
                     preserve the elevation for inclined tangents.
pitch_up/down(th)    circular arc about the local axis left = normalize(z x t)
                     (pitch up = rotation about -left, so the tangent turns
                     towards +z).
skew(tilt, +-th)     circular arc about the tilted axis
                     a(alpha) = cos(alpha) n_yaw + sin(alpha) (-left); for
                     alpha in (0, 90) the arc plane is tilted with respect to
                     both the horizontal plane and the vertical plane, so the
                     motion changes azimuth AND elevation at once.  This is the
                     genuinely oblique 3D primitive.

Sign convention: a positive sweep about axis a rotates the tangent, by the
right-hand rule, towards a x t.  Axes are expressed in the GLOBAL frame (z is
the chip vertical), but are built from the local tangent, so the family is a
local-frame family anchored to the global vertical; this is stated explicitly
so the yaw/pitch definition cannot be mistaken for an XY-only orientation.

All primitives are single circular arcs / straight segments in an arbitrary 3D
plane: z of the next state is produced by the primitive itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from ..geometry.vec import (Vec3, add, cross, dot, norm, scale, sub, unit)

Z_HAT: Vec3 = (0.0, 0.0, 1.0)
Y_HAT: Vec3 = (0.0, 1.0, 0.0)


@dataclass(frozen=True)
class Action:
    name: str
    segments: Tuple[object, ...]
    kind: str = "motion"        # "motion" | "connector" | "finite_*" | "port_attachment"
    axis: Optional[Vec3] = None
    sweep_rad: float = 0.0
    radius_um: float = math.inf
    # A port attachment of length ZERO (the canonical node IS the port pose) has no
    # primitive at all; its two poses are supplied explicitly so that start_pose /
    # end_pose stay defined and length == 0.
    virtual_start: Optional[Pose3D] = None
    virtual_end: Optional[Pose3D] = None

    @property
    def length(self) -> float:
        return sum(s.length for s in self.segments)

    @property
    def start_pose(self) -> Pose3D:
        if self.segments:
            return self.segments[0].start_pose
        if self.virtual_start is None:
            raise ValueError("Action %r has no segments and no virtual start pose"
                             % self.name)
        return self.virtual_start

    @property
    def end_pose(self) -> Pose3D:
        if self.segments:
            return self.segments[-1].end_pose
        if self.virtual_end is None:
            raise ValueError("Action %r has no segments and no virtual end pose"
                             % self.name)
        return self.virtual_end

    @property
    def z_change_um(self) -> float:
        return self.end_pose.position[2] - self.start_pose.position[2]

    @property
    def elevation_change_deg(self) -> float:
        return math.degrees(self.end_pose.elevation_rad - self.start_pose.elevation_rad)

    def to_dict(self) -> dict:
        return {"name": self.name, "kind": self.kind,
                "length_um": self.length,
                "zero_length_port_attachment": (not self.segments),
                "z_change_um": self.z_change_um,
                "elevation_change_deg": self.elevation_change_deg,
                "radius_um": (None if math.isinf(self.radius_um) else self.radius_um),
                "sweep_deg": math.degrees(self.sweep_rad),
                "axis": (None if self.axis is None else list(self.axis)),
                "segments": [s.to_dict() for s in self.segments]}


def local_frame(t: Vec3) -> Tuple[Vec3, Vec3, Vec3]:
    """Return (left, n_yaw, a_up) for a unit tangent t.

    left   = normalize(z x t)          : in-plane side axis (pitch axis)
    n_yaw  = normalize(z - (z.t) t)    : horizontal-perpendicular (yaw axis)
    a_up   = -left                     : pitch-up axis (positive sweep climbs)

    Degenerate case: t nearly parallel to z.  left is then taken as +y (an
    explicit, documented rule; any axis perpendicular to t is admissible and
    this one keeps the local frame continuous with the horizontal case).
    """
    c = cross(Z_HAT, t)
    if norm(c) > 1e-9:
        left = unit(c, what="local left axis")
    else:
        # t parallel to z: any perpendicular axis works; documented deterministic
        # fallback that keeps the frame orthonormal and continuous with the
        # horizontal case
        left = Y_HAT if t[2] >= 0 else scale(Y_HAT, -1.0)
    z_perp = sub(Z_HAT, scale(t, dot(Z_HAT, t)))
    if norm(z_perp) > 1e-9:
        n_yaw = unit(z_perp, what="yaw axis")
    else:
        # vertical tangent: local up-in-plane axis = t x left (unit, perp to t)
        n_yaw = unit(cross(t, left), what="yaw axis (vertical fallback)")
    a_up = scale(left, -1.0)
    return left, n_yaw, a_up


def _arc_action(name: str, pose: Pose3D, axis: Vec3, radius: float,
                sweep: float) -> Action:
    arc = CircularArc3D.from_pose(pose, axis, radius, sweep)
    return Action(name=name, segments=(arc,), kind="motion",
                  axis=tuple(axis), sweep_rad=float(sweep), radius_um=float(radius))


def straight_action(pose: Pose3D, length: float) -> Action:
    p1 = add(pose.position, scale(pose.tangent, length))
    seg = LineSegment3D(pose.position, p1)
    return Action(name="straight_%g" % length, segments=(seg,), kind="motion")


def generate_actions(pose: Pose3D, cfg: RouterConfig,
                     allow_out_of_plane: bool = True) -> List[Action]:
    """Every candidate primitive from this pose (before constraint validation).

    With allow_out_of_plane=False only primitives that keep the whole path in
    the plane z = z(pose) are produced (straight + yaw); this is the 2D control
    mode of the same engine - no old 2D router is involved.
    """
    t = pose.tangent
    left, n_yaw, a_up = local_frame(t)
    out: List[Action] = []

    for L in cfg.straight_lengths_um:
        out.append(straight_action(pose, float(L)))

    for ang_deg in cfg.turn_angles_deg:
        ang = math.radians(float(ang_deg))
        for R in cfg.turn_radii_um:
            out.append(_arc_action("yaw_left_%g_%g" % (ang_deg, R), pose, n_yaw, float(R), +ang))
            out.append(_arc_action("yaw_right_%g_%g" % (ang_deg, R), pose, n_yaw, float(R), -ang))
            if allow_out_of_plane:
                out.append(_arc_action("pitch_up_%g_%g" % (ang_deg, R), pose, a_up, float(R), +ang))
                out.append(_arc_action("pitch_down_%g_%g" % (ang_deg, R), pose, left, float(R), +ang))
                for tilt_deg in cfg.skew_axis_tilt_deg:
                    al = math.radians(float(tilt_deg))
                    axis_raw = add(scale(n_yaw, math.cos(al)), scale(a_up, math.sin(al)))
                    if norm(axis_raw) <= 1e-9:
                        # degenerate tilt (yaw and pitch axes coincide): no skew
                        # plane exists for this pose, the action is simply skipped
                        continue
                    axis = unit(axis_raw, what="skew axis")
                    out.append(_arc_action("skew_tilt%g_plus_%g_%g" % (tilt_deg, ang_deg, R),
                                           pose, axis, float(R), +ang))
                    out.append(_arc_action("skew_tilt%g_minus_%g_%g" % (tilt_deg, ang_deg, R),
                                           pose, axis, float(R), -ang))
    return out


def primitive_is_in_plane(action: Action, z0: float, tol: float = 1e-9) -> bool:
    for seg in action.segments:
        lo, hi = seg.aabb()
        if abs(lo[2] - z0) > tol or abs(hi[2] - z0) > tol:
            return False
    return True
