"""Phase 1 geometry: Pose3D, LineSegment3D, CircularArc3D.

All lengths are micrometres (um), all angles radians.

A CircularArc3D is a genuine space curve of an arbitrary plane: it stores the
centre, an orthonormal in-plane basis (u, v), the radius and the signed sweep
theta.  The plane normal is n = u x v.  Point and tangent evaluation:

    P(phi) = C + R (cos(phi) u + sin(phi) v),      phi in [0, theta]
    T(phi) = sign(theta) * (-sin(phi) u + cos(phi) v)
    L      = R |theta|

It is NOT an XY arc with a layer label attached.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from .vec import (Vec3, cross, dot, is_finite_vec, lerp, norm, scale, sub, unit)


@dataclass(frozen=True)
class Pose3D:
    """3D position plus a true 3D unit tangent."""
    position: Vec3
    tangent: Vec3

    def __post_init__(self) -> None:
        if not is_finite_vec(self.position):
            raise ValueError("Pose3D position must be 3 finite numbers: %r" % (self.position,))
        if not is_finite_vec(self.tangent):
            raise ValueError("Pose3D tangent must be 3 finite numbers: %r" % (self.tangent,))
        object.__setattr__(self, "position", tuple(float(c) for c in self.position))
        object.__setattr__(self, "tangent", unit(tuple(float(c) for c in self.tangent),
                                                 what="Pose3D.tangent"))

    @property
    def x(self) -> float:
        return self.position[0]

    @property
    def y(self) -> float:
        return self.position[1]

    @property
    def z(self) -> float:
        return self.position[2]

    @property
    def azimuth_rad(self) -> float:
        return math.atan2(self.tangent[1], self.tangent[0])

    @property
    def elevation_rad(self) -> float:
        return math.asin(max(-1.0, min(1.0, self.tangent[2])))

    def to_dict(self) -> dict:
        return {"position": list(self.position), "tangent": list(self.tangent)}

    @classmethod
    def from_dict(cls, d: dict) -> "Pose3D":
        return cls(tuple(d["position"]), tuple(d["tangent"]))


@dataclass(frozen=True)
class LineSegment3D:
    """Straight space segment; analytic length, position and tangent."""
    p0: Vec3
    p1: Vec3
    _aabb_cache: tuple = field(init=False, default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not is_finite_vec(self.p0) or not is_finite_vec(self.p1):
            raise ValueError("LineSegment3D endpoints must be 3 finite numbers")
        object.__setattr__(self, "p0", tuple(float(c) for c in self.p0))
        object.__setattr__(self, "p1", tuple(float(c) for c in self.p1))
        if norm(sub(self.p1, self.p0)) <= 1e-9:
            raise ValueError("degenerate LineSegment3D (zero length): %r -> %r"
                             % (self.p0, self.p1))
        object.__setattr__(self, "_aabb_cache",
                           (tuple(min(a, b) for a, b in zip(self.p0, self.p1)),
                            tuple(max(a, b) for a, b in zip(self.p0, self.p1))))

    kind = "line"

    @property
    def length(self) -> float:
        return norm(sub(self.p1, self.p0))

    @property
    def direction(self) -> Vec3:
        return unit(sub(self.p1, self.p0), what="LineSegment3D direction")

    @property
    def start_pose(self) -> Pose3D:
        return Pose3D(self.p0, self.direction)

    @property
    def end_pose(self) -> Pose3D:
        return Pose3D(self.p1, self.direction)

    @property
    def min_radius_um(self) -> float:
        return math.inf

    def point_at(self, s: float) -> Vec3:
        return lerp(self.p0, self.p1, s / self.length)

    def tangent_at(self, s: float) -> Vec3:
        return self.direction

    def curvature_at(self, s: float) -> float:
        return 0.0

    def aabb(self) -> Tuple[Vec3, Vec3]:
        return self._aabb_cache

    def sample(self, n: int) -> List[Vec3]:
        if n < 2:
            raise ValueError("sample needs n >= 2")
        L = self.length
        return [self.point_at(L * i / (n - 1)) for i in range(n)]

    def to_dict(self) -> dict:
        return {"kind": "line", "p0": list(self.p0), "p1": list(self.p1),
                "length_um": self.length}

    @classmethod
    def from_dict(cls, d: dict) -> "LineSegment3D":
        return cls(tuple(d["p0"]), tuple(d["p1"]))


def _phase_in_interval(phase: float, lo: float, hi: float) -> bool:
    """True when some phase + 2 k pi lies inside [lo, hi] (lo <= hi)."""
    two_pi = 2.0 * math.pi
    k = math.floor((lo - phase) / two_pi)
    for kk in (k, k + 1, k + 2):
        p = phase + kk * two_pi
        if lo - 1e-12 <= p <= hi + 1e-12:
            return True
    return False


@dataclass(frozen=True)
class CircularArc3D:
    """Circular arc in an arbitrary 3D plane.

    centre  : arc centre
    u, v    : orthonormal in-plane basis (v = tangent direction at phi = 0)
    radius  : R > 0
    theta   : signed sweep in radians, theta != 0
    """
    center: Vec3
    u: Vec3
    v: Vec3
    radius: float
    theta: float
    _aabb_cache: tuple = field(init=False, default=None, repr=False, compare=False)
    _phi_lo: float = field(init=False, default=0.0, repr=False, compare=False)
    _phi_hi: float = field(init=False, default=0.0, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not is_finite_vec(self.center) or not is_finite_vec(self.u) or not is_finite_vec(self.v):
            raise ValueError("CircularArc3D centre/basis must be 3 finite numbers")
        object.__setattr__(self, "center", tuple(float(c) for c in self.center))
        u = tuple(float(c) for c in self.u)
        v = tuple(float(c) for c in self.v)
        R = float(self.radius)
        th = float(self.theta)
        if not math.isfinite(R) or R <= 0.0:
            raise ValueError("illegal CircularArc3D radius (must be finite and > 0): %r" % (self.radius,))
        if not math.isfinite(th) or abs(th) <= 1e-12:
            raise ValueError("degenerate CircularArc3D sweep theta (finite, non-zero): %r" % (self.theta,))
        nu, nv = norm(u), norm(v)
        if abs(nu - 1.0) > 1e-9 or abs(nv - 1.0) > 1e-9:
            raise ValueError("CircularArc3D basis vectors must be unit length")
        if abs(dot(u, v)) > 1e-9:
            raise ValueError("CircularArc3D basis vectors must be orthogonal")
        object.__setattr__(self, "u", u)
        object.__setattr__(self, "v", v)
        object.__setattr__(self, "radius", R)
        object.__setattr__(self, "theta", th)
        object.__setattr__(self, "_phi_lo", 0.0 if th > 0 else th)
        object.__setattr__(self, "_phi_hi", th if th > 0 else 0.0)
        object.__setattr__(self, "_aabb_cache", self._compute_aabb())

    kind = "arc"

    @property
    def normal(self) -> Vec3:
        """Plane normal n = u x v (right handed with the (u,v) basis)."""
        return cross(self.u, self.v)

    @property
    def length(self) -> float:
        return self.radius * abs(self.theta)

    @property
    def min_radius_um(self) -> float:
        return self.radius

    @property
    def curvature(self) -> float:
        return 1.0 / self.radius

    def point_at_phi(self, phi: float) -> Vec3:
        c, s = math.cos(phi), math.sin(phi)
        return (self.center[0] + self.radius * (c * self.u[0] + s * self.v[0]),
                self.center[1] + self.radius * (c * self.u[1] + s * self.v[1]),
                self.center[2] + self.radius * (c * self.u[2] + s * self.v[2]))

    def tangent_at_phi(self, phi: float) -> Vec3:
        c, s = math.cos(phi), math.sin(phi)
        sgn = 1.0 if self.theta > 0 else -1.0
        return (sgn * (-s * self.u[0] + c * self.v[0]),
                sgn * (-s * self.u[1] + c * self.v[1]),
                sgn * (-s * self.u[2] + c * self.v[2]))

    def phi_of_arclength(self, s: float) -> float:
        return (s / self.radius) * (1.0 if self.theta > 0 else -1.0)

    def point_at(self, s: float) -> Vec3:
        return self.point_at_phi(self.phi_of_arclength(s))

    def tangent_at(self, s: float) -> Vec3:
        return self.tangent_at_phi(self.phi_of_arclength(s))

    def curvature_at(self, s: float) -> float:
        return self.curvature

    @property
    def start(self) -> Vec3:
        return self.point_at_phi(0.0)

    @property
    def end(self) -> Vec3:
        return self.point_at_phi(self.theta)

    @property
    def start_pose(self) -> Pose3D:
        return Pose3D(self.start, self.tangent_at_phi(0.0))

    @property
    def end_pose(self) -> Pose3D:
        return Pose3D(self.end, self.tangent_at_phi(self.theta))

    def sample(self, n: int) -> List[Vec3]:
        if n < 2:
            raise ValueError("sample needs n >= 2")
        return [self.point_at(self.length * i / (n - 1)) for i in range(n)]

    def _phi_in_sweep(self, phi: float) -> bool:
        lo, hi = (0.0, self.theta) if self.theta > 0 else (self.theta, 0.0)
        two_pi = 2.0 * math.pi
        k = math.floor((lo - phi) / two_pi)
        for kk in (k, k + 1, k + 2):
            p = phi + kk * two_pi
            if lo - 1e-12 <= p <= hi + 1e-12:
                return True
        return False

    def aabb(self) -> Tuple[Vec3, Vec3]:
        return self._aabb_cache

    def _compute_aabb(self) -> Tuple[Vec3, Vec3]:
        """EXACT axis-aligned bounding box (analytic coordinate extrema).

        coordinate i on the arc is  c_i + A cos(phi - phi_max)  with
        A = R hypot(u_i, v_i) and phi_max = atan2(v_i, u_i), so its maximum is
        attained exactly at phi = phi_max (+ 2 k pi) and its minimum exactly at
        phi = phi_max + pi (+ 2 k pi).  Each extreme is added ONLY when such a
        phase really lies inside the sweep.

        v0.1 tested phi_max and phi_max + pi in a loop that also added the
        opposite extreme (ci + amp for the phi_max + pi branch), so the box was
        inflated to [c - A, c + A] whenever only ONE of the two extremes was
        reached.  The box stayed conservative but was no longer the claimed
        "exact analytic bounding box", and it could reject a legal arc (see
        test_inflated_arc_aabb_does_not_falsely_reject_a_legal_arc).
        """
        lo_phi = 0.0 if self.theta > 0 else self.theta
        hi_phi = self.theta if self.theta > 0 else 0.0
        span = hi_phi - lo_phi
        p0 = self.point_at_phi(0.0)
        p1 = self.point_at_phi(self.theta)
        lo: List[float] = []
        hi: List[float] = []
        for i in range(3):
            ui, vi, ci = self.u[i], self.v[i], self.center[i]
            vals = [p0[i], p1[i]]
            amp = self.radius * math.hypot(ui, vi)
            if amp > 0.0:
                if span >= 2.0 * math.pi - 1e-12:
                    vals.append(ci + amp)
                    vals.append(ci - amp)
                else:
                    phi_max = math.atan2(vi, ui)
                    if _phase_in_interval(phi_max, lo_phi, hi_phi):
                        vals.append(ci + amp)
                    if _phase_in_interval(phi_max + math.pi, lo_phi, hi_phi):
                        vals.append(ci - amp)
            lo.append(min(vals))
            hi.append(max(vals))
        return (tuple(lo), tuple(hi))

    def to_dict(self) -> dict:
        return {"kind": "arc", "center": list(self.center), "u": list(self.u),
                "v": list(self.v), "radius_um": self.radius,
                "theta_rad": self.theta, "theta_deg": math.degrees(self.theta),
                "normal": list(self.normal), "length_um": self.length}

    @classmethod
    def from_dict(cls, d: dict) -> "CircularArc3D":
        return cls(tuple(d["center"]), tuple(d["u"]), tuple(d["v"]),
                   float(d["radius_um"]), float(d["theta_rad"]))

    @classmethod
    def from_pose(cls, pose: Pose3D, normal: Vec3, radius: float, sweep: float) -> "CircularArc3D":
        """Arc leaving the pose inside the plane with the given normal.

        A positive sweep rotates the tangent about +normal (right-hand rule),
        i.e. the tangent turns towards normal x tangent.  The stored theta is
        always positive; a negative sweep is represented by flipping the
        in-plane basis so that the start tangent stays exactly pose.tangent.
        """
        R = float(radius)
        if not math.isfinite(R) or R <= 0.0:
            raise ValueError("illegal radius for CircularArc3D.from_pose: %r" % (radius,))
        if not math.isfinite(sweep) or abs(sweep) <= 1e-12:
            raise ValueError("degenerate sweep for CircularArc3D.from_pose: %r" % (sweep,))
        n = unit(normal, what="arc plane normal")
        t = pose.tangent
        if abs(dot(t, n)) > 1e-7:
            raise ValueError("arc plane normal must be perpendicular to the tangent "
                             "(dot=%.3e)" % dot(t, n))
        w = n if sweep > 0 else scale(n, -1.0)
        v = t
        u = cross(v, w)
        center = sub(pose.position, scale(u, R))
        return cls(center, u, v, R, abs(sweep))


__all__ = ["Pose3D", "LineSegment3D", "CircularArc3D"]