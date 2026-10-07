"""Small pure-python 3D vector helpers (tuples of floats, micrometres)."""

from __future__ import annotations

import math

Vec3 = tuple

__all__ = ["Vec3", "add", "sub", "scale", "dot", "cross", "norm", "unit",
           "distance", "lerp", "angle_between", "rotate_about", "signed_angle",
           "is_finite_vec", "rot90"]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def unit(a: Vec3, tol: float = 1e-12, what: str = "vector") -> Vec3:
    n = norm(a)
    if not math.isfinite(n) or n <= tol:
        raise ValueError("degenerate (zero-length) %s: %r" % (what, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def distance(a: Vec3, b: Vec3) -> float:
    return norm(sub(a, b))


def lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


def angle_between(a: Vec3, b: Vec3) -> float:
    """Angle in radians in [0, pi], numerically robust for tiny angles."""
    return math.atan2(norm(cross(a, b)), dot(a, b))


def rotate_about(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    """Rodrigues rotation of v about unit axis by angle radians."""
    c = math.cos(angle)
    s = math.sin(angle)
    k = dot(axis, v)
    cr = cross(axis, v)
    return (v[0] * c + cr[0] * s + axis[0] * k * (1.0 - c),
            v[1] * c + cr[1] * s + axis[1] * k * (1.0 - c),
            v[2] * c + cr[2] * s + axis[2] * k * (1.0 - c))


def signed_angle(u: Vec3, v: Vec3, axis: Vec3) -> float:
    """Signed angle from u to v measured about unit axis (in [-pi, pi])."""
    return math.atan2(dot(cross(u, v), axis), dot(u, v))


def rot90(v: Vec3, axis: Vec3) -> Vec3:
    """Right-handed +90 degree rotation of v about unit axis."""
    return cross(axis, v)


def is_finite_vec(a, n: int = 3) -> bool:
    if len(a) != n:
        return False
    try:
        for c in a:
            if not math.isfinite(c):
                return False
    except TypeError:
        return False
    return True
