"""v0.1.3 PART B3 regression: the near-unit basis / circular-arc contract.

THE DEFECT (v0.1.2, reproduced for real before the repair)

``CircularArc3D.__post_init__`` accepted a basis whose norm error was up to
1e-9 and then KEPT THE ORIGINAL BASIS.  With

    R = 2000 um, u = (1 + 8e-10, 0, 0), v = (0, 1, 0), theta = pi/2

the stored "circle" C + R (cos(phi) u + sin(phi) v) is really an ELLIPSE whose
minimum radius of curvature is R / (1 + 8e-10) = 1999.9999984 um, i.e. below the
admissible floor Rmin - radius_tol = 2000 - 1e-6 = 1999.999999 um (breach
6.0e-7 um), while every consumer used the single field R = 2000:

    production radius_check = safe, independent validator verdict = pass.

THE CONTRACT AFTER THE REPAIR (identical in geometry.primitives and in
validation.validate, so both accept and reject the SAME inputs)

* u and v are normalised to unit length and the STORED basis is the normalised
  one, so |P(phi) - C| = R for every stored point (up to a relative
  1e-12 + 2.3e-16);
* the input is rejected unless the unit defect of the INPUT basis
  (abs(|u| - 1), measured BEFORE normalising - after normalisation it is 0 by
  construction and the counterexample would slip through), the unit defect after
  normalisation and |dot(u_hat, v_hat)| are all <= 1e-12;
* normalisation is allowed only INSIDE that tolerance: a non-orthonormal
  geometry is never silently repaired and then certified as if the original
  geometry had been a circle;
* the requested radius is never rounded, snapped or quantised (this project has
  no lattice of admissible radii), so `radius` == `min_radius_um` ==
  1/`curvature` == the circumradius of the stored basis, bit for bit;
* `length` = R|theta|, the tangents, `phi_of_arclength`, `normal` and the
  analytic AABB all describe that same stored circle, and to_dict/from_dict is
  lossless for a legal arc.

MEASURED MARGIN.  Every real construction path of this project
(CircularArc3D.from_pose, cases.tilted_plane_arc, search.connector,
search.reference_graph, CircularArc3D.from_dict) produces a unit defect of
~1.1e-16 or less, i.e. more than three orders of magnitude inside 1e-12; the
probe artifacts/v013/probe/v013_validation_probe.py prints the measured worst
case over all of them.
"""

from __future__ import annotations

import json
import math

import pytest

from native3d.cases import tilted_plane_arc
from native3d.config import RouterConfig
from native3d.constraints.checks import radius_check
from native3d.geometry import CircularArc3D, Pose3D
from native3d.geometry.primitives import (BASIS_ORTHOGONALITY_TOL,
                                          BASIS_UNIT_DEFECT_TOL, basis_defects)
from native3d.validation.validate import check_segment_dict, validate_path_dict

from v013_helpers import prefix_module

R_BIG = 2000.0
DELTA = 8e-10
NEAR_UNIT_U = (1.0 + DELTA, 0.0, 0.0)
NEAR_UNIT_V = (0.0, 1.0, 0.0)
THETA = math.pi / 2.0
FLOOR_UM = 2000.0 - 1e-6                     # Rmin - radius_tol
ELLIPSE_RMIN_UM = R_BIG / (1.0 + DELTA)      # exact minimum curvature radius


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def arc_dict(u=NEAR_UNIT_U, v=NEAR_UNIT_V, R=R_BIG, theta=THETA):
    return {"kind": "arc", "center": [0.0, 0.0, 0.0], "u": list(u), "v": list(v),
            "radius_um": R, "theta_rad": theta}


def curvature_radius_min_of_ellipse(R, u, v, n=200000):
    """Minimum radius of curvature of C + R(cos phi u + sin phi v), by formula.

    With a = R|u| (on the u axis) and b = R|v| (on the v axis) the curve is an
    ellipse of semi-axes a, b when u . v == 0, and its radius of curvature is
    (a^2 sin^2 t + b^2 cos^2 t)^{3/2} / (a b) with a minimum of b^2/a at t = 0.
    The theta-free sampling below is only the numerical cross check.
    """
    a, b = R * math.hypot(*u), R * math.hypot(*v)
    worst = math.inf
    for i in range(0, 4 * n + 1):
        t = 2.0 * math.pi * i / (4 * n)
        num = (a * a * math.sin(t) ** 2 + b * b * math.cos(t) ** 2) ** 1.5
        worst = min(worst, num / (a * b))
    return worst


# ===========================================================================
# the counterexample
# ===========================================================================
def test_the_exact_ellipse_minimum_curvature_radius_documents_the_rejection():
    assert ELLIPSE_RMIN_UM == pytest.approx(1999.9999984, abs=1e-9)
    assert ELLIPSE_RMIN_UM < FLOOR_UM, "the real curve is below the allowed floor"
    assert FLOOR_UM - ELLIPSE_RMIN_UM == pytest.approx(6.0e-7, abs=1e-12)
    # independent numerical curvature: min over the ellipse is exactly R/(1+delta)
    numeric = curvature_radius_min_of_ellipse(R_BIG, NEAR_UNIT_U, NEAR_UNIT_V)
    assert numeric == pytest.approx(ELLIPSE_RMIN_UM, rel=1e-9)
    assert numeric < FLOOR_UM


def test_the_near_unit_basis_is_rejected_by_the_production_constructor():
    with pytest.raises(ValueError) as exc:
        CircularArc3D((0.0, 0.0, 0.0), NEAR_UNIT_U, NEAR_UNIT_V, R_BIG, THETA)
    msg = str(exc.value)
    assert "unit" in msg
    assert "8.000e-10" in msg
    # WHY the input defect must be measured BEFORE normalising: normalising this
    # vector gives EXACTLY unit in double precision, so a post-normalisation unit
    # check alone (defect 0.0) would let the counterexample through and only the
    # orthogonality check would remain - and it is 0.0 for this basis.
    unit_defect, unit_hat_defect, orth = basis_defects(NEAR_UNIT_U, NEAR_UNIT_V)
    dodgy = tuple(c / math.hypot(*NEAR_UNIT_U) for c in NEAR_UNIT_U)
    assert dodgy == (1.0, 0.0, 0.0)
    assert unit_defect == pytest.approx(DELTA, rel=1e-6)
    assert unit_hat_defect <= 2.3e-16
    assert orth == 0.0


def test_the_frozen_v012_constructor_really_accepted_it():
    """Red -> green: the frozen v0.1.2 snapshot keeps the raw basis and R."""
    frozen = prefix_module("geometry.primitives")
    arc = frozen.CircularArc3D((0.0, 0.0, 0.0), NEAR_UNIT_U, NEAR_UNIT_V, R_BIG,
                               THETA)
    assert arc.u == NEAR_UNIT_U                    # the ORIGINAL, non-unit basis
    assert arc.v == NEAR_UNIT_V
    assert arc.radius == R_BIG
    assert arc.min_radius_um == R_BIG              # claimed, but not the truth
    assert ELLIPSE_RMIN_UM < FLOOR_UM < arc.min_radius_um


def test_the_near_unit_basis_is_rejected_by_the_independent_validator():
    seg = arc_dict()
    with pytest.raises(ValueError) as exc:
        check_segment_dict(seg, "path.segments[0]")
    assert "unit" in str(exc.value)
    # as a PATH segment and as a REFERENCE centreline alike
    scene = {"chip_volume": {"lo": [-5000.0] * 3, "hi": [5000.0] * 3},
             "obstacles": [], "reference_centerlines": [],
             "source_pose": {"position": [0.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [R_BIG, 0.0, 0.0], "tangent": [0.0, 1.0, 0.0]}}
    with pytest.raises(ValueError):
        validate_path_dict({"segments": [seg]}, scene, cfg_dict())
    scene_ref = dict(scene)
    scene_ref["reference_centerlines"] = [seg]
    line_path = {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                               "p1": [100.0, 0.0, 0.0]}]}
    with pytest.raises(ValueError):
        validate_path_dict(line_path, scene_ref, cfg_dict())


def test_the_frozen_v012_validator_called_that_ellipse_a_pass():
    """The exact v0.1.2 verdict of the counterexample, from the frozen snapshot."""
    frozen_v = prefix_module("validation.validate")
    frozen_c = prefix_module("constraints.checks")
    frozen_cfg_mod = prefix_module("config")
    arc = prefix_module("geometry.primitives").CircularArc3D(
        (0.0, 0.0, 0.0), NEAR_UNIT_U, NEAR_UNIT_V, R_BIG, THETA)
    cfg = frozen_cfg_mod.RouterConfig(rmin_um=2000.0, radius_tol_um=1e-6,
                                      turn_radii_um=(2000.0,),
                                      connector_radii_um=(2000.0,))
    rc = frozen_c.radius_check([arc], cfg)
    assert rc.status == "safe"
    assert rc.bound_um == R_BIG
    scene = {"chip_volume": {"lo": [-5000.0] * 3, "hi": [5000.0] * 3},
             "obstacles": [], "reference_centerlines": [],
             "source_pose": {"position": list(arc.start),
                             "tangent": list(arc.tangent_at_phi(0.0))},
             "target_pose": {"position": list(arc.end),
                             "tangent": list(arc.tangent_at_phi(arc.theta))}}
    v0 = frozen_v.validate_path_dict({"segments": [arc_dict()]}, scene,
                                     cfg_dict(rmin_um=2000.0, radius_tol_um=1e-6,
                                              turn_radii_um=[2000.0],
                                              connector_radii_um=[2000.0]))
    assert v0["verdict"] == "pass"
    assert v0["radius_violations"] == []
    # AFTER the repair the same input cannot even be constructed, so the false
    # "safe"/"pass" channel is closed at the source and at the validator
    with pytest.raises(ValueError):
        CircularArc3D((0.0, 0.0, 0.0), NEAR_UNIT_U, NEAR_UNIT_V, R_BIG, THETA)
    with pytest.raises(ValueError):
        check_segment_dict(arc_dict(), "path.segments[0]")


# ===========================================================================
# constructor and validator accept/reject exactly the same bases
# ===========================================================================
ILLEGAL_BASES = {
    "near unit (1+8e-10)": ((1.0 + 8e-10, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "near unit (1-8e-10)": ((1.0 - 8e-10, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "exactly 2x too long": ((2.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "1e-6 too long": ((1.0 + 1e-6, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "anti-parallel": ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
    "near orthogonal 1e-9": ((1.0, 0.0, 0.0), (1e-9, 1.0, 0.0)),
    "near orthogonal 1e-6": ((1.0, 0.0, 0.0),
                             (math.cos(1e-6), math.sin(1e-6), 0.0)),
    "zero u": ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "zero v": ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "parallel equal": ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
}
LEGAL_BASES = {
    "xy": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "rotated xz": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    "sign flipped u": ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "sign flipped v": ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "tilted (1,1,1)": ((1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0),
                       (1.0 / math.sqrt(6.0), 1.0 / math.sqrt(6.0),
                        -2.0 / math.sqrt(6.0))),
    "near orthogonal 1e-13": ((1.0, 0.0, 0.0), (1e-13, 1.0, 0.0)),
}


@pytest.mark.parametrize("label", sorted(ILLEGAL_BASES))
def test_every_illegal_basis_is_rejected_by_BOTH_implementations(label):
    u, v = ILLEGAL_BASES[label]
    with pytest.raises(ValueError):
        CircularArc3D((0.0, 0.0, 0.0), u, v, R_BIG, THETA)
    with pytest.raises(ValueError):
        check_segment_dict(arc_dict(u, v), "path.segments[0]")


@pytest.mark.parametrize("label", sorted(LEGAL_BASES))
def test_every_legal_basis_is_accepted_by_BOTH_implementations(label):
    # the "near orthogonal 1e-13" entry is inside the documented 1e-12 tolerance
    u, v = LEGAL_BASES[label]
    arc = CircularArc3D((0.0, 0.0, 0.0), u, v, 50.0, THETA)
    check_segment_dict(arc_dict(u, v), "path.segments[0]")
    assert math.hypot(*arc.u) == pytest.approx(1.0, abs=2.3e-16)
    assert math.hypot(*arc.v) == pytest.approx(1.0, abs=2.3e-16)
    assert abs(arc.u[0] * arc.v[0] + arc.u[1] * arc.v[1] + arc.u[2] * arc.v[2]) \
        <= BASIS_ORTHOGONALITY_TOL


def test_a_base_from_a_sign_flip_is_a_rotation_not_a_defect():
    """(u, -v) mirrors the plane: it is still an exact circle of radius R."""
    a = CircularArc3D((1.0, 2.0, 3.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 40.0, 0.7)
    b = CircularArc3D((1.0, 2.0, 3.0), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0), 40.0, 0.7)
    assert a.normal == pytest.approx(tuple(-c for c in b.normal), abs=1e-15)
    for i in range(21):
        phi = 0.7 * i / 20.0
        pa, pb = a.point_at_phi(phi), b.point_at_phi(phi)
        assert math.dist(pa, (1.0, 2.0, 3.0)) == pytest.approx(40.0, abs=1e-12)
        assert math.dist(pb, (1.0, 2.0, 3.0)) == pytest.approx(40.0, abs=1e-12)
        assert math.dist(pa, pb) >= 0.0


def test_the_constructor_reports_the_defect_it_measured():
    for u, v in ((NEAR_UNIT_U, NEAR_UNIT_V),
                 ((1.0, 0.0, 0.0), (1e-9, 1.0, 0.0)),
                 ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))):
        unit_defect, unit_hat_defect, orth = basis_defects(u, v)
        assert unit_hat_defect <= 2.3e-16
        assert orth >= 0.0
        assert unit_defect == pytest.approx(max(abs(math.hypot(*u) - 1.0),
                                               abs(math.hypot(*v) - 1.0)))
    unit_defect, _, orth = basis_defects(NEAR_UNIT_U, NEAR_UNIT_V)
    assert unit_defect == pytest.approx(DELTA, rel=1e-6)
    assert unit_defect > BASIS_UNIT_DEFECT_TOL
    assert orth == 0.0
    unit_defect4, _, orth4 = basis_defects((1.0, 0.0, 0.0), (1e-9, 1.0, 0.0))
    assert unit_defect4 <= BASIS_UNIT_DEFECT_TOL
    assert orth4 > BASIS_ORTHOGONALITY_TOL


def test_normalisation_happens_only_inside_the_documented_tolerance():
    """1e-13 inside tolerance: the stored basis is unit and the curve a circle."""
    arc = CircularArc3D((0.0, 0.0, 0.0), (1.0 + 1e-13, 0.0, 0.0), (0.0, 1.0, 0.0),
                        2000.0, THETA)
    assert math.hypot(*arc.u) == pytest.approx(1.0, abs=2.3e-16)
    for i in range(11):
        p = arc.point_at_phi(THETA * i / 10.0)
        assert math.dist(p, arc.center) == pytest.approx(2000.0, rel=1e-12)
    # 1e-11 outside tolerance: rejected, never silently repaired
    with pytest.raises(ValueError):
        CircularArc3D((0.0, 0.0, 0.0), (1.0 + 1e-11, 0.0, 0.0), (0.0, 1.0, 0.0),
                      2000.0, THETA)


# ===========================================================================
# a legal arc really is a circle, and the radius policy is explicit
# ===========================================================================
def rotation_basis(normal):
    n = tuple(c / math.sqrt(sum(x * x for x in normal)) for c in normal)
    ref = (1.0, 0.0, 0.0)
    if abs(sum(ref[i] * n[i] for i in range(3))) > 0.9:
        ref = (0.0, 1.0, 0.0)
    d = sum(ref[i] * n[i] for i in range(3))
    u = tuple(ref[i] - d * n[i] for i in range(3))
    nu = math.sqrt(sum(c * c for c in u))
    u = tuple(c / nu for c in u)
    v = (n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2],
         n[0] * u[1] - n[1] * u[0])
    return u, v, n


@pytest.mark.parametrize("normal", [(0.0, 0.0, 1.0), (1.0, 1.0, 1.0),
                                    (1.0, -2.0, 0.5), (0.0, 1.0, 0.0),
                                    (-3.0, 0.25, 2.0)])
@pytest.mark.parametrize("theta", [math.radians(40.0), math.radians(200.0),
                                   -math.radians(75.0), -2.0 * math.pi - 0.3])
def test_a_normal_rotated_tilted_basis_is_a_genuine_circle(normal, theta):
    R = 37.5
    u, v, n = rotation_basis(normal)
    center = (10.0, -4.0, 7.0)
    arc = CircularArc3D(center, u, v, R, theta)
    # the stored basis is orthonormal
    assert math.hypot(*arc.u) == pytest.approx(1.0, abs=2.3e-16)
    assert math.hypot(*arc.v) == pytest.approx(1.0, abs=2.3e-16)
    assert sum(arc.u[i] * arc.v[i] for i in range(3)) == pytest.approx(0.0, abs=1e-15)
    assert math.hypot(*arc.normal) == pytest.approx(1.0, abs=1e-15)
    # every point is exactly R from the centre and in the plane
    pts = [arc.point_at_phi(theta * i / 40.0) for i in range(41)]
    for p in pts:
        q = tuple(p[i] - center[i] for i in range(3))
        assert math.sqrt(sum(c * c for c in q)) == pytest.approx(R, abs=1e-12)
        assert sum(q[i] * arc.normal[i] for i in range(3)) == pytest.approx(0.0,
                                                                          abs=1e-12)
    # the radius recomputed from three stored points is R (circumradius)
    A, B, C = pts[0], pts[13], pts[27]
    ab = math.dist(A, B)
    bc = math.dist(B, C)
    ca = math.dist(C, A)
    e1 = tuple(B[i] - A[i] for i in range(3))
    e2 = tuple(C[i] - A[i] for i in range(3))
    cross = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
             e1[0] * e2[1] - e1[1] * e2[0])
    area2 = math.sqrt(sum(c * c for c in cross))
    assert ab * bc * ca / (2.0 * area2) == pytest.approx(R, rel=1e-9)
    # length, curvature and min radius all describe the STORED circle
    assert arc.length == pytest.approx(R * abs(theta), rel=1e-15)
    assert arc.min_radius_um == R
    assert arc.curvature == pytest.approx(1.0 / R, rel=1e-15)
    for i in range(11):
        s = arc.length * i / 10.0
        t = arc.tangent_at(s)
        assert math.hypot(*t) == pytest.approx(1.0, abs=1e-12)
        q = tuple(arc.point_at(s)[j] - center[j] for j in range(3))
        assert sum(q[j] * t[j] for j in range(3)) == pytest.approx(0.0, abs=1e-9)
    # the analytic AABB agrees with dense sampling
    lo, hi = arc.aabb()
    for i in range(3):
        vals = [p[i] for p in pts]
        assert lo[i] <= min(vals) + 1e-9
        assert hi[i] >= max(vals) - 1e-9


@pytest.mark.parametrize("bad", [("zero", [0.0, 0.0, 0.0]), ("nan", [float("nan"), 0.0, 0.0]),
                                 ("inf", [float("inf"), 0.0, 0.0]),
                                 ("two", [1.0, 0.0]), ("four", [1.0, 0.0, 0.0, 0.0]),
                                 ("string", ["1.0", "0.0", "0.0"])])
def test_the_validator_rejects_malformed_basis_vectors(bad):
    _label, value = bad
    seg = arc_dict(NEAR_UNIT_U, NEAR_UNIT_V)
    seg["u"] = value
    with pytest.raises(ValueError):
        check_segment_dict(seg, "path.segments[0]")
    seg2 = arc_dict(NEAR_UNIT_U, NEAR_UNIT_V)
    seg2["v"] = value
    with pytest.raises(ValueError):
        check_segment_dict(seg2, "path.segments[0]")


def test_the_requested_radius_is_stored_exactly_and_never_rounded():
    """Documented policy: no rounding and no lattice snapping of the radius."""
    u, v, _n = rotation_basis((1.0, 1.0, 1.0))
    for R in (2000.0, 2000.3, 1999.999999, 1e-3, 12345.6789):
        arc = CircularArc3D((0.0, 0.0, 0.0), u, v, R, 0.9)
        assert arc.radius == R                     # bit for bit, no snapping
        assert arc.min_radius_um == R
        assert arc.curvature == 1.0 / R
        assert arc.length == R * 0.9
        # and the stored radius really is the circumradius of the stored basis
        for i in range(7):
            p = arc.point_at_phi(0.9 * i / 6.0)
            assert math.dist(p, arc.center) == pytest.approx(R, rel=1e-12)
    # an integral request is accepted and stays integral
    integral = CircularArc3D((0.0, 0.0, 0.0), u, v, 2000.0, math.pi / 2.0)
    assert integral.radius == 2000.0
    assert float(integral.radius).is_integer()


def test_to_dict_and_from_dict_are_lossless_for_a_legal_arc():
    u, v, _n = rotation_basis((1.0, -2.0, 0.5))
    arc = CircularArc3D((10.0, -4.0, 7.0), u, v, 37.5, -1.9)
    d = arc.to_dict()
    again = CircularArc3D.from_dict(d)
    assert again.center == arc.center
    assert again.u == arc.u
    assert again.v == arc.v
    assert again.radius == arc.radius
    assert again.theta == arc.theta
    assert again.aabb() == arc.aabb()
    assert again.normal == arc.normal
    assert again.length == arc.length
    # ... including through a real JSON round trip
    third = CircularArc3D.from_dict(json.loads(json.dumps(d)))
    assert (third.center, third.u, third.v, third.radius, third.theta) == \
        (arc.center, arc.u, arc.v, arc.radius, arc.theta)


def test_the_real_construction_paths_are_all_inside_the_tolerance():
    """Every documented construction path, with its MEASURED defect."""
    worst_unit = 0.0
    worst_orth = 0.0
    bases = []
    # from_pose: 8 tangents x 4 normals x both sweep signs
    tangents = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                (1.0, 1.0, 0.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0),
                (1.0, 1.0, 1.0), (-1.0, 2.0, -3.0)]
    normals = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, -1.0, 0.5)]
    for t in tangents:
        pose = Pose3D((3.0, -4.0, 5.0), t)
        for n in normals:
            dot = sum(t[i] * n[i] for i in range(3)) / math.hypot(*n)
            if abs(dot) > 1e-7:
                continue                      # from_pose itself rejects this
            for sweep in (math.radians(90.0), -math.radians(90.0)):
                bases.append(CircularArc3D.from_pose(pose, n, 50.0, sweep))
    # tilted_plane_arc for many normals and sweeps
    for normal in ((0, 0, 1), (1, 1, 1), (1, -2, 0.5), (0, 1, 0), (-1, 0, 0.25)):
        for theta in (math.radians(40.0), math.radians(200.0), -math.radians(75.0),
                      -4.0 * math.pi - 0.3):
            bases.append(tilted_plane_arc((3.0, -4.0, 5.0), normal, 30.0, theta))
    # from_dict of every arc that the two above produce
    for arc in list(bases):
        bases.append(CircularArc3D.from_dict(arc.to_dict()))
    assert len(bases) > 50
    for arc in bases:
        unit_defect, unit_hat_defect, orth = basis_defects(arc.u, arc.v)
        worst_unit = max(worst_unit, unit_defect, unit_hat_defect)
        worst_orth = max(worst_orth, orth)
        assert unit_defect <= BASIS_UNIT_DEFECT_TOL
        assert unit_hat_defect <= BASIS_UNIT_DEFECT_TOL
        assert orth <= BASIS_ORTHOGONALITY_TOL
        # the stored radius is the distance from the centre to the stored start
        start = arc.point_at_phi(0.0)
        assert abs(math.dist(start, arc.center) - arc.radius) <= 1e-9 * arc.radius
    # the measured margin (printed by the probe as well)
    assert worst_unit < 1e-14
    assert worst_orth < 1e-14


def test_a_legal_arc_radius_check_still_accepts_an_on_lattice_radius():
    """The repair must not break the production radius check for legal arcs."""
    u, v, _n = rotation_basis((0.0, 0.0, 1.0))
    arc = CircularArc3D((0.0, 0.0, 0.0), u, v, 2000.0, THETA)
    cfg = RouterConfig(rmin_um=2000.0, radius_tol_um=1e-6,
                       turn_radii_um=(2000.0,), connector_radii_um=(2000.0,))
    res = radius_check([arc], cfg)
    assert res.status == "safe"
    assert res.bound_um == 2000.0
    assert arc.min_radius_um == 2000.0
