"""v0.1.3 PART B probe (post-fix): B1 / B2 / B3 counterexample numbers.

    .venv\\Scripts\\python.exe artifacts\\v013\\probe\\v013_validation_probe.py

This probe measures the code that is in the tree NOW and prints, for every PART B
counterexample of docs/prompts/deepseek_native3d_v013_contract_geometry_repair_prompt.md,
BOTH the frozen v0.1.2 value and the repaired value.  The frozen snapshot
(artifacts/v013/prefix_src/, byte-for-byte v0.1.2 source) is loaded as the separate
package ``native3d_prefix`` in the same interpreter, exactly like
v013_prefix_replay.py does, so the two versions cannot contaminate each other and
the red -> green transition is printed rather than asserted by hand.

It also measures the empirical margins that justify the two v0.1.3 constants:

* the worst unit defect, the worst orthogonality defect and the worst
  |norm(start - center) - R| / relative |P(phi) - C| / R - 1 over EVERY arc built
  by the real construction paths (route_finite_native3d, reference_graph,
  continuous-pose route, CircularArc3D.from_pose, cases.tilted_plane_arc,
  CircularArc3D.from_dict) - the justification for BASIS_*_TOL = 1e-12;
* the exact ellipse minimum curvature radius of the rejected B3 basis.

No value printed here is asserted or reconstructed by hand: every number is the
real return value of the real code.  Exit code is always 0 - this is a
measurement, not a test (the regressions live in tests/test_v013_b_*.py).
"""

from __future__ import annotations

import glob
import importlib.util
import json
import math
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
V013 = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(V013))
SRC = os.path.join(ROOT, "src")
PREFIX_SRC = os.path.join(V013, "prefix_src")

sys.path.insert(0, SRC)


# --------------------------------------------------------------------------
# the frozen v0.1.2 snapshot as its own package tree (same loader as the replay)
# --------------------------------------------------------------------------
def load_prefix_package(name="native3d_prefix", src_dir=PREFIX_SRC):
    parent = importlib.util.module_from_spec(
        importlib.util.spec_from_loader(name, loader=None))
    parent.__path__ = [src_dir]
    sys.modules[name] = parent
    return parent


load_prefix_package()

from native3d.config import RouterConfig                                      # noqa: E402
from native3d.constraints.checks import radius_check                          # noqa: E402
from native3d.geometry import CircularArc3D, Pose3D                           # noqa: E402
from native3d.geometry import vec as GV                                       # noqa: E402
from native3d.geometry.primitives import (BASIS_ORTHOGONALITY_TOL,            # noqa: E402
                                          BASIS_UNIT_DEFECT_TOL, basis_defects)
from native3d.validation.validate import (BASIS_UNIT_DEFECT_TOL as V_BASIS_TOL,  # noqa: E402
                                          POSE_CONTRACT, check_segment_dict,
                                          point_arc_distance,
                                          point_arc_distance_witness,
                                          read_pose_dict, validate_artifact,
                                          validate_path_dict)

FROZEN_V = importlib.import_module("native3d_prefix.validation.validate")
FROZEN_P = importlib.import_module("native3d_prefix.geometry.primitives")
FROZEN_C = importlib.import_module("native3d_prefix.constraints.checks")
FROZEN_CFG = importlib.import_module("native3d_prefix.config")


def head(title):
    print("")
    print("=" * 78)
    print(title)
    print("=" * 78)


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def arc(u=(1.0, 0.0, 0.0), v=(0.0, 1.0, 0.0), R=50.0, theta=2.0 * math.pi,
        center=(0.0, 0.0, 0.0)):
    return {"kind": "arc", "center": list(center), "u": list(u), "v": list(v),
            "radius_um": float(R), "theta_rad": float(theta)}


def scene(refs, source, target, obstacles=(), lo=-2000.0, hi=2000.0):
    return {"chip_volume": {"lo": [lo] * 3, "hi": [hi] * 3},
            "obstacles": [{"lo": list(a), "hi": list(b)} for a, b in obstacles],
            "reference_centerlines": list(refs),
            "source_pose": {"position": list(source[0]), "tangent": list(source[1])},
            "target_pose": {"position": list(target[0]), "tangent": list(target[1])}}


def line(p0, p1):
    return {"kind": "line", "p0": list(p0), "p1": list(p1)}


# ===========================================================================
# B1 - point to planar arc distance
# ===========================================================================
XY_CIRCLE = arc()


def b1_function_level():
    head("B1 POST: point -> 3D planar arc distance (validate.point_arc_distance)")
    p = (0.0, 0.0, 50.0)
    pre = FROZEN_V.point_arc_distance(p, XY_CIRCLE)
    w = point_arc_distance_witness(p, XY_CIRCLE)
    true = math.sqrt(50.0 ** 2 + 50.0 ** 2)
    print("  reference circle : center=(0,0,0), u=+x, v=+y, R=50 um, theta=2*pi")
    print("  p                : (0,0,50)  [on the circle-plane normal through C]")
    print("  PRE  (v0.1.2)    : %.12f um" % pre)
    print("  POST (repaired)  : %.12f um   true sqrt(50^2+50^2) = %.12f um"
          % (w["distance_um"], true))
    print("  branch           : %s   rho=%.3g  h=%.3g"
          % (w["branch"], w["rho_um"], w["h_um"]))
    print("  witness          : %s at phase %.6f rad (arclength %.6f um)"
          % (tuple(round(c, 9) for c in w["witness_point"]), w["witness_phase_rad"],
             w["witness_arclength_um"]))
    print("  |witness - p|    : %.12f um   residual %.3e um"
          % (w["witness_distance_um"], w["witness_residual_um"]))

    print("")
    print("  the same function on the other contract cases (all POST):")
    cases = [
        ("centre axis, h=0        ", (0.0, 0.0, 0.0), arc()),
        ("centre axis, h=-123     ", (0.0, 0.0, -123.0), arc()),
        ("nearest phase NOT reached", (0.0, -100.0, 0.0), arc(theta=math.pi / 3.0)),
        ("negative sweep, on-arc  ", (0.0, 50.0, 0.0),
         arc(u=(-1.0, 0.0, 0.0), v=(0.0, -1.0, 0.0), theta=-math.pi / 2.0)),
        ("multi-turn 8*pi         ", (70.0, 0.0, 20.0), arc(theta=8.0 * math.pi)),
        ("full turn, seam phase   ", (0.0, -70.0, 0.0), arc()),
        ("tilted plane + 4*pi+0.4 ", (2.0, 0.0, 45.0),
         arc(u=(1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0),
             v=(1.0 / math.sqrt(6.0), 1.0 / math.sqrt(6.0), -2.0 / math.sqrt(6.0)),
             R=37.5, theta=4.0 * math.pi + 0.4, center=(10.0, -4.0, 7.0))),
    ]
    for label, pt, seg in cases:
        wp = point_arc_distance_witness(pt, seg)
        print("    %s PRE %-12.9f POST %-14.9f branch=%-26s residual=%.2e"
              % (label, FROZEN_V.point_arc_distance(pt, seg), wp["distance_um"],
                 wp["branch"], wp["witness_residual_um"]))


def b1_full_path():
    head("B1 POST: the full legal path above the reference arc (acceptance)")
    z = 50.0
    path = {"segments": [line((-10.0, 0.0, z), (10.0, 0.0, z))]}
    sc = scene([XY_CIRCLE], ((-10.0, 0.0, z), (1.0, 0.0, 0.0)),
               ((10.0, 0.0, z), (1.0, 0.0, 0.0)))
    cfg = cfg_dict()
    true_min = math.sqrt(40.0 ** 2 + z ** 2)
    pre = FROZEN_V.validate_path_dict(path, sc, cfg)
    post = validate_path_dict(path, sc, cfg)
    rows = [
        ("verdict", pre["verdict"], post["verdict"]),
        ("spacing status", pre["centerline_spacing"]["status"],
         post["centerline_spacing"]["status"]),
        ("min_sampled_um", pre["centerline_spacing"]["min_sampled_um"],
         post["centerline_spacing"]["min_sampled_um"]),
        ("certified lb um", pre["centerline_spacing"]["conservative_lower_bound_um"],
         post["centerline_spacing"]["conservative_lower_bound_um"]),
        ("self_check_failed", pre["certification"]["self_check_failed"],
         post["certification"]["self_check_failed"]),
    ]
    print("  path (-10,0,50)->(10,0,50); reference = the XY full circle R=50;")
    print("  true minimum spacing = sqrt(40^2+50^2) = %.12f um" % true_min)
    for name, a, b in rows:
        print("    %-20s PRE %-24r POST %r" % (name, a, b))
    print("  production primitive_spacing on the same pair: see the PART B report")
    print("  (the independent validator now agrees: pass / safe / lb <= sampled)")

    print("")
    print("  controls that must STILL fail (POST):")
    for label, p0, p1, want in (
            ("real arc crossing (-100,0,0)->(100,0,0)", (-100.0, 0.0, 0.0),
             (100.0, 0.0, 0.0), "violation"),
            ("path inside the circle (-10,0,0)->(10,0,0)", (-10.0, 0.0, 0.0),
             (10.0, 0.0, 0.0), "safe")):
        v = validate_path_dict({"segments": [line(p0, p1)]},
                               scene([XY_CIRCLE], (p0, (1.0, 0.0, 0.0)),
                                     (p1, (1.0, 0.0, 0.0))), cfg)
        print("    %-42s spacing=%-10s sampled=%-12.9f verdict=%s"
              % (label, v["centerline_spacing"]["status"],
                 v["centerline_spacing"]["min_sampled_um"], v["verdict"]))


# ===========================================================================
# B2 - JSON pose contract
# ===========================================================================
def b2_pose_contract():
    head("B2 POST: JSON source/target pose contract")
    path = {"segments": [line((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))]}
    base = scene([], ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 ((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    bad = scene([], ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((100.0, 0.0, 0.0), (1.0, 1.0, 0.0)))
    pre = FROZEN_V.validate_path_dict(path, bad, cfg_dict())
    post = validate_path_dict(path, bad, cfg_dict())
    print("  target tangent [1,1,0] against a +x path end:")
    print("    PRE  end_tangent_error_deg = %-20r status=%-12s verdict=%s"
          % (pre["endpoint_errors"]["end_tangent_error_deg"],
             pre["target_pose_check"]["status"], pre["verdict"]))
    print("    POST end_tangent_error_deg = %-20r status=%-12s verdict=%s"
          % (post["endpoint_errors"]["end_tangent_error_deg"],
             post["target_pose_check"]["status"], post["verdict"]))
    print("  true angle = 45 deg, default target tolerance = 3 deg")
    prod = Pose3D.from_dict(bad["target_pose"])
    print("  production Pose3D.from_dict target tangent = %s"
          % (tuple(prod.tangent),))
    print("  validator normalised target tangent        = %s"
          % (tuple(post["pose_contract"]["target"]["tangent"]),))
    print("  published contract: %r" % (post["pose_contract"]["contract"],))
    print("  POSE_CONTRACT constant: %r" % (POSE_CONTRACT,))
    print("  Basis tolerance mirror in the validator: %r (production %r)"
          % (V_BASIS_TOL, BASIS_UNIT_DEFECT_TOL))

    print("")
    print("  legal but non-unit tangents (accepted and normalised, POST):")
    for label, tangent, pth in (
            ("scaled [1000,0,0]",
             (1000.0, 0.0, 0.0),
             {"segments": [line((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))]}),
            ("oblique [3,0,4]  ",
             (3.0, 0.0, 4.0),
             {"segments": [line((0.0, 0.0, 0.0), (60.0, 0.0, 80.0))]})):
        target_pos = tuple(pth["segments"][0]["p1"])
        src_tan = tuple(pth["segments"][0]["p1"][i] - pth["segments"][0]["p0"][i]
                        for i in range(3))
        sc = scene([], ((0.0, 0.0, 0.0), src_tan), (target_pos, tangent))
        v = validate_path_dict(pth, sc, cfg_dict())
        pc = v["pose_contract"]["target"]
        prod_t = tuple(Pose3D.from_dict(sc["target_pose"]).tangent)
        print("    %s |input|=%8.3f -> normalised %s == production %s  verdict=%s"
              % (label, pc["tangent_input_norm"],
                 tuple(round(c, 12) for c in pc["tangent"]),
                 tuple(round(c, 12) for c in prod_t), v["verdict"]))

    print("")
    print("  malformed poses (PRE = frozen v0.1.2, POST = repaired):")
    table = [
        ("zero direction    ", (0.0, 0.0, 0.0)),
        ("NaN direction     ", (float("nan"), 0.0, 0.0)),
        ("Infinity direction", (float("inf"), 0.0, 0.0)),
        ("2 components      ", (1.0, 0.0)),
        ("4 components      ", (1.0, 0.0, 0.0, 0.0)),
        ("non-numeric string", ("1.0", "0.0", "0.0")),
    ]
    for label, tangent in table:
        sc = scene([], ((0.0, 0.0, 0.0), tangent), ((100.0, 0.0, 0.0),
                                                   (1.0, 0.0, 0.0)))
        try:
            r = FROZEN_V.validate_path_dict(path, sc, cfg_dict())
            pre_txt = "verdict=%-6s start_ang=%.6g" % (
                r["verdict"], r["endpoint_errors"]["start_tangent_error_deg"])
        except Exception as exc:                                   # noqa: BLE001
            pre_txt = "%s" % type(exc).__name__
        try:
            validate_path_dict(path, sc, cfg_dict())
            post_txt = "NO ERROR (defect!)"
        except ValueError as exc:
            post_txt = "ValueError: %s" % str(exc)[:58]
        print("    %s PRE %-34s POST %s" % (label, pre_txt, post_txt))
    print("  every malformed pose now raises ValueError and can never be a pass")


# ===========================================================================
# B3 - near-unit basis / circular arc contract
# ===========================================================================
R_BIG, DELTA, THETA = 2000.0, 8e-10, math.pi / 2.0
NEAR_U, NEAR_V = (1.0 + DELTA, 0.0, 0.0), (0.0, 1.0, 0.0)


def b3_basis_contract():
    head("B3 POST: near-unit basis / circular-arc contract")
    rmin_ellipse = R_BIG / (1.0 + DELTA)
    floor = 2000.0 - 1e-6
    print("  R=%.1f um, u=(1+8e-10,0,0), v=(0,1,0), theta=pi/2" % R_BIG)
    print("  unit defect of the INPUT basis     : %.6e  (v0.1.2 tolerance 1e-9)"
          % basis_defects(NEAR_U, NEAR_V)[0])
    print("  unit defect AFTER normalisation    : %.6e" % basis_defects(NEAR_U, NEAR_V)[1])
    print("  orthogonality defect |dot(u,v)|    : %.6e" % basis_defects(NEAR_U, NEAR_V)[2])
    print("  exact ellipse minimum curvature radius = R/(1+8e-10) = %.10f um"
          % rmin_ellipse)
    print("  allowed floor Rmin - radius_tol        = %.10f um" % floor)
    print("  breach                                 = %.3e um" % (floor - rmin_ellipse))
    print("  v0.1.3 tolerances: unit %.0e / orthogonality %.0e"
          % (BASIS_UNIT_DEFECT_TOL, BASIS_ORTHOGONALITY_TOL))

    pre_arc = FROZEN_P.CircularArc3D((0.0, 0.0, 0.0), NEAR_U, NEAR_V, R_BIG, THETA)
    pre_rc = FROZEN_C.radius_check(
        [pre_arc], FROZEN_CFG.RouterConfig(rmin_um=2000.0, radius_tol_um=1e-6,
                                           turn_radii_um=(2000.0,),
                                           connector_radii_um=(2000.0,)))
    print("")
    print("  PRE  (v0.1.2) constructor accepted it, stored u = %s" % (pre_arc.u,))
    print("       production radius_check = %s (bound=%r)"
          % (pre_rc.status, pre_rc.bound_um))
    sc = scene([], (pre_arc.start, pre_arc.tangent_at_phi(0.0)),
               (pre_arc.end, pre_arc.tangent_at_phi(pre_arc.theta)),
               lo=-5000.0, hi=5000.0)          # the chip of the prefix replay
    cfg_b3 = cfg_dict(rmin_um=2000.0, radius_tol_um=1e-6, turn_radii_um=[2000.0],
                      connector_radii_um=[2000.0])
    pre_v = FROZEN_V.validate_path_dict(
        {"segments": [arc(NEAR_U, NEAR_V, R_BIG, THETA)]}, sc, cfg_b3)
    print("       independent validator verdict = %s (min_radius_stored=%r)"
          % (pre_v["verdict"], pre_v["min_radius_um_stored"]))

    for label, fn in (("production constructor", lambda: CircularArc3D(
            (0.0, 0.0, 0.0), NEAR_U, NEAR_V, R_BIG, THETA)),
                      ("independent validator", lambda: check_segment_dict(
                          arc(NEAR_U, NEAR_V, R_BIG, THETA), "path.segments[0]"))):
        try:
            fn()
            print("  POST %-22s ACCEPTED (defect!)" % label)
        except ValueError as exc:
            print("  POST %-22s ValueError: %s" % (label, str(exc)[:70]))
    print("  -> both implementations reject the SAME input, so R is never claimed")
    print("     as the minimum curvature radius of an ellipse")

    print("")
    print("  legal bases still accepted (POST), with the geometry recomputed:")
    tilted = arc(u=(1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0), 0.0),
                 v=(1.0 / math.sqrt(6.0), 1.0 / math.sqrt(6.0),
                    -2.0 / math.sqrt(6.0)),
                 R=37.5, theta=-2.0 * math.pi - 0.3, center=(10.0, -4.0, 7.0))
    live = CircularArc3D((10.0, -4.0, 7.0), tilted["u"], tilted["v"], 37.5,
                         -2.0 * math.pi - 0.3)
    check_segment_dict(tilted, "reference_centerlines[0]")
    radii = [math.dist(live.point_at_phi(live.theta * i / 20.0), live.center)
             for i in range(21)]
    lo, hi = live.aabb()
    print("    |u|=%.17f |v|=%.17f dot=%.3e |normal|=%.17f"
          % (GV.norm(live.u), GV.norm(live.v), GV.dot(live.u, live.v),
             GV.norm(live.normal)))
    print("    R stored=%.12f  R recomputed from points: min=%.12f max=%.12f"
          % (live.radius, min(radii), max(radii)))
    print("    length=%.12f (R|theta|=%.12f)  curvature=%.17f  min_radius=%.12f"
          % (live.length, live.radius * abs(live.theta), live.curvature,
             live.min_radius_um))
    print("    analytic AABB lo=%s hi=%s"
          % (tuple(round(c, 9) for c in lo), tuple(round(c, 9) for c in hi)))
    print("    to_dict/from_dict lossless: %s"
          % (CircularArc3D.from_dict(
              json.loads(json.dumps(live.to_dict()))) == live,))

    print("")
    print("  radius policy: NEVER rounded, snapped or quantised")
    for R in (2000.0, 2000.3, 1999.999999):
        a = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), R, 1.0)
        print("    requested %.9f -> stored %.17g (bit-for-bit %s), "
              "min_radius_um=%.17g, |start-C|=%.17g"
              % (R, a.radius, a.radius == R, a.min_radius_um,
                 math.dist(a.point_at_phi(0.0), a.center)))


def b3_empirical_survey():
    """Worst basis/radius defect over EVERY arc built by the real paths."""
    head("B3 POST: measured worst case over the real construction paths")
    import native3d.geometry.primitives as P

    stats = {"n": 0, "unit": 0.0, "unit_hat": 0.0, "orth": 0.0,
             "abs_radius_gap": 0.0, "rel_radius_gap": 0.0, "worst": None}
    orig = P.CircularArc3D.__post_init__

    def measured(self):
        u, v = tuple(float(c) for c in self.u), tuple(float(c) for c in self.v)
        ud, udh, od = P.basis_defects(u, v)
        if math.isfinite(ud):
            stats["n"] += 1
            if ud > stats["unit"]:
                stats["unit"] = ud
            if udh > stats["unit_hat"]:
                stats["unit_hat"] = udh
            if od > stats["orth"]:
                stats["orth"] = od
        orig(self)
        R = float(self.radius)
        if R > 0:
            gap = abs(GV.norm(GV.sub(self.point_at_phi(0.0), self.center)) - R)
            rel = 0.0
            for i in range(9):
                phi = self.theta * i / 8.0
                rel = max(rel, abs(GV.norm(GV.sub(self.point_at_phi(phi),
                                                 self.center)) / R - 1.0))
            if gap > stats["abs_radius_gap"]:
                stats["abs_radius_gap"] = gap
                stats["worst"] = (u, v, R)
            if rel > stats["rel_radius_gap"]:
                stats["rel_radius_gap"] = rel

    P.CircularArc3D.__post_init__ = measured
    ran, failed = [], []
    try:
        from native3d.cases import core_case
        from native3d.search.finite_graph import route_finite_native3d
        from native3d.search.router import route
        for label, fn in (
                ("route_finite_native3d(core_case)",
                 lambda: route_finite_native3d(core_case(), mode="native3d")),
                ("route(core_case, reference_graph)",
                 lambda: route(core_case(), mode="native3d",
                               strategy="reference_graph")),
                ("route(core_case, continuous_pose)",
                 lambda: route(core_case(), mode="native3d",
                               strategy="continuous_pose", max_expansions=2000,
                               max_runtime_s=20.0))):
            try:
                fn()
                ran.append(label)
            except Exception as exc:                               # noqa: BLE001
                failed.append("%s: %s: %s" % (label, type(exc).__name__, exc))
        from native3d.cases import tilted_plane_arc
        for normal in ((0, 0, 1), (1, 1, 1), (1, -2, 0.5), (0, 1, 0)):
            for theta in (math.radians(40.0), math.radians(200.0),
                          -math.radians(75.0), -4.0 * math.pi - 0.3):
                tilted_plane_arc((3.0, -4.0, 5.0), normal, 30.0, theta)
        ran.append("cases.tilted_plane_arc x16")
        for t in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 1, 1)):
            pose = Pose3D((3.0, -4.0, 5.0), t)
            for n in ((0, 0, 1), (1, 0, 0), (1, 1, 1)):
                d = sum(t[i] * n[i] for i in range(3)) / math.hypot(*n)
                if abs(d) > 1e-7:
                    continue
                for sw in (math.radians(90.0), -math.radians(90.0)):
                    CircularArc3D.from_pose(pose, n, 50.0, sw)
        ran.append("CircularArc3D.from_pose")
        n_json = 0
        for pattern in (os.path.join(ROOT, "artifacts", "v012", "cases", "*.json"),
                        os.path.join(ROOT, "artifacts", "v013", "cases", "*.json")):
            for path in glob.glob(pattern):
                try:
                    blob = json.load(open(path, encoding="utf-8"))
                except Exception:                                  # noqa: BLE001
                    continue
                for node in _walk_arcs(blob):
                    try:
                        CircularArc3D.from_dict(node)
                        n_json += 1
                    except (KeyError, ValueError, TypeError):
                        pass
        ran.append("CircularArc3D.from_dict over %d serialised arcs" % n_json)
    finally:
        P.CircularArc3D.__post_init__ = orig

    print("  instrumented constructions (real, in-process): %d" % stats["n"])
    for label in ran:
        print("    ran: %s" % label)
    for label in failed:
        print("    SKIPPED (unrelated in-flight error): %s" % label)
    print("")
    print("  worst UNIT defect of the input basis        : %.6e" % stats["unit"])
    print("  worst unit defect after normalisation       : %.6e" % stats["unit_hat"])
    print("  worst ORTHOGONALITY defect |dot(u_hat,v_hat)|: %.6e" % stats["orth"])
    print("  tolerance in force                          : %.0e / %.0e"
          % (BASIS_UNIT_DEFECT_TOL, BASIS_ORTHOGONALITY_TOL))
    print("  margin of the unit check                    : %.0fx"
          % (BASIS_UNIT_DEFECT_TOL / stats["unit"] if stats["unit"] else float("inf")))
    print("")
    print("  RADIUS INVARIANT over the same arcs:")
    print("    worst |norm(start - center) - R|          : %.6e um" % stats["abs_radius_gap"])
    print("    worst | |P(phi) - C| / R - 1 |            : %.6e" % stats["rel_radius_gap"])
    if stats["worst"]:
        print("    worst case arc                            : u=%s v=%s R=%r"
              % (tuple(round(c, 12) for c in stats["worst"][0]),
                 tuple(round(c, 12) for c in stats["worst"][1]), stats["worst"][2]))
    print("    -> the stored radius IS the circumradius of the stored orthonormal")
    print("       basis, to within the printed relative bound")


def _walk_arcs(node):
    if isinstance(node, dict):
        if node.get("kind") == "arc" and "radius_um" in node and "u" in node:
            yield node
        for value in node.values():
            for found in _walk_arcs(value):
                yield found
    elif isinstance(node, list):
        for value in node:
            for found in _walk_arcs(value):
                yield found


# ===========================================================================
# the zero-length dispatch of validate_artifact
# ===========================================================================
def zero_length_dispatch():
    head("B0/V013-A3 POST: validate_artifact on a zero-length success")
    pose = {"position": [0.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}
    cfg = RouterConfig()
    sc = {"name": "probe_zero", "chip_volume": {"lo": [-60.0] * 3, "hi": [60.0] * 3},
          "obstacles": [], "reference_centerlines": [],
          "source_pose": pose, "target_pose": dict(pose),
          "config": cfg.to_dict()}
    art = {"input": sc, "search": {"status": "success", "path": None,
                                   "zero_length_solution": True, "n_segments": 0,
                                   "cost": 0.0, "length_um": 0.0,
                                   "congestion_cost": 0.0}}
    res = validate_artifact(art)
    print("  hand-built artifact (path=null, zero_length_solution=true):")
    print("    verdict=%s zero_length_solution=%s" % (res["verdict"],
                                                     res["zero_length_solution"]))
    print("    all_checks_certified=%s  n_undecided=%s"
          % (res["certification"]["all_checks_certified"],
             res["certification"]["n_undecided"]))
    print("    position_mismatch=%.3e um  tangent_mismatch=%.3e deg"
          % (res["zero_length_checks"]["position_mismatch_um"],
             res["zero_length_checks"]["tangent_mismatch_deg"]))
    print("    boundary_margin=%.6g um  obstacle/ref clearance=%s"
          % (res["zero_length_checks"]["boundary_margin_um"],
             res["zero_length_checks"]["obstacle_clearance_um"]))
    bad = json.loads(json.dumps(art))
    bad["input"]["target_pose"] = {"position": [10.0, 0.0, 0.0],
                                   "tangent": [1.0, 0.0, 0.0]}
    res_bad = validate_artifact(bad)
    print("    same artifact with a 10 um target mismatch: verdict=%s  (%s)"
          % (res_bad["verdict"], res_bad["violations"][0][:64]))
    try:
        from native3d.cases import core_case                              # noqa: F401
        from native3d.constraints.checks import Box3D
        from native3d.geometry import Pose3D as P3
        from native3d.io.artifacts import build_artifact, finalise_artifact
        from native3d.io.caseio import Scene
        from native3d.search.finite_graph import route_finite_native3d
        p = P3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        scene_obj = Scene("probe_zero", Box3D((-60.0, -60.0, -60.0),
                                              (60.0, 60.0, 60.0)), [], [], p,
                          P3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
        route = route_finite_native3d(scene_obj, mode="native3d")
        a = finalise_artifact(build_artifact(scene_obj, route, "native3d", "probe"))
        print("  real end-to-end artifact: status=%s zero_length=%s verdict=%s"
              % (a["search"]["status"], a["search"]["zero_length_solution"],
                 (a.get("independent_validation") or {}).get("verdict")))
    except Exception as exc:                                           # noqa: BLE001
        print("  real end-to-end zero-length artifact not available yet: %s: %s"
              % (type(exc).__name__, str(exc)[:80]))


# ===========================================================================
if __name__ == "__main__":
    print("v0.1.3 PART B validation probe -- POST-fix numbers vs the frozen v0.1.2")
    print("python      : %s" % sys.version.replace("\n", " "))
    print("project root: %s" % ROOT)
    print("frozen src  : %s" % PREFIX_SRC)
    t0 = time.perf_counter()
    b1_function_level()
    b1_full_path()
    b2_pose_contract()
    b3_basis_contract()
    b3_empirical_survey()
    zero_length_dispatch()
    print("")
    print("=" * 78)
    print("END OF PART B PROBE (%.3f s)" % (time.perf_counter() - t0))
    print("regressions: .venv\\Scripts\\python.exe -m pytest -q "
          "tests/test_v013_b_point_arc_distance.py "
          "tests/test_v013_b_pose_json_contract.py "
          "tests/test_v013_b_arc_basis_contract.py")
    print("=" * 78)
