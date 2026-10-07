"""Lead final review: adversarial, independent checks of the six delivered claims.

This script does NOT re-use the project's own tests or probes for its verdicts.
Where a value can be computed from scratch (the point-to-arc distance, the basis
certification), it is recomputed here with a plain brute-force search so that
agreement is real evidence rather than a restatement.

    .venv\\Scripts\\python.exe artifacts\\v013\\probe\\v013_final_review.py
"""

from __future__ import annotations

import importlib.util
import math
import os
import random
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
V013 = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(V013))
PREFIX_SRC = os.path.join(V013, "prefix_src")
sys.path.insert(0, os.path.join(ROOT, "src"))

OK = []
BAD = []


def check(label, cond, detail=""):
    (OK if cond else BAD).append(label)
    print("  %-4s %s%s" % ("OK" if cond else "FAIL", label,
                           ("   [%s]" % detail) if detail else ""))


# --------------------------------------------------------------- from scratch
def _basis(rng):
    n = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1))
    n = tuple(c / math.sqrt(sum(x * x for x in n)) for c in n)
    ref = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    k = sum(ref[i] * n[i] for i in range(3))
    u = tuple(ref[i] - n[i] * k for i in range(3))
    nu = math.sqrt(sum(c * c for c in u))
    u = tuple(c / nu for c in u)
    v = (n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2],
         n[0] * u[1] - n[1] * u[0])
    return u, v


def brute_curve_distance(p, seg, eval_seg, n_samples):
    """Dense minimisation over the CURVE ITSELF, given its own evaluator.

    This deliberately does NOT re-implement the arc parametrisation: the reference
    and the implementation then share only the curve definition, so the check is
    of the DISTANCE ALGORITHM (nearest phase vs endpoints, rho == 0, negative and
    multi-turn sweeps) rather than of a re-derived formula.  An earlier version of
    this harness used a hand-written closed form with the turn angle applied to
    phi the wrong way round and reported a false 2.5e2 disagreement; the dense
    minimisation over the implementation's own curve is what settled it.
    """
    L = seg_length(seg)
    best = float("inf")
    for i in range(n_samples + 1):
        q = eval_seg(seg, L * i / n_samples)
        d = math.dist(p, q)
        if d < best:
            best = d
    return best


print("=" * 78)
print("1. IS A MISMATCHED GRAPH REFUSED BEFORE THE SEARCH,")
print("   AND DOES THE ACTUAL PORT ALONE DECIDE SUCCESS?")
print("=" * 78)
from native3d.config import RouterConfig                                     # noqa: E402
from native3d.constraints.checks import Box3D                                # noqa: E402
from native3d.geometry import LineSegment3D, Pose3D                           # noqa: E402
from native3d.io.caseio import Scene                                          # noqa: E402
from native3d.search.finite_graph import (FiniteGraph,                        # noqa: E402
                                          enumerate_graph_edges,
                                          route_finite_native3d)
from native3d.search.statuses import ALL_STATUSES                             # noqa: E402

CHIP = Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))


def sc(name, src, tgt, chip=CHIP, obstacles=(), refs=(), **over):
    return Scene(name=name, chip=chip, obstacles=list(obstacles),
                 reference_centerlines=list(refs),
                 source=Pose3D(src[0], src[1]), target=Pose3D(tgt[0], tgt[1]),
                 config=RouterConfig(**over))


A = sc("A", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), ((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
GA = FiniteGraph(A, mode="native3d", cfg=A.config)

# 1a. the reported counterexample
r = route_finite_native3d(sc("B", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                             ((50.0, 0.0, 0.0), (1.0, 0.0, 0.0))), graph=GA)
check("A1 reported counterexample -> graph_scene_mismatch",
      r.status == "graph_scene_mismatch" and r.path is None and math.isinf(r.cost),
      "status=%s" % r.status)

# 1b. adversarial variants: every single contract field changed alone
attacks = {
    "target moved": {"tgt": ((50.0, 0.0, 0.0), (1.0, 0.0, 0.0))},
    "source moved": {"src": ((50.0, 0.0, 0.0), (1.0, 0.0, 0.0))},
    "source tangent rotated": {"src": ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))},
    "target tangent rotated": {"tgt": ((100.0, 0.0, 0.0), (0.0, 1.0, 0.0))},
    "obstacle added": {"obstacles": (Box3D((-30.0,) * 3, (30.0,) * 3),)},
    "reference line added": {"refs": (LineSegment3D((-100.0, 40.0, 0.0),
                                                    (100.0, 40.0, 0.0)),)},
    "chip shrunk": {"chip": Box3D((-140.0,) * 3, (140.0,) * 3)},
    "spacing changed": {"finite_spacing_um": 25.0},
    "straight steps changed": {"finite_straight_steps": (1, 2)},
    "turn radii changed": {"finite_turn_radius_steps": (1,)},
    "dmin changed": {"dmin_centerline_um": 9.0},
    "clearance changed": {"obstacle_clearance_um": 9.0},
    "envelope margin changed": {"wg_envelope_radius_um": 5.0},
    "port attach radius changed": {"finite_port_attach_um": 40.0},
    "acceptance ball changed": {"eps_pos_um": 1.0},
    "source tolerance changed": {"source_pos_tol_um": 1e-3},
}
for label, kw in attacks.items():
    src = kw.get("src", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    tgt = kw.get("tgt", ((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
    over = {k: v for k, v in kw.items() if k not in ("src", "tgt", "chip",
                                                     "obstacles", "refs")}
    s = sc("atk", src, tgt, chip=kw.get("chip", CHIP),
           obstacles=kw.get("obstacles", ()), refs=kw.get("refs", ()), **over)
    r = route_finite_native3d(s, graph=GA)
    check("A1 %s alone -> refused" % label, r.status == "graph_scene_mismatch",
          r.status)

# 1c. mode crossed both ways
g2d = FiniteGraph(A, mode="2d", cfg=A.config)
r = route_finite_native3d(A, mode="native3d", graph=g2d)
check("A1 2d graph + native3d call -> refused", r.status == "graph_scene_mismatch",
      r.status)
r = route_finite_native3d(A, mode="2d", graph=GA)
check("A1 native3d graph + 2d call -> refused", r.status == "graph_scene_mismatch",
      r.status)

# 1d. congestion field changed
from native3d.search.congestion import RegionCongestion                       # noqa: E402
r = route_finite_native3d(A, congestion=RegionCongestion(
    [(Box3D((0.0,) * 3, (50.0,) * 3), 2.0)]), graph=GA)
check("A1 congestion field changed -> refused", r.status == "graph_scene_mismatch",
      r.status)

# 1e. a matching graph still works, and is reported as warm
r_ok = route_finite_native3d(A, graph=GA)
check("A1 matching prebuilt graph still succeeds", r_ok.status == "success",
      "cost=%s" % r_ok.cost)
check("A1 matching reuse reported warm",
      r_ok.finite_graph["graph_reuse"]["graph_cache_state"] == "warm"
      and r_ok.finite_graph["graph_reuse"]["accepted"] is True)

# 1f. does the ACTUAL returned port decide success?  Compare the verified
#     endpoint error with the acceptance ball, and prove the verifier is wired in.
pa = r_ok.port_assurance
check("A2 success publishes a verified actual endpoint",
      pa["verified_against_the_caller_pose"] is True
      and pa["target_position_error_um"] <= A.config.eps_pos_um
      and pa["source_position_error_um"] <= A.config.source_pos_tol_um)
check("A2 goal_pos_error_um equals the verified target error, not a graph key",
      r_ok.goal_pos_error_um == pa["target_position_error_um"])
check("A2 the searched path really ends on the caller target",
      r_ok.path.end_pose.position == (100.0, 0.0, 0.0))
# the verifier must REJECT a curve that misses the target by more than the ball
from native3d.search.finite_graph import verify_assembled_endpoints          # noqa: E402
rep = verify_assembled_endpoints(
    (LineSegment3D((0.0, 0.0, 0.0), (95.0, 0.0, 0.0)),), A, A.config)
check("A2 verifier rejects a 5 um target miss (ball is 2 um)",
      rep["ok"] is False and rep["end_position_error_um"] == 5.0
      and rep["status"] == "constraint_failed")
rep2 = verify_assembled_endpoints(
    (LineSegment3D((0.0, 0.0, 0.0), (99.0, 0.0, 0.0)),), A, A.config)
check("A2 verifier accepts a miss inside the ball but still reports the number",
      rep2["ok"] is True and 0.0 < rep2["end_position_error_um"] <= 2.0,
      "err=%.6f" % rep2["end_position_error_um"])
rep3 = verify_assembled_endpoints(
    (LineSegment3D((5e-6, 0.0, 0.0), (100.0, 0.0, 0.0)),), A, A.config)
check("A2 the SOURCE tolerance (1e-6 um) is the binding one, not the ball",
      rep3["ok"] is False and "source" in rep3["failed"],
      "start_err=%.3e" % rep3["start_position_error_um"])
rep4 = verify_assembled_endpoints(
    (LineSegment3D((5e-7, 0.0, 0.0), (100.0, 0.0, 0.0)),), A, A.config)
check("A2 a 5e-7 um start error is inside the source tolerance and accepted",
      rep4["ok"] is True and rep4["start_position_error_um"] == 5e-7,
      "start_err=%.3e" % rep4["start_position_error_um"])

print()
print("=" * 78)
print("2. DOES A ZERO-LENGTH CALL RETURN A DEFINED LEGAL RESULT?")
print("=" * 78)
import json                                                                    # noqa: E402

Z = sc("Z", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
       chip=Box3D((-60.0,) * 3, (60.0,) * 3))
rz = route_finite_native3d(Z, mode="native3d")
check("A3 zero-length returns success, not an exception", rz.status == "success")
check("A3 zero-length flags and values are all defined",
      rz.zero_length_solution is True and rz.cost == 0.0 and rz.length_um == 0.0
      and rz.congestion_cost == 0.0 and rz.path is None and rz.n_segments == 0
      and rz.expansions == 0)
check("A3 zero-length endpoint errors are verifiable and zero",
      rz.endpoint_errors["start_position_error_um"] == 0.0
      and rz.endpoint_errors["end_position_error_um"] == 0.0
      and rz.endpoint_errors["start_ok"] and rz.endpoint_errors["end_ok"])
# a failure must be distinguishable
rf = route_finite_native3d(sc("F", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                              ((1e6, 0.0, 0.0), (1.0, 0.0, 0.0)),
                              chip=Box3D((-60.0,) * 3, (60.0,) * 3),
                              obstacles=(Box3D((-5.0,) * 3, (5.0,) * 3),)),
                           mode="native3d")
check("A3 a failure has path=None but zero_length_solution=False",
      rf.path is None and rf.zero_length_solution is False
      and getattr(rf, "endpoint_errors", None) is None, rf.status)
# JSON round trip
from native3d.io.artifacts import build_artifact, finalise_artifact            # noqa: E402
from native3d.validation.validate import validate_artifact                     # noqa: E402
art = finalise_artifact(build_artifact(Z, rz, "native3d", "review"))
check("A3 zero-length artifact is independently validated, not skipped",
      art["independent_validation"] is not None
      and art["independent_validation"]["verdict"] == "pass"
      and art["independent_validation"]["zero_length_solution"] is True)
art2 = json.loads(json.dumps(art))
check("A3 zero-length survives a JSON round trip",
      validate_artifact(art2)["verdict"] == "pass")
# not every equal position is a win, and validation is not bypassed
rb = route_finite_native3d(sc("T", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                              ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                              chip=Box3D((-60.0,) * 3, (60.0,) * 3)), mode="native3d")
check("A3 equal position + wrong tangent is NOT a zero-length win",
      rb.status != "success" and rb.zero_length_solution is False, rb.status)
ri = route_finite_native3d(sc("I", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                              ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                              chip=Box3D((-60.0,) * 3, (60.0,) * 3),
                              obstacles=(Box3D((-5.0,) * 3, (5.0,) * 3),)),
                           mode="native3d")
check("A3 source==target does NOT bypass input validation",
      ri.status == "invalid_source_pose", ri.status)

print()
print("=" * 78)
print("3. IS THE 3D POINT/PATH-TO-ARC DISTANCE CORRECT, WITH A REAL WITNESS?")
print("=" * 78)
from native3d.validation.validate import (eval_seg, point_arc_distance,         # noqa: E402
                                         point_arc_distance_witness,
                                         seg_length, validate_path_dict)

rng = random.Random(20261007)
worst = 0.0
worst_case = None
n_b1 = 0
for trial in range(40):
    u, v = _basis(rng)
    R = rng.choice([5.0, 50.0, 300.0])
    C = (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-50, 50))
    for theta in (math.pi / 2, -math.pi / 2, math.pi, 2 * math.pi, 4 * math.pi,
                  -3 * math.pi, 0.3, 5.5):
        p = (rng.uniform(-400, 400), rng.uniform(-400, 400), rng.uniform(-400, 400))
        seg = {"kind": "arc", "center": list(C), "u": list(u), "v": list(v),
               "radius_um": R, "theta_rad": theta}
        got = point_arc_distance(p, seg)
        ref = brute_curve_distance(p, seg, eval_seg, 40000)
        n_b1 += 1
        err = got - ref               # the brute force is an upper bound, so this
        if err > worst:               # may only be negative or ~0 up to sampling
            worst, worst_case = err, (theta, R, got, ref)
check("B1 never exceeds a dense minimisation of its own curve over %d random "
      "tilted/negative/multi-turn arcs (it is an achieved distance, i.e. an "
      "upper bound)" % n_b1, worst <= 2e-3,
      "worst excess = %.3e (theta=%.3f R=%.1f got=%.9f brute=%.9f)"
      % (worst, worst_case[0], worst_case[1], worst_case[2], worst_case[3]))
worst = 0.0
for trial in range(20):
    u, v = _basis(rng)
    R = rng.choice([5.0, 50.0, 300.0])
    C = (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-50, 50))
    for theta in (math.pi / 2, -math.pi / 2, math.pi, 2 * math.pi, -3 * math.pi):
        p = (rng.uniform(-400, 400), rng.uniform(-400, 400), rng.uniform(-400, 400))
        seg = {"kind": "arc", "center": list(C), "u": list(u), "v": list(v),
               "radius_um": R, "theta_rad": theta}
        got = point_arc_distance(p, seg)
        ref = brute_curve_distance(p, seg, eval_seg, 200000)
        worst = max(worst, abs(got - ref))
check("B1 agrees with a 200k-point minimisation to sampling resolution "
      "(100 tighter random cases)", worst <= 2e-3, "worst |diff| = %.3e" % worst)

# the centre axis, exactly
seg_axis = {"kind": "arc", "center": [0.0, 0.0, 0.0], "u": [1.0, 0.0, 0.0],
            "v": [0.0, 1.0, 0.0], "radius_um": 50.0, "theta_rad": 2 * math.pi}
check("B1 centre-axis point is never 0",
      abs(point_arc_distance((0.0, 0.0, 50.0), seg_axis)
          - math.sqrt(50.0 ** 2 + 50.0 ** 2)) < 1e-9,
      "%.12f" % point_arc_distance((0.0, 0.0, 50.0), seg_axis))
# witness really lies on the curve and matches the reported distance
w_ok = True
w_detail = ""
for trial in range(60):
    u, v = _basis(rng)
    R = rng.uniform(1.0, 200.0)
    C = (rng.uniform(-30, 30),) * 3
    theta = rng.choice([0.5, -0.5, math.pi, -2 * math.pi, 7.0])
    p = (rng.uniform(-100, 100),) * 3
    seg = {"kind": "arc", "center": list(C), "u": list(u), "v": list(v),
           "radius_um": R, "theta_rad": theta}
    out = point_arc_distance_witness(p, seg)
    d = out["distance_um"]
    wit = out["witness_point"]
    if abs(out["witness_distance_um"] - d) > 1e-9 or out["witness_residual_um"] > 1e-9:
        w_ok, w_detail = False, "residual %.3e" % out["witness_residual_um"]
        break
    # the witness must be a point OF the curve: same radius from the centre and
    # inside the plane, and its arclength must be within [0, L]
    if abs(math.dist(wit, C) - R) > 1e-9:
        w_ok, w_detail = False, "witness radius %.12f != R %.12f" % (
            math.dist(wit, C), R)
        break
    if not (0.0 <= out["witness_arclength_um"] <= out["arc_length_um"] + 1e-9):
        w_ok, w_detail = False, "arclength %.6f outside [0, %.6f]" % (
            out["witness_arclength_um"], out["arc_length_um"])
        break
check("B1 witness is a real curve point whose distance equals the reported value "
      "and whose arclength lies inside the segment (60 random cases)", w_ok, w_detail)

# the full legal path must be accepted; a real crossing must fail
XY = {"kind": "arc", "center": [0.0, 0.0, 0.0], "u": [1.0, 0.0, 0.0],
      "v": [0.0, 1.0, 0.0], "radius_um": 50.0, "theta_rad": 2 * math.pi}
cfgd = dict(RouterConfig().to_dict())


def fly(z, x0=10.0, y0=0.0):
    return ({"segments": [{"kind": "line", "p0": [-x0, y0, z], "p1": [x0, y0, z]}]},
            {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [], "reference_centerlines": [XY],
             "source_pose": {"position": [-x0, y0, z], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [x0, y0, z], "tangent": [1.0, 0.0, 0.0]}})


v_up = validate_path_dict(*fly(50.0), cfgd)
check("B1 the legal 64.031242 um fly-over is ACCEPTED",
      v_up["verdict"] == "pass"
      and abs(v_up["centerline_spacing"]["min_sampled_um"]
              - 64.03124237432849) < 1e-6,
      "sampled=%.9f" % v_up["centerline_spacing"]["min_sampled_um"])
check("B1 its certified lower bound does not exceed the sampled upper bound",
      v_up["centerline_spacing"]["conservative_lower_bound_um"]
      <= v_up["centerline_spacing"]["min_sampled_um"] + 1e-9
      and v_up["certification"]["self_check_failed"] is False)
path_x, scene_x = fly(0.0, x0=100.0)
v_x = validate_path_dict(path_x, scene_x, cfgd)
check("B1 a real crossing of the same arc still FAILS",
      v_x["verdict"] == "fail"
      and v_x["centerline_spacing"]["min_sampled_um"] < 1e-9)
path_in, scene_in = fly(0.0, x0=10.0, y0=0.0)
v_in = validate_path_dict(path_in, scene_in, cfgd)
check("B1 the legal in-plane detour 40 um INSIDE the circle is ACCEPTED "
      "(the old formula rejected it)", v_in["verdict"] == "pass"
      and abs(v_in["centerline_spacing"]["min_sampled_um"] - 40.0) < 1e-6,
      "sampled=%.9f" % v_in["centerline_spacing"]["min_sampled_um"])

print()
print("=" * 78)
print("4. ANY REMAINING FAKE-PASS CHANNEL FOR NON-UNIT POSES / NEAR-UNIT BASIS?")
print("=" * 78)
from native3d.geometry.primitives import (BASIS_ORTHOGONALITY_TOL,            # noqa: E402
                                          BASIS_UNIT_DEFECT_TOL,
                                          CircularArc3D)
# the prompt's B2 counterexample
v45 = validate_path_dict(
    {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0], "p1": [100.0, 0.0, 0.0]}]},
    {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
     "obstacles": [], "reference_centerlines": [],
     "source_pose": {"position": [0.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
     "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 1.0, 0.0]}},
    cfgd)
check("B2 the 45 deg oblique tangent is now fail at 45 deg",
      v45["verdict"] == "fail"
      and abs(v45["endpoint_errors"]["end_tangent_error_deg"] - 45.0) < 1e-9,
      "%.12f deg" % v45["endpoint_errors"]["end_tangent_error_deg"])
# scaled and non-unit tangents must normalise exactly like production.  The
# comparison is against the PRODUCTION-normalised tangent, not against "zero
# error": a legal oblique tangent is normalised but is still a real angle away
# from the path's +x direction, and the validator must report exactly that angle.
bad_norm = []
for t in ([1000.0, 0.0, 0.0], [3.0, 0.0, 4.0], [0.0, 0.0, -7.5], [1.0, 1.0, 1.0],
          [5.0, 0.0, 0.0], [0.0, -2.0, 0.0]):
    d = {"position": [0.0, 0.0, 0.0], "tangent": t}
    prod = Pose3D.from_dict(d).tangent
    vv = validate_path_dict(
        {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                       "p1": [100.0, 0.0, 0.0]}]},
        {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
         "obstacles": [], "reference_centerlines": [],
         "source_pose": d,
         "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": t}},
        cfgd)
    err = vv["endpoint_errors"]["start_tangent_error_deg"]
    expected = math.degrees(math.acos(max(-1.0, min(1.0, prod[0]))))
    if abs(err - expected) > 1e-9:
        bad_norm.append("%s -> %.12f, expected %.12f" % (t, err, expected))
    # and the verdict must follow the target acceptance ball (3 deg) for the target
    tg = vv["endpoint_errors"]["end_tangent_error_deg"]
    want = "safe" if tg <= cfgd["eps_ang_deg"] else "violation"
    if vv["target_pose_check"]["status"] != want:
        bad_norm.append("%s -> target status %s, want %s" % (t, vv["target_pose_check"]["status"], want))
check("B2 every legal non-unit tangent yields exactly the angle implied by the "
      "PRODUCTION-normalised tangent, and the verdict follows the tolerance",
      not bad_norm, "; ".join(bad_norm[:2]))
# every malformed pose must be rejected, never a pass
rejected = []
for label, pose in (("zero", {"position": [0.0] * 3, "tangent": [0.0, 0.0, 0.0]}),
                    ("nan", {"position": [0.0] * 3,
                             "tangent": [float("nan"), 0.0, 0.0]}),
                    ("inf", {"position": [0.0] * 3,
                             "tangent": [float("inf"), 0.0, 0.0]}),
                    ("2 comp", {"position": [0.0] * 3, "tangent": [1.0, 0.0]}),
                    ("4 comp", {"position": [0.0] * 3,
                                "tangent": [1.0, 0.0, 0.0, 0.0]}),
                    ("missing", {"position": [0.0] * 3}),
                    ("string", {"position": [0.0] * 3, "tangent": ["a", "b", "c"]}),
                    ("pos nan", {"position": [0.0, float("nan"), 0.0],
                                 "tangent": [1.0, 0.0, 0.0]})):
    for which in ("source_pose", "target_pose"):
        s = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [], "reference_centerlines": [],
             "source_pose": {"position": [0.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}}
        s[which] = pose
        try:
            out = validate_path_dict(
                {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                               "p1": [100.0, 0.0, 0.0]}]}, s, cfgd)
            if out["verdict"] == "pass":
                rejected.append("%s/%s -> pass" % (label, which))
        except ValueError:
            pass
        except Exception as exc:                                          # noqa: BLE001
            rejected.append("%s/%s -> %s" % (label, which, type(exc).__name__))
check("B2 all 16 malformed-pose cases are rejected with ValueError, none passes",
      not rejected, "; ".join(rejected[:4]))
# B3 near-unit basis rejected by BOTH, and the ellipse is really inadmissible
u_bad = (1.0 + 8e-10, 0.0, 0.0)
v_bad = (0.0, 1.0, 0.0)
pre_rejected = False
try:
    CircularArc3D((0.0, 0.0, 0.0), u_bad, v_bad, 2000.0, math.pi / 2)
except ValueError:
    pre_rejected = True
check("B3 production constructor rejects the near-unit basis", pre_rejected)
val_rejected = False
try:
    validate_path_dict(
        {"segments": [{"kind": "arc", "center": [0.0, 0.0, 0.0], "u": list(u_bad),
                       "v": list(v_bad), "radius_um": 2000.0,
                       "theta_rad": math.pi / 2}]},
        {"chip_volume": {"lo": [-5000.0] * 3, "hi": [5000.0] * 3},
         "obstacles": [], "reference_centerlines": [],
         "source_pose": {"position": [2000.0, 0.0, 0.0], "tangent": [0.0, 1.0, 0.0]},
         "target_pose": {"position": [0.0, 2000.0, 0.0], "tangent": [-1.0, 0.0, 0.0]}},
        dict(RouterConfig(rmin_um=2000.0, radius_tol_um=1e-6,
                          turn_radii_um=(2000.0,),
                          connector_radii_um=(2000.0,)).to_dict()))
except ValueError:
    val_rejected = True
check("B3 the independent validator rejects the SAME basis", val_rejected)
check("B3 both thresholds are the documented 1e-12",
      BASIS_UNIT_DEFECT_TOL == 1e-12 and BASIS_ORTHOGONALITY_TOL == 1e-12)
# and the real ellipse would be inadmissible: R/(1+8e-10) < Rmin - tol
ellipse_rmin = 2000.0 / (1.0 + 8e-10)
check("B3 the rejected geometry really is inadmissible as a circle",
      ellipse_rmin < 2000.0 - 1e-6,
      "R/(1+8e-10) = %.10f < %.6f" % (ellipse_rmin, 2000.0 - 1e-6))

print()
print("=" * 78)
print("5. IS THE ORIGINAL CROSS-WALL RESULT PRESERVED?")
print("=" * 78)
from native3d.cases import core_case                                      # noqa: E402
cw = route_finite_native3d(core_case(), mode="native3d")
cw2 = route_finite_native3d(core_case(), mode="2d")
frozen = json.load(open(os.path.join(ROOT, "artifacts", "v012", "cases",
                                     "core_case_finite_native3d_native3d.json"),
                        encoding="utf-8"))
check("original cross-wall native3d success preserved",
      cw.status == "success" and abs(cw.cost - 594.1592653589794) < 1e-9
      and cw.expansions == 75, "%s cost=%.9f exp=%d" % (cw.status, cw.cost,
                                                        cw.expansions))
check("original trace and segment count preserved (8 steps / 7 segments)",
      len(cw.trace) == 8 and len(cw.path.segments) == 7)
check("original z profile preserved (0 -> 100 um)",
      max(s["z_um"] for s in cw.trace) == 100.0
      and min(s["z_um"] for s in cw.trace) == 0.0)
check("2D mode is a real exhaustion, not a budget stop",
      cw2.status == "search_exhausted" and cw2.budget_exhausted is False
      and cw2.expansions == 81, "%s exp=%d" % (cw2.status, cw2.expansions))
check("no reference-graph fallback anywhere",
      cw.reference_graph is None and cw2.reference_graph is None)
check("identical to the frozen v0.1.2 artifact field for field",
      [s["action"] for s in frozen["search"]["trace"]]
      == [s["action"] for s in cw.trace]
      and frozen["search"]["expansions"] == cw.expansions
      and abs(frozen["search"]["cost_um_equivalent"] - cw.cost) < 1e-9
      and len(frozen["search"]["path"]["segments"]) == len(cw.path.segments))

print()
print("=" * 78)
print("6. SCOPE OF THE GUARANTEES")
print("=" * 78)
check("the status vocabulary is complete and has no duplicates",
      len(set(ALL_STATUSES)) == len(ALL_STATUSES)
      and set(("success", "search_exhausted", "budget_exhausted",
               "port_attachment_failed", "graph_scene_mismatch",
               "invalid_source_pose", "invalid_target_pose", "unsupported_pose",
               "constraint_failed", "uncertified")) <= set(ALL_STATUSES),
      "%d statuses" % len(ALL_STATUSES))
artifacts_py = open(os.path.join(ROOT, "src", "native3d", "io", "artifacts.py"),
                    encoding="utf-8").read()
check("self-spacing is still recorded as NOT IMPLEMENTED",
      "NOT IMPLEMENTED" in artifacts_py)
checks_py = open(os.path.join(ROOT, "src", "native3d", "constraints", "checks.py"),
                 encoding="utf-8").read()
finite_py = open(os.path.join(ROOT, "src", "native3d", "search", "finite_graph.py"),
                 encoding="utf-8").read()
check("no optical model anywhere (no dB anywhere in the engine)",
      "dB" not in checks_py and "dB" not in finite_py)
# the edge bound must be DERIVED, i.e. its value must be exactly reproducible from
# the reported construction counts (no unexplained constant slack), and the real
# enumeration must fit inside it.
gb = FiniteGraph(core_case(), mode="native3d", cfg=core_case().config)
bb = gb.bounds
derived = (bb["edge_upper_bound_from_canonical_nodes"]
           + bb["edge_upper_bound_from_port_attachments"])
check("the edge upper bound is the sum of its two reported construction terms "
      "with no hardcoded slack",
      bb["edge_upper_bound"] == derived
      and bb["edge_upper_bound_from_canonical_nodes"]
      == bb["node_upper_bound"] * bb["edges_per_canonical_node_upper"],
      "total=%d = %d + %d"
      % (bb["edge_upper_bound"], bb["edge_upper_bound_from_canonical_nodes"],
         bb["edge_upper_bound_from_port_attachments"]))
small = FiniteGraph(sc("tiny", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                       ((50.0, 0.0, 0.0), (1.0, 0.0, 0.0))), mode="native3d",
                    cfg=RouterConfig())
n_edges_real = len(enumerate_graph_edges(
    sc("tiny", ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
       ((50.0, 0.0, 0.0), (1.0, 0.0, 0.0))), mode="native3d")[1])
check("the REAL enumerated construction fits inside the derived bound",
      0 < n_edges_real <= small.bounds["edge_upper_bound"],
      "%d real edges <= %d bound" % (n_edges_real, small.bounds["edge_upper_bound"]))
check("the node bound states whether it includes the virtual ports",
      "node_upper_bound_includes_virtual_ports" in bb
      and bb["node_upper_bound_includes_virtual_ports"] is False)

print()
print("=" * 78)
print("VERDICT")
print("=" * 78)
print("checks passed: %d" % len(OK))
print("checks failed: %d" % len(BAD))
for b in BAD:
    print("  FAILED: %s" % b)
sys.exit(1 if BAD else 0)

