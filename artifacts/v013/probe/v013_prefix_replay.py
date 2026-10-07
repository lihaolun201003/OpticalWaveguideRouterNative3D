"""v0.1.3 prefix (RED) replay: every reported counterexample, run on v0.1.2 code.

This probe answers ONE question honestly: what did the UNMODIFIED v0.1.2
implementation really return for each counterexample of
docs/prompts/deepseek_native3d_v013_contract_geometry_repair_prompt.md?

It does this by loading the frozen v0.1.2 source snapshot stored next to it
(artifacts/v013/prefix_src/, copied byte for byte from
OpticalWaveguideRouterNative3D/src/native3d before the first v0.1.3 edit) as a
SEPARATE package ``native3d_prefix`` inside the same interpreter, so both
versions run in one process and neither can contaminate the other:

    .venv\\Scripts\\python.exe artifacts\\v013\\probe\\v013_prefix_replay.py

Everything printed is the real output of the real code: no value here is
asserted or reconstructed by hand.

Raw numbers:

* importlib load of the frozen snapshot (no sitecustomize, no monkeypatching);
* the geometry of the counterexamples is rebuilt from the prompt's description;
* exit code 0 always (this is a measurement, not a test) - the caller records
  the exit code in the log file name.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
V013 = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(V013))
PREFIX_SRC = os.path.join(V013, "prefix_src")


# --------------------------------------------------------------------------
# load the frozen v0.1.2 snapshot as its own package tree
# --------------------------------------------------------------------------
def load_prefix_package(name: str = "native3d_prefix", src_dir: str = PREFIX_SRC):
    parent = importlib.util.module_from_spec(
        importlib.util.spec_from_loader(name, loader=None))
    parent.__path__ = [src_dir]
    sys.modules[name] = parent
    return parent


load_prefix_package()

from native3d_prefix.config import RouterConfig                          # noqa: E402
from native3d_prefix.constraints.checks import (Box3D,                   # noqa: E402
                                                primitive_spacing,
                                                radius_check)
from native3d_prefix.geometry.primitives import (CircularArc3D,          # noqa: E402
                                                 LineSegment3D, Pose3D)
from native3d_prefix.io.caseio import Scene                              # noqa: E402
from native3d_prefix.search.finite_graph import (FiniteGraph,            # noqa: E402
                                                 graph_config_fingerprint,
                                                 route_finite_native3d)
from native3d_prefix.validation.validate import (point_arc_distance,     # noqa: E402
                                                 validate_path_dict)


def head(title):
    print("")
    print("=" * 78)
    print(title)
    print("=" * 78)


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


# ======================================================================= A1
def a1_graph_injection_mismatch():
    head("A1 RED: mismatched prebuilt graph passed to route_finite_native3d")

    # The chip is the smallest one that keeps this counterexample faithful: h = 50
    # and margin 2 um give the lattice -2..2 per axis, so (0,0,0), (50,0,0) and
    # (100,0,0) are all lattice points, the SOURCE pose is valid (|0| < 148) so the
    # search really runs, the target is a DISTINCT lattice point from the source (a
    # fresh route therefore really costs 50 um and is not a zero-length solution),
    # and the target node's pose key still collides with the lattice node
    # (1, 0, 0, +x).  This is what makes the reported counterexample reproduce
    # exactly.  (A [-60,60]^3 chip cannot be used here: the source at (0,0,0) is
    # outside the volume shrunk by the 2 um boundary margin, so the run would stop
    # at invalid_source_pose before the defect could show; the earlier draft of this
    # probe did exactly that and is why the numbers below were verified twice.)
    chip = Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))

    def scene(target_x):
        return Scene(name="a1_target_%g" % target_x,
                     chip=chip,
                     obstacles=[], reference_centerlines=[],
                     source=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                     target=Pose3D((target_x, 0.0, 0.0), (1.0, 0.0, 0.0)),
                     config=RouterConfig())

    sc_a, sc_b = scene(100.0), scene(50.0)
    graph_a = FiniteGraph(sc_a, mode="native3d", cfg=sc_a.config)

    res = route_finite_native3d(sc_b, graph=graph_a)
    end = (list(res.path.end_pose.position) if res.path is not None else None)
    print("  scene A target = (100,0,0), scene B target = (50,0,0), tangent = +x")
    print("  chip = [-150,150]^3 (h = 50 um), lattice positions -2..2 per axis")
    print("  graph = FiniteGraph(scene_A);  result = route_finite_native3d(scene_B, graph=graph)")
    print("  graph.source_identity_node = %s" % (graph_a.source_identity_node,))
    print("  graph.target_identity_node = %s" % (graph_a.target_identity_node,))
    print("  graph.pose_key_to_node[pose_key(('target',))] = %s   <-- the OLD target node"
          % (graph_a.pose_key_to_node.get(graph_a.node_pose_key(("target",))),))
    print("  status              = %s" % res.status)
    print("  cost_um             = %s" % res.cost)
    print("  length_um           = %s" % res.length_um)
    print("  goal_pos_error_um   = %s" % res.goal_pos_error_um)
    print("  actual endpoint     = %s  (scene B target is (50,0,0))" % (end,))
    print("  fresh scene B (no graph= injection):")
    fresh = route_finite_native3d(scene(50.0), mode="native3d")
    print("    status=%s cost=%s endpoint=%s"
          % (fresh.status, fresh.cost,
             list(fresh.path.end_pose.position) if fresh.path else None))

    # --- also: the fingerprint that is stored but never checked
    bad = []
    sc_c = scene(50.0)
    sc_c.obstacles = [Box3D((20.0, -10.0, -10.0), (30.0, 10.0, 10.0))]
    fp_a = graph_a.fingerprint
    fp_c = graph_config_fingerprint(sc_c.config, sc_c)
    print("  stored fingerprint exists: keys=%d" % len(fp_a))
    print("  obstacles field of the stored fingerprint = %s" % (fp_a["obstacles"],))
    print("  obstacles field for scene C (one obstacle)  = %s" % (fp_c["obstacles"],))
    print("  reference_centrelines field                = %s (count only)"
          % (fp_a["reference_centrelines"],))
    print("  is the fingerprint compared anywhere? -> see: search for it in "
          "finite_graph.route_finite_native3d")
    print("  same-count different-geometry reference centrelines produce identical "
          "fingerprints: %s" % (fp_a["reference_centrelines"] == fp_c["reference_centrelines"]))

    # --- 2D graph reused in native3d mode
    graph2d = FiniteGraph(scene(100.0), mode="2d", cfg=sc_a.config)
    res_mix = route_finite_native3d(scene(100.0), mode="native3d", graph=graph2d)
    print("  prebuilt 2d graph reused with mode='native3d': status=%s" % res_mix.status)


# ======================================================================= A2
def a2_success_port_verification():
    head("A2 RED: success does not verify the actual assembled port poses")
    print("  (A1 already shows the assembled path ending at the OLD target while the")
    print("   status is success and goal_pos_error_um is silently 50 um.)")
    print("  Direct probe: does the public entry point contain any comparison of the")
    print("  assembled path endpoints against scene.source/scene.target?")
    src = open(os.path.join(PREFIX_SRC, "search", "finite_graph.py"),
               encoding="utf-8").read()
    for needle in ("end_pose", "start_pose", "scene.target.position",
                   "scene.source.position", "source_pos_tol_um",
                   "source_ang_tol_deg", "eps_pos_um", "eps_ang_deg"):
        print("    %-24s occurrences in finite_graph.py: %d"
              % (needle, src.count(needle)))
    print("  the only use of scene.source.position is the lattice anchor and")
    print("  scene.target.position only enters the goal test; neither is compared")
    print("  against the ASSEMBLED curve that is finally returned as success.")


# ======================================================================= A3
def a3_zero_length():
    head("A3 RED: source == target, default config, no obstacles/centrelines")
    from native3d_prefix.geometry.primitives import Pose3D
    sc = Scene(name="a3_zero_length",
               chip=Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)),
               obstacles=[], reference_centerlines=[],
               source=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               target=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               config=RouterConfig())
    print("  source = target = Pose3D((0,0,0),(1,0,0)), chip = [-60,60]^3, default config")
    try:
        res = route_finite_native3d(sc, mode="native3d")
        print("  status=%s cost=%s length=%s path=%s"
              % (res.status, res.cost, res.length_um, res.path))
    except Exception as exc:                                  # noqa: BLE001
        print("  RAISED %s: %s" % (type(exc).__name__, exc))
        tb = traceback.format_exc().strip().splitlines()
        for line in tb[-6:]:
            print("    | %s" % line)

    print("")
    print("  same position, WRONG target tangent (target tangent = +y):")
    from native3d_prefix.geometry.primitives import Pose3D as P3
    sc2 = Scene(name="a3_zero_length_bad_tangent",
                chip=Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)),
                obstacles=[], reference_centerlines=[],
                source=P3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                target=P3((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                config=RouterConfig())
    try:
        res2 = route_finite_native3d(sc2, mode="native3d")
        print("    status=%s cost=%s length=%s" % (res2.status, res2.cost, res2.length_um))
    except Exception as exc:                                  # noqa: BLE001
        print("    RAISED %s: %s" % (type(exc).__name__, exc))

    print("")
    print("  source == target but the source pose violates the obstacle keep-out:")
    wall = Box3D((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0))
    sc3 = Scene(name="a3_source_in_obstacle",
                chip=Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)),
                obstacles=[wall], reference_centerlines=[],
                source=P3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                target=P3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                config=RouterConfig())
    try:
        res3 = route_finite_native3d(sc3, mode="native3d")
        print("    status=%s reason=%s" % (res3.status, res3.reason[:90]))
    except Exception as exc:                                  # noqa: BLE001
        print("    RAISED %s: %s" % (type(exc).__name__, exc))


# ======================================================================= B1
def b1_point_arc_distance():
    head("B1 RED: point -> 3D planar arc distance")
    seg = {"kind": "arc", "center": [0.0, 0.0, 0.0], "u": [1.0, 0.0, 0.0],
           "v": [0.0, 1.0, 0.0], "radius_um": 50.0, "theta_rad": 2.0 * math.pi}
    p = (0.0, 0.0, 50.0)
    got = point_arc_distance(p, seg)
    print("  reference circle: center=(0,0,0), u=+x, v=+y, R=50, theta=2*pi")
    print("  p = (0,0,50)   (on the circle-plane normal through the centre)")
    print("  validator point_arc_distance = %.12f um" % got)
    print("  true distance = sqrt(50^2+50^2) = %.12f um" % math.sqrt(50.0 ** 2 + 50.0 ** 2))
    print("  -> reported 0 (the point is NOT on the curve)")

    path = {"segments": [{"kind": "line", "p0": [-10.0, 0.0, 50.0],
                          "p1": [10.0, 0.0, 50.0]}]}
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [],
             "reference_centerlines": [seg],
             "source_pose": {"position": [-10.0, 0.0, 50.0], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [10.0, 0.0, 50.0], "tangent": [1.0, 0.0, 0.0]}}
    v = validate_path_dict(path, scene, cfg_dict())
    sp = v["centerline_spacing"]
    print("  full legal path (-10,0,50)->(10,0,50), reference = that XY full circle:")
    print("    verdict            = %s" % v["verdict"])
    print("    spacing status     = %s" % sp["status"])
    print("    min_sampled_um     = %s" % sp["min_sampled_um"])
    print("    certified lb       = %s" % sp["conservative_lower_bound_um"])
    print("    self_check_failed  = %s" % v["certification"]["self_check_failed"])
    print("    true min distance  = sqrt(40^2+50^2) = %.12f um"
          % math.sqrt(40.0 ** 2 + 50.0 ** 2))
    print("  the production constraint stack on the same path:")
    arc = CircularArc3D.from_dict(seg)
    prod = primitive_spacing(LineSegment3D((-10.0, 0.0, 50.0), (10.0, 0.0, 50.0)),
                             [arc], RouterConfig().dmin_centerline_um, RouterConfig())
    print("    production status=%s bound=%.12f um" % (prod.status, prod.bound_um))


# ======================================================================= B2
def b2_json_tangent_normalisation():
    head("B2 RED: JSON source/target tangent is not normalised before acos")
    path = {"segments": [{"kind": "line", "p0": [0.0, 0.0, 0.0],
                          "p1": [100.0, 0.0, 0.0]}]}
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [], "reference_centerlines": [],
             "source_pose": {"position": [0.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 1.0, 0.0]}}
    v = validate_path_dict(path, scene, cfg_dict())
    ep = v["endpoint_errors"]
    print("  path (0,0,0)->(100,0,0); source tangent=[1,0,0]; target tangent=[1,1,0]")
    print("  validator end_tangent_error_deg = %r" % ep["end_tangent_error_deg"])
    print("  validator target_pose_check.status = %s" % v["target_pose_check"]["status"])
    print("  verdict = %s" % v["verdict"])
    print("  true angle = 45 deg (default target tolerance is 3 deg)")
    prod = Pose3D.from_dict(scene["target_pose"])
    print("  production Pose3D.from_dict target tangent = %s (normalised)" % (prod.tangent,))
    print("")
    print("  other malformed JSON poses (all must be REJECTED, never pass):")
    for label, src_t, tgt_t in (
            ("zero direction      ", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("NaN direction       ", [float("nan"), 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("Infinity direction  ", [float("inf"), 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("2-component tangent ", [1.0, 0.0], [1.0, 0.0, 0.0]),
            ("scaled tangent x1000", [1000.0, 0.0, 0.0], [1.0, 0.0, 0.0])):
        sc = dict(scene)
        sc["source_pose"] = {"position": [0.0, 0.0, 0.0], "tangent": list(src_t)}
        sc["target_pose"] = {"position": [100.0, 0.0, 0.0], "tangent": list(tgt_t)}
        try:
            r = validate_path_dict(path, sc, cfg_dict())
            print("    %s -> NO ERROR, verdict=%s start_ang_err=%r"
                  % (label, r["verdict"], r["endpoint_errors"]["start_tangent_error_deg"]))
        except Exception as exc:                              # noqa: BLE001
            print("    %s -> %s: %s" % (label, type(exc).__name__, str(exc)[:70]))


# ======================================================================= B3
def b3_near_unit_basis():
    head("B3 RED: near-unit (non-orthonormal) basis is accepted as a circular arc")
    R = 2000.0
    u = (1.0 + 8e-10, 0.0, 0.0)
    v = (0.0, 1.0, 0.0)
    theta = math.pi / 2.0
    print("  R=%g um, u=(1+8e-10,0,0), v=(0,1,0), theta=pi/2" % R)
    print("  norm(u) - 1 = %r  (constructor tolerance is 1e-9)" % (math.hypot(*u) - 1.0))
    arc = CircularArc3D((0.0, 0.0, 0.0), u, v, R, theta)
    print("  constructor ACCEPTED it; stored u = %s" % (arc.u,))
    print("  stored basis is NOT orthonormal -> the real curve is an ELLIPSE, not a circle")
    rmin_effective = R / (1.0 + 8e-10)
    print("  true minimum curvature radius = R/(1+8e-10) = %.10f um" % rmin_effective)
    print("  allowed lower limit = Rmin - radius_tol = %r um" % (2000.0 - 1e-6))
    print("  breach = %.3e um (>= 0 -> the real curve is NOT admissible as a circle)"
          % ((2000.0 - 1e-6) - rmin_effective))

    cfg = cfg_dict(rmin_um=2000.0, radius_tol_um=1e-6,
                   turn_radii_um=[2000.0], connector_radii_um=[2000.0])
    rc = radius_check([arc], RouterConfig(rmin_um=2000.0, radius_tol_um=1e-6,
                                          turn_radii_um=(2000.0,),
                                          connector_radii_um=(2000.0,)))
    print("  production radius_check = %s (bound=%r)" % (rc.status, rc.bound_um))

    scene = {"chip_volume": {"lo": [-5000.0] * 3, "hi": [5000.0] * 3},
             "obstacles": [], "reference_centerlines": [],
             "source_pose": {"position": list(arc.start),
                             "tangent": list(arc.tangent_at_phi(0.0))},
             "target_pose": {"position": list(arc.end),
                             "tangent": list(arc.tangent_at_phi(arc.theta))}}
    path = {"segments": [{"kind": "arc", "center": [0.0, 0.0, 0.0], "u": list(u),
                          "v": list(v), "radius_um": R, "theta_rad": theta}]}
    try:
        vv = validate_path_dict(path, scene, cfg)
        print("  independent validator verdict = %s" % vv["verdict"])
        print("    min_radius_um_stored = %r" % vv["min_radius_um_stored"])
        print("    min_radius_um_refit  = %r" % vv["min_radius_um_refit"])
        print("    radius_refit_max_rel_error = %r" % vv["radius_refit_max_rel_error"])
        print("    radius_violations    = %r" % vv["radius_violations"])
    except Exception as exc:                                  # noqa: BLE001
        print("  independent validator RAISED %s: %s" % (type(exc).__name__, exc))
    print("  both the production check and the validator called this a CIRCLE while")
    print("  the stored basis makes it an ellipse whose true Rmin is 1999.9999984 um")


# ======================================================================= PART C
def part_c_audit_points():
    head("PART C RED: bounds, timing wording and status vocabulary")
    src = open(os.path.join(PREFIX_SRC, "search", "finite_graph.py"),
               encoding="utf-8").read()
    for line in src.splitlines():
        if "4096" in line or "n_edges_upper" in line or "n_nodes_upper" in line:
            print("  | %s" % line.strip())
    for line in src.splitlines():
        if "edge_check_s is a subset" in line or "lazily during the search" in line:
            print("  | %s" % line.strip())
    astar = open(os.path.join(PREFIX_SRC, "search", "astar.py"), encoding="utf-8").read()
    print("  statuses returned by astar_search: success/invalid_source_pose/"
          "invalid_target_pose/search_exhausted/budget_exhausted + the router adds "
          "port_attachment_failed/uncertified")
    print("  constraint_failed is NOT a status anywhere: %s"
          % (not ("constraint_failed" in astar or "constraint_failed" in src)))
    print("  the assembled-path failure branch reuses search_exhausted:")
    for i, line in enumerate(src.splitlines()):
        if 'res.status = "uncertified" if undecided else "search_exhausted"' in line:
            print("    line %d: %s" % (i + 1, line.strip()))


if __name__ == "__main__":
    print("v0.1.3 PREFIX (RED) replay -- runs the FROZEN v0.1.2 snapshot")
    print("python      : %s" % sys.version.replace("\n", " "))
    print("project root: %s" % ROOT)
    print("snapshot    : %s" % PREFIX_SRC)
    t0 = time.perf_counter()
    a1_graph_injection_mismatch()
    a2_success_port_verification()
    a3_zero_length()
    b1_point_arc_distance()
    b2_json_tangent_normalisation()
    b3_near_unit_basis()
    part_c_audit_points()
    print("")
    print("=" * 78)
    print("END OF PREFIX REPLAY (%.3f s)" % (time.perf_counter() - t0))
    print("=" * 78)
