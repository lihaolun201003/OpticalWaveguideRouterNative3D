"""v0.1.3 acceptance probe: real runs, real numbers, real evidence files.

    .venv\\Scripts\\python.exe artifacts\\v013\\probe\\v013_acceptance_probe.py

It produces, under artifacts/v013/cases/:

    core_case_finite_native3d_native3d.json   the original cross-wall case
    core_case_finite_native3d_2d.json         its real 2D exhaustion
    v013_legal_3d_above_reference_arc.json    a full legal 3D path over a reference
                                              arc (the acceptance v0.1.2 missed)
    v013_zero_length_solution.json            the zero-length contract
    v013_graph_reuse_mismatch.json            a refused mismatched prebuilt graph
    v013_acceptance_summary.json              every number of this run

and prints the same numbers so the log file is the evidence.

TIMING AND MEMORY SCOPE (stated, not implied)
---------------------------------------------
* ``timings`` breaks one call into graph_build_s, search_s, assembled_path_check_s,
  independent_validation_s and end_to_end_s.  The END-TO-END number is the frozen
  acceptance quantity (60 s); the parts are disclosed separately because they
  measure different things.
* The default branch builds a COLD graph (no warm cache, no reuse); the warm
  reuse numbers are reported separately under ``graph_reuse`` and are NEVER
  extrapolated to the cold branch.
* ``tracemalloc`` peak is the peak TRACKED PYTHON ALLOCATION of the traced
  region.  It is NOT the process RSS and not the total memory of the interpreter:
  numpy buffers, the interpreter itself and any allocation made outside the traced
  region are excluded by construction.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import tracemalloc

HERE = os.path.dirname(os.path.abspath(__file__))
V013 = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(V013))
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.cases import core_case                                      # noqa: E402
from native3d.config import RouterConfig                                  # noqa: E402
from native3d.constraints.checks import Box3D, primitive_spacing          # noqa: E402
from native3d.geometry import CircularArc3D, LineSegment3D, Pose3D        # noqa: E402
from native3d.io.artifacts import (build_artifact, finalise_artifact)     # noqa: E402
from native3d.io.caseio import Scene, write_json                          # noqa: E402
from native3d.search.finite_graph import (SOURCE_NODE, TARGET_NODE,          # noqa: E402
                                          FiniteGraph,
                                          enumerate_graph_edges,
                                          route_finite_native3d)
from native3d.validation.validate import validate_path_dict               # noqa: E402

CASES = os.path.join(V013, "cases")
RESULTS = {}


def head(title):
    print("")
    print("=" * 78)
    print(title)
    print("=" * 78)


def cfg_dict(**over):
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


def independent_dijkstra(edges, start, goal):
    """A plain Dijkstra written here, using none of search/astar.py."""
    import heapq
    adj = {}
    for e in edges:
        adj.setdefault(e.from_node, []).append((e.to_node, e.cost))
    dist = {start: 0.0}
    heap = [(0.0, 0, start)]
    counter = 0
    done = set()
    while heap:
        d, _c, u = heapq.heappop(heap)
        if u in done:
            continue
        done.add(u)
        if u == goal:
            return d, len(done)
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, math.inf) - 1e-15:
                dist[v] = nd
                counter += 1
                heapq.heappush(heap, (nd, counter, v))
    return math.inf, len(done)


def fg_of(res):
    return res.finite_graph or {}


# =============================================================== 1. cross wall
def part_1_cross_wall():
    head("1. ORIGINAL CROSS-WALL CASE (the result that must not change)")
    scene = core_case()
    cfg = scene.config
    print("frozen acceptance budget: <= %.1f s end-to-end, <= %d expansions"
          % (cfg.finite_max_runtime_s, cfg.finite_max_expansions))

    tracemalloc.start()
    t0 = time.perf_counter()
    res = route_finite_native3d(scene, mode="native3d")
    wall_cold_s = time.perf_counter() - t0
    _cur, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    t = res.timings
    print("native3d: status=%s cost=%.9f um length=%.9f um expansions=%d trace_steps=%d"
          % (res.status, res.cost, res.length_um, res.expansions, len(res.trace)))
    fg = fg_of(res)
    b = fg.get("bounds", {})
    print("   bounds: node_upper=%s edge_upper=%s lattice_positions=%s directions=%s"
          % (b.get("node_upper_bound"), b.get("edge_upper_bound"),
             b.get("n_lattice_positions"), b.get("n_directions")))
    print("   bounds includes the virtual ports: %s"
          % (b.get("node_upper_bound_includes_virtual_ports"),))
    print("   edges validated=%s rejected=%s (undecided %s, end-pose-rejected %s) "
          "cache_hits=%s"
          % (fg.get("edges_validated"), fg.get("edges_rejected"),
             fg.get("edges_rejected_undecided"), fg.get("edges_end_pose_rejected"),
             fg.get("cache_hits")))
    print("   max curve-end vs canonical node error: position %s um, tangent %s deg"
          % (fg.get("max_curve_end_position_error_um"),
             fg.get("max_curve_end_tangent_error_deg")))
    print("   port attachment kinds: %s"
          % {k: (v or {}).get("kind") for k, v in fg.get("port_attachment", {}).items()})
    print("   graph reuse: %s" % json.dumps(fg.get("graph_reuse", {}))[:220])
    pa = getattr(res, "port_assurance", None)
    if pa:
        print("   port assurance: start %.3e um / %.3e deg, end %.3e um / %.3e deg, "
              "junctions %.3e um / %.3e deg"
              % (pa.get("source_position_error_um", float("nan")),
                 pa.get("source_tangent_error_deg", float("nan")),
                 pa.get("target_position_error_um", float("nan")),
                 pa.get("target_tangent_error_deg", float("nan")),
                 pa.get("assembled_path_check", {}).get(
                     "max_junction_position_error_um", float("nan")),
                 pa.get("assembled_path_check", {}).get(
                     "max_junction_tangent_error_deg", float("nan"))))
    print("   trace:")
    for s in res.trace:
        print("     %-34s (%8.3f,%8.3f,%8.3f) dir=%s"
              % (s["action"], s["x_um"], s["y_um"], s["z_um"],
                 ["%+.0f" % c for c in s["direction"]]))

    # ---- independent cross-check over the fully enumerated graph
    #
    # Two traps live here and both produced a bogus inf before:
    #  1. the START of the enumerated graph is the node the ROUTER starts from: the
    #     canonical node that IS the source pose when the source is a lattice node,
    #     otherwise the virtual source port.  Rooting a Dijkstra at
    #     `source_identity_node` unconditionally fails whenever that is None;
    #  2. the port nodes are the TUPLES ("source",) / ("target",), i.e.
    #     SOURCE_NODE / TARGET_NODE.  Passing the bare string "target" builds a goal
    #     that no edge can ever reach, so the search settles the whole graph and
    #     returns inf while the router returns 594.159265.
    graph, edges = enumerate_graph_edges(scene, mode="native3d")
    start = (graph.source_identity_node if graph.source_identity_node is not None
             else SOURCE_NODE)
    goal = (graph.target_identity_node if graph.target_identity_node is not None
            else TARGET_NODE)
    assert any(e.from_node == start for e in edges), \
        "the enumerated graph must contain the start node the router used"
    assert any(e.to_node == goal for e in edges), \
        "the enumerated graph must contain the goal node the router used"
    t1 = time.perf_counter()
    best, n_done = independent_dijkstra(edges, start, goal)
    dij_s = time.perf_counter() - t1
    assert best < math.inf, "the independent Dijkstra must reach the goal node"
    print("   independent Dijkstra: best=%.9f um (A* %.9f, diff=%.3e), %d settled, "
          "%d edges, %.4f s"
          % (best, res.cost, abs(best - res.cost), n_done, len(edges), dij_s))
    print("   cost = length + congestion: %.9f = %.9f + %.9f"
          % (res.cost, res.length_um, res.congestion_cost))

    # ---- independent validation of the stored artifact
    art_path = os.path.join(CASES, "core_case_finite_native3d_native3d.json")
    t2 = time.perf_counter()
    art = finalise_artifact(
        build_artifact(scene, res, "native3d",
                       ".venv\\Scripts\\python.exe "
                       "artifacts\\v013\\probe\\v013_acceptance_probe.py",
                       extra={"artifact": "native3d_v013_core_case_evidence",
                              "engine": "OpticalWaveguideRouterNative3D / native3d v0.1.3"}),
        art_path)
    val_s = time.perf_counter() - t2
    val = art["independent_validation"]
    t.setdefault("independent_validation_s", val_s)
    t.setdefault("assembled_path_check_s", None)
    print("   timings: graph_build=%.4f s  search=%.4f s  assembled_check=%s  "
          "end_to_end=%.4f s  (wall %.4f s)"
          % (t.get("graph_build_s", float("nan")), t.get("search_s", float("nan")),
             ("%.4f s" % t["assembled_path_check_s"]
              if isinstance(t.get("assembled_path_check_s"), float) else "n/a"),
             t.get("end_to_end_s", float("nan")), wall_cold_s))
    print("   plus an INDEPENDENT validation pass outside the router: %.4f s" % val_s)
    print("   peak TRACKED python allocation = %.3f MiB  (NOT process RSS)"
          % (peak / 1024.0 / 1024.0))
    print("   independent validation: verdict=%s spacing=%s boundary=%s "
          "c1=%s radius_violations=%d self_check_failed=%s"
          % (val["verdict"], val["centerline_spacing"]["status"],
             val["boundary"]["status"], val["c1_continuous"],
             len(val["radius_violations"]),
             val["certification"]["self_check_failed"]))
    print("   artifact written: %s" % art_path)

    # ---- 2D: a real exhaustion
    t3 = time.perf_counter()
    res2d = route_finite_native3d(core_case(), mode="2d")
    wall_2d = time.perf_counter() - t3
    print("2d      : status=%s budget_exhausted=%s expansions=%d wall=%.4f s"
          % (res2d.status, res2d.budget_exhausted, res2d.expansions, wall_2d))
    art2d = finalise_artifact(
        build_artifact(core_case(), res2d, "2d",
                       ".venv\\Scripts\\python.exe "
                       "artifacts\\v013\\probe\\v013_acceptance_probe.py",
                       extra={"artifact": "native3d_v013_core_case_evidence",
                              "engine": "OpticalWaveguideRouterNative3D / native3d v0.1.3"}),
        os.path.join(CASES, "core_case_finite_native3d_2d.json"))
    print("   independent validation of the 2D artifact: %s"
          % (None if art2d["independent_validation"] is None
             else art2d["independent_validation"]["verdict"]))

    budget_ok = (res.timings["end_to_end_s"] <= cfg.finite_max_runtime_s
                 and res.expansions <= cfg.finite_max_expansions)
    print("acceptance budget check: end-to-end %.4f s <= %.1f s and expansions %d <= "
          "%d -> %s" % (res.timings["end_to_end_s"], cfg.finite_max_runtime_s,
                        res.expansions, cfg.finite_max_expansions, budget_ok))

    RESULTS["cross_wall"] = {
        "status": res.status, "cost_um": res.cost, "length_um": res.length_um,
        "expansions": res.expansions, "trace_steps": len(res.trace),
        "timings": dict(t), "wall_cold_s": wall_cold_s,
        "peak_tracked_python_allocation_mib": peak / 1024.0 / 1024.0,
        "memory_scope": ("tracemalloc peak of TRACKED PYTHON ALLOCATIONS in the "
                         "traced region; not process RSS, not the whole interpreter"),
        "bounds": b, "edges_validated": fg.get("edges_validated"),
        "edges_rejected": fg.get("edges_rejected"),
        "edges_end_pose_rejected": fg.get("edges_end_pose_rejected"),
        "cache_hits": fg.get("cache_hits"),
        "graph_reuse": fg.get("graph_reuse"),
        "port_attachment_kinds": {k: (v or {}).get("kind")
                                  for k, v in fg.get("port_attachment", {}).items()},
        "endpoint_errors": getattr(res, "endpoint_errors", None),
        "port_assurance": pa,
        "zero_length_solution": getattr(res, "zero_length_solution", None),
        "independent_dijkstra": {"best_cost_um": best, "nodes_settled": n_done,
                                 "n_edges": len(edges), "runtime_s": dij_s,
                                 "difference_um": abs(best - res.cost)},
        "independent_validation_verdict": val["verdict"],
        "independent_validation_timing_s": t.get("independent_validation_s"),
        "budget_ok": budget_ok,
        "v012_reference": {"cost_um": 594.1592653589794, "expansions": 75},
        "two_d": {"status": res2d.status, "budget_exhausted": res2d.budget_exhausted,
                  "expansions": res2d.expansions, "wall_s": wall_2d},
    }


# =================================================== 2. warm reuse vs cold build
def part_2_cold_vs_warm():
    head("2. COLD DEFAULT BRANCH vs EXPLICIT WARM GRAPH REUSE (kept separate)")
    scene = core_case()
    t0 = time.perf_counter()
    cold = route_finite_native3d(core_case(), mode="native3d")
    cold_s = time.perf_counter() - t0
    graph = FiniteGraph(scene, mode="native3d", cfg=scene.config, congestion=None)
    t1 = time.perf_counter()
    warm = route_finite_native3d(scene, graph=graph)
    warm_s = time.perf_counter() - t1
    t2 = time.perf_counter()
    warm2 = route_finite_native3d(scene, graph=graph)
    warm2_s = time.perf_counter() - t2
    print("cold default branch (builds its own graph): status=%s wall=%.4f s "
          "graph_build=%.4f s graph_reuse=%s"
          % (cold.status, cold_s, cold.timings.get("graph_build_s"),
             json.dumps(fg_of(cold).get("graph_reuse", {}))))
    print("warm call 1 (prebuilt matching graph):      status=%s wall=%.4f s "
          "graph_build=%.4f s reuse=%s"
          % (warm.status, warm_s, warm.timings.get("graph_build_s"),
             json.dumps(fg_of(warm).get("graph_reuse", {}))))
    print("warm call 2 (same object again):            status=%s wall=%.4f s "
          "reuse=%s"
          % (warm2.status, warm2_s, json.dumps(fg_of(warm2).get("graph_reuse", {}))))
    print("the 60 s / 200000 expansion budget is the COLD DEFAULT branch quantity; "
          "the warm numbers are reported separately and are never extrapolated to it")
    print("cost equality across the branches: cold=%.9f warm=%.9f warm2=%.9f"
          % (cold.cost, warm.cost, warm2.cost))
    RESULTS["cold_vs_warm"] = {
        "cold": {"status": cold.status, "wall_s": cold_s,
                 "graph_build_s": cold.timings.get("graph_build_s"),
                 "graph_reuse": fg_of(cold).get("graph_reuse")},
        "warm_1": {"status": warm.status, "wall_s": warm_s,
                   "graph_build_s": warm.timings.get("graph_build_s"),
                   "graph_reuse": fg_of(warm).get("graph_reuse")},
        "warm_2": {"status": warm2.status, "wall_s": warm2_s,
                   "graph_reuse": fg_of(warm2).get("graph_reuse")},
        "cost_um": {"cold": cold.cost, "warm_1": warm.cost, "warm_2": warm2.cost},
        "scope_note": ("the frozen 60 s budget belongs to the cold default branch; "
                       "warm reuse timings are separate and not extrapolated"),
    }


# ============================================ 3. the legal 3D path above an arc
XY_FULL_CIRCLE = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                               50.0, 2.0 * math.pi)


def part_3_legal_3d_above_a_reference_arc():
    head("3. NEW: a FULL LEGAL 3D PATH above a reference arc must be accepted")
    # The reference is the XY full circle {(50 cos p, 50 sin p, 0)}.  The path runs
    # along x at height z, so its nearest point to the circle is the in-plane
    # projection of (10, 0, z), and the exact minimum distance is
    # sqrt((50 - hypot(10, z))^2 + 0) = sqrt(40^2 + z^2) for z >= 0.
    z = 50.0
    true_min = math.sqrt(40.0 ** 2 + z ** 2)
    scene = {"chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
             "obstacles": [],
             "reference_centerlines": [XY_FULL_CIRCLE.to_dict()],
             "source_pose": {"position": [-10.0, 0.0, z], "tangent": [1.0, 0.0, 0.0]},
             "target_pose": {"position": [10.0, 0.0, z], "tangent": [1.0, 0.0, 0.0]}}
    path = {"segments": [{"kind": "line", "p0": [-10.0, 0.0, z],
                          "p1": [10.0, 0.0, z]}]}
    cfg = RouterConfig()
    prod = primitive_spacing(LineSegment3D((-10.0, 0.0, z), (10.0, 0.0, z)),
                             [XY_FULL_CIRCLE], cfg.dmin_centerline_um, cfg)
    v = validate_path_dict(path, scene, cfg_dict())
    print("path (-10,0,50) -> (10,0,50) flown over the XY full circle "
          "(centre (0,0,0), u=+x, v=+y, R=50, theta=2*pi)")
    print("   true minimum distance = sqrt(40^2+50^2) = %.12f um" % true_min)
    print("   production constraint stack : status=%s bound=%.12f um"
          % (prod.status, prod.bound_um))
    print("   independent validator       : verdict=%s spacing=%s sampled=%.12f um "
          "lb=%.12f ub=%s self_check_failed=%s"
          % (v["verdict"], v["centerline_spacing"]["status"],
             v["centerline_spacing"]["min_sampled_um"],
             v["centerline_spacing"]["conservative_lower_bound_um"],
             v["centerline_spacing"].get("upper_bound_um"),
             v["certification"]["self_check_failed"]))
    print("   v0.1.2 was: validator verdict=fail, sampled minimum 0.0, "
          "self_check_failed=True (see artifacts/v013/logs/00_prefix_v013_defect_replay.log)")

    # A REAL crossing of the same circle: the segment y = z = 0 from x = -100 to
    # x = +100 passes through the curve and its interior (true distance 0.0).
    scene0 = json.loads(json.dumps(scene))
    scene0["source_pose"]["position"] = [-100.0, 0.0, 0.0]
    scene0["target_pose"]["position"] = [100.0, 0.0, 0.0]
    path0 = {"segments": [{"kind": "line", "p0": [-100.0, 0.0, 0.0],
                           "p1": [100.0, 0.0, 0.0]}]}
    v0 = validate_path_dict(path0, scene0, cfg_dict())
    prod0 = primitive_spacing(LineSegment3D((-100.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
                              [XY_FULL_CIRCLE], cfg.dmin_centerline_um, cfg)
    print("   control (y=z=0 from x=-100 to x=+100, which really crosses the arc): "
          "production=%s validator=%s sampled=%.3e"
          % (prod0.status, v0["verdict"], v0["centerline_spacing"]["min_sampled_um"]))

    # A legal in-plane detour that stays INSIDE the circle: 40 um away, so it must
    # pass.  This is the kind of legal configuration v0.1.2 wrongly rejected.
    scene_in = json.loads(json.dumps(scene))
    scene_in["source_pose"]["position"] = [-10.0, 0.0, 0.0]
    scene_in["target_pose"]["position"] = [10.0, 0.0, 0.0]
    path_in = {"segments": [{"kind": "line", "p0": [-10.0, 0.0, 0.0],
                             "p1": [10.0, 0.0, 0.0]}]}
    v_in = validate_path_dict(path_in, scene_in, cfg_dict())
    print("   control (inside the circle, 40 um away): validator=%s sampled=%.6f"
          % (v_in["verdict"], v_in["centerline_spacing"]["min_sampled_um"]))

    # A genuinely too-close 3D configuration: reference circle R = 12 in the XZ
    # plane and the same path at y = z = 0 -> true distance 12 - 10 = 2 um < dmin.
    small = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                          12.0, 2.0 * math.pi)
    scene_small = json.loads(json.dumps(scene_in))
    scene_small["reference_centerlines"] = [small.to_dict()]
    v_small = validate_path_dict(path_in, scene_small, cfg_dict())
    prod_small = primitive_spacing(LineSegment3D((-10.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
                                   [small], cfg.dmin_centerline_um, cfg)
    print("   control (reference R=12, true distance 2 um < dmin 5 um): "
          "production=%s validator=%s sampled=%.6f"
          % (prod_small.status, v_small["verdict"],
             v_small["centerline_spacing"]["min_sampled_um"]))
    assert v["verdict"] == "pass", v["verdict"]
    assert v_in["verdict"] == "pass", v_in["verdict"]
    assert v0["verdict"] == "fail", v0["verdict"]
    assert v_small["verdict"] == "fail", v_small["verdict"]

    write_json(os.path.join(CASES, "v013_legal_3d_above_reference_arc.json"),
               {"artifact": "native3d_v013_legal_3d_above_reference_arc",
                "path": path, "scene": scene,
                "true_minimum_distance_um": true_min,
                "production": {"status": prod.status, "bound_um": prod.bound_um},
                "independent_validation": v,
                "control_crossing_from_x_minus_100": {
                    "production_status": prod0.status,
                    "validator_verdict": v0["verdict"],
                    "validator_sampled_min_um":
                        v0["centerline_spacing"]["min_sampled_um"]},
                "control_legal_inside_the_circle": {
                    "validator_verdict": v_in["verdict"],
                    "validator_sampled_min_um":
                        v_in["centerline_spacing"]["min_sampled_um"]},
                "control_too_close_R12": {
                    "production_status": prod_small.status,
                    "validator_verdict": v_small["verdict"],
                    "validator_sampled_min_um":
                        v_small["centerline_spacing"]["min_sampled_um"]},
                "note": ("v0.1.2 rejected this legal path (sampled minimum 0.0) "
                         "because the point-to-arc distance was the radial "
                         "difference to the sphere through the arc")})
    RESULTS["legal_3d_above_reference_arc"] = {
        "true_minimum_distance_um": true_min,
        "production_status": prod.status, "production_bound_um": prod.bound_um,
        "validator_verdict": v["verdict"],
        "validator_spacing_status": v["centerline_spacing"]["status"],
        "validator_sampled_min_um": v["centerline_spacing"]["min_sampled_um"],
        "validator_lower_bound_um":
            v["centerline_spacing"]["conservative_lower_bound_um"],
        "self_check_failed": v["certification"]["self_check_failed"],
        "control_crossing_validator_verdict": v0["verdict"],
        "control_crossing_sampled_min_um":
            v0["centerline_spacing"]["min_sampled_um"],
        "control_inside_circle_validator_verdict": v_in["verdict"],
        "control_too_close_validator_verdict": v_small["verdict"],
        "control_too_close_sampled_min_um":
            v_small["centerline_spacing"]["min_sampled_um"],
    }


# ==================================================== 4. mismatched graph reuse
def part_4_graph_reuse_mismatch():
    head("4. A MISMATCHED PREBUILT GRAPH (must be refused BEFORE the search)")
    # chip [-150,150]^3 with h = 50 um: the lattice is -2..2 per axis, the source
    # (0,0,0) is strictly inside the shrunken volume, scene B's target (50,0,0) is a
    # DIFFERENT lattice point from the source (so a fresh route is a real 50 um edge
    # and not a zero-length solution), and scene A's target (100,0,0) is the lattice
    # node (2,0,0,+x) that the stale graph keeps as its goal.
    chip = Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))

    def scene(name, tx):
        return Scene(name=name, chip=chip, obstacles=[], reference_centerlines=[],
                     source=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                     target=Pose3D((tx, 0.0, 0.0), (1.0, 0.0, 0.0)),
                     config=RouterConfig())

    graph_a = FiniteGraph(scene("A", 100.0), mode="native3d",
                          cfg=scene("A", 100.0).config)
    res = route_finite_native3d(scene("B", 50.0), graph=graph_a)
    print("scene A: source (0,0,0)+x target (100,0,0)+x ; scene B: same source, "
          "target (50,0,0)+x ; chip [-150,150]^3")
    print("   graph = FiniteGraph(scene A); route_finite_native3d(scene B, graph=graph)")
    print("   v0.1.3: status=%s path=%s cost=%s" % (res.status, res.path, res.cost))
    print("   v0.1.2 was: status=success, cost=100.0, endpoint=(100,0,0), "
          "goal_pos_error_um=50 (see the prefix replay log)")
    rep = fg_of(res).get("graph_reuse", {})
    print("   graph_reuse: accepted=%s caller_supplied=%s mismatches=%s "
          "detail=%s state=%s"
          % (rep.get("accepted"), rep.get("caller_supplied_graph"),
             rep.get("mismatches"), json.dumps(rep.get("mismatch_detail", {})),
             rep.get("graph_cache_state")))
    fresh = route_finite_native3d(scene("B2", 50.0), mode="native3d")
    print("   fresh scene B (no injection): status=%s cost=%s endpoint=%s"
          % (fresh.status, fresh.cost,
             list(fresh.path.end_pose.position) if fresh.path else None))
    assert res.status == "graph_scene_mismatch"
    assert fresh.status == "success"
    assert fresh.cost == 50.0

    art = finalise_artifact(
        build_artifact(scene("B", 50.0), res, "native3d",
                       ".venv\\Scripts\\python.exe "
                       "artifacts\\v013\\probe\\v013_acceptance_probe.py",
                       extra={"artifact": "native3d_v013_graph_reuse_mismatch",
                              "engine": "OpticalWaveguideRouterNative3D / native3d v0.1.3"}),
        os.path.join(CASES, "v013_graph_reuse_mismatch.json"))
    RESULTS["graph_reuse_mismatch"] = {
        "status": res.status, "path": None, "cost": res.cost,
        "graph_reuse": rep,
        "fresh_scene_b": {"status": fresh.status, "cost_um": fresh.cost,
                          "endpoint": (list(fresh.path.end_pose.position)
                                       if fresh.path else None)},
        "artifact_written": os.path.join(CASES, "v013_graph_reuse_mismatch.json"),
    }


# ============================================================== 5. zero length
def part_5_zero_length():
    head("5. THE ZERO-LENGTH SOLUTION CONTRACT")
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = Scene("v013_zero_length",
               Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)), [], [],
               pose, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    res = route_finite_native3d(sc, mode="native3d")
    print("source = target = Pose3D((0,0,0),(1,0,0)), chip=[-60,60]^3, default config")
    print("   v0.1.3: status=%s zero_length_solution=%s cost=%s length=%s path=%r "
          "n_segments=%s expansions=%s"
          % (res.status, getattr(res, "zero_length_solution", None), res.cost,
             res.length_um, res.path, getattr(res, "n_segments", None),
             res.expansions))
    print("   v0.1.2 was: AttributeError: 'NoneType' object has no attribute "
          "'segments' (see the prefix replay log)")
    ep = getattr(res, "endpoint_errors", None)
    print("   endpoint errors: %s" % json.dumps(ep))
    art = finalise_artifact(
        build_artifact(sc, res, "native3d",
                       ".venv\\Scripts\\python.exe "
                       "artifacts\\v013\\probe\\v013_acceptance_probe.py",
                       extra={"artifact": "native3d_v013_zero_length_solution",
                              "engine": "OpticalWaveguideRouterNative3D / native3d v0.1.3"}),
        os.path.join(CASES, "v013_zero_length_solution.json"))
    print("   JSON: path=%r zero_length_solution=%s n_segments=%s | independent "
          "validation verdict=%s zero_length=%s"
          % (art["search"].get("path"), art["search"].get("zero_length_solution"),
             art["search"].get("n_segments"),
             None if art["independent_validation"] is None
             else art["independent_validation"]["verdict"],
             None if art["independent_validation"] is None
             else art["independent_validation"].get("zero_length_solution")))
    print("   the same position with a tangent outside the acceptance ball:")
    from math import cos, radians, sin
    th = radians(10.0)
    sc2 = Scene("v013_zero_length_bad_tangent",
                Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)), [], [],
                pose, Pose3D((0.0, 0.0, 0.0), (cos(th), sin(th), 0.0)),
                RouterConfig())
    res2 = route_finite_native3d(sc2, mode="native3d")
    print("      status=%s zero_length_solution=%s path=%s"
          % (res2.status, getattr(res2, "zero_length_solution", None), res2.path))
    print("   the source pose inside an obstacle keep-out:")
    sc3 = Scene("v013_zero_length_bad_source",
                Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0)),
                [Box3D((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0))], [],
                pose, Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    res3 = route_finite_native3d(sc3, mode="native3d")
    print("      status=%s" % res3.status)
    RESULTS["zero_length"] = {
        "status": res.status,
        "zero_length_solution": getattr(res, "zero_length_solution", None),
        "cost_um": res.cost, "length_um": res.length_um, "path": None,
        "n_segments": getattr(res, "n_segments", None),
        "expansions": res.expansions,
        "endpoint_errors": ep,
        "artifact_verdict": (None if art["independent_validation"] is None
                             else art["independent_validation"]["verdict"]),
        "wrong_tangent": {"status": res2.status,
                          "zero_length_solution": getattr(res2, "zero_length_solution",
                                                          None)},
        "invalid_source": {"status": res3.status},
    }


def main():
    print("v0.1.3 acceptance probe")
    print("python : %s" % sys.version.replace("\n", " "))
    print("root   : %s" % ROOT)
    os.makedirs(CASES, exist_ok=True)
    t0 = time.perf_counter()
    part_1_cross_wall()
    part_2_cold_vs_warm()
    part_3_legal_3d_above_a_reference_arc()
    part_4_graph_reuse_mismatch()
    part_5_zero_length()
    total = time.perf_counter() - t0
    print("")
    print("=" * 78)
    print("SUMMARY")
    print("=" * 78)
    print("total probe wall time: %.3f s" % total)
    write_json(os.path.join(CASES, "v013_acceptance_summary.json"),
               {"artifact": "native3d_v013_acceptance_summary",
                "engine": "OpticalWaveguideRouterNative3D / native3d v0.1.3",
                "probe_wall_s": total,
                "results": RESULTS})
    print("summary written: %s" % os.path.join(CASES, "v013_acceptance_summary.json"))
    print("")
    print("=" * 78)
    print("END OF v0.1.3 ACCEPTANCE PROBE")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
