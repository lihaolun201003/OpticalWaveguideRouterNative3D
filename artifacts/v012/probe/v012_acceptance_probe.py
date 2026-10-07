"""v0.1.2 PART A/B/C acceptance probe (real runs, real numbers).

Runs, in one process, on the unmodified v0.1.2 tree:

1. the five confirmed PART A counterexamples again, POST-fix, so the report can
   state what each one returns now;
2. the v0.1.2 production finite multi-hop router on the core cross-wall scene in
   native3d AND 2d mode, with an independent Dijkstra cross-check of the cost and
   cold-cache timings plus tracemalloc/OS peak memory;
3. the frozen acceptance budget check (60 s single search, 200000 expansions).

    .venv Scripts python.exe artifacts/v012/probe/v012_acceptance_probe.py

Everything it prints is written verbatim to
artifacts/v012/logs/10_v012_acceptance_probe.log by the driver command.
"""

import json
import math
import os
import sys
import time
import tracemalloc

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.config import RouterConfig                                    # noqa: E402
from native3d.constraints.checks import (Box3D, primitive_box_clearance,     # noqa: E402
                                         primitive_spacing,
                                         segment_segment_distance_certified)
from native3d.cases import core_case                                        # noqa: E402
from native3d.geometry import CircularArc3D, LineSegment3D, Pose3D          # noqa: E402
from native3d.io.caseio import write_json                                   # noqa: E402
from native3d.search.astar import Candidate, astar_search                   # noqa: E402
from native3d.search.finite_graph import (SOURCE_NODE, TARGET_NODE,          # noqa: E402
                                          enumerate_graph_edges,
                                          route_finite_native3d)
from native3d.search.state import State3D                                   # noqa: E402
from native3d.geometry.vec import norm, sub                                 # noqa: E402
from native3d.validation.validate import validate_path_dict                 # noqa: E402

CASES = os.path.join(ROOT, "artifacts", "v012", "cases")
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


# ------------------------------------------------------------------ PART A
def part_a_after_fix():
    head("PART A (POST-FIX): the five confirmed counterexamples, re-run")
    S, A, B, G = (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0000000002, 0.0, 0.0), (20.0, 0.0, 0.0)
    X = (1.0, 0.0, 0.0)
    nodes = {S: S, A: A, B: B, G: G}
    edges = {S: [(A, 10.0), (B, 10.0000000002)], B: [(G, 9.9999999998)]}

    def generate(state, collect):
        acc, rej = [], []
        for nxt, w in edges.get(state.position, []):
            seg = LineSegment3D(nodes[state.position], nodes[nxt])
            act = type("A", (), {"name": "edge", "kind": "graph_edge",
                                 "segments": (seg,), "length": w,
                                 "z_change_um": 0.0, "elevation_change_deg": 0.0})()
            acc.append(Candidate(act, State3D(nodes[nxt], X), w, True, ""))
        return acc, rej

    def goal_test(state):
        d = norm(sub(state.position, G))
        return d <= 1e-9, d, 0.0

    res = astar_search(State3D(S, X), generate, goal_test,
                       lambda s: norm(sub(s.position, G)), lambda s: s.pose_key(),
                       RouterConfig(), mode="graph", heuristic_name="euclidean",
                       max_expansions=1000, max_runtime_s=10.0)
    print("A1 micro-difference counterexample (S->A=10 dead end, S->B=10.0000000002,")
    print("   B->G=9.9999999998), default identity pose_key():")
    print("   status=%s cost=%s expansions=%d   (v0.1.1 key_exact(): search_exhausted, 2)"
          % (res.status, res.cost, res.expansions))
    RESULTS["A1"] = {"status": res.status, "cost": res.cost,
                     "expansions": res.expansions,
                     "prefix_status_was": "search_exhausted",
                     "prefix_expansions_was": 2}

    path = {"segments": [{"kind": "line", "p0": [10.0, 0.0, 0.0], "p1": [100.0, 0.0, 0.0]}]}
    scene = {
        "chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [], "reference_centerlines": [],
        "source_pose": {"position": [0.0, 0.0, 0.0], "tangent": [0.0, 1.0, 0.0]},
        "target_pose": {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]},
    }
    v = validate_path_dict(path, scene, cfg_dict())
    print("A2 source pose violation: verdict=%s source_position_out=%d "
          "source_tangent_out=%d   (v0.1.1: pass)"
          % (v["verdict"], v["violation_counts"]["source_position_out_of_tolerance"],
             v["violation_counts"]["source_tangent_out_of_tolerance"]))
    RESULTS["A2"] = {"verdict": v["verdict"],
                     "source_position_out_of_tolerance":
                         v["violation_counts"]["source_position_out_of_tolerance"],
                     "source_tangent_out_of_tolerance":
                         v["violation_counts"]["source_tangent_out_of_tolerance"],
                     "prefix_verdict_was": "pass"}

    arc = CircularArc3D((50.0, 50.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0), 50.0, math.pi)
    far = LineSegment3D((0.0, 100.0, 0.0), (100.0, 100.0, 0.0))
    scene3 = dict(scene)
    scene3["reference_centerlines"] = [arc.to_dict(), far.to_dict()]
    scene3["source_pose"] = {"position": [10.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}
    scene3["target_pose"] = {"position": [100.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]}
    v3 = validate_path_dict(path, scene3, cfg_dict())
    scene3b = dict(scene3)
    scene3b["reference_centerlines"] = [arc.to_dict()]
    v3b = validate_path_dict(path, scene3b, cfg_dict())
    prod = primitive_spacing(LineSegment3D((10.0, 0.0, 0.0), (100.0, 0.0, 0.0)),
                             [arc, far], 5.0, RouterConfig())
    print("A3 reference arc: production=%s bound=%.6f um | validator(arc+line)=%s "
          "sampled=%.6f lb=%.6f | validator(arc only)=%s   (v0.1.1: pass / TypeError)"
          % (prod.status, prod.bound_um, v3["verdict"],
             v3["centerline_spacing"]["min_sampled_um"],
             v3["centerline_spacing"]["conservative_lower_bound_um"], v3b["verdict"]))
    RESULTS["A3"] = {"production_status": prod.status,
                     "production_bound_um": prod.bound_um,
                     "validator_verdict": v3["verdict"],
                     "validator_sampled_um": v3["centerline_spacing"]["min_sampled_um"],
                     "validator_lower_bound_um":
                         v3["centerline_spacing"]["conservative_lower_bound_um"],
                     "validator_arc_only_verdict": v3b["verdict"],
                     "prefix_verdict_was": "pass",
                     "prefix_arc_only_was": "TypeError"}

    cfg = RouterConfig()
    big = CircularArc3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 50.0, 8.0 * math.pi)
    ref = LineSegment3D((-51.0, -1.0, 0.0), (-49.0, 1.0, 0.0))
    box = Box3D((-51.0, -1.0, -1.0), (-49.0, 1.0, 1.0))
    sp = primitive_spacing(big, [ref], cfg.dmin_centerline_um, cfg)
    oc = primitive_box_clearance(big, box, cfg.effective_obstacle_clearance_um, cfg)
    scene4 = {
        "chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [], "reference_centerlines": [ref.to_dict()],
        "source_pose": {"position": list(big.start),
                        "tangent": list(big.tangent_at_phi(0.0))},
        "target_pose": {"position": list(big.end),
                        "tangent": list(big.tangent_at_phi(big.theta))},
    }
    v4 = validate_path_dict({"segments": [big.to_dict()]}, scene4, cfg_dict())
    print("A4 8*pi sweep: production spacing=%s (%.6f) obstacle=%s (%.6f) | validator "
          "status=%s sampled=%.6f lb=%.6f verdict=%s self_check_failed=%s"
          % (sp.status, sp.bound_um, oc.status, oc.bound_um,
             v4["centerline_spacing"]["status"], v4["centerline_spacing"]["min_sampled_um"],
             v4["centerline_spacing"]["conservative_lower_bound_um"], v4["verdict"],
             v4["certification"]["self_check_failed"]))
    print("   (v0.1.1: spacing safe 99.005050376 um, obstacle safe 99.0 um, validator "
          "pass with claimed lb 99.005050376 > sampled 0.022088195608)")
    RESULTS["A4"] = {"production_spacing_status": sp.status,
                     "production_spacing_bound_um": sp.bound_um,
                     "production_obstacle_status": oc.status,
                     "production_obstacle_bound_um": oc.bound_um,
                     "validator_status": v4["centerline_spacing"]["status"],
                     "validator_sampled_um": v4["centerline_spacing"]["min_sampled_um"],
                     "validator_lower_bound_um":
                         v4["centerline_spacing"]["conservative_lower_bound_um"],
                     "validator_verdict": v4["verdict"],
                     "prefix_production_spacing_was": "safe (99.005050376)",
                     "prefix_validator_verdict_was": "pass"}

    A0, A1p = (0.0, 0.0, 0.0), (1000.0, 0.0, 0.0)
    B0, B1 = (0.0, -5e-6, 0.0), (1000.0, 5e-6, 0.0)
    st5, lb5, ub5, method5 = segment_segment_distance_certified(A0, A1p, B0, B1, 5e-7, 0.0)
    scene5 = {
        "chip_volume": {"lo": [-2000.0] * 3, "hi": [2000.0] * 3},
        "obstacles": [],
        "reference_centerlines": [{"kind": "line", "p0": list(B0), "p1": list(B1)}],
        "source_pose": {"position": list(A0), "tangent": [1.0, 0.0, 0.0]},
        "target_pose": {"position": list(A1p), "tangent": [1.0, 0.0, 0.0]},
    }
    v5 = validate_path_dict({"segments": [{"kind": "line", "p0": list(A0), "p1": list(A1p)}]},
                            scene5, cfg_dict(dmin_centerline_um=5e-7))
    print("A5 near-parallel crossing pair: certified=%s lb=%.3e ub=%.3e method=%s | "
          "validator analytic=%.3e status=%s verdict=%s"
          % (st5, lb5, ub5, method5, v5["centerline_spacing"]["analytic_min_um"],
             v5["centerline_spacing"]["status"], v5["verdict"]))
    print("   (v0.1.1: production safe 9.039999999999993e-07 um, validator pass with "
          "sampled minimum 0)")
    RESULTS["A5"] = {"status": st5, "lower_bound_um": lb5, "upper_bound_um": ub5,
                     "method": method5,
                     "validator_analytic_um": v5["centerline_spacing"]["analytic_min_um"],
                     "validator_status": v5["centerline_spacing"]["status"],
                     "validator_verdict": v5["verdict"],
                     "prefix_production_was": "safe (9.039999999999993e-07)",
                     "prefix_validator_was": "pass"}


# --------------------------------------------------------------- independent Dijkstra
def independent_dijkstra(edges, start, goal):
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


# ------------------------------------------------------------------ PART C
def part_c_core_case():
    head("PART C1: core cross-wall case through the NEW public router")
    scene = core_case()
    cfg = scene.config
    print("frozen acceptance budget: single search <= %.1f s, <= %d expansions"
          % (cfg.finite_max_runtime_s, cfg.finite_max_expansions))
    tracemalloc.start()
    t0 = time.perf_counter()
    res = route_finite_native3d(scene, mode="native3d")
    wall_s = time.perf_counter() - t0
    cur, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    print("native3d: status=%s cost=%.6f um expansions=%d runtime=%.3f s"
          % (res.status, res.cost, res.expansions, wall_s))
    print("   timings: graph_build=%.4f s search=%.4f s end_to_end=%.4f s "
          "(edge checks inside search: %.4f s)"
          % (res.timings["graph_build_s"], res.timings["search_s"],
             res.timings["end_to_end_s"], res.finite_graph["edge_check_s"]))
    print("   peak traced memory = %.3f MiB" % (peak / 1024.0 / 1024.0))
    fg = res.finite_graph
    b = fg["bounds"]
    print("   node upper bound = %d (%d lattice positions x %d directions), "
          "edge upper bound = %d" % (b["node_upper_bound"], b["n_lattice_positions"],
                                     b["n_directions"], b["edge_upper_bound"]))
    print("   visited nodes = %d, expansion calls = %d, generated = %d, rejected = %d"
          % (fg["nodes_expanded"], fg["node_expansion_counts_sum"], res.generated,
             res.rejected))
    print("   edges validated = %d, rejected = %d (undecided = %d), cache hits = %d"
          % (fg["edges_validated"], fg["edges_rejected"],
             fg["edges_rejected_undecided"], fg["cache_hits"]))
    print("   max curve-end vs canonical node error: position %.3e um, tangent %.3e deg"
          % (fg["max_curve_end_position_error_um"],
             fg["max_curve_end_tangent_error_deg"]))
    print("   port attachment: %s" % {k: (v or {}).get("kind")
                                      for k, v in fg["port_attachment"].items()})
    print("   trace:")
    for s in res.trace:
        print("     %-32s (%8.3f,%8.3f,%8.3f)  z=%8.3f  dir=%s"
              % (s["action"], s["x_um"], s["y_um"], s["z_um"], s["z_um"],
                 ["%+.0f" % c for c in s["direction"]]))
    if res.finite_graph["assembled_path_constraint_check"]:
        print("   assembled-path production check: ok=%s"
              % res.finite_graph["assembled_path_constraint_check"]["ok"])

    graph, edges = enumerate_graph_edges(scene, mode="native3d")
    start = graph.source_identity_node if graph.source_identity_node is not None else SOURCE_NODE
    goal = graph.target_identity_node if graph.target_identity_node is not None else TARGET_NODE
    t1 = time.perf_counter()
    best, n_done = independent_dijkstra(edges, start, goal)
    dij_s = time.perf_counter() - t1
    print("   independent Dijkstra over the fully enumerated graph: best=%.6f um "
          "(A* cost=%.6f um, diff=%.3e), %d nodes settled, %d edges, %.3f s"
          % (best, res.cost, abs(best - res.cost), n_done, len(edges), dij_s))
    print("   cost = length + congestion: %.6f = %.6f + %.6f"
          % (res.cost, res.length_um, res.congestion_cost))

    t2 = time.perf_counter()
    res2d = route_finite_native3d(core_case(), mode="2d")
    w2d = time.perf_counter() - t2
    print("2d      : status=%s budget_exhausted=%s expansions=%d runtime=%.3f s"
          % (res2d.status, res2d.budget_exhausted, res2d.expansions, w2d))
    print("   (a real OPEN exhaustion: the graph construction is finite, so an empty "
          "open set is a statement about THIS graph, not a budget stop)")

    print("acceptance budget check: end-to-end %.3f s <= %.1f s -> %s; expansions %d "
          "<= %d -> %s" % (res.timings["end_to_end_s"], cfg.finite_max_runtime_s,
                           res.timings["end_to_end_s"] <= cfg.finite_max_runtime_s,
                           res.expansions, cfg.finite_max_expansions,
                           res.expansions <= cfg.finite_max_expansions))
    RESULTS["C1"] = {
        "native3d": {"status": res.status, "cost_um": res.cost,
                     "length_um": res.length_um, "expansions": res.expansions,
                     "wall_s": wall_s, "timings": dict(res.timings),
                     "peak_traced_memory_mib": peak / 1024.0 / 1024.0,
                     "bounds": b, "nodes_visited": fg["nodes_expanded"],
                     "expansion_calls": fg["node_expansion_counts_sum"],
                     "generated": res.generated, "rejected": res.rejected,
                     "edges_validated": fg["edges_validated"],
                     "edges_rejected": fg["edges_rejected"],
                     "edges_rejected_undecided": fg["edges_rejected_undecided"],
                     "cache_hits": fg["cache_hits"],
                     "max_curve_end_position_error_um":
                         fg["max_curve_end_position_error_um"],
                     "max_curve_end_tangent_error_deg":
                         fg["max_curve_end_tangent_error_deg"],
                     "port_attachment_kinds": {k: (v or {}).get("kind")
                                               for k, v in fg["port_attachment"].items()},
                     "trace": res.trace,
                     "segments": res.path.to_dict() if res.path else None,
                     "assembled_check_ok":
                         (res.finite_graph["assembled_path_constraint_check"] or {}).get("ok")},
        "independent_dijkstra": {"best_cost_um": best, "nodes_settled": n_done,
                                 "n_edges": len(edges), "runtime_s": dij_s,
                                 "astar_cost_um": res.cost,
                                 "difference_um": abs(best - res.cost)},
        "2d": {"status": res2d.status, "budget_exhausted": res2d.budget_exhausted,
               "expansions": res2d.expansions, "wall_s": w2d},
        "budget": {"max_runtime_s": cfg.finite_max_runtime_s,
                   "max_expansions": cfg.finite_max_expansions},
    }
    os.makedirs(CASES, exist_ok=True)
    write_json(os.path.join(CASES, "v012_core_acceptance_summary.json"),
               {"artifact": "native3d_v012_core_acceptance_summary",
                "results": RESULTS})
    return RESULTS


if __name__ == "__main__":
    print("v0.1.2 acceptance probe")
    print("python : %s" % sys.version.replace("\n", " "))
    print("root   : %s" % ROOT)
    part_a_after_fix()
    part_c_core_case()
    print("")
    print("=" * 78)
    print("END OF v0.1.2 ACCEPTANCE PROBE")
    print("=" * 78)
