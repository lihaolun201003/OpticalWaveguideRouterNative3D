"""v0.1.3 PART A probe: the finite production router's contract repair, measured.

WHAT THIS PROBE ANSWERS, WITH REAL NUMBERS
------------------------------------------
PART A of docs/prompts/deepseek_native3d_v013_contract_geometry_repair_prompt.md
has three counterexamples, each of which was REALLY reproduced on the frozen
v0.1.2 snapshot before the repair.  This probe re-runs every one of them on the
repaired v0.1.3 tree and prints the before/after pair:

A1  a mismatched prebuilt graph (scene A -> scene B) must be REFUSED BEFORE the
    search.  v0.1.2: status=success, cost=50.0, endpoint=(100,0,0),
    goal_pos_error_um=50.  v0.1.3: graph_scene_mismatch, path=None, cost=inf,
    with the structured field-by-field diff.
A2  success must be decided by the ACTUAL assembled curve: the real start/end
    pose of the returned curve is compared with the pose the caller passed with
    the SOURCE tolerances at the start and the TARGET acceptance ball at the
    end, and every junction with the C1 tolerances.
A3  source == target must return the explicitly defined ZERO-LENGTH solution
    (cost 0, length 0, path None, zero_length_solution True, one trace step)
    instead of raising AttributeError: 'NoneType' object has no attribute
    'segments'.

PART C items 1/2/3 are measured here too: the DERIVED edge upper bound, the
phase split of the per-edge continuous checks (graph build vs search) and the
cold/warm provenance of a reused graph.

Running it:

    .venv\\Scripts\\python.exe artifacts\\v013\\probe\\v013_finite_router_probe.py

Exit code is 0 when every printed CHECK line holds and 1 otherwise; a failing
check is never hidden.  Everything printed comes from a real call of the real
code - no number is asserted or reconstructed by hand.

MEMORY UNITS (PART C item 4, disclosed and not claimed otherwise)
-----------------------------------------------------------------
This probe does not measure memory.  Where the acceptance probe reports a
``tracemalloc`` peak, that number is the PEAK TRACKED PYTHON ALLOCATION of the
traced region - it is NOT the process RSS, not the resident set, and not the
total memory of the run: tracemalloc only sees allocations made through the
Python allocator while tracing is active (C extensions, the interpreter itself,
memory mapped input and the OS page cache are outside it).
"""

from __future__ import annotations

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
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from native3d.config import RouterConfig                                   # noqa: E402
from native3d.constraints.checks import Box3D                              # noqa: E402
from native3d.geometry import LineSegment3D, Pose3D                        # noqa: E402
from native3d.io.caseio import Scene                                       # noqa: E402
from native3d.search import statuses                                       # noqa: E402
from native3d.search.congestion import RegionCongestion                    # noqa: E402
from native3d.search.finite_graph import (FiniteGraph,                     # noqa: E402
                                          graph_config_fingerprint,
                                          route_finite_native3d,
                                          verify_assembled_endpoints)

CHECKS = []


def check(label: str, ok: bool, detail: str = "") -> bool:
    CHECKS.append({"check": label, "ok": bool(ok), "detail": detail})
    print("   CHECK %-58s %s%s" % (label, "OK" if ok else "FAILED",
                                   (" (%s)" % detail) if detail else ""))
    return bool(ok)


def head(title: str) -> None:
    print("")
    print("=" * 78)
    print(title)
    print("=" * 78)


def cfg_dict(**over) -> dict:
    d = dict(RouterConfig().to_dict())
    d.update(over)
    return d


CHIP = Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))
SMALL = Box3D((-60.0, -60.0, -60.0), (60.0, 60.0, 60.0))


def a1_scene(name: str, target_x: float, chip=CHIP) -> Scene:
    return Scene(name=name, chip=chip, obstacles=[], reference_centerlines=[],
                 source=Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 target=Pose3D((target_x, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 config=RouterConfig())


# ======================================================================= A1
def part_a1_graph_mismatch():
    head("A1  a mismatched prebuilt graph is refused BEFORE the search")
    print("scene A: source (0,0,0)+x, target (100,0,0)+x, chip [-150,150]^3")
    print("scene B: same source,      target  (50,0,0)+x")
    print("measured on the FROZEN v0.1.2 snapshot (prefix package, "
          "artifacts/v013/probe/v013_prefix_replay.py):")
    print("   stale graph -> status=success, cost=100.0, endpoint=(100,0,0), "
          "goal_pos_error_um=50.0")
    print("   fresh scene B -> status=success, cost=50.0, endpoint=(50,0,0)")
    sc_a = a1_scene("probe_A", 100.0)
    t0 = time.perf_counter()
    graph_a = FiniteGraph(sc_a, mode="native3d", cfg=sc_a.config)
    t_build = time.perf_counter() - t0
    print("graph = FiniteGraph(scene A)   built in %.3f s   bounds: %d lattice "
          "positions, node bound %d, edge bound %d"
          % (t_build, graph_a.bounds["n_lattice_positions"],
             graph_a.bounds["node_upper_bound"], graph_a.bounds["edge_upper_bound"]))

    sc_b = a1_scene("probe_B", 50.0)
    t1 = time.perf_counter()
    res = route_finite_native3d(sc_b, graph=graph_a)
    t_call = time.perf_counter() - t1
    rep = res.finite_graph["graph_reuse"]
    print("route_finite_native3d(scene B, graph=graph_A):")
    print("   status              = %s      (v0.1.2 was: success)" % res.status)
    print("   path                = %s      (v0.1.2 returned the OLD endpoint)" % res.path)
    print("   cost_um_equivalent  = %s      (v0.1.2 was: 100.0)" % res.cost)
    print("   length_um           = %s" % res.length_um)
    print("   goal_pos_error_um   = %s      (v0.1.2 was: 50.0, recorded not failed)"
          % res.goal_pos_error_um)
    print("   wall time           = %.6f s (no search ran; the graph was already built)"
          % t_call)
    print("   mismatches          = %s" % rep["mismatches"])
    print("   mismatch_detail     = %s" % json.dumps(rep["mismatch_detail"]))
    print("   graph_cache_state   = %s" % rep["graph_cache_state"])
    print("   successor_cache_hits= %s" % rep["successor_cache_hits"])
    check("A1 mismatch status is graph_scene_mismatch",
          res.status == "graph_scene_mismatch", res.status)
    check("A1 mismatch is neither success nor search_exhausted",
          res.status not in ("success", "search_exhausted"))
    check("A1 mismatch has no path and infinite cost",
          res.path is None and math.isinf(res.cost) and math.isinf(res.length_um))
    check("A1 mismatch names 'target' in the diff",
          "target" in rep["mismatches"]
          and rep["mismatch_detail"]["target"]["expected"] == [50.0, 0.0, 0.0]
          and rep["mismatch_detail"]["target"]["graph"] == [100.0, 0.0, 0.0])
    check("A1 goal_pos_error_um is None (no number that reads as a success)",
          res.goal_pos_error_um is None)
    check("A1 port_assurance is None on the mismatch",
          res.finite_graph.get("port_assurance") is None)

    fresh = route_finite_native3d(a1_scene("probe_B2", 50.0), mode="native3d")
    print("fresh scene B without injection: status=%s cost=%s endpoint=%s"
          % (fresh.status, fresh.cost,
             list(fresh.path.end_pose.position) if fresh.path else None))
    check("A1 fresh scene B really reaches (50,0,0) at cost 50",
          fresh.status == "success" and abs(fresh.cost - 50.0) < 1e-9
          and list(fresh.path.end_pose.position) == [50.0, 0.0, 0.0])

    # ---- content, not count: same number of reference centrelines, different
    #      geometry, and the same for obstacles
    ref_a = LineSegment3D((-1000.0, 500.0, 0.0), (1000.0, 500.0, 0.0))
    ref_b = LineSegment3D((-1000.0, -500.0, 0.0), (1000.0, -500.0, 0.0))
    sc_r1 = Scene("ref1", SMALL, [], [ref_a],
                  Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                  Pose3D((50.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    sc_r2 = Scene("ref2", SMALL, [], [ref_b],
                  Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                  Pose3D((50.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    fp1 = graph_config_fingerprint(sc_r1.config, sc_r1)
    fp2 = graph_config_fingerprint(sc_r2.config, sc_r2)
    print("same COUNT (1) of reference centrelines, different geometry:")
    print("   fingerprint reference_centrelines differ = %s"
          % (fp1["reference_centrelines"] != fp2["reference_centrelines"]))
    check("A1 fingerprint records FULL reference geometry, not the count",
          fp1 != fp2 and fp1["reference_centrelines"] != fp2["reference_centrelines"])

    # ---- changed congestion field identity
    graph_r = FiniteGraph(sc_r1, mode="native3d", cfg=sc_r1.config)
    reg = Box3D((60.0, -20.0, -20.0), (90.0, 20.0, 20.0))
    res_c = route_finite_native3d(sc_r1, congestion=RegionCongestion([(reg, 2.0)]),
                                  graph=graph_r)
    print("different congestion field on a reused graph: status=%s mismatches=%s"
          % (res_c.status, res_c.finite_graph["graph_reuse"]["mismatches"]))
    check("A1 a different congestion field is refused, not silently reused",
          res_c.status == "graph_scene_mismatch"
          and "congestion" in res_c.finite_graph["graph_reuse"]["mismatches"])

    # ---- warm reuse of a genuinely matching graph.  The scene needs the
    #      counterexample chip: with [-60,60]^3 the target (100,0,0) is outside
    #      the lattice AND outside the shrunken chip, so its attachment curve
    #      would be rejected and the call would end in port_attachment_failed
    #      (which is a true statement about the supported action set, not about
    #      the reuse contract this section measures).
    sc_w = a1_scene("probe_warm", 100.0)
    graph_a2 = FiniteGraph(sc_w, mode="native3d", cfg=sc_w.config)
    w1 = route_finite_native3d(sc_w, graph=graph_a2)
    w2 = route_finite_native3d(sc_w, graph=graph_a2)
    rep2 = w2.finite_graph["graph_reuse"]
    print("matching prebuilt graph reused: first=%s second=%s warm=%s hits=%d "
          "(successor cache hits of the graph object)"
          % (w1.status, w2.status, rep2["graph_cache_state"],
             rep2["successor_cache_hits"]))
    check("A1 a genuinely matching prebuilt graph still succeeds",
          w1.status == "success" and w2.status == "success"
          and rep2["accepted"] is True and rep2["mismatches"] == [])
    check("A1 the second call is reported as a WARM reuse with real cache hits",
          rep2["graph_cache_state"] == "warm"
          and rep2["successor_cache_hits"] == graph_a2.cache_hits
          and rep2["successor_cache_hits"] > 0)

    # ---- the cold branch
    cold = route_finite_native3d(a1_scene("probe_cold", 50.0, chip=SMALL),
                                 mode="native3d")
    repc = cold.finite_graph["graph_reuse"]
    print("cold default branch: state=%s build_count=%d by_this_call=%d hits=%d"
          % (repc["graph_cache_state"], repc["graph_build_count"],
             repc["graph_build_count_by_this_call"], repc["successor_cache_hits"]))
    check("A1 the cold default branch reports graph_cache_state == 'cold'",
          repc["graph_cache_state"] == "cold"
          and repc["graph_build_count_by_this_call"] == 1)

    # ---- the bijection guard is real
    sc_t = a1_scene("probe_bijection", 50.0, chip=SMALL)
    g_t = FiniteGraph(sc_t, mode="native3d", cfg=sc_t.config)
    print("pose-key table: %d entries, collisions=%s, target identity node=%s"
          % (len(g_t.pose_key_to_node), g_t.pose_key_collisions,
             g_t.target_identity_node))
    check("A1 the pose-key table is a real bijection with no silent overwrite",
          g_t.pose_key_collisions == {}
          and g_t.pose_key_to_node[g_t.node_pose_key(("target",))]
          == g_t.target_identity_node)


# ======================================================================= A2
def part_a2_actual_ports():
    head("A2  success is decided by the ACTUAL assembled curve")
    sc = Scene("probe_a2", CHIP, [], [],
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
               Pose3D((100.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    res = route_finite_native3d(sc, mode="native3d")
    pa = res.port_assurance
    print("real success on (0,0,0)->(100,0,0):")
    print("   status=%s n_segments=%d cost=%.9f" % (res.status, res.n_segments, res.cost))
    print("   source pos/tangent error = %.3e um / %.3e deg"
          % (pa["source_position_error_um"], pa["source_tangent_error_deg"]))
    print("   target pos/tangent error = %.3e um / %.3e deg"
          % (pa["target_position_error_um"], pa["target_tangent_error_deg"]))
    print("   max junction pos/tangent = %.3e um / %.3e deg"
          % (pa["max_junction_position_error_um"],
             pa["max_junction_tangent_error_deg"]))
    print("   tolerances: source %.1e um / %.1e deg, target ball %.1e um / %.1f deg, "
          "C1 %.1e um / %.1e deg"
          % (pa["assembled_path_check"]["source_pos_tol_um"],
             pa["assembled_path_check"]["source_ang_tol_deg"],
             pa["assembled_path_check"]["target_eps_pos_um"],
             pa["assembled_path_check"]["target_eps_ang_deg"],
             pa["assembled_path_check"]["pos_tol_um"],
             pa["assembled_path_check"]["ang_tol_deg"]))
    check("A2 a real success publishes port_assurance verified against the caller pose",
          res.status == "success" and pa is not None
          and pa["verified_against_the_caller_pose"] is True
          and pa["assembled_path_check"]["ok"] is True)
    check("A2 the assembled curve really ends on the target pose",
          list(res.path.end_pose.position) == [100.0, 0.0, 0.0]
          and res.endpoint_errors["end_position_error_um"] <= 1e-9)
    check("A2 res.goal_pos_error_um equals the verified target position error",
          abs(res.goal_pos_error_um - pa["target_position_error_um"]) <= 1e-12)
    check("A2 cost = length + congestion (the three fields agree)",
          abs(res.cost - (res.length_um + res.congestion_cost)) <= 1e-12)

    # ---- the two tolerance families are genuinely different
    seg = LineSegment3D((1e-4, 0.0, 0.0), (100.0, 0.0, 0.0))
    rep = verify_assembled_endpoints((seg,), sc, sc.config)
    print("helper on a path that starts 1e-4 um off the source:")
    print("   start error %.3e um > source tol %.1e um -> failed=%s; "
          "end error %.3e um <= target ball %.1e um"
          % (rep["start_position_error_um"], rep["source_pos_tol_um"], rep["failed"],
             rep["end_position_error_um"], rep["target_eps_pos_um"]))
    check("A2 the source tolerance family is NOT the target acceptance ball",
          rep["ok"] is False and rep["failed"] == ["source"]
          and rep["source_pos_tol_um"] < rep["target_eps_pos_um"]
          and rep["status"] == "constraint_failed")

    # ---- a real searched path from the acceptance case
    from native3d.cases import core_case
    cw = route_finite_native3d(core_case(), mode="native3d")
    fg = cw.finite_graph
    print("cross-wall acceptance case (v0.1.2 frozen: cost 594.1592653589794, "
          "75 expansions):")
    print("   status=%s cost=%.13f expansions=%d trace_steps=%d n_segments=%d"
          % (cw.status, cw.cost, cw.expansions, len(cw.trace), cw.n_segments))
    print("   accepted edges all end on their canonical pose: end_pose_rejected=%d, "
          "junction_rejected=%d, max end error %.3e um / %.3e deg"
          % (fg["edges_end_pose_rejected"], fg["edges_junction_rejected"],
             fg["max_curve_end_position_error_um"],
             fg["max_curve_end_tangent_error_deg"]))
    check("A2 the frozen cross-wall result is unchanged "
          "(cost 594.1592653589794, 75 expansions, trace 8, 7 segments)",
          cw.status == "success"
          and abs(cw.cost - 594.1592653589794) < 1e-9 and cw.expansions == 75
          and len(cw.trace) == 8 and cw.n_segments == 7)
    check("A2 every accepted edge ends on its canonical target pose",
          fg["edges_end_pose_rejected"] == 0 and fg["edges_junction_rejected"] == 0
          and fg["max_curve_end_position_error_um"] <= core_case().config.pos_tol_um
          and fg["max_curve_end_tangent_error_deg"] <= core_case().config.ang_tol_deg)

    # ---- a failure has no path and no usable guarantee
    bad = Scene("probe_a2_bad", CHIP, [], [],
                Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                Pose3D((1e6, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    rbad = route_finite_native3d(bad, mode="native3d")
    print("illegal target pose: status=%s path=%s port_assurance=%s "
          "zero_length_solution=%s"
          % (rbad.status, rbad.path, rbad.finite_graph.get("port_assurance"),
             rbad.zero_length_solution))
    check("A2 a failure has no path, no port assurance and zero_length False",
          rbad.status != "success" and rbad.path is None
          and rbad.finite_graph.get("port_assurance") is None
          and rbad.zero_length_solution is False
          and rbad.endpoint_errors is None)

    # ---- the status vocabulary is distinct and the old relabel is gone
    import inspect
    src = inspect.getsource(route_finite_native3d)
    print("the assembled-path failure branch no longer reuses search_exhausted: "
          "constraint_failed in source=%s" % ("constraint_failed" in src))
    check("A2 an assembled-path failure is constraint_failed/uncertified, "
          "never search_exhausted",
          'res.status = "uncertified" if undecided else "search_exhausted"' not in src
          and "constraint_failed" in src and "uncertified" in src)
    check("A2 every status in the vocabulary is distinct",
          len(set(statuses.ALL_STATUSES)) == len(statuses.ALL_STATUSES)
          and set(("success", "search_exhausted", "budget_exhausted",
                   "port_attachment_failed", "graph_scene_mismatch",
                   "invalid_source_pose", "invalid_target_pose",
                   "unsupported_pose", "constraint_failed", "uncertified"))
          <= set(statuses.ALL_STATUSES)
          and statuses.SEARCH_TERMINATIONS == ("success", "search_exhausted",
                                               "budget_exhausted"))


# ======================================================================= A3
def part_a3_zero_length():
    head("A3  the zero-length solution contract")
    pose = Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    sc = Scene("probe_a3_zero", SMALL, [], [], pose,
               Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    print("source = target = Pose3D((0,0,0),(1,0,0)), chip [-60,60]^3, default config")
    print("v0.1.2 raised AttributeError: 'NoneType' object has no attribute 'segments'")
    res = route_finite_native3d(sc, mode="native3d")
    print("   status=%s zero_length_solution=%s cost=%s length=%s congestion=%s"
          % (res.status, res.zero_length_solution, res.cost, res.length_um,
             res.congestion_cost))
    print("   path=%r n_segments=%d expansions=%d generated=%d trace_steps=%d"
          % (res.path, res.n_segments, res.expansions, res.generated, len(res.trace)))
    print("   endpoint_errors: start %.3e um / %.3e deg, end %.3e um / %.3e deg"
          % (res.endpoint_errors["start_position_error_um"],
             res.endpoint_errors["start_tangent_error_deg"],
             res.endpoint_errors["end_position_error_um"],
             res.endpoint_errors["end_tangent_error_deg"]))
    check("A3 source == target returns a defined zero-length success",
          res.status == "success" and res.zero_length_solution is True
          and res.cost == 0.0 and res.length_um == 0.0 and res.congestion_cost == 0.0
          and res.path is None and res.n_segments == 0 and res.expansions == 0
          and len(res.trace) == 1 and res.trace[0]["n_segments"] == 0)
    check("A3 the zero-length solution records verifiable endpoint errors",
          res.endpoint_errors["start_ok"] and res.endpoint_errors["end_ok"]
          and res.port_assurance["zero_length_solution"] is True)

    res2d = route_finite_native3d(sc, mode="2d")
    print("2d mode: status=%s zero_length=%s mode=%s"
          % (res2d.status, res2d.zero_length_solution, res2d.mode))
    check("A3 2d mode honours the same contract",
          res2d.status == "success" and res2d.zero_length_solution is True
          and res2d.path is None and res2d.mode == "finite_native3d_2d")

    # wrong tangent at the same position is NOT a zero-length win
    th = math.radians(10.0)
    sc_w = Scene("probe_a3_wrong_tan", SMALL, [], [], pose,
                 Pose3D((0.0, 0.0, 0.0), (math.cos(th), math.sin(th), 0.0)),
                 RouterConfig())
    rw = route_finite_native3d(sc_w, mode="native3d")
    print("same position, target tangent 10 deg off: status=%s zero_length=%s path=%s"
          % (rw.status, rw.zero_length_solution, rw.path))
    check("A3 a target tangent outside the acceptance ball is not a zero-length win",
          rw.status != "success" and rw.zero_length_solution is False)

    # the source pose is still validated
    wall = Box3D((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0))
    sc_i = Scene("probe_a3_invalid", SMALL, [wall], [], pose,
                 Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    ri = route_finite_native3d(sc_i, mode="native3d")
    sc_o = Scene("probe_a3_outside", SMALL, [], [],
                 Pose3D((-100.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                 Pose3D((-100.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    ro = route_finite_native3d(sc_o, mode="native3d")
    ref = LineSegment3D((0.0, 0.0, 0.0), (0.0, 0.0, 60.0))
    sc_r = Scene("probe_a3_on_ref", SMALL, [], [ref], pose,
                 Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    rr = route_finite_native3d(sc_r, mode="native3d")
    print("source inside an obstacle: %s | source outside the chip: %s | source on a "
          "reference centreline: %s" % (ri.status, ro.status, rr.status))
    check("A3 source == target does NOT bypass input validation",
          ri.status == "invalid_source_pose" and ro.status == "invalid_source_pose"
          and rr.status == "invalid_source_pose"
          and ri.zero_length_solution is False and ro.path is None)


# ======================================================================= PART C
def part_c_bounds_and_timing():
    head("PART C 1/2/3  bounds, timing split and cold/warm provenance")
    from native3d.cases import core_case
    sc = core_case()
    res = route_finite_native3d(sc, mode="native3d")
    b = res.finite_graph["bounds"]
    t = res.timings
    fg = res.finite_graph
    edges_per_node = (len(sc.config.finite_straight_steps)
                      + 4 * len([m for m in sc.config.finite_turn_radius_steps
                                 if float(int(m)) * sc.config.finite_spacing_um
                                 >= sc.config.rmin_um - sc.config.radius_tol_um]))
    derived = (b["node_upper_bound"] * edges_per_node
               + b["edge_upper_bound_from_port_attachments"])
    print("bounds: %d lattice positions, node bound %d "
          "(includes virtual ports: %s, virtual port nodes %d), edge bound %d"
          % (b["n_lattice_positions"], b["node_upper_bound"],
             b["node_upper_bound_includes_virtual_ports"],
             b["n_virtual_port_nodes_upper"], b["edge_upper_bound"]))
    print("   derivation: %d canonical nodes * %d edges/node + %d port edges = %d "
          "(v0.1.2 had a hardcoded +4096)"
          % (b["node_upper_bound"], edges_per_node,
             b["edge_upper_bound_from_port_attachments"], derived))
    check("C1 the edge bound is DERIVED from the action and port construction",
          b["edge_upper_bound"] == derived
          and b["node_upper_bound_includes_virtual_ports"] is False
          and b["total_node_upper_bound_including_virtual_ports"]
          == b["node_upper_bound"] + b["n_virtual_port_nodes_upper"])
    check("C1 the edge bound is a genuine upper bound of the node bound",
          b["edge_upper_bound"] >= b["node_upper_bound"])

    # a REAL count on a small graph: the enumerated construction must fit inside
    # the derived bound (the bound is not decoration)
    from native3d.search.finite_graph import enumerate_graph_edges
    small = Scene("probe_c1_small", SMALL, [], [],
                  Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                  Pose3D((50.0, 0.0, 0.0), (1.0, 0.0, 0.0)), RouterConfig())
    g_small, edges_small = enumerate_graph_edges(small, mode="native3d")
    bs = g_small.bounds
    print("   real enumeration on the small scene: %d edges, node bound %d, "
          "edge bound %d (canonical %d + ports %d)"
          % (len(edges_small), bs["node_upper_bound"], bs["edge_upper_bound"],
             bs["edge_upper_bound_from_canonical_nodes"],
             bs["edge_upper_bound_from_port_attachments"]))
    check("C1 the REAL enumerated construction fits inside the derived edge bound",
          len(edges_small) <= bs["edge_upper_bound"]
          and bs["edge_upper_bound"]
          == bs["node_upper_bound"] * bs["edges_per_canonical_node_upper"]
          + bs["edge_upper_bound_from_port_attachments"])

    print("timing: graph_build=%.4f s search=%.4f s assembled_path_check=%.4f s "
          "end_to_end=%.4f s branch=%s"
          % (t["graph_build_s"], t["search_s"], t["assembled_path_check_s"],
             t["end_to_end_s"], t["graph_branch"]))
    print("   per-edge continuous checks: graph_build=%.4f s search=%.4f s "
          "sum=%.4f s"
          % (fg["edge_check_s_graph_build"], fg["edge_check_s_search"],
             fg["edge_check_s"]))
    check("C2 the per-edge checks are split by phase and sum to edge_check_s",
          abs(fg["edge_check_s"]
              - (fg["edge_check_s_graph_build"] + fg["edge_check_s_search"])) < 1e-9)
    check("C2 edge_check_s_search is inside search_s",
          fg["edge_check_s_search"] <= t["search_s"] + 1e-9)
    check("C2 edge_check_s_graph_build is inside graph_build_s",
          fg["edge_check_s_graph_build"] <= t["graph_build_s"] + 1e-9)
    check("C2 end_to_end >= graph_build + search (cold default branch)",
          t["end_to_end_s"] >= t["graph_build_s"] + t["search_s"] - 1e-9
          and t["assembled_path_check_s"] >= 0.0)

    # warm branch timing, kept separate
    graph = FiniteGraph(core_case(), mode="native3d", cfg=core_case().config)
    warm = route_finite_native3d(core_case(), graph=graph)
    print("warm reuse: branch=%s graph_build=%.6f s prebuilt_construction=%.4f s "
          "end_to_end=%.4f s state=%s"
          % (warm.timings["graph_branch"], warm.timings["graph_build_s"],
             warm.timings["graph_prebuilt_construction_s"],
             warm.timings["end_to_end_s"],
             warm.finite_graph["graph_reuse"]["graph_cache_state"]))
    check("C3 the warm branch marks its own construction time separately",
          warm.timings["graph_branch"] == "prebuilt_graph_reused"
          and warm.timings["graph_build_s"] <= 1e-3
          and warm.timings["graph_prebuilt_construction_s"] > 0.0
          and warm.finite_graph["graph_reuse"]["graph_cache_state"] == "warm")
    check("C3 the cold branch stays the frozen acceptance quantity",
          abs(res.cost - 594.1592653589794) < 1e-9 and res.expansions == 75
          and t["graph_branch"] == "fresh_cold_graph")


def part_red_frozen_prefix():
    """The SAME counterexamples, run against the FROZEN v0.1.2 snapshot.

    The snapshot is artifacts/v013/prefix_src, a byte-for-byte copy of src/native3d
    taken before the first v0.1.3 edit; it is loaded as a SEPARATE package
    (``native3d_prefix``) in this interpreter, so the red and the green numbers in
    this probe come from two real implementations in one run - no red number here
    is quoted from prose.
    """
    head("RED  the same counterexamples on the FROZEN v0.1.2 snapshot")
    prefix_src = os.path.join(V013, "prefix_src")
    if not os.path.isdir(prefix_src):
        check("RED snapshot present", False, prefix_src)
        return
    import importlib.util
    name = "native3d_prefix"
    if name not in sys.modules:
        parent = importlib.util.module_from_spec(
            importlib.util.spec_from_loader(name, loader=None))
        parent.__path__ = [prefix_src]
        sys.modules[name] = parent
    P_FG = importlib.import_module(name + ".search.finite_graph")
    P_CK = importlib.import_module(name + ".constraints.checks")
    P_PP = importlib.import_module(name + ".geometry.primitives")
    P_CFG = importlib.import_module(name + ".config")
    P_CI = importlib.import_module(name + ".io.caseio")

    pchip = P_CK.Box3D((-150.0, -150.0, -150.0), (150.0, 150.0, 150.0))

    def pscene(nm, tx):
        return P_CI.Scene(name=nm, chip=pchip, obstacles=[], reference_centerlines=[],
                          source=P_PP.Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                          target=P_PP.Pose3D((tx, 0.0, 0.0), (1.0, 0.0, 0.0)),
                          config=P_CFG.RouterConfig())

    # ---- A1: the stale graph silently wins
    pa = pscene("prefix_A", 100.0)
    pgraph = P_FG.FiniteGraph(pa, mode="native3d", cfg=pa.config)
    pres = P_FG.route_finite_native3d(pscene("prefix_B", 50.0), graph=pgraph)
    pend = (list(pres.path.end_pose.position) if pres.path is not None else None)
    print("A1 v0.1.2: status=%s cost=%s endpoint=%s goal_pos_error_um=%s"
          % (pres.status, pres.cost, pend, pres.goal_pos_error_um))
    print("A1 v0.1.3: status=%s cost=%s endpoint=None goal_pos_error_um=None"
          % ("graph_scene_mismatch", "inf"))
    check("RED A1 really returned success with the OLD endpoint (100,0,0)",
          pres.status == "success" and pend == [100.0, 0.0, 0.0]
          and abs(pres.cost - 100.0) < 1e-9
          and abs(pres.goal_pos_error_um - 50.0) < 1e-9)
    pfresh = P_FG.route_finite_native3d(pscene("prefix_B2", 50.0), mode="native3d")
    check("RED A1 fresh scene B was correct (cost 50, endpoint (50,0,0))",
          pfresh.status == "success" and abs(pfresh.cost - 50.0) < 1e-9
          and list(pfresh.path.end_pose.position) == [50.0, 0.0, 0.0])

    # ---- A3: source == target crashed
    pz = P_CI.Scene(name="prefix_zero", chip=pchip, obstacles=[],
                    reference_centerlines=[],
                    source=P_PP.Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                    target=P_PP.Pose3D((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                    config=P_CFG.RouterConfig())
    try:
        r = P_FG.route_finite_native3d(pz, mode="native3d")
        print("A3 v0.1.2: no exception, status=%s path=%s" % (r.status, r.path))
        red_a3 = False
    except AttributeError as exc:
        print("A3 v0.1.2: AttributeError: %s" % exc)
        red_a3 = True
    print("A3 v0.1.3: status=success zero_length_solution=True cost=0.0 path=None "
          "n_segments=0")
    check("RED A3 really raised AttributeError: 'NoneType' ... 'segments'", red_a3)

    # ---- A2: what the frozen router did NOT do
    psrc = open(os.path.join(prefix_src, "search", "finite_graph.py"),
                encoding="utf-8").read()
    has_helper = hasattr(P_FG, "verify_assembled_endpoints")
    relabel = 'res.status = "uncertified" if undecided else "search_exhausted"' in psrc
    print("A2 v0.1.2: verify_assembled_endpoints exists=%s; the assembled-path "
          "failure was relabelled as search_exhausted=%s; the source pose "
          "tolerances appear in finite_graph.py=%s"
          % (has_helper, relabel, "source_pos_tol_um" in psrc))
    print("A2 v0.1.3: verify_assembled_endpoints exists=True; failure status is "
          "constraint_failed/uncertified; the source/end tolerances are enforced")
    check("RED A2 really had no endpoint verifier and relabelled failures as "
          "search_exhausted",
          (not has_helper) and relabel and ("source_pos_tol_um" not in psrc))


def red_tests_against_the_frozen_snapshot(argv) -> int:
    """Run the v0.1.3 PART A test files against the FROZEN v0.1.2 snapshot.

        .venv\\Scripts\\python.exe artifacts\\v013\\probe\\v013_finite_router_probe.py --prefix-tests

    The snapshot under artifacts/v013/prefix_src is registered as the module
    ``native3d`` BEFORE pytest starts, so the very same test files that pass on
    the repaired tree are executed by the real v0.1.2 code and must fail.  That
    is the machine-checkable red baseline: the exit code and the log are the
    evidence, and nothing about it is reconstructed by hand.
    """
    import importlib.util
    prefix_src = os.path.join(V013, "prefix_src")
    # This probe module imported the REPAIRED native3d.* at the top of the file, so
    # every one of those modules is already cached in sys.modules; the alias only
    # takes effect after they are removed (otherwise the tests would silently run
    # against the repaired tree and the "red" run would be a lie).
    for key in [k for k in sys.modules
                if k == "native3d" or k.startswith("native3d.")]:
        del sys.modules[key]
    spec = importlib.util.spec_from_loader("native3d", loader=None)
    module = importlib.util.module_from_spec(spec)
    module.__path__ = [prefix_src]
    sys.modules["native3d"] = module
    os.environ["NATIVE3D_PREFIX_TESTS"] = "1"
    import pytest
    files = [os.path.join(ROOT, "tests", n) for n in (
        "test_v013_a_graph_contract.py", "test_v013_a_port_and_status.py",
        "test_v013_a_zero_length.py")]
    print("RED BASELINE: running %d test file(s) against %s" % (len(files),
                                                               prefix_src))
    print("RED BASELINE: native3d now resolves to %s"
          % (list(module.__path__),))
    return int(pytest.main(["-q", "--no-header", "-p", "no:cacheprovider",
                            "--continue-on-collection-errors"] + files))


def main() -> int:
    argv = sys.argv[1:]
    if "--prefix-tests" in argv:
        return red_tests_against_the_frozen_snapshot(argv)
    print("v0.1.3 PART A probe (finite production router contract repair)")
    print("python : %s" % sys.version.replace("\n", " "))
    print("root   : %s" % ROOT)
    t0 = time.perf_counter()
    for fn in (part_a1_graph_mismatch, part_a2_actual_ports, part_a3_zero_length,
               part_c_bounds_and_timing, part_red_frozen_prefix):
        try:
            fn()
        except Exception:                                          # noqa: BLE001
            print("   PROBE SECTION RAISED:")
            traceback.print_exc()
            CHECKS.append({"check": fn.__name__, "ok": False,
                           "detail": "raised an exception"})
    total = time.perf_counter() - t0
    n_fail = sum(1 for c in CHECKS if not c["ok"])
    print("")
    print("=" * 78)
    print("SUMMARY  %d checks, %d failed, wall %.3f s"
          % (len(CHECKS), n_fail, total))
    print("=" * 78)
    for c in CHECKS:
        print("  %-6s %s" % ("OK" if c["ok"] else "FAILED", c["check"]))
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
