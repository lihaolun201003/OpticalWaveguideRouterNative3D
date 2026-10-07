"""Regenerate the core acceptance-case evidence under artifacts/v012/cases/.

    cd OpticalWaveguideRouterNative3D
    .venv Scripts python.exe examples/run_core_case.py

Runs, on the SAME core geometry, all three strategies:

* the v0.1.2 PRODUCTION finite multi-hop router (search/finite_graph.py) in
  mode='native3d' (success, inside the frozen 60 s / 200000 expansion budget) and
  mode='2d' (a real search_exhausted, never a budget stop);
* the FINITE canonical reference graph (search/reference_graph.py, v0.1.1
  two-hop) in both modes, kept for comparison;
* the continuous-pose debug search with a disclosed budget (it is expected to hit
  the budget on this scene; a budget stop is never unreachability);
* the initial-state action probe of every strategy.

Output directory is parameterised (NATIVE3D_ARTIFACT_DIR, default
artifacts/v012/cases) so that the frozen v0.1 and v0.1.1 evidence is never
overwritten.
"""

import json
import os
import sys
import time
import tracemalloc

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.cases import core_case                              # noqa: E402
from native3d.io.artifacts import (artifact_dir, build_artifact,  # noqa: E402
                                   finalise_artifact)
from native3d.io.caseio import write_json                         # noqa: E402
from native3d.search.finite_graph import route_finite_native3d    # noqa: E402
from native3d.search.reference_graph import reference_search      # noqa: E402
from native3d.search.router import route                          # noqa: E402

ART = artifact_dir("cases")
RERUN = "examples/run_core_case.py  (from the project root)"
PROD_BUDGET = {"max_expansions": 20000, "max_runtime_s": 120.0}


def main() -> int:
    os.makedirs(ART, exist_ok=True)
    out = {}

    # ------------------------------------------------ v0.1.2 PRODUCTION router
    for mode, fname in (("native3d", "core_case_finite_native3d_native3d.json"),
                        ("2d", "core_case_finite_native3d_2d.json")):
        scene = core_case()
        tracemalloc.start()
        t0 = time.perf_counter()
        res = route_finite_native3d(scene, mode=mode)
        dt = time.perf_counter() - t0
        _cur, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        finalise_artifact(
            build_artifact(scene, res, mode + "_finite_native3d", RERUN,
                           extra={"v012_production_budget": {
                               "max_expansions": scene.config.finite_max_expansions,
                               "max_runtime_s": scene.config.finite_max_runtime_s},
                               "peak_traced_memory_mib": peak / 1024.0 / 1024.0}),
            os.path.join(ART, fname))
        out["finite_" + mode] = {
            "status": res.status, "expansions": res.expansions,
            "runtime_s": round(dt, 3), "timings": dict(res.timings),
            "cost_um": (None if res.cost == float("inf") else res.cost),
            "peak_traced_memory_mib": round(peak / 1024.0 / 1024.0, 4),
            "node_upper_bound": res.finite_graph["bounds"]["node_upper_bound"]}
        print("finite     %-8s status=%-16s expansions=%6d runtime=%7.3fs cost=%s"
              % (mode, res.status, res.expansions, dt,
                 "-" if res.path is None else "%.6f um" % res.cost))
        print("           timings graph_build=%.4fs search=%.4fs end_to_end=%.4fs, "
              "peak traced memory %.3f MiB"
              % (res.timings["graph_build_s"], res.timings["search_s"],
                 res.timings["end_to_end_s"], peak / 1024.0 / 1024.0))

    # ------------------------------------------- v0.1.1 finite reference graph
    for mode, fname in (("2d", "core_case_reference_graph_2d.json"),
                        ("native3d", "core_case_reference_graph_native3d.json")):
        scene = core_case()
        t0 = time.perf_counter()
        res = reference_search(scene, mode=mode, max_expansions=20000,
                               max_runtime_s=600.0)
        dt = time.perf_counter() - t0
        finalise_artifact(build_artifact(scene, res, mode + "_reference_graph", RERUN),
                          os.path.join(ART, fname))
        out["reference_" + mode] = {
            "status": res.status, "expansions": res.expansions,
            "runtime_s": round(dt, 3),
            "cost_um": (None if res.cost == float("inf") else res.cost),
            "n_canonical_mids": (res.reference_graph or {}).get("n_canonical_mids")}
        print("reference  %-8s status=%-16s expansions=%6d runtime=%7.3fs cost=%s"
              % (mode, res.status, res.expansions, dt,
                 "-" if res.path is None else "%.6f um" % res.cost))

    # --------------------------------------------- continuous debug strategy
    for mode, fname in (("2d", "core_case_2d_mode.json"),
                        ("native3d", "core_case_native3d_mode.json")):
        scene = core_case()
        t0 = time.perf_counter()
        res = route(scene, mode=mode, max_expansions=PROD_BUDGET["max_expansions"],
                    max_runtime_s=PROD_BUDGET["max_runtime_s"])
        dt = time.perf_counter() - t0
        finalise_artifact(build_artifact(scene, res, mode, RERUN,
                                         extra={"production_budget": PROD_BUDGET}),
                          os.path.join(ART, fname))
        out["continuous_" + mode] = {"status": res.status,
                                     "expansions": res.expansions,
                                     "runtime_s": round(dt, 3)}
        print("continuous %-8s status=%-16s expansions=%6d runtime=%7.3fs"
              % (mode, res.status, res.expansions, dt))

    # ---------------------------------------------------- initial-state probe
    results = {}
    for name, fn in (("continuous_2d", lambda: route(core_case(), mode="2d",
                                                     max_expansions=1,
                                                     max_runtime_s=30.0)),
                     ("continuous_native3d", lambda: route(core_case(), mode="native3d",
                                                           max_expansions=1,
                                                           max_runtime_s=30.0)),
                     ("finite_2d", lambda: route_finite_native3d(core_case(), mode="2d")),
                     ("finite_native3d", lambda: route_finite_native3d(core_case(),
                                                                       mode="native3d"))):
        results[name] = fn()
    probe = {
        "artifact": "native3d_v012_initial_state_action_probe",
        "note": ("candidate actions generated from the INITIAL state of each strategy, "
                 "before any search expansion"),
        "input": core_case().summary(),
        "self_spacing_check": ("NOT IMPLEMENTED in the acceptance path (v0.1.2 "
                               "erratum): constraints.checks.path_self_spacing() is "
                               "never called by the router, the result assembly or the "
                               "independent validator"),
        "strategies": {
            name: {"status": r.status,
                   "n_candidates": len(r.initial_candidates),
                   "n_accepted_z_changing": sum(
                       1 for c in r.initial_candidates
                       if c["accepted"] and abs(c["z_change_um"]) > 1e-9),
                   "candidates": r.initial_candidates}
            for name, r in results.items()},
    }
    write_json(os.path.join(ART, "core_case_initial_action_probe.json"), probe)
    print("initial-state z-changing feasible candidates:")
    for name, d in probe["strategies"].items():
        print("   %-22s %d of %d" % (name, d["n_accepted_z_changing"], d["n_candidates"]))
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
