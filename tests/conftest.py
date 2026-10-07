"""Session fixtures: run the core acceptance case ONCE with every strategy.

v0.1.2 keeps three explicitly separated strategies and the fixture runs the ones
that carry acceptance evidence:

* route()  (continuous pose debug search) - budget-bounded here; its status
  semantics (a budget stop is never unreachability) are checked, and this graph
  is generally infinite so it is not the reachability evidence;
* reference_search() - the v0.1.1 two-hop finite canonical reference graph, kept
  for comparison;
* route_finite_native3d() - the v0.1.2 PRODUCTION finite multi-hop integer-node
  router, which really solves the cross-wall case and really exhausts in 2D mode.

The pytest run regenerates the machine-readable evidence under artifacts/v012/ so
that a fresh checkout reproduces it with

    .venv Scripts python.exe -m pytest      (from the project root)
"""

import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src"))

from native3d.cases import core_case                      # noqa: E402
from native3d.io.artifacts import (artifact_dir, build_artifact,      # noqa: E402
                                   finalise_artifact)
from native3d.io.caseio import write_json                 # noqa: E402
from native3d.search.finite_graph import route_finite_native3d  # noqa: E402
from native3d.search.reference_graph import reference_search   # noqa: E402
from native3d.search.router import route                  # noqa: E402

# Parameterised output location: the frozen v0.1 / v0.1.1 evidence under
# artifacts/cases/, artifacts/v011/ and artifacts/v011_preserved/ is NEVER
# written by a v0.1.2 run.  Override with NATIVE3D_ARTIFACT_DIR.
ARTIFACT_DIR = artifact_dir("cases")
RERUN = "pytest  (from the project root, with the project venv)"
# Disclosed production budget: the continuous-pose graph cannot be exhausted on
# the core scene, so these runs exist to pin the status semantics, not to decide.
PROD_MAX_EXP = 1500
PROD_MAX_RT = 8.0


@pytest.fixture(scope="session")
def core_case_run():
    """Same engine, same geometry: every strategy on the same core scene."""
    scene_2d = core_case()
    scene_3d = core_case()
    res_2d = route(scene_2d, mode="2d", max_expansions=PROD_MAX_EXP,
                   max_runtime_s=PROD_MAX_RT)
    res_3d = route(scene_3d, mode="native3d", max_expansions=PROD_MAX_EXP,
                   max_runtime_s=PROD_MAX_RT)
    ref_2d = reference_search(core_case(), mode="2d")
    ref_3d = reference_search(core_case(), mode="native3d")
    fin_2d = route_finite_native3d(core_case(), mode="2d")
    fin_3d = route_finite_native3d(core_case(), mode="native3d")

    os.makedirs(ARTIFACT_DIR, exist_ok=True)
    art_2d = finalise_artifact(
        build_artifact(scene_2d, res_2d, "2d", RERUN,
                       extra={"production_budget": {"max_expansions": PROD_MAX_EXP,
                                                    "max_runtime_s": PROD_MAX_RT}}),
        os.path.join(ARTIFACT_DIR, "core_case_2d_mode.json"))
    art_3d = finalise_artifact(
        build_artifact(scene_3d, res_3d, "native3d", RERUN,
                       extra={"production_budget": {"max_expansions": PROD_MAX_EXP,
                                                    "max_runtime_s": PROD_MAX_RT}}),
        os.path.join(ARTIFACT_DIR, "core_case_native3d_mode.json"))
    art_ref_2d = finalise_artifact(
        build_artifact(core_case(), ref_2d, "2d_reference_graph", RERUN),
        os.path.join(ARTIFACT_DIR, "core_case_reference_graph_2d.json"))
    art_ref_3d = finalise_artifact(
        build_artifact(core_case(), ref_3d, "native3d_reference_graph", RERUN),
        os.path.join(ARTIFACT_DIR, "core_case_reference_graph_native3d.json"))
    art_fin_2d = finalise_artifact(
        build_artifact(core_case(), fin_2d, "2d_finite_native3d", RERUN),
        os.path.join(ARTIFACT_DIR, "core_case_finite_native3d_2d.json"))
    art_fin_3d = finalise_artifact(
        build_artifact(core_case(), fin_3d, "native3d_finite_native3d", RERUN),
        os.path.join(ARTIFACT_DIR, "core_case_finite_native3d_native3d.json"))

    def _z_probe(res):
        return {"status": res.status,
                "n_candidates": len(res.initial_candidates),
                "n_accepted_z_changing": sum(
                    1 for c in res.initial_candidates
                    if c["accepted"] and abs(c["z_change_um"]) > 1e-9),
                "candidates": res.initial_candidates}

    probe = {
        "artifact": "native3d_v012_initial_state_action_probe",
        "note": ("candidate actions generated from the INITIAL state of each "
                 "strategy, before any search expansion: proof that the initial "
                 "state already has feasible actions that change z / pitch"),
        "continuous_2d_mode": _z_probe(res_2d),
        "continuous_native3d_mode": _z_probe(res_3d),
        "finite_2d_mode": _z_probe(fin_2d),
        "finite_native3d_mode": _z_probe(fin_3d),
        "self_spacing_check": ("NOT IMPLEMENTED in the acceptance path (v0.1.2 "
                               "erratum): constraints.checks.path_self_spacing() is "
                               "never called by the router, by the result assembly or "
                               "by the independent validator, so no self-spacing "
                               "result enters any verdict"),
        "input": scene_3d.summary(),
    }
    write_json(os.path.join(ARTIFACT_DIR, "core_case_initial_action_probe.json"), probe)

    return {"scene_2d": scene_2d, "scene_3d": scene_3d,
            "res_2d": res_2d, "res_3d": res_3d,
            "ref_2d": ref_2d, "ref_3d": ref_3d,
            "fin_2d": fin_2d, "fin_3d": fin_3d,
            "artifact_2d": art_2d, "artifact_3d": art_3d,
            "artifact_ref_2d": art_ref_2d, "artifact_ref_3d": art_ref_3d,
            "artifact_fin_2d": art_fin_2d, "artifact_fin_3d": art_fin_3d,
            "artifact_dir": ARTIFACT_DIR, "probe": probe,
            "production_budget": {"max_expansions": PROD_MAX_EXP,
                                  "max_runtime_s": PROD_MAX_RT}}

# =====================================================================
# v0.2 multi-channel fixtures
# =====================================================================
@pytest.fixture(scope="session")
def v020_cfg():
    from native3d.multinet.cases_v020 import v020_config
    return v020_config()


@pytest.fixture(scope="session")
def v020_policy(v020_cfg):
    from native3d.multinet.models import SelfSpacingPolicy
    return SelfSpacingPolicy(ell_local_um=2.0 * v020_cfg.dmin_centerline_um,
                             kind="two_times_dmin")
