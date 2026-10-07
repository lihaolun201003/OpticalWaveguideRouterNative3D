"""Machine-readable evidence writer for the acceptance cases."""

from __future__ import annotations

import datetime
import json
import os
from typing import Dict, List, Optional

from ..validation.validate import validate_artifact
from .caseio import Scene, write_json

# Project root = .../OpticalWaveguideRouterNative3D (io/ -> native3d/ -> src/ -> root)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

# Evidence written from v0.1.1 onwards goes to its OWN directory.  The v0.1
# evidence under artifacts/cases and artifacts/logs is frozen: a rerun of the
# test suite or of examples/run_core_case.py must never overwrite it.
EVIDENCE_VERSION = os.environ.get("NATIVE3D_EVIDENCE_VERSION", "v013")
ENGINE_VERSION = "0.1.3"


def artifact_dir(kind: str = "cases", version: Optional[str] = None) -> str:
    """Parameterised evidence directory (never the frozen v0.1 directory).

    NATIVE3D_ARTIFACT_DIR overrides the whole path; otherwise the directory is
    <project root>/artifacts/<version>/<kind>.
    """
    env = os.environ.get("NATIVE3D_ARTIFACT_DIR")
    if env:
        return os.path.abspath(env)
    return os.path.join(PROJECT_ROOT, "artifacts", version or EVIDENCE_VERSION, kind)


def segment_provenance(result) -> List[dict]:
    """Which searched action produced each segment of the raw path."""
    out: List[dict] = []
    idx = 0
    for step in result.trace:
        n = step.get("n_segments", 0)
        for k in range(n):
            out.append({"segment_index": idx, "search_step": step["step"],
                        "action": step["action"], "kind": step["kind"]})
            idx += 1
    return out


def trace_z_summary(result) -> Dict:
    tr = result.trace
    if not tr:
        return {"n_steps": 0, "z_values_um": [], "z_varies": False,
                "note": "no trace (failure result)"}
    zs = [s["z_um"] for s in tr]
    zchg = [abs(tr[i + 1]["z_um"] - tr[i]["z_um"]) for i in range(len(tr) - 1)]
    kinds = {}
    for s in tr:
        kinds[s["action"].split("_")[0]] = kinds.get(s["action"].split("_")[0], 0) + 1
    return {"n_steps": len(tr), "z_values_um": zs,
            "z_min_um": min(zs), "z_max_um": max(zs),
            "z_varies": (max(zs) - min(zs)) > 1e-9,
            "n_distinct_z_um": len({round(z, 6) for z in zs}),
            "max_step_z_change_um": (max(zchg) if zchg else 0.0),
            "action_families": kinds,
            "actions_changing_z": sum(1 for s in tr
                                      if s["action"].startswith(("pitch", "skew",
                                                                 "connector",
                                                                 "ref_connector"))),
            "elevation_range_deg": [min(s["elevation_deg"] for s in tr),
                                    max(s["elevation_deg"] for s in tr)]}


def build_artifact(scene: Scene, result, mode: str, rerun_command: str,
                   extra: Optional[Dict] = None) -> Dict:
    art = {
        "artifact": "native3d_%s_case_evidence" % EVIDENCE_VERSION,
        "engine": "OpticalWaveguideRouterNative3D / native3d v%s" % ENGINE_VERSION,
        "strategy": ("route_finite_native3d (finite multi-hop integer-node graph)"
                     if (result.finite_graph is not None) else
                     ("reference_search (v0.1.1 two-hop finite reference graph)"
                      if (result.reference_graph is not None) else
                      "route (continuous-pose debug search)")),
        "self_spacing_check": ("NOT IMPLEMENTED in the acceptance path: "
                               "constraints.checks.path_self_spacing() exists but "
                               "neither the production router, the result assembly nor "
                               "this independent validator calls it, so no self-spacing "
                               "result enters any verdict (v0.1.2 erratum)"),
        "generated_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "case": scene.name,
        "mode": mode,
        "units": "um",
        "rerun_command": rerun_command,
        "input": scene.to_dict(),
        "input_summary": scene.summary(),
        "search": result.to_dict(include_path=True),
        "segment_provenance": segment_provenance(result),
        "trace_z_summary": trace_z_summary(result),
        "no_optical_loss_model": ("this engine computes geometry only; no dB, no "
                                  "insertion loss and no crosstalk number is produced"),
    }
    if extra:
        art.update(extra)
    return art


def finalise_artifact(art: Dict, path: Optional[str] = None) -> Dict:
    """Attach the independent validation and (optionally) write the JSON file.

    v0.1.2 skipped the independent validation whenever ``search.path`` was falsy.
    That is correct for a FAILURE but wrong for a ZERO-LENGTH SUCCESS, whose
    contract (v0.1.3 PART A3) is exactly ``status == "success"`` with no primitive
    to serialise: the path is empty, so the validator must be asked to validate
    the zero-length solution instead of being skipped.  Both cases are routed here
    so no artifact can silently carry ``independent_validation = null`` while
    claiming success.
    """
    search = art.get("search") or {}
    validated = bool(search.get("path")) or bool(search.get("zero_length_solution"))
    if validated:
        art["independent_validation"] = validate_artifact(art)
    else:
        art["independent_validation"] = None
    if path:
        write_json(path, art)
    return art