"""v0.2 multi-channel sequential baseline: the public runnable entry point.

    cd "C:\\Users\\lihao\\Desktop\\Graduation Project\\OpticalWaveguideRouterNative3D"
    .venv\\Scripts\\python.exe examples\\run_multichannel_baseline.py

Options:

    --only NAME[,NAME]   run only the named cases (substring match)
    --no-controls        skip the declared control runs
    --figures            also regenerate the board-layout figures
    --quick              reduce nothing: kept for symmetry with older runners

What it writes (nothing outside artifacts/v020/):

    artifacts/v020/cases/<case>.json             input + every returned path + occupancy
    artifacts/v020/cases/<case>.validated.json   the same payload + the INDEPENDENT
                                                 joint validation re-derived from the
                                                 written file
    artifacts/v020/logs/<case>.log               the human-readable per-net table
    artifacts/v020/metrics/summary.json          every case's counts in one place
    artifacts/v020/metrics/<case>_metrics.json   per-case metrics

The joint validator is ALWAYS fed the JSON that was actually written to disk (read
back with json.load), so the independent second implementation really sees only the
file.  It imports nothing from the router:

    $env:PYTHONPATH = "src"
    .venv\\Scripts\\python.exe -m native3d.validation.joint_validate artifacts\\v020\\cases\\<case>.json
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "src"))

from native3d.io.caseio import write_json                      # noqa: E402
from native3d.multinet.cases_v020 import (CONTROL_NETS, all_cases,  # noqa: E402
                                          with_nets)
from native3d.multinet.sequential import (joint_validation_payload,  # noqa: E402
                                          route_multinet)
from native3d.validation import joint_validate as jv           # noqa: E402

ART = os.path.join(_ROOT, "artifacts", "v020")
CASES_DIR = os.path.join(ART, "cases")
LOGS_DIR = os.path.join(ART, "logs")
METRICS_DIR = os.path.join(ART, "metrics")


def _fmt(x, nd=6):
    if x is None:
        return "-"
    if isinstance(x, float):
        return "%.*f" % (nd, x)
    return str(x)


def case_log(case, result, joint, controls) -> str:
    lines = []
    w = lines.append
    w("=" * 100)
    w("case: %s" % case.name)
    w("channel_count: %d      order: %s"
      % (result.channel_count, " -> ".join(result.order)))
    w("order_rule: input order, one net at a time; no net ordering, no rip-up & reroute")
    w("self-spacing policy: ell_local = %g um (%s)   dmin = %g um"
      % (result.policy.ell_local_um, result.policy.kind,
         case.config.dmin_centerline_um))
    w("per-net fresh finite graph: %s   graph= argument never passed: %s"
      % (result.metadata.get("per_net_fresh_graph"),
         result.metadata.get("graph_argument_never_passed")))
    c = result.counts()
    w("")
    w("routed_count   = %d" % c["routed_count"])
    w("verified_count = %d" % c["verified_count"])
    w("committed_count= %d" % c["committed_count"])
    w("failed_count   = %d   failure_categories = %s"
      % (c["failed_count"], json.dumps(c["failure_categories"], sort_keys=True)))
    w("all_verified   = %s   partial_success = %s"
      % (c["all_verified"], c["partial_success"]))
    w("total_length_um= %s   total_runtime_s(net wall)=%s   max_net_wall=%s"
      % (_fmt(c["total_length_um"], 3), _fmt(c["total_runtime_s"], 3),
         _fmt(c["max_net_runtime_s"], 3)))
    w("")
    w("%-10s %-26s %-10s %-8s %-9s %-9s %-8s %-9s %-9s %s"
      % ("net", "status", "length", "segments", "expansions", "occ_rev", "z_range",
         "self_sp", "inter_net", "failure_category"))
    for o in result.outcomes:
        zr = "-"
        if o.trace_z and o.trace_z.get("n_steps"):
            zr = "[%g,%g]" % (o.trace_z["z_min_um"], o.trace_z["z_max_um"])
        ss = (o.self_spacing or {}).get("status", "-")
        ins = (o.inter_net_spacing or {}).get("status", "-")
        w("%-10s %-26s %-10s %-8d %-9d %-9s %-8s %-9s %-9s %s"
          % (o.net_id, o.status, _fmt(o.length_um, 3), len(o.candidate_segments),
             o.expansions, "%d->%d" % (o.occupancy_revision_before,
                                       o.occupancy_revision_after),
             zr, ss, ins, o.failure_category or "-"))
    w("")
    w("raw candidate vs published path:")
    for o in result.outcomes:
        w("  %-10s routed_candidate=%-5s n_candidate_segments=%-3d published=%s"
          % (o.net_id, o.routed_candidate, len(o.candidate_segments),
             "yes (revision %s)" % o.occupancy_revision_after if o.committed_segment_ids
             else "NO"))
    w("")
    mins = [o.inter_net_spacing for o in result.outcomes if o.inter_net_spacing]
    if mins:
        w("inter-net certified spacing actually achieved (per net, first net has none):")
        for o in result.outcomes:
            if not o.inter_net_spacing:
                continue
            w("  %-10s status=%-10s min_certified_lower_bound=%s  n_pairs=%d  kinds=%s"
              % (o.net_id, o.inter_net_spacing["status"],
                 _fmt(o.inter_net_spacing["min_certified_lower_bound_um"]),
                 o.inter_net_spacing["n_pairs"],
                 json.dumps(o.inter_net_spacing["n_pairs_by_kind"], sort_keys=True)))
    w("")
    w("self-spacing certification actually achieved:")
    for o in result.outcomes:
        if not o.self_spacing:
            continue
        ss = o.self_spacing
        w("  %-10s status=%-10s LB=%-12s UB=%-12s exempt_pairs=%-3d undecided=%d"
          % (o.net_id, ss["status"], _fmt(ss.get("certified_lower_bound_um")),
             _fmt(ss.get("certified_upper_bound_um")),
             ss.get("n_pairs_fully_exempt", 0), ss.get("n_pairs_undecided", 0)))
    w("")
    w("occupancy: revision=%d curves=%d (static=%d, published=%d) immutable=%s"
      % (result.occupancy_revision, result.occupancy["n_curves"],
         result.occupancy["n_static_reference_curves"],
         result.occupancy["n_published_curves"],
         result.occupancy["immutability"]["ok"]))
    for net_id, n in (result.occupancy["per_net_published_curve_counts"] or {}).items():
        w("   published %-10s %d curve(s)" % (net_id, n))
    if controls:
        w("")
        w("control runs (what the published occupancy is responsible for):")
        for name, ctl_result in controls:
            cc = ctl_result.counts()
            w("  %-28s committed=%d/%d  statuses=%s"
              % (name, cc["committed_count"], cc["n_nets"],
                 ",".join("%s:%s" % (o.net_id, o.status)
                          for o in ctl_result.outcomes)))
    if joint is not None:
        w("")
        w("INDEPENDENT joint validation (re-derived from the written JSON file):")
        w("  overall_verdict = %s" % joint["overall_verdict"])
        jc = joint["counts"]
        w("  independent pass/fail/uncertified = %d/%d/%d   declared verified=%d "
          "committed=%d" % (jc["n_pass"], jc["n_fail"], jc["n_uncertified"],
                            jc["declared_verified"], jc["declared_committed"]))
        w("  inter-net net-pairs=%d violations=%d undecided=%d"
          % (jc["n_inter_net_pairs"], jc["n_inter_net_violations"],
             jc["n_inter_net_undecided"]))
        w("  disagreements with the declared statuses = %d" % jc["n_disagreements"])
        w("  occupancy bookkeeping ok = %s" % joint["occupancy_bookkeeping"]["ok"])
    w("=" * 100)
    return "\n".join(lines) + "\n"


def metrics_of(case, result, joint, controls, wall_s) -> dict:
    c = result.counts()
    per_net = []
    for o in result.outcomes:
        per_net.append({
            "net_id": o.net_id,
            "status": o.status,
            "search_status": o.search_status,
            "routed_candidate": o.routed_candidate,
            "length_um": o.length_um,
            "expansions": o.expansions,
            "generated": o.generated,
            "rejected": o.rejected,
            "runtime_s": o.runtime_s,
            "wall_s": o.timings.get("multinet_net_wall_s"),
            "n_segments": len(o.candidate_segments),
            "kinds": "".join("A" if s.kind == "arc" else "L"
                             for s in o.candidate_segments),
            "occupancy_revision_before": o.occupancy_revision_before,
            "occupancy_revision_after": o.occupancy_revision_after,
            "committed": o.status == "committed",
            "failure_category": o.failure_category,
            "graph_fresh_cold": o.graph_fresh,
            "z_min_um": (o.trace_z or {}).get("z_min_um"),
            "z_max_um": (o.trace_z or {}).get("z_max_um"),
            "self_spacing_status": (o.self_spacing or {}).get("status"),
            "self_spacing_lower_bound_um":
                (o.self_spacing or {}).get("certified_lower_bound_um"),
            "inter_net_status": (o.inter_net_spacing or {}).get("status"),
            "inter_net_min_lower_bound_um":
                (o.inter_net_spacing or {}).get("min_certified_lower_bound_um"),
        })
    return {
        "case": case.name,
        "channel_count": result.channel_count,
        "order": list(result.order),
        "counts": c,
        "totals": result.totals,
        "wall_s": wall_s,
        "self_spacing_policy": result.policy.to_dict(),
        "occupancy": {"revision": result.occupancy_revision,
                      "n_curves": result.occupancy["n_curves"],
                      "n_static_reference_curves":
                          result.occupancy["n_static_reference_curves"],
                      "n_published_curves": result.occupancy["n_published_curves"],
                      "per_net_published_curve_counts":
                          result.occupancy["per_net_published_curve_counts"],
                      "immutable": result.occupancy["immutability"]["ok"]},
        "nets": per_net,
        "controls": [{"name": n, "counts": r.counts(),
                      "statuses": {o.net_id: o.status for o in r.outcomes}}
                     for n, r in controls],
        "joint_validation": (None if joint is None else {
            "overall_verdict": joint["overall_verdict"],
            "counts": joint["counts"],
            "occupancy_bookkeeping_ok": joint["occupancy_bookkeeping"]["ok"]}),
    }


def run_one(case, *, run_controls=True, verbose=True):
    t0 = time.perf_counter()
    result = route_multinet(case)
    wall = time.perf_counter() - t0

    controls = []
    if run_controls:
        for name, net_ids in CONTROL_NETS.get(case.name, []):
            controls.append((name, route_multinet(with_nets(case, list(net_ids)))))

    payload = joint_validation_payload(case, result)
    case_path = os.path.join(CASES_DIR, "%s.json" % case.name)
    write_json(case_path, payload)

    # ---- the independent validator reads the file that was just written -----
    with open(case_path, "r", encoding="utf-8") as fh:
        from_file = json.load(fh)
    joint = jv.validate_joint_artifact(from_file)
    write_json(os.path.join(CASES_DIR, "%s.validated.json" % case.name),
               dict(payload, joint_validation_from_file=joint))

    run_metrics = metrics_of(case, result, joint, controls, wall)
    write_json(os.path.join(METRICS_DIR, "%s_metrics.json" % case.name), run_metrics)
    log = case_log(case, result, joint, controls)
    os.makedirs(LOGS_DIR, exist_ok=True)
    with open(os.path.join(LOGS_DIR, "%s.log" % case.name), "w",
              encoding="utf-8") as fh:
        fh.write(log)
    if verbose:
        print(log)
    return run_metrics


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Native3D v0.2 multi-channel baseline")
    ap.add_argument("--only", default=None,
                    help="comma separated case-name substrings")
    ap.add_argument("--no-controls", action="store_true")
    ap.add_argument("--figures", action="store_true")
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args(argv)

    os.makedirs(CASES_DIR, exist_ok=True)
    os.makedirs(LOGS_DIR, exist_ok=True)
    os.makedirs(METRICS_DIR, exist_ok=True)

    cases = all_cases()
    if args.only:
        wanted = [s.strip() for s in args.only.split(",") if s.strip()]
        cases = [c for c in cases if any(w in c.name for w in wanted)]
    if not cases:
        print("no case selected")
        return 2

    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    summary = {"artifact": "native3d_v020_multichannel_baseline_summary",
               "started_at_utc": started,
               "engine": ("OpticalWaveguideRouterNative3D / native3d v0.2 multinet "
                          "sequential baseline"),
               "rerun_command": ("cd OpticalWaveguideRouterNative3D && "
                                 ".venv\\Scripts\\python.exe "
                                 "examples\\run_multichannel_baseline.py"),
               "cases": []}
    t_all = time.perf_counter()
    for case in cases:
        m = run_one(case, run_controls=not args.no_controls)
        summary["cases"].append({k: m[k] for k in
                                 ("case", "channel_count", "order", "counts",
                                  "totals", "wall_s", "occupancy", "controls",
                                  "joint_validation")})
    summary["wall_s"] = time.perf_counter() - t_all
    summary["finished_at_utc"] = datetime.datetime.now(
        datetime.timezone.utc).isoformat()
    write_json(os.path.join(METRICS_DIR, "summary.json"), summary)

    print()
    print("=" * 100)
    print("%-46s %5s %8s %9s %10s %10s %s"
          % ("case", "N", "routed", "verified", "committed", "joint", "wall_s"))
    for c in summary["cases"]:
        jv_ = c["joint_validation"]
        print("%-46s %5d %8d %9d %10d %10s %.2f"
              % (c["case"], c["channel_count"], c["counts"]["routed_count"],
                 c["counts"]["verified_count"], c["counts"]["committed_count"],
                 (jv_ or {}).get("overall_verdict", "-"), c["wall_s"]))
    print("total wall time %.2f s" % summary["wall_s"])
    print("summary: %s" % os.path.join(METRICS_DIR, "summary.json"))

    if args.figures:
        if _HERE not in sys.path:
            sys.path.insert(0, _HERE)
        from plot_board_layout import main as plot_main
        plot_main([])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
