"""The v0.2 sequential multi-channel scheduler: one net at a time, fresh graph.

WHAT THIS SCHEDULER DOES, IN ORDER (prompt sections 2, 3.2 and 4)
-----------------------------------------------------------------
For every net, in the INPUT order (A -> B -> C ...):

1. build a BRAND NEW Scene for that net: deep-copied case geometry plus the
   frozen occupancy (the static reference centrelines first, then every curve
   published by an earlier net), with a deep-copied ``RouterConfig``;
2. search it with ``route_finite_native3d`` and **never** pass ``graph=``: the
   graph is constructed inside that call from this net's own snapshot, so the
   known mutable-alias hazard of a prebuilt graph (a graph holding a live
   Scene/config/cost-field reference) is isolated by construction rather than
   "fixed" in the cache.  The congestion field is ``ZeroCongestion``: the cost is
   the real path length and the fixed occupancy is a HARD constraint, never a
   penalty that could be paid;
3. if the search returns a candidate, run the DECLARED ACCEPTANCE on the ACTUAL
   returned curve:
   * production acceptance - already decided inside the router (assembled-curve
     hard constraints + the real port/C1 endpoint check) and recorded here;
   * the post-hoc GLOBAL-ARCLENGTH self-spacing check (multinet/self_spacing.py);
   * the pre-publication inter-net spacing against every published curve, with no
     local exemption between nets (multinet/inter_net.py);
4. only a candidate that passes ALL of them is published: its ACTUAL primitives
   are frozen into the occupancy at a new revision, and every later net must
   avoid them through the existing hard ``centerline_spacing`` constraint;
5. a net that fails anywhere is recorded with its own status - including the raw
   candidate evidence when a candidate existed - and is NOT published.  A later
   net is still attempted (``continue_after_failure``), and the case is then a
   documented PARTIAL SUCCESS, never an "all_success".

A post-check failure is NEVER relabelled ``search_exhausted``: the search did
return a path, and the distinction is kept in every artifact.
"""

from __future__ import annotations

import copy
import time
from typing import Callable, Dict, List, Optional, Sequence

from ..io.artifacts import trace_z_summary
from ..search.finite_graph import route_finite_native3d
from ..search.statuses import SUCCESS as SEARCH_SUCCESS
from .inter_net import inter_net_spacing_check
from .models import (STATUS_COMMITTED, STATUS_INPUT_INVALID,
                     STATUS_INTER_NET_SPACING_UNCERTIFIED,
                     STATUS_INTER_NET_SPACING_VIOLATION,
                     STATUS_SELF_SPACING_UNCERTIFIED,
                     STATUS_SELF_SPACING_VIOLATION,
                     STATUS_VERIFIED_NOT_COMMITTED, MultiNetResult, NetOutcome,
                     SelfSpacingPolicy, default_self_spacing_policy)
from .occupancy import Occupancy, scene_for_net
from .self_spacing import path_self_spacing_global


def route_multinet(case, policy: Optional[SelfSpacingPolicy] = None,
                   continue_after_failure: bool = True,
                   publish_on_verify: bool = True,
                   mode: str = "native3d",
                   max_expansions: Optional[int] = None,
                   max_runtime_s: Optional[float] = None,
                   run_inter_net_check: bool = True,
                   expand_hook=None,
                   on_outcome: Optional[Callable[[NetOutcome], None]] = None,
                   ) -> MultiNetResult:
    """Run the whole sequential multi-net baseline of one case.

    Every net is searched with its OWN fresh finite graph; ``graph=`` is never
    passed, which is what ``graph_fresh`` / ``graph_branch`` record per net.
    """
    if mode not in ("2d", "native3d"):
        raise ValueError("mode must be '2d' or 'native3d'")
    cfg = case.config
    policy = policy or default_self_spacing_policy(cfg)
    required = float(cfg.dmin_centerline_um)

    occupancy = Occupancy(case.reference_centerlines,
                          dmin_centerline_um=cfg.dmin_centerline_um)
    t_run0 = time.perf_counter()
    outcomes: List[NetOutcome] = []

    for index, net in enumerate(case.nets):
        t0 = time.perf_counter()
        rev_before = occupancy.revision
        n_published_input = len(occupancy.published_entries())
        scene = scene_for_net(case, net, occupancy)

        # ---- 2. search: fresh graph, zero congestion, hard occupancy ---------
        res = route_finite_native3d(scene, mode=mode, congestion=None,
                                    max_expansions=max_expansions,
                                    max_runtime_s=max_runtime_s,
                                    expand_hook=expand_hook, graph=None)
        fg = res.finite_graph or {}
        reuse = (fg.get("graph_reuse") or {})
        branch = res.timings.get("graph_branch")
        graph_fresh = (not bool(reuse.get("caller_supplied_graph", False))
                       and branch == "fresh_cold_graph")

        segments = tuple(res.path.segments) if res.path is not None else ()
        outcome = NetOutcome(
            net_id=net.net_id,
            index=index,
            status=res.status,
            reason=res.reason,
            search_status=res.status,
            search_reason=res.reason,
            routed_candidate=(res.status == SEARCH_SUCCESS),
            length_um=(None if res.length_um == float("inf") else float(res.length_um)),
            cost_um_equivalent=(None if res.cost == float("inf") else float(res.cost)),
            expansions=int(res.expansions),
            generated=int(res.generated),
            rejected=int(res.rejected),
            runtime_s=float(res.runtime_s),
            budgets={"max_expansions": int(res.max_expansions),
                     "max_runtime_s": float(res.max_runtime_s)},
            candidate_segments=list(segments),
            candidate_path_dict=(res.path.to_dict() if res.path is not None else None),
            occupancy_revision_before=rev_before,
            occupancy_revision_after=rev_before,
            n_published_curves_input=n_published_input,
            graph_branch=branch,
            graph_fresh=bool(graph_fresh),
            timings=dict(res.timings or {}),
            trace_z=trace_z_summary(res),
            failure_category=(None if res.status == SEARCH_SUCCESS else res.status),
            production_acceptance={
                "decided_by": "route_finite_native3d (assembled curve + real ports)",
                "search_status": res.status,
                "zero_length_solution": bool(res.zero_length_solution),
                "n_segments": int(res.n_segments),
                "hard_constraint_check": fg.get("assembled_path_constraint_check"),
                "endpoint_check": fg.get("assembled_endpoint_check"),
                "port_assurance": res.port_assurance,
                "graph_reuse": {"accepted": reuse.get("accepted"),
                                "caller_supplied_graph":
                                    reuse.get("caller_supplied_graph"),
                                "graph_cache_state": reuse.get("graph_cache_state"),
                                "mismatches": reuse.get("mismatches")},
                "graph_branch": branch,
                "graph_fresh_cold": bool(graph_fresh),
                "warm_graph_reuse_not_exercised_by_this_round": True,
            },
        )

        if res.status != SEARCH_SUCCESS:
            outcome.reason = (
                "search/accepted-input failure: %s" % (res.reason or ""))
        elif res.zero_length_solution:
            outcome.status = STATUS_INPUT_INVALID
            outcome.failure_category = "zero_length_net"
            outcome.reason = ("source == target: the v0.2 multi-net baseline does not "
                              "route a zero-length net, so this net publishes no "
                              "occupancy (the router's zero-length contract itself is "
                              "unchanged)")
        else:
            # ---- 3a. post-hoc global-arclength self-spacing ------------------
            ss = path_self_spacing_global(segments, cfg, policy)
            outcome.self_spacing = ss
            if ss["status"] == "violation":
                outcome.status = STATUS_SELF_SPACING_VIOLATION
                outcome.failure_category = STATUS_SELF_SPACING_VIOLATION
                outcome.reason = ("the search returned a candidate, but the post-hoc "
                                  "GLOBAL-ARCLENGTH self-spacing check found a "
                                  "violation witness inside the check domain: %r"
                                  % (ss.get("witness"),))
            elif ss["status"] != "safe":
                outcome.status = STATUS_SELF_SPACING_UNCERTIFIED
                outcome.failure_category = STATUS_SELF_SPACING_UNCERTIFIED
                outcome.reason = ("the search returned a candidate, but its "
                                  "self-spacing could not be certified inside the "
                                  "refinement cap (certified lower bound %r < dmin %r); "
                                  "NOT a search failure and NOT search_exhausted"
                                  % (ss.get("certified_lower_bound_um"), required))

        if outcome.status == SEARCH_SUCCESS and run_inter_net_check:
            # ---- 3b. pre-publication inter-net spacing -----------------------
            published = occupancy.published_curves()
            published_ids = [e.net_id for e in occupancy.published_entries()]
            ins = inter_net_spacing_check(
                segments, published, required, cfg,
                published_net_ids=[str(x) for x in published_ids])
            outcome.inter_net_spacing = ins
            if ins["status"] == "violation":
                outcome.status = STATUS_INTER_NET_SPACING_VIOLATION
                outcome.failure_category = STATUS_INTER_NET_SPACING_VIOLATION
                outcome.reason = ("the candidate is closer than dmin to an already "
                                  "published curve: %r" % (ins.get("violation"),))
            elif ins["status"] == "undecided":
                outcome.status = STATUS_INTER_NET_SPACING_UNCERTIFIED
                outcome.failure_category = STATUS_INTER_NET_SPACING_UNCERTIFIED
                outcome.reason = ("the candidate's spacing to the published occupancy "
                                  "could not be certified inside the refinement cap")

        if outcome.status == SEARCH_SUCCESS:
            # ---- 4. publication of a fully verified path ---------------------
            if publish_on_verify:
                rev = occupancy.publish(net.net_id, segments, verified=True)
                outcome.status = STATUS_COMMITTED
                outcome.occupancy_revision_after = rev
                outcome.committed_segment_ids = list(range(len(segments)))
                outcome.reason = ("verified and published at occupancy revision %d: "
                                  "the actual %d primitive(s) became fixed occupancy "
                                  "for every later net" % (rev, len(segments)))
            else:
                outcome.status = STATUS_VERIFIED_NOT_COMMITTED
                outcome.reason = ("verified but NOT published (publish_on_verify="
                                  "False): this run is a verification-only run")

        outcome.timings = dict(outcome.timings or {})
        outcome.timings["multinet_net_wall_s"] = time.perf_counter() - t0
        outcomes.append(outcome)
        if on_outcome is not None:
            on_outcome(outcome)
        if not outcome.accepted and not continue_after_failure:
            break

    immutability = occupancy.verify_immutability()
    wall = time.perf_counter() - t_run0
    lengths = [o.length_um for o in outcomes if o.length_um is not None]
    runtimes = [o.timings.get("multinet_net_wall_s", 0.0) for o in outcomes]
    result = MultiNetResult(
        case_name=case.name,
        channel_count=len(case.nets),
        order=[n.net_id for n in case.nets][:len(outcomes)],
        policy=policy,
        outcomes=outcomes,
        occupancy_revision=occupancy.revision,
        occupancy=occupancy.to_dict(),
        totals={
            "wall_time_s": wall,
            "sum_net_wall_time_s": sum(runtimes),
            "max_net_wall_time_s": (max(runtimes) if runtimes else 0.0),
            "total_path_length_um": sum(lengths),
            "sum_search_expansions": sum(o.expansions for o in outcomes),
            "sum_search_runtime_s": sum(o.runtime_s for o in outcomes),
            "occupancy_revision_final": occupancy.revision,
            "occupancy_immutable": bool(immutability["ok"]),
            "n_nets_attempted": len(outcomes),
            "stopped_after_failure": (not continue_after_failure
                                      and len(outcomes) < len(case.nets)),
        },
        metadata={
            "mode": mode,
            "per_net_fresh_graph": True,
            "graph_argument_never_passed": True,
            "congestion_field": "ZeroCongestion (cost = real path length)",
            "occupancy_is_a_hard_constraint": True,
            "order_rule": ("input order; the scheduler adds no ordering heuristic "
                           "and performs no rip-up & reroute"),
            "warm_graph_reuse_scope": ("the warm/prebuilt-graph branch and the "
                                       "curved-input alias hazard of the public "
                                       "cache path are NOT exercised or certified by "
                                       "this round; v0.2 isolates them by never "
                                       "passing graph= and says so"),
            "self_spacing_wired_into_the_main_flow": True,
            "no_optical_model": True,
            "immutability": immutability,
        },
    )
    return result


def joint_validation_payload(case, result: MultiNetResult,
                             extra: Optional[Dict] = None) -> Dict:
    """The JSON payload the INDEPENDENT joint validator reads (paths + occupancy).

    It carries no verdict of this scheduler: only the inputs, the posed nets, the
    returned segment geometry, the published occupancy and the declared policy, so
    the validator re-derives every number itself.
    """
    payload = {
        "artifact": "native3d_v020_multinet_case",
        "engine": "OpticalWaveguideRouterNative3D / native3d v0.2 multinet baseline",
        "units": "um",
        "case": case.name,
        "mode": result.metadata.get("mode", "native3d"),
        "channel_count": result.channel_count,
        "order": list(result.order),
        "chip_volume": case.chip.to_dict(),
        "obstacles": [b.to_dict() for b in case.obstacles],
        "static_reference_centerlines": [s.to_dict()
                                         for s in case.reference_centerlines],
        "config": case.config.to_dict(),
        "self_spacing_policy": result.policy.to_dict(),
        "nets": [
            {
                "net_id": o.net_id,
                "index": o.index,
                "input": case.nets[o.index].to_dict(),
                "status": o.status,
                "accepted": o.accepted,
                "search_status": o.search_status,
                "length_um": o.length_um,
                "expansions": o.expansions,
                "occupancy_revision_before": o.occupancy_revision_before,
                "occupancy_revision_after": o.occupancy_revision_after,
                "committed": o.status == STATUS_COMMITTED,
                "committed_segment_ids": list(o.committed_segment_ids),
                "path": o.candidate_path_dict,
                "segments": [s.to_dict() for s in o.candidate_segments],
            }
            for o in result.outcomes
        ],
        "occupancy": {"revision": result.occupancy_revision,
                      "entries": result.occupancy.get("entries"),
                      "n_entries": len(result.occupancy.get("entries") or []),
                      "n_published_entries": sum(
                          1 for e in (result.occupancy.get("entries") or [])
                          if e.get("origin") == "committed_net_path"),
                      "n_static_reference_entries": sum(
                          1 for e in (result.occupancy.get("entries") or [])
                          if e.get("origin") == "static_reference_centerline"),
                      "per_net_published_curve_counts":
                          result.occupancy.get("per_net_published_curve_counts"),
                      "immutability": result.occupancy.get("immutability"),
                      "publication_rule": result.occupancy.get("publication_rule")},
        "counts": result.counts(),
        "totals": result.totals,
        "no_optical_loss_model": ("geometry only: no dB, no insertion loss and no "
                                  "crosstalk number is produced"),
    }
    if extra:
        payload.update(extra)
    return payload
