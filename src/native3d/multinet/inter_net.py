"""Inter-net spacing of the v0.2 sequential baseline: NO arclength exemption.

Between two DIFFERENT nets there is no self-spacing local exemption (prompt
section 4.2): two nets may not ignore a conflict because their segment indices
coincide, because their ports are close, or because of any ``net_id`` rule.  Every
pair of continuous curves of A and B is decided - line/line, line/arc, arc/line
and arc/arc - and the certified lower bound subtracts the geometric approximation
error of BOTH sides when either side is a curve.

Three numbers are always reported separately and are never conflated:

* ``sampled_estimate_um``      - a value measured at finitely many points of the
  two curves; a sampled MINIMUM can only be too LARGE, so it is never a
  certificate;
* ``certified_lower_bound_um`` - a number that can never exceed the true minimum
  distance (it is what turns "no violation found" into "certified safe");
* ``violation witness``        - a pair of points that really lie ON the two
  curves and are closer than required, which is what proves a violation.

An ``undecided`` pair is never a pass.
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import (SEGMENT_DISTANCE_ERROR_FLOOR_UM, aabb_gap,
                                  pair_clearance_bounds)

PAIR_KINDS = ("line-line", "line-arc", "arc-line", "arc-arc")


def pair_kind(A, B) -> str:
    return "%s-%s" % (A.kind, B.kind)


def sampled_pair_distance(A, B, n: int = 64) -> float:
    """SAMPLED minimum distance: an estimate, never a certificate."""
    pa = [A.point_at(float(A.length) * i / n) for i in range(n + 1)]
    pb = [B.point_at(float(B.length) * i / n) for i in range(n + 1)]
    best = math.inf
    for p in pa:
        for q in pb:
            d = math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2)
            if d < best:
                best = d
    return best


def pair_spacing_certified(A, B, required: float, cfg: RouterConfig) -> Dict:
    """Certified spacing decision of ONE pair of continuous curves."""
    out: Dict = {"kind": pair_kind(A, B), "required_um": required,
                 "sampled_estimate_um": None, "certified_lower_bound_um": None,
                 "certified_upper_bound_um": None, "status": "undecided",
                 "witness": None, "level": None, "method": None}
    lo1, hi1 = A.aabb()
    lo2, hi2 = B.aabb()
    gap = aabb_gap(tuple(lo1), tuple(hi1), tuple(lo2), tuple(hi2))
    if gap >= required:
        out.update({"status": "safe", "certified_lower_bound_um": gap,
                    "certified_upper_bound_um": None, "method": "aabb_gap",
                    "sampled_estimate_um": None})
        return out
    out["sampled_estimate_um"] = sampled_pair_distance(A, B)
    tol = cfg.clearance_decision_tol_um
    level = 0
    while True:
        lb, ub, witness = pair_clearance_bounds(A, B, level)
        if witness + SEGMENT_DISTANCE_ERROR_FLOOR_UM < required - tol:
            out.update({"status": "violation", "certified_lower_bound_um": lb,
                        "certified_upper_bound_um": ub, "level": level,
                        "method": "violation_witness_on_the_curve",
                        "witness": {"distance_um": witness,
                                    "note": "a point pair ON the two curves is "
                                            "closer than required"}})
            return out
        if lb >= required - tol:
            out.update({"status": "safe", "certified_lower_bound_um": lb,
                        "certified_upper_bound_um": ub, "level": level,
                        "method": "conservative_chord_bounds"})
            return out
        if ub < required - tol:
            out.update({"status": "violation", "certified_lower_bound_um": lb,
                        "certified_upper_bound_um": ub, "level": level,
                        "method": "conservative_chord_bounds",
                        "witness": {"distance_upper_bound_um": ub,
                                    "note": "an upper bound of the true minimum "
                                            "is already below required"}})
            return out
        if level >= cfg.max_subdiv_level:
            out.update({"status": "undecided", "certified_lower_bound_um": lb,
                        "certified_upper_bound_um": ub, "level": level,
                        "method": "conservative_chord_bounds",
                        "note": "refinement cap reached without a decision"})
            return out
        level += 1


def inter_net_spacing_check(segments: Sequence[object],
                            published: Sequence[object],
                            required: float, cfg: RouterConfig,
                            published_net_ids: Sequence[str] = (),
                            segment_net_id: Sequence[str] = ()) -> Dict:
    """Every curve pair between a candidate net path and the published occupancy."""
    segs = list(segments)
    pubs = list(published)
    n_by_kind = {k: 0 for k in PAIR_KINDS}
    pairs: List[dict] = []
    violation = None
    n_undecided = 0
    min_lb = math.inf
    min_sampled = math.inf
    best_pair = None
    for i, s in enumerate(segs):
        for j, p in enumerate(pubs):
            res = pair_spacing_certified(s, p, required, cfg)
            n_by_kind[res["kind"]] = n_by_kind.get(res["kind"], 0) + 1
            entry = {"candidate_segment": i,
                     "published_curve": j,
                     "published_net_id": (published_net_ids[j]
                                          if j < len(published_net_ids) else None),
                     "status": res["status"],
                     "certified_lower_bound_um": res["certified_lower_bound_um"],
                     "sampled_estimate_um": res["sampled_estimate_um"],
                     "method": res["method"], "level": res["level"]}
            if res["certified_lower_bound_um"] is not None and \
                    res["certified_lower_bound_um"] < min_lb:
                min_lb = res["certified_lower_bound_um"]
                best_pair = (i, j)
            if res["sampled_estimate_um"] is not None and \
                    res["sampled_estimate_um"] < min_sampled:
                min_sampled = res["sampled_estimate_um"]
            if res["status"] == "undecided":
                n_undecided += 1
            if res["status"] == "violation" and violation is None:
                violation = dict(entry)
                violation["witness"] = res["witness"]
            pairs.append(entry)
    if not pubs:
        status = "safe"
        note = ("the occupancy is empty: this net is the first one, so there is no "
                "inter-net pair to decide")
    elif violation is not None:
        status = "violation"
        note = ("at least one candidate/published curve pair is closer than dmin "
                "(or its certified upper bound already is)")
    elif n_undecided:
        status = "undecided"
        note = "at least one curve pair could not be decided inside the refinement cap"
    else:
        status = "safe"
        note = "every curve pair is certified at or above the required spacing"
    min_allowed = min_lb if not math.isinf(min_lb) else None
    return {
        "name": "inter_net_spacing",
        "no_local_exemption_between_nets": True,
        "required_um": required,
        "status": status,
        "note": note,
        "n_candidate_segments": len(segs),
        "n_published_curves": len(pubs),
        "n_pairs": len(pairs),
        "n_pairs_by_kind": n_by_kind,
        "n_pairs_undecided": n_undecided,
        "min_certified_lower_bound_um": min_allowed,
        "min_certified_lower_bound_pair": (list(best_pair) if best_pair else None),
        "min_sampled_estimate_um": (min_sampled if not math.isinf(min_sampled) else None),
        "violation": violation,
        "pub_definition": ("sampled estimate = measured at finitely many points (can "
                           "only be too large); certified lower bound = can never "
                           "exceed the true minimum; witness = a point pair ON the "
                           "two curves below the requirement"),
        "pairs": pairs,
    }
