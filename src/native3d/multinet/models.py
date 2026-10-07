"""Input / output data model of the v0.2 multi-channel sequential baseline.

The vocabulary of this module keeps FOUR different things apart, because the
whole point of the v0.2 round is that they are not the same thing:

    routed      the search returned a candidate path (status "success")
    verified    that candidate passed EVERY declared acceptance check
                (production ports/C1/Rmin/boundary/obstacle/spacing, the
                post-hoc GLOBAL-ARCLENGTH self-spacing check, and the
                pre-publication inter-net spacing against the occupancy)
    committed   the verified path was published to the occupancy, so the NEXT
                net must avoid it (this round: committed == verified, and the
                publication is what makes it a hard constraint)
    failed      the net produced no verified path, with the specific failure
                category recorded

"16 candidate paths" is NOT "16 legal paths", so the summary counts are reported
separately and never conflated.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import Box3D
from ..geometry.primitives import Pose3D

# --------------------------------------------------------------- statuses
# A net that reached "verified" and was published.
STATUS_COMMITTED = "committed"
# A net whose candidate passed every declared acceptance check but that was NOT
# published (only possible with publish_on_verify=False; kept explicit so the
# two counts can never be silently equated).
STATUS_VERIFIED_NOT_COMMITTED = "verified_not_committed"

# Declared acceptance checks that are decided AFTER the search returned a
# candidate.  They are distinct from any search termination on purpose: a
# candidate that violates a post-check is NEVER relabelled search_exhausted.
STATUS_SELF_SPACING_VIOLATION = "self_spacing_violation"
STATUS_SELF_SPACING_UNCERTIFIED = "self_spacing_uncertified"
STATUS_INTER_NET_SPACING_VIOLATION = "inter_net_spacing_violation"
STATUS_INTER_NET_SPACING_UNCERTIFIED = "inter_net_spacing_uncertified"
STATUS_INPUT_INVALID = "input_invalid"

# Post-search statuses of THIS layer (v0.2).
POST_SEARCH_MULTINET_FAILURES = (
    STATUS_SELF_SPACING_VIOLATION,
    STATUS_SELF_SPACING_UNCERTIFIED,
    STATUS_INTER_NET_SPACING_VIOLATION,
    STATUS_INTER_NET_SPACING_UNCERTIFIED,
)

MULTINET_ACCEPTED_STATUSES = (STATUS_COMMITTED, STATUS_VERIFIED_NOT_COMMITTED)

# Every status that means "this net contributes no published occupancy".
MULTINET_FAILURE_STATUSES = (
    "search_exhausted",
    "budget_exhausted",
    "port_attachment_failed",
    "graph_scene_mismatch",
    "invalid_source_pose",
    "invalid_target_pose",
    "unsupported_pose",
    "constraint_failed",
    "uncertified",
    STATUS_SELF_SPACING_VIOLATION,
    STATUS_SELF_SPACING_UNCERTIFIED,
    STATUS_INTER_NET_SPACING_VIOLATION,
    STATUS_INTER_NET_SPACING_UNCERTIFIED,
    STATUS_INPUT_INVALID,
)


def default_self_spacing_policy(cfg: RouterConfig) -> "SelfSpacingPolicy":
    """The v0.2 baseline local geometric exemption policy: ``ell_local = 2*dmin``.

    This is an EXPLICIT GEOMETRIC POLICY of this baseline, chosen so that the
    arclength neighbourhood of one waveguide core is exempt from the
    self-spacing comparison.  It is NOT a paper parameter, NOT an optical safety
    distance and NOT a certified consequence of any curvature bound; the report
    discloses it as such and states its scope of application.
    """
    return SelfSpacingPolicy(ell_local_um=2.0 * float(cfg.dmin_centerline_um),
                             kind="two_times_dmin")


@dataclass(frozen=True)
class SelfSpacingPolicy:
    """The global-arclength exemption band of the self-spacing check, in um.

    Semantics (mandatory, see multinet/self_spacing.py): the path is the curve
    P(s), s in [0, L] with s the CUMULATIVE arclength of the assembled path.  The
    check domain is ``|s - t| >= ell_local_um``; only the near-neighbour band
    ``|s - t| < ell_local_um`` is exempt.  ``ell_local_um`` is a pure geometric
    length: it does not depend on the sampling step, on the chord subdivision
    level, or on any segment index.
    """

    ell_local_um: float
    kind: str = "explicit"
    note: str = ("local geometric exemption of the assembled-path arclength "
                 "neighbourhood; not a paper parameter and not an optical "
                 "safety distance")

    def __post_init__(self) -> None:
        if not (self.ell_local_um >= 0.0) or math.isinf(self.ell_local_um):
            raise ValueError("ell_local_um must be a finite number >= 0")

    def to_dict(self) -> dict:
        return {"ell_local_um": float(self.ell_local_um), "kind": self.kind,
                "note": self.note}


@dataclass
class NetSpec:
    """One net: the minimum input of prompt section 3.1 (net_id + two poses)."""

    net_id: str
    source: Pose3D
    target: Pose3D
    metadata: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"net_id": self.net_id,
                "source_pose": self.source.to_dict(),
                "target_pose": self.target.to_dict(),
                "metadata": self.metadata}


@dataclass
class MultiNetCase:
    """Common multi-net input: chip, static obstacles, STATIC reference
    centrelines, one RouterConfig, and the ORDERED net list.

    The static reference centrelines are pre-existing fixed waveguides; they are
    occupied geometry that EVERY net must avoid, but they are NEVER attributed to
    a net and are never mistaken for the path of a net of this run.
    """

    name: str
    chip: Box3D
    obstacles: List[Box3D]
    reference_centerlines: List[object]
    config: RouterConfig
    nets: List[NetSpec]
    metadata: Dict = field(default_factory=dict)

    def net_ids(self) -> List[str]:
        return [n.net_id for n in self.nets]

    def to_dict(self) -> dict:
        return {
            "case": self.name,
            "units": "um",
            "chip_volume": self.chip.to_dict(),
            "obstacles": [b.to_dict() for b in self.obstacles],
            "static_reference_centerlines": [s.to_dict()
                                             for s in self.reference_centerlines],
            "n_static_reference_centerlines": len(self.reference_centerlines),
            "config": self.config.to_dict(),
            "nets": [n.to_dict() for n in self.nets],
            "metadata": self.metadata,
        }


@dataclass
class NetOutcome:
    """Everything known about ONE net of the sequence, in one record.

    It carries the raw candidate evidence even when the net FAILED a post-check:
    the candidate segments stay available, so "the search found something that
    was then rejected" can never be confused with "nothing was found".
    """

    net_id: str
    index: int
    status: str
    reason: str
    # search result view (never a post-check verdict)
    search_status: str
    search_reason: str
    routed_candidate: bool
    length_um: Optional[float]
    cost_um_equivalent: Optional[float]
    expansions: int
    generated: int
    rejected: int
    runtime_s: float
    budgets: Dict
    # candidate geometry, kept even on post-check failure
    candidate_segments: List[object] = field(default_factory=list)
    candidate_path_dict: Optional[dict] = None
    # acceptance evidence
    production_acceptance: Optional[dict] = None
    self_spacing: Optional[dict] = None
    inter_net_spacing: Optional[dict] = None
    # publication bookkeeping
    occupancy_revision_before: Optional[int] = None
    occupancy_revision_after: Optional[int] = None
    committed_segment_ids: List[int] = field(default_factory=list)
    n_published_curves_input: int = 0
    graph_branch: Optional[str] = None
    graph_fresh: bool = False
    timings: Dict = field(default_factory=dict)
    trace_z: Dict = field(default_factory=dict)
    failure_category: Optional[str] = None

    @property
    def accepted(self) -> bool:
        return self.status in MULTINET_ACCEPTED_STATUSES

    def to_dict(self) -> dict:
        return {
            "net_id": self.net_id,
            "index": self.index,
            "status": self.status,
            "reason": self.reason,
            "accepted": self.accepted,
            "routed_candidate": self.routed_candidate,
            "failure_category": self.failure_category,
            "search_status": self.search_status,
            "search_reason": self.search_reason,
            "length_um": self.length_um,
            "cost_um_equivalent": self.cost_um_equivalent,
            "expansions": self.expansions,
            "generated": self.generated,
            "rejected": self.rejected,
            "runtime_s": self.runtime_s,
            "budgets": self.budgets,
            "n_segments": len(self.candidate_segments),
            "candidate_segments": [s.to_dict() for s in self.candidate_segments],
            "candidate_path": self.candidate_path_dict,
            "production_acceptance": self.production_acceptance,
            "self_spacing": self.self_spacing,
            "inter_net_spacing": self.inter_net_spacing,
            "occupancy_revision_before": self.occupancy_revision_before,
            "occupancy_revision_after": self.occupancy_revision_after,
            "committed_segment_ids": list(self.committed_segment_ids),
            "n_published_curves_input": self.n_published_curves_input,
            "graph_branch": self.graph_branch,
            "graph_fresh": self.graph_fresh,
            "timings": self.timings,
            "trace_z": self.trace_z,
        }


@dataclass
class MultiNetResult:
    """The whole sequential run of one case: every net outcome plus the counts."""

    case_name: str
    channel_count: int
    order: List[str]
    policy: SelfSpacingPolicy
    outcomes: List[NetOutcome]
    occupancy_revision: int
    occupancy: dict
    totals: Dict = field(default_factory=dict)
    joint_validation: Optional[dict] = None
    metadata: Dict = field(default_factory=dict)

    # ------------------------------------------------------------------ counts
    def counts(self) -> Dict:
        routed = sum(1 for o in self.outcomes if o.routed_candidate)
        verified = sum(1 for o in self.outcomes if o.accepted)
        committed = sum(1 for o in self.outcomes if o.status == STATUS_COMMITTED)
        categories: Dict[str, int] = {}
        for o in self.outcomes:
            if o.accepted:
                continue
            key = o.failure_category or o.status
            categories[key] = categories.get(key, 0) + 1
        lengths = [o.length_um for o in self.outcomes if o.length_um is not None]
        runtimes = [o.runtime_s for o in self.outcomes if o.runtime_s is not None]
        return {
            "channel_count": self.channel_count,
            "n_nets": len(self.outcomes),
            "routed_count": routed,
            "verified_count": verified,
            "committed_count": committed,
            "failed_count": len(self.outcomes) - verified,
            "failure_categories": categories,
            "all_verified": verified == len(self.outcomes) and len(self.outcomes) > 0,
            "partial_success": 0 < verified < len(self.outcomes),
            "total_length_um": (sum(lengths) if lengths else 0.0),
            "total_runtime_s": (sum(runtimes) if runtimes else 0.0),
            "max_net_runtime_s": (max(runtimes) if runtimes else 0.0),
            "sum_net_expansions": sum(o.expansions for o in self.outcomes),
        }

    def to_dict(self) -> dict:
        return {
            "case": self.case_name,
            "channel_count": self.channel_count,
            "order": list(self.order),
            "order_rule": ("input order, one net at a time; no net ordering "
                           "optimisation and no rip-up & reroute in v0.2"),
            "self_spacing_policy": self.policy.to_dict(),
            "counts": self.counts(),
            "totals": self.totals,
            "occupancy": {"revision": self.occupancy_revision,
                          "n_curves": self.occupancy.get("n_curves"),
                          "n_static_reference_curves":
                              self.occupancy.get("n_static_reference_curves"),
                          "n_published_curves":
                              self.occupancy.get("n_published_curves"),
                          "per_net_published_curve_counts":
                              self.occupancy.get("per_net_published_curve_counts"),
                          "immutability": self.occupancy.get("immutability")},
            "nets": [o.to_dict() for o in self.outcomes],
            "joint_validation": self.joint_validation,
            "metadata": self.metadata,
        }
