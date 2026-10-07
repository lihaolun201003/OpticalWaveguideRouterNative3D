"""Phase 4: native 3D A* over the pose state space.

    edge_cost = primitive_length + congestion_penalty      (congestion >= 0)
    g         = sum of real edge costs along the node chain
    f         = g + h

Heuristic (admissible, consistent):
    h(s) = max(0, |target_position - s.position| - eps_pos)

Admissibility argument: every legal path from s to the goal set must be at least
as long as the straight-line distance from s.position to the goal position, and
it is allowed to stop anywhere within eps_pos of the goal position, hence any
legal path length >= max(0, dist - eps_pos).  Each edge cost is
(length of the primitive) + (non-negative penalty) >= the Euclidean distance
between its endpoints (a straight segment attains equality, a circular arc is
longer than its chord).  Constraints only remove paths, so h never overestimates
the true remaining cost.

TERMINATION (v0.1.1, repaired): a goal state is returned only when it is POPPED
from the open set with the minimum f.  v0.1 executed the goal test inside the
candidate-generation loop and returned the first goal NEIGHBOUR that happened to
be generated, which is not optimal even for h = 0 (see
tests/test_repair_a_astar_optimality.py for the finite-graph counterexample:
S->G costs 100 and is generated first, S->A->G costs 2, v0.1 returned 100).

COST COMPARISON TOLERANCE (v0.1.2, explicitly stated)
-------------------------------------------------
"Better g" is decided with a documented tolerance, not with exact real
arithmetic:

    g_improves(new_g, old_g) := new_g < old_g - tol
    tol := G_COMPARISON_ABS_TOL_UM + G_COMPARISON_REL_TOL * max(|new_g|, |old_g|)

with G_COMPARISON_ABS_TOL_UM = G_COMPARISON_REL_TOL = 1e-12 (1e-12 um = 1e-18 m,
i.e. twelve orders of magnitude below the smallest feature of this project).
Consequence, stated instead of hidden: the returned cost is the best cost the
enumerated graph offers UP TO THIS TOLERANCE.  When two g values differ by less
than the tolerance the search does not distinguish them, so the result is
epsilon-optimal (epsilon = the tolerance above), NOT "optimal over the reals".
Everything else about optimality is claimed only for the enumerated graph.

Conditions required for the optimality claim:

* the search graph is the graph actually enumerated (nodes = distinct states, as
  identified by key_fn; edges = the candidates the generator accepts), and every
  edge cost is finite and non-negative;
* h is admissible (h(s) <= true optimal cost from s to the goal set) and
  consistent (h(s) <= c(s, s') + h(s') for every edge), which the 3D Euclidean
  bound above satisfies whenever every edge cost is >= the Euclidean distance
  between its endpoints;
* the goal set is the set of states the goal test accepts, so a state may be
  accepted anywhere inside the documented tolerances;
* stale heap entries are discarded, a state is expanded once per improving g,
  and a state whose g improves after expansion is re-opened (closed entries are
  discarded on improvement), so a better path found later is still used.

Consequences and limits (stated, not hidden):

* the claim is optimality on the enumerated graph, i.e. for the given state
  identity, action generator and goal test.  It is NOT a claim of global
  optimality or completeness over the continuous pose space;
* with the v0.1.1 safe exact-pose identity the enumerated graph is generally
  infinite, so a run can end in budget_exhausted instead of search_exhausted;
* with the explicitly approximate quantised identity the enumerated graph is the
  quantised abstract graph: exhausting it does NOT prove that the continuous
  space is unreachable.

Statuses returned: success, invalid_source_pose, invalid_target_pose,
search_exhausted (open set empty -> the finite graph really was exhausted),
budget_exhausted (max expansions or max runtime hit).  A budget stop is NEVER
reported as unreachable.  The full vocabulary and what each status means is
defined in search/statuses.py.

TWO KINDS OF SUCCESS (v0.1.3)
-----------------------------
1. a normal success: the goal state is POPPED with minimal f, the node chain is
   concatenated into a Path3D, ``n_segments == len(path.segments)`` and
   ``zero_length_solution is False``.
2. the ZERO-LENGTH solution: the start state already satisfies the goal test, so
   there is no primitive to concatenate.  ``cost == length_um == 0``,
   ``path is None``, ``n_segments == 0``, ``zero_length_solution is True`` and
   the trace has exactly one step.  v0.1.2 returned the same shape but did not
   say so, and the finite router then dereferenced ``path.segments`` and crashed
   with AttributeError; the flag is what makes the contract checkable.
A failure also has ``path is None``, which is exactly why the separate boolean
exists: never infer success from ``path is not None``, and never infer failure
from ``path is None`` without reading ``zero_length_solution``.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..geometry.path3d import Path3D
from ..identity import POSE_FLOAT_STATE_IDENTITY
from .state import State3D

# Documented cost comparison tolerance (see the module docstring).  Costs are
# "um equivalent"; 1e-12 um is far below any geometric feature of this project,
# and these two constants are the ONLY place where a g comparison is softened.
G_COMPARISON_ABS_TOL_UM = 1e-12
G_COMPARISON_REL_TOL = 1e-12


def g_tolerance(new_g: float, old_g: float) -> float:
    """Documented comparison tolerance for two cost values."""
    return G_COMPARISON_ABS_TOL_UM + G_COMPARISON_REL_TOL * max(abs(new_g),
                                                                abs(old_g))


def g_improves(new_g: float, old_g: float) -> bool:
    """True when new_g is better than old_g by MORE than the tolerance.

    A difference inside the tolerance is deliberately NOT treated as an
    improvement: it is indistinguishable from floating point noise at the
    documented scale, and pretending otherwise would be an unstated tie-break.
    """
    return new_g < old_g - g_tolerance(new_g, old_g)


def g_is_worse(new_g: float, old_g: float) -> bool:
    """True when new_g is worse than old_g by MORE than the tolerance."""
    return new_g > old_g + g_tolerance(new_g, old_g)


@dataclass
class Candidate:
    action: object
    new_state: State3D
    cost: float
    accepted: bool
    reason: str = ""
    validation: Optional[dict] = None


@dataclass
class Node:
    state: State3D
    g: float
    parent: int
    action_name: str
    action_kind: str
    depth: int
    segments: tuple = ()


@dataclass
class SearchResult:
    status: str
    reason: str
    mode: str
    heuristic: str
    expansions: int
    generated: int
    rejected: int
    runtime_s: float
    cost: float = math.inf
    length_um: float = math.inf
    congestion_cost: float = 0.0
    path: Optional[Path3D] = None
    trace: List[dict] = field(default_factory=list)
    initial_candidates: List[dict] = field(default_factory=list)
    rejection_summary: Dict[str, int] = field(default_factory=dict)
    max_expansions: int = 0
    max_runtime_s: float = 0.0
    # which state identity the enumerated graph used; approximate_state_identity
    # is True only for the explicitly approximate quantised mode, whose
    # exhaustion is NOT a proof of unreachability
    state_identity: str = POSE_FLOAT_STATE_IDENTITY
    approximate_state_identity: bool = False
    # populated only by search/reference_graph.py
    reference_graph: Optional[dict] = None
    # populated only by search/finite_graph.py (the v0.1.2 production strategy)
    finite_graph: Optional[dict] = None
    # wall clock breakdown of the strategy that produced this result
    timings: Dict[str, float] = field(default_factory=dict)
    goal_pos_error_um: Optional[float] = None
    goal_tangent_error_deg: Optional[float] = None
    budget_exhausted: bool = False
    # ---------------------------------------------------------------- v0.1.3
    # ADDITIVE result-contract fields (the search and termination logic above is
    # unchanged).  They exist so that "success" can be told apart from "a failure
    # whose path is also None", which v0.1.2 left ambiguous:
    #
    # zero_length_solution
    #     True only for the explicitly defined ZERO-LENGTH solution: the start
    #     state already satisfies the goal test, so there is NO primitive to
    #     concatenate, cost/length are exactly 0 and path stays None.  A failure
    #     also has path None, so THIS flag is what separates the two.  It is
    #     False on every failure and on every non-zero-length success.
    # n_segments
    #     Number of primitives in the returned path (0 for the zero-length
    #     solution and for every failure).  Never a fabricated micro-segment.
    # endpoint_errors
    #     The verified endpoint numbers against the pose the CALLER passed
    #     (filled in by search/finite_graph.py); None for a pure failure.
    # port_assurance
    #     The assembled-path / port assurance block of a verified success
    #     (search/finite_graph.py); None for a pure failure.
    zero_length_solution: bool = False
    n_segments: int = 0
    endpoint_errors: Optional[dict] = None
    port_assurance: Optional[dict] = None

    def to_dict(self, include_path: bool = True) -> dict:
        d = {
            "status": self.status,
            "reason": self.reason,
            "mode": self.mode,
            "heuristic": self.heuristic,
            "expansions": self.expansions,
            "generated": self.generated,
            "rejected": self.rejected,
            "runtime_s": self.runtime_s,
            "cost_um_equivalent": (None if math.isinf(self.cost) else self.cost),
            "length_um": (None if math.isinf(self.length_um) else self.length_um),
            "congestion_cost_um_equivalent": self.congestion_cost,
            "budget": {"max_expansions": self.max_expansions,
                       "max_runtime_s": self.max_runtime_s},
            "budget_exhausted": self.budget_exhausted,
            "state_identity": self.state_identity,
            "approximate_state_identity": self.approximate_state_identity,
            "reference_graph": self.reference_graph,
            "finite_graph": self.finite_graph,
            "timings": self.timings,
            "rejection_summary": self.rejection_summary,
            "trace": self.trace,
            "initial_candidates": self.initial_candidates,
            "goal_pos_error_um": self.goal_pos_error_um,
            "goal_tangent_error_deg": self.goal_tangent_error_deg,
            "zero_length_solution": self.zero_length_solution,
            "n_segments": self.n_segments,
            "endpoint_errors": self.endpoint_errors,
            "port_assurance": self.port_assurance,
        }
        if include_path:
            # An explicit JSON null: "path": null means "no primitive was
            # returned".  It does NOT by itself mean failure - zero_length_solution
            # above is what distinguishes an empty legal solution from a failure.
            d["path"] = self.path.to_dict() if self.path is not None else None
        return d


def astar_search(
    start_state: State3D,
    generate: Callable[[State3D, bool], Tuple[List[Candidate], List[Candidate]]],
    goal_test: Callable[[State3D], Tuple[bool, float, float]],
    h_fn: Callable[[State3D], float],
    key_fn: Callable[[State3D], tuple],
    cfg: RouterConfig,
    mode: str = "native3d",
    heuristic_name: str = "3d_euclidean_minus_eps",
    source_invalid_reason: Optional[str] = None,
    target_invalid_reason: Optional[str] = None,
    max_expansions: Optional[int] = None,
    max_runtime_s: Optional[float] = None,
    expand_hook: Optional[Callable[[Node, float, float], None]] = None,
    state_identity: str = POSE_FLOAT_STATE_IDENTITY,
    approximate_state_identity: bool = False,
) -> SearchResult:
    max_exp = int(cfg.max_expansions if max_expansions is None else max_expansions)
    max_rt = float(cfg.max_runtime_s if max_runtime_s is None else max_runtime_s)
    t0 = time.perf_counter()
    ident = dict(state_identity=state_identity,
                 approximate_state_identity=bool(approximate_state_identity))

    if source_invalid_reason is not None:
        return SearchResult("invalid_source_pose", source_invalid_reason, mode,
                            heuristic_name, 0, 0, 0, time.perf_counter() - t0,
                            max_expansions=max_exp, max_runtime_s=max_rt, **ident)
    if target_invalid_reason is not None:
        return SearchResult("invalid_target_pose", target_invalid_reason, mode,
                            heuristic_name, 0, 0, 0, time.perf_counter() - t0,
                            max_expansions=max_exp, max_runtime_s=max_rt, **ident)

    nodes: List[Node] = [Node(start_state, 0.0, -1, "<source>", "source", 0)]
    counter = 0
    heap: List[Tuple[float, float, int, int]] = [(h_fn(start_state), 0.0, counter, 0)]
    best_g: Dict[tuple, float] = {key_fn(start_state): 0.0}
    closed: set = set()
    expansions = 0
    generated = 0
    rejected = 0
    rej_summary: Dict[str, int] = {}
    initial_candidates: List[dict] = []

    ok0, pe0, ae0 = goal_test(start_state)
    if ok0:
        # ZERO-LENGTH SOLUTION: the start state is already in the goal set, so
        # there is no primitive to concatenate.  The contract is explicit: cost
        # and length are exactly 0, path stays None (nothing is fabricated) and
        # zero_length_solution is True, while the trace still has exactly the one
        # start step so the result is not an empty object.
        return SearchResult("success", "source pose already satisfies the goal test",
                            mode, heuristic_name, 0, 0, 0, time.perf_counter() - t0,
                            cost=0.0, length_um=0.0, path=None,
                            trace=_trace(nodes, 0),
                            goal_pos_error_um=pe0, goal_tangent_error_deg=ae0,
                            zero_length_solution=True, n_segments=0,
                            max_expansions=max_exp, max_runtime_s=max_rt, **ident)

    while heap:
        f, g, _, idx = heapq.heappop(heap)
        node = nodes[idx]
        nkey = key_fn(node.state)
        if g_is_worse(node.g, best_g.get(nkey, math.inf)):
            continue                      # stale heap entry: a better g exists
        if nkey in closed:
            continue                      # already expanded with g' <= node.g
        closed.add(nkey)

        # CORRECT termination: only a goal state POPPED with the minimal f ends
        # the search.  A goal check on generation is not a valid proof.
        ok, pe, ae = goal_test(node.state)
        if ok:
            path = _build_path(nodes, idx)
            return SearchResult("success",
                                "goal state popped with minimal f=%.12g after %d "
                                "expansions" % (f, expansions),
                                mode, heuristic_name, expansions, generated, rejected,
                                time.perf_counter() - t0, cost=node.g,
                                length_um=path.total_length,
                                congestion_cost=node.g - path.total_length, path=path,
                                trace=_trace(nodes, idx),
                                initial_candidates=initial_candidates,
                                rejection_summary=rej_summary,
                                max_expansions=max_exp, max_runtime_s=max_rt,
                                goal_pos_error_um=pe, goal_tangent_error_deg=ae,
                                zero_length_solution=False,
                                n_segments=len(path.segments), **ident)

        if expansions >= max_exp:
            return SearchResult("budget_exhausted",
                                "max_expansions=%d reached; this is NOT a proof of unreachability"
                                % max_exp, mode, heuristic_name, expansions, generated,
                                rejected, time.perf_counter() - t0,
                                trace=_trace(nodes, idx), initial_candidates=initial_candidates,
                                rejection_summary=rej_summary,
                                max_expansions=max_exp, max_runtime_s=max_rt,
                                budget_exhausted=True, **ident)
        elapsed = time.perf_counter() - t0
        if elapsed > max_rt:
            return SearchResult("budget_exhausted",
                                "max_runtime_s=%g reached; this is NOT a proof of unreachability"
                                % max_rt, mode, heuristic_name, expansions, generated,
                                rejected, elapsed, trace=_trace(nodes, idx),
                                initial_candidates=initial_candidates,
                                rejection_summary=rej_summary,
                                max_expansions=max_exp, max_runtime_s=max_rt,
                                budget_exhausted=True, **ident)

        if expand_hook is not None:
            expand_hook(node, f, h_fn(node.state))
        collect = (len(initial_candidates) == 0 and expansions == 0)
        accepted, rej = generate(node.state, collect)
        expansions += 1
        if collect:
            for c in accepted:
                initial_candidates.append(_cand_dict(c, h_fn))
            for c in rej:
                initial_candidates.append(_cand_dict(c, h_fn))
        for c in rej:
            rejected += 1
            rej_summary[c.reason] = rej_summary.get(c.reason, 0) + 1

        for c in accepted:
            generated += 1
            ng = node.g + c.cost
            nkey2 = key_fn(c.new_state)
            prev = best_g.get(nkey2)
            if prev is not None and not g_improves(ng, prev):
                continue                  # same state reached no better: keep the old
            best_g[nkey2] = ng
            closed.discard(nkey2)         # a genuine improvement re-opens the state
            nodes.append(Node(c.new_state, ng, idx, c.action.name,
                              getattr(c.action, "kind", "motion"), node.depth + 1,
                              tuple(c.action.segments)))
            counter += 1
            heapq.heappush(heap, (ng + h_fn(c.new_state), ng, counter, len(nodes) - 1))

    if state_identity == "canonical_reference_graph":
        reason = ("open set empty after %d expansions: the FINITE canonical reference "
                  "graph is exhausted (nodes are canonical poses, edges are exact "
                  "Dubins connectors that end on canonical poses).  No connector route "
                  "with at most two hops of this reference graph reaches the goal. "
                  "This is a real exhaustion, not a budget stop; it is a statement "
                  "about the reference graph, not about the continuous pose space."
                  % expansions)
    elif approximate_state_identity:
        reason = ("open set empty after %d expansions: the finite QUANTISED abstract "
                  "graph (state_identity=%s, cell %g um / %g deg) reachable from the "
                  "source is exhausted.  This is NOT a proof that the continuous pose "
                  "space is unreachable: two poses in one cell are different states. "
                  "Use state_identity='exact' and a finite reference graph / geometric "
                  "argument for any unreachability claim."
                  % (expansions, state_identity, cfg.pos_resolution_um,
                     cfg.dir_resolution_deg))
    else:
        reason = ("open set empty after %d expansions: the enumerated pose graph "
                  "(state_identity=%s, the stored float pose with no rounding) "
                  "reachable from the source is exhausted, so no sequence of the "
                  "configured primitives that the constraint stack accepts reaches "
                  "the goal set.  This is a statement about the enumerated graph, "
                  "not about the continuous pose space."
                  % (expansions, state_identity))
    return SearchResult("search_exhausted", reason,
                        mode, heuristic_name, expansions, generated, rejected,
                        time.perf_counter() - t0, trace=[],
                        initial_candidates=initial_candidates,
                        rejection_summary=rej_summary,
                        max_expansions=max_exp, max_runtime_s=max_rt, **ident)


def _cand_dict(c: Candidate, h_fn) -> dict:
    a = c.action
    d = {"name": a.name, "kind": getattr(a, "kind", "motion"),
         "accepted": c.accepted, "reason": c.reason,
         "length_um": a.length,
         "z_change_um": a.z_change_um,
         "elevation_change_deg": a.elevation_change_deg,
         "h_after_um": h_fn(c.new_state),
         "end_position": list(c.new_state.position),
         "end_direction": list(c.new_state.direction)}
    if c.validation is not None:
        d["validation_ok"] = c.validation.get("ok")
        d["validation_failed"] = c.validation.get("failed", [])
        d["clearance_bound_um"] = c.validation.get("clearance_bound_um")
    return d


def _trace(nodes: List[Node], goal_idx: int) -> List[dict]:
    chain: List[Node] = []
    i = goal_idx
    while i >= 0:
        chain.append(nodes[i])
        i = nodes[i].parent
    chain.reverse()
    out = []
    g = 0.0
    for k, n in enumerate(chain):
        out.append({"step": k, "action": n.action_name, "kind": n.action_kind,
                    "n_segments": len(n.segments),
                    "x_um": n.state.position[0], "y_um": n.state.position[1],
                    "z_um": n.state.position[2],
                    "direction": list(n.state.direction),
                    "azimuth_deg": n.state.azimuth_deg,
                    "elevation_deg": n.state.elevation_deg,
                    "g_um_equivalent": n.g,
                    "depth": n.depth})
    return out


def _build_path(nodes: List[Node], goal_idx: int) -> Path3D:
    """Raw searched path: concatenation of the primitives of the node chain."""
    chain: List[Node] = []
    i = goal_idx
    while i >= 0:
        chain.append(nodes[i])
        i = nodes[i].parent
    chain.reverse()
    segs: List[object] = []
    for n in chain:
        segs.extend(n.segments)
    if not segs:
        raise ValueError("node chain produced no primitive (source == goal)")
    return Path3D(tuple(segs))