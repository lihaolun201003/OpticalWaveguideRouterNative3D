"""Finite canonical reference graph (v0.1.1).

WHY IT EXISTS
-------------
The production search (search/router.py, state identity "exact") keeps every
real pose.  That graph is generally infinite and A* with the CORRECT termination
must expand every state whose f is below the optimum, so on the wall scenes the
production run ends in budget_exhausted.  This is measured, not guessed: on the
core acceptance scene the A* frontier moves from f = 486.4 to f = 491.0 while
expanding 20 000 states, so the number of states below any interesting optimum
grows super-exponentially (see docs/reports/native3d_v011_correctness_report.md).
A budget stop is never unreachability, and it is not usable acceptance evidence
either.

The v0.1.1 acceptance therefore adds this explicitly FINITE reference graph:

* NODES are canonical poses - positions on a lattice anchored at the source pose
  and directions from the 26 integer directions of the cube - plus the source
  pose and the target pose.  The node set is fixed by construction and finite.
* EDGES are single continuous primitive sequences: one planar Dubins
  arc-straight-arc connector (or the degenerate straight connector) that starts
  at the canonical pose of its source node and ends EXACTLY on the canonical pose
  of its target node.  Successors are therefore a deterministic function of the
  node, the assembled path is exactly C1 connected, and NOTHING has to be snapped
  or repaired afterwards.
* every edge is checked by the SAME full constraint stack as the production
  router (min radius, boundary, obstacle keep-out, centreline spacing) before it
  may enter the open set, and the search is the SAME astar_search, with the same
  correct termination and the same admissible Euclidean heuristic, so the route
  it returns is optimal FOR THIS FINITE GRAPH.

The graph contains the direct source->target connector and every
source->mid->target pair of connectors, with "mid" ranging over the canonical
pose set.  That keeps it small (a few thousand nodes) while covering the
"climb, then connect" and "S-curve, then connect" route shapes.

WHAT IT DOES AND DOES NOT PROVE
-------------------------------
* success -> a genuine, fully constrained, C1 route with per-segment search
  provenance (the same evidence standard as the production router).
* search_exhausted -> no route exists in THIS finite graph.  That is a real
  exhaustion (not a budget stop), but it is a statement about this reference
  graph.  For the 2D control mode the unreachability conclusion is additionally
  carried by the geometric separation argument recorded with the case evidence
  (the wall's keep-out spans the whole chip cross-section of the z = plane, so no
  continuous legal curve can cross it).
* it is NOT a complete search of the continuous pose space and it is NOT the
  production router state strategy.
"""

from __future__ import annotations

import itertools
import math
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import validate_segments
from ..geometry.primitives import Pose3D
from ..geometry.vec import norm, sub
from .actions import primitive_is_in_plane
from .astar import Candidate, SearchResult, astar_search
from .congestion import CongestionField, ZeroCongestion
from .connector import planar_connectors
from .state import State3D

# The 26 integer directions of the cube.
#
# ERRATUM (v0.1.2): the v0.1.1 report claimed the coplanarity of two integer
# directions and a lattice offset was decided by an EXACT integer test
# (offset . (t x d) == 0) that "never depends on a floating point tolerance".
# No such test exists anywhere in this package.  Coplanarity is decided inside
# connector.planar_connectors() by comparing |dot(dp, n)| against the FLOATING
# POINT tolerance coplanar_tol_um = 1e-6 um, and the directions are converted to
# unit float vectors before use.  It is a float test with a documented tolerance,
# not an exact integer test, and the v0.1.2 report says so.
CANONICAL_DIRECTIONS: Tuple[Tuple[int, int, int], ...] = tuple(
    d for d in itertools.product((-1, 0, 1), repeat=3) if d != (0, 0, 0))


def _unit(d: Tuple[int, int, int]) -> Tuple[float, float, float]:
    n = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
    return (d[0] / n, d[1] / n, d[2] / n)


def canonical_poses(scene, spacing_um: float,
                    include_source_tangent: bool = True) -> List[Pose3D]:
    """The finite canonical mid pose set: lattice x 26 directions, feasible only.

    The lattice is anchored at the source position, so the source and (when the
    target happens to lie on it) the target are lattice points themselves.
    """
    cfg = scene.config
    inner = scene.chip.shrink(cfg.effective_boundary_margin_um)
    src = scene.source.position
    span = [inner.hi[i] - inner.lo[i] for i in range(3)]
    n_i = int(span[0] / spacing_um) + 2
    n_j = int(span[1] / spacing_um) + 2
    n_k = int(span[2] / spacing_um) + 2
    out: List[Pose3D] = []
    seen = set()
    for i in range(-n_i, n_i + 1):
        for j in range(-n_j, n_j + 1):
            for k in range(-n_k, n_k + 1):
                p = (src[0] + spacing_um * i, src[1] + spacing_um * j,
                     src[2] + spacing_um * k)
                if not inner.contains_point(p, 1e-9):
                    continue
                for d in CANONICAL_DIRECTIONS:
                    t = _unit(d)
                    key = (p, t)
                    if key in seen:
                        continue
                    seen.add(key)
                    out.append(Pose3D(p, t))
    return out


def _pose_key(pose: Pose3D) -> tuple:
    """Canonical-node lookup key of THIS finite reference graph.

    v0.1.1 rounded the six numbers to 9 decimals here.  That rounding is gone:
    the canonical nodes of this graph are CONSTRUCTED (source position plus an
    integer multiple of the lattice spacing, and an integer direction), so two
    constructions of the same node produce the same floats and no tolerance is
    needed.  The key is used to look a state up in the node table, never to
    decide that two different poses are one state.
    """
    return (pose.position, pose.tangent)


def best_connector_edge(scene, pose_a: Pose3D, pose_b: Pose3D,
                        mode: str = "native3d", cfg: Optional[RouterConfig] = None,
                        congestion: Optional[CongestionField] = None,
                        in_plane=None) -> Optional[Candidate]:
    """Cheapest legal single connector edge pose_a -> pose_b, or None.

    Public so that the optimality of the reference-graph A* can be cross-checked
    against a brute force enumeration of the same edge set.

    The edge is a real primitive sequence that starts exactly at pose_a and ends
    exactly at pose_b; every candidate radius of the connector family is tried and
    the whole sequence is checked by the full constraint stack before it may be
    used.  In '2d' mode the primitive must additionally stay in the plane
    z = z(pose_a) - the same rule the production 2D control mode uses.
    """
    cfg = cfg or scene.config
    field = congestion if congestion is not None else ZeroCongestion()
    allow = (mode == "native3d")

    def _default_in_plane(segs) -> bool:
        if allow:
            return True
        z0 = pose_a.position[2]
        for seg in segs:
            lo, hi = seg.aabb()
            if abs(lo[2] - z0) > 1e-9 or abs(hi[2] - z0) > 1e-9:
                return False
        return True

    check_plane = in_plane or _default_in_plane
    radii = sorted({float(r) for r in cfg.connector_radii_um} | {cfg.rmin_um})
    best: Optional[Candidate] = None
    from .actions import Action
    for R in radii:
        for segs in planar_connectors(pose_a, pose_b, R):
            if not check_plane(segs):
                continue
            total = sum(s.length for s in segs)
            if total > cfg.connector_max_length_um:
                continue
            val = validate_segments(segs, scene.chip, scene.obstacles,
                                    scene.reference_centerlines, cfg, detailed=False)
            if not val["ok"]:
                continue
            act = Action(name="ref_connector_R%g" % R, segments=tuple(segs),
                         kind="connector", radius_um=R)
            cost = total + field.penalty_of_segments(segs)
            cand = Candidate(act, State3D(pose_b.position, pose_b.tangent), cost, True, "")
            if best is None or cost < best.cost:
                best = cand
    return best


def reference_search(scene, mode: str = "native3d",
                     spacing_um: Optional[float] = None,
                     congestion: Optional[CongestionField] = None,
                     max_expansions: Optional[int] = 20_000,
                     max_runtime_s: Optional[float] = 600.0,
                     expand_hook=None) -> SearchResult:
    """A* over the finite canonical reference graph (see the module docstring)."""
    if mode not in ("2d", "native3d"):
        raise ValueError("mode must be '2d' or 'native3d'")
    cfg = scene.config
    spacing = float(cfg.reference_spacing_um if spacing_um is None else spacing_um)
    allow = (mode == "native3d")
    z0 = scene.source.position[2]
    field = congestion if congestion is not None else ZeroCongestion()

    mids = canonical_poses(scene, spacing)
    source_pose = scene.source
    target_pose = scene.target
    mids = [q for q in mids if _pose_key(q) != _pose_key(source_pose)
            and _pose_key(q) != _pose_key(target_pose)]
    mid_keys = {}
    for i, q in enumerate(mids):
        mid_keys.setdefault(_pose_key(q), i)
    target_key = _pose_key(target_pose)

    def _in_plane(segs) -> bool:
        if allow:
            return True
        for seg in segs:
            lo, hi = seg.aabb()
            if abs(lo[2] - z0) > 1e-9 or abs(hi[2] - z0) > 1e-9:
                return False
        return True

    def _edges(pose_a: Pose3D, pose_b: Pose3D) -> List[Candidate]:
        cand = best_connector_edge(scene, pose_a, pose_b, mode=mode, cfg=cfg,
                                   congestion=field, in_plane=_in_plane)
        return [cand] if cand is not None else []

    def generate(state: State3D, collect: bool):
        accepted: List[Candidate] = []
        pose = state.pose
        key = _pose_key(pose)
        if key == _pose_key(source_pose):
            for q in mids:
                accepted.extend(_edges(source_pose, q))
            accepted.extend(_edges(source_pose, target_pose))
        elif key in mid_keys:
            accepted.extend(_edges(mids[mid_keys[key]], target_pose))
        # the target node is a sink: no outgoing edges
        return accepted, []

    def goal_test(state: State3D):
        pos_err = norm(sub(state.position, target_pose.position))
        from ..geometry.vec import angle_between
        ang_err = math.degrees(angle_between(state.direction, target_pose.tangent))
        return (pos_err <= cfg.eps_pos_um and ang_err <= cfg.eps_ang_deg, pos_err, ang_err)

    def h_fn(state: State3D) -> float:
        return max(0.0, norm(sub(state.position, target_pose.position)) - cfg.eps_pos_um)

    start = State3D(source_pose.position, source_pose.tangent)
    res = astar_search(
        start, generate, goal_test, h_fn,
        lambda s: s.pose_key(), cfg, mode="native3d_reference_graph",
        heuristic_name="max(0, |target_pos - pos| - eps_pos) [3D Euclidean, admissible]",
        max_expansions=max_expansions, max_runtime_s=max_runtime_s,
        expand_hook=expand_hook,
        state_identity="canonical_reference_graph",
        approximate_state_identity=False)
    res.reference_graph = {"n_canonical_mids": len(mids), "spacing_um": spacing,
                           "directions": len(CANONICAL_DIRECTIONS),
                           "max_hops": 2,
                           "note": ("finite canonical reference graph: nodes are "
                                    "canonical poses, edges are exact Dubins "
                                    "connectors ending on canonical poses; optimality "
                                    "is claimed for THIS finite graph only")}
    return res