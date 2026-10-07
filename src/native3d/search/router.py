"""Search driver: scene + mode -> raw Path3D or an explicit failure status."""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import (Box3D, CheckResult, point_box_distance,
                                  segment_segment_distance, validate_segments)
from ..geometry.primitives import Pose3D
from ..geometry.vec import Vec3, angle_between, dot, norm, sub
from ..identity import APPROXIMATE_STATE_IDENTITY, POSE_FLOAT_STATE_IDENTITY
from .actions import generate_actions, primitive_is_in_plane
from .astar import Candidate, SearchResult, astar_search
from .congestion import CongestionField, ZeroCongestion
from .connector import connector_actions
from .state import State3D


def point_segment_distance(p: Vec3, a: Vec3, b: Vec3) -> float:
    return segment_segment_distance(p, p, a, b)


def pose_invalid_reason(scene, pose: Pose3D) -> Optional[str]:
    cfg = scene.config
    inner = scene.chip.shrink(cfg.effective_boundary_margin_um)
    if not inner.contains_point(pose.position, 1e-9):
        return ("pose position %r outside the chip volume shrunk by the boundary "
                "margin %g um" % (list(pose.position), cfg.effective_boundary_margin_um))
    req = cfg.effective_obstacle_clearance_um
    for k, box in enumerate(scene.obstacles):
        d = point_box_distance(pose.position, box)
        if d < req - cfg.clearance_decision_tol_um:
            return ("pose position is %.6g um from obstacle %d, below the required "
                    "keep-out %.6g um" % (d, k, req))
    for k, ref in enumerate(scene.reference_centerlines):
        d = point_segment_distance(pose.position, ref.p0, ref.p1) if ref.kind == "line" \
            else None
        if d is not None and d < cfg.dmin_centerline_um - cfg.clearance_decision_tol_um:
            return ("pose position is %.6g um from reference centreline %d, below "
                    "dmin %.6g um" % (d, k, cfg.dmin_centerline_um))
    return None


STRATEGIES = ("continuous_pose", "reference_graph", "finite_native3d")


def route(scene, mode: str = "native3d", congestion: Optional[CongestionField] = None,
          max_expansions: Optional[int] = None,
          max_runtime_s: Optional[float] = None,
          expand_hook=None, strategy: str = "continuous_pose") -> SearchResult:
    """Dispatch to one of the THREE explicitly named strategies.

    strategy='continuous_pose' (default) -> the continuous-pose debug search of
        this module (state identity "pose_float"; generally infinite graph).
    strategy='reference_graph' -> the v0.1.1 finite canonical reference graph
        (search/reference_graph.py; at most two connector hops).
    strategy='finite_native3d' -> the v0.1.2 PRODUCTION finite multi-hop native
        3D router (search/finite_graph.py; integer node ids, arbitrary hops).

    The three must never be confused in reports, examples or artifacts, so the
    strategy is returned on the result object as well.
    """
    if strategy in ("continuous_pose", "continuous"):
        return _route_continuous(scene, mode=mode, congestion=congestion,
                                 max_expansions=max_expansions,
                                 max_runtime_s=max_runtime_s, expand_hook=expand_hook)
    if strategy == "reference_graph":
        from .reference_graph import reference_search
        return reference_search(scene, mode=mode, congestion=congestion,
                                max_expansions=max_expansions,
                                max_runtime_s=max_runtime_s, expand_hook=expand_hook)
    if strategy == "finite_native3d":
        from .finite_graph import route_finite_native3d
        return route_finite_native3d(scene, mode=mode, congestion=congestion,
                                     max_expansions=max_expansions,
                                     max_runtime_s=max_runtime_s,
                                     expand_hook=expand_hook)
    raise ValueError("strategy must be one of %r, got %r" % (list(STRATEGIES), strategy))


def _route_continuous(scene, mode: str = "native3d",
                      congestion: Optional[CongestionField] = None,
                      max_expansions: Optional[int] = None,
                      max_runtime_s: Optional[float] = None,
                      expand_hook=None) -> SearchResult:
    """CONTINUOUS-POSE DEBUG / REFERENCE strategy: the v0.1 native 3D A* engine.

    Run the SAME engine in 'native3d' mode or in the '2d' control mode.

    mode='2d' restricts the action set to primitives that keep z == z(source)
    (straight + yaw) and rejects anything leaving that plane; no external 2D
    router is called anywhere in this package.

    Which strategy is which (v0.1.2 keeps all three and they must not be mixed):

    * route()                      - continuous pose debug search (this function).
                                     state identity "pose_float" by default; the
                                     enumerated graph is generally INFINITE, so a
                                     run usually ends in budget_exhausted, and a
                                     budget stop is never unreachability.
    * reference_search()           - the v0.1.1 two-hop finite reference graph
                                     (search/reference_graph.py), kept for
                                     comparison; max_hops = 2.
    * route_finite_native3d()      - the v0.1.2 PRODUCTION router
                                     (search/finite_graph.py): integer node ids,
                                     a finite lattice x six axis directions,
                                     arbitrary many hops.
    """
    if mode not in ("2d", "native3d"):
        raise ValueError("mode must be '2d' or 'native3d'")
    cfg = scene.config
    allow = (mode == "native3d")
    field = congestion if congestion is not None else ZeroCongestion()
    z0 = scene.source.position[2]

    def generate(state: State3D, collect: bool) -> Tuple[List[Candidate], List[Candidate]]:
        pose = state.pose
        actions = list(generate_actions(pose, cfg, allow))
        actions.extend(connector_actions(pose, scene.target, cfg, allow))
        accepted: List[Candidate] = []
        rejected: List[Candidate] = []
        for action in actions:
            if not allow and not primitive_is_in_plane(action, z0, 1e-9):
                if collect:
                    rejected.append(Candidate(action, state, math.inf, False,
                                              "out_of_plane_forbidden_in_2d_mode"))
                continue
            val = validate_segments(action.segments, scene.chip, scene.obstacles,
                                    scene.reference_centerlines, cfg, detailed=collect)
            new_state = State3D(action.end_pose.position, action.end_pose.tangent)
            if val["ok"]:
                cost = action.length + field.penalty_of_segments(action.segments)
                accepted.append(Candidate(action, new_state, cost, True, "",
                                          val if collect else None))
            else:
                if collect:
                    rejected.append(Candidate(action, new_state, math.inf, False,
                                              ",".join(val.get("failed", ["unknown"])),
                                              val))
        return accepted, rejected

    def goal_test(state: State3D):
        pos_err = norm(sub(state.position, scene.target.position))
        ang_err = math.degrees(angle_between(state.direction, scene.target.tangent))
        return (pos_err <= cfg.eps_pos_um and ang_err <= cfg.eps_ang_deg, pos_err, ang_err)

    def h_fn(state: State3D) -> float:
        return max(0.0, norm(sub(state.position, scene.target.position)) - cfg.eps_pos_um)

    approximate = (cfg.state_identity == APPROXIMATE_STATE_IDENTITY)

    def key_fn(state: State3D):
        """State identity of the search graph.

        "pose_float" (default) is the identity of the stored pose itself: the six
        position/direction floats, with no rounding, so two states are merged
        only when they are the same pose.  "approximate_quantised" is the v0.1
        coarse cell key and is an explicitly approximate exploration mode.
        """
        if approximate:
            return state.key(cfg.pos_resolution_um, cfg.dir_resolution_deg)
        return state.pose_key()

    start = State3D(scene.source.position, scene.source.tangent)
    res = astar_search(
        start, generate, goal_test, h_fn, key_fn, cfg, mode=mode,
        heuristic_name="max(0, |target_pos - pos| - eps_pos)  [3D Euclidean lower bound]",
        source_invalid_reason=pose_invalid_reason(scene, scene.source),
        target_invalid_reason=pose_invalid_reason(scene, scene.target),
        max_expansions=max_expansions, max_runtime_s=max_runtime_s,
        expand_hook=expand_hook,
        state_identity=cfg.state_identity,
        approximate_state_identity=approximate)
    res.rejection_summary = dict(res.rejection_summary)
    return res