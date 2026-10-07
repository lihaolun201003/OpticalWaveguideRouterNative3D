"""v0.1.2 PRODUCTION router: finite, multi-hop, native 3D state graph.

WHAT THIS IS (and what it is not)
---------------------------------
This is the v0.1.2 production strategy for the single-net native 3D router.  It
is NOT the continuous-pose debug search of search/router.py (whose enumerated
graph is generally infinite and usually ends in budget_exhausted) and it is NOT
the v0.1.1 two-hop reference graph of search/reference_graph.py (whose successors
are limited to source -> target and source -> mid -> target).  All three live in
this package and the evidence distinguishes them explicitly.

THE FINITE GRAPH
----------------
* NODE IDENTITY IS AN INTEGER TUPLE.  A canonical node is (i, j, k, d) with
  i, j, k integers and d an index into the SIX axis directions
  +x, -x, +y, -y, +z, -z.  Each id has EXACTLY ONE canonical pose:

      position(i, j, k) = lattice_anchor + spacing * (i, j, k)
      tangent(d)        = the d-th axis direction, converted to a unit vector

  and every edge is generated FROM that canonical pose.  Two ids are the same
  node iff they are the same tuple; de-duplication is by node id, never by a
  rounded or tolerance-merged pose.  The lattice anchor is the source position
  (the port phase is therefore fixed by the input, not chosen by the router).
* FINITE BY CONSTRUCTION.  The integer index ranges follow from the chip volume
  shrunk by the boundary margin, so the node set, the edge set and their upper
  bounds are computed from the construction (recorded under "bounds" in the
  evidence) instead of being inferred from "the search stopped expanding".
* ACTIONS (all generated from the canonical pose of the node):
    - straight along d by n * spacing, n in finite_straight_steps;
    - a +-90 degree circular arc in the coordinate plane spanned by d and a
      PERPENDICULAR axis direction d', with radius m * spacing,
      m in finite_turn_radius_steps and m * spacing >= Rmin.
  A 90 degree arc from d to d' has displacement R * (d + d'), so its end point
  is again a lattice point and its end tangent is exactly the axis direction d'.
  The arc is produced by the same CircularArc3D.from_pose() geometry as every
  other primitive, so it is a genuine space curve in a coordinate plane.
* The direction set contains the three axes in both senses, so the FIRST search
  step already has a z degree of freedom: the turn families that involve +-z
  change z by +-R (and the ones between two horizontal directions keep z, which
  is the horizontal yaw).  Nothing is a "2D orientation plus a layer number".
* MID -> MID SUCCESSORS.  Every canonical node has the same successor rule; there
  is no source / mid / target classification and no hop limit.  A path may walk
  an arbitrary number of intermediate nodes.

EDGES ARE FULLY CHECKED CONTINUOUS GEOMETRY
-------------------------------------------
Before an edge may enter OPEN, the complete continuous curve is checked by the
same constraint stack as the rest of the package (min bend radius, chip
boundary, obstacle keep-out, reference-centreline spacing) with
constraints.checks.validate_segments.  The curve is generated from the canonical
pose, never snapped to it afterwards; the difference between the curve end point
and the canonical target pose is recorded per edge (it is zero by construction up
to floating point, and the recorded maximum is reported in the evidence).

PORTS
-----
The source and the target are VIRTUAL nodes.  They are connected to the
canonical graph by validated attachment curves:
  - a single straight attachment when the port tangent is one of the six axis
    directions and a canonical node of that direction lies on the port ray
    within finite_port_attach_um (including the zero-length case where a
    canonical node IS the port pose);
  - otherwise a v0.1.1 planar Dubins arc-straight-arc connector
    (search/connector.planar_connectors) to the nearest canonical nodes inside
    the attachment neighbourhood, which covers coplanar pose pairs with an
    arbitrary port tangent.
If neither exists, the result is port_attachment_failed / unsupported_pose; that
is a statement about the SUPPORTED ACTION SET, never about physical
reachability.  The attachment neighbourhood is a generic configuration radius
and a cap on the number of candidate nodes; no case-specific waypoint, height or
route is ever supplied to the router.

SEARCH AND GUARANTEES
---------------------
The search is the shared search/astar.py with the documented cost
    cost = real_path_length + nonnegative_congestion_penalty
and the admissible 3D Euclidean heuristic h = max(0, |target - pos| - eps_pos).
The state identity used for de-duplication is the integer node id.  The status
vocabulary is defined once in search/statuses.py and every status is distinct:
    success, search_exhausted, budget_exhausted, port_attachment_failed,
    graph_scene_mismatch, invalid_source_pose, invalid_target_pose,
    unsupported_pose, constraint_failed, uncertified.
  - search_exhausted means THE CONSTRUCTED FINITE GRAPH was exhausted, which is a
    real exhaustion and not a budget stop;
  - optimality is claimed for the constructed finite graph only, up to the
    documented cost comparison tolerance of search/astar.py, and never for the
    continuous pose space;
  - congestion is a fixed non-negative single-net cost; no multi-net scheduling.

GRAPH REUSE CONTRACT (v0.1.3)
----------------------------
``route_finite_native3d(scene, graph=prebuilt)`` is a PUBLIC reuse point, and
reuse is only legal when the graph really describes the input being routed.  A
FiniteGraph therefore records an immutable CONSTRUCTION SNAPSHOT
(``graph.snapshot``) taken at construction time:

* the chip volume with coordinates;
* EVERY obstacle box with its coordinates;
* EVERY reference centreline with its FULL geometry (kind/p0/p1 or
  centre/u/v/radius/theta) - not a count;
* the source and target position AND 3D tangent;
* the mode ("2d" / "native3d");
* every configuration field that can change an edge's geometry, feasibility,
  cost or the endpoint contract (spacing, straight steps, turn radius steps,
  rmin/radius tolerance, centreline spacing, obstacle clearance and its envelope
  flag, envelope radius, boundary margin (raw and effective), clearance decision
  tolerance, subdivision cap, port attachment radius and candidate cap,
  connector radii, the C1/endpoint tolerances and the finite budget);
* the congestion field's ``identity()`` token.

Before ANY search the snapshot is compared with the scene/config/mode/congestion
actually passed.  On a mismatch the call returns ``graph_scene_mismatch`` with
``path=None``, ``cost=inf``, ``length_um=inf``, ``goal_pos_error_um=None`` and a
structured diff under ``res.finite_graph["graph_reuse"]``.  It NEVER returns
``success`` and never ``search_exhausted``, because the search never ran.

Object identity is NOT a valid check (``Scene`` is a mutable dataclass and the
suite mutates ``scene.obstacles`` / ``scene.chip`` after construction), so the
comparison is by CONTENT.  The cost of that choice is explicit: building and
comparing the snapshot is O(size of the scene), not O(1), and two scenes that
differ only in a field no edge can observe are still reported as a mismatch
(conservative refusal, never a silent stale reuse).

The node-id <-> pose table is a BIJECTION.  A pose key that would be assigned
twice is never silently overwritten: it is kept for the first node and recorded
in ``graph.pose_key_collisions`` (empty in the normal case).  The virtual source
and target nodes are registered in ``graph.virtual_pose_keys`` only when no
canonical node already has their pose, so ``pose_key_to_node`` never has to
guess.

SUCCESS IS DECIDED BY THE ACTUAL ASSEMBLED CURVE (v0.1.3)
--------------------------------------------------------
Every accepted edge must really END on the canonical pose of its target node
within ``pos_tol_um`` / ``ang_tol_deg``; an edge that misses it is REJECTED with
the reason ``curve_end_misses_the_canonical_pose`` (counted in
``graph.edges_end_pose_rejected``), and an edge with an internal junction beyond
the same tolerances (a multi-segment connector) is rejected with
``curve_junction_exceeds_continuity_tolerance``.  Before success is reported, the
ACTUAL assembled curve is compared with the pose the caller passed by
``verify_assembled_endpoints`` (source with the source tolerances, target with
the target acceptance ball, every junction with the C1 tolerances).  A failure
there returns ``constraint_failed`` (or ``uncertified`` when a check could not be
decided) and leaves ``path=None`` with no port-assurance block.  v0.1.2
relabelled such a failure ``search_exhausted``, which described the opposite of
what had happened.

ZERO-LENGTH SOLUTIONS (v0.1.3)
------------------------------
When the source pose already satisfies the goal test there is no primitive to
concatenate.  The result is a first-class success with ``zero_length_solution is
True``, ``cost == length_um == congestion_cost == 0``, ``path is None``,
``n_segments == 0``, ``expansions == 0`` and a one-step trace.  Nothing is
fabricated (no micro-segment, no degenerate primitive).  A FAILURE also has
``path is None``, so ``zero_length_solution`` is what separates them; it is
False on every failure.  The source pose is still validated (inside the chip,
inside an obstacle keep-out, on a fixed reference centreline) and a target
tangent outside the acceptance ball never becomes a zero-length win.

BOUNDS, TIMING AND CACHE PROVENANCE (v0.1.3, PART C)
----------------------------------------------------
* ``bounds["edge_upper_bound"]`` is DERIVED: every canonical node can generate at
  most ``len(finite_straight_steps)`` straights plus one turn per (perpendicular
  axis direction, turn radius step) pair, plus the port attachment edges that
  were really built.  The v0.1.2 hardcoded ``+ 4096`` is gone;
  ``bounds["node_upper_bound_includes_virtual_ports"]`` states that the node
  bound counts CANONICAL nodes only, and the virtual ports are reported
  separately.
* The per-edge continuous checks are attributed to the phase they really happen
  in: port attachments are validated while the graph is CONSTRUCTED
  (``edge_check_s_graph_build``) and every canonical successor is validated
  lazily when the search first expands its node
  (``edge_check_s_search``).  ``edge_check_s`` is their sum for THIS call, and
  ``graph_object_totals`` reports the cumulative totals of the graph object.
  The v0.1.2 wording ("edge_check_s is a subset of search_s") was false for the
  construction-phase checks and is no longer used.
* ``res.finite_graph["graph_reuse"]`` reports ``graph_cache_state`` ("cold" for a
  graph this call built itself, "warm" for a caller-supplied graph it reused),
  ``graph_build_count``, ``successor_cache_hits`` and the structured mismatch
  diff, so the cold default branch and a warm reuse are never conflated.

DISCRETE-ACTION LIMITATION (disclosed, not hidden)
--------------------------------------------------
A lattice of spacing h with quarter turns of radius m*h cannot reproduce every
continuous route: the set of reachable positions is a sublattice, the turning
radius is quantised to multiples of h, and a shape that cannot be assembled from
axis-aligned quarter turns is simply not in the graph.  A longer legal route than
the v0.1.1 reference graph produced is therefore expected and is not a defect.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import Box3D, validate_segments
from ..geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from ..geometry.vec import (Vec3, add, angle_between, cross, dot, norm, scale,
                            sub, unit)
from ..identity import POSE_FLOAT_STATE_IDENTITY
from .actions import Action, primitive_is_in_plane
from .astar import Candidate, SearchResult, astar_search
from .congestion import CongestionField, ZeroCongestion
from .connector import planar_connectors
from .router import pose_invalid_reason
from .state import State3D

# The six axis directions of the finite graph.  Order is part of the node id.
AXIS_DIRECTIONS: Tuple[Tuple[int, int, int], ...] = (
    (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))

SOURCE_NODE: tuple = ("source",)
TARGET_NODE: tuple = ("target",)

HALF_PI = math.pi / 2.0


def direction_unit(d: Tuple[int, int, int]) -> Vec3:
    return (float(d[0]), float(d[1]), float(d[2]))


def perpendicular_axis_directions(d_index: int) -> List[int]:
    """Indices of the four axis directions perpendicular to direction d_index."""
    d = AXIS_DIRECTIONS[d_index]
    out = []
    for j, e in enumerate(AXIS_DIRECTIONS):
        if dot(direction_unit(d), direction_unit(e)) == 0.0:
            out.append(j)
    return out


@dataclass
class Edge:
    """One generated edge: from a canonical/source node to a canonical/target node."""
    from_node: tuple
    to_node: tuple
    action: Action
    cost: float
    end_position_error_um: float
    end_tangent_error_deg: float
    validation: Optional[dict] = None

    def to_dict(self) -> dict:
        return {"from": list(self.from_node), "to": list(self.to_node),
                "action": self.action.name, "kind": self.action.kind,
                "cost_um": self.cost, "length_um": self.action.length,
                "n_segments": len(self.action.segments),
                "z_change_um": self.action.z_change_um,
                "elevation_change_deg": self.action.elevation_change_deg,
                "end_position_error_um": self.end_position_error_um,
                "end_tangent_error_deg": self.end_tangent_error_deg}


# =====================================================================
# v0.1.3 CONSTRUCTION CONTRACT: snapshot, fingerprint, reuse diff
# =====================================================================
# Version of the contract recorded in a graph snapshot.  A prebuilt graph may be
# reused only when a snapshot taken with THIS version compares equal, field by
# field, with the scene/config/mode/congestion really passed to the router.
GRAPH_CONTRACT_VERSION = 1

# Config fields that can change an edge's geometry, feasibility or cost, the
# node/port construction, or the endpoint contract that decides success.
CONFIG_CONTRACT_FIELDS: Tuple[str, ...] = (
    "finite_spacing_um",
    "finite_straight_steps",
    "finite_turn_radius_steps",
    "rmin_um",
    "radius_tol_um",
    "dmin_centerline_um",
    "obstacle_clearance_um",
    "obstacle_clearance_includes_envelope",
    "wg_envelope_radius_um",
    "boundary_margin_um",
    "effective_boundary_margin_um",
    "effective_obstacle_clearance_um",
    "clearance_decision_tol_um",
    "max_subdiv_level",
    "finite_port_attach_um",
    "finite_port_attach_max_nodes",
    "connector_radii_um",
    "pos_tol_um",
    "ang_tol_deg",
    "eps_pos_um",
    "eps_ang_deg",
    "source_pos_tol_um",
    "source_ang_tol_deg",
    "finite_max_expansions",
    "finite_max_runtime_s",
)

# Snapshot keys that are compared directly (one value each).
SNAPSHOT_SCALAR_FIELDS: Tuple[str, ...] = (
    "contract_version",
    "mode",
    "chip",
    "obstacles",
    "reference_centrelines",
    "source",
    "source_tangent",
    "target",
    "target_tangent",
    "congestion",
)


def _plain(value):
    """Canonical JSON-comparable form of a config/snapshot value.

    Tuples become lists so that a tuple and the list it round-trips through JSON
    compare equal; ints/floats/bools/strings/None are kept as they are.  The
    transformation is applied to BOTH sides of every comparison, so it can never
    hide a difference.
    """
    if isinstance(value, (tuple, list)):
        return [_plain(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in sorted(value.items())}
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value)
    return value


def _reference_geometry(seg) -> dict:
    """FULL serialisable geometry of one reference centreline (never a count).

    An unsupported kind is not silently dropped: it is recorded as such with its
    repr, which is the only equality evidence available for it.
    """
    kind = getattr(seg, "kind", None)
    if kind == "line":
        return {"kind": "line",
                "p0": [float(c) for c in seg.p0],
                "p1": [float(c) for c in seg.p1]}
    if kind == "arc":
        return {"kind": "arc",
                "center": [float(c) for c in seg.center],
                "u": [float(c) for c in seg.u],
                "v": [float(c) for c in seg.v],
                "radius_um": float(seg.radius),
                "theta_rad": float(seg.theta)}
    return {"kind": "unsupported", "repr": repr(seg)}


def _config_contract(cfg: RouterConfig) -> dict:
    return {name: _plain(getattr(cfg, name, None)) for name in CONFIG_CONTRACT_FIELDS}


def scene_contract_snapshot(scene, mode: str, cfg: RouterConfig,
                            congestion: Optional[CongestionField]) -> dict:
    """Immutable, JSON-serialisable construction snapshot of one routing input.

    See the module docstring (GRAPH REUSE CONTRACT) for what is covered and why
    it is compared by content rather than by object identity.  Floats are stored
    as they are, so the comparison is exact: a changed value is reported, and a
    difference no edge can observe would also be reported (conservative).
    """
    field = congestion if congestion is not None else ZeroCongestion()
    return {
        "contract_version": GRAPH_CONTRACT_VERSION,
        "mode": mode,
        "chip": {"lo": [float(c) for c in scene.chip.lo],
                 "hi": [float(c) for c in scene.chip.hi]},
        "obstacles": [{"lo": [float(c) for c in b.lo],
                       "hi": [float(c) for c in b.hi]} for b in scene.obstacles],
        "reference_centrelines": [_reference_geometry(s)
                                  for s in scene.reference_centerlines],
        "source": [float(c) for c in scene.source.position],
        "source_tangent": [float(c) for c in scene.source.tangent],
        "target": [float(c) for c in scene.target.position],
        "target_tangent": [float(c) for c in scene.target.tangent],
        "config": _config_contract(cfg),
        "congestion": field.identity(),
    }


def scene_geometry_fingerprint(scene) -> str:
    """Stable sha1 of the scene GEOMETRY only (chip, obstacles, references, ports).

    It is a short identity token for reports, not a decision: the decision is the
    field-by-field snapshot comparison, which can name the mismatching fields.
    """
    payload = {
        "chip": {"lo": [float(c) for c in scene.chip.lo],
                 "hi": [float(c) for c in scene.chip.hi]},
        "obstacles": [{"lo": [float(c) for c in b.lo],
                       "hi": [float(c) for c in b.hi]} for b in scene.obstacles],
        "reference_centrelines": [_reference_geometry(s)
                                  for s in scene.reference_centerlines],
        "source_pose": {"position": [float(c) for c in scene.source.position],
                        "tangent": [float(c) for c in scene.source.tangent]},
        "target_pose": {"position": [float(c) for c in scene.target.position],
                        "tangent": [float(c) for c in scene.target.tangent]},
    }
    text = json.dumps(_plain(payload), sort_keys=True, separators=(",", ":"))
    return hashlib.sha1(text.encode("utf-8")).hexdigest()


def graph_config_fingerprint(cfg: RouterConfig, scene, mode: str = "native3d",
                             congestion: Optional[CongestionField] = None) -> dict:
    """Everything that can change an edge's geometry, feasibility or cost.

    The static edge cache of one search is keyed by this fingerprint (plus the
    scene's own geometry), so a cached edge can never be reused with different
    geometry, constraints or costs.  v0.1.3 additionally covers the source/target
    pose, the MODE and the congestion field identity, and records the FULL
    reference-centreline geometry instead of a count; the historical
    ``spacing_um`` key is kept unchanged.  The result is JSON-serialisable.
    """
    field = congestion if congestion is not None else ZeroCongestion()
    return {
        "spacing_um": cfg.finite_spacing_um,
        "straight_steps": _plain(cfg.finite_straight_steps),
        "turn_radius_steps": _plain(cfg.finite_turn_radius_steps),
        "rmin_um": cfg.rmin_um,
        "radius_tol_um": cfg.radius_tol_um,
        "dmin_centerline_um": cfg.dmin_centerline_um,
        "obstacle_clearance_um": cfg.obstacle_clearance_um,
        "obstacle_clearance_includes_envelope":
            bool(cfg.obstacle_clearance_includes_envelope),
        "wg_envelope_radius_um": cfg.wg_envelope_radius_um,
        "boundary_margin_um": cfg.effective_boundary_margin_um,
        "obstacle_clearance_effective_um": cfg.effective_obstacle_clearance_um,
        "clearance_decision_tol_um": cfg.clearance_decision_tol_um,
        "max_subdiv_level": cfg.max_subdiv_level,
        "port_attach_um": cfg.finite_port_attach_um,
        "port_attach_max_nodes": cfg.finite_port_attach_max_nodes,
        "connector_radii_um": _plain(cfg.connector_radii_um),
        "pos_tol_um": cfg.pos_tol_um,
        "ang_tol_deg": cfg.ang_tol_deg,
        "eps_pos_um": cfg.eps_pos_um,
        "eps_ang_deg": cfg.eps_ang_deg,
        "source_pos_tol_um": cfg.source_pos_tol_um,
        "source_ang_tol_deg": cfg.source_ang_tol_deg,
        "finite_max_expansions": cfg.finite_max_expansions,
        "finite_max_runtime_s": cfg.finite_max_runtime_s,
        "mode": mode,
        "congestion": field.identity(),
        "contract_version": GRAPH_CONTRACT_VERSION,
        "chip": [list(scene.chip.lo), list(scene.chip.hi)],
        "obstacles": [[list(b.lo), list(b.hi)] for b in scene.obstacles],
        "reference_centrelines": [_reference_geometry(s)
                                  for s in scene.reference_centerlines],
        "source_pose": {"position": list(scene.source.position),
                        "tangent": list(scene.source.tangent)},
        "target_pose": {"position": list(scene.target.position),
                        "tangent": list(scene.target.tangent)},
    }


def graph_reuse_report(graph, scene, mode: str, cfg: RouterConfig,
                       congestion: Optional[CongestionField],
                       caller_supplied: bool) -> dict:
    """Field-by-field diff of a graph's construction snapshot against this input.

    ``accepted`` is True only when EVERY contract field is equal.  The report is
    JSON-serialisable and is attached to ``res.finite_graph["graph_reuse"]`` on
    every return path, so a reader can always see whether a prebuilt graph was
    reused, which fields differed and whether the graph was cold or warm.

    ``graph_cache_state`` follows provenance: "cold" for the graph this call
    built itself, "warm" for a caller-supplied graph.  A caller-supplied graph is
    reported as "warm" even when it is refused, because the refusal is not a
    cache state; ``accepted`` and ``rejected_before_search`` say what happened.
    """
    current = scene_contract_snapshot(scene, mode, cfg, congestion)
    stored = graph.snapshot
    cfg_now = current["config"]
    cfg_then = stored.get("config", {})
    mismatches: List[str] = []
    detail: Dict[str, dict] = {}

    def record(name: str, expected, got) -> None:
        mismatches.append(name)
        detail[name] = {"expected": expected, "graph": got}

    for name in SNAPSHOT_SCALAR_FIELDS:
        exp = _plain(current.get(name))
        got = _plain(stored.get(name))
        if exp != got:
            record(name, exp, got)
    for name in CONFIG_CONTRACT_FIELDS:
        exp = _plain(cfg_now.get(name))
        got = _plain(cfg_then.get(name))
        if exp != got:
            record(name, exp, got)

    accepted = not mismatches
    report = {
        "accepted": accepted,
        "caller_supplied_graph": bool(caller_supplied),
        "mismatches": mismatches,
        "mismatch_detail": detail,
        "graph_cache_state": "warm" if caller_supplied else "cold",
        "graph_build_count": 1,
        "graph_build_count_by_this_call": 0 if caller_supplied else 1,
        "graph_prebuilt_construction_s": float(getattr(graph, "build_s", 0.0)),
        "successor_cache_hits": int(graph.cache_hits),
        "scene_geometry_fingerprint": scene_geometry_fingerprint(scene),
        "graph_scene_geometry_fingerprint": graph.scene_fingerprint,
        "graph_config_fingerprint": graph.fingerprint,
        "expected_config_fingerprint": graph_config_fingerprint(
            cfg, scene, mode=mode, congestion=congestion),
        "checked_fields": list(SNAPSHOT_SCALAR_FIELDS) + list(CONFIG_CONTRACT_FIELDS),
        "contract_version": GRAPH_CONTRACT_VERSION,
        "cache_provenance": (
            "graph_cache_state is 'cold' for the graph this call constructed and "
            "'warm' for a caller-supplied graph; successor_cache_hits is that "
            "graph's successor-cache hit counter, read after the search"),
        "note": ("the graph is reused only when every contract field is equal; "
                 "the comparison is by CONTENT (Scene is mutable), and a mismatch "
                 "is refused before any search expansion" if accepted else
                 "the prebuilt graph was built for a different input and is "
                 "refused BEFORE the search: no goal node, lattice anchor or edge "
                 "of that graph is used for this call"),
    }
    if not accepted:
        report["rejected_before_search"] = True
        report["reason"] = (
            "the prebuilt FiniteGraph does not match the scene/config/mode/"
            "congestion actually passed: mismatching field(s) %s.  Refused before "
            "the search, because the goal node id, the lattice anchor, the "
            "boundary and every cached edge of that graph describe a DIFFERENT "
            "input.  Rebuild the graph from this scene (graph=None) to route it."
            % ", ".join(mismatches))
    return report


# =====================================================================
# v0.1.3: the ACTUAL assembled curve is what decides success
# =====================================================================
def _angle_deg(u, v) -> float:
    """Angle between two directions in degrees (atan2 form, no acos clamp)."""
    return math.degrees(angle_between(tuple(float(c) for c in u),
                                      tuple(float(c) for c in v)))


def _curve_contract_errors(segments, want_pose: Pose3D):
    """(end pos err, end tangent err deg, max junction pos err, max junction ang err).

    ``segments`` must be non-empty.  The end of the LAST segment is compared with
    the canonical pose of the target node and every internal junction is measured
    between the end pose of one segment and the start pose of the next.
    """
    end = segments[-1].end_pose
    perr = norm(sub(end.position, want_pose.position))
    aerr = _angle_deg(end.tangent, want_pose.tangent)
    jp = 0.0
    ja = 0.0
    for i in range(len(segments) - 1):
        a = segments[i].end_pose
        b = segments[i + 1].start_pose
        jp = max(jp, norm(sub(a.position, b.position)))
        ja = max(ja, _angle_deg(a.tangent, b.tangent))
    return perr, aerr, jp, ja


def verify_assembled_endpoints(segments, scene, cfg) -> dict:
    """Compare the ACTUAL assembled curve with the pose the CALLER passed.

    This is the check that decides success (A2).  It evaluates the real first and
    last primitive of the assembled path - not a node id, not a cached pose - and
    uses TWO SEPARATE tolerance families exactly as v0.1.2 defined them:

    * the START is compared with ``scene.source`` using ``source_pos_tol_um`` and
      ``source_ang_tol_deg`` (the target acceptance ball is NOT reused);
    * the END is compared with ``scene.target`` using the acceptance ball
      ``eps_pos_um`` / ``eps_ang_deg``;
    * every junction between consecutive primitives is compared with the C1
      tolerances ``pos_tol_um`` / ``ang_tol_deg``.

    ``segments == ()`` is the ZERO-LENGTH solution: there is no curve, so the
    caller's source pose is compared with the source tolerances and the same pose
    is checked against the target acceptance ball (that is exactly what the goal
    test asserted).  ``zero_length_solution`` says which case was evaluated.

    The returned status is "safe" or "constraint_failed"; this function never
    invents an "undecided" state (it is a comparison, not a subdivision bound).
    An empty ``failed`` list means every endpoint and junction is inside its
    tolerance.  Nothing here is a certified geometric lower bound.
    """
    segs = tuple(segments)
    source_pos_tol = float(cfg.source_pos_tol_um)
    source_ang_tol = float(cfg.source_ang_tol_deg)
    target_pos_tol = float(cfg.eps_pos_um)
    target_ang_tol = float(cfg.eps_ang_deg)
    pos_tol = float(cfg.pos_tol_um)
    ang_tol = float(cfg.ang_tol_deg)

    zero_length = (len(segs) == 0)
    if zero_length:
        start_pose = Pose3D(scene.source.position, scene.source.tangent)
        end_pose = start_pose
    else:
        start_pose = segs[0].start_pose
        end_pose = segs[-1].end_pose

    start_pos_err = norm(sub(start_pose.position, scene.source.position))
    start_ang_err = _angle_deg(start_pose.tangent, scene.source.tangent)
    end_pos_err = norm(sub(end_pose.position, scene.target.position))
    end_ang_err = _angle_deg(end_pose.tangent, scene.target.tangent)

    max_j_pos = 0.0
    max_j_ang = 0.0
    junction_failures: List[dict] = []
    for i in range(len(segs) - 1):
        a = segs[i].end_pose
        b = segs[i + 1].start_pose
        dp = norm(sub(a.position, b.position))
        da = _angle_deg(a.tangent, b.tangent)
        max_j_pos = max(max_j_pos, dp)
        max_j_ang = max(max_j_ang, da)
        if dp > pos_tol or da > ang_tol:
            junction_failures.append({"junction": i, "position_error_um": dp,
                                      "tangent_error_deg": da})

    start_pos_ok = start_pos_err <= source_pos_tol
    start_ang_ok = start_ang_err <= source_ang_tol
    end_pos_ok = end_pos_err <= target_pos_tol
    end_ang_ok = end_ang_err <= target_ang_tol
    start_ok = start_pos_ok and start_ang_ok
    end_ok = end_pos_ok and end_ang_ok

    failed: List[str] = []
    if not start_pos_ok:
        failed.append("source")
    if not start_ang_ok:
        failed.append("source_tangent")
    if not end_pos_ok:
        failed.append("target")
    if not end_ang_ok:
        failed.append("target_tangent")
    if junction_failures:
        failed.append("junction")
    ok = not failed

    return {
        "ok": ok,
        "status": "safe" if ok else "constraint_failed",
        "failed": failed,
        "n_segments": len(segs),
        "zero_length_solution": zero_length,
        "start_position_error_um": start_pos_err,
        "start_tangent_error_deg": start_ang_err,
        "end_position_error_um": end_pos_err,
        "end_tangent_error_deg": end_ang_err,
        "start_ok": start_ok,
        "end_ok": end_ok,
        "max_junction_position_error_um": max_j_pos,
        "max_junction_tangent_error_deg": max_j_ang,
        "junction_failures": junction_failures,
        "source_pos_tol_um": source_pos_tol,
        "source_ang_tol_deg": source_ang_tol,
        "target_eps_pos_um": target_pos_tol,
        "target_eps_ang_deg": target_ang_tol,
        "pos_tol_um": pos_tol,
        "ang_tol_deg": ang_tol,
        "method": ("the FIRST and LAST primitive of the assembled path are "
                   "evaluated at their own endpoints and compared with the source "
                   "and target poses the caller passed; every junction is "
                   "measured between the end pose of one primitive and the start "
                   "pose of the next.  Achieved errors, not certified bounds."),
        "guarantee": ("ok=True means the returned curve really starts at the "
                      "source pose (source tolerances), really ends on the target "
                      "acceptance ball, and is C0/C1 continuous inside the C1 "
                      "tolerances.  It is NOT a proof about any other point of "
                      "the curve; the hard constraints of the whole curve are "
                      "checked separately by constraints.checks.validate_segments."),
    }


class FiniteGraph:
    """The finite canonical node/edge construction of one scene and one mode.

    A FiniteGraph is built for EXACTLY ONE input and records the construction
    snapshot it was built from (``self.snapshot``); ``route_finite_native3d``
    refuses to reuse it for any other scene/config/mode/congestion (see the
    module docstring, GRAPH REUSE CONTRACT).
    """

    def __init__(self, scene, mode: str = "native3d", cfg: Optional[RouterConfig] = None,
                 congestion: Optional[CongestionField] = None):
        if mode not in ("2d", "native3d"):
            raise ValueError("mode must be '2d' or 'native3d'")
        t_build0 = time.perf_counter()
        self.scene = scene
        self.mode = mode
        self.cfg = cfg if cfg is not None else scene.config
        self.field = congestion if congestion is not None else ZeroCongestion()
        self.allow_out_of_plane = (mode == "native3d")
        self.z0 = float(scene.source.position[2])

        self.spacing = float(self.cfg.finite_spacing_um)
        self.straight_steps = tuple(int(n) for n in self.cfg.finite_straight_steps)
        radii = []
        for m in self.cfg.finite_turn_radius_steps:
            R = float(int(m)) * self.spacing
            if R >= self.cfg.rmin_um - self.cfg.radius_tol_um:
                radii.append((int(m), R))
        self.turn_radii = tuple(radii)
        if not self.straight_steps:
            raise ValueError("finite_straight_steps must not be empty")
        if not self.turn_radii:
            raise ValueError(
                "no finite turn radius step m*h reaches Rmin = %g um (h = %g um); "
                "increase finite_turn_radius_steps"
                % (self.cfg.rmin_um, self.spacing))

        self.anchor = tuple(float(c) for c in scene.source.position)
        inner = scene.chip.shrink(self.cfg.effective_boundary_margin_um)
        self.inner = inner
        self.index_ranges = []
        for axis in range(3):
            lo = math.ceil((inner.lo[axis] - self.anchor[axis]) / self.spacing - 1e-9)
            hi = math.floor((inner.hi[axis] - self.anchor[axis]) / self.spacing + 1e-9)
            self.index_ranges.append((int(lo), int(hi)))
        self.n_positions = 1
        for lo, hi in self.index_ranges:
            self.n_positions *= max(0, hi - lo + 1)
        self.n_nodes_upper = self.n_positions * len(AXIS_DIRECTIONS)

        # ---- counters.  The *_call_* set is reset by begin_call() for every
        #      route call so that the evidence can separate THIS call's work from
        #      the graph object's lifetime totals (used with a reused graph).
        self._succ_cache: Dict[tuple, Tuple[List[Candidate], List[Candidate]]] = {}
        self.cache_hits = 0
        self.search_count = 0
        self.edges_validated = 0
        self.edges_rejected = 0
        self.edges_undecided = 0
        self.edges_end_pose_rejected = 0
        self.edges_junction_rejected = 0
        self.max_curve_end_position_error_um = 0.0
        self.max_curve_end_tangent_error_deg = 0.0
        self.edge_check_s = 0.0
        self.edge_check_s_graph_build = 0.0
        self.edge_check_s_search = 0.0
        self.begin_call()

        # ---- the node table is a BIJECTION between node ids and canonical poses.
        #      A pose key that would be assigned twice is never silently
        #      overwritten: the first node keeps it and the collision is recorded.
        self._constructing = True
        self.pose_key_to_node: Dict[tuple, tuple] = {}
        self.pose_key_collisions: Dict[tuple, List[tuple]] = {}
        self.n_canonical_pose_collisions = 0
        for n in self.all_canonical_nodes():
            self._register_pose_key(self.node_pose_key(n), n)

        # ---- port attachments (virtual source / target nodes)
        #
        # If a canonical node already HAS the port pose (which is guaranteed for
        # the source, because the lattice is anchored there), that node IS the
        # port: no virtual node is created, so the node-id <-> pose mapping stays
        # a bijection and the search cannot collapse two different ids into one.
        # Otherwise the virtual node is registered explicitly in
        # virtual_pose_keys, so a caller never has to guess which id owns the key.
        self.source_identity_node = self.pose_key_to_node.get(
            self.node_pose_key(SOURCE_NODE))
        self.target_identity_node = self.pose_key_to_node.get(
            self.node_pose_key(TARGET_NODE))
        self.virtual_pose_keys: Dict[tuple, tuple] = {}
        if self.source_identity_node is None:
            self._register_pose_key(self.node_pose_key(SOURCE_NODE), SOURCE_NODE,
                                    virtual=True)
            self.virtual_pose_keys[self.node_pose_key(SOURCE_NODE)] = SOURCE_NODE
        if self.target_identity_node is None:
            self._register_pose_key(self.node_pose_key(TARGET_NODE), TARGET_NODE,
                                    virtual=True)
            self.virtual_pose_keys[self.node_pose_key(TARGET_NODE)] = TARGET_NODE

        # ---- BOUNDS, derived from the construction (PART C item 1).  The node
        #      bound counts CANONICAL nodes only; the virtual port nodes are
        #      reported separately (the v0.1.2 +4096 edge slack is gone).
        edges_per_node = (len(self.straight_steps) + 4 * len(self.turn_radii))
        n_virtual_ports = ((1 if self.source_identity_node is None else 0)
                           + (1 if self.target_identity_node is None else 0))
        # computed after the port attachments are known; see below
        self._edges_per_canonical_node_upper = edges_per_node
        self._n_virtual_port_nodes = n_virtual_ports

        self.snapshot = scene_contract_snapshot(scene, mode, self.cfg, self.field)
        self.scene_fingerprint = scene_geometry_fingerprint(scene)
        self.fingerprint = graph_config_fingerprint(self.cfg, scene, mode=mode,
                                                    congestion=self.field)

        self.source_edges: List[Edge] = []
        self.target_edges: List[Edge] = []
        self.port_report: Dict = {"source": None, "target": None}
        self._build_port_attachments()

        # ---- the edge upper bound must be a genuine bound on the REAL
        #      construction: every canonical node can generate at most
        #      edges_per_node edges, and the port attachment edges are counted
        #      one by one (each is the successor of exactly one node).
        n_edges_canonical = self.n_nodes_upper * edges_per_node
        n_edges_ports = len(self.source_edges) + len(self.target_edges)
        self.n_edges_upper = n_edges_canonical + n_edges_ports
        self.bounds = {
            "spacing_um": self.spacing,
            "anchor_um": list(self.anchor),
            "index_ranges": [[lo, hi] for lo, hi in self.index_ranges],
            "n_lattice_positions": self.n_positions,
            "n_directions": len(AXIS_DIRECTIONS),
            "node_upper_bound": self.n_nodes_upper,
            "node_upper_bound_includes_virtual_ports": False,
            "n_virtual_port_nodes_upper": n_virtual_ports,
            "virtual_port_nodes": {
                "source": self.source_identity_node is None,
                "target": self.target_identity_node is None},
            "total_node_upper_bound_including_virtual_ports":
                self.n_nodes_upper + n_virtual_ports,
            "edge_upper_bound": self.n_edges_upper,
            "edges_per_canonical_node_upper": edges_per_node,
            "edge_upper_bound_from_canonical_nodes": n_edges_canonical,
            "edge_upper_bound_from_port_attachments": n_edges_ports,
            "edge_upper_bound_derivation": (
                "node_upper_bound * (len(finite_straight_steps) + 4 * "
                "len(finite_turn_radius_steps)) + the number of validated port "
                "attachment edges really built (source + target); no hardcoded "
                "constant.  A canonical node generates at most one straight per "
                "straight step and at most one turn per (perpendicular axis "
                "direction, turn radius step) pair, and every port attachment "
                "edge is the successor of exactly one node."),
            "straight_steps": list(self.straight_steps),
            "turn_radius_steps": [m for m, _R in self.turn_radii],
            "turn_radii_um": [R for _m, R in self.turn_radii],
            "max_hops": None,
            "bounds_are_computed_from_the_construction": True,
        }

        self._constructing = False
        self.build_s = time.perf_counter() - t_build0

    # --------------------------------------------------------- call accounting
    def _register_pose_key(self, key: tuple, node: tuple,
                           virtual: bool = False) -> bool:
        """Insert one (pose key -> node) entry WITHOUT silently overwriting.

        Returns True when the key was free.  On a collision the FIRST entry wins
        and the collision is recorded in ``self.pose_key_collisions`` together
        with both nodes; a collision between two canonical nodes additionally
        increments ``self.n_canonical_pose_collisions``, which is what the router
        refuses to search with (two different node ids must never share one pose).
        """
        existing = self.pose_key_to_node.get(key)
        if existing is not None:
            if existing != node:
                self.pose_key_collisions.setdefault(key, [existing]).append(node)
                if not virtual and existing not in (SOURCE_NODE, TARGET_NODE):
                    self.n_canonical_pose_collisions += 1
            return False
        self.pose_key_to_node[key] = node
        return True

    def begin_call(self) -> None:
        """Start a new route call on this graph object: reset the per-call counters.

        The graph object keeps lifetime totals (used to report a WARM reuse) and
        these per-call counters, so an evidence reader can tell what THIS call
        validated from what the object did over its whole life.
        """
        self.call_edge_check_s = 0.0
        self.call_edge_check_s_graph_build = 0.0
        self.call_edge_check_s_search = 0.0
        self.call_edges_validated = 0
        self.call_edges_rejected = 0
        self.call_edges_undecided = 0
        self.call_edges_end_pose_rejected = 0
        self.call_edges_junction_rejected = 0
        self.call_max_curve_end_position_error_um = 0.0
        self.call_max_curve_end_tangent_error_deg = 0.0
        self.call_cache_hits_at_start = self.cache_hits

    # ------------------------------------------------------------- geometry
    def position(self, i: int, j: int, k: int) -> Vec3:
        return (self.anchor[0] + self.spacing * i,
                self.anchor[1] + self.spacing * j,
                self.anchor[2] + self.spacing * k)

    def canonical_pose(self, node: tuple) -> Pose3D:
        if node == SOURCE_NODE:
            return self.scene.source
        if node == TARGET_NODE:
            return self.scene.target
        i, j, k, d = node
        return Pose3D(self.position(i, j, k), direction_unit(AXIS_DIRECTIONS[d]))

    def node_pose_key(self, node: tuple) -> tuple:
        return State3D.from_pose(self.canonical_pose(node)).pose_key()

    def _node_of_position(self, p: Vec3, d: int) -> Optional[tuple]:
        idx = []
        for axis in range(3):
            q = (p[axis] - self.anchor[axis]) / self.spacing
            r = round(q)
            if abs(q - r) > 1e-9:
                return None
            lo, hi = self.index_ranges[axis]
            if not (lo <= r <= hi):
                return None
            idx.append(int(r))
        return (idx[0], idx[1], idx[2], d)

    # ------------------------------------------------------- edge validation
    def _accept(self, segments, from_node, to_node, name, kind, radius_um=math.inf,
                collect: bool = False):
        """Validate one candidate edge; return (Edge or None, rejection reason).

        v0.1.3 hardens the acceptance rule (A2): the candidate's OWN curve must
        really END on the canonical pose of its target node within
        ``pos_tol_um`` / ``ang_tol_deg``, and every internal junction of a
        multi-segment action (a port connector) must be continuous inside the
        same tolerances.  v0.1.2 only RECORDED those errors and still let the edge
        into OPEN, so an edge whose curve missed its node could still be part of
        a reported success.

        Everything else is unchanged: the full continuous curve is validated by
        the production constraint stack (min bend radius, chip boundary, obstacle
        keep-out, reference-centreline spacing) and only a fully safe edge is
        accepted.  The time spent in that stack is attributed to the phase that
        really performs it (graph construction vs search) - see the module
        docstring.
        """
        if not self.allow_out_of_plane:
            for seg in segments:
                if not primitive_is_in_plane(Action(name, tuple(segments)), self.z0, 1e-9):
                    return None, "out_of_plane_forbidden_in_2d_mode"
        segments = tuple(segments)
        if segments:
            want = self.canonical_pose(to_node)
            perr, aerr, jperr, jaerr = _curve_contract_errors(segments, want)
            if jperr > self.cfg.pos_tol_um or jaerr > self.cfg.ang_tol_deg:
                self.edges_rejected += 1
                self.edges_junction_rejected += 1
                self.call_edges_rejected += 1
                self.call_edges_junction_rejected += 1
                return None, "curve_junction_exceeds_continuity_tolerance"
            if perr > self.cfg.pos_tol_um or aerr > self.cfg.ang_tol_deg:
                self.edges_rejected += 1
                self.edges_end_pose_rejected += 1
                self.call_edges_rejected += 1
                self.call_edges_end_pose_rejected += 1
                return None, "curve_end_misses_the_canonical_pose"
        t_check = time.perf_counter()
        val = validate_segments(segments, self.scene.chip, self.scene.obstacles,
                                self.scene.reference_centerlines, self.cfg,
                                detailed=collect)
        dt = time.perf_counter() - t_check
        self.edge_check_s += dt
        self.call_edge_check_s += dt
        if self._constructing:
            self.edge_check_s_graph_build += dt
            self.call_edge_check_s_graph_build += dt
        else:
            self.edge_check_s_search += dt
            self.call_edge_check_s_search += dt
        self.edges_validated += 1
        self.call_edges_validated += 1
        if not val["ok"]:
            self.edges_rejected += 1
            self.call_edges_rejected += 1
            if any(c["status"] == "undecided" for c in val.get("checks", {}).values()):
                self.edges_undecided += 1
                self.call_edges_undecided += 1
            return None, ",".join(val.get("failed", ["unknown"]))
        total = sum(s.length for s in segments)
        cost = total + self.field.penalty_of_segments(segments)
        act = Action(name=name, segments=segments, kind=kind, radius_um=radius_um)
        end = act.end_pose
        want = self.canonical_pose(to_node)
        perr = norm(sub(end.position, want.position))
        aerr = _angle_deg(end.tangent, want.tangent)
        # recorded for ACCEPTED edges only: it is the evidence that an accepted
        # edge really ends on its canonical pose (the rejected ones are counted
        # separately in edges_end_pose_rejected / edges_junction_rejected).
        self.max_curve_end_position_error_um = max(self.max_curve_end_position_error_um, perr)
        self.max_curve_end_tangent_error_deg = max(self.max_curve_end_tangent_error_deg, aerr)
        self.call_max_curve_end_position_error_um = max(
            self.call_max_curve_end_position_error_um, perr)
        self.call_max_curve_end_tangent_error_deg = max(
            self.call_max_curve_end_tangent_error_deg, aerr)
        return Edge(from_node, to_node, act, cost, perr, aerr,
                    val if collect else None), ""

    def canonical_successors(self, node: tuple, collect: bool = False):
        """The deterministic successor set of one canonical node."""
        if node == SOURCE_NODE:
            return list(self.source_edges), []
        if node == TARGET_NODE:
            return [], []                      # the target is a sink
        i, j, k, d = node
        pos = self.position(i, j, k)
        dvec = direction_unit(AXIS_DIRECTIONS[d])
        pose = Pose3D(pos, dvec)
        accepted: List[Edge] = []
        rejected: List[dict] = []

        for n_steps in self.straight_steps:
            end = add(pos, scale(dvec, n_steps * self.spacing))
            tgt_node = self._node_of_position(end, d)
            if tgt_node is None:
                rejected.append({"action": "straight_%d" % n_steps,
                                 "reason": "outside_the_bounded_lattice"})
                continue
            seg = LineSegment3D(pos, end)
            edge, why = self._accept((seg,), node, tgt_node,
                                     "straight_%d" % n_steps, "finite_straight",
                                     collect=collect)
            if edge is None:
                rejected.append({"action": "straight_%d" % n_steps, "reason": why})
            else:
                accepted.append(edge)

        for d2 in perpendicular_axis_directions(d):
            d2vec = direction_unit(AXIS_DIRECTIONS[d2])
            normal = cross(dvec, d2vec)         # +90 deg rotation from d to d2
            for m, R in self.turn_radii:
                end = add(pos, scale(add(dvec, d2vec), R))
                tgt_node = self._node_of_position(end, d2)
                name = "turn_%s_to_%s_R%g" % (
                    "".join("%+d" % c for c in AXIS_DIRECTIONS[d]),
                    "".join("%+d" % c for c in AXIS_DIRECTIONS[d2]), R)
                if tgt_node is None:
                    rejected.append({"action": name,
                                     "reason": "outside_the_bounded_lattice"})
                    continue
                arc = CircularArc3D.from_pose(pose, normal, R, HALF_PI)
                edge, why = self._accept((arc,), node, tgt_node, name, "finite_turn",
                                         radius_um=R, collect=collect)
                if edge is None:
                    rejected.append({"action": name, "reason": why})
                else:
                    accepted.append(edge)

        # a canonical node also owns its TARGET attachment edges: the attachment is
        # an ordinary successor of the node it starts from, checked like any other
        # edge, so it is part of this node's deterministic successor set.
        for e in self.target_edges:
            if e.from_node == node:
                accepted.append(e)
        return accepted, rejected

    # ------------------------------------------------------------ successors
    def successors(self, node: tuple, collect: bool = False):
        """The successor set of one node, with the per-search successor cache.

        The set is a pure function of the node id, so the FIRST computation for a
        node is cached and every later request for that node is a real cache hit
        (``self.cache_hits``).  The v0.1.2 cache skipped the initial
        ``collect=True`` request, so a short search (one or two expansions, e.g.
        the counterexample scenes) cached nothing and a WARM reuse of the same
        graph object could never show a hit; the cached set is now the one the
        first request produced, so a reused graph really reports the hits it
        served.  A cached entry computed with ``collect=False`` carries no
        ``validation`` payload; that can only affect the DETAIL of a later
        ``collect=True`` evidence dict, never a search decision (every decision is
        made by ``validate_segments`` when the edge is built).
        """
        cached = self._succ_cache.get(node)
        if cached is not None:
            self.cache_hits += 1
            return cached
        out = self.canonical_successors(node, collect=collect)
        self._succ_cache[node] = out
        return out

    # ---------------------------------------------------- port attachments
    def _attachment_candidates(self, port: Pose3D) -> List[tuple]:
        """Canonical nodes inside the generic attachment neighbourhood of a port.

        The neighbourhood is the Euclidean ball of radius finite_port_attach_um
        around THAT port, capped to the finite_port_attach_max_nodes nearest
        canonical nodes.  It is a generic configuration rule: no case-specific
        waypoint, height or route is ever inserted here.
        """
        r = float(self.cfg.finite_port_attach_um)
        cands = []
        for i, j, k in itertools.product(*[range(lo, hi + 1)
                                           for lo, hi in self.index_ranges]):
            p = self.position(i, j, k)
            if norm(sub(p, port.position)) > r:
                continue
            for d in range(len(AXIS_DIRECTIONS)):
                cands.append((i, j, k, d))
        cands.sort(key=lambda nd: norm(sub(self.canonical_pose(nd).position,
                                           port.position)))
        return cands[:int(self.cfg.finite_port_attach_max_nodes)]

    def _build_port_attachments(self) -> None:
        if self.source_identity_node is not None:
            self.port_report["source"] = {
                "kind": "identity",
                "canonical_node": list(self.source_identity_node),
                "n_edges": 0,
                "note": ("the source pose IS the canonical pose of this node (the "
                         "lattice is anchored at the source), so the port is the node "
                         "itself and no attachment curve is needed")}
        else:
            self.source_edges, src_report = self._attach_port(
                self.scene.source, self._attachment_candidates(self.scene.source),
                SOURCE_NODE, "source")
            self.port_report["source"] = src_report
        if self.target_identity_node is not None:
            self.port_report["target"] = {
                "kind": "identity",
                "canonical_node": list(self.target_identity_node),
                "n_edges": 0,
                "note": "the target pose IS the canonical pose of this node"}
        else:
            self.target_edges, tgt_report = self._attach_port(
                self.scene.target, self._attachment_candidates(self.scene.target),
                TARGET_NODE, "target")
            self.port_report["target"] = tgt_report

    def _attach_port(self, port: Pose3D, cands: Sequence[tuple], virtual_node: tuple,
                     which: str):
        """Validated attachment edges between a port pose and canonical nodes.

        which == 'source': edge virtual_node -> canonical node, the curve starts
        at the port pose.  which == 'target': edge canonical node -> virtual_node,
        the curve ends at the port pose.
        """
        edges: List[Edge] = []
        tries = 0
        axis_match = None
        for idx, d in enumerate(AXIS_DIRECTIONS):
            if norm(sub(direction_unit(d), port.tangent)) <= 1e-12:
                axis_match = idx
        want = port.tangent if which == "source" else scale(port.tangent, -1.0)
        for node in cands:
            i, j, k, d = node
            pose = self.canonical_pose(node)
            if axis_match is not None and d != axis_match:
                continue
            delta = sub(pose.position, port.position)
            L = norm(delta)
            same_dir = dot(pose.tangent, port.tangent) > 1.0 - 1e-12
            if L == 0.0:
                # the canonical node IS the port pose: a zero-length attachment
                if not same_dir:
                    continue
                act = Action(name="port_%s_zero_length" % which, segments=(),
                             kind="port_attachment", virtual_start=port,
                             virtual_end=port)
                edges.append(Edge(virtual_node if which == "source" else node,
                                  node if which == "source" else virtual_node,
                                  act, 0.0, 0.0, 0.0))
                continue
            if L > float(self.cfg.finite_port_attach_um) + 1e-9:
                continue
            if not same_dir or norm(sub(scale(delta, 1.0 / L), want)) > 1e-9:
                continue
            seg = (LineSegment3D(port.position, pose.position) if which == "source"
                   else LineSegment3D(pose.position, port.position))
            frm = virtual_node if which == "source" else node
            to = node if which == "source" else virtual_node
            tries += 1
            edge, why = self._accept((seg,), frm, to,
                                     "port_%s_straight" % which, "port_attachment",
                                     collect=False)
            if edge is not None:
                edges.append(edge)
        if edges:
            edges.sort(key=lambda e: e.cost)
            return edges, {"kind": "straight" if axis_match is not None else "mixed",
                           "n_candidate_nodes": len(cands), "n_edges": len(edges),
                           "n_validated_tries": tries,
                           "attachment_radius_um": self.cfg.finite_port_attach_um,
                           "note": ("validated local attachment curves inside the "
                                    "generic attachment neighbourhood; no case-specific "
                                    "waypoint, height or route is supplied")}
        # ---- fall back to the v0.1.1 planar Dubins connector family
        radii = sorted({float(r) for r in self.cfg.connector_radii_um} | {self.cfg.rmin_um})
        for node in cands:
            pose = self.canonical_pose(node)
            if norm(sub(pose.position, port.position)) > \
                    float(self.cfg.finite_port_attach_um) + 1e-9:
                continue
            a, b = (port, pose) if which == "source" else (pose, port)
            for R in radii:
                for segs in planar_connectors(a, b, R):
                    frm = virtual_node if which == "source" else node
                    to = node if which == "source" else virtual_node
                    tries += 1
                    edge, why = self._accept(tuple(segs), frm, to,
                                             "port_%s_connector_R%g" % (which, R),
                                             "port_attachment", radius_um=R,
                                             collect=False)
                    if edge is not None:
                        edges.append(edge)
            if len(edges) >= 8:
                break
        if edges:
            edges.sort(key=lambda e: e.cost)
            return edges, {"kind": "planar_dubins_connector",
                           "n_candidate_nodes": len(cands), "n_edges": len(edges),
                           "n_validated_tries": tries,
                           "attachment_radius_um": self.cfg.finite_port_attach_um,
                           "note": ("coplanar pose pairs only (no LRL/RLR family); a "
                                    "missing connector means the supported action set "
                                    "found none, never physical unreachability")}
        return [], {"kind": "none", "n_candidate_nodes": len(cands), "n_edges": 0,
                    "n_validated_tries": tries,
                    "attachment_radius_um": self.cfg.finite_port_attach_um}

    # ---------------------------------------------------------- enumeration
    def all_canonical_nodes(self) -> List[tuple]:
        out = []
        for i, j, k in itertools.product(*[range(lo, hi + 1)
                                           for lo, hi in self.index_ranges]):
            for d in range(len(AXIS_DIRECTIONS)):
                out.append((i, j, k, d))
        return out


def _state_of(node: tuple, graph: FiniteGraph) -> State3D:
    return State3D.from_pose(graph.canonical_pose(node))


def _graph_evidence(graph: FiniteGraph, mode: str, reuse: dict, graph_branch: str,
                    *, assembled=None, endpoint_check=None, port_assurance=None,
                    n_segments: int = 0, zero_length: bool = False,
                    start_node=None, goal_node=None, seen_expansions=None) -> dict:
    """The ``res.finite_graph`` evidence block of ONE route call.

    Per-call numbers and the graph object's lifetime totals are reported
    separately (``graph_object_totals``), because a WARM reuse inherits the
    object's history: a reader must be able to tell what THIS call validated from
    what the object did before.  The cold default branch has
    ``per-call == lifetime totals``.
    """
    return {
        "strategy": ("route_finite_native3d (v0.1.3 production finite multi-hop "
                     "router; three-dimensional geometric contract)"),
        "mode": mode,
        "bounds": graph.bounds,
        "port_attachment": graph.port_report,
        "cache_key_fingerprint": graph.fingerprint,
        "edge_cache_scope": ("one search; keyed by node id + the geometry/constraint/"
                             "cost fingerprint above, never shared across scenes"),
        "graph_reuse": reuse,
        "graph_branch": graph_branch,
        "cache_hits": graph.cache_hits,
        "cache_hits_graph_object_total": graph.cache_hits,
        "cache_hits_this_call": (graph.cache_hits - graph.call_cache_hits_at_start),
        "edge_check_s": graph.call_edge_check_s,
        "edge_check_s_graph_build": graph.call_edge_check_s_graph_build,
        "edge_check_s_search": graph.call_edge_check_s_search,
        "edge_check_timing_note": (
            "the per-edge continuous constraint checks are attributed to the phase "
            "that really performs them: port attachments are validated while the "
            "graph is CONSTRUCTED (edge_check_s_graph_build, inside graph_build_s) "
            "and canonical successors lazily during the search "
            "(edge_check_s_search, inside search_s).  edge_check_s is their sum for "
            "THIS call; graph_object_totals holds the graph object's lifetime "
            "totals, which a warm reuse inherits"),
        "edges_validated": graph.call_edges_validated,
        "edges_rejected": graph.call_edges_rejected,
        "edges_rejected_undecided": graph.call_edges_undecided,
        "edges_end_pose_rejected": graph.call_edges_end_pose_rejected,
        "edges_junction_rejected": graph.call_edges_junction_rejected,
        "max_curve_end_position_error_um": graph.call_max_curve_end_position_error_um,
        "max_curve_end_tangent_error_deg": graph.call_max_curve_end_tangent_error_deg,
        "graph_object_totals": {
            "edge_check_s": graph.edge_check_s,
            "edge_check_s_graph_build": graph.edge_check_s_graph_build,
            "edge_check_s_search": graph.edge_check_s_search,
            "edges_validated": graph.edges_validated,
            "edges_rejected": graph.edges_rejected,
            "edges_end_pose_rejected": graph.edges_end_pose_rejected,
            "edges_junction_rejected": graph.edges_junction_rejected,
            "cache_hits": graph.cache_hits,
            "construction_s": graph.build_s,
            "search_count": graph.search_count,
            "note": ("lifetime totals of the FiniteGraph OBJECT; for the cold default "
                     "branch they equal this call's numbers, for a warm reuse they "
                     "include the earlier call(s)"),
        },
        "start_node": (list(start_node) if start_node is not None else None),
        "goal_node": (list(goal_node) if goal_node is not None else None),
        "port_nodes_are_the_ports_themselves": {
            "source": graph.source_identity_node is not None,
            "target": graph.target_identity_node is not None},
        "nodes_expanded": (len(seen_expansions) if seen_expansions else 0),
        "node_expansion_counts_sum": (sum(seen_expansions.values())
                                      if seen_expansions else 0),
        "n_segments": n_segments,
        "zero_length_solution": zero_length,
        "assembled_path_constraint_check": assembled,
        "assembled_endpoint_check": endpoint_check,
        "port_assurance": port_assurance,
        "builder_guarantee": ("the graph is only reused when its construction "
                              "snapshot matches this input field by field; the "
                              "edge upper bound is derived from the real action and "
                              "port-attachment construction"),
        "guarantee": ("optimality is claimed for THIS constructed finite graph only, "
                      "up to the documented cost comparison tolerance of astar.py; "
                      "search_exhausted means this graph was exhausted, not that the "
                      "continuous pose space is unreachable.  success additionally "
                      "means the ACTUAL assembled curve was verified against the pose "
                      "the caller passed (or is the explicit zero-length solution)"),
    }


def route_finite_native3d(scene, mode: str = "native3d",
                          congestion: Optional[CongestionField] = None,
                          max_expansions: Optional[int] = None,
                          max_runtime_s: Optional[float] = None,
                          expand_hook=None,
                          graph: Optional[FiniteGraph] = None) -> SearchResult:
    """v0.1.3 PRODUCTION entry point: finite, multi-hop, native 3D routing.

    Flow, in order (each step is a distinct status, see search/statuses.py):

    1. build the graph, or CHECK a caller-supplied one against its construction
       snapshot - a mismatch returns ``graph_scene_mismatch`` BEFORE any search;
    2. reject an illegal source/target pose (``invalid_source_pose`` /
       ``invalid_target_pose``), including when source == target;
    3. reject a port that no supported action can attach to
       (``port_attachment_failed``);
    4. run the finite A* (``success`` / ``search_exhausted`` / ``budget_exhausted``);
    5. verify the ACTUAL assembled curve against the pose the caller passed; a
       failure returns ``constraint_failed`` (or ``uncertified`` when a check
       could not be decided) and leaves ``path=None`` with no port assurance;
    6. publish the timings per phase (graph build, search, assembled-path check)
       and the cold/warm provenance of the graph that was used.

    Timing (recorded under "timings"): graph_build_s, search_s,
    assembled_path_check_s and end_to_end_s.  For the cold default branch
    ``end_to_end_s >= graph_build_s + search_s``; a caller-supplied graph reports
    its own ``graph_prebuilt_construction_s`` (inside
    ``res.finite_graph["graph_reuse"]``) and is never mixed into this call's
    end-to-end budget.
    """
    cfg = scene.config
    if mode not in ("2d", "native3d"):
        raise ValueError("mode must be '2d' or 'native3d'")
    max_exp = int(cfg.finite_max_expansions if max_expansions is None else max_expansions)
    max_rt = float(cfg.finite_max_runtime_s if max_runtime_s is None else max_runtime_s)
    field = congestion if congestion is not None else ZeroCongestion()
    t_start = time.perf_counter()

    src_bad = pose_invalid_reason(scene, scene.source)
    tgt_bad = pose_invalid_reason(scene, scene.target)

    # ------------------------------------------------ 1. graph + reuse contract
    caller_supplied = graph is not None
    branch = "prebuilt_graph" if caller_supplied else "fresh_cold_graph"
    t0 = time.perf_counter()
    if graph is None:
        graph = FiniteGraph(scene, mode=mode, cfg=cfg, congestion=field)
    t_graph = time.perf_counter() - t0
    reuse = graph_reuse_report(graph, scene, mode, cfg, field,
                               caller_supplied=caller_supplied)
    if not reuse["accepted"]:
        return _fail("graph_scene_mismatch", reuse["reason"], mode, graph, t_start,
                     t_graph, reuse=reuse, graph_branch="prebuilt_graph_rejected")
    branch = "prebuilt_graph_reused" if caller_supplied else "fresh_cold_graph"
    if caller_supplied:
        # a reused graph starts a NEW call: reset the per-call counters, keep the
        # object's lifetime totals (they are what makes the reuse "warm")
        graph.begin_call()
    if graph.n_canonical_pose_collisions:
        raise AssertionError(
            "two canonical graph nodes share one pose key (%d collision(s)): the "
            "node-id <-> pose mapping must stay a bijection, otherwise "
            "de-duplication by node id would merge different nodes"
            % graph.n_canonical_pose_collisions)

    # ------------------------------------------------- 2. input pose contract
    if src_bad is not None or tgt_bad is not None:
        status = "invalid_source_pose" if src_bad is not None else "invalid_target_pose"
        return _fail(status, src_bad or tgt_bad, mode, graph, t_start, t_graph,
                     reuse=reuse, graph_branch=branch)

    # ------------------------------------------------------- 3. port contract
    if graph.source_identity_node is None and not graph.source_edges:
        return _fail("port_attachment_failed",
                     "no validated attachment curve connects the SOURCE pose "
                     "(position %r, tangent %r) to any canonical node inside the "
                     "attachment neighbourhood (radius %g um).  This is a statement "
                     "about the supported action set (six axis directions, quarter "
                     "turns of radius m*h, and the coplanar planar connector family), "
                     "never about physical reachability."
                     % (list(scene.source.position), list(scene.source.tangent),
                        cfg.finite_port_attach_um),
                     mode, graph, t_start, t_graph, reuse=reuse, graph_branch=branch)
    if graph.target_identity_node is None and not graph.target_edges:
        return _fail("port_attachment_failed",
                     "no validated attachment curve connects the TARGET pose "
                     "(position %r, tangent %r) to any canonical node inside the "
                     "attachment neighbourhood (radius %g um).  Like the source "
                     "case, this is a statement about the supported action set and "
                     "is never about physical reachability."
                     % (list(scene.target.position), list(scene.target.tangent),
                        cfg.finite_port_attach_um),
                     mode, graph, t_start, t_graph, reuse=reuse, graph_branch=branch)

    node_of_key: Dict[tuple, tuple] = dict(graph.pose_key_to_node)
    for key, node in graph.virtual_pose_keys.items():
        node_of_key.setdefault(key, node)
    start_node = (graph.source_identity_node if graph.source_identity_node is not None
                  else SOURCE_NODE)
    goal_node = (graph.target_identity_node if graph.target_identity_node is not None
                 else TARGET_NODE)
    target_key = graph.node_pose_key(goal_node)
    seen_expansions: Dict[tuple, int] = {}

    def _node_of_state(state: State3D) -> tuple:
        key = state.pose_key()
        node = node_of_key.get(key)
        if node is None:
            raise AssertionError(
                "a search state is not a canonical node of this finite graph: %r.  "
                "Every successor is built from the canonical pose of its node id, so "
                "this can only be a construction bug." % (state,))
        return node

    def generate(state: State3D, collect: bool):
        node = _node_of_state(state)
        seen_expansions[node] = seen_expansions.get(node, 0) + 1
        accepted, rejected = graph.successors(node, collect=collect)
        acc, rej = [], []
        for e in accepted:
            acc.append(Candidate(e.action, _state_of(e.to_node, graph), e.cost, True, "",
                                 e.validation))
        for r in rejected:
            rej.append(Candidate(Action(name=str(r.get("action", "edge")), segments=(),
                                        kind="rejected", virtual_start=state.pose,
                                        virtual_end=state.pose),
                                 state, math.inf, False,
                                 str(r.get("reason", "rejected")), None))
        return acc, rej

    def goal_test(state: State3D):
        pos_err = norm(sub(state.position, scene.target.position))
        ang_err = math.degrees(math.acos(max(-1.0, min(
            1.0, dot(state.direction, scene.target.tangent)))))
        return (state.pose_key() == target_key, pos_err, ang_err)

    def h_fn(state: State3D) -> float:
        return max(0.0, norm(sub(state.position, scene.target.position)) - cfg.eps_pos_um)

    start = _state_of(start_node, graph)
    graph.search_count += 1
    t1 = time.perf_counter()
    res = astar_search(
        start, generate, goal_test, h_fn,
        lambda s: _node_of_state(s), cfg, mode="finite_native3d_%s" % mode,
        heuristic_name="max(0, |target_pos - pos| - eps_pos) [3D Euclidean lower bound]",
        source_invalid_reason=None, target_invalid_reason=None,
        max_expansions=max_exp, max_runtime_s=max_rt,
        expand_hook=expand_hook,
        state_identity="finite_lattice_node_id",
        approximate_state_identity=False)
    t_search = time.perf_counter() - t1

    # ---- 5. the ACTUAL assembled curve decides success ----------------------
    assembled = None
    endpoint_check = None
    port_assurance = None
    t_ass0 = time.perf_counter()
    if res.status == "success":
        if res.zero_length_solution:
            # No primitive exists: the caller's source pose IS the whole curve.
            segments = ()
        else:
            # res.path is the raw concatenation of the searched primitives (the
            # zero-length port attachment contributes nothing): re-check the whole
            # assembled curve with the production constraint stack.
            segments = tuple(res.path.segments)
        endpoint_check = verify_assembled_endpoints(segments, scene, cfg)
        assembled = validate_segments(segments, scene.chip, scene.obstacles,
                                      scene.reference_centerlines, cfg, detailed=True)
        undecided = any(c["status"] == "undecided"
                        for c in assembled.get("checks", {}).values())
        if not (assembled["ok"] and endpoint_check["ok"]):
            # NEVER search_exhausted here: OPEN was not empty, the search DID
            # return a path and the PATH (or its ports) was rejected.
            res.status = "uncertified" if undecided else "constraint_failed"
            res.reason = (
                "the assembled path failed %s (%s); the search itself returned a "
                "path, so this is a constraint/port failure and NOT search_exhausted"
                % ("a check that could not be decided" if undecided
                   else "the hard constraint or port verification",
                   ",".join(list(assembled.get("failed", ["unknown"]))
                            + list(endpoint_check["failed"]))))
            res.path = None
            res.cost = math.inf
            res.length_um = math.inf
            res.congestion_cost = 0.0
            res.zero_length_solution = False
            res.n_segments = 0
            res.goal_pos_error_um = None
            res.goal_tangent_error_deg = None
            res.endpoint_errors = None
            res.port_assurance = None
            endpoint_check = None
            assembled = None
        else:
            res.n_segments = len(segments)
            res.endpoint_errors = dict(endpoint_check)
            port_assurance = {
                "verified_against_the_caller_pose": True,
                "zero_length_solution": bool(res.zero_length_solution),
                "n_segments": len(segments),
                "source_position_error_um": endpoint_check["start_position_error_um"],
                "source_tangent_error_deg": endpoint_check["start_tangent_error_deg"],
                "target_position_error_um": endpoint_check["end_position_error_um"],
                "target_tangent_error_deg": endpoint_check["end_tangent_error_deg"],
                "max_junction_position_error_um":
                    endpoint_check["max_junction_position_error_um"],
                "max_junction_tangent_error_deg":
                    endpoint_check["max_junction_tangent_error_deg"],
                "assembled_path_check": endpoint_check,
                "hard_constraints_ok": bool(assembled["ok"]),
                "guarantee": (
                    "the returned curve was evaluated at its own first/last "
                    "primitive and compared with the source/target pose the CALLER "
                    "passed, using the source tolerances at the start and the "
                    "target acceptance ball at the end, and the whole curve was "
                    "re-validated by the production constraint stack"),
            }
            res.port_assurance = port_assurance
    t_assembled = time.perf_counter() - t_ass0

    res.state_identity = "finite_lattice_node_id"
    res.approximate_state_identity = False
    reuse["successor_cache_hits"] = int(graph.cache_hits)
    res.finite_graph = _graph_evidence(
        graph, mode, reuse, branch, assembled=assembled,
        endpoint_check=endpoint_check, port_assurance=port_assurance,
        n_segments=res.n_segments, zero_length=bool(res.zero_length_solution),
        start_node=start_node, goal_node=goal_node, seen_expansions=seen_expansions)
    res.timings = {"graph_build_s": t_graph, "search_s": t_search,
                   "assembled_path_check_s": t_assembled,
                   "end_to_end_s": time.perf_counter() - t_start,
                   "graph_branch": branch,
                   "graph_prebuilt_construction_s": (
                       graph.build_s if caller_supplied else 0.0)}
    return res


def _fail(status, reason, mode, graph, t_start, t_graph, reuse: Optional[dict] = None,
          graph_branch: str = "fresh_cold_graph") -> SearchResult:
    """An explicit failure: no path, no cost, no guarantee field that reads as success."""
    res = SearchResult(status, reason, "finite_native3d_%s" % mode,
                       "max(0, |target_pos - pos| - eps_pos)", 0, 0, 0,
                       time.perf_counter() - t_start,
                       max_expansions=graph.cfg.finite_max_expansions,
                       max_runtime_s=graph.cfg.finite_max_runtime_s,
                       state_identity="finite_lattice_node_id",
                       approximate_state_identity=False)
    res.path = None
    res.cost = math.inf
    res.length_um = math.inf
    res.congestion_cost = 0.0
    res.zero_length_solution = False
    res.n_segments = 0
    res.endpoint_errors = None
    res.port_assurance = None
    res.finite_graph = _graph_evidence(graph, mode, reuse or {}, graph_branch)
    res.timings = {"graph_build_s": t_graph, "search_s": 0.0,
                   "assembled_path_check_s": 0.0,
                   "end_to_end_s": time.perf_counter() - t_start,
                   "graph_branch": graph_branch}
    return res


def enumerate_graph_edges(scene, mode: str = "native3d",
                          congestion: Optional[CongestionField] = None,
                          cfg: Optional[RouterConfig] = None,
                          graph: Optional[FiniteGraph] = None):
    """Full enumeration of the finite graph (for independent cross-checks).

    Returns (graph, edges) where edges is a list of Edge.  Source and target
    attachment edges are included.  This is what an INDEPENDENT shortest-path
    implementation (for example a plain Dijkstra written elsewhere) can run on
    without using any part of astar_search.
    """
    field = congestion if congestion is not None else ZeroCongestion()
    if graph is None:
        graph = FiniteGraph(scene, mode=mode, cfg=cfg, congestion=field)
    edges: List[Edge] = []
    if graph.source_identity_node is None:
        edges.extend(graph.source_edges)
    for node in graph.all_canonical_nodes():
        accepted, _rej = graph.canonical_successors(node, collect=False)
        edges.extend(accepted)
    return graph, edges
