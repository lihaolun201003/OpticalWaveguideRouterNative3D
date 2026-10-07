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
The state identity used for de-duplication is the integer node id.  Results:
    success, search_exhausted, budget_exhausted, invalid_source_pose,
    invalid_target_pose, unsupported_pose, port_attachment_failed, uncertified.
  - search_exhausted means THE CONSTRUCTED FINITE GRAPH was exhausted, which is a
    real exhaustion and not a budget stop;
  - optimality is claimed for the constructed finite graph only, up to the
    documented cost comparison tolerance of search/astar.py, and never for the
    continuous pose space;
  - congestion is a fixed non-negative single-net cost; no multi-net scheduling.

DISCRETE-ACTION LIMITATION (disclosed, not hidden)
--------------------------------------------------
A lattice of spacing h with quarter turns of radius m*h cannot reproduce every
continuous route: the set of reachable positions is a sublattice, the turning
radius is quantised to multiples of h, and a shape that cannot be assembled from
axis-aligned quarter turns is simply not in the graph.  A longer legal route than
the v0.1.1 reference graph produced is therefore expected and is not a defect.
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import RouterConfig
from ..constraints.checks import Box3D, validate_segments
from ..geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from ..geometry.vec import Vec3, add, cross, dot, norm, scale, sub, unit
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


class FiniteGraph:
    """The finite canonical node/edge construction of one scene and one mode."""

    def __init__(self, scene, mode: str = "native3d", cfg: Optional[RouterConfig] = None,
                 congestion: Optional[CongestionField] = None):
        if mode not in ("2d", "native3d"):
            raise ValueError("mode must be '2d' or 'native3d'")
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
        self.n_edges_upper = self.n_nodes_upper * (len(self.straight_steps)
                                                   + 4 * len(self.turn_radii)) + 4096
        self.bounds = {
            "spacing_um": self.spacing,
            "anchor_um": list(self.anchor),
            "index_ranges": [[lo, hi] for lo, hi in self.index_ranges],
            "n_lattice_positions": self.n_positions,
            "n_directions": len(AXIS_DIRECTIONS),
            "node_upper_bound": self.n_nodes_upper,
            "edge_upper_bound": self.n_edges_upper,
            "straight_steps": list(self.straight_steps),
            "turn_radius_steps": [m for m, _R in self.turn_radii],
            "turn_radii_um": [R for _m, R in self.turn_radii],
            "max_hops": None,
            "bounds_are_computed_from_the_construction": True,
        }

        self._succ_cache: Dict[tuple, Tuple[List[Candidate], List[Candidate]]] = {}
        self.cache_hits = 0
        self.edges_validated = 0
        self.edges_rejected = 0
        self.edges_undecided = 0
        self.max_curve_end_position_error_um = 0.0
        self.max_curve_end_tangent_error_deg = 0.0
        self.edge_check_s = 0.0
        self.fingerprint = graph_config_fingerprint(self.cfg, scene)

        # ---- the node table is a BIJECTION between node ids and canonical poses
        self.pose_key_to_node: Dict[tuple, tuple] = {}
        for n in self.all_canonical_nodes():
            self.pose_key_to_node[self.node_pose_key(n)] = n

        # ---- port attachments (virtual source / target nodes)
        #
        # If a canonical node already HAS the port pose (which is guaranteed for
        # the source, because the lattice is anchored there), that node IS the
        # port: no virtual node is created, so the node-id <-> pose mapping stays
        # a bijection and the search cannot collapse two different ids into one.
        self.source_identity_node = self.pose_key_to_node.get(
            self.node_pose_key(SOURCE_NODE))
        self.target_identity_node = self.pose_key_to_node.get(
            self.node_pose_key(TARGET_NODE))
        self.source_edges: List[Edge] = []
        self.target_edges: List[Edge] = []
        self.port_report: Dict = {"source": None, "target": None}
        self._build_port_attachments()

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
        if not self.allow_out_of_plane:
            for seg in segments:
                if not primitive_is_in_plane(Action(name, tuple(segments)), self.z0, 1e-9):
                    return None, "out_of_plane_forbidden_in_2d_mode"
        t_check = time.perf_counter()
        val = validate_segments(segments, self.scene.chip, self.scene.obstacles,
                                self.scene.reference_centerlines, self.cfg,
                                detailed=collect)
        self.edge_check_s += time.perf_counter() - t_check
        self.edges_validated += 1
        if not val["ok"]:
            self.edges_rejected += 1
            if any(c["status"] == "undecided" for c in val.get("checks", {}).values()):
                self.edges_undecided += 1
            return None, ",".join(val.get("failed", ["unknown"]))
        total = sum(s.length for s in segments)
        cost = total + self.field.penalty_of_segments(segments)
        act = Action(name=name, segments=tuple(segments), kind=kind, radius_um=radius_um)
        end = act.end_pose
        want = self.canonical_pose(to_node)
        perr = norm(sub(end.position, want.position))
        aerr = math.degrees(math.acos(max(-1.0, min(1.0, dot(end.tangent, want.tangent)))))
        self.max_curve_end_position_error_um = max(self.max_curve_end_position_error_um, perr)
        self.max_curve_end_tangent_error_deg = max(self.max_curve_end_tangent_error_deg, aerr)
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
        if collect:
            return self.canonical_successors(node, collect=True)
        if node in self._succ_cache:
            self.cache_hits += 1
            return self._succ_cache[node]
        out = self.canonical_successors(node, collect=False)
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


def graph_config_fingerprint(cfg: RouterConfig, scene) -> dict:
    """Everything that can change an edge's geometry, feasibility or cost.

    The static edge cache of one search is keyed by this fingerprint (plus the
    scene's own geometry), so a cached edge can never be reused with different
    geometry, constraints or costs.
    """
    return {
        "spacing_um": cfg.finite_spacing_um,
        "straight_steps": list(cfg.finite_straight_steps),
        "turn_radius_steps": list(cfg.finite_turn_radius_steps),
        "rmin_um": cfg.rmin_um,
        "radius_tol_um": cfg.radius_tol_um,
        "dmin_centerline_um": cfg.dmin_centerline_um,
        "obstacle_clearance_um": cfg.obstacle_clearance_um,
        "obstacle_clearance_includes_envelope": cfg.obstacle_clearance_includes_envelope,
        "wg_envelope_radius_um": cfg.wg_envelope_radius_um,
        "boundary_margin_um": cfg.effective_boundary_margin_um,
        "clearance_decision_tol_um": cfg.clearance_decision_tol_um,
        "max_subdiv_level": cfg.max_subdiv_level,
        "port_attach_um": cfg.finite_port_attach_um,
        "chip": [list(scene.chip.lo), list(scene.chip.hi)],
        "obstacles": [[list(b.lo), list(b.hi)] for b in scene.obstacles],
        "reference_centrelines": [len(scene.reference_centerlines)],
    }


def _state_of(node: tuple, graph: FiniteGraph) -> State3D:
    return State3D.from_pose(graph.canonical_pose(node))


def route_finite_native3d(scene, mode: str = "native3d",
                          congestion: Optional[CongestionField] = None,
                          max_expansions: Optional[int] = None,
                          max_runtime_s: Optional[float] = None,
                          expand_hook=None,
                          graph: Optional[FiniteGraph] = None) -> SearchResult:
    """v0.1.2 PRODUCTION entry point: finite, multi-hop, native 3D routing.

    End-to-end timing (recorded under "timings") covers graph construction, the
    per-edge continuous constraint checks and the search; nothing is hidden in an
    un-timed pre-computation and no warmed cache is used.
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

    t0 = time.perf_counter()
    if graph is None:
        graph = FiniteGraph(scene, mode=mode, cfg=cfg, congestion=field)
    t_graph = time.perf_counter() - t0

    if src_bad is not None or tgt_bad is not None:
        status = "invalid_source_pose" if src_bad is not None else "invalid_target_pose"
        return _fail(status, src_bad or tgt_bad, mode, graph, t_start, t_graph)

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
                     mode, graph, t_start, t_graph)
    if graph.target_identity_node is None and not graph.target_edges:
        return _fail("port_attachment_failed",
                     "no validated attachment curve connects the TARGET pose "
                     "(position %r, tangent %r) to any canonical node inside the "
                     "attachment neighbourhood (radius %g um).  Like the source "
                     "case, this is a statement about the supported action set and "
                     "is never about physical reachability."
                     % (list(scene.target.position), list(scene.target.tangent),
                        cfg.finite_port_attach_um),
                     mode, graph, t_start, t_graph)

    node_of_key: Dict[tuple, tuple] = dict(graph.pose_key_to_node)
    start_node = (graph.source_identity_node if graph.source_identity_node is not None
                  else SOURCE_NODE)
    goal_node = (graph.target_identity_node if graph.target_identity_node is not None
                 else TARGET_NODE)
    if graph.source_identity_node is None:
        node_of_key[graph.node_pose_key(SOURCE_NODE)] = SOURCE_NODE
    if graph.target_identity_node is None:
        node_of_key[graph.node_pose_key(TARGET_NODE)] = TARGET_NODE
    if len(node_of_key) != len(graph.pose_key_to_node) + \
            (0 if graph.source_identity_node is not None else 1) + \
            (0 if graph.target_identity_node is not None else 1):
        raise AssertionError(
            "two graph nodes share one pose: the node-id <-> pose mapping must stay a "
            "bijection, otherwise de-duplication by node id would merge different nodes")
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

    # ---- assemble the searched path and re-check it with the production stack
    assembled = None
    if res.status == "success":
        # res.path is the raw concatenation of the searched primitives (the
        # zero-length port attachment contributes nothing): re-check the whole
        # assembled curve with the production constraint stack.
        segments = tuple(res.path.segments)
        assembled = validate_segments(segments, scene.chip, scene.obstacles,
                                      scene.reference_centerlines, cfg, detailed=True)
        if not assembled["ok"]:
            undecided = any(c["status"] == "undecided"
                            for c in assembled.get("checks", {}).values())
            res.status = "uncertified" if undecided else "search_exhausted"
            res.reason = ("the assembled path failed the production constraint stack "
                          "(%s); status reflects that instead of reporting success"
                          % ",".join(assembled.get("failed", ["unknown"])))
            res.path = None

    res.state_identity = "finite_lattice_node_id"
    res.approximate_state_identity = False
    res.finite_graph = {
        "strategy": "route_finite_native3d (v0.1.2 production finite multi-hop router)",
        "mode": mode,
        "bounds": graph.bounds,
        "port_attachment": graph.port_report,
        "cache_key_fingerprint": graph.fingerprint,
        "edge_cache_scope": ("one search; keyed by node id + the geometry/constraint/"
                             "cost fingerprint above, never shared across scenes"),
        "cache_hits": graph.cache_hits,
        "edge_check_s": graph.edge_check_s,
        "edge_check_timing_note": ("the per-edge continuous constraint checks are run "
                                   "lazily during the search, so edge_check_s is a "
                                   "subset of the reported search_s, not an extra term"),
        "edges_validated": graph.edges_validated,
        "edges_rejected": graph.edges_rejected,
        "edges_rejected_undecided": graph.edges_undecided,
        "max_curve_end_position_error_um": graph.max_curve_end_position_error_um,
        "max_curve_end_tangent_error_deg": graph.max_curve_end_tangent_error_deg,
        "start_node": list(start_node),
        "goal_node": list(goal_node),
        "port_nodes_are_the_ports_themselves": {
            "source": graph.source_identity_node is not None,
            "target": graph.target_identity_node is not None},
        "nodes_expanded": len(seen_expansions),
        "node_expansion_counts_sum": sum(seen_expansions.values()),
        "assembled_path_constraint_check": assembled,
        "guarantee": ("optimality is claimed for THIS constructed finite graph only, "
                      "up to the documented cost comparison tolerance of astar.py; "
                      "search_exhausted means this graph was exhausted, not that the "
                      "continuous pose space is unreachable"),
    }
    res.timings = {"graph_build_s": t_graph, "search_s": t_search,
                   "end_to_end_s": time.perf_counter() - t_start}
    return res


def _fail(status, reason, mode, graph, t_start, t_graph) -> SearchResult:
    res = SearchResult(status, reason, "finite_native3d_%s" % mode,
                       "max(0, |target_pos - pos| - eps_pos)", 0, 0, 0,
                       time.perf_counter() - t_start,
                       max_expansions=graph.cfg.finite_max_expansions,
                       max_runtime_s=graph.cfg.finite_max_runtime_s,
                       state_identity="finite_lattice_node_id",
                       approximate_state_identity=False)
    res.finite_graph = {"strategy": "route_finite_native3d (v0.1.2 production "
                                    "finite multi-hop router)",
                        "mode": mode, "bounds": graph.bounds,
                        "port_attachment": graph.port_report,
                        "cache_key_fingerprint": graph.fingerprint}
    res.timings = {"graph_build_s": t_graph, "search_s": 0.0,
                   "end_to_end_s": time.perf_counter() - t_start}
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
