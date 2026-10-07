"""Configuration for the native 3D waveguide router (Native3D v0.1).

All lengths are micrometres (um).  Angles in this configuration object are in
degrees; every angle inside native3d.geometry is in radians.

Three distance notions are kept strictly separate (never silently conflated):

dmin_centerline_um
    centreline-to-centreline spacing between two waveguide cores.
obstacle_clearance_um
    centreline-to-obstacle keep-out distance (clearance from a core centreline
    to the surface of a forbidden solid).
wg_envelope_radius_um
    physical half-width (process envelope) of one waveguide; used as the margin
    the centreline must keep from the chip-volume boundary, and optionally
    (only when obstacle_clearance_includes_envelope is True) added to the
    obstacle keep-out.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class RouterConfig:
    # ---------------------------------------------------------------- units
    length_unit: str = "um"
    angle_config_unit: str = "deg"

    # ---------------------------------------------------- numeric tolerances
    pos_tol_um: float = 1e-6          # C1 position continuity tolerance
    ang_tol_deg: float = 1e-6         # C1 tangent continuity tolerance
    radius_tol_um: float = 1e-6       # slack on R >= Rmin decisions
    zero_tol_um: float = 1e-9         # below this an object is degenerate
    parallel_tol: float = 1e-9        # |sin| below this counts as parallel
    clearance_decision_tol_um: float = 1e-9
    max_subdiv_level: int = 9         # conservative subdivision depth cap

    # ------------------------------------------------------- hard constraints
    rmin_um: float = 50.0
    dmin_centerline_um: float = 5.0
    obstacle_clearance_um: float = 5.0
    obstacle_clearance_includes_envelope: bool = False
    wg_envelope_radius_um: float = 2.0
    boundary_margin_um: Optional[float] = None  # None -> wg_envelope_radius_um

    # ----------------------------------------------------------------- search
    allow_out_of_plane: bool = True
    # State identity of the A* graph.  "pose_float" (default) is the identity of
    # the stored pose itself: position and direction are the six floats they are,
    # with NO rounding and no tolerance, so two states are merged only when they
    # are the same pose.  The optimality claim of search/astar.py is made for the
    # graph enumerated with that identity, up to the documented cost comparison
    # tolerance (astar.G_COMPARISON_ABS_TOL_UM / G_COMPARISON_REL_TOL).
    # "approximate_quantised" is the v0.1 coarse cell key: it is an explicitly
    # APPROXIMATE exploration mode, its exhaustion is not a proof of
    # unreachability, and the correctness acceptance never uses it.
    # ("exact" is accepted only as a v0.1.1 read alias and resolves to
    # "pose_float"; the v0.1.1 mode of that name rounded poses to 1e-9 um.)
    state_identity: str = "pose_float"
    pos_resolution_um: float = 10.0   # APPROXIMATE mode: cell size on position
    dir_resolution_deg: float = 15.0  # APPROXIMATE mode: cell size on direction
    eps_pos_um: float = 2.0           # TARGET acceptance ball: position
    eps_ang_deg: float = 3.0          # TARGET acceptance ball: tangent
    # SOURCE pose tolerances.  The path must START at the source pose; this is a
    # different (and much tighter) requirement than the target acceptance ball,
    # which is why it has its own two numbers instead of reusing eps_pos/eps_ang.
    # The independent validator applies them to the path start.
    source_pos_tol_um: float = 1e-6
    source_ang_tol_deg: float = 1e-6
    straight_lengths_um: Tuple[float, ...] = (10.0, 20.0, 40.0)
    turn_angles_deg: Tuple[float, ...] = (15.0, 30.0, 45.0, 60.0, 90.0)
    turn_radii_um: Tuple[float, ...] = (50.0, 75.0, 100.0)
    skew_axis_tilt_deg: Tuple[float, ...] = (45.0, 135.0)
    connector_radii_um: Tuple[float, ...] = (50.0, 75.0, 100.0)
    connector_max_length_um: float = 3000.0
    max_expansions: int = 200_000
    max_runtime_s: float = 300.0

    # ------------------------------------------------- finite reference graph
    # Lattice spacing of the canonical pose set used by
    # search/reference_graph.py (positions are anchored at the source pose).
    reference_spacing_um: float = 60.0

    # --------------------------------------------- v0.1.2 finite PRODUCTION graph
    # search/finite_graph.py: integer node ids (i, j, k, direction), six axis
    # directions, straights of n * spacing, quarter turns of radius m * spacing.
    # The lattice is anchored at the source position (the port phase is an input,
    # never a router decision).
    finite_spacing_um: float = 50.0
    finite_straight_steps: Tuple[int, ...] = (1, 2, 3)
    finite_turn_radius_steps: Tuple[int, ...] = (1, 2)
    # Generic port attachment neighbourhood: radius in um and a cap on the number
    # of candidate canonical nodes tried per port.
    finite_port_attach_um: float = 150.0
    finite_port_attach_max_nodes: int = 64
    finite_max_expansions: int = 200_000
    finite_max_runtime_s: float = 60.0

    def __post_init__(self) -> None:
        if self.rmin_um <= 0:
            raise ValueError("rmin_um must be > 0")
        if self.dmin_centerline_um < 0 or self.obstacle_clearance_um < 0:
            raise ValueError("spacing/clearance must be >= 0")
        if self.wg_envelope_radius_um < 0:
            raise ValueError("wg_envelope_radius_um must be >= 0")
        if self.pos_resolution_um <= 0 or self.dir_resolution_deg <= 0:
            raise ValueError("quantisation resolutions must be > 0")
        from .identity import (POSE_FLOAT_STATE_IDENTITY, STATE_IDENTITIES,
                               normalise_state_identity)
        self.state_identity = normalise_state_identity(str(self.state_identity))
        if self.state_identity not in STATE_IDENTITIES:
            raise ValueError(
                "state_identity must be %r (default; the stored float pose, no "
                "rounding) or 'approximate_quantised' (explicitly approximate "
                "exploration); got %r"
                % (POSE_FLOAT_STATE_IDENTITY, self.state_identity))
        if self.eps_pos_um < 0 or self.eps_ang_deg < 0:
            raise ValueError("goal tolerances must be >= 0")
        if self.source_pos_tol_um < 0 or self.source_ang_tol_deg < 0:
            raise ValueError("source pose tolerances must be >= 0")
        if self.max_subdiv_level < 1:
            raise ValueError("max_subdiv_level must be >= 1")
        if self.reference_spacing_um <= 0:
            raise ValueError("reference_spacing_um must be > 0")
        if self.finite_spacing_um <= 0:
            raise ValueError("finite_spacing_um must be > 0")
        if not self.finite_straight_steps or any(int(n) < 1 for n in self.finite_straight_steps):
            raise ValueError("finite_straight_steps must be a non-empty tuple of "
                             "integers >= 1")
        if not self.finite_turn_radius_steps or any(int(m) < 1
                                                    for m in self.finite_turn_radius_steps):
            raise ValueError("finite_turn_radius_steps must be a non-empty tuple of "
                             "integers >= 1")
        if self.finite_port_attach_um <= 0:
            raise ValueError("finite_port_attach_um must be > 0")
        if self.finite_port_attach_max_nodes < 1:
            raise ValueError("finite_port_attach_max_nodes must be >= 1")
        if self.finite_max_expansions < 1 or self.finite_max_runtime_s <= 0:
            raise ValueError("finite budgets must be positive")
        for r in tuple(self.turn_radii_um) + tuple(self.connector_radii_um):
            if r < self.rmin_um - self.radius_tol_um:
                raise ValueError("action radius %r below rmin_um %r" % (r, self.rmin_um))

    # ------------------------------------------------------------- accessors
    @property
    def effective_boundary_margin_um(self) -> float:
        """Margin the centreline keeps from the chip-volume boundary."""
        if self.boundary_margin_um is None:
            return self.wg_envelope_radius_um
        return self.boundary_margin_um

    @property
    def effective_obstacle_clearance_um(self) -> float:
        """Centreline-to-obstacle keep-out actually enforced."""
        c = self.obstacle_clearance_um
        if self.obstacle_clearance_includes_envelope:
            c += self.wg_envelope_radius_um
        return c

    def to_dict(self) -> dict:
        d = dict(self.__dict__)
        for k, v in list(d.items()):
            if isinstance(v, tuple):
                d[k] = list(v)
        d["effective_boundary_margin_um"] = self.effective_boundary_margin_um
        d["effective_obstacle_clearance_um"] = self.effective_obstacle_clearance_um
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "RouterConfig":
        keys = set(cls.__dataclass_fields__)
        kw = {k: v for k, v in d.items() if k in keys}
        for k in ("straight_lengths_um", "turn_angles_deg", "turn_radii_um",
                  "skew_axis_tilt_deg", "connector_radii_um",
                  "finite_straight_steps", "finite_turn_radius_steps"):
            if k in kw and kw[k] is not None:
                kw[k] = tuple(float(x) for x in kw[k])
        return cls(**kw)