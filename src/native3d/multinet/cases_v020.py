"""The v0.2 multi-channel cases (synthetic geometry, micrometres).

Every number here is a SYNTHETIC GEOMETRY PARAMETER for correctness testing, not
a material parameter of any paper; no optical loss in dB and no crosstalk number
is derived from them anywhere.

Two families:

* ``two_channel_*`` - the five minimum acceptance cases of prompt section 5:
  two independent straights; a barrier that forces a native 3D detour; an arc that
  must really enter the occupancy; a case in which the second net has NO solution
  under the published occupancy (partial success, the first net untouched); and
  the same barrier case with the net order REVERSED.
* ``crosswall_*`` and ``stress_swap_*`` - the 16-channel series of prompt section
  6 (the first N nets of the same list, so the flow is identical for 2/4/8/16),
  and the deterministic crossing/permutation pressure cases.

None of these cases contains a hand-supplied path, layer height, midpoint or
layer-change position: the z evolution of every route comes from the 3D search.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from ..config import RouterConfig
from ..constraints.checks import Box3D
from ..geometry.primitives import Pose3D
from .models import MultiNetCase, NetSpec

# --------------------------------------------------------------- base config
CHIP_LARGE = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
CHIP_THIN = Box3D((0.0, 0.0, -30.0), (600.0, 300.0, 30.0))
WALL = Box3D((290.0, -50.0, -70.0), (310.0, 350.0, 40.0))


def v020_config(**over) -> RouterConfig:
    """The v0.2 baseline configuration: Rmin 50, dmin 5, 2 um envelope margin.

    The per-net budget is the project default of the finite production strategy:
    60 s and 200 000 expansions PER NET (never shared, never extended).
    """
    cfg = dict(
        rmin_um=50.0,
        dmin_centerline_um=5.0,
        obstacle_clearance_um=5.0,
        obstacle_clearance_includes_envelope=False,
        wg_envelope_radius_um=2.0,
        boundary_margin_um=2.0,
        allow_out_of_plane=True,
        pos_resolution_um=20.0,
        dir_resolution_deg=30.0,
        eps_pos_um=2.0,
        eps_ang_deg=3.0,
        straight_lengths_um=(10.0, 20.0, 40.0),
        turn_angles_deg=(30.0, 45.0, 90.0),
        turn_radii_um=(50.0, 75.0),
        skew_axis_tilt_deg=(45.0, 135.0),
        connector_radii_um=(50.0, 100.0),
        connector_max_length_um=3000.0,
        max_expansions=100_000,
        max_runtime_s=300.0,
        finite_spacing_um=50.0,
        finite_straight_steps=(1, 2, 3),
        finite_turn_radius_steps=(1, 2),
        finite_port_attach_um=150.0,
        finite_port_attach_max_nodes=64,
        finite_max_expansions=200_000,
        finite_max_runtime_s=60.0,
    )
    cfg.update(over)
    return RouterConfig(**cfg)


def _px(x: float, y: float, z: float = 0.0) -> Pose3D:
    return Pose3D((x, y, z), (1.0, 0.0, 0.0))


def _py(x: float, y: float, z: float = 0.0) -> Pose3D:
    return Pose3D((x, y, z), (0.0, 1.0, 0.0))


def _net(net_id: str, source: Pose3D, target: Pose3D, **meta) -> NetSpec:
    return NetSpec(net_id=net_id, source=source, target=target, metadata=meta)


# =====================================================================
# prompt section 5: the five two-channel minimum acceptance cases
# =====================================================================
def case_two_parallel_straights() -> MultiNetCase:
    """(1) Two mutually independent straight nets; both must be committed."""
    return MultiNetCase(
        name="two_channel_01_parallel_straights",
        chip=CHIP_LARGE, obstacles=[], reference_centerlines=[],
        config=v020_config(),
        nets=[_net("netA", _px(60, 60), _px(510, 60)),
              _net("netB", _px(60, 240), _px(510, 240))],
        metadata={
            "purpose": "two independent straight nets, no interaction at all",
            "expect": "both committed; identical 450.0 um straights",
            "synthetic": True})


def case_barrier_3d_avoidance() -> MultiNetCase:
    """(2) A blocks B's planar path; B must detour with a NATIVE 3D search."""
    return MultiNetCase(
        name="two_channel_02_barrier_3d_avoidance",
        chip=CHIP_LARGE, obstacles=[], reference_centerlines=[],
        config=v020_config(),
        nets=[_net("netA", _py(300, 2), _py(300, 298)),
              _net("netB", _px(60, 150), _px(540, 150))],
        metadata={
            "purpose": ("the prompt's barrier geometry: netA is a straight bar in the "
                        "z = 0 plane that spans the whole inner y range, so netB's "
                        "planar route is separated"),
            "expect": ("netA committed (straight, z = 0); netB committed with a z "
                       "excursion because only the 3D search can pass"),
            "netA_geometry": {"source": [300, 2, 0], "target": [300, 298, 0],
                              "tangent": [0, 1, 0]},
            "netB_geometry": {"source": [60, 150, 0], "target": [540, 150, 0],
                              "tangent": [1, 0, 0]},
            "synthetic": True})


def case_arc_occupancy() -> MultiNetCase:
    """(3) netA contains an ARC; netB really has to avoid that complete arc."""
    return MultiNetCase(
        name="two_channel_03_arc_occupancy",
        chip=CHIP_LARGE, obstacles=[], reference_centerlines=[],
        config=v020_config(),
        nets=[_net("netA", _px(60, 60), _py(410, 110)),
              _net("netB", _px(200, 90), _px(500, 90))],
        metadata={
            "purpose": ("netA turns from +x to +y, so its returned path contains a "
                        "quarter-turn arc; netB's straight z = 0 route crosses that "
                        "arc, so the arc has to be a real occupancy curve"),
            "expect": ("netA committed with at least one arc segment; netB committed "
                       "with a detour, and the joint validator certifies the spacing "
                       "to that arc"),
            "netB_control_length_um": 300.0,
            "arc_region": {"center": [360, 110, 0], "radius_um": 50.0,
                           "crosses_netB_line_at_x_um": 405.8},
            "synthetic": True})


def case_no_solution_partial_success() -> MultiNetCase:
    """(4) netB has NO solution under the published occupancy: partial success."""
    return MultiNetCase(
        name="two_channel_04_no_solution_under_occupancy",
        chip=CHIP_THIN, obstacles=[], reference_centerlines=[],
        config=v020_config(),
        nets=[_net("netA", _py(300, 2), _py(300, 298)),
              _net("netB", _px(60, 150), _px(540, 150))],
        metadata={
            "purpose": ("the chip is only 60 um tall, so the finite lattice has the "
                        "z = 0 plane only; netA spans the whole inner y range in that "
                        "plane, hence netB cannot detour in y (boundary) or in z "
                        "(no lattice level inside the shrunk volume)"),
            "expect": ("netA committed; netB search_exhausted -> PARTIAL SUCCESS with "
                       "netA left untouched"),
            "control": ("the same netB alone in the same chip is committed, which is "
                        "what shows the published occupancy is the cause"),
            "chip_z_inner_range_um": [-28.0, 28.0],
            "lattice_z_levels_um": [0.0],
            "synthetic": True})


def case_reversed_order() -> MultiNetCase:
    """(5) The barrier case with the input order REVERSED (B first, then A)."""
    c = case_barrier_3d_avoidance()
    return MultiNetCase(
        name="two_channel_05_reversed_order",
        chip=c.chip, obstacles=list(c.obstacles),
        reference_centerlines=list(c.reference_centerlines), config=c.config,
        nets=list(reversed(c.nets)),
        metadata={
            "purpose": ("the same geometry as two_channel_02 with the nets swapped, to "
                        "show that the order really matters: no ordering heuristic is "
                        "used in v0.2"),
            "expect": ("netB committed first as a plain straight (480.0 um), then netA "
                       "committed with a z excursion (different from case 02, where "
                       "netA was the straight bar)"),
            "same_geometry_as": "two_channel_02_barrier_3d_avoidance",
            "synthetic": True})


def two_channel_cases() -> List[MultiNetCase]:
    return [case_two_parallel_straights(), case_barrier_3d_avoidance(),
            case_arc_occupancy(), case_no_solution_partial_success(),
            case_reversed_order()]


# =====================================================================
# prompt section 6.1: the 16-channel cross-wall series
# =====================================================================
def crosswall_case(channel_count: int) -> MultiNetCase:
    """The first ``channel_count`` nets of the SAME ordered 16-net list.

    Each net crosses the wall in its own y lane; the z excursion comes from the 3D
    search (no hand-supplied height).  There is no static waveguide occupancy at
    the start, and each net is published as soon as it is verified.
    """
    nets = []
    for i in range(channel_count):
        y = 60.0 + 10.0 * i
        nets.append(_net("net%02d" % i, _px(60, y), _px(540, y),
                         lane_y_um=y, index=i))
    return MultiNetCase(
        name="crosswall_%02d_channels" % channel_count,
        chip=CHIP_LARGE, obstacles=[WALL], reference_centerlines=[],
        config=v020_config(), nets=nets,
        metadata={
            "purpose": ("the prompt's 16-channel must-pass baseline: a wall blocks the "
                        "port plane, every net crosses it by its own 3D detour"),
            "expect": "all %d nets committed and jointly verified" % channel_count,
            "series": ("2/4/8/16 all use the FIRST N nets of this list, so the main "
                       "flow is identical"),
            "wall": {"lo": [290, -50, -70], "hi": [310, 350, 40]},
            "net_i_source": "x=60, y=60+10*i, z=0, tangent +x",
            "net_i_target": "x=540, y=60+10*i, z=0, tangent +x",
            "no_hand_supplied_height": True,
            "synthetic": True})


def crosswall_cases() -> List[MultiNetCase]:
    return [crosswall_case(n) for n in (2, 4, 8, 16)]


# =====================================================================
# prompt section 6.2: the deterministic pressure cases
# =====================================================================
def stress_swap_case(net_count: int) -> MultiNetCase:
    """Deterministic crossing/permutation case (prompt section 6.2).

    net i starts in lane i and must end in lane ``net_count-1-i``, so the nets
    cross; every net is still a single straight-line port pair in the port plane,
    but the order in which they are routed decides who gets through.  Partial
    failure is EXPECTED and is reported; v0.2 adds no rip-up & reroute and no net
    ordering optimisation.
    """
    nets = []
    for i in range(net_count):
        y_src = 60.0 + 10.0 * i
        y_tgt = 60.0 + 10.0 * (net_count - 1 - i)
        nets.append(_net("net%02d" % i, _px(60, y_src), _px(540, y_tgt),
                         source_lane_y_um=y_src, target_lane_y_um=y_tgt, index=i))
    return MultiNetCase(
        name="stress_swap_%02d_channels" % net_count,
        chip=CHIP_LARGE, obstacles=[WALL], reference_centerlines=[],
        config=v020_config(), nets=nets,
        metadata={
            "purpose": ("a deterministic port-permutation pressure case: the nets must "
                        "cross each other and cross the wall, and the already published "
                        "occupancy really changes what the later searches can do"),
            "expect": ("partial failure is allowed and is reported per net with its own "
                       "failure category; the case is never reported as all_success"),
            "partial_success_is_allowed": True,
            "synthetic": True})


def stress_cases() -> List[MultiNetCase]:
    return [stress_swap_case(4), stress_swap_case(8)]


# --------------------------------------------------------------- registries
CONTROL_NETS = {
    "two_channel_04_no_solution_under_occupancy": [
        ("netB_alone_control", ["netB"]),
        ("netA_alone_control", ["netA"]),
    ],
    "two_channel_02_barrier_3d_avoidance": [
        ("netB_before_A_control", ["netB"]),
    ],
    "two_channel_03_arc_occupancy": [
        ("netB_without_A_control", ["netB"]),
    ],
}


def with_nets(case: MultiNetCase, net_ids: List[str]) -> MultiNetCase:
    by_id = {n.net_id: n for n in case.nets}
    return MultiNetCase(name="%s::control" % case.name, chip=case.chip,
                        obstacles=list(case.obstacles),
                        reference_centerlines=list(case.reference_centerlines),
                        config=case.config, nets=[by_id[i] for i in net_ids],
                        metadata=dict(case.metadata, control=True))


def all_cases() -> List[MultiNetCase]:
    return (two_channel_cases() + crosswall_cases() + stress_cases())
