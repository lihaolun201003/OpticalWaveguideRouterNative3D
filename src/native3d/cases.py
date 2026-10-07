"""Case definitions (synthetic geometry, micrometres).

All numbers here are SYNTHETIC geometry parameters for correctness testing.
They are NOT material parameters of any paper and no optical loss in dB is
derived from them anywhere in this project.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

from .config import RouterConfig
from .constraints.checks import Box3D
from .geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from .geometry.path3d import Path3D
from .geometry.vec import Vec3, add, cross, dot, norm, scale, sub, unit
from .io.caseio import Scene


def base_config(**over) -> RouterConfig:
    cfg = dict(
        rmin_um=50.0,
        dmin_centerline_um=5.0,
        obstacle_clearance_um=5.0,
        obstacle_clearance_includes_envelope=False,
        wg_envelope_radius_um=2.0,
        boundary_margin_um=None,
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
        max_expansions=50000,
        max_runtime_s=300.0,
    )
    cfg.update(over)
    return RouterConfig(**cfg)


# ------------------------------------------------------------------ core case
def core_case(config: Optional[RouterConfig] = None) -> Scene:
    """Identical inputs for both modes: source and target both at z = 0.

    A solid wall spans the complete lateral width of the chip in the z = 0
    slice (its y range exceeds the chip's y range, so there is no side gap) and
    covers z from below the chip floor up to z = 40, so source and target lie in
    different connected regions of the z = 0 plane.  Above z = 40 + clearance
    there is open volume up to the chip ceiling, which is where a 3D path can
    pass.  Going outside the chip volume is forbidden by the boundary
    constraint.
    """
    cfg = config or base_config()
    # z = 0 is the port plane INSIDE the chip volume: the volume extends
    # 60 um below it, and the wall reaches below the chip floor, so there is no
    # way under the wall either.
    chip = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))
    wall = Box3D((290.0, -50.0, -70.0), (310.0, 350.0, 40.0))
    ref = LineSegment3D((150.0, 120.0, 0.0), (450.0, 120.0, 0.0))
    return Scene(
        name="core_2d_unreachable_vs_native3d_reachable",
        chip=chip,
        obstacles=[wall],
        reference_centerlines=[ref],
        source=Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
        target=Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
        config=cfg,
        metadata={
            "purpose": "2D mode must be unreachable, Native3D mode must be reachable",
            "wall_in_plane_separation": {
                "wall_x_range_um": [290.0, 310.0],
                "wall_covers_y": "wall y range [-50, 350] contains chip y range [0, 300]",
                "wall_z_range_um": [-70.0, 40.0],
                "chip_z_range_um": [-60.0, 120.0],
                "port_plane_z_um": 0.0,
                "argument": ("in the z = 0 plane the wall plus the chip boundary "
                             "separates the source region x <= 285 from the target "
                             "region x >= 315; nothing in that plane connects them"),
            },
            "free_height_above_wall_um": 120.0 - 40.0,
            "synthetic": True,
        })


# ------------------------------------------------------------- extra cases
def sealed_target_case() -> Scene:
    """Target individually legal but its feasible space is sealed off."""
    cfg = base_config(max_expansions=20000, max_runtime_s=120.0,
                      pos_resolution_um=20.0, dir_resolution_deg=30.0)
    chip = Box3D((0.0, 0.0, 0.0), (200.0, 200.0, 60.0))
    walls = [
        Box3D((80.0, 80.0, -10.0), (90.0, 140.0, 50.0)),     # left wall
        Box3D((130.0, 80.0, -10.0), (140.0, 140.0, 50.0)),   # right wall
        Box3D((80.0, 80.0, -10.0), (140.0, 90.0, 50.0)),     # front wall
        Box3D((80.0, 130.0, -10.0), (140.0, 140.0, 50.0)),   # back wall
        Box3D((80.0, 80.0, 40.0), (140.0, 140.0, 50.0)),     # ceiling
    ]
    return Scene(
        name="sealed_target_unreachable",
        chip=chip, obstacles=walls, reference_centerlines=[],
        source=Pose3D((20.0, 30.0, 10.0), (1.0, 0.0, 0.0)),
        target=Pose3D((110.0, 110.0, 10.0), (1.0, 0.0, 0.0)),
        config=cfg,
        metadata={"purpose": "search exhaustion vs budget exhaustion",
                  "argument": ("the cavity x in [90,130], y in [90,130], z in [0,40] is "
                               "closed by four walls that reach below the chip floor and "
                               "by a ceiling; the target sits inside it, so no legal "
                               "primitive sequence can enter"),
                  "synthetic": True})


def terminal_tangent_case(correct_tangent: bool = True,
                          config: Optional[RouterConfig] = None) -> Scene:
    """Dead-end pocket: only an arrival along +x can be legal.

    Pocket free centreline space: x in [160, 175], y in [45, 55], z in [2, 38]
    (10 um wide in y).  A 90 degree turn needs R >= Rmin = 50 um, five times the
    corridor width, so an arrival with the +y tangent is geometrically
    impossible; the same scene with the +x tangent is reachable.
    """
    cfg = config or base_config(max_expansions=20000, max_runtime_s=180.0,
                                pos_resolution_um=15.0, dir_resolution_deg=30.0)
    chip = Box3D((0.0, 0.0, 0.0), (240.0, 100.0, 40.0))
    walls = [
        Box3D((160.0, 60.0, -10.0), (200.0, 75.0, 45.0)),   # upper wall
        Box3D((160.0, 25.0, -10.0), (200.0, 40.0, 45.0)),   # lower wall
        Box3D((180.0, 40.0, -10.0), (200.0, 60.0, 45.0)),   # back wall of the pocket
    ]
    tangent = (1.0, 0.0, 0.0) if correct_tangent else (0.0, 1.0, 0.0)
    return Scene(
        name="terminal_tangent_%s" % ("correct" if correct_tangent else "wrong"),
        chip=chip, obstacles=walls, reference_centerlines=[],
        source=Pose3D((40.0, 50.0, 5.0), (1.0, 0.0, 0.0)),
        target=Pose3D((172.0, 50.0, 5.0), tangent),
        config=cfg,
        metadata={"purpose": "goal test must check position AND tangent",
                  "pocket_free_space": {"x_um": [160.0, 175.0], "y_um": [45.0, 55.0],
                                        "z_um": [2.0, 38.0]},
                  "synthetic": True})


def congestion_case(config: Optional[RouterConfig] = None) -> Scene:
    """Same geometry as the core case but with a penalty region over the wall."""
    sc = core_case(config)
    sc.name = "core_case_with_congestion_region"
    sc.metadata = dict(sc.metadata)
    sc.metadata["congestion_region"] = {"box": [[250.0, 100.0, 40.0], [350.0, 200.0, 120.0]],
                                        "density_um_per_um": 0.5}
    return sc


def mid_curve_obstacle_case() -> Scene:
    """Legal endpoints, but the arc must pass through the wall (=> rejected)."""
    cfg = base_config()
    chip = Box3D((0.0, 0.0, 0.0), (400.0, 200.0, 100.0))
    wall = Box3D((180.0, -20.0, -20.0), (220.0, 220.0, 60.0))
    return Scene(name="mid_curve_obstacle", chip=chip, obstacles=[wall],
                 reference_centerlines=[],
                 source=Pose3D((60.0, 100.0, 80.0), (1.0, 0.0, 0.0)),
                 target=Pose3D((340.0, 100.0, 80.0), (1.0, 0.0, 0.0)),
                 config=cfg,
                 metadata={"purpose": "collision in the MIDDLE of a curve",
                           "synthetic": True})


# ---------------------------------------------------- pure geometry helpers
def straight_path(p0: Vec3, p1: Vec3) -> Path3D:
    return Path3D((LineSegment3D(p0, p1),))


def tilted_plane_arc(center: Vec3, normal: Vec3, radius: float, theta: float,
                     start_angle: float = 0.0) -> CircularArc3D:
    """Arc in the plane with the given (arbitrary) normal."""
    n = unit(normal, what="plane normal")
    ref = (1.0, 0.0, 0.0)
    if abs(dot(ref, n)) > 0.9:
        ref = (0.0, 1.0, 0.0)
    u = unit(sub(ref, scale(n, dot(ref, n))), what="in-plane axis")
    v = cross(n, u)
    ca, sa = math.cos(start_angle), math.sin(start_angle)
    u2 = add(scale(u, ca), scale(v, sa))
    v2 = cross(n, u2)
    return CircularArc3D(center, u2, v2, radius, theta)
