"""v0.2 pre-implementation spike, part 2: the five two-channel acceptance cases.

Throwaway reconnaissance, not evidence.  Confirms each of the five minimum cases
of prompt section 5 has a real, reproducible outcome with the EXISTING v0.1.3
router before the multichannel scheduler is written.
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "src"))

from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D, validate_segments
from native3d.geometry.primitives import Pose3D
from native3d.io.caseio import Scene
from native3d.search.finite_graph import route_finite_native3d


def cfg_factory(**over):
    cfg = dict(rmin_um=50.0, dmin_centerline_um=5.0, obstacle_clearance_um=5.0,
               obstacle_clearance_includes_envelope=False,
               wg_envelope_radius_um=2.0, boundary_margin_um=2.0,
               allow_out_of_plane=True, eps_pos_um=2.0, eps_ang_deg=3.0,
               finite_spacing_um=50.0, finite_straight_steps=(1, 2, 3),
               finite_turn_radius_steps=(1, 2), finite_port_attach_um=150.0,
               finite_port_attach_max_nodes=64,
               finite_max_expansions=200_000, finite_max_runtime_s=60.0)
    cfg.update(over)
    return RouterConfig(**cfg)


def route(name, chip, obstacles, src, tgt, published=(), static_refs=()):
    cfg = cfg_factory()
    sc = Scene(name, chip, list(obstacles), list(static_refs) + list(published),
               src, tgt, cfg, {})
    t0 = time.perf_counter()
    res = route_finite_native3d(sc, mode="native3d")
    dt = time.perf_counter() - t0
    segs = list(res.path.segments) if res.path is not None else []
    kinds = "".join("A" if s.kind == "arc" else "L" for s in segs)
    zr = ("[%.1f,%.1f]" % (min(s.aabb()[0][2] for s in segs),
                           max(s.aabb()[1][2] for s in segs))) if segs else "-"
    print("  %-26s %-18s len=%-9s seg=%-3d %-10s z=%-12s exp=%-6d %.3fs"
          % (name, res.status,
             ("%.3f" % res.length_um) if res.length_um != float("inf") else "inf",
             len(segs), kinds, zr, res.expansions, dt))
    if res.status not in ("success",):
        print("        reason: %s" % (res.reason or "")[:200])
    elif segs:
        v = validate_segments(tuple(segs), sc.chip, sc.obstacles,
                              sc.reference_centerlines, sc.config, detailed=False)
        if not v["ok"]:
            print("        HARD CONSTRAINT FAILED: %s" % v.get("failed"))
    return res


CHIP = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))


def main():
    print("CASE 1: two independent straight lines")
    n1 = route("s1", CHIP, [], Pose3D((60, 60, 0), (1, 0, 0)),
               Pose3D((510, 60, 0), (1, 0, 0)))
    n2 = route("s2", CHIP, [], Pose3D((60, 240, 0), (1, 0, 0)),
               Pose3D((510, 240, 0), (1, 0, 0)),
               published=n1.path.segments if n1.path else ())

    print("CASE 2: A bar blocks B's planar path (prompt geometry)")
    A = route("A_bar", CHIP, [], Pose3D((300, 2, 0), (0, 1, 0)),
              Pose3D((300, 298, 0), (0, 1, 0)))
    route("B_free(control)", CHIP, [], Pose3D((60, 150, 0), (1, 0, 0)),
          Pose3D((540, 150, 0), (1, 0, 0)))
    route("B_after_A", CHIP, [], Pose3D((60, 150, 0), (1, 0, 0)),
          Pose3D((540, 150, 0), (1, 0, 0)),
          published=A.path.segments if A.path else ())

    print("CASE 3: A contains an arc; B crosses that arc")
    A3 = route("A_arc", CHIP, [], Pose3D((60, 60, 0), (1, 0, 0)),
               Pose3D((410, 110, 0), (0, 1, 0)))
    route("B_free(control)", CHIP, [], Pose3D((200, 90, 0), (1, 0, 0)),
          Pose3D((500, 90, 0), (1, 0, 0)))
    route("B_after_A_arc", CHIP, [], Pose3D((200, 90, 0), (1, 0, 0)),
          Pose3D((500, 90, 0), (1, 0, 0)),
          published=A3.path.segments if A3.path else ())

    print("CASE 4: thin chip; A seals the y direction; B has no 3D option")
    thin = Box3D((0.0, 0.0, -30.0), (600.0, 300.0, 30.0))
    A4 = route("A_seal", thin, [], Pose3D((300, 2, 0), (0, 1, 0)),
               Pose3D((300, 298, 0), (0, 1, 0)))
    route("B_free(control)", thin, [], Pose3D((60, 150, 0), (1, 0, 0)),
          Pose3D((540, 150, 0), (1, 0, 0)))
    route("B_after_A_seal", thin, [], Pose3D((60, 150, 0), (1, 0, 0)),
          Pose3D((540, 150, 0), (1, 0, 0)),
          published=A4.path.segments if A4.path else ())

    print("CASE 5: reversed input order on the case-2 geometry")
    B5 = route("B_first", CHIP, [], Pose3D((60, 150, 0), (1, 0, 0)),
               Pose3D((540, 150, 0), (1, 0, 0)))
    route("A_after_B", CHIP, [], Pose3D((300, 2, 0), (0, 1, 0)),
          Pose3D((300, 298, 0), (0, 1, 0)),
          published=B5.path.segments if B5.path else ())


if __name__ == "__main__":
    main()
