"""v0.2 pre-implementation geometry spike (throwaway, not evidence).

Confirms with the EXISTING v0.1.3 router that the two geometries proposed by the
v0.2 prompt are really routable, so the multichannel scheduler is not built on a
geometry that cannot work.  Run with:

    .venv\\Scripts\\python.exe artifacts\\v020\\probe\\v020_geometry_spike.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "src"))

from native3d.config import RouterConfig
from native3d.constraints.checks import Box3D
from native3d.constraints.checks import validate_segments
from native3d.geometry.primitives import LineSegment3D, Pose3D
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


def run(label, scene, refs_extra=()):
    sc = Scene(name=scene.name, chip=scene.chip, obstacles=scene.obstacles,
               reference_centerlines=list(scene.reference_centerlines) + list(refs_extra),
               source=scene.source, target=scene.target, config=scene.config,
               metadata=dict(scene.metadata))
    t0 = time.perf_counter()
    res = route_finite_native3d(sc, mode="native3d")
    dt = time.perf_counter() - t0
    n_seg = len(res.path.segments) if res.path is not None else 0
    print("%-28s status=%-22s len=%-10s exp=%-7d seg=%-3d t=%.3fs"
          % (label, res.status,
             ("%.3f" % res.length_um) if res.length_um != float("inf") else "inf",
             res.expansions, n_seg, dt))
    if res.status != "success":
        print("      reason: %s" % (res.reason or "")[:220])
        fg = res.finite_graph or {}
        print("      port_report: %s" % (fg.get("port_attachment_report")
                                         or fg.get("port_report")))
    else:
        v = validate_segments(tuple(res.path.segments), sc.chip, sc.obstacles,
                              sc.reference_centerlines, sc.config, detailed=False)
        print("      hard_constraints_ok=%s  z-range=[%.3f, %.3f]"
              % (v["ok"],
                 min(s.aabb()[0][2] for s in res.path.segments),
                 max(s.aabb()[1][2] for s in res.path.segments)))
    return res


CHIP = Box3D((0.0, 0.0, -60.0), (600.0, 300.0, 120.0))


def case2_bar():
    """Prompt section 5 item 2: A bar blocks B's planar path."""
    cfg = cfg_factory()
    a_src = Pose3D((300.0, 2.0, 0.0), (0.0, 1.0, 0.0))
    a_tgt = Pose3D((300.0, 298.0, 0.0), (0.0, 1.0, 0.0))
    scA = Scene("A_bar_exact_prompt", CHIP, [], [], a_src, a_tgt, cfg, {})
    resA = run("A_bar(2->298) as-prompt", scA)

    a_tgt2 = Pose3D((300.0, 252.0, 0.0), (0.0, 1.0, 0.0))
    scA2 = Scene("A_bar_canonical_target", CHIP, [], [], a_src, a_tgt2, cfg, {})
    resA2 = run("A_bar(2->252) canonical", scA2)

    scB = Scene("B_cross", CHIP, [], [],
                Pose3D((60.0, 150.0, 0.0), (1.0, 0.0, 0.0)),
                Pose3D((540.0, 150.0, 0.0), (1.0, 0.0, 0.0)), cfg, {})
    run("B_cross free", scB)
    if resA.path is not None:
        run("B_cross + A_bar(2->298)", scB, refs_extra=list(resA.path.segments))
    if resA2.path is not None:
        run("B_cross + A_bar(2->252)", scB, refs_extra=list(resA2.path.segments))


def crosswall_net(i, n, swap=False):
    cfg = cfg_factory()
    wall = Box3D((290.0, -50.0, -70.0), (310.0, 350.0, 40.0))
    src = Pose3D((60.0, 60.0 + 10.0 * i, 0.0), (1.0, 0.0, 0.0))
    tgt_y = 60.0 + 10.0 * ((n - 1 - i) if swap else i)
    tgt = Pose3D((540.0, tgt_y, 0.0), (1.0, 0.0, 0.0))
    return Scene("crosswall_net%02d" % i, CHIP, [wall], [], src, tgt, cfg, {})


def main():
    print("=" * 78)
    print("SPIKE 1: prompt section 5 two-channel barrier geometry")
    print("=" * 78)
    case2_bar()

    print()
    print("=" * 78)
    print("SPIKE 2: 16-channel cross-wall series, first four nets sequentially")
    print("=" * 78)
    published = []
    for i in range(4):
        sc = crosswall_net(i, 16)
        res = run("crosswall net %d" % i, sc, refs_extra=published)
        if res.path is None:
            break
        published.extend(list(res.path.segments))

    print()
    print("=" * 78)
    print("SPIKE 3: same series with swapped (crossing) targets, first four")
    print("=" * 78)
    published = []
    for i in range(4):
        sc = crosswall_net(i, 4, swap=True)
        res = run("swap net %d" % i, sc, refs_extra=published)
        if res.path is None:
            break
        published.extend(list(res.path.segments))


if __name__ == "__main__":
    main()
