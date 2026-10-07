"""Self-spacing module smoke / regression probe (v0.2 development)."""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "src"))

from native3d.config import RouterConfig
from native3d.geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from native3d.multinet.models import SelfSpacingPolicy
from native3d.multinet.self_spacing import path_self_spacing_global

CFG = RouterConfig(rmin_um=50.0, dmin_centerline_um=5.0, obstacle_clearance_um=5.0,
                   wg_envelope_radius_um=2.0, boundary_margin_um=2.0,
                   finite_spacing_um=50.0, finite_max_runtime_s=60.0)
ELL = SelfSpacingPolicy(ell_local_um=2.0 * CFG.dmin_centerline_um, kind="two_times_dmin")


def show(label, segs, expect=None):
    r = path_self_spacing_global(segs, CFG, ELL)
    w = r["witness"]
    extra = ""
    if w:
        extra = " witness_gap=%.4f dist=%.4f in_domain=%s" % (
            w["arclength_separation_um"], w["distance_sampled_um"],
            w["in_check_domain"])
    print("%-46s status=%-11s LB=%-12s UB=%-10s undec=%d%s"
          % (label, r["status"],
             ("%.6f" % r["certified_lower_bound_um"]
              if r["certified_lower_bound_um"] is not None else "None"),
             ("%.6f" % r["certified_upper_bound_um"]
              if r["certified_upper_bound_um"] is not None else "None"),
             r["n_pairs_undecided"], extra))
    if expect and r["status"] != expect:
        print("   !!! EXPECTED %s" % expect)
    return r


def arc(center, u, v, R, theta):
    return CircularArc3D(center, u, v, R, theta)


DEG = math.pi / 180.0

print("--- 1/2: identical conclusion split vs unsplit (C1 straight) ---")
show("one straight 0..400", (LineSegment3D((0, 0, 0), (400, 0, 0)),), "safe")
show("same straight split 4x", (LineSegment3D((0, 0, 0), (100, 0, 0)),
                               LineSegment3D((100, 0, 0), (200, 0, 0)),
                               LineSegment3D((200, 0, 0), (300, 0, 0)),
                               LineSegment3D((300, 0, 0), (400, 0, 0)),), "safe")

print()
print("--- 3: a normal spatial bend passes ---")
q = arc((0, 0, 50), (1, 0, 0), (0, 0, -1), 50.0, 90 * DEG)
show("quarter arc R=50 in xz", (q,), "safe")
q2 = arc((0, 0, 50), (1, 0, 0), (0, 1, 0), 50.0, 90 * DEG)
show("quarter arc R=50 in xy", (q2,), "safe")

print()
print("--- 4: multi-turn re-entry, single segment and split ---")
full = arc((0, 0, 0), (1, 0, 0), (0, 1, 0), 50.0, 2 * math.pi)
show("ONE full-turn arc (single segment)", (full,), "violation")
parts = tuple(arc((0, 0, 0), (1, 0, 0), (0, 1, 0), 50.0, 0.5 * math.pi)
              for _ in range(4))
# the four quarter arcs must be re-anchored consecutively to form the same circle
chains = []
for k in range(4):
    a = arc((0, 0, 0), (1, 0, 0), (0, 1, 0), 50.0, 0.5 * math.pi)
    # rotate the basis so the sub-arc starts where the previous one ended
    ang = k * 0.5 * math.pi
    u = (math.cos(ang), math.sin(ang), 0.0)
    v = (-math.sin(ang), math.cos(ang), 0.0)
    chains.append(arc((0, 0, 0), u, v, 50.0, 0.5 * math.pi))
show("same circle split into 4 arcs", tuple(chains), "violation")
two_turn = arc((0, 0, 0), (1, 0, 0), (0, 1, 0), 50.0, 4 * math.pi)
show("two-turn arc (single segment)", (two_turn,), "violation")

print()
print("--- 5: real self-approach, witness must be in the check domain ---")
show("two parallel lines 2 um apart",
     (LineSegment3D((0, 0, 0), (100, 0, 0)),
      LineSegment3D((0, 2, 0), (100, 2, 0))), "violation")
show("U-turn legs 4 um apart",
     (LineSegment3D((0, 0, 0), (100, 0, 0)),
      LineSegment3D((100, 4, 0), (0, 4, 0))), "violation")
show("same pair but 40 um apart",
     (LineSegment3D((0, 0, 0), (100, 0, 0)),
      LineSegment3D((100, 40, 0), (0, 40, 0))), "safe")

print()
print("--- 6: adjacent segments joined by a real quarter turn (band handling) ---")
j = (LineSegment3D((0, 0, 0), (200, 0, 0)),)
# the correct turn: from (200,0,0) heading +x to (250,50,0) heading +y
turn = arc((200, 50, 0), (0, -1, 0), (1, 0, 0), 50.0, 90 * DEG)
show("straight + quarter turn (C1 junction)", (j[0], turn), "safe")
show("straight + quarter turn + straight",
     (j[0], turn, LineSegment3D((250, 50, 0), (250, 200, 0))), "safe")

print()
print("--- 7: degenerate small path (whole path inside the band) ---")
show("5 um straight (< ell_local)", (LineSegment3D((0, 0, 0), (5, 0, 0)),))
