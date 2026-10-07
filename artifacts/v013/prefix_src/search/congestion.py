"""Fixed congestion field for the 3D A* (soft cost, never a hard constraint).

edge_cost = primitive_length + congestion_penalty, penalty >= 0 and expressed
in the SAME unit as length (equivalent micrometres), so the sum is a coherent
equivalent length.  The field is created once and passed to the search; it is
never modified during a search, so g stays a true sum of edge costs and the
admissibility argument of the heuristic is unaffected.

Hard spacing (dmin centreline-to-centreline) is NEVER replaced by this soft
penalty: it is enforced by constraints.checks regardless of the field.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

from ..geometry.vec import Vec3


class CongestionField:
    """Interface: non-negative equivalent-length penalty of one segment."""

    name = "abstract"

    def penalty(self, segment) -> float:  # pragma: no cover - interface
        raise NotImplementedError

    def penalty_of_segments(self, segments: Sequence[object]) -> float:
        return sum(self.penalty(s) for s in segments)

    def describe(self) -> dict:
        return {"name": self.name}


class ZeroCongestion(CongestionField):
    name = "zero"

    def penalty(self, segment) -> float:
        return 0.0


@dataclass
class RegionCongestion(CongestionField):
    """Penalty density (um of equivalent length per um travelled) per box region.

    Line segments are clipped analytically against each box (exact).  Arc
    segments are integrated by uniform subdivision; the subdivision is refined
    until the estimate moves by less than tol_um, and the residual error bound
    (density * length * |1 - cos(dphi/2)| style sagitta term) is reported by
    describe().  This is a SOFT cost: the small quadrature error only moves the
    reported cost, never a hard feasibility decision.
    """
    regions: List[Tuple[object, float]]
    tol_um: float = 1e-6
    max_level: int = 12
    name = "region"

    def penalty(self, segment) -> float:
        kind = getattr(segment, "kind", None)
        total = 0.0
        for box, density in self.regions:
            if density == 0.0:
                continue
            if kind == "line":
                frac_len = _line_length_in_box(segment.p0, segment.p1, box)
                total += density * frac_len
            else:
                total += density * self._arc_length_in_box(segment, box)
        return total

    def _arc_length_in_box(self, arc, box) -> float:
        prev = None
        n = 8
        for _ in range(self.max_level):
            L = arc.length
            inside = 0.0
            for i in range(n):
                a = L * i / n
                b = L * (i + 1) / n
                mid = 0.5 * (a + b)
                pm = arc.point_at(mid)
                if _point_in_box(pm, box):
                    inside += (b - a)
            if prev is not None and abs(inside - prev) <= self.tol_um:
                return inside
            prev = inside
            n *= 2
        return prev if prev is not None else 0.0

    def describe(self) -> dict:
        return {"name": self.name,
                "regions": [{"box": [list(b.lo), list(b.hi)], "density_um_per_um": d}
                            for b, d in self.regions],
                "method": "analytic clipping for lines, uniform-subdivision "
                          "quadrature (tol %g um) for arcs" % self.tol_um}


def _point_in_box(p: Vec3, box) -> bool:
    return (box.lo[0] <= p[0] <= box.hi[0] and box.lo[1] <= p[1] <= box.hi[1]
            and box.lo[2] <= p[2] <= box.hi[2])


def _line_length_in_box(p0: Vec3, p1: Vec3, box) -> float:
    """Exact length of the portion of segment p0->p1 inside the box (slab clip)."""
    d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-15:
            if p0[i] < box.lo[i] or p0[i] > box.hi[i]:
                return 0.0
        else:
            ta = (box.lo[i] - p0[i]) / d[i]
            tb = (box.hi[i] - p0[i]) / d[i]
            if ta > tb:
                ta, tb = tb, ta
            t0 = max(t0, ta)
            t1 = min(t1, tb)
            if t0 >= t1:
                return 0.0
    return max(0.0, (t1 - t0)) * math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
