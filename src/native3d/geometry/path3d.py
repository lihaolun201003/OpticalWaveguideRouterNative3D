"""Phase 1: ordered segment collection with cumulative length and C1 checks."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

from .primitives import CircularArc3D, LineSegment3D, Pose3D
from .vec import Vec3, angle_between, distance


@dataclass
class Path3D:
    segments: Tuple[object, ...]

    def __post_init__(self) -> None:
        if not self.segments:
            raise ValueError("Path3D needs at least one segment")
        for seg in self.segments:
            if getattr(seg, "kind", None) not in ("line", "arc"):
                raise ValueError("Path3D segments must be LineSegment3D or CircularArc3D")

    @property
    def total_length(self) -> float:
        return sum(seg.length for seg in self.segments)

    @property
    def cumulative_lengths(self) -> List[float]:
        out = [0.0]
        for seg in self.segments:
            out.append(out[-1] + seg.length)
        return out

    @property
    def spans(self) -> List[Tuple[float, float]]:
        cl = self.cumulative_lengths
        return [(cl[i], cl[i + 1]) for i in range(len(self.segments))]

    @property
    def start_pose(self) -> Pose3D:
        return self.segments[0].start_pose

    @property
    def end_pose(self) -> Pose3D:
        return self.segments[-1].end_pose

    @property
    def min_radius_um(self) -> float:
        return min(seg.min_radius_um for seg in self.segments)

    def point_at(self, s: float) -> Vec3:
        spans = self.spans
        if s <= 0.0:
            return self.segments[0].point_at(0.0)
        if s >= spans[-1][1]:
            return self.segments[-1].point_at(self.segments[-1].length)
        for i, (a, b) in enumerate(spans):
            if a <= s <= b:
                return self.segments[i].point_at(s - a)
        raise AssertionError("unreachable")

    def tangent_at(self, s: float) -> Vec3:
        spans = self.spans
        if s <= 0.0:
            return self.segments[0].tangent_at(0.0)
        if s >= spans[-1][1]:
            return self.segments[-1].tangent_at(self.segments[-1].length)
        for i, (a, b) in enumerate(spans):
            if a <= s <= b:
                return self.segments[i].tangent_at(s - a)
        raise AssertionError("unreachable")

    def sample(self, n_per_segment: int = 32) -> List[Vec3]:
        pts: List[Vec3] = []
        for seg in self.segments:
            sp = seg.sample(n_per_segment)
            if pts and distance(pts[-1], sp[0]) < 1e-12:
                sp = sp[1:]
            pts.extend(sp)
        return pts

    def continuity_report(self, pos_tol_um: float = 1e-6,
                          ang_tol_deg: float = 1e-6,
                          curvature_tol: float = 1e-9) -> Dict:
        """Position/tangent continuity plus separately reported features.

        non-C1 corners   : junction tangent angle > ang_tol -> illegal corner
        curvature jumps  : |k_before - k_after| > curvature_tol -> recorded but
                           NOT a C1 violation (line->arc always has one).
        """
        pos_err: List[float] = []
        ang_err_deg: List[float] = []
        corners: List[Dict] = []
        curv_jumps: List[Dict] = []
        for i in range(len(self.segments) - 1):
            a = self.segments[i]
            b = self.segments[i + 1]
            pa, pb = a.end_pose, b.start_pose
            pe = distance(pa.position, pb.position)
            ae = math.degrees(angle_between(pa.tangent, pb.tangent))
            pos_err.append(pe)
            ang_err_deg.append(ae)
            if pe > pos_tol_um:
                corners.append({"junction": i, "type": "position_gap",
                                "position_error_um": pe})
            if ae > ang_tol_deg:
                corners.append({"junction": i, "type": "tangent_discontinuity",
                                "tangent_error_deg": ae})
            ka, kb = a.curvature_at(a.length), b.curvature_at(0.0)
            if abs(ka - kb) > curvature_tol:
                curv_jumps.append({"junction": i,
                                   "kappa_before_1_per_um": ka,
                                   "kappa_after_1_per_um": kb,
                                   "delta_1_per_um": kb - ka,
                                   "note": "curvature discontinuity, not a C1 violation"})
        return {
            "n_segments": len(self.segments),
            "total_length_um": self.total_length,
            "position_errors_um": pos_err,
            "tangent_errors_deg": ang_err_deg,
            "max_position_error_um": max(pos_err) if pos_err else 0.0,
            "max_tangent_error_deg": max(ang_err_deg) if ang_err_deg else 0.0,
            "c1_continuous": not corners,
            "corners": corners,
            "curvature_jumps": curv_jumps,
            "min_radius_um": self.min_radius_um,
        }

    def to_dict(self) -> dict:
        return {
            "total_length_um": self.total_length,
            "min_radius_um": self.min_radius_um,
            "segments": [seg.to_dict() for seg in self.segments],
            "cumulative_lengths_um": self.cumulative_lengths,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Path3D":
        segs = []
        for sd in d["segments"]:
            if sd["kind"] == "line":
                segs.append(LineSegment3D.from_dict(sd))
            elif sd["kind"] == "arc":
                segs.append(CircularArc3D.from_dict(sd))
            else:
                raise ValueError("unknown segment kind %r" % (sd.get("kind"),))
        return cls(tuple(segs))
