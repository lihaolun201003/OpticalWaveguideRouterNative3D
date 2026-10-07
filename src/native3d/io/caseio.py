"""Case input/output contract (JSON, micrometres, right-handed x/y/z).

Scene = chip volume + solid obstacles + FIXED pre-existing waveguide centrelines
+ source pose + target pose + RouterConfig.  Everything the router is allowed to
know is in this file.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from ..config import RouterConfig
from ..constraints.checks import Box3D
from ..geometry.primitives import CircularArc3D, LineSegment3D, Pose3D
from ..geometry.vec import Vec3


@dataclass
class Scene:
    name: str
    chip: Box3D
    obstacles: List[Box3D]
    reference_centerlines: List[object]
    source: Pose3D
    target: Pose3D
    config: RouterConfig = field(default_factory=RouterConfig)
    metadata: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "units": "um",
            "chip_volume": self.chip.to_dict(),
            "obstacles": [b.to_dict() for b in self.obstacles],
            "reference_centerlines": [s.to_dict() for s in self.reference_centerlines],
            "source_pose": self.source.to_dict(),
            "target_pose": self.target.to_dict(),
            "config": self.config.to_dict(),
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Scene":
        refs = []
        for sd in d.get("reference_centerlines", []):
            refs.append(LineSegment3D.from_dict(sd) if sd["kind"] == "line"
                        else CircularArc3D.from_dict(sd))
        return cls(name=d["name"],
                   chip=Box3D.from_dict(d["chip_volume"]),
                   obstacles=[Box3D.from_dict(b) for b in d.get("obstacles", [])],
                   reference_centerlines=refs,
                   source=Pose3D.from_dict(d["source_pose"]),
                   target=Pose3D.from_dict(d["target_pose"]),
                   config=RouterConfig.from_dict(d.get("config", {})),
                   metadata=d.get("metadata", {}))

    def summary(self) -> dict:
        return {
            "name": self.name,
            "units": "um",
            "chip_volume": self.chip.to_dict(),
            "n_obstacles": len(self.obstacles),
            "obstacles": [b.to_dict() for b in self.obstacles],
            "n_reference_centerlines": len(self.reference_centerlines),
            "source_pose": self.source.to_dict(),
            "target_pose": self.target.to_dict(),
            "constraints": {
                "rmin_um": self.config.rmin_um,
                "dmin_centerline_um": self.config.dmin_centerline_um,
                "obstacle_clearance_um": self.config.obstacle_clearance_um,
                "obstacle_clearance_includes_envelope": self.config.obstacle_clearance_includes_envelope,
                "effective_obstacle_clearance_um": self.config.effective_obstacle_clearance_um,
                "wg_envelope_radius_um": self.config.wg_envelope_radius_um,
                "effective_boundary_margin_um": self.config.effective_boundary_margin_um,
                "eps_pos_um": self.config.eps_pos_um,
                "eps_ang_deg": self.config.eps_ang_deg,
            },
        }


def write_json(path: str, payload: dict) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=False)
    return os.path.abspath(path)


def read_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)
