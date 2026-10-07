"""Phase 2: native 3D search state = (x, y, z, direction-as-3D-unit-vector).

direction is a genuine unit 3D vector, never a 2D orientation plus a layer
label.  The state is exactly the pose: every motion primitive of Phase 3 is a
function of the pose alone (straight / yaw / pitch / skew arcs need nothing
else; the terminal connector additionally needs the fixed target pose), so no
further history variable is required for feasibility.  The ONE feasibility
quantity that would need history is path self-approach spacing; that is
reported as a limitation and checked post-hoc by the independent validator.

STATE IDENTITY (v0.1.2)
-----------------------
Two identities are available and the difference matters:

* pose_key() - the DEFAULT identity.  It is the identity of the stored pose
  itself: the position triple and the direction triple are used as they are,
  as floats, with NO rounding, no quantisation and no tolerance.  Two states
  share an identity if and only if they are the same stored pose.

  Why the default changed in v0.1.2: v0.1.1 used key_exact(), which rounded the
  whole pose to 9 decimals and was documented as "safe".  That was still an
  approximation: round(10.0000000002, 9) == round(10.0, 9), so two poses
  2e-10 um apart shared a key, the search kept only the cheaper one and deleted
  the other - and if only the deleted pose had a viable successor the graph was
  reported exhausted.  A tolerance far below Rmin proves nothing about the
  successor set, because the successor set is a function of the exact pose, not
  of a bucket.  See tests/test_v012_a1_pose_identity.py and the real pre-fix
  replay in artifacts/v012/logs/00_prefix_v012_defect_replay.log.

  What pose_key() does NOT give: the enumerated graph is generally infinite
  (positions are real numbers), so a run can end in budget_exhausted and a
  budget stop is never a proof of unreachability.  A rounding-free floating
  point identity is also NOT a complete search of the continuous pose space:
  only the poses reachable through a finite sequence of the configured
  primitives are enumerated, the identity is the identity of THIS program's
  floats, and the budget still truncates.  No completeness claim is made.

  Cost note: because two different real poses are never merged, the same
  geometric pose can in principle be reached through different floating point
  evaluation orders and then appears as two nodes.  That is a completeness /
  size cost, never a correctness cost: no pose is ever deleted.  The finite
  canonical graph of search/finite_graph.py exists exactly because of this:
  there the identity is an INTEGER node id, so the node set is finite by
  construction and the search can really be exhausted.

* key(pos_res_um, dir_res_deg) - the v0.1 QUANTISED key.  It merges every pose
  inside a cell of pos_res_um / dir_res_deg, even though the successors, the
  collision checks and the terminal connector are still computed from the real
  pose.  That merging is NOT sound: two poses in one cell are different states,
  and keeping only the lowest-g one can delete the only pose with a viable
  successor (tests/test_repair_b_state_identity.py).  It is retained only as an
  explicitly APPROXIMATE exploration mode
  (RouterConfig.state_identity = "approximate_quantised"); exhausting it proves
  nothing about the continuous space and must never be reported as
  unreachability.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from ..geometry.primitives import Pose3D
from ..geometry.vec import Vec3
from ..identity import (APPROXIMATE_STATE_IDENTITY,                 # noqa: F401
                        LEGACY_STATE_IDENTITY_ALIASES,
                        POSE_FLOAT_STATE_IDENTITY, STATE_IDENTITIES,
                        normalise_state_identity)


@dataclass(frozen=True)
class State3D:
    position: Vec3
    direction: Vec3

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", tuple(float(c) for c in self.position))
        d = tuple(float(c) for c in self.direction)
        n = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
        if not math.isfinite(n) or n <= 1e-12:
            raise ValueError("State3D direction must be a non-zero 3D vector")
        object.__setattr__(self, "direction", (d[0] / n, d[1] / n, d[2] / n))

    @property
    def pose(self) -> Pose3D:
        return Pose3D(self.position, self.direction)

    @property
    def x(self) -> float:
        return self.position[0]

    @property
    def y(self) -> float:
        return self.position[1]

    @property
    def z(self) -> float:
        return self.position[2]

    @property
    def azimuth_deg(self) -> float:
        return math.degrees(math.atan2(self.direction[1], self.direction[0]))

    @property
    def elevation_deg(self) -> float:
        return math.degrees(math.asin(max(-1.0, min(1.0, self.direction[2]))))

    def pose_key(self) -> Tuple[Vec3, Vec3]:
        """DEFAULT state identity: the stored pose itself, with no rounding.

        The returned tuple is a hashable value; equality of two keys means the
        two states hold bit-identical position and direction floats, i.e. they
        are the same pose.  Nothing is merged by tolerance here.
        """
        return (self.position, self.direction)

    def key(self, pos_res_um: float, dir_res_deg: float) -> Tuple[int, int, int, int, int]:
        """v0.1 QUANTISED key: APPROXIMATE exploration only.

        Position AND direction are part of the key (never position alone), but
        poses in one cell are NOT the same state and pruning between them is
        unsound: it can delete the only pose with a viable successor.
        """
        px = int(round(self.position[0] / pos_res_um))
        py = int(round(self.position[1] / pos_res_um))
        pz = int(round(self.position[2] / pos_res_um))
        az = self.azimuth_deg % 360.0
        n_az = max(1, int(round(360.0 / dir_res_deg)))
        ka = int(round(az / dir_res_deg)) % n_az
        ke = int(round(self.elevation_deg / dir_res_deg))
        return (px, py, pz, ka, ke)

    def to_dict(self) -> dict:
        return {"position": list(self.position), "direction": list(self.direction),
                "azimuth_deg": self.azimuth_deg, "elevation_deg": self.elevation_deg}

    @classmethod
    def from_pose(cls, pose: Pose3D) -> "State3D":
        return cls(pose.position, pose.tangent)
