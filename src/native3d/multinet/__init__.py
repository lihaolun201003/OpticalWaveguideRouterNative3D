"""Native3D v0.2 multi-channel sequential routing baseline.

This package is the v0.2 layer that turns the verified v0.1.3 SINGLE-net finite
3D router into a working multi-net SEQUENTIAL baseline.  It deliberately does
NOT touch the geometry kernel or the search architecture:

* one net at a time, in the input order;
* every net is searched with a freshly constructed finite graph (the ``graph=``
  argument of ``route_finite_native3d`` is never passed), on its own deep-copied
  Scene / RouterConfig snapshot;
* the ONLY coupling between nets is the published occupancy: the actual
  straight/arc primitives of an already verified net enter the NEXT net's Scene
  as fixed reference centrelines, so the existing hard
  ``centerline_spacing`` constraint enforces the keep-out during the search;
* the congestion penalty is exactly zero and is never a substitute for the hard
  occupancy constraint.

Nothing here is a proof about the continuous pose space, and no optical quantity
(loss, dB, crosstalk) is produced anywhere.
"""

from __future__ import annotations

from .inter_net import inter_net_spacing_check, pair_spacing_certified
from .models import (MULTINET_ACCEPTED_STATUSES, MULTINET_FAILURE_STATUSES,
                     MultiNetCase, MultiNetResult, NetOutcome, NetSpec,
                     SelfSpacingPolicy, default_self_spacing_policy)
from .occupancy import Occupancy, OccupancyEntry, OccupancyError, scene_for_net
from .self_spacing import path_self_spacing_global
from .sequential import joint_validation_payload, route_multinet

__all__ = [
    "MultiNetCase", "NetSpec", "NetOutcome", "MultiNetResult",
    "SelfSpacingPolicy", "default_self_spacing_policy",
    "MULTINET_ACCEPTED_STATUSES", "MULTINET_FAILURE_STATUSES",
    "Occupancy", "OccupancyEntry", "OccupancyError", "scene_for_net",
    "path_self_spacing_global",
    "inter_net_spacing_check", "pair_spacing_certified",
    "route_multinet", "joint_validation_payload",
]
