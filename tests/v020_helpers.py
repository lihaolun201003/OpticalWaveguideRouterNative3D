"""Shared helpers for the v0.2 multi-channel tests.

The tests here are deliberately SMALL: they exercise the v0.2 layer
(multinet/occupancy/self_spacing/inter_net/sequential and the independent joint
validator) on the smallest interesting geometries, so the suite stays usable.  The
big 2/4/8/16 runs live in examples/run_multichannel_baseline.py and their real
numbers are quoted in the v0.2 report.
"""

from __future__ import annotations

import json
import math
import os
import sys

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SRC = os.path.join(_ROOT, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from native3d.config import RouterConfig                     # noqa: E402
from native3d.constraints.checks import Box3D                # noqa: E402
from native3d.geometry.primitives import (CircularArc3D,     # noqa: E402
                                          LineSegment3D, Pose3D)
from native3d.multinet.models import SelfSpacingPolicy       # noqa: E402
from native3d.multinet.cases_v020 import v020_config         # noqa: E402
from native3d.validation import joint_validate as jv         # noqa: E402

DEG = math.pi / 180.0


def cfg(**over) -> RouterConfig:
    return v020_config(**over)


def policy(cfg_) -> SelfSpacingPolicy:
    return SelfSpacingPolicy(ell_local_um=2.0 * cfg_.dmin_centerline_um,
                             kind="two_times_dmin")


def line(p0, p1) -> LineSegment3D:
    return LineSegment3D(tuple(float(c) for c in p0), tuple(float(c) for c in p1))


def arc(center, u, v, R, theta) -> CircularArc3D:
    return CircularArc3D(tuple(float(c) for c in center),
                         tuple(float(c) for c in u), tuple(float(c) for c in v),
                         float(R), float(theta))


def quarter_arc(center, u, v, R=50.0) -> CircularArc3D:
    return arc(center, u, v, R, 90.0 * DEG)


def full_circle(center, R=50.0) -> CircularArc3D:
    return arc(center, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), R, 2.0 * math.pi)


def json_roundtrip(obj):
    """Through JSON, so a test can never rely on object identity."""
    return json.loads(json.dumps(obj))


def as_payload(chip, obstacles, static_refs, cfg_, nets, case_name="t"):
    """Build the joint-validator payload by hand (no scheduler involved)."""
    return {
        "artifact": "native3d_v020_multinet_case",
        "units": "um",
        "case": case_name,
        "channel_count": len(nets),
        "order": [n["net_id"] for n in nets],
        "chip_volume": chip.to_dict(),
        "obstacles": [b.to_dict() for b in obstacles],
        "static_reference_centerlines": [s.to_dict() for s in static_refs],
        "config": cfg_.to_dict(),
        "self_spacing_policy": policy(cfg_).to_dict(),
        "nets": nets,
        "occupancy": {"revision": 0, "entries": []},
        "counts": {},
        "totals": {},
    }


def _as_dict(s):
    return s if isinstance(s, dict) else s.to_dict()


def _length_of(s):
    if isinstance(s, dict):
        return float(s.get("length_um", 0.0))
    return float(s.length)


def net_entry(net_id, source, target, segments, status="committed",
              accepted=True, committed=True):
    segs = list(segments)
    return {"net_id": net_id, "index": 0,
            "input": {"net_id": net_id, "source_pose": source.to_dict(),
                      "target_pose": target.to_dict(), "metadata": {}},
            "status": status, "accepted": accepted, "search_status": "success",
            "length_um": sum(_length_of(s) for s in segs),
            "expansions": 0, "occupancy_revision_before": 0,
            "occupancy_revision_after": 1, "committed": committed,
            "committed_segment_ids": list(range(len(segs))) if committed else [],
            "path": None,
            "segments": [_as_dict(s) for s in segs]}


def occupancy_entry(net_id, segment_id, seg, revision=1):
    return {"origin": "committed_net_path", "net_id": net_id,
            "segment_id": segment_id, "occupancy_revision": revision,
            "kind": seg.kind, "length_um": float(seg.length),
            "fingerprint": "manual-test", "geometry": seg.to_dict()}


# The session fixtures used by the v0.2 test modules are defined in tests/conftest.py
# (a fixture imported from a helper module is NOT registered by pytest).

