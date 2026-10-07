"""Sequential fixed-occupancy publication for the v0.2 multi-channel baseline.

WHAT IS PUBLISHED
-----------------
When a net is verified, EVERY ACTUAL PRIMITIVE of its returned path is frozen and
published: the real ``LineSegment3D`` / ``CircularArc3D`` objects, with their full
geometric content.  Never a node id, never a control point, never a polyline of
chords and never an AABB.  Each published curve records

    net_id, segment_id, occupancy_revision, kind, geometry

so a reader of the JSON can attribute every occupied curve to a net and to the
revision at which it became occupied.  The static (pre-existing) reference
centrelines of the case are carried in the SAME geometry list - because every net
must keep the hard centreline spacing from them too - but with the explicit
origin ``static_reference_centerline`` and with ``net_id = None``.  They are
never attributed to a net of this run and are never mistaken for a path.

RULES ENFORCED HERE (not documented but actually checked)
--------------------------------------------------------
1. A net may be published AT MOST ONCE, and only through ``publish`` with
   ``verified=True``.  A failed or unverified path cannot be published: the call
   raises ``OccupancyError`` instead of silently adding geometry.
2. A published curve is never mutated afterwards.  Publication stores a deep copy
   plus a content fingerprint; ``verify_immutability()`` re-reads every stored
   curve's content and reports any change.  ``scene_for_net`` hands the next net
   deep copies again, so a net cannot reach into the occupancy either.
3. Publication is monotone: ``publish`` bumps the revision and never removes or
   edits an existing curve.  There is no implicit rip-up & reroute in v0.2.
4. Nothing here elevates a 2D path: the published objects are exactly the 3D
   primitives the 3D search returned.
"""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..io.caseio import Scene


class OccupancyError(RuntimeError):
    """A publication was attempted that the v0.2 publication rules forbid."""


STATIC_ORIGIN = "static_reference_centerline"
PUBLISHED_ORIGIN = "committed_net_path"


def _geometry_dict(seg) -> dict:
    d = seg.to_dict()
    if d.get("kind") not in ("line", "arc"):
        raise OccupancyError("only line/arc primitives can be occupied; got %r"
                             % (d.get("kind"),))
    return d


def geometry_fingerprint(seg) -> str:
    """Content hash of one primitive (nothing about the object identity)."""
    payload = json.dumps(_geometry_dict(seg), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:32]


@dataclass(frozen=True)
class OccupancyEntry:
    """One occupied curve, with its provenance and revision."""

    origin: str
    net_id: Optional[str]
    segment_id: Optional[int]
    occupancy_revision: int
    kind: str
    length_um: float
    fingerprint: str
    geometry: dict

    def to_dict(self) -> dict:
        return {"origin": self.origin,
                "net_id": self.net_id,
                "segment_id": self.segment_id,
                "occupancy_revision": self.occupancy_revision,
                "kind": self.kind,
                "length_um": self.length_um,
                "fingerprint": self.fingerprint,
                "geometry": self.geometry}


class Occupancy:
    """The frozen-occupancy store of one sequential multi-net run."""

    def __init__(self, static_reference_centerlines: Sequence[object] = (),
                 dmin_centerline_um: Optional[float] = None) -> None:
        self._entries: List[OccupancyEntry] = []
        self._curves: List[object] = []
        self._published_net_ids: List[str] = []
        self._per_net_counts: Dict[str, int] = {}
        self.dmin_centerline_um = dmin_centerline_um
        self.revision = 0
        # static, pre-existing reference centrelines: occupied for everybody,
        # owned by nobody.  Revision 0 = "before any net was published".
        for k, seg in enumerate(static_reference_centerlines):
            self._entries.append(OccupancyEntry(
                origin=STATIC_ORIGIN, net_id=None, segment_id=k,
                occupancy_revision=0, kind=seg.kind, length_um=float(seg.length),
                fingerprint=geometry_fingerprint(seg),
                geometry=_geometry_dict(seg)))
            self._curves.append(copy.deepcopy(seg))

    # ------------------------------------------------------------------ access
    @property
    def entries(self) -> Tuple[OccupancyEntry, ...]:
        return tuple(self._entries)

    @property
    def published_net_ids(self) -> Tuple[str, ...]:
        return tuple(self._published_net_ids)

    def curves(self) -> List[object]:
        """Deep copies of every occupied curve: static first, then published.

        The copies are what a new per-net Scene is built from, so no net can
        reach a published object even through a mutable attribute.
        """
        return [copy.deepcopy(seg) for seg in self._curves]

    def published_curves(self) -> List[object]:
        return [copy.deepcopy(seg) for e, seg in zip(self._entries, self._curves)
                if e.origin == PUBLISHED_ORIGIN]

    def published_entries(self) -> List[OccupancyEntry]:
        return [e for e in self._entries if e.origin == PUBLISHED_ORIGIN]

    def static_entries(self) -> List[OccupancyEntry]:
        return [e for e in self._entries if e.origin == STATIC_ORIGIN]

    def counts(self) -> Dict:
        return {"n_curves": len(self._entries),
                "n_static_reference_curves": len(self.static_entries()),
                "n_published_curves": len(self.published_entries()),
                "per_net_published_curve_counts": dict(self._per_net_counts)}

    # -------------------------------------------------------------- publication
    def publish(self, net_id: str, segments: Sequence[object], verified: bool,
                revision_note: str = "") -> int:
        """Freeze a VERIFIED net's actual primitives and bump the revision.

        Returns the new occupancy revision.  Raises ``OccupancyError`` when the
        path was not verified, is empty, or the net was already published.
        """
        if not verified:
            raise OccupancyError(
                "refusing to publish %r: only a verified path may be published "
                "(a failed or unverified path must never enter the occupancy)"
                % (net_id,))
        if net_id in self._per_net_counts:
            raise OccupancyError(
                "refusing to publish %r a second time: the v0.2 baseline never "
                "modifies or re-publishes an already published net" % (net_id,))
        segs = list(segments)
        if not segs:
            raise OccupancyError("refusing to publish %r: the path has no "
                                 "primitive" % (net_id,))
        self.revision += 1
        rev = self.revision
        for sid, seg in enumerate(segs):
            geo = _geometry_dict(seg)
            self._entries.append(OccupancyEntry(
                origin=PUBLISHED_ORIGIN, net_id=net_id, segment_id=sid,
                occupancy_revision=rev, kind=seg.kind,
                length_um=float(seg.length),
                fingerprint=geometry_fingerprint(seg), geometry=geo))
            self._curves.append(copy.deepcopy(seg))
        self._published_net_ids.append(net_id)
        self._per_net_counts[net_id] = len(segs)
        return rev

    # ------------------------------------------------------------ immutability
    def verify_immutability(self) -> Dict:
        """Re-read every stored curve and prove nothing was changed."""
        changed: List[dict] = []
        for idx, (entry, seg) in enumerate(zip(self._entries, self._curves)):
            fp = geometry_fingerprint(seg)
            live = _geometry_dict(seg)
            if fp != entry.fingerprint or live != entry.geometry:
                changed.append({"index": idx, "origin": entry.origin,
                                "net_id": entry.net_id,
                                "segment_id": entry.segment_id,
                                "published_fingerprint": entry.fingerprint,
                                "observed_fingerprint": fp})
        return {"ok": not changed, "n_curves": len(self._entries),
                "n_changed": len(changed), "changed": changed,
                "rule": ("a published curve is frozen: neither a later net nor "
                         "the next scene assembly may modify it")}

    def to_dict(self) -> dict:
        d = self.counts()
        d["revision"] = self.revision
        d["published_net_ids"] = list(self._published_net_ids)
        d["entries"] = [e.to_dict() for e in self._entries]
        d["immutability"] = self.verify_immutability()
        d["publication_rule"] = (
            "published curves are the ACTUAL primitives of a VERIFIED net path: "
            "net_id + segment_id + occupancy_revision + full geometry; nodes, "
            "control points, chord polylines and AABBs are never the occupancy")
        return d


def scene_for_net(case, net, occupancy: Occupancy, suffix: str = "") -> Scene:
    """A brand new Scene for ONE net: case geometry + the frozen occupancy.

    Independently deep-copied inputs and a deep-copied RouterConfig, so the
    search of this net shares no mutable object with the case, with another net,
    or with the occupancy.  The static reference centrelines come first (they are
    fixed pre-existing waveguides), then every published curve of the earlier
    nets, in publication order.
    """
    static = [copy.deepcopy(s) for s in case.reference_centerlines]
    published = occupancy.published_curves()
    cfg = copy.deepcopy(case.config)
    name = "%s::%s%s" % (case.name, net.net_id, suffix)
    return Scene(
        name=name,
        chip=copy.deepcopy(case.chip),
        obstacles=[copy.deepcopy(b) for b in case.obstacles],
        reference_centerlines=static + published,
        source=copy.deepcopy(net.source),
        target=copy.deepcopy(net.target),
        config=cfg,
        metadata={
            "multinet_case": case.name,
            "net_id": net.net_id,
            "occupancy_revision": occupancy.revision,
            "n_static_reference_centerlines": len(static),
            "n_published_curves": len(published),
            "n_reference_centerlines_total": len(static) + len(published),
            "reference_centerlines_are_static_plus_published_occupancy": True,
            "static_reference_centerlines_are_not_a_net_path": True,
            "net_metadata": dict(net.metadata),
        })
