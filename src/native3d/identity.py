"""State identity names shared by the configuration and the search state.

This module has no dependencies on purpose: the configuration validates the
identity name and the search state implements it, so the names must not live in
either of them.

v0.1.2 identity policy
----------------------
* "pose_float" (default): the identity of the stored pose itself.  The position
  triple and the direction triple are used as they are, as floats, with no
  rounding, no quantisation and no tolerance.
* "approximate_quantised": the v0.1 cell key; explicitly APPROXIMATE exploration
  only, its exhaustion is not a proof of unreachability.
* "exact": a v0.1.1 READ ALIAS only.  v0.1.1 used the name "exact" for a pose
  rounded to 1e-9 um, which was still an approximation that could merge two
  poses with different successors.  The old name resolves to "pose_float" so
  that stored v0.1.1 configuration files still load; it is NOT a separate mode
  and it no longer rounds anything.
"""

from __future__ import annotations

POSE_FLOAT_STATE_IDENTITY = "pose_float"
APPROXIMATE_STATE_IDENTITY = "approximate_quantised"
STATE_IDENTITIES = (POSE_FLOAT_STATE_IDENTITY, APPROXIMATE_STATE_IDENTITY)
LEGACY_STATE_IDENTITY_ALIASES = {"exact": POSE_FLOAT_STATE_IDENTITY}


def normalise_state_identity(name: str) -> str:
    """Resolve a configured identity name (including the v0.1.1 read alias)."""
    return LEGACY_STATE_IDENTITY_ALIASES.get(name, name)


__all__ = ["POSE_FLOAT_STATE_IDENTITY", "APPROXIMATE_STATE_IDENTITY",
           "STATE_IDENTITIES", "LEGACY_STATE_IDENTITY_ALIASES",
           "normalise_state_identity"]
