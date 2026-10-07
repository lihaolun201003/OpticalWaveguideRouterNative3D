"""The v0.1.3 status vocabulary of the native 3D search strategies.

WHY THIS MODULE EXISTS
----------------------
v0.1.2 used almost the same word for very different events.  In particular the
finite production router relabelled a failed ASSEMBLED-PATH constraint check as
``search_exhausted`` (see the v0.1.2 line

    res.status = "uncertified" if undecided else "search_exhausted"

in search/finite_graph.py), which is the opposite of what had happened: OPEN was
not empty, the search had found a path and the PATH was rejected.  A reader of
the artifact could not tell the two apart.  This module is the single place that
defines what each status means, and the tests assert that the router really uses
distinct ones.

THE STATUSES (all distinct, none is an alias of another)
--------------------------------------------------------
success
    A legal result was produced and verified.  Either the returned path really
    starts and ends on the caller's source/target pose within the documented
    tolerances, or the solution is the explicitly defined ZERO-LENGTH solution
    (``zero_length_solution is True``, no primitive, ``path is None``).
search_exhausted
    The FINITE graph reachable from the source was completely expanded and OPEN
    became empty without reaching the goal set.  This is a real exhaustion of
    the CONSTRUCTED graph, never a budget stop and never a statement about the
    continuous pose space or about physical reachability.
budget_exhausted
    ``max_expansions`` or ``max_runtime_s`` was hit.  Deliberately distinct from
    search_exhausted: a budget stop proves nothing about reachability.
port_attachment_failed
    No validated attachment curve connects a port pose (source or target) to any
    canonical node inside the generic attachment neighbourhood.  A statement
    about the SUPPORTED ACTION SET (six axis directions, quarter turns of radius
    m*h, the coplanar connector family), never about physical reachability.
graph_scene_mismatch
    A caller-supplied FiniteGraph does not match the scene/config/mode/congestion
    actually passed to the router.  Refused BEFORE any search: no goal node, no
    lattice anchor, no edge of that graph may be used for this input.  This is a
    contract error, not a search outcome.
invalid_source_pose
    The source pose itself is illegal (outside the chip volume shrunk by the
    boundary margin, inside an obstacle keep-out, or on a fixed reference
    centreline).  Checked BEFORE the search, so it is not bypassed by
    source == target.
invalid_target_pose
    Same, for the target pose.
unsupported_pose
    The pose is legal but no action of the supported set can start from it.
constraint_failed
    The search produced a path, but the ASSEMBLED continuous curve violates a
    HARD constraint (chip boundary, obstacle keep-out, reference-centreline
    spacing, minimum bend radius) or fails the actual-endpoint/port verification
    against the pose the caller passed.  Never reported as success, never
    reported as search_exhausted.
uncertified
    A check could not be DECIDED inside its documented refinement cap (an
    ``undecided`` constraint result), or a certification self check failed.  An
    undecided check is never counted as a pass.

SEARCH TERMINATIONS
-------------------
``SEARCH_TERMINATIONS`` lists the statuses that are outcomes OF THE A* ITSELF
(the search ran to one of its own stopping conditions).  Every other status is
an INPUT or CONTRACT decision taken before or after the search, which is exactly
the distinction v0.1.2 blurred.
"""

from __future__ import annotations

SUCCESS = "success"
SEARCH_EXHAUSTED = "search_exhausted"
BUDGET_EXHAUSTED = "budget_exhausted"
PORT_ATTACHMENT_FAILED = "port_attachment_failed"
GRAPH_SCENE_MISMATCH = "graph_scene_mismatch"
INVALID_SOURCE_POSE = "invalid_source_pose"
INVALID_TARGET_POSE = "invalid_target_pose"
UNSUPPORTED_POSE = "unsupported_pose"
CONSTRAINT_FAILED = "constraint_failed"
UNCERTIFIED = "uncertified"

# Ordered so that a report can print the vocabulary canonically.
ALL_STATUSES = (
    SUCCESS,
    SEARCH_EXHAUSTED,
    BUDGET_EXHAUSTED,
    PORT_ATTACHMENT_FAILED,
    GRAPH_SCENE_MISMATCH,
    INVALID_SOURCE_POSE,
    INVALID_TARGET_POSE,
    UNSUPPORTED_POSE,
    CONSTRAINT_FAILED,
    UNCERTIFIED,
)

# The statuses astar_search itself can return (see search/astar.py).
SEARCH_TERMINATIONS = (SUCCESS, SEARCH_EXHAUSTED, BUDGET_EXHAUSTED)

# Statuses that are decided BEFORE any search expansion.
PRE_SEARCH_STATUSES = (GRAPH_SCENE_MISMATCH, INVALID_SOURCE_POSE,
                       INVALID_TARGET_POSE, PORT_ATTACHMENT_FAILED,
                       UNSUPPORTED_POSE)

# Statuses that mean "the search ran but the result is not a verified success".
POST_SEARCH_FAILURE_STATUSES = (CONSTRAINT_FAILED, UNCERTIFIED)

__all__ = ["SUCCESS", "SEARCH_EXHAUSTED", "BUDGET_EXHAUSTED",
           "PORT_ATTACHMENT_FAILED", "GRAPH_SCENE_MISMATCH",
           "INVALID_SOURCE_POSE", "INVALID_TARGET_POSE", "UNSUPPORTED_POSE",
           "CONSTRAINT_FAILED", "UNCERTIFIED", "ALL_STATUSES",
           "SEARCH_TERMINATIONS", "PRE_SEARCH_STATUSES",
           "POST_SEARCH_FAILURE_STATUSES"]
