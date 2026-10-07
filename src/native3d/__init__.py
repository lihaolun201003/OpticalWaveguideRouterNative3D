"""Native3D v0.1.3 - single-net native three-dimensional waveguide router.

v0.1.3 keeps every v0.1.2 result (the finite multi-hop integer-node production
router, the six axis directions, the certified continuous constraint stack) and
closes the residual PUBLIC-CONTRACT defects that v0.1.2 could still report as a
false success:

* a mismatched prebuilt graph is now refused BEFORE the search
  (status ``graph_scene_mismatch``) instead of returning the OLD target as a
  success (search/finite_graph.py);
* ``success`` is decided by the ACTUAL assembled curve: every accepted edge must
  end on its canonical node pose and the returned path must really start and end
  on the poses the CALLER passed, otherwise the status is ``constraint_failed``
  or ``uncertified`` - never ``search_exhausted``;
* the zero-length solution is a first-class, documented result
  (``status=success``, ``zero_length_solution=True``, ``cost=length=0``,
  ``path=None``, ``n_segments=0``) instead of an AttributeError;
* the independent validator's geometry is closed: the 3D point-to-planar-arc
  distance, the strict JSON pose contract and the circular-arc basis contract
  (a near-unit basis is rejected rather than certified as a circle).

Geometry only: no optical loss model, no dB, no crosstalk.
"""

__version__ = "0.1.3"
