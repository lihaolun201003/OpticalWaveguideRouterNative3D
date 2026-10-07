"""Native3D v0.1.2 - single-net native three-dimensional waveguide router.

v0.1.2 closes the residual correctness defects of v0.1.1 (state identity, source
pose validation, reference arcs, the sagitta domain of multi-turn arcs, and the
near-parallel segment distance) and adds the finite multi-hop native 3D
production router (search/finite_graph.py) with integer node ids.  Geometry only:
no optical loss model, no dB, no crosstalk.
"""

__version__ = "0.1.2"
