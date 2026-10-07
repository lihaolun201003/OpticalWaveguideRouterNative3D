"""Search package: state, 3D motion primitives, A* and the three strategies.

v0.1.2 keeps three explicitly separated strategies:

* route()                    - continuous-pose debug search (this is the v0.1
                               engine; its enumerated graph is generally
                               infinite, so budget_exhausted is normal there)
* reference_search()         - the v0.1.1 finite two-hop reference graph
* route_finite_native3d()    - the v0.1.2 PRODUCTION finite multi-hop native
                               3D router over integer lattice node ids
"""
from .state import State3D
from .actions import Action, generate_actions
from .astar import SearchResult, astar_search, g_improves, g_tolerance
from .congestion import CongestionField, ZeroCongestion, RegionCongestion
from .connector import planar_connectors, connector_actions
from .router import STRATEGIES, route
from .reference_graph import reference_search
from .finite_graph import (AXIS_DIRECTIONS, FiniteGraph, enumerate_graph_edges,
                           route_finite_native3d)

__all__ = ["State3D", "Action", "generate_actions", "SearchResult", "astar_search",
           "g_improves", "g_tolerance", "CongestionField", "ZeroCongestion",
           "RegionCongestion", "planar_connectors", "connector_actions", "route",
           "STRATEGIES", "reference_search", "route_finite_native3d", "FiniteGraph",
           "enumerate_graph_edges", "AXIS_DIRECTIONS"]
