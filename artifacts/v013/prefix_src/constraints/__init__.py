"""Hard constraints package."""
from .checks import (MAX_CHORD_SWEEP_RAD, SEGMENT_DISTANCE_ERROR_FLOOR_UM,
                     Box3D, CheckResult, arc_box_clearance, boundary_check,
                     chord_count_for_domain, line_box_clearance,
                     pair_clearance_bounds, pair_clearance_lb_ub,
                     path_self_spacing, point_box_distance, pose_match,
                     primitive_box_clearance, primitive_spacing, radius_check,
                     sagitta_bound_um, segment_box_distance,
                     segment_segment_bounds, segment_segment_distance,
                     segment_segment_distance_certified, validate_segments)

__all__ = ["Box3D", "CheckResult", "validate_segments", "boundary_check",
           "radius_check", "pose_match", "primitive_spacing",
           "path_self_spacing", "segment_box_distance", "segment_segment_distance",
           "segment_segment_distance_certified", "segment_segment_bounds",
           "point_box_distance", "arc_box_clearance", "line_box_clearance",
           "primitive_box_clearance", "pair_clearance_lb_ub",
           "pair_clearance_bounds", "chord_count_for_domain", "sagitta_bound_um",
           "MAX_CHORD_SWEEP_RAD", "SEGMENT_DISTANCE_ERROR_FLOOR_UM"]
