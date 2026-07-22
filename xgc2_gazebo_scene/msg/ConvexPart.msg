uint8 SHAPE_BOX=1
uint8 SHAPE_SPHERE=2
uint8 SHAPE_CYLINDER=3

string part_id
uint8 shape
geometry_msgs/Pose local_pose

# Exact primitive parameters. Box uses size; sphere uses radius; cylinder uses
# radius and length. Planners should prefer these analytical values when able.
geometry_msgs/Vector3 size
float64 radius
float64 length

# Conservative local-frame convex vertices. Box vertices are exact; curved
# primitives use an enclosing polytope so vertex-only planners remain safe.
geometry_msgs/Point[] conservative_vertices
