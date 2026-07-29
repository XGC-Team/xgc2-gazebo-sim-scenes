uint8 SHAPE_BOX=1
uint8 SHAPE_SPHERE=2
uint8 SHAPE_CYLINDER=3
uint8 SHAPE_CONVEX_MESH=4

string part_id
uint8 shape
geometry_msgs/Pose local_pose

# Exact primitive parameters. Box uses size; sphere uses radius; cylinder uses
# radius and length. Planners should prefer these analytical values when able.
geometry_msgs/Vector3 size
float64 radius
float64 length

# Source metadata for SHAPE_CONVEX_MESH. mesh_scale is the collision scale
# already applied to conservative_vertices.
string mesh_uri
string mesh_submesh
bool mesh_center_submesh
geometry_msgs/Vector3 mesh_scale

# Conservative local-frame convex vertices. Box vertices are exact; curved
# primitives use an enclosing polytope so vertex-only planners remain safe.
# For SHAPE_CONVEX_MESH this is the exact collision-mesh convex vertex set.
geometry_msgs/Point[] conservative_vertices
