#pragma once

#include <ignition/math/Vector3.hh>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace xgc2_gazebo_scene {

/// Local-frame geometry extracted from one Gazebo mesh collision.
struct ConvexMeshGeometry {
    std::string uri;
    std::string resolved_path;
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    std::vector<ignition::math::Vector3d> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

/// Load the mesh selected by an SDF collision and apply its collision scale.
///
/// The returned vertices use the exact local coordinates consumed by Gazebo.
/// If a named submesh is centered by SDF, the same AABB-center translation is
/// applied before scale.
bool LoadConvexMeshGeometry(const std::string& uri, const ignition::math::Vector3d& scale,
                            const std::string& submesh_name, bool center_submesh, ConvexMeshGeometry* geometry,
                            std::string* error);

/// Validate that every triangle is a boundary face of one closed convex body.
///
/// This deliberately rejects concave, open, degenerate, and multi-body mesh
/// collisions. Publishing a convex hull approximation would silently change
/// physical planning geometry, so invalid meshes fail closed.
bool ValidateClosedConvexMesh(const ConvexMeshGeometry& geometry, std::string* error);

} // namespace xgc2_gazebo_scene
