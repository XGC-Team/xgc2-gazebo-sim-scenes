#include "xgc2_gazebo_scene/convex_mesh_geometry.hpp"

#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/SystemPaths.hh>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

namespace xgc2_gazebo_scene {
namespace {

using Index = std::uint32_t;
using Triangle = std::array<Index, 3>;
using VertexKey = std::tuple<double, double, double>;

VertexKey Key(const ignition::math::Vector3d& vertex) {
    const auto canonical = [](double value) {
        return value == 0.0 ? 0.0 : value;
    };
    return {canonical(vertex.X()), canonical(vertex.Y()), canonical(vertex.Z())};
}

Index AddVertex(const ignition::math::Vector3d& vertex, std::map<VertexKey, Index>* indices,
                std::vector<ignition::math::Vector3d>* vertices) {
    const VertexKey key = Key(vertex);
    const auto found = indices->find(key);
    if (found != indices->end()) {
        return found->second;
    }
    const Index index = static_cast<Index>(vertices->size());
    vertices->push_back(vertex);
    indices->emplace(key, index);
    return index;
}

void AppendTriangle(Index first, Index second, Index third, std::vector<Triangle>* triangles) {
    if (first != second && second != third && first != third) {
        triangles->push_back({first, second, third});
    }
}

bool AppendSubMesh(const gazebo::common::SubMesh& submesh, const ignition::math::Vector3d& center,
                   const ignition::math::Vector3d& scale, std::map<VertexKey, Index>* indices,
                   ConvexMeshGeometry* geometry, std::string* error) {
    std::vector<Index> remapped;
    remapped.reserve(submesh.GetVertexCount());
    for (unsigned int index = 0; index < submesh.GetVertexCount(); ++index) {
        const ignition::math::Vector3d centered = submesh.Vertex(index) - center;
        const ignition::math::Vector3d scaled(centered.X() * scale.X(), centered.Y() * scale.Y(),
                                              centered.Z() * scale.Z());
        remapped.push_back(AddVertex(scaled, indices, &geometry->vertices));
    }

    std::vector<Index> mesh_indices;
    mesh_indices.reserve(submesh.GetIndexCount());
    for (unsigned int index = 0; index < submesh.GetIndexCount(); ++index) {
        const unsigned int source_index = submesh.GetIndex(index);
        if (source_index >= remapped.size()) {
            *error = "mesh index exceeds the submesh vertex array";
            return false;
        }
        mesh_indices.push_back(remapped[source_index]);
    }

    switch (submesh.GetPrimitiveType()) {
    case gazebo::common::SubMesh::TRIANGLES:
        if (mesh_indices.size() % 3 != 0) {
            *error = "triangle mesh index count is not divisible by three";
            return false;
        }
        for (std::size_t index = 0; index < mesh_indices.size(); index += 3) {
            AppendTriangle(mesh_indices[index], mesh_indices[index + 1], mesh_indices[index + 2], &geometry->triangles);
        }
        break;
    case gazebo::common::SubMesh::TRIFANS:
        for (std::size_t index = 1; index + 1 < mesh_indices.size(); ++index) {
            AppendTriangle(mesh_indices[0], mesh_indices[index], mesh_indices[index + 1], &geometry->triangles);
        }
        break;
    case gazebo::common::SubMesh::TRISTRIPS:
        for (std::size_t index = 0; index + 2 < mesh_indices.size(); ++index) {
            if (index % 2 == 0) {
                AppendTriangle(mesh_indices[index], mesh_indices[index + 1], mesh_indices[index + 2],
                               &geometry->triangles);
            } else {
                AppendTriangle(mesh_indices[index + 1], mesh_indices[index], mesh_indices[index + 2],
                               &geometry->triangles);
            }
        }
        break;
    default:
        *error = "mesh uses non-triangular primitive topology";
        return false;
    }
    return true;
}

} // namespace

bool LoadConvexMeshGeometry(const std::string& uri, const ignition::math::Vector3d& scale,
                            const std::string& submesh_name, bool center_submesh, ConvexMeshGeometry* geometry,
                            std::string* error) {
    if (geometry == nullptr || error == nullptr) {
        return false;
    }
    *geometry = ConvexMeshGeometry{};
    error->clear();
    if (uri.empty()) {
        *error = "mesh URI is empty";
        return false;
    }
    if (scale.X() <= 0.0 || scale.Y() <= 0.0 || scale.Z() <= 0.0 || !std::isfinite(scale.X()) ||
        !std::isfinite(scale.Y()) || !std::isfinite(scale.Z())) {
        *error = "mesh scale must contain finite positive values";
        return false;
    }

    std::string resolved = gazebo::common::SystemPaths::Instance()->FindFileURI(uri);
    if (resolved.empty()) {
        resolved = gazebo::common::SystemPaths::Instance()->FindFile(uri);
    }
    if (resolved.empty()) {
        *error = "cannot resolve mesh URI: " + uri;
        return false;
    }
    const gazebo::common::Mesh* mesh = gazebo::common::MeshManager::Instance()->Load(resolved);
    if (mesh == nullptr) {
        *error = "Gazebo cannot load mesh: " + resolved;
        return false;
    }

    std::vector<const gazebo::common::SubMesh*> submeshes;
    if (!submesh_name.empty()) {
        const gazebo::common::SubMesh* submesh = mesh->GetSubMesh(submesh_name);
        if (submesh == nullptr) {
            *error = "mesh does not contain submesh: " + submesh_name;
            return false;
        }
        submeshes.push_back(submesh);
    } else {
        for (unsigned int index = 0; index < mesh->GetSubMeshCount(); ++index) {
            submeshes.push_back(mesh->GetSubMesh(index));
        }
    }
    if (submeshes.empty()) {
        *error = "mesh has no submeshes";
        return false;
    }

    geometry->uri = uri;
    geometry->resolved_path = resolved;
    geometry->scale = scale;
    std::map<VertexKey, Index> indices;
    for (const gazebo::common::SubMesh* submesh : submeshes) {
        ignition::math::Vector3d center = ignition::math::Vector3d::Zero;
        if (center_submesh) {
            center = (submesh->Min() + submesh->Max()) * 0.5;
        }
        if (!AppendSubMesh(*submesh, center, scale, &indices, geometry, error)) {
            return false;
        }
    }
    if (!ValidateClosedConvexMesh(*geometry, error)) {
        return false;
    }
    return true;
}

bool ValidateClosedConvexMesh(const ConvexMeshGeometry& geometry, std::string* error) {
    if (error == nullptr) {
        return false;
    }
    error->clear();
    if (geometry.vertices.size() < 4 || geometry.triangles.size() < 4) {
        *error = "mesh does not contain a three-dimensional polyhedron";
        return false;
    }

    double coordinate_scale = 1.0;
    for (const auto& vertex : geometry.vertices) {
        if (!std::isfinite(vertex.X()) || !std::isfinite(vertex.Y()) || !std::isfinite(vertex.Z())) {
            *error = "mesh contains a non-finite vertex";
            return false;
        }
        coordinate_scale = std::max(coordinate_scale, vertex.Length());
    }
    const double distance_tolerance = coordinate_scale * 1.0e-8;
    std::map<std::pair<Index, Index>, std::size_t> edge_counts;
    std::set<Index> referenced_vertices;
    bool has_non_coplanar_vertex = false;

    for (const Triangle& triangle : geometry.triangles) {
        if (triangle[0] >= geometry.vertices.size() || triangle[1] >= geometry.vertices.size() ||
            triangle[2] >= geometry.vertices.size() || triangle[0] == triangle[1] || triangle[1] == triangle[2] ||
            triangle[0] == triangle[2]) {
            *error = "mesh contains an invalid triangle";
            return false;
        }
        referenced_vertices.insert(triangle.begin(), triangle.end());
        for (std::size_t index = 0; index < 3; ++index) {
            const Index first = triangle[index];
            const Index second = triangle[(index + 1) % 3];
            ++edge_counts[std::minmax(first, second)];
        }

        const ignition::math::Vector3d& first = geometry.vertices[triangle[0]];
        ignition::math::Vector3d normal =
            (geometry.vertices[triangle[1]] - first).Cross(geometry.vertices[triangle[2]] - first);
        if (normal.Length() <= std::numeric_limits<double>::epsilon() * coordinate_scale * coordinate_scale) {
            *error = "mesh contains a degenerate triangle";
            return false;
        }
        normal.Normalize();
        double minimum = 0.0;
        double maximum = 0.0;
        for (const auto& vertex : geometry.vertices) {
            const double distance = normal.Dot(vertex - first);
            minimum = std::min(minimum, distance);
            maximum = std::max(maximum, distance);
            has_non_coplanar_vertex = has_non_coplanar_vertex || std::abs(distance) > distance_tolerance;
        }
        if (minimum < -distance_tolerance && maximum > distance_tolerance) {
            *error = "mesh is concave or contains internal triangle faces";
            return false;
        }
    }

    if (!has_non_coplanar_vertex) {
        *error = "mesh vertices are coplanar";
        return false;
    }
    for (const auto& edge : edge_counts) {
        if (edge.second != 2) {
            *error = "mesh is open or non-manifold";
            return false;
        }
    }
    if (referenced_vertices.size() != geometry.vertices.size()) {
        *error = "mesh contains unreferenced vertices";
        return false;
    }
    return true;
}

} // namespace xgc2_gazebo_scene
