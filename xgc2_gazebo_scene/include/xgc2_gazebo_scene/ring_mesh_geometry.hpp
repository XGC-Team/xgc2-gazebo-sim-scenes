#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace xgc2_gazebo_scene {

// A backend-only approximation of the Euclidean tube, never a convex hull or
// a scaled circular torus. The polygon tube is sampled radially, so overlapping
// normal offsets at tight ellipse curvature cannot introduce internal sheets.
struct RingMeshGeometry {
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    double centerline_error = 0;
    double surface_error_bound = 0;
};

inline RingMeshGeometry MakeRingMesh(double a, double b, double r) {
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(r) || a <= 0 || b <= 0 || r <= 0 ||
        r >= std::min(a, b))
        throw std::invalid_argument("ring requires finite positive radii and tube smaller than both radii");
    constexpr double pi = 3.14159265358979323846;
    const double hole = std::min(a, b) - r;
    // Bound the centerline chord error by both tube and hole dimensions.
    const double tolerance = std::min({0.0005, r * 0.01, hole * 0.01});
    const double major = std::max(a, b);
    const double requested = std::ceil(pi * std::sqrt(major / (2 * tolerance)));
    const double requested_sides = std::ceil(pi * std::sqrt(r / (2 * tolerance)));
    if (requested > 8192 || requested_sides > 2048)
        throw std::runtime_error("ring mesh precision exceeds backend capacity");
    const unsigned n = std::max(32u, 4u * static_cast<unsigned>(std::ceil(requested / 4)));
    const unsigned m = std::max(16u, 4u * static_cast<unsigned>(std::ceil(requested_sides / 4)));
    if (static_cast<std::size_t>(n) * m > 4000000)
        throw std::runtime_error("ring mesh vertex capacity exceeded");
    std::vector<std::array<double, 2>> points(n), normals(n);
    std::vector<double> supports(n);
    for (unsigned i = 0; i < n; ++i) {
        const double u = 2 * pi * i / n;
        points[i] = {a * std::cos(u), b * std::sin(u)};
    }
    for (unsigned i = 0; i < n; ++i) {
        const auto& p = points[i];
        const auto& q = points[(i + 1) % n];
        const double dx = q[0] - p[0], dy = q[1] - p[1];
        const double length = std::hypot(dx, dy);
        normals[i] = {dy / length, -dx / length};
        supports[i] = normals[i][0] * p[0] + normals[i][1] * p[1];
    }
    RingMeshGeometry result;
    result.centerline_error = major * (2 * pi / n) * (2 * pi / n) / 8;
    result.vertices.reserve(static_cast<std::size_t>(n) * m);
    for (unsigned i = 0; i < n; ++i) {
        const double base = std::hypot(points[i][0], points[i][1]);
        const double dx = points[i][0] / base, dy = points[i][1] / base;
        for (unsigned j = 0; j < m; ++j) {
            const double v = 2 * pi * j / m;
            const double s = r * std::abs(std::cos(v));
            double rho = base;
            if (j == m / 4 || j == 3 * m / 4) {
                rho = base;
            } else if (std::cos(v) < 0) {
                // Erosion of a convex polygon: intersect all inward-offset
                // halfplanes. This remains valid beyond the curvature radius.
                rho = std::numeric_limits<double>::infinity();
                for (unsigned k = 0; k < n; ++k) {
                    const double dot = normals[k][0] * dx + normals[k][1] * dy;
                    if (dot > 0)
                        rho = std::min(rho, (supports[k] - s) / dot);
                }
            } else {
                // Polygon Minkowski disk: intersect outward offset edges and
                // endpoint circles analytically. Taking the farthest valid
                // intersection is the outer radial boundary of this convex set.
                rho = base;
                for (unsigned k = 0; k < n; ++k) {
                    const auto& p = points[k];
                    const auto& q = points[(k + 1) % n];
                    const double projection = p[0] * dx + p[1] * dy;
                    const double perpendicular = p[0] * dy - p[1] * dx;
                    const double discriminant = s * s - perpendicular * perpendicular;
                    if (discriminant >= 0)
                        rho = std::max(rho, projection + std::sqrt(discriminant));
                    const double dot = normals[k][0] * dx + normals[k][1] * dy;
                    if (dot > 0) {
                        const double candidate = (supports[k] + s) / dot;
                        const double ex = q[0] - p[0], ey = q[1] - p[1];
                        const double t =
                            ((candidate * dx - p[0]) * ex + (candidate * dy - p[1]) * ey) / (ex * ex + ey * ey);
                        if (t >= 0 && t <= 1)
                            rho = std::max(rho, candidate);
                    }
                }
            }
            result.vertices.push_back({rho * dx, rho * dy, r * std::sin(v)});
        }
    }
    for (unsigned i = 0; i < n; ++i) {
        for (unsigned j = 0; j < m; ++j) {
            const auto p = i * m + j, q = ((i + 1) % n) * m + j;
            const auto next_p = i * m + (j + 1) % m, next_q = ((i + 1) % n) * m + (j + 1) % m;
            result.triangles.push_back({p, q, next_q});
            result.triangles.push_back({p, next_q, next_p});
        }
    }
    // Conservative Hausdorff enclosure: each convex inner/outer horizontal
    // section contains the origin ball h and lies in radius R. Its radial
    // function is Lipschitz with angular constant R^2/h and offset constant
    // R/h. Include each full parameter cell's diameter, not just chord error.
    const double h = hole - result.centerline_error, radius = major + r;
    const double angular_step = (major / std::min(a, b)) * 2 * pi / n;
    result.surface_error_bound =
        result.centerline_error + radius * radius / h * angular_step + (radius / h + 1) * r * 2 * pi / m;
    return result;
}

} // namespace xgc2_gazebo_scene
