// Per-frame cost of ScanProjection::Project against the frozen 1.4.0-2
// projection, on first-return scans of simple analytic scenes. Both run in
// the same process, alternately, and every frame is checked byte for byte.
// Build and run: test/run_standalone_benchmark.sh (no ROS needed).
#include "legacy_scan_projection.hpp"
#include "xgc2_simple_lidar/scan_projection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace {

struct Scene {
    const char* name;
    unsigned width, height, stride;
    double pitch_min, pitch_max;
    bool gpu;
    int kind; // 0 room, 1 open ground, 2 ground and pillars, 3 random half misses
};

double Cast(int kind, const ignition::math::Vector3d& o, const ignition::math::Vector3d& d) {
    double best = std::numeric_limits<double>::infinity();
    const auto plane = [&](double origin, double direction, double value) {
        if (direction != 0 && (value - origin) / direction > 0)
            best = std::min(best, (value - origin) / direction);
    };
    plane(o.Z(), d.Z(), 0);
    if (kind == 0) {
        plane(o.X(), d.X(), -6), plane(o.X(), d.X(), 6), plane(o.Y(), d.Y(), -5), plane(o.Y(), d.Y(), 5);
        plane(o.Z(), d.Z(), 3);
    }
    if (kind == 2)
        for (int k = 0; k < 48; ++k) {
            const int column = k % 8, row = k / 8;
            const double cx = -14 + column * 4.1, cy = -12 + row * 4.7, radius = 0.3;
            const double ox = o.X() - cx, oy = o.Y() - cy, a = d.X() * d.X() + d.Y() * d.Y();
            const double b = 2 * (ox * d.X() + oy * d.Y()), c = ox * ox + oy * oy - radius * radius;
            if (a > 0 && b * b >= 4 * a * c) {
                const double t = (-b - std::sqrt(b * b - 4 * a * c)) / (2 * a);
                if (t > 0 && o.Z() + t * d.Z() < 6)
                    best = std::min(best, t);
            }
        }
    return best;
}

std::vector<float> Scan(const Scene& scene, const ignition::math::Pose3d& pose, std::mt19937& random) {
    std::vector<float> scan(static_cast<std::size_t>(scene.width) * scene.height * scene.stride, 0.5f);
    std::uniform_real_distribution<float> unit(0, 1);
    for (unsigned j = 0; j < scene.height; ++j) {
        const double pitch = scene.pitch_min + (scene.pitch_max - scene.pitch_min) * j / (scene.height - 1);
        for (unsigned i = 0; i < scene.width; ++i) {
            const double yaw = -M_PI + 2 * M_PI * i / (scene.width - 1);
            const auto d = pose.Rot().RotateVector(
                {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)});
            const double range = scene.kind == 3 ? (unit(random) < 0.5f ? 1e9 : 0.2 + 19 * unit(random))
                                                 : Cast(scene.kind, pose.Pos(), d);
            scan[scene.stride * (static_cast<std::size_t>(j) * scene.width + i)] =
                static_cast<float>(std::min(range, 1e9));
        }
    }
    return scan;
}

template <class Projection>
double Microseconds(Projection& projection, const std::vector<float>& scan, const ignition::math::Pose3d* poses,
                    int frames) {
    const auto start = std::chrono::steady_clock::now();
    unsigned sink = 0;
    for (int frame = 0; frame < frames; ++frame)
        sink += projection.Project(scan.data(), poses[frame % 4], ros::Time(1)).width;
    const auto stop = std::chrono::steady_clock::now();
    if (sink == 1)
        std::puts("");
    return std::chrono::duration<double, std::micro>(stop - start).count() / frames;
}

int Run(int rounds) {
    const Scene scenes[] = {
        {"GPU room 360x16", 360, 16, 3, -0.5, 0.5, true, 0},
        {"GPU open ground 360x16", 360, 16, 3, -0.5, 0.5, true, 1},
        {"GPU pillars 360x32 vfov 180", 360, 32, 3, -M_PI_2, M_PI_2, true, 2},
        {"GPU random half 360x16", 360, 16, 3, -0.5, 0.5, true, 3},
        {"CPU room 360x16", 360, 16, 1, -0.5, 0.5, false, 0},
    };
    std::mt19937 random(7);
    std::printf("%-30s %7s %10s %10s %7s %s\n", "scene", "points", "legacy us", "current us", "ratio", "identical");
    bool all_identical = true;
    for (const auto& scene : scenes) {
        const ignition::math::Pose3d poses[] = {{0.3, -0.2, 1.2, 0.05, -0.03, 0.7},
                                                {1.0, 2.0, 1.1, 0.1, 0.0, -2.0},
                                                {-3.0, 1.0, 0.8, 0.0, 0.02, 3.0},
                                                {0.0, 0.0, 1.5, 0.0, 0.0, 0.1}};
        const auto scan = Scan(scene, poses[0], random);
        xgc2_simple_lidar::ScanProjection current(scene.width, scene.height, -M_PI, M_PI, scene.pitch_min,
                                                  scene.pitch_max, 0.15, 20, scene.gpu, scene.stride);
        xgc2_simple_lidar_test::LegacyScanProjection legacy(scene.width, scene.height, -M_PI, M_PI, scene.pitch_min,
                                                            scene.pitch_max, 0.15, 20, scene.gpu, scene.stride);
        bool identical = true;
        unsigned points = 0;
        for (const auto& pose : poses) {
            const auto& a = legacy.Project(scan.data(), pose, ros::Time(1));
            const auto& b = current.Project(scan.data(), pose, ros::Time(1));
            identical = identical && a.width == b.width && a.data.size() == b.data.size() &&
                        std::memcmp(a.data.data(), b.data.data(), a.data.size()) == 0;
            points = b.width;
        }
        std::vector<double> legacy_us, current_us;
        for (int round = 0; round < rounds; ++round) {
            legacy_us.push_back(Microseconds(legacy, scan, poses, 64));
            current_us.push_back(Microseconds(current, scan, poses, 64));
        }
        std::sort(legacy_us.begin(), legacy_us.end());
        std::sort(current_us.begin(), current_us.end());
        const double before = legacy_us[legacy_us.size() / 2], after = current_us[current_us.size() / 2];
        std::printf("%-30s %7u %10.1f %10.1f %6.2fx %s\n", scene.name, points, before, after, before / after,
                    identical ? "yes" : "NO");
        all_identical = all_identical && identical;
    }
    std::puts("Median of rounds of 64 frames each; legacy and current alternate in one process.");
    return all_identical ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    try {
        return Run(argc > 1 ? std::max(1, std::atoi(argv[1])) : 15);
    } catch (...) {
        std::fputs("benchmark failed\n", stderr);
        return 2;
    }
}
