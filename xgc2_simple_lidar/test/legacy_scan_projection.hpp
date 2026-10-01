// Frozen copy of ScanProjection as of xgc2-gazebo-sim-scenes 1.4.0-2: one
// ignition RotateVector, with its quaternion inverse, per point, and range
// limits compared as doubles. The tests and the benchmark compare the
// current projection with it (SameProjection below); only a range equal to a
// limit's float rounding is treated differently now (see
// LimitsAreComparedAsFloats). Test-only; never installed. The only edit to
// the class is the explicit size_t offset.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <sensor_msgs/PointCloud2.h>

namespace xgc2_simple_lidar_test {

// Gazebo rays use +X forward, +Y left, +Z up, not camera optical axes.
// Configure once; frames reuse direction and message storage without PCL.
class LegacyScanProjection {
  public:
    LegacyScanProjection(unsigned width, unsigned height, double yaw_min, double yaw_max, double pitch_min,
                         double pitch_max, double range_min, double range_max, bool gpu_layout = true,
                         unsigned stride = 3)
        : range_min_(range_min), range_max_(range_max) {
        if (width < 2 || height < 2)
            throw std::invalid_argument("simple lidar requires at least two samples on each axis");
        if (!stride)
            throw std::invalid_argument("simple lidar scan stride must be positive");
        const unsigned columns = width - (gpu_layout ? 1 : 0);
        rays_.reserve(static_cast<std::size_t>(columns) * height);
        for (unsigned j = 0; j < height; ++j) {
            const double pitch = pitch_min + (pitch_max - pitch_min) * j / (height - 1);
            // Gazebo 11 GpuLaser::CreateMesh repeats the previous ray in the last
            // column when it caps the texture index. Do not emit that depth at the
            // requested final angle: it creates points off surfaces at the seam.
            for (unsigned i = 0; i < columns; ++i) {
                const double yaw = yaw_min + (yaw_max - yaw_min) * i / (width - 1);
                rays_.push_back({stride * (static_cast<std::size_t>(j) * width + i),
                                 {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)}});
            }
        }
        cloud_.header.frame_id = "world";
        cloud_.height = 1;
        cloud_.point_step = 3 * sizeof(float);
        cloud_.is_dense = true;
        const std::uint16_t endian = 1;
        cloud_.is_bigendian = *reinterpret_cast<const std::uint8_t*>(&endian) == 0;
        for (unsigned i = 0; i < 3; ++i) {
            sensor_msgs::PointField field;
            field.name = std::string(1, "xyz"[i]);
            field.offset = i * sizeof(float);
            field.datatype = sensor_msgs::PointField::FLOAT32;
            field.count = 1;
            cloud_.fields.push_back(field);
        }
        cloud_.data.reserve(rays_.size() * cloud_.point_step);
    }

    const sensor_msgs::PointCloud2& Project(const float* scan, const ignition::math::Pose3d& sensor_in_world,
                                            const ros::Time& stamp) {
        cloud_.header.stamp = stamp;
        ++cloud_.header.seq;
        cloud_.data.resize(rays_.size() * cloud_.point_step);
        unsigned count = 0;
        for (const auto& ray : rays_) {
            // Gazebo's raw GPU frame is range, retro, unused. It precedes the
            // sensor's noise/invalid-range postprocessing; this is an ideal sensor.
            const double range = scan[ray.offset];
            if (!std::isfinite(range) || range <= range_min_ || range >= range_max_)
                continue;
            const auto point = sensor_in_world.Pos() + sensor_in_world.Rot().RotateVector(ray.direction * range);
            const float xyz[] = {static_cast<float>(point.X()), static_cast<float>(point.Y()),
                                 static_cast<float>(point.Z())};
            std::memcpy(cloud_.data.data() + static_cast<std::size_t>(count) * cloud_.point_step, xyz, sizeof(xyz));
            ++count;
        }
        cloud_.width = count;
        cloud_.row_step = count * cloud_.point_step;
        cloud_.data.resize(cloud_.row_step);
        return cloud_;
    }

  private:
    double range_min_, range_max_;
    struct Ray {
        std::size_t offset;
        ignition::math::Vector3d direction;
    };
    std::vector<Ray> rays_;
    sensor_msgs::PointCloud2 cloud_;
};

// Whether a frame from ScanProjection carries the same projection as the
// frozen one. Builds that cannot fuse a multiply and an add (every generic
// x86-64 build) must give identical bytes. Where the compiler may contract to
// FMA (arm64, or x86-64 with FMA enabled), each loop is contracted its own
// way and a coordinate may round differently in its last bits (GCC 15 with
// -march=x86-64-v3: 0.18% of coordinates, at most 7e-15 m); there every
// coordinate must still be within one float ulp, or 1e-9 m near zero.
inline bool SameProjection(const sensor_msgs::PointCloud2& a, const sensor_msgs::PointCloud2& b) {
    if (!(a.header.seq == b.header.seq && a.header.stamp == b.header.stamp && a.header.frame_id == b.header.frame_id &&
          a.height == b.height && a.width == b.width && a.point_step == b.point_step && a.row_step == b.row_step &&
          a.is_bigendian == b.is_bigendian && a.is_dense == b.is_dense && a.fields.size() == b.fields.size() &&
          a.data.size() == b.data.size()))
        return false;
#if defined(__FMA__) || defined(__FP_FAST_FMA) || defined(__ARM_FEATURE_FMA)
    for (std::size_t offset = 0; offset < a.data.size(); offset += sizeof(float)) {
        float left = 0, right = 0;
        std::memcpy(&left, a.data.data() + offset, sizeof(float));
        std::memcpy(&right, b.data.data() + offset, sizeof(float));
        const float ulp = std::nextafter(std::fabs(left), std::numeric_limits<float>::infinity()) - std::fabs(left);
        if (!(std::fabs(left - right) <= std::max(ulp, 1e-9f)))
            return false;
    }
    return true;
#else
    return std::memcmp(a.data.data(), b.data.data(), a.data.size()) == 0;
#endif
}

} // namespace xgc2_simple_lidar_test
