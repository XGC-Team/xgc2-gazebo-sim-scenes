// Frozen copy of ScanProjection as of xgc2-gazebo-sim-scenes 1.4.0-2: one
// ignition RotateVector, with its quaternion inverse, per point. The tests
// and the benchmark compare the current projection with it bit for bit.
// Test-only; never installed. The only edit is the explicit size_t offset.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
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

} // namespace xgc2_simple_lidar_test
