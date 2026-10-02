#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <sensor_msgs/PointCloud2.h>

namespace xgc2_simple_lidar {

// Gazebo rays use +X forward, +Y left, +Z up, not camera optical axes.
// Configure once; frames reuse direction and message storage without PCL.
class ScanProjection {
  public:
    ScanProjection(unsigned width, unsigned height, double yaw_min, double yaw_max, double pitch_min, double pitch_max,
                   double range_min, double range_max, bool gpu_layout = true, unsigned stride = 3)
        // Ranges arrive as floats. A double limit with no exact float, such as
        // 20.9, let the GPU's no-hit value float(20.9) through as a sphere of
        // points at the maximum range, and float(0.15) passed the near limit.
        : range_min_(static_cast<float>(range_min)), range_max_(static_cast<float>(range_max)) {
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
        cloud_.point_step = kPointStep;
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
        cloud_.data.reserve(rays_.size() * kPointStep);
    }

    const sensor_msgs::PointCloud2& Project(const float* scan, const ignition::math::Pose3d& sensor_in_world,
                                            const ros::Time& stamp) {
        cloud_.header.stamp = stamp;
        ++cloud_.header.seq;
        // Frame constants. Points are stored through a byte pointer, which may
        // alias anything, so reading the pose inside the loop made every point
        // reload it and recompute Quaternion::Inverse (a norm and four
        // divisions). The per-point products are still ignition's
        // RotateVector, q * (v * q^-1), so every point is bit-identical.
        const auto& measured_rotation = sensor_in_world.Rot();
        const ignition::math::Quaterniond rotation(measured_rotation.W(), measured_rotation.X(), measured_rotation.Y(),
                                                   measured_rotation.Z());
        const ignition::math::Quaterniond inverse = rotation.Inverse();
        const double x = sensor_in_world.Pos().X(), y = sensor_in_world.Pos().Y(), z = sensor_in_world.Pos().Z();
        const float range_min = range_min_, range_max = range_max_;
        // The capacity reserved at construction holds every ray: no allocation.
        cloud_.data.resize(rays_.size() * kPointStep);
        std::uint8_t* const out = cloud_.data.data();
        std::size_t count = 0;
        for (const auto& ray : rays_) {
            // Gazebo's raw GPU frame is range, retro, unused. It precedes the
            // sensor's noise/invalid-range postprocessing; this is an ideal sensor.
            // A range must lie strictly between the limits; NaN and inf fail.
            const float range = scan[ray.offset];
            if (!(range > range_min && range < range_max))
                continue;
            const auto v = ray.direction * static_cast<double>(range);
            const auto rotated = rotation * (ignition::math::Quaterniond(0.0, v.X(), v.Y(), v.Z()) * inverse);
            const float xyz[] = {static_cast<float>(x + rotated.X()), static_cast<float>(y + rotated.Y()),
                                 static_cast<float>(z + rotated.Z())};
            std::memcpy(out + count * kPointStep, xyz, sizeof(xyz));
            ++count;
        }
        cloud_.width = static_cast<std::uint32_t>(count);
        cloud_.row_step = static_cast<std::uint32_t>(count * kPointStep);
        cloud_.data.resize(cloud_.row_step);
        return cloud_;
    }

  private:
    static constexpr std::uint32_t kPointStep = 3 * sizeof(float);

    float range_min_, range_max_;
    struct Ray {
        std::size_t offset;
        ignition::math::Vector3d direction;
    };
    std::vector<Ray> rays_;
    sensor_msgs::PointCloud2 cloud_;
};

} // namespace xgc2_simple_lidar
