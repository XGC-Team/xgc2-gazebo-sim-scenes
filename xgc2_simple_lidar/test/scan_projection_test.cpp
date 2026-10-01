#include "legacy_scan_projection.hpp"
#include "xgc2_simple_lidar/scan_projection.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <vector>

namespace {

using xgc2_simple_lidar::ScanProjection;
using xgc2_simple_lidar_test::LegacyScanProjection;

ignition::math::Vector3d point(const sensor_msgs::PointCloud2& cloud, unsigned i) {
    float xyz[3];
    std::memcpy(xyz, cloud.data.data() + static_cast<std::size_t>(i) * cloud.point_step, sizeof(xyz));
    return {xyz[0], xyz[1], xyz[2]};
}

TEST(ScanProjection, WorldPlanesAndMountingTransform) {
    // Columns look forward/left; rows look horizontally/up at 45 degrees.
    xgc2_simple_lidar::ScanProjection projection(3, 2, 0, M_PI, 0, M_PI_4, 0.1, 20);
    // Last column repeats the second column in Gazebo; it must not yield a
    // third point rotated to pi. Intensity/unused fields are never coordinates.
    const float scan[] = {
        2, 11, 0, 3, 12, 0, 3, 12, 0, 2 * std::sqrt(2.f), 13, 0, 3 * std::sqrt(2.f), 14, 0, 3 * std::sqrt(2.f), 14, 0};
    const ignition::math::Pose3d mount(4, 5, 6, 0, 0, M_PI_2);
    const auto& cloud = projection.Project(scan, mount, ros::Time(12, 345));
    EXPECT_EQ(cloud.header.frame_id, "world");
    EXPECT_EQ(cloud.header.stamp, ros::Time(12, 345));
    ASSERT_EQ(cloud.width, 4u);
    EXPECT_LT(point(cloud, 0).Distance({4, 7, 6}), 1e-5);
    EXPECT_LT(point(cloud, 1).Distance({1, 5, 6}), 1e-5);
    EXPECT_LT(point(cloud, 2).Distance({4, 7, 8}), 1e-5);
    EXPECT_LT(point(cloud, 3).Distance({1, 5, 9}), 1e-5);
    EXPECT_EQ(cloud.data.size(), 48u);
    EXPECT_EQ(cloud.fields.size(), 3u);
}

TEST(ScanProjection, CpuRangesKeepTheirFinalColumnAndMeasurementPose) {
    xgc2_simple_lidar::ScanProjection projection(3, 2, 0, M_PI, 0, M_PI_4, 0.1, 20, false, 1);
    const float ranges[] = {2, 3, 4, 2 * std::sqrt(2.f), 3 * std::sqrt(2.f), 4 * std::sqrt(2.f)};
    const ignition::math::Pose3d measurement_pose(4, 5, 6, 0, 0, M_PI_2);
    const auto& cloud = projection.Project(ranges, measurement_pose, ros::Time(12, 345));
    ASSERT_EQ(cloud.width, 6u);
    EXPECT_LT(point(cloud, 0).Distance({4, 7, 6}), 1e-5);
    EXPECT_LT(point(cloud, 2).Distance({4, 1, 6}), 1e-5);
    EXPECT_LT(point(cloud, 5).Distance({4, 1, 10}), 1e-5);
    EXPECT_EQ(cloud.header.frame_id, "world");
    EXPECT_EQ(cloud.header.stamp, ros::Time(12, 345));
}

TEST(ScanProjection, InvalidReturnsDoNotBecomeObstaclesOrStalePoints) {
    xgc2_simple_lidar::ScanProjection projection(3, 2, 0, 1, 0, 1, 0.1, 20);
    const float invalid[] = {0,
                             0,
                             0,
                             20,
                             0,
                             0,
                             1,
                             0,
                             0,
                             std::numeric_limits<float>::infinity(),
                             0,
                             0,
                             std::numeric_limits<float>::quiet_NaN(),
                             0,
                             0,
                             1,
                             0,
                             0};
    const float valid[] = {1, 0, 0, 2, 0, 0, 1, 0, 0, 3, 0, 0, 4, 0, 0, 1, 0, 0};
    EXPECT_EQ(projection.Project(valid, {}, ros::Time(1)).width, 4u);
    const auto& empty = projection.Project(invalid, {}, ros::Time(2));
    EXPECT_EQ(empty.width, 0u);
    EXPECT_EQ(empty.row_step, 0u);
    EXPECT_TRUE(empty.data.empty());
    EXPECT_EQ(empty.header.stamp, ros::Time(2));
    EXPECT_EQ(projection.Project(valid, {}, ros::Time(0)).header.stamp, ros::Time(0));
}

bool SameCloud(const sensor_msgs::PointCloud2& a, const sensor_msgs::PointCloud2& b) {
    return a.header.seq == b.header.seq && a.header.stamp == b.header.stamp && a.header.frame_id == b.header.frame_id &&
           a.height == b.height && a.width == b.width && a.point_step == b.point_step && a.row_step == b.row_step &&
           a.is_bigendian == b.is_bigendian && a.is_dense == b.is_dense && a.fields.size() == b.fields.size() &&
           a.data.size() == b.data.size() && std::memcmp(a.data.data(), b.data.data(), a.data.size()) == 0;
}

// First-return ranges to the walls of a room around the origin. Points on a
// wall cancel the sensor position in one coordinate down to rounding
// residue, where any change in the rotation arithmetic shows in the floats.
void RoomRanges(std::vector<float>& scan, unsigned width, unsigned height, unsigned stride, double yaw_min,
                double yaw_max, double pitch_min, double pitch_max, const ignition::math::Pose3d& pose) {
    const double walls[3][2] = {{-6, 6}, {-5, 5}, {0, 3}};
    for (unsigned j = 0; j < height; ++j) {
        const double pitch = pitch_min + (pitch_max - pitch_min) * j / (height - 1);
        for (unsigned i = 0; i < width; ++i) {
            const double yaw = yaw_min + (yaw_max - yaw_min) * i / (width - 1);
            const auto d = pose.Rot().RotateVector(
                {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)});
            const double origin[3] = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()}, axis[3] = {d.X(), d.Y(), d.Z()};
            double range = std::numeric_limits<double>::infinity();
            for (unsigned k = 0; k < 3; ++k)
                for (const double wall : walls[k])
                    if (axis[k] != 0 && (wall - origin[k]) / axis[k] > 0)
                        range = std::min(range, (wall - origin[k]) / axis[k]);
            scan[stride * (static_cast<std::size_t>(j) * width + i)] = static_cast<float>(range);
        }
    }
}

// Random scans and room scans from random poses, GPU and CPU layouts, full
// and partial fields of view, valid and invalid ranges: every frame must
// equal the frozen 1.4.0-2 projection byte for byte, also for a non-unit
// quaternion.
TEST(ScanProjection, FramesAreBitIdenticalToThePerPointRotation) {
    std::mt19937 random(20261001);
    std::uniform_real_distribution<double> unit(0.0, 1.0), angle(-M_PI, M_PI);
    const float special[] = {0.0f,
                             -1.0f,
                             0.15f,
                             20.0f,
                             25.0f,
                             std::numeric_limits<float>::infinity(),
                             -std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::quiet_NaN()};
    unsigned frames = 0, points = 0;
    for (int config = 0; config < 24; ++config) {
        const bool gpu = config % 2 == 0;
        const unsigned width = 2u + static_cast<unsigned>(unit(random) * 400),
                       height = 2u + static_cast<unsigned>(config % 17);
        const double yaw_span = config % 3 == 0 ? 2 * M_PI : unit(random) * 2 * M_PI;
        const double pitch_min = -unit(random) * M_PI_2, pitch_max = unit(random) * M_PI_2;
        const unsigned stride = gpu ? 3 : 1;
        ScanProjection current(width, height, -yaw_span / 2, yaw_span / 2, pitch_min, pitch_max, 0.15, 20, gpu, stride);
        LegacyScanProjection legacy(width, height, -yaw_span / 2, yaw_span / 2, pitch_min, pitch_max, 0.15, 20, gpu,
                                    stride);
        std::vector<float> scan(static_cast<std::size_t>(width) * height * stride);
        for (int frame = 0; frame < 6; ++frame) {
            ignition::math::Quaterniond rotation(angle(random) / 8, angle(random) / 8, angle(random));
            if (frame == 5)
                rotation = {rotation.W() * 1.7, rotation.X() * 1.7, rotation.Y() * 1.7, rotation.Z() * 1.7};
            const bool room = frame >= 3;
            const ignition::math::Vector3d position =
                room ? ignition::math::Vector3d(angle(random), angle(random), 1.2)
                     : ignition::math::Vector3d(angle(random) * 30, angle(random) * 30, angle(random));
            const ignition::math::Pose3d pose(position, rotation);
            for (auto& range : scan) {
                const double draw = unit(random);
                range = draw < 0.1 ? special[static_cast<std::size_t>(draw * 80) % 8]
                                   : static_cast<float>(unit(random) * 24.0);
            }
            if (room)
                RoomRanges(scan, width, height, stride, -yaw_span / 2, yaw_span / 2, pitch_min, pitch_max, pose);
            const ros::Time stamp(static_cast<uint32_t>(frame), 17u);
            const auto& expected = legacy.Project(scan.data(), pose, stamp);
            const auto& actual = current.Project(scan.data(), pose, stamp);
            ASSERT_TRUE(SameCloud(expected, actual));
            ++frames;
            points += actual.width;
        }
    }
    EXPECT_EQ(frames, 144u);
    EXPECT_TRUE(points > 100000u);
}

TEST(ScanProjection, FramesReuseOneAllocation) {
    ScanProjection projection(360, 16, -M_PI, M_PI, -0.5, 0.5, 0.15, 20);
    std::vector<float> full(std::size_t{360} * 16 * 3, 5.0f), sparse(full.size(), 30.0f);
    sparse[0] = 5.0f;
    const auto* storage = projection.Project(full.data(), {}, ros::Time(1)).data.data();
    for (int i = 0; i < 10; ++i) {
        const auto& cloud = projection.Project(i % 2 ? sparse.data() : full.data(), {}, ros::Time(2 + i));
        EXPECT_EQ(cloud.width, i % 2 ? 1u : 359u * 16u);
        EXPECT_EQ(cloud.data.data(), storage);
    }
}

} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
