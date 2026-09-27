#include "xgc2_simple_lidar/scan_projection.hpp"
#include <cmath>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>

namespace {

ignition::math::Vector3d point(const sensor_msgs::PointCloud2& cloud, unsigned i) {
    float xyz[3];
    std::memcpy(xyz, cloud.data.data() + i * cloud.point_step, sizeof(xyz));
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

} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
