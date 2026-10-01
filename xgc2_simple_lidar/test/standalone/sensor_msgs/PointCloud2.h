// Minimal sensor_msgs/PointCloud2 stand-in with the ROS field layout, for
// compiling scan_projection.hpp without ROS. Serialization is not modelled.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <ros/time.h>

namespace std_msgs {
struct Header {
    uint32_t seq{0};
    ros::Time stamp;
    std::string frame_id;
};
} // namespace std_msgs

namespace sensor_msgs {

struct PointField {
    enum : uint8_t { INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4, INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8 };
    std::string name;
    uint32_t offset{0};
    uint8_t datatype{0};
    uint32_t count{0};
};

struct PointCloud2 {
    std_msgs::Header header;
    uint32_t height{0};
    uint32_t width{0};
    std::vector<PointField> fields;
    uint8_t is_bigendian{0};
    uint32_t point_step{0};
    uint32_t row_step{0};
    std::vector<uint8_t> data;
    uint8_t is_dense{0};
};

} // namespace sensor_msgs
