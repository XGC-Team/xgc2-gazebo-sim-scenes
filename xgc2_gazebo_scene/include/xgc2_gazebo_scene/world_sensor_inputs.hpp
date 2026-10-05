#pragma once

#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "xgc2_world_lidar/world_sensor_system.hpp"

namespace xgc2_gazebo_scene {

constexpr char kWorldSensorInputsEnvironment[] = "XGC_GAZEBO_WORLD_LIDAR_INPUTS";

// Cold transport projection of the SAME frozen WorldLidarInputs.shared values.
// Topic bindings are source/equipment indices, not another robot roster.
struct WorldSensorInputs {
    struct PoseInput { std::string topic, type; };
    struct CloudInput {
        std::string topic, gpu_frame;
        std::vector<std::size_t> groups;
    };
    xgc2_world_lidar::WorldSensorConfiguration configuration;
    std::vector<PoseInput> poses;
    std::vector<CloudInput> clouds;
    std::vector<std::string> outputs;
};

inline std::vector<std::string> SensorTopics(const boost::property_tree::ptree& input,
                                             const char* key) {
    std::vector<std::string> values;
    for (const auto& item : input.get_child(key)) {
        const auto value = item.second.get_value<std::string>();
        if (!item.first.empty() || value.empty() || value.front() != '/')
            throw std::invalid_argument(std::string("original absolute topic array required: ") + key);
        values.push_back(value);
    }
    return values;
}

inline WorldSensorInputs ParseWorldSensorInputs(const boost::property_tree::ptree& input) {
    if (input.get_child_optional("sceneFile"))
        throw std::invalid_argument("SceneFile is not a sensor geometry authority");
    if (input.get_child_optional("normal"))
        throw std::invalid_argument("this Gazebo attachment accepts shared equipment; normal keeps its original caller");
    WorldSensorInputs result;
    std::map<std::string, std::size_t> pose_indices, cloud_indices;
    std::set<std::string> output_topics;
    for (const auto& item : input.get_child("shared")) {
        if (!item.first.empty()) throw std::invalid_argument("shared must be an array");
        const auto& source = item.second;
        xgc2_world_lidar::SharedSensorEquipment equipment;
        auto& metadata = equipment.metadata;
        metadata.observation_model = source.get<std::string>("observation_model");
        metadata.backend = source.get<std::string>("backend");
        metadata.range_m = source.get<double>("range_m");
        metadata.publish_rate_hz = source.get<double>("publish_rate_hz");
        metadata.frame_id = source.get<std::string>("frame_id");
        metadata.stamp_policy = source.get<std::string>("stamp_policy");
        metadata.input_cloud_topic = source.get<std::string>("input_cloud_topic");
        metadata.pose_type = source.get<std::string>("pose_type");
        if (metadata.input_cloud_topic.empty() || metadata.input_cloud_topic.front() != '/')
            throw std::invalid_argument("original absolute static cloud topic required");
        if (metadata.pose_type != "nav_msgs/Odometry" && metadata.pose_type != "geometry_msgs/PoseStamped")
            throw std::invalid_argument("unsupported original pose type");
        if (const auto value = source.get_optional<double>("heading_cos_min")) metadata.heading_cos_min = *value;
        if (const auto value = source.get_optional<double>("vertical_slab_tan")) metadata.vertical_slab_tan = *value;
        if (metadata.backend == "cpu") {
            std::size_t index = 0;
            for (const auto& value : source.get_child("prevoxel_leaf_m")) {
                if (!value.first.empty() || index >= 3) throw std::invalid_argument("prevoxel XYZ required");
                metadata.prevoxel_leaf_m[index++] = value.second.get_value<float>();
            }
            if (index != 3) throw std::invalid_argument("prevoxel XYZ required");
            xgc2_world_lidar::validateSensorMetadata(metadata);
        } else if (metadata.backend == "gpu") {
            if (source.get_child_optional("prevoxel_leaf_m"))
                throw std::invalid_argument("GPU does not implement CPU prevoxel");
            metadata.min_range_m = source.get<double>("min_range_m");
            metadata.h_fov_deg = source.get<double>("h_fov_deg");
            metadata.v_fov_deg = source.get<double>("v_fov_deg");
            metadata.h_res = source.get<int>("h_res"); metadata.v_res = source.get<int>("v_res");
            metadata.point_cover_spacing_m = source.get<double>("point_cover_spacing_m");
            xgc2_world_lidar::validateGpuSensorMetadata(metadata);
        } else throw std::invalid_argument("unsupported declared backend; no fallback");
        const auto poses = SensorTopics(source, "pose_topics"), outputs = SensorTopics(source, "output_topics");
        if (poses.empty() || poses.size() != outputs.size())
            throw std::invalid_argument("original pose/output arrays required");
        if (metadata.backend == "cpu") {
            const int requested = source.get<int>("worker_threads", 1);
            if (requested < 1) throw std::invalid_argument("original CPU worker_threads must be positive");
            equipment.worker_threads = std::min<std::size_t>(requested, poses.size());
        }
        for (std::size_t i = 0; i < poses.size(); ++i) {
            auto inserted = pose_indices.emplace(poses[i], result.poses.size());
            if (inserted.second) result.poses.push_back({poses[i], metadata.pose_type});
            else if (result.poses[inserted.first->second].type != metadata.pose_type)
                throw std::invalid_argument("one source topic cannot declare different ROS types");
            equipment.source_indices.push_back(inserted.first->second);
            if (!output_topics.insert(outputs[i]).second)
                throw std::invalid_argument("duplicate output authority");
            result.outputs.push_back(outputs[i]);
        }
        auto cloud = cloud_indices.emplace(metadata.input_cloud_topic, result.clouds.size());
        if (cloud.second) result.clouds.push_back({metadata.input_cloud_topic, "", {}});
        auto& binding = result.clouds[cloud.first->second];
        if (metadata.backend == "gpu") {
            if (!binding.gpu_frame.empty() && binding.gpu_frame != metadata.frame_id)
                throw std::invalid_argument("same GPU input cannot have two declared world frames");
            binding.gpu_frame = metadata.frame_id;
        }
        binding.groups.push_back(result.configuration.shared.size());
        result.configuration.shared.push_back(std::move(equipment));
    }
    if (result.configuration.shared.empty()) throw std::invalid_argument("frozen shared attachment is empty");
    result.configuration.source_count = result.poses.size();
    return result;
}

} // namespace xgc2_gazebo_scene
