#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/GpuLaser.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <ros/ros.h>

#include "xgc2_simple_lidar/scan_projection.hpp"

namespace xgc2_simple_lidar {

class GpuLidarPlugin final : public gazebo::SensorPlugin {
    // Callback work owns this state for its duration. Deleting a model need not
    // wait on the ROS callback queue or leave a callback pointing at the plugin.
    struct State {
        gazebo::sensors::GpuRaySensorPtr sensor;
        std::unique_ptr<ros::NodeHandle> node;
        ros::Publisher publisher;
        std::unique_ptr<ScanProjection> projection;
        unsigned width{0}, height{0};
        ~State() {
            sensor->SetActive(false);
            publisher.shutdown();
        }
        void UpdateActivity() { sensor->SetActive(publisher.getNumSubscribers() > 0); }
        void Frame(const float* ranges, unsigned columns, unsigned rows, unsigned depth) {
            if (!publisher.getNumSubscribers())
                return;
            if (columns != width || rows != height || depth != 3) {
                ROS_ERROR_ONCE("simple lidar received an unexpected GPU frame layout");
                return;
            }
            // Both are read in Gazebo's rendering callback before the next
            // PreRender. The camera pose already contains sensor mounting pose.
            const auto stamp = sensor->LastMeasurementTime();
            const auto pose = sensor->LaserCamera()->WorldPose();
            publisher.publish(projection->Project(ranges, pose, ros::Time(stamp.sec, stamp.nsec)));
        }
    };

  public:
    ~GpuLidarPlugin() override {
        frame_connection_.reset();
        state_.reset();
        camera_.reset();
    }

    void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr config) override {
        auto gpu = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(sensor);
        if (!gpu || !ros::isInitialized()) {
            gzerr << "simple lidar requires a gpu_ray sensor and gazebo_ros_api_plugin\n";
            return;
        }
        gpu->SetActive(false);
        auto state = std::make_shared<State>();
        state->sensor = gpu;
        state->width = gpu->RangeCount();
        state->height = gpu->VerticalRangeCount();
        if (state->width < 2 || state->height < 2 || gpu->RayCount() != static_cast<int>(state->width) ||
            gpu->VerticalRayCount() != static_cast<int>(state->height)) {
            gzerr << "simple lidar requires 3D rays with scan resolution 1\n";
            return;
        }
        state->projection = std::make_unique<ScanProjection>(
            state->width, state->height, gpu->AngleMin().Radian(), gpu->AngleMax().Radian(),
            gpu->VerticalAngleMin().Radian(), gpu->VerticalAngleMax().Radian(), gpu->RangeMin(), gpu->RangeMax());
        const auto robot_namespace =
            config->HasElement("robotNamespace") ? config->Get<std::string>("robotNamespace") : "";
        state->node = std::make_unique<ros::NodeHandle>(robot_namespace);
        const std::weak_ptr<State> weak_state = state;
        const auto activity = [weak_state](const ros::SingleSubscriberPublisher&) {
            if (auto current = weak_state.lock())
                current->UpdateActivity();
        };
        auto options = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>("simple_lidar/points", 1, activity,
                                                                               activity, ros::VoidConstPtr(), nullptr);
        state->publisher = state->node->advertise(options);
        state->UpdateActivity();
        // GpuRaySensor::Fini removes its camera before destroying sensor plugins.
        // Keep the event source alive until this connection has been disconnected.
        camera_ = gpu->LaserCamera();
        frame_connection_ = camera_->ConnectNewLaserFrame(
            [weak_state](const float* ranges, unsigned width, unsigned height, unsigned depth, const std::string&) {
                if (auto current = weak_state.lock())
                    current->Frame(ranges, width, height, depth);
            });
        state_ = std::move(state);
    }

  private:
    gazebo::rendering::GpuLaserPtr camera_;
    gazebo::event::ConnectionPtr frame_connection_;
    std::shared_ptr<State> state_;
};

GZ_REGISTER_SENSOR_PLUGIN(GpuLidarPlugin)

} // namespace xgc2_simple_lidar
