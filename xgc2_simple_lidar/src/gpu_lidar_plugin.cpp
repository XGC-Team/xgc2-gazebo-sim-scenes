#include <atomic>
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
        // The sensor owns this plugin, and the plugin owns this state. A weak
        // reference keeps that ownership one-way; a callback that outlives the
        // sensor finds it expired instead of keeping it alive.
        std::weak_ptr<gazebo::sensors::GpuRaySensor> sensor;
        std::string label;
        std::unique_ptr<ros::NodeHandle> node;
        ros::Publisher publisher;
        std::unique_ptr<ScanProjection> projection;
        unsigned width{0}, height{0};
        std::atomic<bool> layout_error_reported{false};
        ~State() {
            if (const auto current = sensor.lock())
                current->SetActive(false);
            publisher.shutdown();
        }
        // Every rendered frame is projected and published. Whether anyone
        // subscribes never decides whether the sensor scans: a subscriber
        // dependent sensor hides its cost until the first consumer connects.
        void Frame(const float* ranges, unsigned columns, unsigned rows, unsigned depth) {
            if (columns != width || rows != height || depth != 3) {
                if (!layout_error_reported.exchange(true))
                    ROS_ERROR_STREAM("simple lidar " << label << " received a " << columns << "x" << rows << "x"
                                                     << depth << " GPU frame; expected " << width << "x" << height
                                                     << "x3");
                return;
            }
            const auto current = sensor.lock();
            if (!current)
                return;
            // Both are read in Gazebo's rendering callback before the next
            // PreRender. The camera pose already contains sensor mounting pose.
            const auto stamp = current->LastMeasurementTime();
            const auto pose = current->LaserCamera()->WorldPose();
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
        const auto robot_namespace =
            config->HasElement("robotNamespace") ? config->Get<std::string>("robotNamespace") : "";
        const auto label = robot_namespace + " (" + (sensor ? sensor->ScopedName() : std::string("no sensor")) + ")";
        auto gpu = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(sensor);
        // The sensor renders only after this plugin is complete. A plugin that
        // cannot load leaves it inactive whatever the SDF always_on says, so
        // no frame is rendered that nothing projects.
        if (gpu)
            gpu->SetActive(false);
        if (!gpu || !ros::isInitialized()) {
            gzerr << "simple lidar " << label << " requires a gpu_ray sensor and gazebo_ros_api_plugin\n";
            return;
        }
        auto state = std::make_shared<State>();
        state->sensor = gpu;
        state->label = label;
        state->width = gpu->RangeCount();
        state->height = gpu->VerticalRangeCount();
        if (state->width < 2 || state->height < 2 || gpu->RayCount() != static_cast<int>(state->width) ||
            gpu->VerticalRayCount() != static_cast<int>(state->height)) {
            gzerr << "simple lidar " << label << " requires 3D rays with scan resolution 1\n";
            return;
        }
        state->projection = std::make_unique<ScanProjection>(
            state->width, state->height, gpu->AngleMin().Radian(), gpu->AngleMax().Radian(),
            gpu->VerticalAngleMin().Radian(), gpu->VerticalAngleMax().Radian(), gpu->RangeMin(), gpu->RangeMax());
        state->node = std::make_unique<ros::NodeHandle>(robot_namespace);
        const std::weak_ptr<State> weak_state = state;
        state->publisher = state->node->advertise<sensor_msgs::PointCloud2>("simple_lidar/points", 1);
        // GpuRaySensor::Fini removes its camera before destroying sensor plugins.
        // Keep the event source alive until this connection has been disconnected.
        camera_ = gpu->LaserCamera();
        frame_connection_ = camera_->ConnectNewLaserFrame(
            [weak_state](const float* ranges, unsigned width, unsigned height, unsigned depth, const std::string&) {
                if (auto current = weak_state.lock())
                    current->Frame(ranges, width, height, depth);
            });
        state_ = std::move(state);
        gpu->SetActive(true);
    }

  private:
    gazebo::rendering::GpuLaserPtr camera_;
    gazebo::event::ConnectionPtr frame_connection_;
    std::shared_ptr<State> state_;
};

GZ_REGISTER_SENSOR_PLUGIN(GpuLidarPlugin)

} // namespace xgc2_simple_lidar
