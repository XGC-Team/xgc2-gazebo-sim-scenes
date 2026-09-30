#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ros/ros.h>

#include "xgc2_simple_lidar/scan_projection.hpp"

namespace xgc2_simple_lidar {

class CpuLidarPlugin final : public gazebo::SensorPlugin {
    struct State {
        std::weak_ptr<gazebo::sensors::RaySensor> sensor;
        gazebo::physics::WorldPtr world;
        std::unique_ptr<ros::NodeHandle> node;
        ros::Publisher publisher;
        std::unique_ptr<ScanProjection> projection;
        std::vector<double> native_ranges;
        std::vector<float> ranges;
        std::mutex frame_mutex;
        double period{0.1}, next_scan{0}, previous_time{0};
        std::atomic<bool> closing{false};

        void Stop() {
            closing = true;
            std::lock_guard<std::mutex> lock(frame_mutex);
            publisher.shutdown();
        }

        void Frame() {
            if (closing || !publisher.getNumSubscribers())
                return;
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (closing)
                return;
            auto current = sensor.lock();
            if (!current)
                return;
            auto parent = world->EntityByName(current->ParentName());
            if (!parent)
                return;
            const double now = world->SimTime().Double();
            if (now < previous_time)
                next_scan = now;
            previous_time = now;
            if (now + 1e-9 < next_scan)
                return;
            const double elapsed = std::max(0.0, now - next_scan);
            next_scan += (std::floor(elapsed / period) + 1) * period;
            // RaySensor's background worker stays inactive (no Gazebo scan
            // transport subscriber). Updating in WorldUpdateEnd avoids the
            // SensorContainer -> physics / model deletion -> SensorContainer
            // lock inversion in Gazebo Classic. Physics and measurement pose
            // are sampled on this one world thread; delivery may be delayed.
            current->Update(true);
            current->Ranges(native_ranges);
            if (native_ranges.size() != ranges.size()) {
                ROS_ERROR_ONCE("simple lidar received an unexpected CPU frame layout");
                return;
            }
            for (std::size_t i = 0; i < ranges.size(); ++i)
                ranges[i] = static_cast<float>(native_ranges[i]);
            const auto pose = current->Pose() + parent->WorldPose();
            const auto stamp = current->LastMeasurementTime();
            publisher.publish(projection->Project(ranges.data(), pose, ros::Time(stamp.sec, stamp.nsec)));
        }
    };

  public:
    ~CpuLidarPlugin() override {
        frame_connection_.reset();
        if (state_)
            state_->Stop();
        state_.reset();
    }

    void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr config) override {
        auto cpu = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
        if (!cpu || !ros::isInitialized()) {
            gzerr << "simple CPU lidar requires a ray sensor and gazebo_ros_api_plugin\n";
            return;
        }
        cpu->SetActive(false);
        const auto width = cpu->RangeCount(), height = cpu->VerticalRangeCount();
        if (width < 2 || height < 2 || cpu->RayCount() != width || cpu->VerticalRayCount() != height ||
            cpu->UpdateRate() <= 0) {
            gzerr << "simple CPU lidar requires 3D rays, scan resolution 1 and a positive rate\n";
            return;
        }
        auto state = std::make_shared<State>();
        state->sensor = cpu;
        state->world = gazebo::physics::get_world(cpu->WorldName());
        state->period = 1.0 / cpu->UpdateRate();
        state->projection = std::make_unique<ScanProjection>(
            width, height, cpu->AngleMin().Radian(), cpu->AngleMax().Radian(), cpu->VerticalAngleMin().Radian(),
            cpu->VerticalAngleMax().Radian(), cpu->RangeMin(), cpu->RangeMax(), false, 1);
        state->ranges.resize(static_cast<std::size_t>(width) * height);
        const auto robot_namespace =
            config->HasElement("robotNamespace") ? config->Get<std::string>("robotNamespace") : "";
        state->node = std::make_unique<ros::NodeHandle>(robot_namespace);
        state->publisher = state->node->advertise<sensor_msgs::PointCloud2>("simple_lidar/points", 1);
        const std::weak_ptr<State> weak = state;
        frame_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd([weak]() {
            if (auto current = weak.lock())
                current->Frame();
        });
        state_ = std::move(state);
    }

  private:
    gazebo::event::ConnectionPtr frame_connection_;
    std::shared_ptr<State> state_;
};

GZ_REGISTER_SENSOR_PLUGIN(CpuLidarPlugin)

} // namespace xgc2_simple_lidar
