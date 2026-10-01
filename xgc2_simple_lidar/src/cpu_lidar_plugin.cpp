#include <atomic>
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
#include "xgc2_simple_lidar/scan_schedule.hpp"

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
        ScanSchedule schedule{0.1};
        std::atomic<bool> closing{false};
        // Kept by the connect and disconnect callbacks (ROS spinner threads),
        // which read the count and store it as one step under the mutex.
        // Frame() runs after every world update and reads only this flag:
        // Publisher::getNumSubscribers searches every topic the gzserver
        // process advertises under roscpp's global topic lock.
        std::mutex subscriber_mutex;
        std::atomic<bool> subscribed{false};

        void Stop() {
            closing = true;
            const std::scoped_lock lock(frame_mutex, subscriber_mutex);
            publisher.shutdown();
        }

        void UpdateSubscribed() {
            const std::lock_guard<std::mutex> lock(subscriber_mutex);
            subscribed = publisher.getNumSubscribers() > 0;
        }

        void Frame() {
            if (closing || !subscribed)
                return;
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (closing)
                return;
            // Updates between scans stop here. The sensor and its parent are
            // looked up only when a scan is due: EntityByName walks the
            // world's entity tree and copies every scoped name on the way.
            const double now = world->SimTime().Double();
            if (!schedule.Due(now))
                return;
            auto current = sensor.lock();
            if (!current)
                return;
            auto parent = world->EntityByName(current->ParentName());
            if (!parent)
                return;
            schedule.Taken(now);
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
        state->schedule = ScanSchedule(1.0 / cpu->UpdateRate());
        state->projection = std::make_unique<ScanProjection>(
            width, height, cpu->AngleMin().Radian(), cpu->AngleMax().Radian(), cpu->VerticalAngleMin().Radian(),
            cpu->VerticalAngleMax().Radian(), cpu->RangeMin(), cpu->RangeMax(), false, 1);
        state->ranges.resize(static_cast<std::size_t>(width) * height);
        const auto robot_namespace =
            config->HasElement("robotNamespace") ? config->Get<std::string>("robotNamespace") : "";
        state->node = std::make_unique<ros::NodeHandle>(robot_namespace);
        const std::weak_ptr<State> weak = state;
        const auto subscribers = [weak](const ros::SingleSubscriberPublisher&) {
            if (auto current = weak.lock())
                current->UpdateSubscribed();
        };
        auto options = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            "simple_lidar/points", 1, subscribers, subscribers, ros::VoidConstPtr(), nullptr);
        {
            // A subscriber may connect, and its callback run, before advertise
            // returns; the callback reads the publisher under the same lock.
            const std::lock_guard<std::mutex> lock(state->subscriber_mutex);
            state->publisher = state->node->advertise(options);
        }
        state->UpdateSubscribed();
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
