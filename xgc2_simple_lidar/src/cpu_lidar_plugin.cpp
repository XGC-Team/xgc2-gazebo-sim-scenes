#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <ros/ros.h>
#include <xgc2_gazebo_scene/native_runtime_logging.hpp>
#include <xgc2_gazebo_scene/ros_master_registration.hpp>

#include "ode_ray_query.hpp"
#include "xgc2_simple_lidar/scan_projection.hpp"
#include "xgc2_simple_lidar/scan_schedule.hpp"

namespace xgc2_simple_lidar {

// This is a native Gazebo sensor. Its ODE ray batch shares the world's actual
// collision geometry, without constructing a full ODECollision (and its SDF
// tree) for each beam. Scan shape and output remain the authored full grid.
class CpuLidarSensor final : public gazebo::sensors::Sensor {
    struct Beam {
        dGeomID ray = nullptr;
        ignition::math::Vector3d direction;
        float range = 0;
    };

  public:
    CpuLidarSensor() : Sensor(gazebo::sensors::OTHER) {}
    ~CpuLidarSensor() override { Fini(); }

    void Load(const std::string& name, sdf::ElementPtr element) override {
        Sensor::Load(name, element);
        if (!ros::isInitialized())
            gzthrow("native CPU lidar requires the ROS data owner");
        physics_ = boost::dynamic_pointer_cast<gazebo::physics::ODEPhysics>(world->Physics());
        if (!physics_)
            gzthrow("native CPU lidar requires Gazebo ODE collision geometry");
        const auto ray = element->GetElement("ray");
        const auto scan = ray->GetElement("scan");
        const auto horizontal = scan->GetElement("horizontal");
        const auto vertical = scan->GetElement("vertical");
        const auto range = ray->GetElement("range");
        const auto width = horizontal->Get<unsigned>("samples");
        const auto height = vertical->Get<unsigned>("samples");
        if (width < 2 || height < 2 || horizontal->Get<double>("resolution") != 1 ||
            vertical->Get<double>("resolution") != 1 || element->Get<double>("update_rate") <= 0)
            gzthrow("native CPU lidar requires 3D rays, scan resolution 1 and a positive rate");
        const double yaw_min = horizontal->Get<double>("min_angle");
        const double yaw_max = horizontal->Get<double>("max_angle");
        const double pitch_min = vertical->Get<double>("min_angle");
        const double pitch_max = vertical->Get<double>("max_angle");
        range_min_ = range->Get<double>("min");
        range_max_ = range->Get<double>("max");
        projection_ = std::make_unique<ScanProjection>(width, height, yaw_min, yaw_max, pitch_min, pitch_max,
                                                       range_min_, range_max_, false, 1);
        beams_.resize(static_cast<std::size_t>(width) * height);
        ranges_.resize(beams_.size());
        ray_space_ = dSimpleSpaceCreate(nullptr);
        dSpaceSetCleanup(ray_space_, 0);
        ray_query_.space = ray_space_;
        ray_query_.minimum_range = range_min_;
        dGeomSetCategoryBits(reinterpret_cast<dGeomID>(ray_space_), GZ_SENSOR_COLLIDE);
        dGeomSetCollideBits(reinterpret_cast<dGeomID>(ray_space_), ~GZ_SENSOR_COLLIDE);
        for (unsigned j = 0; j < height; ++j) {
            const double pitch = pitch_min + (pitch_max - pitch_min) * j / (height - 1);
            for (unsigned i = 0; i < width; ++i) {
                auto& beam = beams_[static_cast<std::size_t>(j) * width + i];
                const double yaw = yaw_min + (yaw_max - yaw_min) * i / (width - 1);
                beam.direction = {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
                beam.ray = dCreateRay(ray_space_, range_max_ - range_min_);
                dGeomSetData(beam.ray, &beam.range);
                dGeomSetCategoryBits(beam.ray, GZ_SENSOR_COLLIDE);
                dGeomSetCollideBits(beam.ray, ~GZ_SENSOR_COLLIDE);
                dGeomRaySetParams(beam.ray, 0, 0);
                dGeomRaySetClosestHit(beam.ray, 1);
            }
        }
        const double period = 1.0 / element->Get<double>("update_rate");
        schedule_ = ScanSchedule(period, ScanSchedule::SensorPhase(period, ParentName()));
        // The SensorManager still owns lifecycle and the ACK plugin. Sampling
        // belongs to WorldUpdateEnd so it never races body mutation/deletion.
        SetActive(false);
        const auto output = Topic();
        if (output.empty())
            gzthrow("native CPU lidar requires an explicit output topic");
        registration_ = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([this, output] {
            node_ = std::make_unique<ros::NodeHandle>();
            publisher_ = node_->advertise<sensor_msgs::PointCloud2>(output, 1);
            if (!publisher_)
                throw std::runtime_error("native CPU lidar ROS registration failed");
            ready_.store(true, std::memory_order_release);
        });
    }

    void Init() override {
        Sensor::Init();
        const std::weak_ptr<gazebo::sensors::Sensor> weak = shared_from_this();
        frame_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd([weak] {
            if (auto sensor = weak.lock())
                static_cast<CpuLidarSensor*>(sensor.get())->Frame();
        });
    }

    void Fini() override {
        frame_connection_.reset();
        closing_ = true;
        registration_.reset();
        std::lock_guard<std::mutex> lock(frame_mutex_);
        publisher_.shutdown();
        for (auto& beam : beams_)
            if (beam.ray) {
                dGeomDestroy(beam.ray);
                beam.ray = nullptr;
            }
        if (ray_space_) {
            dSpaceDestroy(ray_space_);
            ray_space_ = nullptr;
        }
        beams_.clear();
        Sensor::Fini();
    }

  private:
    static void Intersect(void* context, dGeomID first, dGeomID second) {
        auto* sensor = static_cast<CpuLidarSensor*>(context);
        OdeRayQuery::Intersect(&sensor->ray_query_, first, second);
    }

    void ExcludeEmitter(const gazebo::physics::ModelPtr& model) {
        for (const auto& link : model->GetLinks())
            for (const auto& collision : link->GetCollisions())
                if (auto ode = boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(collision))
                    ray_query_.emitter_collisions.insert(ode->GetCollisionId());
        for (const auto& nested : model->NestedModels())
            ExcludeEmitter(nested);
    }

    void Frame() {
        if (closing_ || !ready_.load(std::memory_order_acquire) || publisher_.getNumSubscribers() == 0)
            return;
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (closing_)
            return;
        const auto stamp = world->SimTime();
        if (!schedule_.Due(stamp.Double()))
            return;
        auto parent = world->EntityByName(ParentName());
        if (!parent)
            return;
        schedule_.Taken(stamp.Double());
        const auto measured = Pose() + parent->WorldPose();
        const auto& rotation = measured.Rot();
        const auto& origin = measured.Pos();
        for (auto& beam : beams_) {
            const auto direction = rotation.RotateVector(beam.direction);
            const auto start = origin + direction * range_min_;
            beam.range = static_cast<float>(range_max_);
            dGeomRaySet(beam.ray, start.X(), start.Y(), start.Z(), direction.X(), direction.Y(), direction.Z());
        }
        {
            boost::recursive_mutex::scoped_lock physics_lock(*physics_->GetPhysicsUpdateMutex());
            // Resolve the emitting MODEL on each scan. Its links (including
            // nested model links) are not obstacles; other models remain rays'
            // collision targets even when they are very close to the sensor.
            const auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(parent);
            if (!link || !link->GetModel())
                return;
            ray_query_.emitter_collisions.clear();
            ExcludeEmitter(link->GetModel());
            dSpaceCollide2(reinterpret_cast<dGeomID>(ray_space_), reinterpret_cast<dGeomID>(physics_->GetSpaceId()),
                           this, &Intersect);
        }
        for (std::size_t i = 0; i < beams_.size(); ++i)
            ranges_[i] = beams_[i].range;
        lastMeasurementTime = stamp;
        publisher_.publish(projection_->Project(ranges_.data(), measured, ros::Time(stamp.sec, stamp.nsec)));
    }

    boost::shared_ptr<gazebo::physics::ODEPhysics> physics_;
    dSpaceID ray_space_ = nullptr;
    OdeRayQuery ray_query_;
    std::vector<Beam> beams_;
    std::vector<float> ranges_;
    double range_min_ = 0, range_max_ = 0;
    std::unique_ptr<ScanProjection> projection_;
    ScanSchedule schedule_{0.1};
    std::unique_ptr<ros::NodeHandle> node_;
    ros::Publisher publisher_;
    std::unique_ptr<xgc2_gazebo_scene::RosMasterRegistration> registration_;
    gazebo::event::ConnectionPtr frame_connection_;
    std::mutex frame_mutex_;
    std::atomic<bool> closing_{false}, ready_{false};
};

class CpuLidarRegistration final : public gazebo::SystemPlugin {
  public:
    void Load(int, char**) override {
        xgc2_gazebo_scene::ConfigureNativeServerLogging();
        gazebo::sensors::SensorFactory::RegisterSensor("xgc2_cpu_lidar", []() -> gazebo::sensors::Sensor* {
            return new CpuLidarSensor();
        });
    }
};
GZ_REGISTER_SYSTEM_PLUGIN(CpuLidarRegistration)
} // namespace xgc2_simple_lidar
