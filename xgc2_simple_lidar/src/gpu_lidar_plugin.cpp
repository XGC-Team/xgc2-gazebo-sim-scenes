#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xgc2_simple_lidar/scan_projection.hpp"
#include "xgc2_simple_lidar/scan_schedule.hpp"
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <ros/ros.h>
#include <xgc2_gazebo_scene/ros_master_registration.hpp>

namespace xgc2_simple_lidar {
// Depth-only face: no RGB target and no six redundant Camera image readbacks.
class RangeFace final : public gazebo::rendering::DepthCamera {
  public:
    RangeFace(const std::string &name, const gazebo::rendering::ScenePtr &scene) : DepthCamera(name, scene, false) {}
    void Prepare(sdf::ElementPtr config, const std::string &name) {
        Load(config);
        DepthCamera::Init();
        texture_name_ = name + "_Depth";
        material_name_ = Name() + "_RttMat_Camera_Depth";
        CreateDepthTexture(texture_name_);
        pixels_.resize(static_cast<std::size_t>(ImageWidth()) * ImageHeight());
        initialized = true;
    }
    void PostRender() override {
        if (!newData)
            return;
        depthTarget->swapBuffers();
        Ogre::PixelBox box(ImageWidth(), ImageHeight(), 1, Ogre::PF_FLOAT32_R, pixels_.data());
        depthTexture->getBuffer()->blitToMemory(box);
        lastRenderWallTime = gazebo::common::Time::GetWallTime();
        newData = false;
    }
    const float *Ranges() const { return pixels_.data(); }
    void Fini() override {
        if (!texture_name_.empty()) {
            depthTarget->removeAllViewports();
            Ogre::TextureManager::getSingleton().remove(texture_name_);
            Ogre::MaterialManager::getSingleton().remove(material_name_);
            texture_name_.clear();
            depthTarget = nullptr;
            depthTexture = nullptr;
        }
        DepthCamera::Fini();
    }
    ~RangeFace() override { Fini(); }

  private:
    std::string texture_name_, material_name_;
    std::vector<float> pixels_;
};
class GpuLidarSensor final : public gazebo::sensors::Sensor {
    struct Beam {
        unsigned face, pixel;
        float forward;
    };
    struct Face {
        std::unique_ptr<RangeFace> camera;
        ignition::math::Quaterniond rotation;
    };

  public:
    GpuLidarSensor() : Sensor(gazebo::sensors::IMAGE) {}
    ~GpuLidarSensor() override { Fini(); }
    void Load(const std::string &name, sdf::ElementPtr element) override {
        Sensor::Load(name, element);
        if (!ros::isInitialized())
            gzthrow("native GPU lidar requires the ROS data owner");
        auto scan = element->GetElement("ray")->GetElement("scan");
        auto horizontal = scan->GetElement("horizontal"), vertical = scan->GetElement("vertical");
        auto range = element->GetElement("ray")->GetElement("range");
        const unsigned width = horizontal->Get<unsigned>("samples"), height = vertical->Get<unsigned>("samples");
        const double rate = element->Get<double>("update_rate");
        if (width < 2 || height < 2 || rate <= 0 || horizontal->Get<double>("resolution") != 1 ||
            vertical->Get<double>("resolution") != 1)
            gzthrow("native GPU lidar requires 3D rays, scan resolution 1 and a positive rate");
        const double yaw_min = horizontal->Get<double>("min_angle"), yaw_max = horizontal->Get<double>("max_angle");
        const double pitch_min = vertical->Get<double>("min_angle"), pitch_max = vertical->Get<double>("max_angle");
        min_range_ = range->Get<double>("min");
        max_range_ = range->Get<double>("max");
        // Keep each cube-face pixel within a quarter of the smallest authored
        // angular cell. Output still contains exactly the authored beam grid.
        const double step = std::min((yaw_max - yaw_min) / (width - 1), (pitch_max - pitch_min) / (height - 1));
        size_ = std::max(2u, static_cast<unsigned>(std::ceil(1.0 / std::tan(step / 8))));
        faces_[0].rotation = ignition::math::Quaterniond(0, 0, 0);
        faces_[1].rotation = ignition::math::Quaterniond(0, 0, M_PI);
        faces_[2].rotation = ignition::math::Quaterniond(0, 0, M_PI_2);
        faces_[3].rotation = ignition::math::Quaterniond(0, 0, -M_PI_2);
        faces_[4].rotation = ignition::math::Quaterniond(0, -M_PI_2, 0);
        faces_[5].rotation = ignition::math::Quaterniond(0, M_PI_2, 0);
        beams_.reserve(static_cast<std::size_t>(width) * height);
        for (unsigned j = 0; j < height; ++j)
            for (unsigned i = 0; i < width; ++i) {
                const double pitch = pitch_min + (pitch_max - pitch_min) * j / (height - 1);
                const double yaw = yaw_min + (yaw_max - yaw_min) * i / (width - 1);
                const ignition::math::Vector3d d(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
                                                 std::sin(pitch));
                unsigned f = 0;
                double forward = -1;
                for (unsigned k = 0; k < 6; ++k) {
                    const auto local = faces_[k].rotation.RotateVectorReverse(d);
                    if (local.X() > forward) {
                        forward = local.X();
                        f = k;
                    }
                }
                const auto local = faces_[f].rotation.RotateVectorReverse(d);
                const auto pixel = [&](double value) {
                    return std::min(size_ - 1,
                                    static_cast<unsigned>(std::max(0.0, std::floor((value + 1) * 0.5 * size_))));
                };
                beams_.push_back({f, pixel(-local.Z() / local.X()) * size_ + pixel(-local.Y() / local.X()),
                                  static_cast<float>(local.X())});
            }
        ranges_.resize(beams_.size());
        projection_ = std::make_unique<ScanProjection>(width, height, yaw_min, yaw_max, pitch_min, pitch_max,
                                                       min_range_, max_range_, false, 1);
        const double period = 1 / rate;
        schedule_ = ScanSchedule(period, ScanSchedule::SensorPhase(period, ParentName()));
        const auto output = Topic();
        if (output.empty())
            gzthrow("native GPU lidar requires an explicit output topic");
        registration_ = std::make_unique<xgc2_gazebo_scene::RosMasterRegistration>([this, output] {
            node_ = std::make_unique<ros::NodeHandle>();
            publisher_ = node_->advertise<sensor_msgs::PointCloud2>(output, 1);
            if (!publisher_)
                throw std::runtime_error("native GPU lidar ROS registration failed");
            ready_.store(true, std::memory_order_release);
        });
    }
    void Init() override {
        if (gazebo::rendering::RenderEngine::Instance()->GetRenderPathType() == gazebo::rendering::RenderEngine::NONE)
            gzthrow("native GPU lidar requires hardware rendering");
        scene = gazebo::rendering::get_scene(world->Name());
        if (!scene)
            scene = gazebo::rendering::create_scene(world->Name(), false, true);
        if (!scene)
            gzthrow("native GPU lidar render scene unavailable");
        for (unsigned i = 0; i < 6; ++i) {
            auto &face = faces_[i];
            const auto name = ScopedName() + "_face_" + std::to_string(i);
            face.camera = std::make_unique<RangeFace>(name, scene);
            auto config = std::make_shared<sdf::Element>();
            sdf::initFile("camera.sdf", config);
            config->GetElement("horizontal_fov")->Set(M_PI_2);
            auto img = config->GetElement("image");
            img->GetElement("width")->Set(size_);
            img->GetElement("height")->Set(size_);
            auto clip = config->GetElement("clip");
            clip->GetElement("near")->Set(min_range_ / std::sqrt(3.0));
            clip->GetElement("far")->Set(max_range_);
            config->GetElement("depth_camera")->GetElement("output")->Set(std::string("depth"));
            face.camera->Prepare(config, name);
        }
        Sensor::Init();
        SetActive(true);
        const std::weak_ptr<gazebo::sensors::Sensor> weak = shared_from_this();
        frame_ = gazebo::event::Events::ConnectRender([weak] {
            if (auto s = weak.lock())
                static_cast<GpuLidarSensor *>(s.get())->Frame();
        });
    }
    void Fini() override {
        frame_.reset();
        closing_ = true;
        registration_.reset();
        std::lock_guard<std::mutex> lock(mutex_);
        publisher_.shutdown();
        for (auto &face : faces_)
            if (face.camera) {
                face.camera.reset();
            }
        Sensor::Fini();
    }

  private:
    void Frame() {
        if (closing_ || !ready_.load(std::memory_order_acquire) || publisher_.getNumSubscribers() == 0)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (closing_)
            return;
        const auto stamp = scene->SimTime();
        if (!schedule_.Due(stamp.Double()))
            return;
        auto parent = world->EntityByName(ParentName());
        if (!parent)
            return;
        schedule_.Taken(stamp.Double());
        const auto measured = Pose() + parent->WorldPose();
        for (auto &face : faces_) {
            face.camera->SetWorldPose({measured.Pos(), measured.Rot() * face.rotation});
            face.camera->Render();
            face.camera->PostRender();
        }
        for (std::size_t i = 0; i < beams_.size(); ++i) {
            const auto &b = beams_[i];
            const auto *depths = faces_[b.face].camera->Ranges();
            ranges_[i] = depths ? depths[b.pixel] / b.forward : static_cast<float>(max_range_);
        }
        lastMeasurementTime = stamp;
        publisher_.publish(projection_->Project(ranges_.data(), measured, ros::Time(stamp.sec, stamp.nsec)));
    }
    std::array<Face, 6> faces_;
    std::vector<Beam> beams_;
    std::vector<float> ranges_;
    unsigned size_ = 0;
    double min_range_ = 0, max_range_ = 0;
    std::unique_ptr<ScanProjection> projection_;
    ScanSchedule schedule_{0.1};
    std::unique_ptr<ros::NodeHandle> node_;
    ros::Publisher publisher_;
    std::unique_ptr<xgc2_gazebo_scene::RosMasterRegistration> registration_;
    gazebo::event::ConnectionPtr frame_;
    std::mutex mutex_;
    std::atomic<bool> ready_{false}, closing_{false};
};
class GpuLidarRegistration final : public gazebo::SystemPlugin {
  public:
    void Load(int, char **) override {
        gazebo::sensors::SensorFactory::RegisterSensor(
            "xgc2_gpu_lidar", []() -> gazebo::sensors::Sensor * { return new GpuLidarSensor(); });
    }
};
GZ_REGISTER_SYSTEM_PLUGIN(GpuLidarRegistration)
} // namespace xgc2_simple_lidar
