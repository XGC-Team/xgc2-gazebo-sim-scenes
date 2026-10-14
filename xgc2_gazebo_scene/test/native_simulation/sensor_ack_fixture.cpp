#include "xgc2_gazebo_scene/native_sensor_controller.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <map>
#include <mutex>
namespace xgc2_gazebo_scene {
namespace {
std::mutex mutex;
struct Held {
    std::string token;
    std::weak_ptr<gazebo::sensors::Sensor> sensor;
};
std::map<std::string, Held> held;
} // namespace
// Init returns to Gazebo even when the first native sensor ACK is withheld.
class SensorAckFixture final : public gazebo::SensorPlugin {
  public:
    ~SensorAckFixture() override {
        if (!token_.empty())
            ReleaseSimulationSensor(token_);
    }
    void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr config) override {
        sensor_ = sensor;
        token_ = config->Get<std::string>("controller_token");
    }
    void Init() override {
        auto sensor = sensor_.lock();
        if (!sensor)
            throw std::runtime_error("fixture sensor vanished");
        const auto name = sensor->Name();
        const auto root = std::getenv("XGC2_CREATE_ACK_FIXTURE_ROOT");
        if (name == "held_sensor" || name == "stop_sensor") {
            std::lock_guard<std::mutex> lock(mutex);
            held[name] = {token_, sensor};
            std::ofstream(std::filesystem::path(root) / (name + ".init")) << token_;
            return;
        }
        if (name == "release_sensor") {
            Held first;
            {
                std::lock_guard<std::mutex> lock(mutex);
                first = held.at("held_sensor");
                held.erase("held_sensor");
            }
            if (auto first_sensor = first.sensor.lock())
                AcknowledgeSimulationSensor(first.token, first_sensor);
            else
                throw std::runtime_error("held fixture sensor vanished before release");
        }
        AcknowledgeSimulationSensor(token_, sensor);
    }

  private:
    std::weak_ptr<gazebo::sensors::Sensor> sensor_;
    std::string token_;
};
GZ_REGISTER_SENSOR_PLUGIN(SensorAckFixture)
} // namespace xgc2_gazebo_scene
