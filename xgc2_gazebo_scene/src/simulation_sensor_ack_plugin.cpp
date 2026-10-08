#include "xgc2_gazebo_scene/native_sensor_controller.hpp"
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
namespace xgc2_gazebo_scene {
class SimulationSensorAckPlugin final : public gazebo::SensorPlugin {
 public:
  ~SimulationSensorAckPlugin() override { if (!token_.empty()) ReleaseSimulationSensor(token_); }
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr element) override {
    sensor_ = sensor;
    if (element && element->HasElement("controller_token")) token_ = element->Get<std::string>("controller_token");
  }
  void Init() override {
    if (auto sensor = sensor_.lock()) AcknowledgeSimulationSensor(token_, std::move(sensor));
  }
 private:
  std::weak_ptr<gazebo::sensors::Sensor> sensor_;
  std::string token_;
};
GZ_REGISTER_SENSOR_PLUGIN(SimulationSensorAckPlugin)
}
