#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sdf/sdf.hh>

// A short-lived, isolated native sensor instance tests Gazebo's own renderer.
int main() {
    if (!gazebo::setupServer())
        return 1;
    int result = 1;
    try {
        auto document = std::make_shared<sdf::SDF>();
        sdf::init(document);
        if (!sdf::readString(R"(<sdf version='1.6'><world name='ray_probe'>
          <model name='probe'><static>true</static><link name='body'/></model>
        </world></sdf>)",
                             document))
            throw std::runtime_error("invalid probe world");
        auto world = gazebo::physics::create_world("ray_probe");
        world->Load(document->Root()->GetElement("world"));
        world->Init();
        auto sensor_document = std::make_shared<sdf::SDF>();
        sdf::init(sensor_document);
        if (!sdf::readString(R"(<sdf version='1.6'><model name='probe'><link name='body'>
          <sensor name='native_gpu_ray' type='gpu_ray'><always_on>true</always_on>
            <ray><scan><horizontal><samples>8</samples><resolution>1</resolution>
              <min_angle>-0.5</min_angle><max_angle>0.5</max_angle></horizontal></scan>
              <range><min>0.1</min><max>2</max><resolution>0.01</resolution></range></ray>
          </sensor></link></model></sdf>)",
                             sensor_document))
            throw std::runtime_error("invalid probe sensor");
        auto sensor = std::make_shared<gazebo::sensors::GpuRaySensor>();
        sensor->SetParent("ray_probe::probe::body", world->ModelByName("probe")->GetLink("body")->GetId());
        sensor->Load(world->Name(),
                     sensor_document->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
        sensor->Init();
        if (sensor->LaserCamera()) {
            sensor->SetActive(true);
            sensor->Update(true);
            if (sensor->RangeCount() == 8)
                result = 0;
        }
        sensor.reset();
    } catch (...) {
        result = 1;
    }
    gazebo::shutdown();
    return result;
}
