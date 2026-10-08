#pragma once
#include <gazebo/physics/PhysicsTypes.hh>
#include <json/json.h>
#include <memory>
#include <string>
#include <stdexcept>

namespace xgc2_gazebo_scene {
// Adapter boundary only. Native domain functions retain their own typed data;
// no XRPC request/reply, headers, socket or operation IDs enter those functions.
struct PreparedWorldExtension { virtual ~PreparedWorldExtension() = default; };
class NativeExtensionError : public std::runtime_error {
 public:
  NativeExtensionError(int status_, std::string code_, std::string message)
      : std::runtime_error(std::move(message)), status(status_), code(std::move(code_)) {}
  const int status;
  const std::string code;
};
class NativeWorldExtension {
 public:
  virtual ~NativeWorldExtension() = default;
  virtual Json::Value Describe() const = 0;
  // IO owner: strict validation and bounded typed preparation.
  virtual std::shared_ptr<const PreparedWorldExtension> Prepare(
      const std::string& method, const std::string& route, const Json::Value& body) = 0;
  // World's fixed management owner, including while physics is paused.
  virtual Json::Value Execute(const std::shared_ptr<const PreparedWorldExtension>& command,
                             bool* effects_started) = 0;
  // Signal/quiesce native work before the world management executor joins.
  virtual void Stop() noexcept = 0;
};
class WorldExtensionBinding {
 public:
  WorldExtensionBinding(gazebo::physics::WorldPtr world, std::string id,
                        std::shared_ptr<NativeWorldExtension> adapter);
  ~WorldExtensionBinding();
  WorldExtensionBinding(const WorldExtensionBinding&) = delete;
  WorldExtensionBinding& operator=(const WorldExtensionBinding&) = delete;
 private:
  gazebo::physics::WorldPtr world_;
  std::string id_;
  std::shared_ptr<NativeWorldExtension> adapter_;
};
}  // namespace xgc2_gazebo_scene
