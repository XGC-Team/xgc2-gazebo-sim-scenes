#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/ogre_gazebo.h>

namespace xgc2_gazebo_rendering {
// Gazebo Classic stores shadow depth in a floating-point colour texture. A pass
// depth bias affects the depth buffer, not the value compared by the receiver.
// Bias that stored value by the light-space surface slope instead.
class StableShadows : public gazebo::SystemPlugin {
 public:
  void Load(int, char**) override {}
  void Init() override {
    connection_ = gazebo::event::Events::ConnectPreRender([this]() { Update(); });
  }
 private:
  void Update() {
    auto* engine = gazebo::rendering::RenderEngine::Instance();
    if (!engine || !engine->SceneCount()) return;
    auto& manager = Ogre::MaterialManager::getSingleton();
    auto material = manager.getByName("XGC2/ShadowCaster");
    if (material.isNull()) {
      const auto base = manager.getByName("Gazebo/shadow_caster");
      if (base.isNull()) return;
      auto program = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
          "XGC2/ShadowDepth", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
      program->setSource(R"glsl(
        varying vec4 vertex_depth;
        void main() {
          float depth = vertex_depth.z / vertex_depth.w;
          float slope = max(abs(dFdx(depth)), abs(dFdy(depth)));
          float bias = max(0.00002, 2.0 * slope);
          gl_FragColor = vec4(vec3(depth + bias), 1.0);
        }
      )glsl");
      program->load();
      material = base->clone("XGC2/ShadowCaster");
      material->getTechnique(0)->getPass(0)->setFragmentProgram("XGC2/ShadowDepth");
      material->load();
    }
    // Shadow setup may be recreated when lights or cameras change. Apply to all
    // rendering scenes, including server-side image sensors and the GUI scene.
    for (unsigned i = 0; i < engine->SceneCount(); ++i) {
      auto scene = engine->GetScene(i);
      if (scene && scene->OgreSceneManager())
        scene->OgreSceneManager()->setShadowTextureCasterMaterial("XGC2/ShadowCaster");
    }
  }
  gazebo::event::ConnectionPtr connection_;
};
GZ_REGISTER_SYSTEM_PLUGIN(StableShadows)
}  // namespace xgc2_gazebo_rendering
