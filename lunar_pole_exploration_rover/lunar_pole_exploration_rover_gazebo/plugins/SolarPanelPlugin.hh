#ifndef SOLAR_PANEL_PLUGIN_HH_
#define SOLAR_PANEL_PLUGIN_HH_

#include <string>
#include <mutex>
#include <memory>

#include <gz/sim/System.hh>
#include <ignition/rendering/Scene.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

  // Forward declaration
  class SolarPanelPluginPrivate;

  /// \brief SolarPanelPlugin SolarPanelPlugin.hh
  /// \brief A plugin that simulates a solar panel
  class SolarPanelPlugin : public System,
                           public ISystemConfigure,
                           public ISystemPostUpdate
  {

    /// \brief Constructor
  public:
    SolarPanelPlugin();

    /// \brief Destructor
  public:
    ~SolarPanelPlugin() override;

    /// Documentation inherited
  public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    /// Documentation inherited
  public:
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Set the scene
  public:
    void SetScene(gz::rendering::ScenePtr _scene);

    /// \brief Private data pointer
  private:
    std::unique_ptr<SolarPanelPluginPrivate> dataPtr;
  };

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif // SOLAR_PANEL_PLUGIN_HH_
