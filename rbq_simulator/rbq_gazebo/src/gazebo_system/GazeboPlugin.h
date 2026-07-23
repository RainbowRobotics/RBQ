#ifndef RBQ_GAZEBO_SYSTEM__RBQ_GAZEBO_PLUGIN_HPP_
#define RBQ_GAZEBO_SYSTEM__RBQ_GAZEBO_PLUGIN_HPP_

#include <memory>
#include <string>

#include <gz/sim/Server.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Imu.hh>

#include "gazebo_system/GazeboSystemWrapper.h"

namespace rbq_gazebo_system
{

class GazeboPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  GazeboPlugin();
  ~GazeboPlugin() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override;

private:
  class Implementation;
  std::unique_ptr<Implementation> dataPtr;
};

bool addToServer(gz::sim::Server &server);

}  // namespace rbq_gazebo_system

#endif  // RBQ_GAZEBO_SYSTEM__RBQ_GAZEBO_PLUGIN_HPP_
