#include "gazebo_system/GazeboPlugin.h"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/Model.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/boolean.pb.h>

#include "Bridge.h"

#include <cstdio>
#include <mutex>

namespace rbq_gazebo_system
{

class GazeboPlugin::Implementation
{
public:
  // Robot variant; set by addToServer() before the server runs.
  rbq_gazebo::RobotSpec spec{rbq_gazebo::rbqSpec()};
  int numJoints{static_cast<int>(spec.joints.size())};

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  std::unique_ptr<GazeboSystemWrapper> wrapper;
  std::vector<std::string> jointNames;
  gz::sim::Entity imuEntity{gz::sim::kNullEntity};
  bool initialized{false};
  bool servicesAdvertised{false};
  gz::transport::Node gzNode;

  // Torque persistence
  std::mutex torqueMutex;
  Eigen::ArrayXf storedTorques{Eigen::ArrayXf::Zero(numJoints)};
  bool torquesActive{false};

  // State snapshot
  std::mutex stateMutex;
  Eigen::Vector4f snapQuat{1.0f, 0.0f, 0.0f, 0.0f};
  Eigen::Vector3f snapGyro{Eigen::Vector3f::Zero()};
  Eigen::ArrayXf  snapJointPos{Eigen::ArrayXf::Zero(numJoints)};
  Eigen::ArrayXf  snapJointVel{Eigen::ArrayXf::Zero(numJoints)};
  // World body GT [px,py,pz, vx,vy,vz] — telemetry/GUI only.
  Eigen::Matrix<float, 6, 1> snapBodyState{Eigen::Matrix<float, 6, 1>::Zero()};

  /// Locate the robot model in the ECM
  bool tryInitFromECM(gz::sim::EntityComponentManager &_ecm);

  bool OnReadStates(gz::msgs::Double_V &_response);
  bool OnSendTorques(const gz::msgs::Double_V &_request, gz::msgs::Boolean &_response);
  bool OnResetJoints(const gz::msgs::Double_V &_request, gz::msgs::Boolean &_response);
};

GazeboPlugin::GazeboPlugin()
  : dataPtr(std::make_unique<Implementation>())
{
}

GazeboPlugin::~GazeboPlugin() = default;

void GazeboPlugin::setRobotSpec(const rbq_gazebo::RobotSpec &spec)
{
  this->dataPtr->spec          = spec;
  this->dataPtr->numJoints     = static_cast<int>(spec.joints.size());
  this->dataPtr->storedTorques = Eigen::ArrayXf::Zero(this->dataPtr->numJoints);
  this->dataPtr->snapJointPos  = Eigen::ArrayXf::Zero(this->dataPtr->numJoints);
  this->dataPtr->snapJointVel  = Eigen::ArrayXf::Zero(this->dataPtr->numJoints);
}

bool GazeboPlugin::Implementation::OnReadStates(gz::msgs::Double_V &_response)
{
  if (!this->initialized) {
    return false;
  }

  // Return the latest snapshot: [quat(4), gyro(3), joint_pos(N), joint_vel(N), body_gt(6)].
  std::lock_guard<std::mutex> lock(this->stateMutex);

  _response.clear_data();

  for (int i = 0; i < 4; ++i) {
    _response.add_data(this->snapQuat[i]);
  }
  for (int i = 0; i < 3; ++i) {
    _response.add_data(this->snapGyro[i]);
  }
  for (int i = 0; i < this->numJoints; ++i) {
    _response.add_data(this->snapJointPos[i]);
  }
  for (int i = 0; i < this->numJoints; ++i) {
    _response.add_data(this->snapJointVel[i]);
  }
  for (int i = 0; i < 6; ++i) {
    _response.add_data(this->snapBodyState[i]);
  }

  return true;
}

bool GazeboPlugin::Implementation::OnSendTorques(
    const gz::msgs::Double_V &_request,
    gz::msgs::Boolean &_response)
{
  if (!this->initialized) {
    _response.set_data(false);
    return false;
  }

  if (_request.data_size() < this->numJoints) {
    std::fprintf(stderr, "[rbq_gazebo][WARN] " "Received %d torques, expected %d\n", _request.data_size(), this->numJoints);
    _response.set_data(false);
    return false;
  }

  // Store torques
  {
    std::lock_guard<std::mutex> lock(this->torqueMutex);
    for (int i = 0; i < this->numJoints; ++i) {
      this->storedTorques[i] = static_cast<float>(_request.data(i));
    }
    this->torquesActive = true;
  }

  _response.set_data(true);
  return true;
}

bool GazeboPlugin::Implementation::OnResetJoints(
    const gz::msgs::Double_V &_request,
    gz::msgs::Boolean &_response)
{
  if (!this->initialized || !this->wrapper) {
    _response.set_data(false);
    return false;
  }

  const int num_joints = this->numJoints;
  if (_request.data_size() != num_joints && _request.data_size() != num_joints * 2) {
    std::fprintf(stderr, "[rbq_gazebo][WARN] " "Received %d values, expected %d or %d\n",
                _request.data_size(), num_joints, num_joints * 2);
    _response.set_data(false);
    return false;
  }

  bool has_velocities = (_request.data_size() == num_joints * 2);

  // Clear stored torques on reset
  {
    std::lock_guard<std::mutex> lock(this->torqueMutex);
    this->storedTorques.setZero();
    this->torquesActive = false;
  }

  bool success = true;
  for (int i = 0; i < num_joints; ++i) {
    double position = _request.data(i);
    double velocity = has_velocities ? _request.data(num_joints + i) : 0.0;

    if (!this->wrapper->resetJointState(i, position, velocity)) {
      success = false;
    }
  }

  _response.set_data(success);
  return success;
}

// Find the robot model in the ECM
bool GazeboPlugin::Implementation::tryInitFromECM(
  gz::sim::EntityComponentManager &_ecm)
{
  auto robotEntity = _ecm.EntityByComponents(
      gz::sim::components::Name(this->spec.name),
      gz::sim::components::Model());

  if (robotEntity == gz::sim::kNullEntity) {
    return false;
  }

  this->modelEntity = robotEntity;
  std::fprintf(stdout, "[rbq_gazebo][INFO] " "Found robot model '%s' (%d joints)\n",
               this->spec.name.c_str(), this->numJoints);

  auto jointEntities = _ecm.ChildrenByComponents(robotEntity, gz::sim::components::Joint());

  auto findJoint = [&](const std::string &targetName) -> gz::sim::Entity {
    for (const auto &jointEntity : jointEntities) {
      const auto *name = _ecm.Component<gz::sim::components::Name>(jointEntity);
      if (name && name->Data() == targetName) return jointEntity;
    }
    return gz::sim::kNullEntity;
  };

  this->jointNames.clear();
  for (const auto &joint : this->spec.joints) {
    if (findJoint(joint.name) != gz::sim::kNullEntity) {
      this->jointNames.push_back(joint.name);
    } else {
      std::fprintf(stderr, "[rbq_gazebo][WARN] " "Joint '%s' not found\n", joint.name.c_str());
    }
  }

  // Abort on a joint-table/SDF mismatch
  if (static_cast<int>(this->jointNames.size()) != this->numJoints) {
    std::fprintf(stderr, "[rbq_gazebo][ERROR] " "Resolved %zu of %d expected joints — aborting init "
                 "(joint table vs SDF mismatch)\n",
                 this->jointNames.size(), this->numJoints);
    return false;
  }

  this->imuEntity = gz::sim::kNullEntity;
  _ecm.Each<gz::sim::components::Imu>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Imu *) -> bool
    {
      this->imuEntity = _entity;
      return false;
    });

  this->wrapper = std::make_unique<GazeboSystemWrapper>();
  if (this->wrapper->init(&_ecm, this->spec.name, this->jointNames, this->imuEntity)) {
    std::fprintf(stdout, "[rbq_gazebo][INFO] " "GazeboSystemWrapper initialized with %zu joints\n",
                this->jointNames.size());
  } else {
    std::fprintf(stderr, "[rbq_gazebo][ERROR] " "Failed to initialize GazeboSystemWrapper\n");
    return false;
  }

  for (size_t i = 0; i < this->jointNames.size(); ++i) {
    this->wrapper->resetJointState(i, this->spec.joints[i].holdPos, 0.0);  // spawn pose
    this->wrapper->resetJointForTorqueControl(i);
  }

  this->initialized = true;
  std::fprintf(stdout, "[rbq_gazebo][INFO] " "Robot initialized — services are ready\n");
  return true;
}

void GazeboPlugin::Configure(
  const gz::sim::Entity &/*_entity*/,
  const std::shared_ptr<const sdf::Element> &/*_sdf*/,
  gz::sim::EntityComponentManager &/*_ecm*/,
  gz::sim::EventManager &/*_eventMgr*/)
{
  if (!this->dataPtr->gzNode.Advertise("/rbq/read_states",
      &GazeboPlugin::Implementation::OnReadStates, this->dataPtr.get())) {
    std::fprintf(stderr, "[rbq_gazebo][ERROR] " "Failed to advertise /rbq/read_states\n");
  }

  if (!this->dataPtr->gzNode.Advertise("/rbq/send_torques",
      &GazeboPlugin::Implementation::OnSendTorques, this->dataPtr.get())) {
    std::fprintf(stderr, "[rbq_gazebo][ERROR] " "Failed to advertise /rbq/send_torques\n");
  }

  if (!this->dataPtr->gzNode.Advertise("/rbq/reset_joints",
      &GazeboPlugin::Implementation::OnResetJoints, this->dataPtr.get())) {
    std::fprintf(stderr, "[rbq_gazebo][ERROR] " "Failed to advertise /rbq/reset_joints\n");
  }

  this->dataPtr->servicesAdvertised = true;
  std::fprintf(stdout, "[rbq_gazebo][INFO] " "Plugin loaded. Waiting for robot model '%s'...\n",
               this->dataPtr->spec.name.c_str());
}

void GazeboPlugin::PreUpdate(
  const gz::sim::UpdateInfo &/*_info*/,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized) {
    this->dataPtr->tryInitFromECM(_ecm);
    return;
  }

  if (!this->dataPtr->wrapper) {
    return;
  }

  // Re-apply stored torques every step
  std::lock_guard<std::mutex> lock(this->dataPtr->torqueMutex);
  if (this->dataPtr->torquesActive) {
    this->dataPtr->wrapper->sendTorqueCommands(this->dataPtr->storedTorques);
  }

}

void GazeboPlugin::PostUpdate(
  const gz::sim::UpdateInfo &/*_info*/,
  const gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (!this->dataPtr->initialized || !this->dataPtr->wrapper) {
    return;
  }

  // Snapshot post-integration state
  this->dataPtr->wrapper->readStates();
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  this->dataPtr->snapQuat     = this->dataPtr->wrapper->quaternion();
  this->dataPtr->snapGyro     = this->dataPtr->wrapper->gyro();
  this->dataPtr->snapJointPos = this->dataPtr->wrapper->jointPos();
  this->dataPtr->snapJointVel = this->dataPtr->wrapper->jointVel();
  this->dataPtr->snapBodyState = this->dataPtr->wrapper->x_W();
}

// Register the RBQ plugin
bool addToServer(gz::sim::Server &server, const rbq_gazebo::RobotSpec &spec)
{
    static std::shared_ptr<GazeboPlugin> system = std::make_shared<GazeboPlugin>();
    system->setRobotSpec(spec);
    const auto added = server.AddSystem(system);
    return added.has_value() && added.value();
}

}  // namespace rbq_gazebo_system

IGNITION_ADD_PLUGIN(
  rbq_gazebo_system::GazeboPlugin,
  gz::sim::System,
  rbq_gazebo_system::GazeboPlugin::ISystemConfigure,
  rbq_gazebo_system::GazeboPlugin::ISystemPreUpdate,
  rbq_gazebo_system::GazeboPlugin::ISystemPostUpdate)
