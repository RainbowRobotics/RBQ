#include "gazebo_system/GazeboSystemWrapper.h"

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>

#include <cmath>
#include <chrono>

namespace rbq_gazebo_system
{

GazeboSystemWrapper::GazeboSystemWrapper()
  : ecm_(nullptr)
  , imu_entity_(sim::kNullEntity)
  , model_entity_(sim::kNullEntity)
  , body_link_(sim::kNullEntity)
  , quat_(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.0f))
  , gyro_(Eigen::Vector3f::Zero())
  , x_W_(Eigen::Matrix<float, 6, 1>::Zero())
{
}

GazeboSystemWrapper::~GazeboSystemWrapper() = default;

bool GazeboSystemWrapper::init(
  sim::EntityComponentManager *ecm,
  const std::vector<std::string> &joint_names,
  sim::Entity imu_entity)
{
  if (!ecm) {
    std::fprintf(stderr, "[rbq_gazebo][ERROR] " "ECM is null\n");
    return false;
  }

  ecm_ = ecm;
  joint_names_ = joint_names;
  imu_entity_ = imu_entity;

  joint_entities_.clear();
  for (const auto &joint_name : joint_names_) {
    sim::Entity joint_entity = findJointEntity(joint_name);
    if (joint_entity == sim::kNullEntity) {
      std::fprintf(stderr, "[rbq_gazebo][WARN] " "Joint '%s' not found\n", joint_name.c_str());
      continue;
    }
    joint_entities_[joint_name] = joint_entity;

    if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointPosition().TypeId())) {
      ecm_->CreateComponent(joint_entity, sim::components::JointPosition());
    }
    if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointVelocity().TypeId())) {
      ecm_->CreateComponent(joint_entity, sim::components::JointVelocity());
    }
    if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointTransmittedWrench().TypeId())) {
      ecm_->CreateComponent(joint_entity, sim::components::JointTransmittedWrench());
    }
    if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointForceCmd().TypeId())) {
      ecm_->CreateComponent(joint_entity, sim::components::JointForceCmd({0.0}));
    }
  }

  joint_pos_.resize(joint_names_.size());
  joint_vel_.resize(joint_names_.size());
  joint_torques_.resize(joint_names_.size());
  joint_pos_.setZero();
  joint_vel_.setZero();
  joint_torques_.setZero();

  // Locate the rbq10 model entity.
  model_entity_ = ecm_->EntityByComponents(
      sim::components::Name("rbq10"),
      sim::components::Model());

  if (model_entity_ == sim::kNullEntity) {
    std::fprintf(stderr, "[rbq_gazebo][WARN] " "Model 'rbq10' not found, using first available model\n");
    ecm_->Each<sim::components::Model>(
      [this](const sim::Entity &entity, const sim::components::Model *) -> bool
      {
        if (model_entity_ == sim::kNullEntity) {
          model_entity_ = entity;
        }
        return false;
      });
  }

  // Locate the trunk link
  body_link_ = sim::kNullEntity;
  ecm_->Each<sim::components::Link, sim::components::Name, sim::components::ParentEntity>(
    [this](const sim::Entity &entity,
           const sim::components::Link *,
           const sim::components::Name *name,
           const sim::components::ParentEntity *parent) -> bool
    {
      if (parent->Data() == model_entity_ && name->Data() == "trunk") {
        body_link_ = entity;
        return false;
      }
      return true;
    });

  if (body_link_ != sim::kNullEntity) {
    if (!ecm_->EntityHasComponentType(body_link_, sim::components::WorldPose().TypeId())) {
      ecm_->CreateComponent(body_link_, sim::components::WorldPose());
    }
    if (!ecm_->EntityHasComponentType(body_link_, sim::components::WorldLinearVelocity().TypeId())) {
      ecm_->CreateComponent(body_link_, sim::components::WorldLinearVelocity());
    }
    if (!ecm_->EntityHasComponentType(body_link_, sim::components::WorldAngularVelocity().TypeId())) {
      ecm_->CreateComponent(body_link_, sim::components::WorldAngularVelocity());
    }
  } else {
    std::fprintf(stderr, "[rbq_gazebo][WARN] " "Trunk link not found - state reading may be incorrect\n");
  }

  std::fprintf(stdout, "[rbq_gazebo][INFO] " "Initialized with %zu joints\n", joint_names_.size());
  return true;
}

bool GazeboSystemWrapper::readStates()
{
  if (!ecm_) {
    return false;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string &joint_name = joint_names_[i];
    auto it = joint_entities_.find(joint_name);
    if (it == joint_entities_.end()) {
      continue;
    }

    sim::Entity joint_entity = it->second;

    const auto *joint_pos = ecm_->Component<sim::components::JointPosition>(joint_entity);
    if (joint_pos && !joint_pos->Data().empty()) {
      joint_pos_[i] = joint_pos->Data()[0];
    }

    const auto *joint_vel = ecm_->Component<sim::components::JointVelocity>(joint_entity);
    if (joint_vel && !joint_vel->Data().empty()) {
      joint_vel_[i] = joint_vel->Data()[0];
    }
  }

  readImuData();
  readModelPose();

  return true;
}

void GazeboSystemWrapper::sendTorqueCommands(const Eigen::ArrayXf &torques)
{
  if (!ecm_) {
    return;
  }

  joint_torques_ = torques.cast<double>();

  for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(joint_torques_.size()); ++i) {
    const std::string &joint_name = joint_names_[i];
    auto it = joint_entities_.find(joint_name);
    if (it == joint_entities_.end()) {
      continue;
    }

    sim::Entity joint_entity = it->second;
    double torque_value = joint_torques_[i];

    auto *joint_force_cmd = ecm_->Component<sim::components::JointForceCmd>(joint_entity);
    if (joint_force_cmd) {
      *joint_force_cmd = sim::components::JointForceCmd({torque_value});
    } else {
      ecm_->CreateComponent(joint_entity, sim::components::JointForceCmd({torque_value}));
    }
  }
}

bool GazeboSystemWrapper::resetJointState(int joint_index, double position, double velocity)
{
  if (!ecm_ || joint_index < 0 || static_cast<size_t>(joint_index) >= joint_names_.size()) {
    return false;
  }

  const std::string &joint_name = joint_names_[joint_index];
  auto it = joint_entities_.find(joint_name);
  if (it == joint_entities_.end()) {
    return false;
  }

  sim::Entity joint_entity = it->second;

  auto *joint_pos = ecm_->Component<sim::components::JointPosition>(joint_entity);
  if (joint_pos && !joint_pos->Data().empty()) {
    joint_pos->Data()[0] = position;
  }

  auto *joint_vel = ecm_->Component<sim::components::JointVelocity>(joint_entity);
  if (joint_vel && !joint_vel->Data().empty()) {
    joint_vel->Data()[0] = velocity;
  }

  if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointPositionReset().TypeId())) {
    ecm_->CreateComponent(joint_entity, sim::components::JointPositionReset({position}));
  } else {
    auto *reset = ecm_->Component<sim::components::JointPositionReset>(joint_entity);
    if (reset) {
      *reset = sim::components::JointPositionReset({position});
    }
  }

  if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointVelocityReset().TypeId())) {
    ecm_->CreateComponent(joint_entity, sim::components::JointVelocityReset({velocity}));
  } else {
    auto *reset = ecm_->Component<sim::components::JointVelocityReset>(joint_entity);
    if (reset) {
      *reset = sim::components::JointVelocityReset({velocity});
    }
  }

  return true;
}

bool GazeboSystemWrapper::resetJointForTorqueControl(int joint_index)
{
  if (!ecm_ || joint_index < 0 || static_cast<size_t>(joint_index) >= joint_names_.size()) {
    return false;
  }

  const std::string &joint_name = joint_names_[joint_index];
  auto it = joint_entities_.find(joint_name);
  if (it == joint_entities_.end()) {
    return false;
  }

  sim::Entity joint_entity = it->second;

  if (!ecm_->EntityHasComponentType(joint_entity, sim::components::JointForceCmd().TypeId())) {
    ecm_->CreateComponent(joint_entity, sim::components::JointForceCmd({0.0}));
  } else {
    auto *joint_force = ecm_->Component<sim::components::JointForceCmd>(joint_entity);
    if (joint_force) {
      *joint_force = sim::components::JointForceCmd({0.0});
    }
  }

  if (ecm_->EntityHasComponentType(joint_entity, sim::components::JointVelocityCmd().TypeId())) {
    auto *vel_cmd = ecm_->Component<sim::components::JointVelocityCmd>(joint_entity);
    if (vel_cmd && !vel_cmd->Data().empty()) {
      std::fill(vel_cmd->Data().begin(), vel_cmd->Data().end(), 0.0);
    }
  }

  return true;
}

bool GazeboSystemWrapper::stepSimulation(int steps)
{
  if (!ecm_) {
    return false;
  }

  if (world_name_.empty()) {
    ecm_->Each<sim::components::World, sim::components::Name>(
      [this](const sim::Entity &,
             const sim::components::World *,
             const sim::components::Name *name) -> bool
      {
        world_name_ = name->Data();
        return false;
      });

    if (world_name_.empty()) {
      std::fprintf(stderr, "[rbq_gazebo][ERROR] " "World name not found\n");
      return false;
    }
  }

  static gz::transport::Node gz_node;

  gz::msgs::WorldControl req;
  req.set_multi_step(steps);

  const std::string service = "/world/" + world_name_ + "/control";
  gz::msgs::Boolean rep;
  bool result = false;

  bool ok = gz_node.Request(service, req, 500, rep, result);
  return ok && result && rep.data();
}

sim::Entity GazeboSystemWrapper::findJointEntity(const std::string &joint_name)
{
  if (!ecm_) {
    return sim::kNullEntity;
  }

  // Match on the Joint marker component.
  return ecm_->EntityByComponents(
      sim::components::Name(joint_name),
      sim::components::Joint());
}

void GazeboSystemWrapper::readImuData()
{
  if (body_link_ == sim::kNullEntity || !ecm_) {
    return;
  }

  const auto *world_pose_comp = ecm_->Component<sim::components::WorldPose>(body_link_);
  if (world_pose_comp) {
    const auto &pose = world_pose_comp->Data();
    const auto &q = pose.Rot();

    quat_[0] = static_cast<float>(q.W());
    quat_[1] = static_cast<float>(q.X());
    quat_[2] = static_cast<float>(q.Y());
    quat_[3] = static_cast<float>(q.Z());
  }

  // Rotate world angular velocity into the body frame
  const auto *ang_vel_comp = ecm_->Component<sim::components::WorldAngularVelocity>(body_link_);
  if (ang_vel_comp && world_pose_comp) {
    const auto &ang_vel_world = ang_vel_comp->Data();
    const auto &rot = world_pose_comp->Data().Rot();

    auto rot_inv = rot.Inverse();
    auto ang_vel_body = rot_inv.RotateVector(ang_vel_world);

    gyro_[0] = static_cast<float>(ang_vel_body.X());
    gyro_[1] = static_cast<float>(ang_vel_body.Y());
    gyro_[2] = static_cast<float>(ang_vel_body.Z());
  }
}

void GazeboSystemWrapper::readModelPose()
{
  if (body_link_ == sim::kNullEntity || !ecm_) {
    return;
  }

  const auto *world_pose_comp = ecm_->Component<sim::components::WorldPose>(body_link_);
  if (world_pose_comp) {
    const auto &pos = world_pose_comp->Data().Pos();
    x_W_[0] = static_cast<float>(pos.X());
    x_W_[1] = static_cast<float>(pos.Y());
    x_W_[2] = static_cast<float>(pos.Z());
  }

  const auto *lin_vel_comp = ecm_->Component<sim::components::WorldLinearVelocity>(body_link_);
  if (lin_vel_comp) {
    const auto &vel = lin_vel_comp->Data();
    x_W_[3] = static_cast<float>(vel.X());
    x_W_[4] = static_cast<float>(vel.Y());
    x_W_[5] = static_cast<float>(vel.Z());
  }
}

}  // namespace rbq_gazebo_system
