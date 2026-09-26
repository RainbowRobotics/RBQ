#ifndef RBQ_GAZEBO_SYSTEM__GAZEBO_SYSTEM_WRAPPER_HPP_
#define RBQ_GAZEBO_SYSTEM__GAZEBO_SYSTEM_WRAPPER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components.hh>
namespace sim = gz::sim;

namespace rbq_gazebo_system
{

class GazeboSystemWrapper
{
public:
  static constexpr int kNumJoints = 12;

  GazeboSystemWrapper();
  ~GazeboSystemWrapper();

  bool init(
    sim::EntityComponentManager *ecm,
    const std::string &model_name,
    const std::vector<std::string> &joint_names,
    sim::Entity imu_entity = sim::kNullEntity);

  bool readStates();

  void sendTorqueCommands(const Eigen::ArrayXf &torques);

  Eigen::ArrayXf jointPos() const { return joint_pos_.cast<float>(); }
  Eigen::ArrayXf jointVel() const { return joint_vel_.cast<float>(); }
  Eigen::Vector4f quaternion() const { return quat_; }
  Eigen::Vector3f gyro() const { return gyro_; }
  Eigen::Matrix<float, 6, 1> x_W() const { return x_W_; }

  bool resetJointState(int joint_index, double position, double velocity = 0.0);
  bool resetJointForTorqueControl(int joint_index);
  bool serverAccessible() const { return ecm_ != nullptr; }
  bool stepSimulation(int steps = 1);

private:
  sim::EntityComponentManager *ecm_;
  std::map<std::string, sim::Entity> joint_entities_;
  std::vector<std::string> joint_names_;
  std::string world_name_;

  Eigen::ArrayXd joint_pos_;
  Eigen::ArrayXd joint_vel_;
  Eigen::ArrayXd joint_torques_;

  sim::Entity imu_entity_;
  sim::Entity model_entity_;
  sim::Entity body_link_;

  Eigen::Vector4f quat_;
  Eigen::Vector3f gyro_;
  Eigen::Matrix<float, 6, 1> x_W_;

  sim::Entity findJointEntity(const std::string &joint_name);
  void readImuData();
  void readModelPose();
};

}  // namespace rbq_gazebo_system

#endif  // RBQ_GAZEBO_SYSTEM__GAZEBO_SYSTEM_WRAPPER_HPP_
