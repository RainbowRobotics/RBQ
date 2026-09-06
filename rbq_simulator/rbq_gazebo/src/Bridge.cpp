#include "Bridge.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/model.pb.h>

#include <rbq_sdk/rbq_sdk.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/rbq/MotionRef_.hpp>
#include <rbq_sdk/idl/rbq/SimInfo_.hpp>

namespace rbq_gazebo
{
namespace
{

constexpr long   kControlPeriodUs = 1000;  // 1000Hz control tick

constexpr char kRefTopic[]  = "rt/rbq/ref/motion/_0";
constexpr char kSimTopic[]  = "rt/rbq/_sim";

// DDS endpoints
rbq_msgs::msg::dds_::MotionRef_ g_ref;
std::unique_ptr<rbq_sdk::Subscriber<rbq_msgs::msg::dds_::MotionRef_>> g_refSub;
std::unique_ptr<rbq_sdk::Publisher<rbq_msgs::msg::dds_::SimInfo_>>    g_infoPub;

// Joint hold damping; the hold Kp is per joint (JointSpec::holdKp).
constexpr double kHoldKd = 5.0;

// Motion handover detection threshold (Nm).
constexpr double kFFtEps = 0.5;

struct RobotState
{
    Eigen::Quaternionf quat{1.0f, 0.0f, 0.0f, 0.0f};
    Eigen::Vector3f    gyro{Eigen::Vector3f::Zero()};
    Eigen::VectorXd    joint_pos;
    Eigen::VectorXd    joint_vel;
    Eigen::Vector3f    body_pos{Eigen::Vector3f::Zero()};
    Eigen::Vector3f    body_vel{Eigen::Vector3f::Zero()};
};

// gz-transport Node
inline gz::transport::Node &gzNode()
{
    static gz::transport::Node node;
    return node;
}

// gz IMU accelerometer
std::mutex g_imuMutex;
float g_imuAccel[3] = {0.0f, 0.0f, 9.81f};
int64_t g_simSec  = 0;
int32_t g_simNsec = 0;

void onImuMsg(const gz::msgs::IMU &msg)
{
    std::lock_guard<std::mutex> lk(g_imuMutex);
    g_imuAccel[0] = static_cast<float>(msg.linear_acceleration().x());
    g_imuAccel[1] = static_cast<float>(msg.linear_acceleration().y());
    g_imuAccel[2] = static_cast<float>(msg.linear_acceleration().z());
    g_simSec  = msg.header().stamp().sec();
    g_simNsec = msg.header().stamp().nsec();
}

// gz frame
bool readStates(RobotState &state, int jointCount)
{
    const int total = jointCount;
    const int minSize = 7 + 2 * total;
    gz::msgs::Double_V response;
    bool result = false;
    const bool ok = gzNode().Request("/rbq/read_states", 500, response, result);
    if (!ok || !result || response.data_size() < minSize) {
        return false;
    }
    state.joint_pos.resize(total);
    state.joint_vel.resize(total);
    state.quat = Eigen::Quaternionf(
        response.data(0), response.data(1), response.data(2), response.data(3));
    state.gyro = Eigen::Vector3f(response.data(4), response.data(5), response.data(6));
    for (int i = 0; i < total; ++i) state.joint_pos[i] = response.data(7 + i);
    for (int i = 0; i < total; ++i) state.joint_vel[i] = response.data(7 + total + i);
    if (response.data_size() >= minSize + 6) {
        const int b = 7 + 2 * total;
        state.body_pos = Eigen::Vector3f(response.data(b),     response.data(b + 1), response.data(b + 2));
        state.body_vel = Eigen::Vector3f(response.data(b + 3), response.data(b + 4), response.data(b + 5));
    }
    return true;
}

bool sendTorques(const Eigen::VectorXd &torques)
{
    gz::msgs::Double_V request;
    for (int i = 0; i < torques.size(); ++i) request.add_data(torques[i]);
    gz::msgs::Boolean response;
    bool result = false;
    return gzNode().Request("/rbq/send_torques", request, 500, response, result)
           && result && response.data();
}

// Lockstep step: advance N steps with server paused
bool stepPhysics(const std::string &worldControlSrv, int steps = 1)
{
    gz::msgs::WorldControl req;
    req.set_multi_step(steps);
    req.set_pause(true);
    gz::msgs::Boolean rep;
    bool result = false;
    return gzNode().Request(worldControlSrv, req, 500, rep, result) && result && rep.data();
}

bool pausePhysics(const std::string &worldControlSrv)
{
    gz::msgs::WorldControl req;
    req.set_pause(true);
    gz::msgs::Boolean rep;
    bool result = false;
    return gzNode().Request(worldControlSrv, req, 500, rep, result);
}

// Pace the loop to 1000Hz
void paceTo(std::chrono::high_resolution_clock::time_point &nextTime)
{
    nextTime += std::chrono::microseconds(kControlPeriodUs);
    const auto now = std::chrono::high_resolution_clock::now();
    if (now > nextTime + std::chrono::microseconds(kControlPeriodUs)) {
        nextTime = now;
        return;
    }
    while (std::chrono::high_resolution_clock::now() < nextTime) {}
}

}  // namespace

// Create rbq_sdk DDS endpoints
void startBridgeDdsEndpoints()
{
    if (!g_refSub)
        g_refSub = std::make_unique<rbq_sdk::Subscriber<rbq_msgs::msg::dds_::MotionRef_>>(&g_ref, kRefTopic);
    if (!g_infoPub)
        g_infoPub = std::make_unique<rbq_sdk::Publisher<rbq_msgs::msg::dds_::SimInfo_>>(kSimTopic);
}

void bridgeLoop(const std::string &worldName, std::atomic<bool> &running,
                const rbq_gazebo::RobotSpec &spec)
{
    const std::string worldControlSrv = "/world/" + worldName + "/control";

    const std::vector<JointSpec> &joints = spec.joints;
    const int total = static_cast<int>(joints.size());

    std::cout << "[bridge] loop started (world=" << worldName << ", robot=" << spec.name
              << ", joints=" << total << ").\n";

    startBridgeDdsEndpoints();
    rbq_msgs::msg::dds_::MotionRef_ &ref = g_ref;
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::SimInfo_> &info_pub = *g_infoPub;

    // world -> robot TF (Gazebo ground truth)
    auto tfPub = gzNode().Advertise<gz::msgs::Pose_V>("/rbq/tf");
    // Real joint angles
    auto jsPub = gzNode().Advertise<gz::msgs::Model>("/rbq/joint_state");

    gzNode().Subscribe("/rbq/imu", onImuMsg);

    pausePhysics(worldControlSrv);

    bool motionActive = false;
    bool warnedNoRef = false;
    long iter = 0;
    auto nextTime = std::chrono::high_resolution_clock::now();
    while (running) {
        ++iter;
        // 1. Advance one physics step per control tick.
        stepPhysics(worldControlSrv, 1);
        RobotState state;
        if (!readStates(state, total)) {
            paceTo(nextTime);
            continue;
        }

        // 2. Build SimInfo_.
        rbq_msgs::msg::dds_::SimInfo_ info;
        // gz's sim clock, not wall-clock: subscribers mix this with /clock-driven TFs.
        {   // written by the gz IMU callback thread; same lock as every other read.
            std::lock_guard<std::mutex> lk(g_imuMutex);
            info.imu().header().stamp().sec() = g_simSec;
            info.imu().header().stamp().nanosec() = g_simNsec;
        }
        info.imu().orientation().w() = state.quat.w();
        info.imu().orientation().x() = state.quat.x();
        info.imu().orientation().y() = state.quat.y();
        info.imu().orientation().z() = state.quat.z();
        info.imu().angular_velocity().x() = state.gyro.x();
        info.imu().angular_velocity().y() = state.gyro.y();
        info.imu().angular_velocity().z() = state.gyro.z();
        // Real accelerometer from the gz IMU.
        {
            std::lock_guard<std::mutex> lk(g_imuMutex);
            info.imu().linear_acceleration().x() = g_imuAccel[0];
            info.imu().linear_acceleration().y() = g_imuAccel[1];
            info.imu().linear_acceleration().z() = g_imuAccel[2];
        }

        // 3. Detect Motion handover
        if (!motionActive) {
            for (int i = 0; i < total; ++i) {
                const auto &r = joints[i].group == JointGroup::Wheel
                                    ? ref.whl_joint().at(joints[i].index)
                                    : ref.leg_joint().at(joints[i].index);
                if (r.kp() > 1.0f || std::fabs(r.torque()) > kFFtEps) { motionActive = true; break; }
            }
            if (motionActive) {
                std::cout << "[bridge] Motion ACTIVE: receiving MotionRef (kp[1]="
                          << ref.leg_joint().at(1).kp() << ", qref[1]="
                          << ref.leg_joint().at(1).pos() << ", q[1]=" << state.joint_pos[1]
                          << ", gap=" << (ref.leg_joint().at(1).pos() - state.joint_pos[1])
                          << ") -> handing over.\n";
            } else if (!warnedNoRef && iter > 1500) {  // ~3s
                std::cout << "[bridge] WARNING: no MotionRef on " << kRefTopic
                          << " after 3s. DDS not connected to Motion (check -i interface / domain)."
                          << " Holding spawn pose.\n";
                warnedNoRef = true;
            }
        }

        // 4. Compute leg torques
        Eigen::VectorXd torques = Eigen::VectorXd::Zero(total);
        for (int gi = 0; gi < total; ++gi) {
            const double q  = state.joint_pos[gi];
            const double dq = state.joint_vel[gi];

            const bool isWheel = (joints[gi].group == JointGroup::Wheel);
            const int  idx     = joints[gi].index;
            const auto &r = isWheel ? ref.whl_joint().at(idx) : ref.leg_joint().at(idx);
            double qref = r.pos(), Kp = r.kp(), Kd = r.kd(), FFt = r.torque();
            if (!motionActive) { qref = joints[gi].holdPos; Kp = joints[gi].holdKp; Kd = kHoldKd; FFt = 0.0; }

            double torque = (qref - q) * Kp + (-dq) * Kd + FFt;
            const double lim = joints[gi].torqueLim;
            if (torque >  lim) torque =  lim;
            if (torque < -lim) torque = -lim;
            torques[gi] = torque;

            auto &js = isWheel ? info.whl_joint()[idx] : info.leg_joint()[idx];
            js.pos()    = q;
            js.vel()    = dq;
            js.torque() = torque;
        }

        // Body GT
        info.body_task().pos()[0] = state.body_pos.x();
        info.body_task().pos()[1] = state.body_pos.y();
        info.body_task().pos()[2] = state.body_pos.z();
        info.body_task().quat()[0] = state.quat.w();
        info.body_task().quat()[1] = state.quat.x();
        info.body_task().quat()[2] = state.quat.y();
        info.body_task().quat()[3] = state.quat.z();
        info.body_task().vel()[0] = state.body_vel.x();
        info.body_task().vel()[1] = state.body_vel.y();
        info.body_task().vel()[2] = state.body_vel.z();
        info.body_task().omega()[0] = state.gyro.x();
        info.body_task().omega()[1] = state.gyro.y();
        info.body_task().omega()[2] = state.gyro.z();

        // world -> robot TF: ground truth, off base_link so it never races local_world -> base_link.
        if (iter % 20 == 0) {
            gz::msgs::Pose_V tf;
            auto *p = tf.add_pose();
            p->set_name("robot");
            auto *hdr = p->mutable_header();
            {
                std::lock_guard<std::mutex> lk(g_imuMutex);
                hdr->mutable_stamp()->set_sec(g_simSec);
                hdr->mutable_stamp()->set_nsec(g_simNsec);
            }
            auto *fid = hdr->add_data();
            fid->set_key("frame_id");
            fid->add_value("world");
            p->mutable_position()->set_x(state.body_pos.x());
            p->mutable_position()->set_y(state.body_pos.y());
            p->mutable_position()->set_z(state.body_pos.z());
            p->mutable_orientation()->set_w(state.quat.w());
            p->mutable_orientation()->set_x(state.quat.x());
            p->mutable_orientation()->set_y(state.quat.y());
            p->mutable_orientation()->set_z(state.quat.z());
            tfPub.Publish(tf);

            // Real joint angles -> /joint_states
            gz::msgs::Model js;
            {
                std::lock_guard<std::mutex> lk(g_imuMutex);
                js.mutable_header()->mutable_stamp()->set_sec(g_simSec);
                js.mutable_header()->mutable_stamp()->set_nsec(g_simNsec);
            }
            for (int gi = 0; gi < total; ++gi) {
                auto *j = js.add_joint();
                j->set_name(joints[gi].name);
                j->mutable_axis1()->set_position(state.joint_pos[gi]);
                j->mutable_axis1()->set_velocity(state.joint_vel[gi]);
            }
            jsPub.Publish(js);
        }

        // 5. Send torques + publish state.
        sendTorques(torques);
        info_pub.write(info);

        // 6. Pace to the next control tick.
        paceTo(nextTime);
    }

    std::cout << "[bridge] loop stopped.\n";
}

}  // namespace rbq_gazebo
