#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/idl/rbq/ArmJointInfo_.hpp>
#include <rbq_sdk/idl/rbq/TaskInfo_.hpp>

#include "arm_helper.hpp"

// 01 — RBA-1 arm state. Read only: prints the joints (rt/rbq/arm_joint) and the end effector
// (rt/rbq/arm_ee) twice a second until Ctrl-C. Joints 0..5 are J0..J5, 6 is the gripper (GRP).

static constexpr int kArmSlots = 7;
static constexpr float kR2D = 57.295779513f;

static std::atomic<bool> g_running{true};

static std::string Usage()
{
    return std::string(
        "Usage: 01_arm_state [target]\n"
        "\n"
        "Prints the RBA-1 arm state twice a second until Ctrl-C. Read only - it sends nothing to the robot,\n"
        "so keep it open in its own terminal while you run the other examples.\n"
        "\n") + arm_helper::kTargetUsage + "\n"
        "Rows J0..J5 (shoulder yaw, shoulder pitch, elbow pitch, elbow roll, wrist pitch, wrist roll) and GRP:\n"
        "  stat/err   board state (NON CON HOM RUN) / error (OK JAM FLT TMP CUR BIG INP)\n"
        "  owner      which program drives the joint (4 = arm controller, 22 = 07_arm_low_level)\n"
        "  ref_* / *  reference next to measured position [deg], velocity [deg/s], torque [Nm]\n"
        "  kp / kd    gains the joint applies [Nm/rad], [Nm*s/rad] - 0 until the arm is started\n"
        "The EE line is the end-effector position [m] and roll/pitch/yaw [deg] in the body frame.\n";
}

int main(int argc, char *argv[])
{
    if (!arm_helper::TargetGiven(argc, argv, Usage())) return arm_helper::HelpAsked(argc, argv) ? 0 : 1;
    if (argc > 2) return arm_helper::ArgError(argv, std::string("unexpected argument '") + argv[2] + "'", Usage());
    std::signal(SIGINT, [](int) { g_running = false; });
    if (arm_helper::DryRun(argv[1], "State")) return 0;
    if (!arm_helper::InitChannel(argv[1])) return 1;

    std::mutex mx;
    bool gotJoint = false, gotEE = false;
    rbq_msgs::msg::dds_::ArmJointInfo_ joints;
    rbq_msgs::msg::dds_::TaskInfo_ ee;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::ArmJointInfo_> subJoint(
        [&](const rbq_msgs::msg::dds_::ArmJointInfo_ &m) { std::lock_guard<std::mutex> l(mx); joints = m; gotJoint = true; },
        "rt/rbq/arm_joint");
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::TaskInfo_> subEE(
        [&](const rbq_msgs::msg::dds_::TaskInfo_ &m) { std::lock_guard<std::mutex> l(mx); ee = m; gotEE = true; },
        "rt/rbq/arm_ee");

    const auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> lock(mx);
        if (!gotJoint) {
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3))
                std::cout << "[01_arm_state] no data on rt/rbq/arm_joint yet (Network not running, or the wrong target?)\n";
            continue;
        }
        // each reference next to its measured value: ref_pos pos, ref_vel vel, ref_torque torque
        std::printf("\n name stat err  owner %14s %14s %14s %14s %14s %14s %11s %12s %7s  T(b/c)\n",
                    "ref_pos[deg]", "pos[deg]", "ref_vel[deg/s]", "vel[deg/s]", "ref_torque[Nm]", "torque[Nm]",
                    "kp[Nm/rad]", "kd[Nms/rad]", "cur[A]");
        for (int i = 0; i < kArmSlots; i++) {
            const auto &j = joints.joint()[i];
            std::printf(" %-4s %-4s %-4s %5d %14.2f %14.2f %14.2f %14.2f %14.3f %14.3f %11.2f %12.3f %7.2f  %d/%d\n",
                        j.name().c_str(), j.status().c_str(), j.error_msg().c_str(), int(j.owner()),
                        j.ref_position() * kR2D, j.pos() * kR2D, j.ref_vel() * kR2D, j.vel() * kR2D,
                        j.ref_ff_torque(), j.torque(),
                        j.kp(), j.kd(), j.current(), int(j.temperature_board()), int(j.temperature_coil()));
        }
        if (gotEE)
            std::printf(" EE (body frame)  pos [m] %7.3f %7.3f %7.3f   rpy [deg] %7.1f %7.1f %7.1f\n",
                        ee.pos()[0], ee.pos()[1], ee.pos()[2], ee.rpy()[0] * kR2D, ee.rpy()[1] * kR2D, ee.rpy()[2] * kR2D);
    }
    return 0;
}
