#ifndef RBQ_GAZEBO__BRIDGE_HPP_
#define RBQ_GAZEBO__BRIDGE_HPP_

#include <atomic>
#include <string>
#include <vector>

namespace rbq_gazebo
{

// Which MotionRef_/SimInfo_ array a joint's ref and state belong to.
enum class JointGroup { Leg, Wheel };

struct JointSpec
{
    std::string name;
    JointGroup  group;
    int         index;      // index into leg_joint[] or whl_joint[]
    double      holdPos;
    double      holdKp;     // 0 for wheels: damp to a stop instead of holding an angle
    double      torqueLim;
};

// One robot variant: model to spawn, joint layout and spawn pose.
struct RobotSpec
{
    std::string            name;      // gz model name, also the resources/models/<dir>
    std::vector<JointSpec> joints;
    double                 spawnZ;    // drop height in the empty world
};

// 12-DOF quadruped.
inline RobotSpec rbqSpec()
{
    static const char *kName[12] = {
        "joint0_HRR",  "joint1_HRP",  "joint2_HRK",
        "joint3_HLR",  "joint4_HLP",  "joint5_HLK",
        "joint6_FRR",  "joint7_FRP",  "joint8_FRK",
        "joint9_FLR",  "joint10_FLP", "joint11_FLK",
    };
    static const double kSit[12] = {
        -0.59341195, 1.13446401, -2.79252680,
         0.59341195, 1.13446401, -2.79252680,
        -0.59341195, 1.13446401, -2.79252680,
         0.59341195, 1.13446401, -2.79252680,
    };

    RobotSpec spec;
    spec.name   = "rbq10";
    spec.spawnZ = 0.70;
    spec.joints.reserve(12);
    // Torque limit is the real motor rating, tighter than the MJCF's ctrlrange.
    for (int i = 0; i < 12; ++i)
        spec.joints.push_back({kName[i], JointGroup::Leg, i, kSit[i], 200.0, (i % 3 == 2) ? 140.0 : 100.0});
    return spec;
}

// 12-DOF quadruped + 4 driven wheels; the wheel trails its leg, so every 4th entry is a wheel.
inline RobotSpec rbqWheelSpec()
{
    static const char *kName[16] = {
        "joint0_HRR",  "joint1_HRP",  "joint2_HRK",  "joint_HRW",
        "joint3_HLR",  "joint4_HLP",  "joint5_HLK",  "joint_HLW",
        "joint6_FRR",  "joint7_FRP",  "joint8_FRK",  "joint_FRW",
        "joint9_FLR",  "joint10_FLP", "joint11_FLK", "joint_FLW",
    };
    static const double kSit[16] = {
        -0.59341195, 1.13446401, -2.78, 0.0,
         0.59341195, 1.13446401, -2.78, 0.0,
        -0.59341195, 1.13446401, -2.78, 0.0,
         0.59341195, 1.13446401, -2.78, 0.0,
    };

    RobotSpec spec;
    spec.name   = "rbq10_wheel";
    spec.spawnZ = 0.70;
    spec.joints.reserve(16);
    for (int i = 0; i < 16; ++i) {
        const int leg = i / 4;
        if (i % 4 == 3)
            spec.joints.push_back({kName[i], JointGroup::Wheel, leg, kSit[i], 0.0, 30.0});
        else
            spec.joints.push_back({kName[i], JointGroup::Leg, leg * 3 + i % 4, kSit[i],
                                   200.0, (i % 4 == 2) ? 140.0 : 100.0});
    }
    return spec;
}

// Create rbq_sdk DDS endpoints
void startBridgeDdsEndpoints();

// 1000Hz bridge loop
void bridgeLoop(const std::string &worldName, std::atomic<bool> &running,
                const rbq_gazebo::RobotSpec &spec);

}  // namespace rbq_gazebo

#endif  // RBQ_GAZEBO__BRIDGE_HPP_
