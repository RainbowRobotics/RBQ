#ifndef RBQ_MUJOCO__ROBOT_H_
#define RBQ_MUJOCO__ROBOT_H_

#include <array>
#include <string>
#include <vector>

namespace rbq_mujoco
{

// Which MotionRef_/SimInfo_ array a joint's ref and state belong to.
enum class JointGroup { Leg, Wheel };

struct JointSpec
{
    std::string name;
    JointGroup  group;
    int         index;      // index into leg_joint[] or whl_joint[]
    double      torqueLim;  // Nm; real motor rating, tighter than the MJCF's ctrlrange
};

// One robot variant: model file, joint layout and spawn state.
struct RobotSpec
{
    std::string                name;
    std::string                modelFile;      // resources/model/<file>, used when -p is absent
    std::vector<JointSpec>     joints;         // MuJoCo order: qpos[7 + i], qvel[6 + i], ctrl[i]
    std::vector<double>        initialQpos;    // 7 free-joint values + one per joint
    std::array<std::string, 4> contactBodies;  // RR, RL, FR, FL bodies carrying ground contact
};

// 12-DOF quadruped.
inline RobotSpec rbqSpec()
{
    static const char *kJoint[12] = {
        "joint0_HRR", "joint1_HRP",  "joint2_HRK",
        "joint3_HLR", "joint4_HLP",  "joint5_HLK",
        "joint6_FRR", "joint7_FRP",  "joint8_FRK",
        "joint9_FLR", "joint10_FLP", "joint11_FLK",
    };

    RobotSpec spec;
    spec.name      = "rbq";
    spec.modelFile = "default.xml";
    for (int i = 0; i < 12; ++i)
        spec.joints.push_back({kJoint[i], JointGroup::Leg, i, (i % 3 == 2) ? 140.0 : 100.0});
    spec.initialQpos = {
        0.0, 0.0, 0.8,  1.0, 0.0, 0.0, 0.0,
        -0.59341195, 1.13446401, -2.79252680,
         0.59341195, 1.13446401, -2.79252680,
        -0.59341195, 1.13446401, -2.79252680,
         0.59341195, 1.13446401, -2.79252680,
    };
    spec.contactBodies = {"RR_foot", "RL_foot", "FR_foot", "FL_foot"};
    return spec;
}

// 12-DOF quadruped + 4 driven wheels; the wheel trails its leg, so every 4th joint is a wheel.
inline RobotSpec rbqWheelSpec()
{
    static const char *kJoint[16] = {
        "joint_HRR", "joint_HRP", "joint_HRK", "joint_HRW",
        "joint_HLR", "joint_HLP", "joint_HLK", "joint_HLW",
        "joint_FRR", "joint_FRP", "joint_FRK", "joint_FRW",
        "joint_FLR", "joint_FLP", "joint_FLK", "joint_FLW",
    };

    RobotSpec spec;
    spec.name      = "rbq_wheel";
    spec.modelFile = "wheel.xml";
    for (int i = 0; i < 16; ++i) {
        const int leg = i / 4;
        if (i % 4 == 3) spec.joints.push_back({kJoint[i], JointGroup::Wheel, leg, 30.0});
        else            spec.joints.push_back({kJoint[i], JointGroup::Leg, leg * 3 + i % 4,
                                               (i % 4 == 2) ? 140.0 : 100.0});
    }
    // Folded sit; knee stops at -2.78, inside this model's -2.79 limit.
    spec.initialQpos = {
        0.0, 0.0, 0.8,  1.0, 0.0, 0.0, 0.0,
        -0.59341195, 1.13446401, -2.78, 0.0,
         0.59341195, 1.13446401, -2.78, 0.0,
        -0.59341195, 1.13446401, -2.78, 0.0,
         0.59341195, 1.13446401, -2.78, 0.0,
    };
    spec.contactBodies = {"RR_wheel", "RL_wheel", "FR_wheel", "FL_wheel"};
    return spec;
}

}  // namespace rbq_mujoco

#endif  // RBQ_MUJOCO__ROBOT_H_
