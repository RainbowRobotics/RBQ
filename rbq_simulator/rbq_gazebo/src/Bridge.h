#ifndef RBQ_GAZEBO__BRIDGE_HPP_
#define RBQ_GAZEBO__BRIDGE_HPP_

#include <atomic>
#include <string>
#include <vector>

namespace rbq_gazebo
{

struct JointSpec
{
    std::string name;
    double      holdPos;
    double      torqueLim;
};

// 12 leg joints sitting/spawn pose.
inline std::vector<JointSpec> legJoints()
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
    std::vector<JointSpec> v;
    v.reserve(12);
    for (int i = 0; i < 12; ++i)
        v.push_back({kName[i], kSit[i], (i % 3 == 2) ? 150.0 : 100.0});
    return v;
}

// Create rbq_sdk DDS endpoints
void startBridgeDdsEndpoints();

// 1000Hz bridge loop
void bridgeLoop(const std::string &worldName, std::atomic<bool> &running,
                const std::vector<rbq_gazebo::JointSpec> &joints);

}  // namespace rbq_gazebo

#endif  // RBQ_GAZEBO__BRIDGE_HPP_
