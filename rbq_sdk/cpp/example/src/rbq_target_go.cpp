#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/rbq/TargetGo_.hpp>

// gait constants (mirror QuadWalk_TARGET_GO handler)
static constexpr int GAIT_WAVE   = 0; // static wave walk
static constexpr int GAIT_STAIRS = 1; // stair walking
static constexpr int GAIT_RL     = 2; // RL-policy walk

// approach mode constants
// 0: rotate → straight → rotate
// 1: rotate to theta → diagonal walk
// 2: diagonal walk → rotate to theta
// 3: simultaneous rotate + diagonal walk
static constexpr int MODE_DEFAULT = 0;

static void printUsage()
{
    std::cout <<
        "Usage: rbq_target_go <interface> [x] [y] [theta_deg] [gait] [mode]\n"
        "\n"
        "  interface   CycloneDDS interface, e.g. lo (sim) or eth0 (real robot)\n"
        "  x           Target X in metres, relative to robot start  (default 1.0)\n"
        "  y           Target Y in metres, relative to robot start  (default 0.0)\n"
        "  theta       Target heading in radians                    (default 0.0)\n"
        "  gait        0=wave(default) / 1=stairs / 2=rl\n"
        "  mode        0=rot-straight-rot(default) / 1=rot-then-diag /\n"
        "              2=diag-then-rot / 3=rot+diag simultaneous\n"
        "\n"
        "Example (sim, 1 m forward with wave gait):\n"
        "  rbq_target_go lo 1.0 0.0 0.0 0 0\n";
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printUsage();
        return 1;
    }

    const std::string interface = argv[1];
    const float x         = (argc > 2) ? std::stof(argv[2]) : 1.0f;
    const float y         = (argc > 3) ? std::stof(argv[3]) : 0.0f;
    const float theta     = (argc > 4) ? std::stof(argv[4]) : 0.0f;
    const int   gait      = (argc > 5) ? std::stoi(argv[5]) : GAIT_WAVE;
    const int   mode      = (argc > 6) ? std::stoi(argv[6]) : MODE_DEFAULT;

    rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, interface);

    rbq_sdk::Publisher<rbq_msgs::msg::dds_::TargetGo_> pub("rt/rbq/cmd/target_go");

    // wait for DDS discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rbq_msgs::msg::dds_::TargetGo_ msg;
    msg.gait(gait);
    msg.mode(mode);
    msg.x(x);
    msg.y(y);
    msg.theta(theta);
    msg.offset_x(0.0f);
    msg.offset_y(0.0f);
    msg.slow(false);
    msg.wide(false);
    msg.vision(false);

    pub.write(msg);

    std::cout << "[rbq_target_go] published:"
              << " gait=" << gait
              << " mode=" << mode
              << " x=" << x
              << " y=" << y
              << " theta=" << theta
              << std::endl;

    return 0;
}
