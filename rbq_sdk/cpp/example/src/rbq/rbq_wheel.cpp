#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/idl/rbq/JointOwnershipCmd_.hpp>
#include <rbq_sdk/idl/rbq/MotionRef_.hpp>

// Wheel order in whl_joint[] (= RBQ_API joint 12..15)
static const char *const kWheelNames[4] = {"HRW", "HLW", "FRW", "FLW"};

static constexpr float kTorqueLimit = 10.0f; // [Nm] this example's cap (robot limit is 30 Nm)

static std::atomic<bool> g_running{true};

static void printUsage()
{
    std::cout <<
        "Usage: rbq_wheel <interface> [torque] [kd] [seconds]\n"
        "\n"
        "Spins the four wheels of an RBQ wheel robot (RBQ-W) from an external app.\n"
        "Takes ownership of the wheel joints only - the legs stay with the robot.\n"
        "\n"
        "  interface  CycloneDDS interface, e.g. lo (on the robot) or eth0 (another host)\n"
        "  torque     feed-forward wheel torque [Nm], |torque| <= 10        (default 1.0)\n"
        "  kd         wheel damping [Nm*s/rad], 0 < kd <= 6                  (default 0.5)\n"
        "  seconds    how long to drive, then release the wheels           (default 3)\n"
        "\n"
        "With kp = 0 the board applies torque - kd * vel, so a free wheel settles at\n"
        "vel = torque / kd [rad/s] (default 2 rad/s). On the ground the robot rolls - give it room.\n"
        "The same command works on CAN FD and classic CAN robots: send physical gains,\n"
        "the robot converts them for its board.\n"
        "\n"
        "Example (2 rad/s for 3 s):\n"
        "  rbq_wheel eth0 1.0 0.5 3\n";
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printUsage();
        return 1;
    }
    const std::string interface = argv[1];
    const float torque  = (argc > 2) ? std::stof(argv[2]) : 1.0f;
    const float kd      = (argc > 3) ? std::stof(argv[3]) : 0.5f;
    const float seconds = (argc > 4) ? std::stof(argv[4]) : 3.0f;
    if (std::abs(torque) > kTorqueLimit || kd <= 0.0f || kd > 6.0f || seconds <= 0.0f) {
        printUsage();
        return 1;
    }

    std::signal(SIGINT, [](int) { g_running = false; });

    rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, interface);

    // app slot 21 (rbq_low_level uses 20, so legs and wheels can be driven by two apps at once)
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::MotionRef_>         pubRef("rt/rbq/ref/motion/_21");
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::JointOwnershipCmd_> pubOwner("rt/rbq/cmd/motion/owner/_21");

    // the reference the robot actually applies (all apps merged), at 500 Hz
    std::mutex mx;
    bool got = false;
    rbq_msgs::msg::dds_::MotionRef_ applied;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::MotionRef_> subApplied(
        [&](const rbq_msgs::msg::dds_::MotionRef_ &m) {
            std::lock_guard<std::mutex> lock(mx);
            applied = m;
            got = true;
        }, "rt/rbq/ref/motion/_0");

    // wait for DDS discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto wheelRef = [](float tq, float d) {
        rbq_msgs::msg::dds_::MotionRef_ m;
        for (auto &w : m.whl_joint()) {
            w.pos(0.0f);
            w.kp(0.0f);
            w.kd(d);
            w.torque(tq);
        }
        return m;
    };

    // publish the first reference before taking ownership so the wheels never see stale values
    pubRef.write(wheelRef(0.0f, kd));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    rbq_msgs::msg::dds_::JointOwnershipCmd_ own;
    own.whl_joint().fill(true);
    pubOwner.write(own);
    std::cout << "[rbq_wheel] wheel ownership requested (app 21)" << std::endl;

    // 500 Hz; ramp the torque over 0.5 s
    const auto period = std::chrono::milliseconds(2);
    const int  ticks  = static_cast<int>(seconds * 500.0f);
    auto next = std::chrono::steady_clock::now();
    for (int t = 0; t < ticks && g_running; t++) {
        const float ramp = std::min(1.0f, t / 250.0f);
        pubRef.write(wheelRef(torque * ramp, kd));

        if (t % 250 == 0) {
            std::lock_guard<std::mutex> lock(mx);
            if (got) {
                std::cout << "[rbq_wheel] applied:";
                for (int i = 0; i < 4; i++) {
                    std::cout << " " << kWheelNames[i] << "(tq=" << applied.whl_joint()[i].torque()
                              << " kd=" << applied.whl_joint()[i].kd() << ")";
                }
                std::cout << std::endl;
            }
        }
        next += period;
        std::this_thread::sleep_until(next);
    }

    // release: zero torque, then damping off - the wheels coast like a freshly started robot
    for (int t = 0; t < 250; t++) {
        pubRef.write(wheelRef(0.0f, kd));
        std::this_thread::sleep_for(period);
    }
    pubRef.write(wheelRef(0.0f, 0.0f));
    std::cout << "[rbq_wheel] done - wheels released (kp = kd = torque = 0)" << std::endl;

    std::lock_guard<std::mutex> lock(mx);
    if (!got) {
        std::cout << "[rbq_wheel] nothing on rt/rbq/ref/motion/_0 (Network not running?)" << std::endl;
    }
    return 0;
}
