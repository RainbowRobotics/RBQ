#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rbq_msgs/msg/target_go.hpp>

// gait constants (mirror QuadWalk_TARGET_GO handler)
// 0: wave walk  1: stairs walk  2: RL walk
static constexpr int GAIT_WAVE   = 0;
static constexpr int GAIT_STAIRS = 1;
static constexpr int GAIT_RL     = 2;

// approach mode
// 0: rotate→straight→rotate
// 1: rotate to theta → diagonal walk
// 2: diagonal walk → rotate to theta
// 3: simultaneous rotate + diagonal walk
static constexpr int MODE_DEFAULT = 0;

static void printUsage()
{
    std::cout <<
        "Usage: rbq_target_go [x] [y] [theta] [gait] [mode]\n"
        "\n"
        "  x          Target X in metres, relative to robot start  (default 1.0)\n"
        "  y          Target Y in metres, relative to robot start  (default 0.0)\n"
        "  theta      Target heading in radians                    (default 0.0)\n"
        "  gait       0=wave(default) / 1=stairs / 2=rl\n"
        "  mode       0=rot-straight-rot(default) / 1=rot-then-diag /\n"
        "             2=diag-then-rot / 3=rot+diag simultaneous\n"
        "\n"
        "Requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp on domain 0.\n"
        "Network process (started by sim.bash / Motion) handles the command directly.\n"
        "\n"
        "Example (sim, 1 m forward with wave gait):\n"
        "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
        "  export CYCLONEDDS_URI='<CycloneDDS><Domain id=\"0\"><General><Interfaces>"
        "<NetworkInterface name=\"lo\"/></Interfaces></General></Domain></CycloneDDS>'\n"
        "  ros2 run rbq_examples rbq_target_go -- 1.0 0.0 0.0 0 0\n";
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // parse positional args that come after ROS2 stripped its own flags
    const auto args = rclcpp::remove_ros_arguments(argc, argv);

    if (args.size() > 1 && (args[1] == "-h" || args[1] == "--help")) {
        printUsage();
        return 0;
    }

    const float x         = (args.size() > 1) ? std::stof(args[1]) : 1.0f;
    const float y         = (args.size() > 2) ? std::stof(args[2]) : 0.0f;
    const float theta     = (args.size() > 3) ? std::stof(args[3]) : 0.0f;
    const int   gait      = (args.size() > 4) ? std::stoi(args[4]) : GAIT_WAVE;
    const int   mode      = (args.size() > 5) ? std::stoi(args[5]) : MODE_DEFAULT;

    auto node = rclcpp::Node::make_shared("rbq_target_go");
    auto pub  = node->create_publisher<rbq_msgs::msg::TargetGo>(
        "rbq/cmd/target_go", 10);

    // spin briefly so the publisher is discovered before we publish and exit
    rclcpp::Rate rate(10);
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rbq_msgs::msg::TargetGo msg;
    msg.gait      = gait;
    msg.mode      = mode;
    msg.x         = x;
    msg.y         = y;
    msg.theta     = theta;
    msg.offset_x  = 0.0f;
    msg.offset_y  = 0.0f;
    msg.slow      = false;
    msg.wide      = false;
    msg.vision    = false;

    pub->publish(msg);

    RCLCPP_INFO(node->get_logger(),
        "published: gait=%d mode=%d x=%.2f y=%.2f theta=%.3f",
        gait, mode, x, y, theta);

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
