#include <atomic>
#include <array>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include <rbq_msgs/msg/high_level_command.hpp>
#include <rbq_msgs/msg/robot_status.hpp>

using HighLevelCommand = rbq_msgs::msg::HighLevelCommand;
using RobotStatus      = rbq_msgs::msg::RobotStatus;

struct GaitOption {
    std::string name;
    int8_t      id;
};

static const std::vector<GaitOption> kGaitOptions = {
    {"sit",            HighLevelCommand::STATE_SIT},
    {"stand",          HighLevelCommand::STATE_STAND},
    {"aim",            HighLevelCommand::STATE_AIM},
    {"walk",           HighLevelCommand::STATE_WALK},
    {"stairs",         HighLevelCommand::STATE_STAIRS},
    {"wave",           HighLevelCommand::STATE_WAVE},
    {"run",            HighLevelCommand::STATE_RUN},
    {"rl_trot",        HighLevelCommand::STATE_RL_TROT},
    {"rl_front_walk",  HighLevelCommand::STATE_RL_FRONT_WALK},
    {"rl_left_walk",   HighLevelCommand::STATE_RL_LEFT_WALK},
    {"rl_right_walk",  HighLevelCommand::STATE_RL_RIGHT_WALK},
    {"rl_bound",       HighLevelCommand::STATE_RL_BOUND},
    {"rl_pace",        HighLevelCommand::STATE_RL_PACE},
    {"rl_pronk",       HighLevelCommand::STATE_RL_PRONK},
    {"rl_3leg_hr",     HighLevelCommand::STATE_RL_3LEG_HR},
    {"rl_3leg_hl",     HighLevelCommand::STATE_RL_3LEG_HL},
    {"rl_3leg_fr",     HighLevelCommand::STATE_RL_3LEG_FR},
    {"rl_3leg_fl",     HighLevelCommand::STATE_RL_3LEG_FL},
    {"rl_trot_vision", HighLevelCommand::STATE_RL_TROT_VISION},
    {"rl_trot_run",    HighLevelCommand::STATE_RL_TROT_RUN},
    {"rl_silent",      HighLevelCommand::STATE_RL_SILENT},
    {"rl_stairs",      HighLevelCommand::STATE_RL_STAIRS},
};

static constexpr float kDeadzone = 0.12f;
static constexpr float kMaxVx    = 1.2f;
static constexpr float kMaxVy    = 0.5f;
static constexpr float kMaxVyaw  = 60.0f;

static float deadzone(float v, float dz)
{
    return (std::abs(v) < dz) ? 0.0f : v;
}

class RbqHighLevel : public rclcpp::Node {
public:
    explicit RbqHighLevel(const std::string) : Node("rbq_high_level")
    {
        pub_cmd_  = create_publisher<HighLevelCommand>("/rbq/cmd/high_level", 10);

        pub_mode_ = create_publisher<std_msgs::msg::Bool>("/rbq/cmd/switch_control_mode", 10);
        pub_gait_ = create_publisher<std_msgs::msg::Int8>("/rbq/cmd/switch_gait", 10);

        sub_status_ = create_subscription<RobotStatus>(
            "/rbq/robot_status", rclcpp::QoS(10).best_effort(),
            [this](RobotStatus::SharedPtr msg) { (void)msg; });

        timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                   [this]() { onTimer(); });

        mode_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
            std_msgs::msg::Bool mode;
            mode.data = true;
            pub_mode_->publish(mode);
            if (++mode_send_count_ >= 1) mode_timer_->cancel();
        });

        printHelp();
    }

    void runTerminal()
    {
        std::string input;
        while (rclcpp::ok() && std::getline(std::cin, input)) {
            handleInput(input);
        }
    }

private:

    rclcpp::Publisher<HighLevelCommand>::SharedPtr         pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      pub_mode_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr      pub_gait_;
    rclcpp::Subscription<RobotStatus>::SharedPtr           sub_status_;
    rclcpp::TimerBase::SharedPtr                           timer_;
    rclcpp::TimerBase::SharedPtr                           mode_timer_;
    int                                                    mode_send_count_{0};

    std::atomic<int8_t> gait_{HighLevelCommand::STATE_STAND};
    std::atomic<float>  vel_x_{0.f};
    std::atomic<float>  vel_y_{0.f};
    std::atomic<float>  omega_z_{0.f};

    void onTimer()
    {
        auto stamp = now();

        // high_level command (gait + velocity)
        HighLevelCommand cmd;
        cmd.header.stamp    = stamp;
        cmd.gait_state      = gait_.load(std::memory_order_relaxed);
        cmd.gait_transition = true;
        cmd.vel_x           = vel_x_.load(std::memory_order_relaxed);
        cmd.vel_y           = vel_y_.load(std::memory_order_relaxed);
        cmd.omega_z         = omega_z_.load(std::memory_order_relaxed);
        pub_cmd_->publish(cmd);

    }

    void handleInput(const std::string& input)
    {
        if (input == "list") {
            for (const auto& opt : kGaitOptions) {
                std::cout << "  " << opt.name << " (id=" << static_cast<int>(opt.id) << ")\n";
            }
            return;
        }
        for (const auto& opt : kGaitOptions) {
            bool byName = (input == opt.name);
            bool byId   = false;
            try { byId = (std::stoi(input) == static_cast<int>(opt.id)); } catch (...) {}
            if (byName || byId) {
                gait_.store(opt.id, std::memory_order_relaxed);
                std_msgs::msg::Int8 gait_msg;
                gait_msg.data = opt.id;
                pub_gait_->publish(gait_msg);
                std::cout << "Gait -> " << opt.name << " (id=" << static_cast<int>(opt.id) << ")\n";
                return;
            }
        }
        std::cout << "Unknown: \"" << input << "\". Type \"list\" for options.\n";
    }

    static void printHelp()
    {
        std::cout << "\n [rbq_high_level]\n"
                  << "  Terminal: type gait name or id (e.g. \"stand\", \"3\")\n"
                  << "            type \"list\" for all options\n";
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RbqHighLevel>("");
    std::thread terminal([&node]() { node->runTerminal(); });
    rclcpp::spin(node);
    rclcpp::shutdown();
    terminal.join();
    return 0;
}
