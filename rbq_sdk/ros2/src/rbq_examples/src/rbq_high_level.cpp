#include <atomic>
#include <array>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/joystick.h>
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
    explicit RbqHighLevel(const std::string& joy_path) : Node("rbq_high_level"), joy_path_(joy_path)
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

        joy_axes_.fill(0.f);
        printHelp();
    }

    void runTerminal()
    {
        std::string input;
        while (rclcpp::ok() && std::getline(std::cin, input)) {
            handleInput(input);
        }
    }

    void runJoystick()
    {
        int fd = open(joy_path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            std::cout << "  Joystick not found: " << joy_path_ << "\n";
            return;
        }
        std::cout << "  Joystick opened: " << joy_path_ << "\n";

        js_event ev;
        while (rclcpp::ok()) {
            ssize_t n = read(fd, &ev, sizeof(ev));
            if (n != sizeof(ev)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if ((ev.type & ~JS_EVENT_INIT) != JS_EVENT_AXIS) continue;
            if (ev.number >= joy_axes_.size()) continue;

            float v = static_cast<float>(ev.value) / 32767.0f;
            joy_axes_[ev.number] = v;

            // update velocity atomics for the high_level command path
            switch (ev.number) {
                case 0: vel_y_.store(  deadzone(-v, kDeadzone) * kMaxVy,   std::memory_order_relaxed); break;
                case 1: vel_x_.store(  deadzone(-v, kDeadzone) * kMaxVx,   std::memory_order_relaxed); break;
                case 3: omega_z_.store(deadzone(-v, kDeadzone) * kMaxVyaw, std::memory_order_relaxed); break;
                default: break;
            }
        }
        close(fd);
    }

private:
    std::string joy_path_;
    std::array<float, 8> joy_axes_;

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
                  << "            type \"list\" for all options\n"
                  << "  Joystick: left-stick=vel_x/vel_y  right-stick=omega_z\n\n";
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string joy_path = "/dev/input/js0";
    if (argc >= 2) joy_path = argv[1];

    auto node = std::make_shared<RbqHighLevel>(joy_path);
    std::thread terminal([&node]() { node->runTerminal(); });
    std::thread joystick([&node]() { node->runJoystick(); });
    rclcpp::spin(node);
    rclcpp::shutdown();
    terminal.join();
    joystick.join();
    return 0;
}
