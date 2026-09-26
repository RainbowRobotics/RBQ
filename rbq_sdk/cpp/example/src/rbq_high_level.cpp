#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <limits>
#include <atomic>
#include <stdexcept>
#include <csignal>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/rbq/HighLevelCommand_.hpp>
#include <rbq_sdk/idl/ros2/Bool_.hpp>
#include <rbq_sdk/Joystick.hpp>
#include <rbq_sdk/Thread.hpp>

using namespace std;

enum GAIT_STATE : int8_t {
    SITTING              = 0,
    STANDING             = 1,
    AIMING               = 2,
    TROTTING             = 3,
    TROT_STAIRS          = 4,
    WAVING               = 5,
    TROT_RUNNING         = 6,
    DOCKING              = 10,
    RL_TROT              = 30,
    RL_FRONT_WALK        = 31,
    RL_LEFT_WALK         = 33,
    RL_RIGHT_WALK        = 34,
    RL_BOUND             = 35,
    RL_PACE              = 36,
    RL_PRONK             = 37,
    RL_3LEG_HR           = 38,
    RL_3LEG_HL           = 39,
    RL_3LEG_FL           = 41,
    RL_TROT_VISION       = 42,
    RL_TROT_RUN          = 45,
    RL_SILENT            = 46,
    RL_STAIRS            = 47,
    RL_END               = 80,
};

struct TestOption
{
    std::string name;
    int id;
};

const vector<TestOption> option_list =
    {
        {"sitting",              SITTING             },
        {"standing",             STANDING            },
        {"aiming",               AIMING              },
        {"trotting",             TROTTING            },
        {"trot_stairs",          TROT_STAIRS         },
        {"waving",               WAVING              },
        {"trot_running",         TROT_RUNNING        },
        {"docking",              DOCKING             },
        {"rl_trot",              RL_TROT             },
        {"rl_front_walk",        RL_FRONT_WALK       },
        {"rl_left_walk",         RL_LEFT_WALK        },
        {"rl_right_walk",        RL_RIGHT_WALK       },
        {"rl_bound",             RL_BOUND            },
        {"rl_pace",              RL_PACE             },
        {"rl_pronk",             RL_PRONK            },
        {"rl_3leg_hr",           RL_3LEG_HR          },
        {"rl_3leg_hl",           RL_3LEG_HL          },
        {"rl_3leg_fl",           RL_3LEG_FL          },
        {"rl_trot_vision",       RL_TROT_VISION      },
        {"rl_trot_run",          RL_TROT_RUN         },
        {"rl_silent",            RL_SILENT           },
        {"rl_stairs",            RL_STAIRS           },
        {"rl_end",               RL_END              },
};

// -1 is a real gait_state (CONTROL_OFF), so use INT_MIN as the parse-failure
// sentinel instead of -1 like sport_client does.
constexpr int kInvalidId = std::numeric_limits<int>::min();

constexpr float kDeadzone = 0.12f;
constexpr float kMaxVx    = 1.2f;   // m/s
constexpr float kMaxVy    = 0.5f;   // m/s
// in deg/s — 60 deg/s ~ 1.05 rad/s of yaw.
constexpr float kMaxVyaw  = 60.0f;   // deg/s

std::atomic<bool>   g_isWorking{true};
std::atomic<int8_t> g_targetGait{static_cast<int8_t>(STANDING)};
std::atomic<float>  g_velX{0.f};
std::atomic<float>  g_velY{0.f};
std::atomic<float>  g_omegaZ{0.f};

void signalHandler(int sig)
{
    std::cout << "Signal received: " << sig << "\n";
    g_isWorking.store(false);
    std::_Exit(sig);
}

int ConvertToInt(const std::string &str)
{
    try {
        return std::stoi(str);
    } catch (const std::invalid_argument &) {
        return kInvalidId;
    } catch (const std::out_of_range &) {
        return kInvalidId;
    }
}

void printHelp()
{
    std::cout << "\n " << APP_NAME << " Help\n";
    std::cout << "\n Usage: " << APP_NAME << " <networkInterface> [joystickPath]\n";
    std::cout << "\n Gait controls (terminal input):\n";
    std::cout << "   list           - list all gait options\n";
    std::cout << "   <name>         - select gait by name (e.g. standing)\n";
    std::cout << "   <id>           - select gait by id   (e.g. 1)\n";
    std::cout << "\n Motion controls (joystick):\n";
    std::cout << "   Left stick     - vel_x / vel_y\n";
    std::cout << "   Right stick X  - omega_z\n\n";
}

// Publishes the current gait + joystick-driven velocities at 50 Hz.
void* controlLoop(void*)
{
    timespec timeEnd;
    timespec timeNext;
    clock_gettime(CLOCK_REALTIME, &timeNext);

    rbq_msgs::msg::dds_::HighLevelCommand_ msg;
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::HighLevelCommand_> pubHighLevelCmd("rt/rbq/cmd/high_level");

    rbq_sdk::Publisher<std_msgs::msg::dds_::Bool_> pubSwitchControlMode("rt/rbq/cmd/switch_control_mode");
    {
        std_msgs::msg::dds_::Bool_ on;
        on.data() = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        pubSwitchControlMode.write(on);
    }

    while (g_isWorking.load()) {
        msg.gait_state()      = g_targetGait.load(std::memory_order_relaxed);
        msg.gait_transition() = true;
        msg.vel_x()           = g_velX.load(std::memory_order_relaxed);
        msg.vel_y()           = g_velY.load(std::memory_order_relaxed);
        msg.omega_z()         = g_omegaZ.load(std::memory_order_relaxed);
        pubHighLevelCmd.write(msg);

        clock_gettime(CLOCK_REALTIME, &timeEnd);
        rbq_sdk::Thread::timespec_add_us(&timeNext, 20 * 1000); // 50 Hz
        if (rbq_sdk::Thread::timespec_cmp(&timeEnd, &timeNext) > 0) {
            clock_gettime(CLOCK_REALTIME, &timeNext);
        } else {
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
        }
    }

    return nullptr;
}

// Polls the joystick at 100 Hz and updates atomic velocity setpoints.
void* joystickLoop(void* arg)
{
    Joystick* joy = static_cast<Joystick*>(arg);
    while (g_isWorking.load()) {
        joy->Poll();
        const float lx = Deadzone(joy->Ax(AXIS_LEFT_X),  kDeadzone);
        const float ly = Deadzone(joy->Ax(AXIS_LEFT_Y),  kDeadzone);
        const float rx = Deadzone(joy->Ax(AXIS_RIGHT_X), kDeadzone);
        g_velX.store(-ly * kMaxVx,   std::memory_order_relaxed);
        g_velY.store(-lx * kMaxVy,   std::memory_order_relaxed);
        g_omegaZ.store(-rx * kMaxVyaw, std::memory_order_relaxed);
        joy->EndFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return nullptr;
}

class UserInterface
{
public:
    void terminalHandle()
    {
        std::string input;
        if (!std::getline(std::cin, input)) {
            g_isWorking.store(false);
            return;
        }

        if (input.compare("list") == 0) {
            for (const TestOption &option : option_list) {
                std::cout << option.name << ", id: " << option.id << "\n";
            }
            return;
        }

        for (const TestOption &option : option_list) {
            if (input.compare(option.name) == 0 || ConvertToInt(input) == option.id) {
                test_option_->id   = option.id;
                test_option_->name = option.name;
                g_targetGait.store(static_cast<int8_t>(option.id), std::memory_order_relaxed);
                std::cout << "Gait: " << test_option_->name << ", gait_state: " << test_option_->id << "\n";
                return;
            }
        }
    }

    TestOption *test_option_;
};

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <networkInterface> [joystickPath]\n";
        return -1;
    }

    signal(SIGTERM, signalHandler);
    signal(SIGINT,  signalHandler);
    signal(SIGHUP,  signalHandler);

    const std::string jpath = PickJoyPath(argc, argv);
    if (jpath.empty()) {
        PrintNoJoystickHint();
        return -1;
    }
    if (argc < 3) {
        std::cout << "Joystick path: " << jpath << '\n';
    }

    Joystick joy(jpath);
    if (!joy.Open()) {
        return -1;
    }

    rbq_sdk::ChannelFactory::Instance().Init(0, argv[1]);

    TestOption test_option;
    test_option.id   = STANDING;
    test_option.name = "standing";
    g_targetGait.store(static_cast<int8_t>(STANDING));

    UserInterface user_interface;
    user_interface.test_option_ = &test_option;

    printHelp();
    std::cout << "Input \"list\" to list all gait options ...\n";

    std::thread joystickThread(joystickLoop, &joy);
    std::thread controlThread(controlLoop, nullptr);

    while (g_isWorking.load()) {
        user_interface.terminalHandle();
    }

    if (joystickThread.joinable()) joystickThread.join();
    if (controlThread.joinable())  controlThread.join();
    return 0;
}
