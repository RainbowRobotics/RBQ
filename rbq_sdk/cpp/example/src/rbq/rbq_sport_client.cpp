#include <iostream>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <stdexcept>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/SportClient.hpp>

using namespace std;

struct TestOption
{
    std::string name;
    int id;
};

const vector<TestOption> option_list =
    {
        {"damp", 0},
        {"balance_stand", 1},
        {"stop_move", 2},
        {"stand_down", 3},
        {"recovery_stand", 4},
        {"move", 5},
        {"switch_gait", 6},
        {"speed_level", 7},
        {"get_state", 8},
        {"body_height", 9},
        {"stand_up", 10},
        
        // Caution:  test in open area
        {"enter_leftside_gait", 11},
        {"exit_leftside_gait", 12},
        {"enter_handstand", 13},
        {"exit_handstand", 14},
        {"pose", 15},
        {"euler", 16}

};

int ConvertToInt(const std::string &str)
{
    try
    {
        std::stoi(str);
        return std::stoi(str);
    }
    catch (const std::invalid_argument &)
    {
        return -1;
    }
    catch (const std::out_of_range &)
    {
        return -1;
    }
}

class UserInterface
{
public:
    UserInterface() {};
    ~UserInterface() {};

    void terminalHandle()
    {
        std::string input;
        std::getline(std::cin, input);

        // -1 = send nothing this round; keeping the old id would resend the last command
        test_option_->id = -1;

        if (input.compare("list") == 0)
        {
            for (TestOption option : option_list)
            {
                std::cout << option.name << ", id: " << option.id << std::endl;
            }
        }

        for (TestOption option : option_list)
        {
            if (input.compare(option.name) == 0 || ConvertToInt(input) == option.id)
            {
                test_option_->id = option.id;
                test_option_->name = option.name;
                std::cout << "Test: " << test_option_->name << ", test_id: " << test_option_->id << std::endl;
            }
        }
    };

    TestOption *test_option_;
};

int main(int argc, char **argv)
{
    // Defaults to loopback, matching the simulator and the on-robot stack.
    // Pass a NIC name to reach a robot from another host.
    const std::string interface = (argc > 1) ? argv[1] : "lo";
    std::cout << "Network interface: " << interface << std::endl;
    rbq_sdk::ChannelFactory::Instance().Init(0, interface);

    TestOption test_option;
    test_option.id = 1;

    rbq_sdk::SportClient sport_client;
    sport_client.SetNoReply(true);

    UserInterface user_interface;
    user_interface.test_option_ = &test_option;

    std::cout << "Input \"list \" to list all test option ..." << std::endl;
    long res_count = 0;
    while (1)
    {
        auto time_start_trick = std::chrono::high_resolution_clock::now();
        static const constexpr auto dt = std::chrono::microseconds(20000); // 50Hz

        user_interface.terminalHandle();
        if (test_option.id < 0)
        {
            continue;
        }

        int res = 1;
        if (test_option.id == 0)
        {
            res = sport_client.Damp();
        }
        else if (test_option.id == 1)
        {
            res = sport_client.BalanceStand();
        }
        else if (test_option.id == 2)
        {
            res = sport_client.StopMove();
        }
        else if (test_option.id == 3)
        {
            res = sport_client.StandDown();
        }
        else if (test_option.id == 4)
        {
            res = sport_client.RecoveryStand();
        }
        else if (test_option.id == 5)
        {
            res = sport_client.Move(0.0, 0.0, 0.5);
        }
        else if (test_option.id == 6)
        {
            res = sport_client.SwitchGait(0);
        }
        else if (test_option.id == 7)
        {
            res = sport_client.SpeedLevel(1);
        }
        else if (test_option.id == 8)
        {
            std::map<std::string, std::string> state_map;
            res = sport_client.GetState(state_map);
            for (const auto &kv : state_map)
                std::cout << kv.first << ": " << kv.second << std::endl;
        }
        else if (test_option.id == 9)
        {
            res = sport_client.BodyHeight(-0.1f);   // 10 cm lower than the default height
        }
        else if (test_option.id == 10)
        {
            res = sport_client.StandUp();
        }
        else if (test_option.id == 11)
        {
            res = sport_client.LeftSideGait(1);
        }
        else if (test_option.id == 12)
        {
            res = sport_client.LeftSideGait(0);
        }
        else if (test_option.id == 13)
        {
            res = sport_client.HandStand(1);
        }
        else if (test_option.id == 14)
        {
            res = sport_client.HandStand(0);
        }
        else if (test_option.id == 15)
        {
            res = sport_client.BodyPosition(0.0f, 0.0f, -0.1f, 0.2f);   // x, y must be 0
        }
        else if (test_option.id == 16)
        {
            res = sport_client.Euler(0.2f, 0.3f, 0.3f);
        }


        if (res != rbq_sdk::RPC_OK)
        {
            res_count += 1;
            std::cout << "Request error for: " << option_list[test_option.id].name << ", code: " << res << ", count: " << res_count << std::endl;
        }
        else
        {
            res_count = 0;
            std::cout << "Request successed: " << option_list[test_option.id].name << ", code: " << res << std::endl;
        }
        std::this_thread::sleep_until(time_start_trick + dt);
    }
    return 0;
}
