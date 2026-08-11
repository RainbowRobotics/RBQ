#include <string>
#include <iostream>
#include <mutex>
#include <atomic>
#include <deque>
#include <chrono>
#include <thread>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/SportClient.hpp>
#include <rbq_sdk/Joystick.hpp>
#include <rbq_sdk/Thread.hpp>

bool g_isWorking = true;
void signalHandler(int signal) {
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    std::_Exit(signal);
}

enum class SportCmd
{
    Damp,           // L2+B
    StopMove,       // BACK
    StandUp,        // L2+A first press: lock joints standing
    StandDown,      // L2+A second press: go prone
    DefaultMode,    // START
    RunningMode,    // L2+START
    LeftSide,       // Double-click X
    RightSide,      // Double-click Y
    Handstand,      // Double-click R1
    BipedStand,     // Double-click R2
    RecoveryStand,  // L2+X 
    Climb,          // R1+X
    FrontFlip,      // L1+X
    BackFlip,       // L1+Y
};

std::mutex cmd_mx;
std::deque<SportCmd> cmd_q;
void pushCmd(SportCmd c) {
    std::lock_guard<std::mutex> lk(cmd_mx);
    cmd_q.push_back(c);
};

std::atomic<float> move_vx{0.f};
std::atomic<float> move_vy{0.f};
std::atomic<float> move_vyaw{0.f};
std::atomic<long> err_streak{0};
void* controlLoop(void*);

constexpr float kDeadzone = 0.12f;
constexpr float kMaxVx = 0.55f;
constexpr float kMaxVy = 0.35f;
constexpr float kMaxVyaw = 1.0f;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "사용법: " << argv[0] << " <NIC> [조이경로]\n";
        return -1;
    }
    const std::string jpath = PickJoyPath(argc, argv);
    if (jpath.empty()) {
        PrintNoJoystickHint();
        return -1;
    }
    if (argc < 3) {
        std::cout << "조이 경로: " << jpath << '\n';
    }

    Joystick joy(jpath);
    if (!joy.Open())
        return -1;

    rbq_sdk::ChannelFactory::Instance().Init(0, argv[1]);

    std::thread controlThread = std::thread(controlLoop, nullptr);

    // A2 Robot's bluetooth gamepad bindings (emulated):
    //   L2 + B        : Damping (e-stop)
    //   L2 + A        : Stand/Prone toggle ("press again to go prone")
    //   START         : Default gait (unlock)
    //   L2 + START    : Running mode
    //   Double-click X: Left-side gait
    //   Double-click Y: Right-side gait
    //   Double-click R1: Handstand
    //   Double-click R2: Biped (erect) stand
    //   L2 + X / R1 + X: Recovery stand (stand up from fall)
    //   L1 + X        : Front flip
    //   L1 + Y        : Back flip
    //   BACK          : StopMove (escape; not in A2 native scheme)

    using clk_t = std::chrono::steady_clock;
    constexpr auto kDoubleClickWindow = std::chrono::milliseconds(400);
    const auto kFarPast = clk_t::time_point::min();
    auto last_x = kFarPast, last_y = kFarPast, last_rb = kFarPast, last_r2 = kFarPast;
    bool r2_prev_pressed = false;
    bool lock_toggle_to_prone = false;  // false → next L2+A goes StandUp; true → StandDown

    auto consume_double = [&](bool edge, clk_t::time_point& last) {
        if (!edge) return false;
        const auto now = clk_t::now();
        if (last != kFarPast && (now - last) < kDoubleClickWindow) {
            last = kFarPast;        // consume; require a fresh pair for the next double
            return true;
        }
        last = now;                  // first of a potential pair
        return false;
    };

    // A2 Robot's bluetooth gamepad button to robot mode switch bindings
    // =================================================================
    // Button           : Mode Switch
    // -----------------------------------------------------------------
    // L2 + B           : Damping Mode(Emergency stop)
    // L2 + A           : Locking Posture 1: lock joints when standing
    // L2 + A           : Locking Posture 2: Press again to go prone
    // START            : Unlock/Default Gait
    // L2 + START       : Running
    // Double Click X   : Left Side
    // Double Click Y   : Right Side
    // Double Click R1  : Handstand
    // Double Click R2  : Erect
    // ----------------------------------------------------------------
    // ----------------------------------------------------------------
    // Button           : Customized Movements
    // -----------------------------------------------------------------
    // L2 + X           : Stand up from fall
    // R1 + X           : Climb
    // L1 + X           : Front Flip
    // L1 + Y           : Back Flip

    while (true)
    {
        joy.Poll();

        const bool L1 = joy.Btn(PAD_LB);
        const bool R1 = joy.Btn(PAD_RB);
        const bool L2 = joy.Ax(AXIS_LT) > 0.f;  // trigger idle is -1, full press +1
        // R2 rising edge from analog axis (treat as virtual button).
        const bool r2_pressed = joy.Ax(AXIS_RT) > 0.f;
        const bool R2_edge = r2_pressed && !r2_prev_pressed;
        r2_prev_pressed = r2_pressed;

        // ---- chord bindings (checked before single-button defaults) ----
        if (joy.Edge(PAD_B) && L2) {
            pushCmd(SportCmd::Damp);
        } else if (joy.Edge(PAD_A) && L2) {
            pushCmd(lock_toggle_to_prone ? SportCmd::StandDown : SportCmd::StandUp);
            lock_toggle_to_prone = !lock_toggle_to_prone;
        } else if (joy.Edge(PAD_START)) {
            pushCmd(L2 ? SportCmd::RunningMode : SportCmd::DefaultMode);
        } else if (joy.Edge(PAD_X) && L2) {
            pushCmd(SportCmd::RecoveryStand);
        } else if (joy.Edge(PAD_X) && R1) {
            pushCmd(SportCmd::Climb);
        } else if (joy.Edge(PAD_X) && L1) {
            pushCmd(SportCmd::FrontFlip);
        } else if (joy.Edge(PAD_Y) && L1) {
            pushCmd(SportCmd::BackFlip);
        }
        // ---- double-click bindings on X, Y, R1 (button) and R2 (trigger) ----
        else if (consume_double(joy.Edge(PAD_X), last_x)) {
            pushCmd(SportCmd::LeftSide);
        } else if (consume_double(joy.Edge(PAD_Y), last_y)) {
            pushCmd(SportCmd::RightSide);
        } else if (consume_double(joy.Edge(PAD_RB), last_rb)) {
            pushCmd(SportCmd::Handstand);
        } else if (consume_double(R2_edge, last_r2)) {
            pushCmd(SportCmd::BipedStand);
        }
        // ---- escape ----
        else if (joy.Edge(PAD_BACK)) {
            pushCmd(SportCmd::StopMove);
        }

        const float lx = Deadzone(joy.Ax(AXIS_LEFT_X), kDeadzone);
        const float ly = Deadzone(joy.Ax(AXIS_LEFT_Y), kDeadzone);
        const float rx = Deadzone(joy.Ax(AXIS_RIGHT_X), kDeadzone);
        move_vx.store(-ly * kMaxVx, std::memory_order_relaxed);
        move_vy.store(-lx * kMaxVy, std::memory_order_relaxed);
        move_vyaw.store(-rx * kMaxVyaw, std::memory_order_relaxed);

        joy.EndFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void* controlLoop(void*)
{
    std::cout << "Control loop started.\n";
    usleep(100 * 1000);

    timespec timeEnd;
    timespec timeNext;
    clock_gettime(CLOCK_REALTIME, &timeNext);

    rbq_sdk::SportClient sport;
    sport.SetNoReply(true);

    while (g_isWorking) {

        bool had_discrete = false;
        while (g_isWorking)
        {
            SportCmd c;
            {
                std::lock_guard<std::mutex> lk(cmd_mx);
                if (cmd_q.empty())
                    break;
                c = cmd_q.front();
                cmd_q.pop_front();
            }
            had_discrete = true;
            int32_t res = 0;
            switch (c)
            {
            case SportCmd::Damp:
                res = sport.Damp();
                std::cout << "Damp\n";
                break;
            case SportCmd::StopMove:
                res = sport.StopMove();
                std::cout << "StopMove\n";
                break;
            case SportCmd::StandUp:
                res = sport.StandUp();
                std::cout << "StandUp (lock)\n";
                break;
            case SportCmd::StandDown:
                res = sport.StandDown();
                std::cout << "StandDown (prone)\n";
                break;
            case SportCmd::DefaultMode:   
                res = sport.SwitchGait(rbq_sdk::ID_DEFAULT_MODE);
                std::cout << "DefaultMode\n"; break;
            case SportCmd::RunningMode:   
                res = sport.SwitchGait(rbq_sdk::ID_RUNNING_MODE);
                std::cout << "RunningMode\n"; 
                break;
            case SportCmd::LeftSide:      
                res = sport.LeftSideGait(1);
                std::cout << "LeftSide\n"; 
                break;
            case SportCmd::RightSide:     
                res = sport.RightSideGait(1);
                std::cout << "RightSide\n"; 
                break;
            case SportCmd::Handstand:     
                res = sport.HandStand(1);
                std::cout << "Handstand\n"; 
                break;
            case SportCmd::BipedStand:    
                res = sport.BipedStand(1);
                std::cout << "BipedStand\n"; 
                break;
            case SportCmd::RecoveryStand: 
                res = sport.RecoveryStand();
                std::cout << "RecoveryStand\n"; 
                break;
            case SportCmd::Climb:
                res = sport.SwitchGait(rbq_sdk::ID_CLIMB_MODE);
                std::cout << "Climb\n";
                break;
            case SportCmd::FrontFlip:     
                res = sport.FrontFlip();
                std::cout << "FrontFlip\n"; 
                break;
            case SportCmd::BackFlip:      
                res = sport.BackFlip();
                std::cout << "BackFlip\n"; 
                break;
            }
            if (res < 0) {
                const long s = err_streak.fetch_add(1) + 1;
                if (s % 50 == 1)
                    std::cerr << "Sport error " << res << " (" << s << ")\n";
            }
            else
                err_streak.store(0);

            std::cout << "res: " << res << "\n";
        }

        if (!had_discrete) {
            int32_t res = sport.Move(move_vx.load(std::memory_order_relaxed),
                                     move_vy.load(std::memory_order_relaxed),
                                     move_vyaw.load(std::memory_order_relaxed));
            if (res != rbq_sdk::RPC_OK) {
                const long s = err_streak.fetch_add(1) + 1;
                if (s % 50 == 1)
                    std::cerr << "Sport error " << res << " (" << s << ")\n";
            }
            else
                err_streak.store(0);
        }


        clock_gettime(CLOCK_REALTIME, &timeEnd);
        rbq_sdk::Thread::timespec_add_us(&timeNext, 100 * 1000);
        if (rbq_sdk::Thread::timespec_cmp(&timeEnd, &timeNext) > 0) {
            std::cout << "Thread deadline ms: " << rbq_sdk::Thread::timediff_us(&timeNext, &timeEnd)*0.001 << " ms over!\n";
            clock_gettime(CLOCK_REALTIME, &timeNext);
        } else {
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
        }
    }
    std::cout << "Control loop exiting.\n";
    return nullptr;
}
