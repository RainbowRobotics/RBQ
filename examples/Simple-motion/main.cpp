#include <iostream>
#include <memory>
#include <signal.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <pthread.h>
#include <ctime>
#include <Eigen/Dense>

#include <onnxruntime_cxx_api.h>

#include "rbq/Api.h"
#include "rbq/Thread.h"
#include "rbq/Parameters.h"
#include "JointControl.h"

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;
constexpr long kControlPeriodUs = kControlPeriodMs * 1000;

JointTable g_jointTable;
std::unique_ptr<RBQ_API> g_api;
std::unique_ptr<JointController> g_jointController;

int g_jointJogNo = 0;
int g_jogMoveDir = 0;
bool g_isWorking = false;

struct termios g_oldTio, g_newTio;
bool g_keyboardInitialized = false;

enum class TaskState {
    Idle = 0,
    Motion,
    JogMove,
    PositionLock,
    Control,
};
TaskState g_currentTask = TaskState::Idle;

enum class UserCommand {
    None = 0,
    Test,
    MotionReady,
    MotionGround,
    JointLockToggle,
};

void initializeKeyboard();
void cleanupKeyboard();
char readKeyboard();
void signalHandler(int signal);
void cleanupResources();
void goToMotionReady();
void goToMotionGround();
void* controlLoop(void*);

int main(int argc, char* argv[]) {
    std::cout << "Starting RobotControlApp...\n";
    std::cout << "Press 'x' for MOTION_GROUND, 'z' for MOTION_READY, 'q' to quit.\n";

    initializeKeyboard();

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
    signal(SIGHUP, signalHandler);
    signal(SIGSEGV, signalHandler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "Memory lock failed.\n";
        return 1;
    }

    Parameters params;
    if (params.Initialize("hw_configs/QuadParameter.ini")) {
        std::cerr << "Failed to initialize parameters.\n";
        return 1;
    }

    try {
        g_api = std::make_unique<RBQ_API>(20);
        g_jointController = std::make_unique<JointController>(g_api.get(), kMaxJoint);
        g_jointController->syncReferenceToRobot();
    } catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << "\n";
        cleanupResources();
        return 1;
    }

    std::cout << "Control thread starting...\n";
    pthread_t controlThread;
    if (!Thread::generate_rt_thread(controlThread, controlLoop, "ControlLoop", 1, 90, nullptr)) {
        std::cerr << "Failed to create control thread\n";
        cleanupResources();
        return 1;
    }

    g_isWorking = true;

    while (g_isWorking && g_api && g_jointController) {
        char key = readKeyboard();
        UserCommand command = UserCommand::None;

        switch (key) {
            case 'x': command = UserCommand::MotionGround; break;
            case 'z': command = UserCommand::MotionReady; break;
            case 'q': g_isWorking = false; break;
        }

        switch (command) {
            case UserCommand::MotionReady:
                std::cout << "Executing Motion Ready...\n";
                g_currentTask = TaskState::Motion;
                goToMotionReady();
                g_currentTask = TaskState::Idle;
                break;

            case UserCommand::MotionGround:
                std::cout << "Executing Motion Ground...\n";
                g_currentTask = TaskState::Motion;
                goToMotionGround();
                g_currentTask = TaskState::Idle;
                break;

            default:
                break;
        }
    }

    std::cout << "Shutting down...\n";
    cleanupKeyboard();
    cleanupResources();
    return 0;
}

void* controlLoop(void*) {
    std::cout << "Control loop started.\n";

    timespec timeNext;
    usleep(100 * 1000);
    clock_gettime(CLOCK_REALTIME, &timeNext);

    while (g_isWorking && g_api && g_jointController) {

        switch (g_currentTask) {
            case TaskState::Idle:
                break;

            case TaskState::Motion:
                g_jointController->updateAllJoints();
                g_jointController->sendReferencesToRobot();
                break;

            case TaskState::JogMove:
                // Add jog logic here if needed
                break;

            case TaskState::Control:
                // Add custom control logic here
                break;

            default:
                break;
        }

        Thread::timespec_add_us(&timeNext, kControlPeriodUs);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
        clock_gettime(CLOCK_REALTIME, &timeNext);
    }

    std::cout << "Control loop exiting.\n";
    return nullptr;
}

void initializeKeyboard() {
    if (g_keyboardInitialized) return;
    tcgetattr(STDIN_FILENO, &g_oldTio);
    g_newTio = g_oldTio;
    g_newTio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &g_newTio);
    g_keyboardInitialized = true;
}

void cleanupKeyboard() {
    if (g_keyboardInitialized) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_oldTio);
        g_keyboardInitialized = false;
    }
}

char readKeyboard() {
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) return c;
    return 0;
}

void signalHandler(int signal) {
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    cleanupResources();
    std::_Exit(signal);
}

void cleanupResources() {
    std::cout << "Cleaning up...\n";
    g_jointController.reset();
    g_api.reset();
    std::cout << "Resources cleaned.\n";
}

void goToMotionReady() {
    if (!g_jointController || !g_api) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    for (int i = 0; i < kMaxJoint; ++i) {
        g_api->Joint.setGainKp(i, 200.0f);
        g_api->Joint.setGainKd(i, 2.5f);
    }
    usleep(500 * 1000);

    float motionTime = 1400.0f;
    float pitchAngles[4];
    g_api->Joint.getPosRef(RBQ_API::JOINT::JointID::HRP, pitchAngles[0]);
    g_api->Joint.getPosRef(RBQ_API::JOINT::JointID::HLP, pitchAngles[1]);
    g_api->Joint.getPosRef(RBQ_API::JOINT::JointID::FRP, pitchAngles[2]);
    g_api->Joint.getPosRef(RBQ_API::JOINT::JointID::FLP, pitchAngles[3]);

    bool isGrounded = (pitchAngles[0] > 60 * kD2R && pitchAngles[1] > 60 * kD2R &&
                       pitchAngles[2] > 60 * kD2R && pitchAngles[3] > 60 * kD2R);

    const auto& table = g_jointTable;
    const auto& jointCmd = isGrounded ? table.folding : table.ready;

    for (int i = 0; i < kMaxJoint; ++i)
        g_jointController->moveJoint(i, jointCmd[i] * kD2R, motionTime, MoveCommandMode::Absolute);
    usleep((motionTime + 100) * 1000);

    if (isGrounded) {
        for (int i = 0; i < kMaxJoint; ++i)
            g_jointController->moveJoint(i, table.ready[i] * kD2R, motionTime, MoveCommandMode::Absolute);
        usleep((motionTime + 100) * 1000);
    }
}

void goToMotionGround() {
    if (!g_jointController || !g_api) return;
    g_jointController->syncReferenceToRobot();
    g_jointController->setAllOwners();

    for (int i = 0; i < kMaxJoint; ++i) {
        g_api->Joint.setGainKp(i, 200.0f);
        g_api->Joint.setGainKd(i, 2.5f);
    }

    float motionTime = 2400.0f;
    for (int i = 0; i < kMaxJoint; ++i)
        g_jointController->moveJoint(i, g_jointTable.folding[i] * kD2R, motionTime, MoveCommandMode::Absolute);
    usleep((motionTime + 100) * 1000);

    for (int i = 0; i < kMaxJoint; ++i)
        g_jointController->moveJoint(i, g_jointTable.ground[i] * kD2R, motionTime, MoveCommandMode::Absolute);
    usleep((motionTime + 100) * 1000);
}
