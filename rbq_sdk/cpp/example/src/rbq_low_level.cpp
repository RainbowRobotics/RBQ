#include <thread>
#include <signal.h>
#include <termios.h>

#include <Eigen/Dense>

#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/rbq/MotionRef_.hpp>
#include <rbq_sdk/idl/rbq/JointOwnershipCmd_.hpp>
#include <rbq_sdk/idl/rbq/UserCommand_.hpp>
#include <rbq_sdk/idl/rbq/LegJointInfo_.hpp>
#include <rbq_sdk/idl/ros2/Imu_.hpp>
#include <rbq_sdk/idl/ros2/Joy_.hpp>

#include <rbq_sdk/JointControl.hpp>
#include <rbq_sdk/Thread.hpp>
#include <rbq_sdk/Timer.hpp>
#include <rbq_sdk/Policy.hpp>
#include <rbq_sdk/Filter.hpp>

constexpr float kR2D = 57.295779513f;
constexpr float kD2R = 0.0174532925f;

rbq_sdk::JointControl g_jointControl;
rbq_msgs::msg::dds_::MotionRef_ msgMotionRefFinal;

std::string g_policyPath = "";

bool g_isWorking = true;

enum class TaskState {
    Idle = 0,
    Motion,
    Control,
};
TaskState g_currentTask = TaskState::Idle;

void signalHandler(int signal) {
    std::cout << "Signal received: " << signal << "\n";
    g_isWorking = false;
    std::_Exit(signal);
}
void goToMotionReady();
void goToMotionGround();
void* controlLoop(void*);

void printHelp() {
    std::cout << "\n";
    std::cout << " " << APP_NAME << " Help\n";
    std::cout << "\n -p | --path <path> : Specify the policy path to load\n";
    std::cout << "\n";
    std::cout << " Controls:\n";
    std::cout << "  'q' - Quit the application\n";
    std::cout << "  'z' - Motion Sit\n";
    std::cout << "  'x' - Motion Stand\n";
    std::cout << "  'c' - Motion Walk\n";
    std::cout << " Velocity Control (when in control mode):\n";
    std::cout << "  'left joystick up/down' - Forward/Backward\n";
    std::cout << "  'left joystick left/right' - Left/Right\n";
    std::cout << "  'right joystick left/right' - Yaw Left/Right\n\n";
}

int main(int argc, char* argv[])
{
    std::cout << "Starting " << APP_NAME << "...\n";
    printHelp();
    std::string path = "";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-p" || arg == "--path") {
            if (i + 1 < argc) {
                path = argv[i + 1];
                ++i;
            } else {
                std::cerr << "Error: Missing value for " << arg << " argument.\n";
                return -1;
            }
        } else if (arg == "-h" || arg == "--help") {
            printHelp();
            return 0;
        }
    }
    if (path.empty()) {
        std::cerr << "Error: Policy path is required. Use -p or --path to specify it.\n";
        return -1;
    } else {
        g_policyPath = path;
        std::cout << "Policy path set to: " << g_policyPath << "\n";
    }

    signal(SIGTERM, signalHandler);
    signal(SIGINT,  signalHandler);
    signal(SIGHUP,  signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGKILL, signalHandler);
    signal(SIGSEGV, signalHandler);

    if (getuid() == 0) {
        std::cout << "Control rt thread starting...\n";
        pthread_t controlThread;
        if (!rbq_sdk::Thread::generate_rt_thread(controlThread, controlLoop, "ControlLoop", 3, 90, nullptr)) {
            std::cerr << "Failed to create control thread\n";
            throw(std::exception());
        }
    } else {
        std::cout << "Control thread starting...\n";
        static std::thread controlThread = std::thread(controlLoop, nullptr);
    }

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    while (g_isWorking) {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            switch(key) {
            case 'q':
                g_isWorking = false;
                break;
            case 'z':
                std::cout << "Executing Motion Ground...\n";
                goToMotionGround();
                printHelp();
                break;
            case 'x':
                std::cout << "Executing Motion Ready...\n";
                goToMotionReady();
                printHelp();
                break;
            case 'c':
                std::cout << "Executing Motion Walk...\n";
                g_currentTask = TaskState::Control;
                printHelp();
                break;
            case 'v':
                // TODO: add vision enabled walk.
                printHelp();
                break;
            default:
                printHelp();
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Shutting down...\n";
    return 0;
}

void* controlLoop(void*)
{
    std::cout << "Control loop started.\n";
    usleep(100 * 1000);

    timespec timeEnd;
    timespec timeNext;
    clock_gettime(CLOCK_REALTIME, &timeNext);

    bool reset = true;

    sensor_msgs::msg::dds_::Joy_ msgJoy;
    rbq_sdk::Subscriber<sensor_msgs::msg::dds_::Joy_> subJoy(&msgJoy, "rt/rbq/joy");

    sensor_msgs::msg::dds_::Imu_ msgImuInfo;
    rbq_sdk::Subscriber<sensor_msgs::msg::dds_::Imu_> subImuInfo(&msgImuInfo, "rt/rbq/imu");

    rbq_msgs::msg::dds_::LegJointInfo_ msgLegJointInfo;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::LegJointInfo_> subLegJointInfo(&msgLegJointInfo, "rt/rbq/leg_joint");

    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::MotionRef_> subLegRefFinal(&msgMotionRefFinal, "rt/rbq/ref/motion/_0");

    rbq_sdk::Publisher<rbq_msgs::msg::dds_::MotionRef_> pubMotionRef("rt/rbq/ref/motion/_20");

    rbq_sdk::Publisher<rbq_msgs::msg::dds_::JointOwnershipCmd_> pubJointOwnershipCmd("rt/rbq/cmd/motion/owner/_20");

    while (g_isWorking) {
        static TaskState lastTask = TaskState::Idle;
        if (lastTask != g_currentTask) {
            lastTask = g_currentTask;
            reset = true;
        }
        switch (g_currentTask) {
        case TaskState::Idle: {
            for (int idx=0; idx<12; idx++) {
                g_jointControl.setPosition(idx, msgMotionRefFinal.leg_joint().at(idx).pos());
            }
            rbq_msgs::msg::dds_::MotionRef_ msgMotionRef;
            for (int idx=0; idx<12; idx++) {
                msgMotionRef.leg_joint()[idx].pos()   = g_jointControl.position(idx);
                msgMotionRef.leg_joint()[idx].kp()    = 200.f;
                msgMotionRef.leg_joint()[idx].kd()    = 2.5f;
                msgMotionRef.leg_joint()[idx].torque()= 0.f;
            }
            pubMotionRef.write(msgMotionRef);
            break;
        }
        case TaskState::Motion: {
            g_jointControl.update();
            rbq_msgs::msg::dds_::MotionRef_ msgMotionRef;
            for (int idx=0; idx<12; idx++) {
                msgMotionRef.leg_joint()[idx].pos()   = g_jointControl.position(idx);
                msgMotionRef.leg_joint()[idx].kp()    = 200.f;
                msgMotionRef.leg_joint()[idx].kd()    = 2.5f;
                msgMotionRef.leg_joint()[idx].torque()= 0.f;
            }
            pubMotionRef.write(msgMotionRef);
            if (reset) {
                reset = false;
                rbq_msgs::msg::dds_::JointOwnershipCmd_ msgJointOwnershipCmd;
                for (int idx=0; idx<12; idx++) {
                    msgJointOwnershipCmd.leg_joint()[idx] = true;
                }
                pubJointOwnershipCmd.write(msgJointOwnershipCmd);
            }
            break;
        }
        case TaskState::Control: {
            bool policyError = false;
            static rbq_sdk::Timer timer;
            static rbq_sdk::PolicyParams params;
            static rbq_sdk::Policy policy;
            static int decimation_cnt = 0;
            if (decimation_cnt == 5) {
                decimation_cnt = 0;
                try {
                    if (!policy.loaded() || !params.loaded() || reset ) {
                        std::cout << "Loading policy from: " << g_policyPath;
                        params.loadFromPath(g_policyPath);
                        policy.release();
                        policy.reload(g_policyPath);
                    }
                    if (policy.error() == rbq_sdk::Policy::ERROR_NONE && policy.loaded() &&
                            params.error() == rbq_sdk::PolicyParams::ERROR_NONE && params.loaded()) {
                        const int jointSize = 12;
                        Eigen::Vector3f gyro;
                        gyro.x() = msgImuInfo.angular_velocity().x();
                        gyro.y() = msgImuInfo.angular_velocity().y();
                        gyro.z() = msgImuInfo.angular_velocity().z();
                        Eigen::Quaternion<float> quat;
                        quat.w() = msgImuInfo.orientation().w();
                        quat.x() = msgImuInfo.orientation().x();
                        quat.y() = msgImuInfo.orientation().y();
                        quat.z() = msgImuInfo.orientation().z();
                        Eigen::VectorXf pos = Eigen::VectorXf::Zero(jointSize);
                        Eigen::VectorXf vel = Eigen::VectorXf::Zero(jointSize);
                        for (int i=0; i<jointSize; i++) {
                            pos[i] = msgLegJointInfo.joint().at(i).pos();
                            vel[i] = msgLegJointInfo.joint().at(i).vel();
                        }
                        static Eigen::Vector3f command = Eigen::Vector3f::Zero();
                        {
                            int error;
                            command(0) = rbq_sdk::Filter::LPF(msgJoy.axes().at(1) *  1, command(0), 1.0, 0.1);
                            command(1) = rbq_sdk::Filter::LPF(msgJoy.axes().at(0) * -1, command(1), 1.0, 0.1);
                            command(2) = rbq_sdk::Filter::LPF(msgJoy.axes().at(2) * -1, command(2), 1.0, 0.1);
                            if (error) {
                                std::cout << "Gamepad read error: " << error;
                            }
                        }
                        static std::vector<float> action_1(jointSize, 0.0f);
                        if (reset) {
                            action_1.assign(jointSize, 0.0f);
                            command.setZero();
                        }
                        std::vector<float> obs;
                        if (params.name == "rbq10") {
                            for (int i = 0; i < 3; i++)
                                obs.push_back(gyro[i] * params.obs_ang_vel_scale);
                            Eigen::Vector3f proj_grav = quat.inverse() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
                            for (int i = 0; i < 3; i++)
                                obs.push_back(proj_grav[i]);
                            static float commands_scale[] = {params.obs_lin_vel_scale, params.obs_lin_vel_scale, params.obs_ang_vel_scale};
                            for (int i = 0; i < 3; i++)
                                obs.push_back(command[i] * commands_scale[i]);
                            for (int i = 0; i < jointSize; i++)
                                obs.push_back((pos[i] - params.default_joint_angles[i]) * params.obs_dof_pos_scale);
                            for (int i = 0; i < jointSize; i++)
                                obs.push_back(vel[i] * params.obs_dof_vel_scale);
                            for (int i = 0; i < jointSize; i++)
                                obs.push_back(action_1[i]);
                        } else if (params.name == "rbq10_trot" || params.name == "rbq10_trot_run") {
                            for (int i = 0; i < 3; i++)
                                obs.push_back(gyro[i] * params.obs_ang_vel_scale);
                            Eigen::Vector3f proj_grav = quat.inverse() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
                            for (int i = 0; i < 3; i++)
                                obs.push_back(proj_grav[i]);
                            static float commands_scale[] = {params.obs_lin_vel_scale, params.obs_lin_vel_scale, params.obs_ang_vel_scale};
                            for (int i = 0; i < 3; i++)
                                obs.push_back(command[i] * commands_scale[i]);
                            static std::vector<float> action_2 (jointSize, 0.0f);
                            static std::vector<float> dof_pos_0(jointSize, 0.0f);
                            static std::vector<float> dof_pos_1(jointSize, 0.0f);
                            static std::vector<float> dof_pos_2(jointSize, 0.0f);
                            static std::vector<float> dof_pos_3(jointSize, 0.0f);
                            static std::vector<float> dof_pos_4(jointSize, 0.0f);
                            static std::vector<float> dof_pos_5(jointSize, 0.0f);
                            static std::vector<float> dof_pos_6(jointSize, 0.0f);
                            static std::vector<float> dof_vel_0(jointSize, 0.0f);
                            static std::vector<float> dof_vel_1(jointSize, 0.0f);
                            static std::vector<float> dof_vel_2(jointSize, 0.0f);
                            static std::vector<float> dof_vel_3(jointSize, 0.0f);
                            static std::vector<float> dof_vel_4(jointSize, 0.0f);
                            static std::vector<float> dof_vel_5(jointSize, 0.0f);
                            static std::vector<float> dof_vel_6(jointSize, 0.0f);
                            if (reset) {
                                for (int i=0; i<jointSize; i++) {
                                    dof_vel_6[i] = 0;
                                    dof_vel_5[i] = 0;
                                    dof_vel_4[i] = 0;
                                    dof_vel_3[i] = 0;
                                    dof_vel_2[i] = 0;
                                    dof_vel_1[i] = 0;
                                    dof_vel_0[i] = vel[i];
                                    dof_pos_6[i] = pos[i];
                                    dof_pos_5[i] = pos[i];
                                    dof_pos_4[i] = pos[i];
                                    dof_pos_3[i] = pos[i];
                                    dof_pos_2[i] = pos[i];
                                    dof_pos_1[i] = pos[i];
                                    dof_pos_0[i] = pos[i];
                                    action_2 [i] = 0;
                                    action_1 [i] = 0;
                                }
                            } else {
                                for (int i=0; i<jointSize; i++) {
                                    dof_vel_6[i] = dof_vel_5[i];
                                    dof_vel_5[i] = dof_vel_4[i];
                                    dof_vel_4[i] = dof_vel_3[i];
                                    dof_vel_3[i] = dof_vel_2[i];
                                    dof_vel_2[i] = dof_vel_1[i];
                                    dof_vel_1[i] = dof_vel_0[i];
                                    dof_vel_0[i] = vel[i];
                                    dof_pos_6[i] = dof_pos_5[i];
                                    dof_pos_5[i] = dof_pos_4[i];
                                    dof_pos_4[i] = dof_pos_3[i];
                                    dof_pos_3[i] = dof_pos_2[i];
                                    dof_pos_2[i] = dof_pos_1[i];
                                    dof_pos_1[i] = dof_pos_0[i];
                                    dof_pos_0[i] = pos[i];
                                    action_2[i]  = action_1[i];
                                }
                            }
                            for (int i = 0; i < jointSize; i++) obs.push_back((dof_pos_0[i] - params.default_joint_angles[i]) * params.obs_dof_pos_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back((dof_pos_2[i] - params.default_joint_angles[i]) * params.obs_dof_pos_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back((dof_pos_4[i] - params.default_joint_angles[i]) * params.obs_dof_pos_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back((dof_pos_6[i] - params.default_joint_angles[i]) * params.obs_dof_pos_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back( dof_vel_0[i] * params.obs_dof_vel_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back( dof_vel_2[i] * params.obs_dof_vel_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back( dof_vel_4[i] * params.obs_dof_vel_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back( dof_vel_6[i] * params.obs_dof_vel_scale);
                            for (int i = 0; i < jointSize; i++) obs.push_back( action_1[i]);
                            for (int i = 0; i < jointSize; i++) obs.push_back( action_2[i]);
                            obs.push_back(0); // added mass
                        }
                        int error;
                        std::vector<float> actions = policy.inference(obs, params.num_observations, params.num_actions, error);
                        for (int i=0; i<jointSize; i++) {
                            actions[i] = std::clamp(actions[i], -params.clip_actions, params.clip_actions);
                            action_1[i] = actions[i];
                        }
                        if (error == rbq_sdk::Policy::ERROR_NONE) {
                            rbq_msgs::msg::dds_::MotionRef_ msgMotionRef;
                            for (int i=0; i<jointSize; i++) {
                                const float posRef = actions[i] * params.action_scale + params.default_joint_angles[i];
                                msgMotionRef.leg_joint()[i].pos()   = posRef;
                                msgMotionRef.leg_joint()[i].kp()    = params.KP[i];
                                msgMotionRef.leg_joint()[i].kd()    = params.KD[i];
                                msgMotionRef.leg_joint()[i].torque()= 0;
                            }
                            pubMotionRef.write(msgMotionRef);
                            if (reset) {
                                rbq_msgs::msg::dds_::JointOwnershipCmd_ msgJointOwnershipCmd;
                                for (int idx=0; idx<12; idx++) {
                                    msgJointOwnershipCmd.leg_joint()[idx] = true;
                                }
                                pubJointOwnershipCmd.write(msgJointOwnershipCmd);
                            }
                        } else {
                            std::cerr << "Policy error occurred: " << static_cast<int>(policy.error()) << std::endl;
                        }
                    } else {
                        std::cerr << "Policy, PolicyParams not loaded or has error.\n";
                        policyError = true;
                    }
                    reset = false;
                } catch (const std::exception& e) {
                    std::cerr << "Policy execution error: " << e.what() << std::endl;
                    policyError = true;
                } catch (...) {
                    std::cerr << "Unknown policy execution error" << std::endl;
                    policyError = true;
                }
            }
            if (policyError) {
                std::cerr << "Exiting control task due to policy error.\n";
                g_currentTask = TaskState::Idle;
                break;
            }
            decimation_cnt++;
            break;
        }
        default:
            break;
        }

        clock_gettime(CLOCK_REALTIME, &timeEnd);
        rbq_sdk::Thread::timespec_add_us(&timeNext, 2000);
        if (rbq_sdk::Thread::timespec_cmp(&timeEnd, &timeNext) > 0) {
            std::cout << "Thread deadline ms: " << rbq_sdk::Thread::timediff_us(&timeNext, &timeEnd)*0.001 << " ms over!";
            clock_gettime(CLOCK_REALTIME, &timeNext);
        } else {
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &timeNext, NULL);
        }
    }
    std::cout << "Control loop exiting.\n";
    return nullptr;
}

void goToMotionReady()
{
    g_currentTask = TaskState::Motion;
    usleep(500 * 1000);
    float motionTime = 1400.0f;
    float pitchAngles[4];
    pitchAngles[0] = msgMotionRefFinal.leg_joint().at(1).pos();
    pitchAngles[1] = msgMotionRefFinal.leg_joint().at(4).pos();
    pitchAngles[2] = msgMotionRefFinal.leg_joint().at(7).pos();
    pitchAngles[3] = msgMotionRefFinal.leg_joint().at(10).pos();
    bool isGrounded = (pitchAngles[0] > 60 * kD2R && pitchAngles[1] > 60 * kD2R &&
            pitchAngles[2] > 60 * kD2R && pitchAngles[3] > 60 * kD2R);
    const auto& jointTable = rbq_sdk::JointTable();
    const auto& jointCmd = isGrounded ? jointTable.folding : jointTable.ready;
    for (int idx=0; idx<12; idx++) {
        g_jointControl.setMove(idx, jointCmd[idx] * kD2R, motionTime, rbq_sdk::JointControl::MoveCommandMode::Absolute);
    }
    usleep((motionTime + 100) * 1000);
    if (isGrounded) {
        for (int idx=0; idx<12; idx++) {
            g_jointControl.setMove(idx, jointTable.ready[idx] * kD2R, motionTime, rbq_sdk::JointControl::MoveCommandMode::Absolute);
        }
        usleep((motionTime + 100) * 1000);
    }
    g_currentTask = TaskState::Idle;
}

void goToMotionGround()
{
    g_currentTask = TaskState::Motion;
    usleep(500 * 1000);
    float motionTime = 2400.0f;
    const auto& jointTable = rbq_sdk::JointTable();
    for (int idx=0; idx<12; idx++) {
        g_jointControl.setMove(idx, jointTable.folding[idx] * kD2R, motionTime, rbq_sdk::JointControl::MoveCommandMode::Absolute);
    }
    usleep((motionTime + 100) * 1000);
    for (int idx=0; idx<12; idx++) {
        g_jointControl.setMove(idx, jointTable.ground[idx] * kD2R, motionTime, rbq_sdk::JointControl::MoveCommandMode::Absolute);
    }
    usleep((motionTime + 100) * 1000);
    g_currentTask = TaskState::Idle;
}
