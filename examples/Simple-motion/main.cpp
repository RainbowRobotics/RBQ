#include <iostream>
#include <signal.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "rbq/Api.h"
#include "rbq/Thread.h"
#include "rbq/Parameters.h"

#include "JointControl.h"

#define R2Df         57.295779513f
#define D2Rf         0.0174532925f

using namespace std;
JOINT_TABLE  JointTable;

int                     JointJog_No;
int                     JogMoveDir;
int                     __IS_WORKING;

RBQ_API                 *rbqApi;
JointControl            *JCON;

void GoMotionReady()
{
    JCON->RefreshToCurrentReference();
    JCON->SetAllMotionOwner();

    for(int i=0; i<12; i++){
        rbqApi->Joint.setGainKp(i, 200.f);
        rbqApi->Joint.setGainKd(i, 2.5f);
    }
    usleep(500*1000);

    float motion_time_ms = 1400;

    bool robot_on_ground;
    float current_pitch_angle[4];


    rbqApi->Joint.getPosRef(RBQ_API::JOINT::JointID::HRP, current_pitch_angle[0]);
    rbqApi->Joint.getPosRef(RBQ_API::JOINT::JointID::HLP, current_pitch_angle[1]);
    rbqApi->Joint.getPosRef(RBQ_API::JOINT::JointID::FRP, current_pitch_angle[2]);
    rbqApi->Joint.getPosRef(RBQ_API::JOINT::JointID::FLP, current_pitch_angle[3]);

    if(current_pitch_angle[0] > 60*D2Rf && current_pitch_angle[1] > 60*D2Rf &&
        current_pitch_angle[2] > 60*D2Rf && current_pitch_angle[3] > 60*D2Rf){
        robot_on_ground = true;
    }
    else{
        robot_on_ground = false;
    }

    if(robot_on_ground){
        for(int i=0; i<MAX_JOINT; i++) {
            JCON->SetMoveJoint(i, JointTable.JointFolding[i]*D2Rf, motion_time_ms, MOVE_ABSOLUTE);
        }
        usleep((motion_time_ms + 100)*1000);
        for(int i=0; i<MAX_JOINT; i++) {
            JCON->SetMoveJoint(i, JointTable.JointReady[i]*D2Rf, motion_time_ms, MOVE_ABSOLUTE);
        }
        usleep((motion_time_ms + 100)*1000);
    }
    else{
        for(int i=0; i<MAX_JOINT; i++) {
            JCON->SetMoveJoint(i, JointTable.JointReady[i]*D2Rf, motion_time_ms, MOVE_ABSOLUTE);
        }
        usleep((motion_time_ms + 100)*1000);
    }
}

void GoMotionGround()
{
    JCON->RefreshToCurrentReference();
    JCON->SetAllMotionOwner();

    for(int i=0; i<12; i++){
        rbqApi->Joint.setGainKp(i, 200.f);
        rbqApi->Joint.setGainKd(i, 2.5f);
    }

    float motion_time_ms = 2400;

    for(int i=0; i<MAX_JOINT; i++) {
        JCON->SetMoveJoint(i, JointTable.JointFolding[i]*D2Rf, motion_time_ms, 0);
    }
    usleep((motion_time_ms + 100)*1000);
    for(int i=0; i<MAX_JOINT; i++) {
        JCON->SetMoveJoint(i, JointTable.JointGround[i]*D2Rf, motion_time_ms, 0);
    }
    usleep((motion_time_ms + 100)*1000);
}

enum TASK_e {
    TASK_IDLE = 0,
    TASK_MOTION,
    TASK_JOG_MOVE,
    TASK_POS_LOCK,
    TASK_CONTROL,
};
TASK_e CURRENT_TASK;

enum _EXAMPLE_COMMAND_SET_{
    EXAMPLE_NO_ACT = 0,
    EXAMPLE_COMMAND_TEST,
    EXAMPLE_COMMAND_MOTION_READY,
    EXAMPLE_COMMAND_MOTION_GROUND,
    EXAMPLE_JOINT_LOCK_UNLOCK,
};

// Real-time thread for control
pthread_t pthread_RBTask;
pthread_t thread_motion1;
pthread_t thread_motion2;

static void *RBTaskThread(void *arg);

void CatchSignals(int _signal)
{
    switch(_signal)
    {
    case SIGHUP:
    case SIGINT:
    case SIGTERM:
    case SIGKILL:
    case SIGSEGV:
        __IS_WORKING = false;
        std::_Exit(_signal);
        // std::string str = "sudo pkill -9 -f ";
        // str.append(APP_NAME);
        // int i = system(str.data());
        break;
    };
}

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);
    signal(SIGINT, CatchSignals);
    signal(SIGHUP, CatchSignals);
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    __IS_WORKING = true;
    std::cout << APP_NAME << "initializing..";

    usleep(100*1000);

    // Create and start real-time thread =======================================
    bool threadID_task = Thread::generate_rt_thread(pthread_RBTask, RBTaskThread, "task_thread", 1, 90, NULL);
    if(threadID_task == false){
        std::cerr << "fail to create task_thread";
        return 1;
    }
    std::cout << "task thread start = OK";

    Parameters params;
    if(!params.Initialize()) return 1;

    __IS_WORKING = true;

    rbqApi = new RBQ_API(20);

    // Initialize internal joint classes =======================================
    JCON = new JointControl(rbqApi, MAX_JOINT);
    JCON->RefreshToCurrentReference();
    // =========================================================================

    ///----------------------------- nrt Command Thread -------------------------------------
    while(__IS_WORKING){
        int user_command = EXAMPLE_NO_ACT;
        switch(user_command){
        case EXAMPLE_COMMAND_TEST:{
            std::cout << "EXAMPLE_COMMAND_TEST Received";
            int para_int_rcv;
            // rbqApi->command.getUserParaInt(0, para_int_rcv);
            float para_float_rcv;
            // rbqApi->command.getUserParaFloat(0, para_float_rcv);
            cout<<"Recieved int parameter: "<<para_int_rcv<<endl;
            cout<<"Recieved float parameter: "<<para_float_rcv<<endl;
            break;
        }
        case EXAMPLE_COMMAND_MOTION_READY:
            std::cout << "EXAMPLE_COMMAND_MOTION_READY Received";
            CURRENT_TASK = TASK_MOTION;
            GoMotionReady();
            CURRENT_TASK = TASK_IDLE;
            break;
        case EXAMPLE_COMMAND_MOTION_GROUND:
            std::cout << "EXAMPLE_COMMAND_MOTION_READY Received";
            CURRENT_TASK = TASK_MOTION;
            GoMotionGround();
            CURRENT_TASK = TASK_IDLE;
            break;
        case EXAMPLE_JOINT_LOCK_UNLOCK:{
            std::cout << "EXAMPLE_JOINT_LOCK_UNLOCK Received";
            int joint_id;
            // rbqApi->command.getUserParaInt(0, joint_id);
            int on_off;
            // rbqApi->command.getUserParaInt(1, on_off); // 0: unlock  1:lock

            if(on_off == 0){ //unlock joint[id]
                float real_kp;
                rbqApi->Joint.setGainKp(joint_id, 0.0f, real_kp);
                float real_kd;
                rbqApi->Joint.setGainKd(joint_id, 0.8f, real_kd);
                rbqApi->Joint.setMotionOwner(joint_id);
            }
            else{ //lock joint[id]
                JCON->RefreshToCurrentPosition_id(joint_id);

                if(joint_id%3 == 0){ //roll joint
                    float real_kp;
                    rbqApi->Joint.setGainKp(joint_id, 70.0f, real_kp);
                    float real_kd;
                    rbqApi->Joint.setGainKd(joint_id, 0.8f, real_kd);
                }
                else if(joint_id%3 == 1){ //pitch joint
                    float real_kp;
                    rbqApi->Joint.setGainKp(joint_id, 70.0f, real_kp);
                    float real_kd;
                    rbqApi->Joint.setGainKd(joint_id, 0.8f, real_kd);
                }
                else if(joint_id%3 == 2){
                    float real_kp;
                    rbqApi->Joint.setGainKp(joint_id, 70.0f, real_kp);
                    float real_kd;
                    rbqApi->Joint.setGainKd(joint_id, 0.8f, real_kd);
                }

                JCON->SetMotionOwner(joint_id);
            }
            break;
        }
        default:
            break;
        }
    }
    std::cerr << APP_NAME <<" is terminated" << std::endl;
    return 0;
}

void *RBTaskThread(void *arg)
{
    const long PERIOD_US = RT_MS * 1000;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    struct timespec TIME_TIC;

    usleep(100*1000);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    while(true){
        clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        ///---------------------------------------------------------------------------------
        ///-------------------RT loop-------------------------------------------------------
        ///---------------------------------------------------------------------------------

        /// basic task for every contol loop

        /// special task for every control loop
        switch(CURRENT_TASK){
        case TASK_IDLE:
        {
            break;
        }
        case TASK_MOTION:

            JCON->MoveAllJoint();
            JCON->JointUpdate();

            break;
        case TASK_JOG_MOVE:
        {
            // float new_angle = JCON->GetJointRefAngle(JointJog_No) + JogMoveDir*10.0*D2Rf/500.0f;
            // if(new_angle >= Parameters::joint_upper_lim.at(JointJog_No)) new_angle = Parameters::joint_upper_lim.at(JointJog_No);
            // if(new_angle <= Parameters::joint_lower_lim.at(JointJog_No)) new_angle = Parameters::joint_lower_lim.at(JointJog_No);
            // JCON->SetJointRefAngle(JointJog_No, new_angle);
            break;
        }
        case TASK_CONTROL: {
            ///--------------------get sensor -----------------
            //-------------joint--------------
            float current_joint_pos[12];
            float current_joint_vel[12];
            float current_joint_torque[12];
            float current_joint_kp[12];
            float current_joint_kd[12];

            for(int i=0; i<12; i++){
                rbqApi->Joint.getPos(i, current_joint_pos[i]);
                rbqApi->Joint.getVel(i, current_joint_vel[i]);
                rbqApi->Joint.getTorque(i, current_joint_torque[i]);
                rbqApi->Joint.getGainKp(i, current_joint_kp[i]);
                rbqApi->Joint.getGainKd(i, current_joint_kd[i]);
            }

            //-------------imu--------------
            Eigen::Quaternion<float> quat;
            Eigen::Vector<float, 3> gyro;
            Eigen::Vector<float, 3> acc;
            Eigen::Vector<float, 3> rpy;

            rbqApi->Imu.getQuaternion(quat);
            rbqApi->Imu.getGyro(gyro);
            rbqApi->Imu.getAcc(acc);
            rbqApi->Imu.getRPY(rpy);

            //-------------gamepad----------
            //Joy stick data (Logitech F710 in 'X' mode)
            float left_jog_x;
            float left_jog_y;
            float left_trigger;
            float right_jog_x;
            float right_jog_y;
            float right_trigger;

            bool BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB;
            bool BTN_BACK, BTN_START, BTN_LOGI, BTN_LJOG, BTN_RJOG;
            bool ARROW_UP, ARROW_DW, ARROW_L, ARROW_R;

            rbqApi->gamepad.getLeftJogX(left_jog_x);
            rbqApi->gamepad.getLeftJogY(left_jog_y);
            rbqApi->gamepad.getLeftTrigger(left_trigger);
            rbqApi->gamepad.getRightJogX(right_jog_x);
            rbqApi->gamepad.getRightJogY(right_jog_y);
            rbqApi->gamepad.getRightTrigger(right_trigger);

            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::A, BTN_A);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::B, BTN_B);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::X, BTN_X);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::Y, BTN_Y);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::LB, BTN_LB);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::RB, BTN_RB);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::BACK, BTN_BACK);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::START, BTN_START);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::LOGITECH, BTN_LOGI);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::LJOG, BTN_LJOG);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::RJOG, BTN_RJOG);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_UP, ARROW_UP);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_DOWN, ARROW_DW);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_LEFT, ARROW_L);
            rbqApi->gamepad.getButtonState(RBQ_API::Gamepad::Button::AR_RIGHT, ARROW_R);


            ///------------------------------------------------------------------



            ///------------------- write your algorithm here---------------------

            ///------------------------------------------------------------------



            ///-----------------set Reference-----------------------

            //User made joint position reference
            float joint_pos_ref[12];

            //User made joint torque reference
            float joint_torque_ref[12];

            //User made joint gain kp reference
            float joint_kp_ref[12];
            float joint_kp_real[12];

            //User made joint gain kd reference
            float joint_kd_ref[12];
            float joint_kd_real[12];

            for(int i=0; i<12; i++){
                rbqApi->Joint.setPosRef(i, joint_pos_ref[i]);
                rbqApi->Joint.setTorqueRef(i, joint_torque_ref[i]);
                rbqApi->Joint.setGainKp(i, joint_kp_ref[i], joint_kp_real[i]);
                rbqApi->Joint.setGainKd(i, joint_kd_ref[i], joint_kd_real[i]);
            }
            ///------------------------------------------------------------------


            break;
        }
        default:
            break;
        }

        /// ----------------------------------------------------------------------------------
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        Thread::timespec_add_us(&TIME_NEXT, PERIOD_US);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if(Thread::timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
        }
    }
}
