#include <unistd.h>
#include <QProcess>
#include <QDebug>

static inline int command_terminal_ping_check(const std::string &ip)
{
    int ret=0;
    QProcess *proc;
    proc = new QProcess();
    proc->start("ping", QStringList() << "-c 2" << ip.data());
    proc->waitForFinished(-1);
    QByteArray output = proc->readAllStandardOutput();
    if (!output.isEmpty()){
        if (-1 != QString(output).indexOf("ttl", 0, Qt::CaseInsensitive)){
            qDebug() << ip.data() << ", PING OK";
            ret = 1;
        }
        else{
            qDebug() << ip.data() << ", PING Fail";
        }
    }
    delete proc;
    return ret;
}

#include <QCoreApplication>
#include <QDebug>

// signal to exit infinite loops
extern bool GLOBAL_KILL_SIGNAL;

#include "RobotApiHandler.h"
#include "NetworkHandler.h"

#include <thread>

NetworkHandler *network = nullptr;
std::thread *th = nullptr;

using namespace RBQ_SDK;

RobotApiHandler::RobotApiHandler(const std::string &host, const int &_commFrequency)
{
    th = new std::thread(&RobotApiHandler::qAppThread, this, host, _commFrequency);
}

RobotApiHandler::~RobotApiHandler()
{
    QCoreApplication::quit();
}

void RobotApiHandler::qAppThread(const std::string &host, const int &_commFrequency)
{
    int argc=0; char* argv[] = { (char*)"RBQ-client" };
    QCoreApplication a(argc, argv);
    network = new NetworkHandler(&a, host, _commFrequency);
    a.exec();
    network->deleteLater();
    return;
}

Motion::RobotState_t *RobotApiHandler::robotState()
{
    if (network != nullptr) {
        return network->robotState();
    } else {
        return nullptr;
    }
}

Motion::LegStateArray_t *RobotApiHandler::legStateArray()
{
    if (network != nullptr) {
        return network->legStateArray();
    } else {
        return nullptr;
    }
}

void RobotApiHandler::setHighLevelCommand(const RBQ_SDK::HighLevelCmd_t &_cmd)
{
    qDebug() << "RobotApiHandler::setHighLevelCommand()";
    if (network != nullptr) {
        QByteArray sendData = QByteArray::fromRawData((const char *) (&_cmd),
                                                      sizeof(HighLevelCmd_t));
        network->sendUDP(sendData);
    }
}

void RobotApiHandler::setUserCommand(const USER_COMMAND &usrCmd)
{
    qDebug() << "RobotApiHandler::setUserCommand()";
    if (network != nullptr) {
        QByteArray sendData = QByteArray::fromRawData((const char *) (&usrCmd),
                                                      sizeof(USER_COMMAND));
        network->sendTCP(sendData);
    } else {
        qDebug() << "RobotApiHandler::setUserCommand() failed network is down";
    }
}

void RobotApiHandler::startMotionProgram(const bool &_start, const int &_id)
{
    qDebug() << "RobotApiHandler::startMotionProgram() start:" << _start << "\tid" << _id;
    // APP_NAME_00 = PODOLAN
    // APP_NAME_01 = Daemon
    // APP_NAME_02 = WalkReady
    // APP_NAME_03 = QuadWalk
    // APP_NAME_04 = Manipulator
    USER_COMMAND cmd;

    if(_start) {
        cmd.USER_COMMAND = 1; // start
    } else {
        cmd.USER_COMMAND = 2; // stop
    }
    cmd.USER_PARA_INT[0] = _id;
    cmd.COMMAND_TARGET = Daemon;

    setUserCommand(cmd);
}

void RobotApiHandler::autoStart()
{
    qDebug() << "RobotApiHandler::autoStart()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = Daemon;
    cmd.USER_COMMAND = DAEMON_INIT_AUTOSTART;
    int mode = 1; // Ground pose : 1,
    cmd.USER_PARA_CHAR[1] = mode;
    setUserCommand(cmd);
}

void RobotApiHandler::canCheck()
{
    qDebug() << "RobotApiHandler::canCheck()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = Daemon;
    cmd.USER_COMMAND = DAEMON_INIT_CHECK_DEVICE;
    cmd.USER_PARA_CHAR[0] = 12; // ALL
    setUserCommand(cmd);
}

void RobotApiHandler::findHome()
{
    qDebug() << "RobotApiHandler::findHome()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = Daemon;
    cmd.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.USER_PARA_CHAR[0] = -1; // ALL
    int mode = 1; // Ground pose : 1,
    cmd.USER_PARA_CHAR[1] = mode;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticLock()
{
    qDebug() << "RobotApiHandler::motionStaticLock()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = WalkReady;
    cmd.USER_COMMAND = WalkReady_CURRENT_POS_LOCK;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticReady()
{
    qDebug() << "RobotApiHandler::motionStaticReady()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = WalkReady;
    cmd.USER_COMMAND = WalkReady_MOTION_READY;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticGround()
{
    qDebug() << "RobotApiHandler::motionStaticGround()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = WalkReady;
    cmd.USER_COMMAND = WalkReady_MOTION_GROUND;
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicGround()
{
    qDebug() << "RobotApiHandler::motionDynamicGround()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_STAND_DOWN;
    setUserCommand(cmd);
    setWalking(false);
}

void RobotApiHandler::motionDynamicReady()
{
    qDebug() << "RobotApiHandler::motionDynamicReady()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_TO_STANCE_MODE;
    setUserCommand(cmd);
    setWalking(false);
}

void RobotApiHandler::motionDynamicWalk()
{
    qDebug() << "RobotApiHandler::motionDynamicWalk()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_TO_WALK_MODE;
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionDynamicWalkSlow()
{
    qDebug() << "RobotApiHandler::motionDynamicWalkSlow()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = TO_WAVE; // Wave
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionDynamicWalkStairs()
{
    qDebug() << "RobotApiHandler::motionDynamicWalkSlow()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = TO_TROT_S; // Stairs
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionDynamicRun()
{
    qDebug() << "RobotApiHandler::motionsDynamicRun()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = TO_RUN; // Run
    setUserCommand(cmd);
    setWalking(true);
}

void RobotApiHandler::motionDynamicAim()
{
    qDebug() << "RobotApiHandler::motionDynamicAim()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_TO_AIMING_MODE;
    setUserCommand(cmd);
    setWalking(false);
}

void RobotApiHandler::motionParametersUpdate()
{
    qDebug() << "RobotApiHandler::motionParametersUpdate()";
    qDebug() << " -- max speed: " << m_maxSpeed
             << " -- body height: " << m_bodyHeight
             << " -- foot height: " << m_footHeight;
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_WALK_PARAMETER_UPDATE;
    cmd.USER_PARA_FLOAT[0]= m_bodyHeight;
    cmd.USER_PARA_FLOAT[1]= m_maxSpeed;
    cmd.USER_PARA_FLOAT[2]= m_footHeight;
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicNavigateTo(const float &_targetPosX, const float &_targetPosY, const float &_targetRotZ, const int &_mode)
{
    qDebug() << "RobotApiHandler::motionDynamicNavigateTo()";
    qDebug() << " -- max speed: " << m_maxSpeed
             << " -- body height: " << m_bodyHeight
             << " -- foot height: " << m_footHeight;
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET  = QuadWalk;
    cmd.USER_COMMAND    = QuadWalk_WAVE_TO_TARGET_POINT;
    cmd.USER_PARA_FLOAT[0]  = _targetPosX;  // target position X (m)
    cmd.USER_PARA_FLOAT[1]  = _targetPosY;  // target position Y (m)
    cmd.USER_PARA_FLOAT[2]  = _targetRotZ;  // target rotation Z (rad)
    cmd.USER_PARA_FLOAT[3]  = 0;  // offset wrt target position X (m)
    cmd.USER_PARA_FLOAT[4]  = 0;  // offset wrt target position Y (m)

    cmd.USER_PARA_INT[0]    = _mode;  // mode

    setUserCommand(cmd);
}

void RobotApiHandler::recovery1()
{
    qDebug() << "RobotApiHandler::recovery1()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = WalkReady;
    cmd.USER_COMMAND = WalkReady_GO_RECOVERY_READY;
    setUserCommand(cmd);
}

void RobotApiHandler::recovery2()
{
    qDebug() << "RobotApiHandler::recovery2()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = WalkReady;
    cmd.USER_COMMAND = WalkReady_FALL_RECOVERY_MOTION;
    setUserCommand(cmd);
}

void RobotApiHandler::calibrateAccelerometer()
{
    qDebug() << "RobotApiHandler::calibrateAccelerometer()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = Daemon;
    cmd.USER_COMMAND = DAEMON_ACC_CALIBRATION;
    setUserCommand(cmd);
}

void RobotApiHandler::comEstimationCompensation(const int &stage)
{
    qDebug() << "RobotApiHandler::comEstimationCompensation()";
    if(-1 < stage && stage < 3) {
        USER_COMMAND cmd;
        cmd.COMMAND_TARGET = QuadWalk;
        cmd.USER_COMMAND = QuadWalk_SYS_ID;
        cmd.USER_PARA_INT[0] = stage;
        setUserCommand(cmd);
    } else {
        qDebug() << "RobotApiHandler::comEstimationCompensation() -- wrong stage: " << stage;
    }
}

void RobotApiHandler::eStop()
{
    qDebug() << "RobotApiHandler::eStop()";
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = QuadWalk;
    cmd.USER_COMMAND = QuadWalk_E_STOP;
    setUserCommand(cmd);
}

void RobotApiHandler::switchPowerOnOff(const PDU_PORT_IDs_e &pdu_port_id, const bool state)
{
    qDebug() << "RobotApiHandler::switchPowerOnOff() switch id " << pdu_port_id << " state: " << state;
    USER_COMMAND cmd;
    cmd.USER_COMMAND = DAEMON_PDU_POWER_CONTROL;
    cmd.COMMAND_TARGET = Daemon;
    cmd.USER_PARA_CHAR[0] = pdu_port_id;
    cmd.USER_PARA_CHAR[1] = state; // 0: OFF;     1: ON;
    setUserCommand(cmd);
}

void RobotApiHandler::switchPowerComm(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerCommunicationModule()" << state_;
    switchPowerOnOff(PDU_PORT_12V_COMM, state_);
}

void RobotApiHandler::switchPowerIrLEDs(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerIrLEDs()" << state_;
    switchPowerOnOff(PDU_PORT_12V_IRLed, state_);
}

void RobotApiHandler::switchPowerExt52V(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerExternalOutput48V()" << state_;
    switchPowerOnOff(PDU_PORT_48V_EXT, state_);
}

void RobotApiHandler::switchPowerLidar(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerLidar()" << state_;
    switchPowerOnOff(PDU_PORT_12V_Lidar, state_);
}

void RobotApiHandler::switchPowerThermal(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerTemperatureCamera()" << state_;
    switchPowerOnOff(PDU_PORT_12V_THER, state_);
}

void RobotApiHandler::switchPowerCctv(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerSecurityCamera()" << state_;
    switchPowerOnOff(PDU_PORT_12V_CCTV, state_);
}

void RobotApiHandler::switchPowerLegs(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerLegs()" << state_;
    switchPowerOnOff(PDU_PORT_48V_LEG, state_);
}

void RobotApiHandler::switchPowerArm(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerArm()" << state_;
    switchPowerOnOff(PDU_PORT_48V_ADD, state_);
}

void RobotApiHandler::switchPowerVisionPC(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerVisionPC()" << state_;
    switchPowerOnOff(PDU_PORT_12V_VisionPC, state_);
}

void RobotApiHandler::switchPowerUsbHub(const bool &state_)
{
    qDebug() << "RobotApiHandler::switchPowerUsbHub()" << state_;
    switchPowerOnOff(PDU_PORT_5V_CAMERAS, state_);
}

void RobotApiHandler::switchSecondaryGamepad(const bool &_secondary)
{
    qDebug() << "RobotApiHandler::switchSecondaryGamepad() " << _secondary;
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = Daemon;
    cmd.USER_COMMAND = DAEMON_EXT_JOYSTICK_ONOFF;
    cmd.USER_PARA_CHAR[0] = _secondary;
    setUserCommand(cmd);
}

void RobotApiHandler::setBodyHeight(const int &newBodyHeight)
{
    qDebug() << "RobotApiHandler::setBodyHeight() :" << newBodyHeight;
    if (-1 < newBodyHeight && newBodyHeight < 101) {
        if (newBodyHeight != m_bodyHeight) {
            m_bodyHeight = newBodyHeight;
            motionParametersUpdate();
        }
    } else {
        qDebug() << "RobotApiHandler::setBodyHeight() invalid height !! ";
    }
}

void RobotApiHandler::setFootHeight(const int &newFootHeight)
{
    qDebug() << "RobotApiHandler::setFootHeight() :" << newFootHeight;
    if (-1 < newFootHeight && newFootHeight < 101) {
        if (newFootHeight != m_footHeight) {
            m_footHeight = newFootHeight;
            motionParametersUpdate();
        }
    } else {
        qDebug() << "RobotApiHandler::setFootHeight() invalid height !! ";
    }
}

void RobotApiHandler::setMaxSpeed(const int &newMaxSpeed)
{
    qDebug() << "RobotApiHandler::setMaxSpeed() :" << newMaxSpeed;
    if (-1 < newMaxSpeed && newMaxSpeed < 101) {
        if (newMaxSpeed != m_maxSpeed) {
            m_maxSpeed = newMaxSpeed;
            motionParametersUpdate();
        }
    } else {
        qDebug() << "RobotApiHandler::setMaxSpeed() invalid speed !! ";
    }
}
