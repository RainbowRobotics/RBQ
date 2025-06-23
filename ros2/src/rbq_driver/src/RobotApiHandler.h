#pragma once

#include "RBTypes.hpp"
#include "rbq_podo_api.h"

class RobotApiHandler
{
public:
    explicit RobotApiHandler(const std::string &host = "192.168.0.10", const int &_commFrequency = 100);
    ~RobotApiHandler();

    RBQ_SDK::Motion::RobotState_t *robotState();

    RBQ_SDK::Motion::LegStateArray_t *legStateArray();

    void setHighLevelCommand(const RBQ_SDK::HighLevelCmd_t &_cmd);

    void startMotionProgram(const bool &_start, const int &_id);

    void autoStart();

    void canCheck();

    void findHome();

    void motionStaticLock();

    void motionStaticReady();

    void motionStaticGround();

    void motionDynamicGround();

    void motionDynamicReady();

    void motionDynamicWalk();

    void motionDynamicWalkSlow();

    void motionDynamicWalkStairs();

    void motionDynamicRun();

    void motionDynamicAim();

    void motionParametersUpdate();

    // approach mode selection
    // approach mode 0 : rotate to targetXY -> straight -> rotate to target Theta
    // approach mode 1 : rotate to target Theta -> diagonal walk to targetXY
    // approach mode 2 : diagonal walk to targetXY-> rotate to rotate to target Theta
    // approach mode 3 : rotate target Theta and dinagonal walk to targetXY simultanuously
    // approach mode 4 : rotate target Theta and dinagonal walk to targetXY simultanuously & wide walking

    ///
    /// \brief dynamic wave walk navigate towards given target pose
    /// \param _targetPosX : target position X
    /// \param _targetPosY : target position Y
    /// \param _targetRotZ : target rotation Z
    /// \param _mode select:
    ///             mode 0 : rotate to targetXY -> straight -> rotate to target Theta
    ///             mode 1 : rotate to target Theta -> diagonal walk to targetXY
    ///             mode 2 : diagonal walk to targetXY-> rotate to rotate to target Theta
    ///             mode 3 : rotate target Theta and dinagonal walk to targetXY simultanuously
    ///             mode 4 : rotate target Theta and dinagonal walk to targetXY simultanuously & wide walking
    ///
    void motionDynamicNavigateTo(const float &_targetPosX, const float &_targetPosY, const float &_targetRotZ, const int &_mode = 0);

    void recovery1();

    void recovery2();

    void eStop();


    void switchPowerComm(const bool &state_ = false);

    void switchPowerIrLEDs(const bool &state_ = false);

    void switchPowerExt52V(const bool &state_ = false);

    void switchPowerLidar(const bool &state_ = false);

    void switchPowerThermal(const bool &state_ = false);

    void switchPowerCctv(const bool &state_ = false);

    void switchPowerLegs(const bool &state_ = false);

    void switchPowerArm(const bool &state_ = false);

    void switchPowerVisionPC(const bool &state_ = false);

    void switchPowerUsbHub(const bool &state_ = false);


    void calibrateAccelerometer();

    void comEstimationCompensation(const int &stage = 0);

    void switchSecondaryGamepad(const bool &_secondary = false);

    void setBodyHeight(const int &newBodyHeight);

    void setFootHeight(const int &newFootHeight);

    void setMaxSpeed(const int &newMaxSpeed);

    bool walking() const { return m_walking; }

    void setWalking(bool newWalking) { m_walking = newWalking; }

private:
    void setUserCommand(const USER_COMMAND &usrCmd);

    void qAppThread(const std::string &host, const int &_commFrequency = 100);

    /// Interface for turning on and off devices powered by PDU in the robot
    /// arg: state_ = false --> turn off
    void switchPowerOnOff(const PDU_PORT_IDs_e &pdu_port_id, const bool state);

private:
    // Motion Parameters in Percents %
    int m_bodyHeight = 50;
    int m_footHeight = 50;
    int m_maxSpeed = 50;

    bool m_walking = false;
};
















