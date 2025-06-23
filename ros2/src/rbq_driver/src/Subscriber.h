// Copyright 2023 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
static Timer cmdJoyEnabledTimer;
static Timer cmdHighLevelTimer;

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rbq_msgs/msg/high_level_command.hpp>

#include "RobotApiHandler.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber(const std::shared_ptr<RobotApiHandler> &robotApiHandler)
        : Node("rbq_cmd_subscriber")
        , m_robotApiHandler(robotApiHandler) 
    {
        m_sub_cmd_highLevel = this->create_subscription<rbq_msgs::msg::HighLevelCommand>(
            "rbq/cmd_highLevel", 10, std::bind(&Subscriber::callback_cmd_highLevel, this, _1));

        m_sub_cmd_navigateTo = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "rbq/cmd_navigateTo", 10, std::bind(&Subscriber::callback_cmd_navigateTo, this, _1));

        m_sub_autoStart = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/autoStart", 10, std::bind(&Subscriber::callback_autoStart, this, _1));

        m_sub_canCheck = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/canCheck", 10, std::bind(&Subscriber::callback_canCheck, this, _1));

        m_sub_findHome = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/findHome", 10, std::bind(&Subscriber::callback_findHome, this, _1));

        m_sub_sit = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/sit", 10, std::bind(&Subscriber::callback_sit, this, _1));

        m_sub_stand = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/stand", 10, std::bind(&Subscriber::callback_stand, this, _1));

        m_sub_walk = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/walk", 10, std::bind(&Subscriber::callback_walk, this, _1));

        m_sub_walkSlow = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/walkSlow", 10, std::bind(&Subscriber::callback_walkSlow, this, _1));

        m_sub_run = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/run", 10, std::bind(&Subscriber::callback_run, this, _1));

        m_sub_switchGamepadPort = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/switchGamepadPort", 10, std::bind(&Subscriber::callback_switchGamepadPort, this, _1));

        m_sub_staticLock = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/staticLock", 10, std::bind(&Subscriber::callback_staticLock, this, _1));

        m_sub_staticReady = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/staticReady", 10, std::bind(&Subscriber::callback_staticReady, this, _1));

        m_sub_staticGround = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/staticGround", 10, std::bind(&Subscriber::callback_staticGround, this, _1));

        m_sub_recoveryErrorClear = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/recoveryErrorClear", 10, std::bind(&Subscriber::callback_recoveryErrorClear, this, _1));

        m_sub_recoveryFlex = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/recoveryFlex", 10, std::bind(&Subscriber::callback_recoveryFlex, this, _1));

        m_sub_emergency = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/emergency", 10, std::bind(&Subscriber::callback_emergency, this, _1));

        m_sub_powerLeg = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerLeg", 10, std::bind(&Subscriber::callback_powerLeg, this, _1));

        m_sub_powerArm = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerArm", 10, std::bind(&Subscriber::callback_powerArm, this, _1));

        m_sub_powerVisionPC = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerVisionPC", 10, std::bind(&Subscriber::callback_powerVisionPC, this, _1));

        m_sub_powerUsbHub = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerUsbHub", 10, std::bind(&Subscriber::callback_powerUsbHub, this, _1));

        m_sub_powerCctv = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerCctv", 10, std::bind(&Subscriber::callback_powerCctv, this, _1));

        m_sub_powerThermal = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerThermal", 10, std::bind(&Subscriber::callback_powerThermal, this, _1));

        m_sub_powerLidar = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerLidar", 10, std::bind(&Subscriber::callback_powerLidar, this, _1));

        m_sub_powerExt52V = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerExt52V", 10, std::bind(&Subscriber::callback_powerExt52V, this, _1));

        m_sub_powerIrLEDs = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerIrLEDs", 10, std::bind(&Subscriber::callback_powerIrLEDs, this, _1));

        m_sub_powerComm = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/powerComm", 10, std::bind(&Subscriber::callback_powerComm, this, _1));

        m_sub_calibrateImu = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/calibrateImu", 10, std::bind(&Subscriber::callback_calibrateImu, this, _1));

        m_sub_comEstimationCompensation = this->create_subscription<std_msgs::msg::Char>(
            "rbq/comEstimationCompensation", 10, std::bind(&Subscriber::callback_comEstimationCompensation, this, _1));

        m_sub_setBodyHeight = this->create_subscription<std_msgs::msg::Char>(
            "rbq/setBodyHeight", 10, std::bind(&Subscriber::callback_setBodyHeight, this, _1));

        m_sub_setFootHeight = this->create_subscription<std_msgs::msg::Char>(
            "rbq/setFootHeight", 10, std::bind(&Subscriber::callback_setFootHeight, this, _1));

        m_sub_setMaxSpeed = this->create_subscription<std_msgs::msg::Char>(
            "rbq/setMaxSpeed", 10, std::bind(&Subscriber::callback_setMaxSpeed, this, _1));
    }

private:
    rclcpp::Subscription<rbq_msgs::msg::HighLevelCommand>::SharedPtr m_sub_cmd_highLevel;
    void callback_cmd_highLevel(const rbq_msgs::msg::HighLevelCommand::SharedPtr _highLevel) const
    {
        if(cmdHighLevelTimer.elapsed() < 0.02f) {
            return;
        }

        RBQ_SDK::HighLevelCmd_t cmd_;

        cmd_.roll         = _highLevel->roll;
        cmd_.pitch        = _highLevel->pitch;
        cmd_.yaw          = _highLevel->yaw;
        cmd_.vel_x        = _highLevel->vel_x;
        cmd_.vel_y        = _highLevel->vel_y;
        cmd_.omega_z      = _highLevel->omega_z;
        cmd_.delta_body_h = _highLevel->delta_body_h;
        cmd_.delta_foot_h = _highLevel->delta_foot_h;

        if(m_robotApiHandler != nullptr) {
            m_robotApiHandler->setHighLevelCommand(cmd_);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_cmd_navigateTo;
    void callback_cmd_navigateTo(const geometry_msgs::msg::PoseStamped::SharedPtr _pose) const
    {
        m_robotApiHandler->motionDynamicNavigateTo(_pose->pose.position.x, _pose->pose.position.y, _pose->pose.orientation.z);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_autoStart;
    void callback_autoStart(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->autoStart();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_canCheck;
    void callback_canCheck(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->canCheck();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_findHome;
    void callback_findHome(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->findHome();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_sit;
    void callback_sit(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicGround();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_stand;
    void callback_stand(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicReady();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_walk;
    void callback_walk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicWalk();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_walkSlow;
    void callback_walkSlow(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicWalkSlow();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_run;
    void callback_run(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionDynamicRun();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_switchGamepadPort;
    void callback_switchGamepadPort(const std_msgs::msg::Bool::SharedPtr _secondChannel) const {
        m_robotApiHandler->switchSecondaryGamepad(_secondChannel.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_staticLock;
    void callback_staticLock(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionStaticLock();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_staticReady;
    void callback_staticReady(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionStaticReady();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_staticGround;
    void callback_staticGround(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->motionStaticGround();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_recoveryErrorClear;
    void callback_recoveryErrorClear(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->recovery1();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_recoveryFlex;
    void callback_recoveryFlex(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->recovery2();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_emergency;
    void callback_emergency(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->eStop();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerLeg;
    void callback_powerLeg(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerLegs(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerArm;
    void callback_powerArm(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerArm(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerVisionPC;
    void callback_powerVisionPC(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerVisionPC(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerUsbHub;
    void callback_powerUsbHub(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerUsbHub(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerCctv;
    void callback_powerCctv(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerCctv(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerThermal;
    void callback_powerThermal(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerThermal(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerLidar;
    void callback_powerLidar(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerLidar(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerExt52V;
    void callback_powerExt52V(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerExt52V(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerIrLEDs;
    void callback_powerIrLEDs(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerIrLEDs(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_powerComm;
    void callback_powerComm(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        m_robotApiHandler->switchPowerComm(_confirm.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_calibrateImu;
    void callback_calibrateImu(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm.get()->data) {
            m_robotApiHandler->calibrateAccelerometer();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_comEstimationCompensation;
    void callback_comEstimationCompensation(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->comEstimationCompensation(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_setBodyHeight;
    void callback_setBodyHeight(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->setBodyHeight(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_setFootHeight;
    void callback_setFootHeight(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->setFootHeight(_stage.get()->data);
    }

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr m_sub_setMaxSpeed;
    void callback_setMaxSpeed(const std_msgs::msg::Char::SharedPtr _stage) const {
        m_robotApiHandler->setMaxSpeed(_stage.get()->data);
    }

    std::shared_ptr<RobotApiHandler> m_robotApiHandler = nullptr;


};
