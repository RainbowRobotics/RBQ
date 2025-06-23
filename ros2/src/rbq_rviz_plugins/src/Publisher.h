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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    Publisher(const std::string &nodeName) : Node("rbq_rviz_panel")
    {
        std::cout << nodeName << std::endl;

        m_pub_autoStart            = this->create_publisher<std_msgs::msg::Bool>("rbq/autoStart",            10);
        m_pub_canCheck             = this->create_publisher<std_msgs::msg::Bool>("rbq/canCheck",             10);
        m_pub_findHome             = this->create_publisher<std_msgs::msg::Bool>("rbq/findHome",             10);
        m_pub_sit                  = this->create_publisher<std_msgs::msg::Bool>("rbq/sit",                  10);
        m_pub_stand                = this->create_publisher<std_msgs::msg::Bool>("rbq/stand",                10);
        m_pub_walk                 = this->create_publisher<std_msgs::msg::Bool>("rbq/walk",                 10);
        m_pub_walkSlow             = this->create_publisher<std_msgs::msg::Bool>("rbq/walkSlow",             10);
        m_pub_run                  = this->create_publisher<std_msgs::msg::Bool>("rbq/run",                  10);
        m_pub_switchGamepadPort    = this->create_publisher<std_msgs::msg::Bool>("rbq/switchGamepadPort",    10);
        m_pub_calibrateImu         = this->create_publisher<std_msgs::msg::Bool>("rbq/calibrateImu",         10);
        m_pub_staticLock           = this->create_publisher<std_msgs::msg::Bool>("rbq/staticLock",           10);
        m_pub_staticReady          = this->create_publisher<std_msgs::msg::Bool>("rbq/staticReady",          10);
        m_pub_staticGround         = this->create_publisher<std_msgs::msg::Bool>("rbq/staticGround",         10);
        m_pub_recoveryErrorClear   = this->create_publisher<std_msgs::msg::Bool>("rbq/recoveryErrorClear",   10);
        m_pub_recoveryFlex         = this->create_publisher<std_msgs::msg::Bool>("rbq/recoveryFlex",         10);
        m_pub_emergency            = this->create_publisher<std_msgs::msg::Bool>("rbq/emergency",            10);
        m_pub_powerLeg             = this->create_publisher<std_msgs::msg::Bool>("rbq/powerLeg",             10);
        m_pub_powerArm             = this->create_publisher<std_msgs::msg::Bool>("rbq/powerArm",             10);
        m_pub_powerVisionPC        = this->create_publisher<std_msgs::msg::Bool>("rbq/powerVisionPC",        10);
        m_pub_powerUsbHub          = this->create_publisher<std_msgs::msg::Bool>("rbq/powerUsbHub",          10);
        m_pub_powerCctv            = this->create_publisher<std_msgs::msg::Bool>("rbq/powerCctv",            10);
        m_pub_powerThermal         = this->create_publisher<std_msgs::msg::Bool>("rbq/powerThermal",         10);
        m_pub_powerLidar           = this->create_publisher<std_msgs::msg::Bool>("rbq/powerLidar",           10);
        m_pub_powerExt52V          = this->create_publisher<std_msgs::msg::Bool>("rbq/powerExt52V",          10);
        m_pub_powerIrLEDs          = this->create_publisher<std_msgs::msg::Bool>("rbq/powerIrLEDs",          10);
        m_pub_powerComm            = this->create_publisher<std_msgs::msg::Bool>("rbq/powerComm",            10);
        m_pub_setBodyHeight        = this->create_publisher<std_msgs::msg::Char>("rbq/setBodyHeight",        10);
        m_pub_setFootHeight        = this->create_publisher<std_msgs::msg::Char>("rbq/setFootHeight",        10);
        m_pub_setMaxSpeed          = this->create_publisher<std_msgs::msg::Char>("rbq/setMaxSpeed",          10);
        m_pub_comEstimation        = this->create_publisher<std_msgs::msg::Char>("rbq/comEstimation",        10);
    }

    void pub_autoStart()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_autoStart->publish(msg);
    }

    void pub_stand()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_stand->publish(msg);
    }

    void pub_sit()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_sit->publish(msg);
    }

    void pub_canCheck()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_canCheck->publish(msg);
    }

    void pub_findHome()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_findHome->publish(msg);
    }

    void pub_walk()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_walk->publish(msg);
    }

    void pub_walkSlow()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_walkSlow->publish(msg);
    }

    void pub_run()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_run->publish(msg);
    }

    void pub_switchGamepadPort(const bool &rosJoy)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = rosJoy;
        m_pub_switchGamepadPort->publish(msg);
    }

    void pub_calibrateImu()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_calibrateImu->publish(msg);
    }

    void pub_staticLock()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_staticLock->publish(msg);
    }

    void pub_staticReady()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_staticReady->publish(msg);
    }

    void pub_staticGround()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_staticGround->publish(msg);
    }

    void pub_recoveryErrorClear()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_recoveryErrorClear->publish(msg);
    }

    void pub_recoveryFlex()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_recoveryFlex->publish(msg);
    }

    void pub_emergency()
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = true;
        m_pub_emergency->publish(msg);
    }

    void pub_powerLeg(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerLeg->publish(msg);
    }

    void pub_powerArm(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerArm->publish(msg);
    }

    void pub_powerVisionPC(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerVisionPC->publish(msg);
    }

    void pub_powerUsbHub(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerUsbHub->publish(msg);
    }

    void pub_powerCctv(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerCctv->publish(msg);
    }

    void pub_powerThermal(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerThermal->publish(msg);
    }

    void pub_powerLidar(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerLidar->publish(msg);
    }

    void pub_powerExt52V(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerExt52V->publish(msg);
    }

    void pub_powerIrLEDs(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerIrLEDs->publish(msg);
    }

    void pub_powerComm(const bool powerON)
    {
        std_msgs::msg::Bool msg = std_msgs::msg::Bool();
        msg.data = powerON;
        m_pub_powerComm->publish(msg);
    }

    void pub_setBodyHeight(const int &addedBodyHeight)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = addedBodyHeight;
        m_pub_setBodyHeight->publish(msg);
    }

    void pub_setFootHeight(const int &addedFootHeight)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = addedFootHeight;
        m_pub_setFootHeight->publish(msg);
    }

    void pub_setMaxSpeed(const int &maxSpeedPercent)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = maxSpeedPercent;
        m_pub_setMaxSpeed->publish(msg);
    }

    void pub_comEstimation(const int &calibrationStage)
    {
        std_msgs::msg::Char msg = std_msgs::msg::Char();
        msg.data = calibrationStage;
        m_pub_comEstimation->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_autoStart           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_stand               ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_sit                 ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_canCheck            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_findHome            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_walk                ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_walkSlow            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_run                 ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_switchGamepadPort   ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_calibrateImu        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_staticLock          ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_staticReady         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_staticGround        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_recoveryErrorClear  ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_recoveryFlex        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_emergency           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerLeg            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerArm            ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerVisionPC       ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerUsbHub         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerCctv           ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerThermal        ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerLidar          ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerExt52V         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerIrLEDs         ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pub_powerComm           ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_setBodyHeight       ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_setFootHeight       ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_setMaxSpeed         ;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr m_pub_comEstimation       ;

};

