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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rbq_msgs/msg/robot_state.hpp>
#include <rbq_msgs/msg/foot_states.hpp>

#include "RobotApiHandler.h"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    Publisher(const std::shared_ptr<RobotApiHandler> &robotApiHandler,
              const std::chrono::milliseconds &_m_sleepTime = 20ms)
        : Node("rbq_state_publisher")
        , m_sleepTime(_m_sleepTime)
        , m_robotApiHandler(robotApiHandler)
    {
        m_publisher_robotState = this->create_publisher<rbq_msgs::msg::RobotState>("rbq10/robot_state", 10);
        m_publisher_footStates = this->create_publisher<rbq_msgs::msg::FootStates>("rbq10/foot_state", 10);
        m_timer_robotState = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishRobotState, this));

        m_publisher_odometry = this->create_publisher<nav_msgs::msg::Odometry>("rbq10/odometry", 10);

        m_publisher_jointStates = this->create_publisher<sensor_msgs::msg::JointState>("rbq10/joint_states", 10);
        m_timer_jointStates = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishJointStates, this));

        m_publisher_IMUState = this->create_publisher<sensor_msgs::msg::Imu>("rbq10/IMU_state", 10);
        m_timer_IMUState = this->create_wall_timer(
            m_sleepTime, std::bind(&Publisher::publishIMUState, this));

    }

private:
    void publishRobotState()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            // RCLCPP_ERROR(this->get_logger(), "Shared Data is set to nullptr.");
            return;
        }
        rbq_msgs::msg::RobotState robotState_msg = rbq_msgs::msg::RobotState();
        // rbq_msgs::msg::FootStates footState_msg = rbq_msgs::msg::FootStates();
        {
            robotState_msg.gait_state               = robotState->robotStatus.GAIT_ID;
            robotState_msg.joints_comm_checked      = robotState->robotStatus.CAN_CHECK;
            robotState_msg.joints_calibrated        = robotState->robotStatus.FIND_HOME;
            robotState_msg.joints_control_started   = robotState->robotStatus.CON_START;
            robotState_msg.comm_connected           = true;
        }
        {
            robotState_msg.imu_state.header.stamp = this->get_clock()->now();
            robotState_msg.imu_state.header.frame_id = "body";
            robotState_msg.imu_state.orientation.w = robotState->imuState.quaternion.at(0);
            robotState_msg.imu_state.orientation.x = robotState->imuState.quaternion.at(1);
            robotState_msg.imu_state.orientation.y = robotState->imuState.quaternion.at(2);
            robotState_msg.imu_state.orientation.z = robotState->imuState.quaternion.at(3);
            robotState_msg.imu_state.angular_velocity.x = robotState->imuState.gyroscope.at(0);
            robotState_msg.imu_state.angular_velocity.y = robotState->imuState.gyroscope.at(1);
            robotState_msg.imu_state.angular_velocity.z = robotState->imuState.gyroscope.at(2);
            robotState_msg.imu_state.linear_acceleration.x = robotState->imuState.accelerometer.at(0);
            robotState_msg.imu_state.linear_acceleration.y = robotState->imuState.accelerometer.at(1);
            robotState_msg.imu_state.linear_acceleration.z = robotState->imuState.accelerometer.at(2);
            // RCLCPP_INFO(this->get_logger(),
            //     "Publishing rbq/imu -> \n\t ori.x: '%f' \n\t ori.y: '%f', \n\t ori.z: '%f'",
            //     imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
        }
        {
            robotState_msg.joint_states.header.stamp = this->get_clock()->now();
            for(int i=0; i<12; i++)
            {
                robotState_msg.joint_states.name.push_back(m_jointNames.at(i));
                robotState_msg.joint_states.position.push_back(  robotState->jointStates.at(i).position);
                // robotState_msg.joint_state.velocity.push_back(  robotState->jointStates.at(i).position);
                robotState_msg.joint_states.effort.push_back(    robotState->jointStates.at(i).torque);
            }
        }
        {
            robotState_msg.battery_state.charge_percentage      = robotState->batteryState.percentage.at(0);
            robotState_msg.battery_state.current                = robotState->batteryState.current.at(0);
            robotState_msg.battery_state.voltage                = robotState->batteryState.voltage.at(0);
            robotState_msg.battery_state.status                 = robotState->batteryState.charging.at(0);
            robotState_msg.battery_state.temperatures.push_back(robotState->batteryState.temperature.at(0));
        }
        {
            RBQ_SDK::Motion::LegStateArray_t* legStateArray_ = m_robotApiHandler->legStateArray();
            if(legStateArray_ != nullptr) {
                rbq_msgs::msg::FootState foot_state = rbq_msgs::msg::FootState();
                foreach (RBQ_SDK::Motion::LegState_t leg_state, legStateArray_->leg_states) {
                    foot_state.foot_position_rt_body.x = leg_state.foot_position_rt_body.at(0);
                    foot_state.foot_position_rt_body.y = leg_state.foot_position_rt_body.at(1);
                    foot_state.foot_position_rt_body.z = leg_state.foot_position_rt_body.at(2);
                    foot_state.foot_velocity_rt_body.x = leg_state.foot_velocity_rt_body.at(0);
                    foot_state.foot_velocity_rt_body.y = leg_state.foot_velocity_rt_body.at(1);
                    foot_state.foot_velocity_rt_body.z = leg_state.foot_velocity_rt_body.at(2);
                    foot_state.contact = leg_state.contact;
                    robotState_msg.foot_states.push_back(foot_state);
                    // RCLCPP_INFO(this->get_logger(),
                        // "Publishing foot state contact -> '%hhu'",
                                // (uint8_t)foot_state.contact);
                }
            }
        }
        {
            robotState_msg.body_pose_rt_world.header.frame_id = "body";
            robotState_msg.body_pose_rt_world.header.stamp = this->get_clock()->now();
            robotState_msg.body_pose_rt_world.pose.position.x       = robotState->odometries.odomWrtWorld.pose.position.at(0);
            robotState_msg.body_pose_rt_world.pose.position.y       = robotState->odometries.odomWrtWorld.pose.position.at(1);
            robotState_msg.body_pose_rt_world.pose.position.z       = robotState->odometries.odomWrtWorld.pose.position.at(2);
            robotState_msg.body_pose_rt_world.pose.orientation.x    = robotState->odometries.odomWrtWorld.pose.quaternion.at(0);
            robotState_msg.body_pose_rt_world.pose.orientation.y    = robotState->odometries.odomWrtWorld.pose.quaternion.at(1);
            robotState_msg.body_pose_rt_world.pose.orientation.z    = robotState->odometries.odomWrtWorld.pose.quaternion.at(2);
            robotState_msg.body_pose_rt_world.pose.orientation.w    = robotState->odometries.odomWrtWorld.pose.quaternion.at(3);

            robotState_msg.body_velocity_rt_world.body_frame_id     = "body";
            robotState_msg.body_velocity_rt_world.header.frame_id   = "";
            robotState_msg.body_velocity_rt_world.header.stamp      = this->get_clock()->now();
            robotState_msg.body_velocity_rt_world.velocity.linear.x = robotState->odometries.odomWrtWorld.velocity.linear.at(0);
            robotState_msg.body_velocity_rt_world.velocity.linear.y = robotState->odometries.odomWrtWorld.velocity.linear.at(1);
            robotState_msg.body_velocity_rt_world.velocity.linear.z = robotState->odometries.odomWrtWorld.velocity.linear.at(2);
            robotState_msg.body_velocity_rt_world.velocity.angular.x= robotState->odometries.odomWrtWorld.velocity.angular.at(0);
            robotState_msg.body_velocity_rt_world.velocity.angular.y= robotState->odometries.odomWrtWorld.velocity.angular.at(1);
            robotState_msg.body_velocity_rt_world.velocity.angular.z= robotState->odometries.odomWrtWorld.velocity.angular.at(2);
        }
        m_publisher_robotState->publish(robotState_msg);
        // footState_msg.header = robotState_msg.header;
        // footState_msg.foot_states = robotState_msg.foot_states;
        // m_publisher_footStates->publish(footState_msg);

        nav_msgs::msg::Odometry odometry_msg = nav_msgs::msg::Odometry();
        {
            odometry_msg.header.stamp               = this->get_clock()->now();
            odometry_msg.header.frame_id            = "local_world";
            odometry_msg.child_frame_id             = "body";
            odometry_msg.pose.pose.position.x       = robotState->odometries.odomWrtWorld.pose.position.at(0);;
            odometry_msg.pose.pose.position.y       = robotState->odometries.odomWrtWorld.pose.position.at(1);;
            odometry_msg.pose.pose.position.z       = robotState->odometries.odomWrtWorld.pose.position.at(2);;
            odometry_msg.pose.pose.orientation.x    = robotState->odometries.odomWrtWorld.pose.quaternion.at(0);;
            odometry_msg.pose.pose.orientation.y    = robotState->odometries.odomWrtWorld.pose.quaternion.at(1);;
            odometry_msg.pose.pose.orientation.z    = robotState->odometries.odomWrtWorld.pose.quaternion.at(2);;
            odometry_msg.pose.pose.orientation.w    = robotState->odometries.odomWrtWorld.pose.quaternion.at(3);;
        }
        m_publisher_odometry->publish(odometry_msg);
    }

    void publishJointStates()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();
        if(robotState == nullptr) {
            // RCLCPP_ERROR(this->get_logger(), "Shared Data is set to nullptr.");
            return;
        }
        sensor_msgs::msg::JointState jointState_msg = sensor_msgs::msg::JointState();
        jointState_msg.header.stamp = this->get_clock()->now();
        for(int i=0; i<12; i++)
        {
            jointState_msg.name.push_back(m_jointNames.at(i));
            jointState_msg.position.push_back(  robotState->jointStates.at(i).position);
            // jointState_msg.velocity.push_back(  robotState->jointStates.at(i).position);
            jointState_msg.effort.push_back(    robotState->jointStates.at(i).torque);
        }

        m_publisher_jointStates->publish(jointState_msg);
    }
    void publishIMUState()
    {
        RBQ_SDK::Motion::RobotState_t* robotState = m_robotApiHandler->robotState();

        sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();

        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "IMU";
        imu_msg.orientation.w = robotState->imuState.quaternion.at(0);
        imu_msg.orientation.x = robotState->imuState.quaternion.at(1);
        imu_msg.orientation.y = robotState->imuState.quaternion.at(2);
        imu_msg.orientation.z = robotState->imuState.quaternion.at(3);
        imu_msg.angular_velocity.x = robotState->imuState.gyroscope.at(0);
        imu_msg.angular_velocity.y = robotState->imuState.gyroscope.at(1);
        imu_msg.angular_velocity.z = robotState->imuState.gyroscope.at(2);
        imu_msg.linear_acceleration.x = robotState->imuState.accelerometer.at(0);
        imu_msg.linear_acceleration.y = robotState->imuState.accelerometer.at(1);
        imu_msg.linear_acceleration.z = robotState->imuState.accelerometer.at(2);
        
        m_publisher_IMUState -> publish(imu_msg);

    }
    std::chrono::milliseconds m_sleepTime = 100ms;

    rclcpp::TimerBase::SharedPtr m_timer_robotState;
    rclcpp::Publisher<rbq_msgs::msg::RobotState>::SharedPtr m_publisher_robotState;
    rclcpp::Publisher<rbq_msgs::msg::FootStates>::SharedPtr m_publisher_footStates;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher_odometry;

    rclcpp::TimerBase::SharedPtr m_timer_jointStates;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher_jointStates;

    rclcpp::TimerBase::SharedPtr m_timer_IMUState;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher_IMUState;

    // std::vector<std::string> m_jointNames{"HRR", "HRP", "HRK", "HLR", "HLP", "HLK", "FRR", "FRP", "FRK", "FLR", "FLP", "FLK"};
    std::vector<std::string> m_jointNames{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                                          "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                                          "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                          "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
    // ['RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint']

    std::shared_ptr<RobotApiHandler> m_robotApiHandler = nullptr;


};

