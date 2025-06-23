// Copyright 2024 Rainbow Robotics Inc.
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

#include <string>
#include <chrono>

// opencv
#include <opencv2/opencv.hpp>

// ros
#include "rclcpp/rclcpp.hpp"

#define RBQ_ROS_MAJOR_VERSION    2
#define RBQ_ROS_MINOR_VERSION    5
#define RBQ_ROS_PATCH_VERSION    1

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define RBQ_ROS_VERSION_STR (VAR_ARG_STRING(RBQ_ROS_MAJOR_VERSION.RBQ_ROS_MINOR_VERSION.RBQ_ROS_PATCH_VERSION))

#define ROS_DEBUG(...) RCLCPP_DEBUG(_logger, __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(_logger, __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(_logger, __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(_logger, __VA_ARGS__)

// Based on: https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(_logger, msg)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(_logger, msg)
#define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(_logger, msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(_logger, msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(_logger, msg)
#define ROS_DEBUG_STREAM_ONCE(msg) RCLCPP_DEBUG_STREAM_ONCE(_logger, msg)
#define ROS_INFO_STREAM_ONCE(msg) RCLCPP_INFO_STREAM_ONCE(_logger, msg)
#define ROS_WARN_STREAM_COND(cond, msg) RCLCPP_WARN_STREAM_EXPRESSION(_logger, cond, msg)

#define ROS_WARN_ONCE(msg) RCLCPP_WARN_ONCE(_logger, msg)
#define ROS_WARN_COND(cond, ...) RCLCPP_WARN_EXPRESSION(_logger, cond, __VA_ARGS__)

#include <sensor_msgs/msg/image.hpp>
// cv_bridge.h last supported version is humble
#if defined(CV_BRDIGE_HAS_HPP)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
namespace enc = sensor_msgs::image_encodings;

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Geometry>

#include "ipc.h"

using namespace std::chrono_literals;

class VisionPublisher : public rclcpp::Node
{
public:
    VisionPublisher(const std::chrono::milliseconds &_m_sleepTime = 20ms)
        : Node("rbq_vision_publisher")
        , m_sleepTime(_m_sleepTime)
        , _logger(this->get_logger())
    {
        m_ipc = new Vision_IPC();

        std::string name_prepend = "rbq/vision/";
        // Bottom 0~3, Front, Rear
        for(int idx = 0; idx < 6; idx++)
        {
            std::string name = (idx == 4) ? "sensor_front" : (idx == 5) ? "sensor_rear" : QString::asprintf("sensor_bottom_%d", idx).toStdString();
            VisionSensor_t sensor_;
            sensor_.setEnabled(name_prepend + name, (Vision_IPC::Sensors_e)idx);
            sensor_.setEnabledIR(this);
            if(idx == 4 || idx == 5) {
                sensor_.setEnabledRGB(this);
                sensor_.setEnabledCompressed(this);
            }
            sensor_.setEnabledDepth(this);
            sensor_.setEnabledCameraInfo(this);
            m_sensors.push_back(sensor_);
        }
        // Left
        {
            std::string name = "sensor_left";
            VisionSensor_t sensor_;
            sensor_.setEnabled(name_prepend + name, Vision_IPC::Sensors_e::Left);
            sensor_.setEnabledRGB(this);
            m_sensors.push_back(sensor_);
        }
        // Right
        {
            std::string name = "sensor_right";
            VisionSensor_t sensor_;
            sensor_.setEnabled(name_prepend + name, Vision_IPC::Sensors_e::Right);
            sensor_.setEnabledRGB(this);
            m_sensors.push_back(sensor_);
        }

        m_timer = this->create_wall_timer(m_sleepTime, std::bind(&VisionPublisher::publishOnce, this));

        restartStaticTransformBroadcaster();
    }

    ~VisionPublisher()
    {
        delete m_ipc;
    }

protected:
    class VisionSensor_t {
    public:
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb, pub_ir, pub_depth;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_rgb_compressed;
        std::string name, name_rgb, name_ir, name_depth, name_camera_info, name_rgb_compressed = "";
        bool enabled, enabled_rgb, enabled_ir, enabled_depth, enabled_pcd, enabled_camera_info = false;
        Vision_IPC::Sensors_e id;
        void setEnabled(const std::string _name, const Vision_IPC::Sensors_e &_id) {
            enabled = true;
            name = _name;
            id = _id;
        }
        void setEnabledIR(rclcpp::Node *_node) {
            enabled_ir = true;
            name_ir = name+"/ir";
            pub_ir = _node->create_publisher<sensor_msgs::msg::Image>(name_ir, 10);
        }
        void setEnabledRGB(rclcpp::Node *_node) {
            enabled_rgb = true;
            name_rgb = name+"/rgb";
            pub_rgb = _node->create_publisher<sensor_msgs::msg::Image>(name_rgb, 10);
        }
        void setEnabledCameraInfo(rclcpp::Node *_node) {
            enabled_camera_info = true;
            name_camera_info = name + "/camera_info";
            pub_camera_info = _node->create_publisher<sensor_msgs::msg::CameraInfo>(name_camera_info, 10);
        }
        void setEnabledDepth(rclcpp::Node *_node) {
            enabled_depth = true;
            name_depth = name + "/depth";
            pub_depth = _node->create_publisher<sensor_msgs::msg::Image>(name_depth, 10);
        }
        void setEnabledCompressed(rclcpp::Node *_node) {
            name_rgb_compressed = name_rgb + "/compressed";
            pub_rgb_compressed = _node->create_publisher<sensor_msgs::msg::CompressedImage>(name_rgb_compressed, 10);
        }
        bool getSubCountIR() {
            return enabled_ir && pub_ir && pub_ir->get_subscription_count() > 0;
        }
        bool getSubCountRgb() {
            return enabled_rgb && pub_rgb && pub_rgb->get_subscription_count() > 0;
        }
        bool getSubCountDepth() {
            return enabled_depth && pub_depth && pub_depth->get_subscription_count() > 0;
        }
        bool getSubCountCameraInfo() {
            return enabled_camera_info && pub_camera_info && pub_camera_info->get_subscription_count() > 0;
        }
        bool getSubCountRGBCompressed() {
            return pub_rgb_compressed && pub_rgb_compressed->get_subscription_count() > 0;
        }

    };

private:
    void publishOnce() {
        if(nullptr == m_ipc) {
            return;
        }
        resetStaticTfMsgs();
        std::string parent_frame_id = "base";

        foreach (auto sensor_, m_sensors) {
            if(sensor_.enabled) {
                if(sensor_.getSubCountIR()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_ir[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_ir_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageIR_t imageIR_;
                        m_ipc->getImageIR(sensor_.id, imageIR_);
                        cv::Mat img_(imageIR_.height, imageIR_.width, CV_8U, (void*)(imageIR_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_ir;
                        sensor_.pub_ir->publish(*(cv_bridge::CvImage(header, enc::MONO8, img_).toImageMsg()));
                    }
                }
                if(sensor_.getSubCountRgb()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_color[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_color_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageColor_t imageColor_;
                        m_ipc->getImageColor(sensor_.id, imageColor_);
                        cv::Mat img_(imageColor_.height, imageColor_.width, CV_8UC3, (void*)(imageColor_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_rgb;
                        sensor_.pub_rgb->publish(*(cv_bridge::CvImage(header, enc::BGR8, img_).toImageMsg()));
                    }
                }
                if(sensor_.getSubCountDepth()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_depth[sensor_.id]);
                    if (0 < tick_ && tick_ != m_ipc->shm_depth_tickWrite[sensor_.id]) {
                        Vision_IPC::Depth_t depth_;
                        if (Vision_IPC::Error_e::noError == m_ipc->getDepth(sensor_.id, depth_)) {
                            cv::Mat depth_img(depth_.height, depth_.width, CV_16UC1, (void*)(depth_.data), cv::Mat::AUTO_STEP);

                            std_msgs::msg::Header header;
                            header.stamp = this->now();
                            header.frame_id = sensor_.name_depth;

                            // Publish depth image (16-bit depth in mm)
                            sensor_.pub_depth->publish(*(cv_bridge::CvImage(header, enc::TYPE_16UC1, depth_img).toImageMsg()));

                            // Publish the transformation matrix for the depth frame
                            appendStaticTfMsg(depth_.TF, parent_frame_id, sensor_.name_depth);
                        }
                        if (sensor_.getSubCountCameraInfo()) {
                            sensor_msgs::msg::CameraInfo::UniquePtr msg_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
                            msg_camera_info->header.stamp = this->now();
                            msg_camera_info->header.frame_id = sensor_.name_depth;
                            msg_camera_info->width = depth_.width;
                            msg_camera_info->height = depth_.height;

                            // Set intrinsic parameters
                            msg_camera_info->k[0] = depth_.intrinsics[0];  // fx
                            msg_camera_info->k[2] = depth_.intrinsics[2];  // cx
                            msg_camera_info->k[4] = depth_.intrinsics[1];  // fy
                            msg_camera_info->k[5] = depth_.intrinsics[3];  // cy
                            msg_camera_info->k[8] = 1.0;  // Identity

                            // Set distortion coefficients
                            msg_camera_info->d = std::vector<double>(5);
                            for (int i = 0; i < 5; i++) {
                                msg_camera_info->d[i] = depth_.coeffs[i];
                            }
                            sensor_.pub_camera_info->publish(std::move(msg_camera_info));
                        }
                    }
                }
                if(sensor_.getSubCountRGBCompressed()) {
                    uint32_t tick_ = m_ipc->getTickWrite(m_ipc->shm_color[sensor_.id]);
                    if(0 < tick_ && tick_ != m_ipc->shm_color_tickWrite[sensor_.id]) {
                        Vision_IPC::ImageColor_t imageColor_;
                        m_ipc->getImageColor(sensor_.id, imageColor_);
                        cv::Mat img_(imageColor_.height, imageColor_.width, CV_8UC3, (void*)(imageColor_.data), cv::Mat::AUTO_STEP);
                        std_msgs::msg::Header header;
                        header.stamp = this->now();
                        header.frame_id = sensor_.name_rgb;
                        sensor_msgs::msg::CompressedImage::UniquePtr compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                        compressed_msg->header = header;
                        compressed_msg->format = "jpeg";
                        cv::imencode(".jpg", img_, compressed_msg->data);
                        sensor_.pub_rgb_compressed->publish(std::move(compressed_msg));
                    }
                }
            }
        }
        publishDynamicTransforms();
    }

    void publishDynamicTransforms()
    {
        if (!m_dynamic_tf_broadcaster)
        {
            m_dynamic_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        rclcpp::Time t = this->now();
        try
        {
            for(auto& msg : m_static_tf_msgs)
                msg.header.stamp = t;
            m_dynamic_tf_broadcaster->sendTransform(m_static_tf_msgs);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("Error publishing dynamic transforms: " << e.what());
        }
    }

    void restartStaticTransformBroadcaster()
    {
        if (m_static_tf_broadcaster) { m_static_tf_broadcaster.reset(); }
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
        options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this,  tf2_ros::StaticBroadcasterQoS(), std::move(options));
        resetStaticTfMsgs();
        m_static_tf_broadcaster->sendTransform(m_static_tf_msgs);
    }

    void resetStaticTfMsgs()
    {
        m_static_tf_msgs.clear();
    }

    void appendStaticTfMsg(const float TfMat[12], const std::string& from, const std::string& to)
    {
        /*
        rotation[0], rotation[3], rotation[6], translation[9 ],
        rotation[1], rotation[4], rotation[7], translation[10],
        rotation[2], rotation[5], rotation[8], translation[11],
                  0,           0,           0,               1;
        */
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = from;
        msg.child_frame_id = to;

        msg.transform.translation.x = TfMat[9];
        msg.transform.translation.y = TfMat[10];
        msg.transform.translation.z = TfMat[11];

        Eigen::Matrix3f m;
        // m<< TfMat[0], TfMat[3], TfMat[6],
        //     TfMat[1], TfMat[4], TfMat[7],
        //     TfMat[2], TfMat[5], TfMat[8];
        m<< 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        Eigen::Quaternionf q(m);

        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();

        m_static_tf_msgs.push_back(msg);
    }

    void appendStaticTfMsg(const Eigen::Matrix4d TfMat, const std::string& from, const std::string& to)
    {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = from;
        msg.child_frame_id = to;

        msg.transform.translation.x = TfMat(0,3);
        msg.transform.translation.y = TfMat(1,3);
        msg.transform.translation.z = TfMat(2,3);

        Eigen::Quaterniond q(TfMat.block<3,3>(0,0));

        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();

        m_static_tf_msgs.push_back(msg);
    }

    std::chrono::milliseconds m_sleepTime = 100ms;
    rclcpp::TimerBase::SharedPtr m_timer;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>    m_static_tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster>          m_dynamic_tf_broadcaster;
    std::vector<geometry_msgs::msg::TransformStamped>       m_static_tf_msgs;

    std::vector<VisionSensor_t> m_sensors;

    Vision_IPC* m_ipc = nullptr;

    rclcpp::Logger _logger;
};

