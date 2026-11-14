#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/ros2/CompressedImage_.hpp>
#include <rbq_sdk/idl/ros2/CameraInfo_.hpp>
#include <rbq_sdk/idl/ros2/PoseStamped_.hpp>

class VisionPublisher
{
public:
    explicit VisionPublisher(int sensorId);

    // bgr: width*height*3 contiguous uint8 (BGR order).
    // pose (optional): camera->base extrinsics as {px,py,pz, qw,qx,qy,qz}; null = skip TF.
    void publishRgb(int width, int height, const uint8_t *bgr,
                    const float intrinsics[4], const float coeffs[8], const double *pose = nullptr);
    // gray: width*height contiguous uint8 (single channel).
    void publishIr(int width, int height, const uint8_t *gray,
                   const float intrinsics[4], const float coeffs[8], const double *pose = nullptr);
    // depthMm: width*height contiguous uint16 (millimeters).
    void publishDepth(int width, int height, const uint16_t *depthMm,
                      const float intrinsics[4], const float coeffs[8], const double *pose = nullptr);

private:
    std::string topicBase() const;

    int m_sensorId;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>> m_rgb;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>      m_rgb_info;
    std::unique_ptr<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>   m_rgb_tf;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>> m_ir;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>      m_ir_info;
    std::unique_ptr<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>   m_ir_tf;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>> m_depth;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>      m_depth_info;
    std::unique_ptr<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>   m_depth_tf;
};
