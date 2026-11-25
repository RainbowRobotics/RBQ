#ifndef RBQ_GAZEBO__VISION_PUBLISHER_HPP_
#define RBQ_GAZEBO__VISION_PUBLISHER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/ros2/CompressedImage_.hpp>
#include <rbq_sdk/idl/ros2/CameraInfo_.hpp>
#include <rbq_sdk/idl/ros2/PoseStamped_.hpp>

namespace rbq_gazebo
{

// Camera->base extrinsics (optical frame x right, y down, z forward).
struct CamPose
{
    double p[3];  // x, y, z
    double q[4];  // w, x, y, z
};

// Per camera: the depth sensor pose and the color sensor pose.
struct CamPoses
{
    CamPose depth;
    CamPose color;
    bool    valid = false;
};

constexpr int kNumTfCams = 6;  // BT0-3, FT0, RR0
using CamPoseTable = std::array<CamPoses, kNumTfCams>;

// Per-sensor compressed image + CameraInfo + camera->base TF over DDS for HAL.
class VisionPublisher
{
public:
    explicit VisionPublisher(int sensorId);

    // bgr: width*height*3 contiguous uint8 (BGR order). tf: optical->base (may be null).
    void publishRgb(int width, int height, const uint8_t *bgr,
                    const float intrinsics[4], const float coeffs[8], const CamPose *tf);
    // gray: width*height contiguous uint8 (single channel).
    void publishIr(int width, int height, const uint8_t *gray,
                   const float intrinsics[4], const float coeffs[8], const CamPose *tf);
    // depthMm: width*height contiguous uint16 (millimeters).
    void publishDepth(int width, int height, const uint16_t *depthMm,
                      const float intrinsics[4], const float coeffs[8], const CamPose *tf);

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

// Subscribe the 6 gz camera topics and publish them over DDS
void visionRun(std::atomic<bool> &running, CamPoseTable tfs);

}  // namespace rbq_gazebo

#endif  // RBQ_GAZEBO__VISION_PUBLISHER_HPP_
