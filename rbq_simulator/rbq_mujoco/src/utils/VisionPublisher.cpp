#include "VisionPublisher.h"

#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>

namespace
{
void stampNow(std_msgs::msg::dds_::Header_ &h, const std::string &frame)
{
    auto now = std::chrono::system_clock::now();
    auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    h.stamp().sec()     = std::chrono::duration_cast<std::chrono::seconds>(ns).count();
    h.stamp().nanosec() = ns.count() % 1000000000ULL;
    h.frame_id()        = frame;
}

// pose: {px,py,pz, qw,qx,qy,qz} camera->base extrinsics (optical frame).
void fillPose(geometry_msgs::msg::dds_::PoseStamped_ &msg, const double pose[7])
{
    stampNow(msg.header(), "base");
    msg.pose().position().x()    = pose[0];
    msg.pose().position().y()    = pose[1];
    msg.pose().position().z()    = pose[2];
    msg.pose().orientation().w() = pose[3];
    msg.pose().orientation().x() = pose[4];
    msg.pose().orientation().y() = pose[5];
    msg.pose().orientation().z() = pose[6];
}

void fillCameraInfo(sensor_msgs::msg::dds_::CameraInfo_ &msg, const std::string &frame,
                    uint32_t width, uint32_t height,
                    const float intrinsics[4], const float coeffs[8])
{
    stampNow(msg.header(), frame);
    msg.width()            = width;
    msg.height()           = height;
    msg.distortion_model() = "rational_polynomial";
    msg.d().assign(coeffs, coeffs + 8);
    msg.k() = {intrinsics[0], 0.0,           intrinsics[2],
               0.0,           intrinsics[1], intrinsics[3],
               0.0,           0.0,           1.0};
    msg.r() = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
    msg.p() = {intrinsics[0], 0.0,           intrinsics[2], 0.0,
               0.0,           intrinsics[1], intrinsics[3], 0.0,
               0.0,           0.0,           1.0,           0.0};
    msg.binning_x() = 0;
    msg.binning_y() = 0;
    msg.roi().x_offset()   = 0;
    msg.roi().y_offset()   = 0;
    msg.roi().width()      = 0;
    msg.roi().height()     = 0;
    msg.roi().do_rectify() = false;
}
}  // namespace

VisionPublisher::VisionPublisher(int sensorId) : m_sensorId(sensorId) {}

std::string VisionPublisher::topicBase() const
{
    return "rt/rbq/vision/sensor_" + std::to_string(m_sensorId);
}

void VisionPublisher::publishRgb(int width, int height, const uint8_t *bgr,
                                 const float intrinsics[4], const float coeffs[8], const double *pose)
{
    const std::string topicName = topicBase() + "/rgb";
    if (!m_rgb || !m_rgb_info) {
        m_rgb      = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>>(topicName + "/compressed");
        m_rgb_info = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>     (topicName + "/camera_info");
        m_rgb_tf   = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>  (topicName + "/tf");
    }
    if (pose && m_rgb_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        fillPose(msg, pose);
        m_rgb_tf->write(msg);
    }
    if (!m_rgb->hasSubscribers() && !m_rgb_info->hasSubscribers()) return;

    if (m_rgb->hasSubscribers()) {
        cv::Mat img(height, width, CV_8UC3, const_cast<uint8_t *>(bgr), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::dds_::CompressedImage_ msg;
        stampNow(msg.header(), topicName);
        msg.format() = "jpeg";
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
        std::vector<uchar> buf;
        if (cv::imencode(".jpg", img, buf, params)) {
            msg.data().assign(buf.begin(), buf.end());
            m_rgb->write(msg);
        }
    }
    if (m_rgb_info->hasSubscribers()) {
        sensor_msgs::msg::dds_::CameraInfo_ msg;
        fillCameraInfo(msg, topicName, width, height, intrinsics, coeffs);
        m_rgb_info->write(msg);
    }
}

void VisionPublisher::publishIr(int width, int height, const uint8_t *gray,
                                const float intrinsics[4], const float coeffs[8], const double *pose)
{
    const std::string topicName = topicBase() + "/ir";
    if (!m_ir || !m_ir_info) {
        m_ir      = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>>(topicName + "/compressed");
        m_ir_info = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>     (topicName + "/camera_info");
        m_ir_tf   = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>  (topicName + "/tf");
    }
    if (pose && m_ir_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        fillPose(msg, pose);
        m_ir_tf->write(msg);
    }
    if (!m_ir->hasSubscribers() && !m_ir_info->hasSubscribers()) return;

    if (m_ir->hasSubscribers()) {
        cv::Mat img(height, width, CV_8U, const_cast<uint8_t *>(gray), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::dds_::CompressedImage_ msg;
        stampNow(msg.header(), topicName);
        msg.format() = "jpeg";
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
        std::vector<uchar> buf;
        if (cv::imencode(".jpg", img, buf, params)) {
            msg.data().assign(buf.begin(), buf.end());
            m_ir->write(msg);
        }
    }
    if (m_ir_info->hasSubscribers()) {
        sensor_msgs::msg::dds_::CameraInfo_ msg;
        fillCameraInfo(msg, topicName, width, height, intrinsics, coeffs);
        m_ir_info->write(msg);
    }
}

void VisionPublisher::publishDepth(int width, int height, const uint16_t *depthMm,
                                   const float intrinsics[4], const float coeffs[8], const double *pose)
{
    const std::string topicName = topicBase() + "/depth";
    if (!m_depth || !m_depth_info) {
        m_depth      = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>>(topicName + "/compressed");
        m_depth_info = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>     (topicName + "/camera_info");
        m_depth_tf   = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>  (topicName + "/tf");
    }
    if (pose && m_depth_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        fillPose(msg, pose);
        m_depth_tf->write(msg);
    }
    if (!m_depth->hasSubscribers() && !m_depth_info->hasSubscribers()) return;

    if (m_depth->hasSubscribers()) {
        cv::Mat img(height, width, CV_16UC1, const_cast<uint16_t *>(depthMm), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::dds_::CompressedImage_ msg;
        stampNow(msg.header(), topicName);
        msg.format() = "png";
        std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 6};
        std::vector<uchar> buf;
        if (cv::imencode(".png", img, buf, params)) {
            msg.data().assign(buf.begin(), buf.end());
            m_depth->write(msg);
        }
    }
    if (m_depth_info->hasSubscribers()) {
        sensor_msgs::msg::dds_::CameraInfo_ msg;
        fillCameraInfo(msg, topicName, width, height, intrinsics, coeffs);
        m_depth_info->write(msg);
    }
}
