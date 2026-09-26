#include "VisionPublisher.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/ros2/CompressedImage_.hpp>
#include <rbq_sdk/idl/ros2/CameraInfo_.hpp>
#include <rbq_sdk/idl/ros2/PoseStamped_.hpp>

namespace rbq_gazebo
{
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

void fillPose(geometry_msgs::msg::dds_::PoseStamped_ &msg, const CamPose &tf)
{
    stampNow(msg.header(), "base");
    msg.pose().position().x()    = tf.p[0];
    msg.pose().position().y()    = tf.p[1];
    msg.pose().position().z()    = tf.p[2];
    msg.pose().orientation().w() = tf.q[0];
    msg.pose().orientation().x() = tf.q[1];
    msg.pose().orientation().y() = tf.q[2];
    msg.pose().orientation().z() = tf.q[3];
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
                                 const float intrinsics[4], const float coeffs[8],
                                 const CamPose *tf)
{
    const std::string topicName = topicBase() + "/rgb";
    if (!m_rgb || !m_rgb_info) {
        m_rgb      = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>>(topicName + "/compressed");
        m_rgb_info = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>     (topicName + "/camera_info");
        m_rgb_tf   = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>  (topicName + "/tf");
    }
    // Query each matched-status once per frame (publication_matched_status is a DDS call).
    const bool wantImg  = m_rgb->hasSubscribers();
    const bool wantInfo = m_rgb_info->hasSubscribers();
    if (tf && m_rgb_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        fillPose(msg, *tf);
        m_rgb_tf->write(msg);
    }
    if (!wantImg && !wantInfo) return;

    if (wantImg) {
        cv::Mat img(height, width, CV_8UC3, const_cast<uint8_t *>(bgr), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::dds_::CompressedImage_ msg;
        stampNow(msg.header(), topicName);
        msg.format() = "jpeg";
        static const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
        thread_local std::vector<uchar> buf;   // reused across frames (no per-frame alloc)
        if (cv::imencode(".jpg", img, buf, params)) {
            msg.data().assign(buf.begin(), buf.end());
            m_rgb->write(msg);
        }
    }
    if (wantInfo) {
        sensor_msgs::msg::dds_::CameraInfo_ msg;
        fillCameraInfo(msg, topicName, width, height, intrinsics, coeffs);
        m_rgb_info->write(msg);
    }
}

void VisionPublisher::publishIr(int width, int height, const uint8_t *gray,
                                const float intrinsics[4], const float coeffs[8],
                                const CamPose *tf)
{
    const std::string topicName = topicBase() + "/ir";
    if (!m_ir || !m_ir_info) {
        m_ir      = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>>(topicName + "/compressed");
        m_ir_info = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>     (topicName + "/camera_info");
        m_ir_tf   = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>  (topicName + "/tf");
    }
    const bool wantImg  = m_ir->hasSubscribers();
    const bool wantInfo = m_ir_info->hasSubscribers();
    if (tf && m_ir_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        fillPose(msg, *tf);
        m_ir_tf->write(msg);
    }
    if (!wantImg && !wantInfo) return;

    if (wantImg) {
        cv::Mat img(height, width, CV_8U, const_cast<uint8_t *>(gray), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::dds_::CompressedImage_ msg;
        stampNow(msg.header(), topicName);
        msg.format() = "jpeg";
        static const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
        thread_local std::vector<uchar> buf;
        if (cv::imencode(".jpg", img, buf, params)) {
            msg.data().assign(buf.begin(), buf.end());
            m_ir->write(msg);
        }
    }
    if (wantInfo) {
        sensor_msgs::msg::dds_::CameraInfo_ msg;
        fillCameraInfo(msg, topicName, width, height, intrinsics, coeffs);
        m_ir_info->write(msg);
    }
}

void VisionPublisher::publishDepth(int width, int height, const uint16_t *depthMm,
                                   const float intrinsics[4], const float coeffs[8],
                                   const CamPose *tf)
{
    const std::string topicName = topicBase() + "/depth";
    if (!m_depth || !m_depth_info) {
        m_depth      = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CompressedImage_>>(topicName + "/compressed");
        m_depth_info = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::CameraInfo_>>     (topicName + "/camera_info");
        m_depth_tf   = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>  (topicName + "/tf");
    }
    const bool wantImg  = m_depth->hasSubscribers();
    const bool wantInfo = m_depth_info->hasSubscribers();
    if (tf && m_depth_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        fillPose(msg, *tf);
        m_depth_tf->write(msg);
    }
    if (!wantImg && !wantInfo) return;

    if (wantImg) {
        cv::Mat img(height, width, CV_16UC1, const_cast<uint16_t *>(depthMm), cv::Mat::AUTO_STEP);
        sensor_msgs::msg::dds_::CompressedImage_ msg;
        stampNow(msg.header(), topicName);
        msg.format() = "png";
        static const std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 6};
        thread_local std::vector<uchar> buf;
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

namespace
{

// ---- per-camera config ----------------------------------------------------
// hfov/sensorId match the camera definitions in the sensor model URDF.
struct CamCfg
{
    const char *name;
    int         sensorId;
    double      hfov;
    bool        ir;       // publish IR  (RGB -> gray)
    bool        color;    // publish RGB (RGB -> BGR)
};

const CamCfg kCams[] = {
    {"BT0", 0, 0.986, true,  false},
    {"BT1", 1, 0.986, true,  false},
    {"BT2", 2, 0.986, true,  false},
    {"BT3", 3, 0.986, true,  false},
    {"FT0", 4, 1.007, false, true },
    {"RR0", 5, 1.007, false, true },
};
constexpr int kNumCams = sizeof(kCams) / sizeof(kCams[0]);

// No lens distortion in sim; CameraInfo.d stays zero.
const float kZeroCoeffs[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void fillIntrinsics(float intr[4], int width, int height, double hfov)
{
    const float fx = static_cast<float>((width / 2.0) / std::tan(hfov / 2.0));
    intr[0] = fx;                                   // fx
    intr[1] = fx;                                   // fy == fx (square pixels)
    intr[2] = static_cast<float>(width  / 2.0);     // ppx
    intr[3] = static_cast<float>(height / 2.0);     // ppy
}

// BT0..BT3: bottom depth cameras (consumed by Heightmap).
constexpr int kNumBottomCams = 4;

// Latest bottom-cam depth passed from the gz callback to the wall-clock relay below.
struct DepthCache
{
    std::mutex            mtx;
    std::vector<uint16_t> data;
    int                   w = 0;
    int                   h = 0;
    float                 intr[4] = {0, 0, 0, 0};
    bool                  valid = false;
};

}  // namespace

void visionRun(std::atomic<bool> &running, CamPoseTable tfs)
{
    gz::transport::Node node;

    // One DDS publisher per camera (sensor id selects the topic suffix).
    std::vector<std::unique_ptr<VisionPublisher>> pubs;
    pubs.reserve(kNumCams);
    for (int i = 0; i < kNumCams; ++i) {
        pubs.push_back(std::make_unique<VisionPublisher>(kCams[i].sensorId));
    }

    // static storage so the callback lambdas reach it without capturing locals.
    static DepthCache depthCache[kNumBottomCams];
    static CamPoseTable camTfs;
    camTfs = tfs;

    // gz callbacks run on the node's thread pool for the node's lifetime.
    for (int i = 0; i < kNumCams; ++i) {
        const CamCfg &c = kCams[i];
        VisionPublisher *pub = pubs[i].get();
        const int sensorId = c.sensorId;
        const double hfov = c.hfov;
        // Static extrinsics of this camera's depth/color render pose (null when unknown).
        const CamPose *depthTf = camTfs[i].valid ? &camTfs[i].depth : nullptr;
        const CamPose *colorTf = camTfs[i].valid ? &camTfs[i].color : nullptr;

        // --- depth ---
        const std::string depthTopic = std::string("/rbq/camera/") + c.name + "/depth";
        node.Subscribe<gz::msgs::Image>(
            depthTopic,
            std::function<void(const gz::msgs::Image &)>(
                [pub, sensorId, hfov, depthTf](const gz::msgs::Image &msg) {
                    const int w = msg.width();
                    const int h = msg.height();
                    if (w <= 0 || h <= 0) return;
                    if (msg.data().size() < static_cast<size_t>(w) * h * sizeof(float)) return;

                    const float *d = reinterpret_cast<const float *>(msg.data().data());
                    std::vector<uint16_t> depth(static_cast<size_t>(w) * h);
                    for (size_t p = 0; p < depth.size(); ++p) {
                        const float m = d[p];
                        depth[p] = (std::isfinite(m) && m > 0.0f)
                            ? static_cast<uint16_t>(std::min(m * 1000.0f, 65535.0f))
                            : 0;
                    }
                    float intr[4];
                    fillIntrinsics(intr, w, h, hfov);
                    // Bottom cams feed the wall-clock relay (sole publisher); others publish directly.
                    if (sensorId < kNumBottomCams) {
                        DepthCache &slot = depthCache[sensorId];
                        std::lock_guard<std::mutex> lk(slot.mtx);
                        slot.data = std::move(depth);
                        slot.w = w;
                        slot.h = h;
                        std::memcpy(slot.intr, intr, sizeof(intr));
                        slot.valid = true;
                    } else {
                        pub->publishDepth(w, h, depth.data(), intr, kZeroCoeffs, depthTf);
                    }
                }));

        // --- color (IR gray, or BGR color) ---
        if (c.ir || c.color) {
            const std::string colorTopic = std::string("/rbq/camera/") + c.name + "/color";
            const bool wantIr = c.ir;
            node.Subscribe<gz::msgs::Image>(
                colorTopic,
                std::function<void(const gz::msgs::Image &)>(
                    [pub, hfov, wantIr, colorTf](const gz::msgs::Image &msg) {
                        const int w = msg.width();
                        const int h = msg.height();
                        if (w <= 0 || h <= 0) return;
                        if (msg.data().size() < static_cast<size_t>(w) * h * 3) return;

                        // gz publishes RGB_INT8; wrap as a Mat for OpenCV conversion.
                        const cv::Mat rgb(h, w, CV_8UC3,
                                          const_cast<char *>(msg.data().data()));
                        float intr[4];
                        fillIntrinsics(intr, w, h, hfov);
                        if (wantIr) {
                            cv::Mat gray;
                            cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
                            pub->publishIr(w, h, gray.data, intr, kZeroCoeffs, colorTf);
                        } else {
                            cv::Mat bgr;
                            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
                            pub->publishRgb(w, h, bgr.data, intr, kZeroCoeffs, colorTf);
                        }
                    }));
        }
    }

    std::printf("[vision] subscribed %d cameras.\n", kNumCams);
    // Wall-clock relay: re-publish bottom-cam depth at a fixed wall rate (decoupled from sim RTF).
    while (running) {
        for (int s = 0; s < kNumBottomCams; ++s) {
            DepthCache &slot = depthCache[s];
            std::lock_guard<std::mutex> lk(slot.mtx);
            if (!slot.valid) continue;
            pubs[s]->publishDepth(slot.w, slot.h, slot.data.data(), slot.intr, kZeroCoeffs,
                                  camTfs[s].valid ? &camTfs[s].depth : nullptr);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("[vision] stopped.\n");
}

}  // namespace rbq_gazebo
