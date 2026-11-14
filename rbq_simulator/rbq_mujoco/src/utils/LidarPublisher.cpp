#include "LidarPublisher.h"

#include <chrono>

#include <rbq_sdk/dds/LidarDds.hpp>

namespace
{
void nowStamp(uint32_t &sec, uint32_t &nanosec)
{
    auto now = std::chrono::system_clock::now();
    auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
    sec     = std::chrono::duration_cast<std::chrono::seconds>(ns).count();
    nanosec = ns.count() % 1000000000ULL;
}
}  // namespace

LidarPublisher::LidarPublisher(const std::string &cloudTopic, const std::string &frameId)
    : m_frameId(frameId)
{
    m_cloud = std::make_unique<rbq_sdk::Publisher<sensor_msgs::msg::dds_::PointCloud2_>>(
        cloudTopic);
    m_tf = std::make_unique<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>>(
        cloudTopic + "/tf");
}

bool LidarPublisher::hasSubscribers()
{
    return m_cloud && m_cloud->hasSubscribers();
}

void LidarPublisher::publish(int numPoints, const float *xyz, const float *intensity,
                             const float *times, const double *pose)
{
    uint32_t sec = 0, nanosec = 0;
    nowStamp(sec, nanosec);

    if (pose && m_tf->hasSubscribers()) {
        geometry_msgs::msg::dds_::PoseStamped_ msg;
        msg.header().stamp().sec()     = sec;
        msg.header().stamp().nanosec() = nanosec;
        msg.header().frame_id()        = "base";
        msg.pose().position().x()    = pose[0];
        msg.pose().position().y()    = pose[1];
        msg.pose().position().z()    = pose[2];
        msg.pose().orientation().w() = pose[3];
        msg.pose().orientation().x() = pose[4];
        msg.pose().orientation().y() = pose[5];
        msg.pose().orientation().z() = pose[6];
        m_tf->write(msg);
    }

    if (numPoints <= 0 || !m_cloud->hasSubscribers()) return;

    sensor_msgs::msg::dds_::PointCloud2_ msg;
    LidarDds::fillPointCloud2(msg, m_frameId, sec, nanosec,
                              static_cast<uint32_t>(numPoints), xyz, intensity, times);
    m_cloud->write(msg);
}
