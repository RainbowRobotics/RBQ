#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/idl/ros2/PointCloud2_.hpp>
#include <rbq_sdk/idl/ros2/PoseStamped_.hpp>

// Publishes simulated LiDAR as sensor_msgs/PointCloud2 over DDS, points in the sensor
// frame, with the lidar->base PoseStamped alongside on <cloudTopic>/tf.
class LidarPublisher
{
public:
    // cloudTopic e.g. "rt/rbq/lidar/lidar_front"; frameId is stamped on the cloud.
    LidarPublisher(const std::string &cloudTopic, const std::string &frameId);

    // Gates the raytracing so an idle sim does no LiDAR work.
    bool hasSubscribers();

    // xyz is 3*numPoints in the sensor frame; times are per-point offsets from the stamp,
    // for deskew. pose (optional): lidar->base as {px,py,pz, qw,qx,qy,qz}; null skips the TF.
    void publish(int numPoints, const float *xyz, const float *intensity,
                 const float *times, const double *pose = nullptr);

private:
    std::string m_frameId;
    std::unique_ptr<rbq_sdk::Publisher<sensor_msgs::msg::dds_::PointCloud2_>> m_cloud;
    std::unique_ptr<rbq_sdk::Publisher<geometry_msgs::msg::dds_::PoseStamped_>> m_tf;
};
