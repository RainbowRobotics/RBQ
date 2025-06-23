#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

class DepthToPointCloud : public rclcpp::Node {
public:
    DepthToPointCloud() : Node("depth_to_pcl") {
        // Define sensor names
        sensor_names_ = {"sensor_front", "sensor_rear", "sensor_left", "sensor_right", "sensor_bottom_0", "sensor_bottom_1", "sensor_bottom_2", "sensor_bottom_3"};

        for (const auto &sensor_name : sensor_names_) {
            // Subscribe to depth image
            auto depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "/rbq/vision/" + sensor_name + "/depth", 10,
                [this, sensor_name](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->depthCallback(msg, sensor_name);
                });

            // Subscribe to camera info
            auto camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/rbq/vision/" + sensor_name + "/camera_info", 10,
                [this, sensor_name](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                    camera_info_map_[sensor_name] = msg;
                });

            // Publish point cloud for this sensor
            auto pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/rbq/vision/" + sensor_name + "/pointcloud", 10);

            depth_subs_[sensor_name] = depth_sub;
            camera_info_subs_[sensor_name] = camera_info_sub;
            pcl_pubs_[sensor_name] = pcl_pub;
        }
    }

private:
    std::vector<std::string> sensor_names_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> depth_subs_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_pubs_;
    std::map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> camera_info_map_;

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string &sensor_name) {
        if (camera_info_map_.find(sensor_name) == camera_info_map_.end()) {
            RCLCPP_WARN(this->get_logger(), "[%s] Waiting for camera info...", sensor_name.c_str());
            return;
        }

        // Convert ROS2 Image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "[%s] cv_bridge exception: %s", sensor_name.c_str(), e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;
        auto camera_info = camera_info_map_[sensor_name];

        double fx = camera_info->k[0];
        double fy = camera_info->k[4];
        double cx = camera_info->k[2];
        double cy = camera_info->k[5];

        int width = depth_image.cols;
        int height = depth_image.rows;

        // Create a new point cloud message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = msg->header;
        cloud_msg.height = 1;
        cloud_msg.width = width * height;
        cloud_msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                uint16_t depth = depth_image.at<uint16_t>(v, u);

                if (depth > 0) {
                    float z = depth * 0.001f;
                    float x = (u - cx) * z / fx;
                    float y = (v - cy) * z / fy;

                    *iter_x = x;
                    *iter_y = y;
                    *iter_z = z;
                } else {
                    *iter_x = std::numeric_limits<float>::quiet_NaN();
                    *iter_y = std::numeric_limits<float>::quiet_NaN();
                    *iter_z = std::numeric_limits<float>::quiet_NaN();
                }

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }

        pcl_pubs_[sensor_name]->publish(cloud_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPointCloud>());
    rclcpp::shutdown();
    return 0;
}

