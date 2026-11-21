#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

class DepthToPointCloud : public rclcpp::Node {
public:
    DepthToPointCloud() : Node("depth_to_pcl") {
        // Vision sensors are addressed by numeric id (sensor_0..sensor_5); all six publish a
        // depth stream. sensor_0..3 are the body cameras, sensor_4 = front, sensor_5 = rear.
        sensor_names_ = {"sensor_0", "sensor_1", "sensor_2", "sensor_3", "sensor_4", "sensor_5"};

        for (const auto &sensor_name : sensor_names_) {
            // Subscribe to the PNG-compressed depth image (16UC1, millimetres).
            auto depth_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "/rbq/vision/" + sensor_name + "/depth/compressed", 10,
                [this, sensor_name](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                    this->depthCallback(msg, sensor_name);
                });

            // Subscribe to the matching depth intrinsics.
            auto camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/rbq/vision/" + sensor_name + "/depth/camera_info", 10,
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
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> depth_subs_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_pubs_;
    std::map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> camera_info_map_;

    void depthCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, const std::string &sensor_name) {
        if (camera_info_map_.find(sensor_name) == camera_info_map_.end()) {
            RCLCPP_WARN(this->get_logger(), "[%s] Waiting for camera info...", sensor_name.c_str());
            return;
        }

        // Decode the PNG-compressed depth frame. IMREAD_UNCHANGED preserves the 16-bit depth.
        cv::Mat depth_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
        if (depth_image.empty() || depth_image.type() != CV_16UC1) {
            RCLCPP_ERROR(this->get_logger(), "[%s] failed to decode 16UC1 depth (empty=%d, type=%d)",
                         sensor_name.c_str(), depth_image.empty(),
                         depth_image.empty() ? -1 : depth_image.type());
            return;
        }

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
