#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <QtWidgets>
#include <QTimer>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#undef NO_ERROR

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rviz_common/panel.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rbq_msgs/msg/leg_joint_info.hpp"
#include "rbq_msgs/msg/robot_status.hpp"
#include "rbq_msgs/msg/battery_state.hpp"

class Publisher : public rclcpp::Node
{
public:
    explicit Publisher(const std::string &nodeName) : Node(nodeName)
    {
        m_pub_autoStart         = create_publisher<std_msgs::msg::Bool>("rbq/cmd/auto_start",            10);
        m_pub_emergency         = create_publisher<std_msgs::msg::Bool>("rbq/cmd/emergency",            10);
        m_pub_switchControlMode = create_publisher<std_msgs::msg::Bool>("rbq/cmd/switch_control_mode",  10);
        m_pub_switchGait        = create_publisher<std_msgs::msg::Int8>("rbq/cmd/switch_gait",          10);
        m_pub_setPortState      = create_publisher<std_msgs::msg::Int8MultiArray>("rbq/cmd/switch_power", 10);
    }

    void pub_autoStart()                           { publishBool(m_pub_autoStart,         true); }
    void pub_emergency()                           { publishBool(m_pub_emergency,         true); }
    void pub_switchControlMode(const bool &extJoy) { publishBool(m_pub_switchControlMode, extJoy); }
    void pub_switchGait(const int &gait_id)
    {
        std_msgs::msg::Int8 msg; msg.data = gait_id; m_pub_switchGait->publish(msg);
    }
    void pub_setPortState(const int &port_id, const bool &on)
    {
        std_msgs::msg::Int8MultiArray msg;
        msg.data = {static_cast<int8_t>(port_id), static_cast<int8_t>(on ? 1 : 0)};
        m_pub_setPortState->publish(msg);
    }

private:
    static void publishBool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &p, bool v)
    {
        std_msgs::msg::Bool msg; msg.data = v; p->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           m_pub_autoStart;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           m_pub_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           m_pub_switchControlMode;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           m_pub_switchGait;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr m_pub_setPortState;
};

class QPushButton;

namespace rbq_rviz_plugins
{

/// Panel to interface to the RBQ robot
class RbqPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit RbqPanel(QWidget * parent = 0);
    virtual ~RbqPanel();

    /// Load and save configuration data
    void load(const rviz_common::Config & config) override;
    void save(rviz_common::Config config) const override;

private:
    // Toggles HighLevel control mode + recolors the button (called from its lambda).
    void switchControlMode();

    // Status indicators
    QLabel* lbl_battery_voltage;
    QLabel* lbl_motor_check_status;
    QLabel* lbl_initialize_pose_status;
    QLabel* lbl_control_start_status;
    QLabel* lbl_imu_status;
    
    // ROS2 node for subscriptions
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    
    // ROS2 subscriptions
    rclcpp::Subscription<rbq_msgs::msg::RobotStatus>::SharedPtr status_subscription_;
    rclcpp::Subscription<rbq_msgs::msg::LegJointInfo>::SharedPtr leg_joint_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<rbq_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;
    
    // Executor thread function
    void executorThreadFunction();
    
    // Gait button color update function
    void updateGaitButtonColors(int currentGaitId);
    
    // Ext joy button color update function
    void updateExtJoyButtonColor(bool extJoyConnected);
    
    // Tab widget
    QTabWidget* tab_widget_;
    
    // Tab creation functions
    QWidget* createMainTab();
    QWidget* createAdvancedTab();
    
    // Update timer for batch table updates (like GUI)
    QTimer* update_timer_;
    
    // Data storage for batch updates
    struct RobotData {
        rbq_msgs::msg::RobotStatus::SharedPtr robot_status;
        rbq_msgs::msg::LegJointInfo::SharedPtr leg_joint;
        nav_msgs::msg::Odometry::SharedPtr odometry;
        sensor_msgs::msg::Imu::SharedPtr imu;
        rbq_msgs::msg::BatteryState::SharedPtr battery_state;
        
        std::mutex data_mutex;
    } robot_data_;
    
    // HighLevel state variable
    bool highLevel_state_;
    
    // Batch update function
    void batchUpdateTables();
};

}  // namespace rbq_rviz_plugins
