#include "rbq_panel.hpp"

#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include <memory>
#include <vector>
#include <utility>
#include <chrono>
#include <string>

#include "rviz_common/display_context.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

std::shared_ptr<Publisher> publisher;

using namespace std::chrono_literals;

namespace rbq_rviz_plugins
{

RbqPanel::RbqPanel(QWidget * parent) : Panel(parent)
{
    // ExtJoy state init (OFF)
    highLevel_state_ = false;
    
    // Create the control button and its tooltip

    publisher = std::make_shared<Publisher>("rbq_rviz_panel");

    // Status indicators - NO CIRCLE SYMBOL
    lbl_motor_check_status = new QLabel("");
    lbl_initialize_pose_status = new QLabel("");
    lbl_imu_status = new QLabel("");
    lbl_control_start_status = new QLabel("");
    lbl_battery_voltage = new QLabel("");
    
    // Status indicator styling - RECTANGULAR INDICATORS
    lbl_motor_check_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_initialize_pose_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_imu_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_control_start_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
    lbl_battery_voltage->setStyleSheet("QLabel { background-color: white; border: 1px solid #ccc; padding: 4px; min-width: 20px; min-height: 20px; }");

    // Create ROS2 node for subscriptions - SAFER IMPLEMENTATION
    try {
        ros_node_ = rclcpp::Node::make_shared("rbq_panel_node_" + std::to_string(getpid()));
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(ros_node_);
        
        // ROS2 subscriptions -- each lambda stashes the latest message under the data
        // mutex; the UI is refreshed from these in batchUpdateTables().
        status_subscription_ = ros_node_->create_subscription<rbq_msgs::msg::RobotStatus>(
            "/rbq/robot_status", 10,
            [this](const rbq_msgs::msg::RobotStatus::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
                robot_data_.robot_status = msg;
            });

        battery_state_subscription_ = ros_node_->create_subscription<rbq_msgs::msg::BatteryState>(
            "/rbq/battery", 10,
            [this](const rbq_msgs::msg::BatteryState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
                robot_data_.battery_state = msg;
            });

        leg_joint_subscription_ = ros_node_->create_subscription<rbq_msgs::msg::LegJointInfo>(
            "/rbq/leg_joint", 10,
            [this](const rbq_msgs::msg::LegJointInfo::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
                robot_data_.leg_joint = msg;
            });

        odometry_subscription_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
            "/rbq/odometry", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
                robot_data_.odometry = msg;
            });

        imu_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
            "/rbq/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
                robot_data_.imu = msg;
            });

        // Start executor in separate thread
        executor_thread_ = std::thread(&RbqPanel::executorThreadFunction, this);
        
        RCLCPP_INFO(ros_node_->get_logger(), "ROS2 subscriptions initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rbq_panel"), "Failed to initialize ROS2 subscriptions: %s", e.what());
        // Continue without subscriptions
    }

    // Create tab widget
    tab_widget_ = new QTabWidget();
    
    // Create tabs
    QWidget* main_tab = createMainTab();
    QWidget* advanced_tab = createAdvancedTab();
    
    // Add tabs to tab widget
    tab_widget_->addTab(main_tab, "Main Control");
    tab_widget_->addTab(advanced_tab, "State");
    
    // Initialize update timer for batch table updates (like GUI)
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50); // 50ms like GUI
    connect(update_timer_, &QTimer::timeout, this, &RbqPanel::batchUpdateTables);
    update_timer_->start();
    
    QVBoxLayout * lyt_buttons = new QVBoxLayout;
    lyt_buttons->addWidget(tab_widget_);

    

    // Lay out the items in the panel
    QVBoxLayout * main_layout = new QVBoxLayout;

    main_layout->insertLayout(0, lyt_buttons);

    main_layout->setContentsMargins(10, 10, 10, 10);
    setLayout(main_layout);

}

RbqPanel::~RbqPanel()
{
    // Stop executor and join thread - SAFER IMPLEMENTATION
    try {
        if (executor_) {
            executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    } catch (const std::exception& e) {
        // Log error but don't crash
        std::cerr << "Error in RbqPanel destructor: " << e.what() << std::endl;
    }
}

void RbqPanel::switchControlMode()
{
    highLevel_state_ = !highLevel_state_;
    publisher->pub_switchControlMode(highLevel_state_);
    updateExtJoyButtonColor(highLevel_state_);  // reflect the new state on the button
}

// Status callback functions - THREAD SAFE IMPLEMENTATION
void RbqPanel::executorThreadFunction()
{
    executor_->spin();
}

void RbqPanel::batchUpdateTables()
{
    // Lock data for reading
    std::lock_guard<std::mutex> lock(robot_data_.data_mutex);
    
    // Update status indicators if robot_status is available
    if (robot_data_.robot_status) {
        auto msg = robot_data_.robot_status;
        if (lbl_motor_check_status || lbl_initialize_pose_status || 
            lbl_control_start_status || lbl_imu_status) {
            
            // Update status indicators based on robot status - RECTANGULAR STYLE
            if (lbl_motor_check_status) {
                if (msg->can_check) {
                    lbl_motor_check_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_motor_check_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            if (lbl_initialize_pose_status) {
                if (msg->find_home) {
                    lbl_initialize_pose_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_initialize_pose_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            if (lbl_control_start_status) {
                if (msg->con_start) {
                    lbl_control_start_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_control_start_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            if (lbl_imu_status) {
                if (msg->imu_success) {
                    lbl_imu_status->setStyleSheet("QLabel { background-color: green; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                } else {
                    lbl_imu_status->setStyleSheet("QLabel { background-color: red; color: white; font-size: 16px; font-weight: bold; border: 1px solid #ccc; padding: 8px; min-width: 20px; min-height: 20px; }");
                }
            }
            
            // Update gait button colors based on current gait
            updateGaitButtonColors(msg->gait_id);
            
            // Update ext joy button color based on ext_joy status
            updateExtJoyButtonColor(msg->ext_joy);
        }
    }
    
    // Update battery voltage if battery_state is available
    if (robot_data_.battery_state && lbl_battery_voltage) {
        auto msg = robot_data_.battery_state;
        float voltage = msg->voltage.at(0);
        uint8_t percentage = msg->charge_percentage.at(0);
        QString voltage_text = QString::number(voltage, 'f', 1) + " V" + "\n" + QString::number(percentage) + " %";
        lbl_battery_voltage->setText(voltage_text);
    }
    
    // Update joint table from LegJointInfo if available
    if (robot_data_.leg_joint && tab_widget_) {
        auto msg = robot_data_.leg_joint;
        QWidget* advanced_tab = tab_widget_->widget(1); // State tab
        if (advanced_tab) {
            QGroupBox* joint_group = advanced_tab->findChild<QGroupBox*>("Joint States");
            if (joint_group) {
                QTableWidget* joint_table = joint_group->findChild<QTableWidget*>();
                if (joint_table) {
                    // disable table updates (for batch update)
                    joint_table->setUpdatesEnabled(false);

                    const QColor okColor(200, 255, 200);   // light green
                    const QColor badColor(255, 200, 200);  // light red

                    // extract data from LegJointInfo.joint[] and update the table
                    for (size_t i = 0; i < msg->joint.size(); ++i) {
                        const auto& j = msg->joint[i];

                        // Row label = joint name from the message (e.g. "HRR")
                        QTableWidgetItem* vh = joint_table->verticalHeaderItem(i);
                        if (!vh) {
                            vh = new QTableWidgetItem();
                            joint_table->setVerticalHeaderItem(i, vh);
                        }
                        vh->setText(QString::fromStdString(j.name));

                        // Status (NON / CON / HOM / RUN)
                        QTableWidgetItem* status_item = joint_table->item(i, 0);
                        if (status_item) {
                            status_item->setText(QString::fromStdString(j.status));
                            const bool offline = (j.status == "NON" || j.status.empty());
                            status_item->setBackground(offline ? badColor : okColor);
                        }

                        // Error (3-letter code; "OK" = normal)
                        QTableWidgetItem* error_item = joint_table->item(i, 1);
                        if (error_item) {
                            error_item->setText(QString::fromStdString(j.error_msg));
                            const bool faulted = !(j.error_msg == "OK" || j.error_msg.empty());
                            error_item->setBackground(faulted ? badColor : okColor);
                        }

                        // Temperature (board / coil)
                        QTableWidgetItem* temp_item = joint_table->item(i, 2);
                        if (temp_item) {
                            temp_item->setText(QString::number(static_cast<int>(j.temperature_board)) + "/" +
                                               QString::number(static_cast<int>(j.temperature_coil)));
                        }

                        // Angle Ref
                        QTableWidgetItem* angle_ref_item = joint_table->item(i, 3);
                        if (angle_ref_item) angle_ref_item->setText(QString::number(j.ref_position, 'f', 3));

                        // Angle Enc
                        QTableWidgetItem* angle_enc_item = joint_table->item(i, 4);
                        if (angle_enc_item) angle_enc_item->setText(QString::number(j.pos, 'f', 3));

                        // Torque Ref
                        QTableWidgetItem* torque_ref_item = joint_table->item(i, 5);
                        if (torque_ref_item) torque_ref_item->setText(QString::number(j.ref_ff_torque, 'f', 3));

                        // Current Sen
                        QTableWidgetItem* current_item = joint_table->item(i, 6);
                        if (current_item) current_item->setText(QString::number(j.current, 'f', 3));

                        // KP
                        QTableWidgetItem* kp_item = joint_table->item(i, 7);
                        if (kp_item) kp_item->setText(QString::number(j.kp, 'f', 3));

                        // KD
                        QTableWidgetItem* kd_item = joint_table->item(i, 8);
                        if (kd_item) kd_item->setText(QString::number(j.kd, 'f', 3));

                        // Owner
                        QTableWidgetItem* owner_item = joint_table->item(i, 9);
                        if (owner_item) owner_item->setText(QString::number(static_cast<int>(j.owner)));
                    }

                    // re-enable table updates and repaint at once
                    joint_table->setUpdatesEnabled(true);
                    joint_table->update(); // update the whole table at once
                }
            }
        }
    }
    
    // Update odometry tables if data is available
    if (robot_data_.odometry && tab_widget_) {
        auto msg = robot_data_.odometry;
        QWidget* advanced_tab = tab_widget_->widget(1); // State tab
        if (advanced_tab) {
            // find position_table and orientation_table in the Robot Pose GroupBox
            QGroupBox* robot_pose_group = advanced_tab->findChild<QGroupBox*>("Robot Pose");
            if (robot_pose_group) {
                QList<QTableWidget*> pose_tables = robot_pose_group->findChildren<QTableWidget*>();
                if (pose_tables.size() >= 2) {
                    QTableWidget* position_table = pose_tables[0]; // first table is position_table
                    QTableWidget* orientation_table = pose_tables[1]; // second table is orientation_table
                    
                    // disable table updates (for batch update)
                    position_table->setUpdatesEnabled(false);
                    orientation_table->setUpdatesEnabled(false);
                    
                    // extract data from the odometry message and update the table
                    // Position (X, Y, Z)
                    QTableWidgetItem* px_item = position_table->item(0, 0);
                    if (px_item) {
                        px_item->setText(QString::number(msg->pose.pose.position.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* py_item = position_table->item(0, 1);
                    if (py_item) {
                        py_item->setText(QString::number(msg->pose.pose.position.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* pz_item = position_table->item(0, 2);
                    if (pz_item) {
                        pz_item->setText(QString::number(msg->pose.pose.position.z, 'f', 3));
                    }
                    
                    // Orientation (Quaternion X, Y, Z, W)
                    QTableWidgetItem* ox_item = orientation_table->item(0, 0);
                    if (ox_item) {
                        ox_item->setText(QString::number(msg->pose.pose.orientation.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* oy_item = orientation_table->item(0, 1);
                    if (oy_item) {
                        oy_item->setText(QString::number(msg->pose.pose.orientation.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* oz_item = orientation_table->item(0, 2);
                    if (oz_item) {
                        oz_item->setText(QString::number(msg->pose.pose.orientation.z, 'f', 3));
                    }
                    
                    QTableWidgetItem* ow_item = orientation_table->item(0, 3);
                    if (ow_item) {
                        ow_item->setText(QString::number(msg->pose.pose.orientation.w, 'f', 3));
                    }
                    
                    // re-enable table updates and repaint at once
                    position_table->setUpdatesEnabled(true);
                    orientation_table->setUpdatesEnabled(true);
                    position_table->update();
                    orientation_table->update();
                }
            }
        }
    }
    
    // Update IMU tables if data is available
    if (robot_data_.imu && tab_widget_) {
        auto msg = robot_data_.imu;
        QWidget* advanced_tab = tab_widget_->widget(1); // State tab
        if (advanced_tab) {
            // find imu_table in the IMU Data GroupBox
            QGroupBox* imu_group = advanced_tab->findChild<QGroupBox*>("IMU Data");
            if (imu_group) {
                QTableWidget* imu_table = imu_group->findChild<QTableWidget*>();
                if (imu_table) {
                    // disable table updates (for batch update)
                    imu_table->setUpdatesEnabled(false);
                    
                    // extract data from the IMU message and update the table
                    // Orientation (convert Quaternion to Euler angles)
                    // Roll (X-axis)
                    QTableWidgetItem* roll_angle_item = imu_table->item(0, 0);
                    if (roll_angle_item) {
                        // compute Roll from Quaternion (simple approximation)
                        double roll = atan2(2.0 * (msg->orientation.w * msg->orientation.x + msg->orientation.y * msg->orientation.z),
                                           1.0 - 2.0 * (msg->orientation.x * msg->orientation.x + msg->orientation.y * msg->orientation.y));
                        roll_angle_item->setText(QString::number(roll, 'f', 3));
                    }
                    
                    // Pitch (Y-axis)
                    QTableWidgetItem* pitch_angle_item = imu_table->item(1, 0);
                    if (pitch_angle_item) {
                        double pitch = asin(2.0 * (msg->orientation.w * msg->orientation.y - msg->orientation.z * msg->orientation.x));
                        pitch_angle_item->setText(QString::number(pitch, 'f', 3));
                    }
                    
                    // Yaw (Z-axis)
                    QTableWidgetItem* yaw_angle_item = imu_table->item(2, 0);
                    if (yaw_angle_item) {
                        double yaw = atan2(2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y),
                                          1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z));
                        yaw_angle_item->setText(QString::number(yaw, 'f', 3));
                    }
                    
                    // Angular velocity (Gyro)
                    QTableWidgetItem* roll_gyro_item = imu_table->item(0, 1);
                    if (roll_gyro_item) {
                        roll_gyro_item->setText(QString::number(msg->angular_velocity.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* pitch_gyro_item = imu_table->item(1, 1);
                    if (pitch_gyro_item) {
                        pitch_gyro_item->setText(QString::number(msg->angular_velocity.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* yaw_gyro_item = imu_table->item(2, 1);
                    if (yaw_gyro_item) {
                        yaw_gyro_item->setText(QString::number(msg->angular_velocity.z, 'f', 3));
                    }
                    
                    // Linear acceleration
                    QTableWidgetItem* roll_acc_item = imu_table->item(0, 2);
                    if (roll_acc_item) {
                        roll_acc_item->setText(QString::number(msg->linear_acceleration.x, 'f', 3));
                    }
                    
                    QTableWidgetItem* pitch_acc_item = imu_table->item(1, 2);
                    if (pitch_acc_item) {
                        pitch_acc_item->setText(QString::number(msg->linear_acceleration.y, 'f', 3));
                    }
                    
                    QTableWidgetItem* yaw_acc_item = imu_table->item(2, 2);
                    if (yaw_acc_item) {
                        yaw_acc_item->setText(QString::number(msg->linear_acceleration.z, 'f', 3));
                    }
                    
                    // re-enable table updates and repaint at once
                    imu_table->setUpdatesEnabled(true);
                    imu_table->update();
                }
            }
        }
    }
}

void RbqPanel::updateGaitButtonColors(int currentGaitId)
{
    static const QString defaultStyle = "QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }";
    static const QString activeStyle  = "QPushButton { background-color: #90EE90; color: #2d5016; font-weight: bold; border: 2px solid #7CCD7C; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #7CCD7C; } QPushButton:pressed { background-color: #6BB86B; }";

    // Gait buttons are tagged with a "gaitId" property in createMainTab().
    for (QPushButton* bt : tab_widget_->findChildren<QPushButton*>()) {
        const QVariant gaitId = bt->property("gaitId");
        if (!gaitId.isValid()) continue;
        bt->setStyleSheet(gaitId.toInt() == currentGaitId ? activeStyle : defaultStyle);
    }
}

void RbqPanel::updateExtJoyButtonColor(bool extJoyConnected)
{
    highLevel_state_ = extJoyConnected;
    QPushButton* bt = tab_widget_->findChild<QPushButton*>("controlModeButton");
    if (!bt) return;
    static const QString green = "QPushButton { background-color: #90EE90; color: #2d5016; font-weight: bold; border: 2px solid #7CCD7C; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #7CCD7C; } QPushButton:pressed { background-color: #6BB86B; }";
    static const QString gray  = "QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }";
    bt->setChecked(extJoyConnected);
    bt->setStyleSheet(extJoyConnected ? green : gray);
}

QWidget* RbqPanel::createMainTab()
{
    QWidget* main_tab = new QWidget();
    QVBoxLayout* main_layout = new QVBoxLayout(main_tab);

    const QString defaultButtonStyle = "QPushButton { background-color: #f0f0f0; color: #333333; font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #e0e0e0; } QPushButton:pressed { background-color: #d0d0d0; }";

    // Gait button: tagged with its gait id so updateGaitButtonColors() can find it.
    auto makeGaitButton = [&](const QString& text, int gaitId) -> QPushButton* {
        auto* bt = new QPushButton(text);
        bt->setStyleSheet(defaultButtonStyle);
        bt->setProperty("gaitId", gaitId);
        QObject::connect(bt, &QPushButton::clicked, [=]() { publisher->pub_switchGait(gaitId); });
        return bt;
    };
    // Power button: checkable toggle; publishes a port on/off state on click.
    auto makePowerButton = [&](const QString& text, int port) -> QPushButton* {
        auto* bt = new QPushButton(text);
        bt->setCheckable(true);
        QObject::connect(bt, &QPushButton::clicked, [=](bool on) { publisher->pub_setPortState(port, on); });
        return bt;
    };

    auto* bt_emergency = new QPushButton("Emergency");
    bt_emergency->setStyleSheet("QPushButton { background-color: #ff4444; color: white; font-weight: bold; font-size: 11px; border: 2px solid #cc0000; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #ff6666; } QPushButton:pressed { background-color: #cc0000; }");
    QObject::connect(bt_emergency, &QPushButton::clicked, [=](){ publisher->pub_emergency(); });
    QHBoxLayout* lyt_emergency = new QHBoxLayout;
    lyt_emergency->addWidget(bt_emergency);
    main_layout->addLayout(lyt_emergency);

    // Status Display Section
    QGroupBox* grp_status = new QGroupBox("Status");
    grp_status->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QHBoxLayout* lyt_status = new QHBoxLayout;
    
    // Motor Check - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_motor_check = new QVBoxLayout;
    QLabel* lbl_motor_check_text = new QLabel("Motor");
    lbl_motor_check_text->setAlignment(Qt::AlignCenter);
    lyt_motor_check->addWidget(lbl_motor_check_text);
    lyt_motor_check->addWidget(lbl_motor_check_status);
    lyt_status->addLayout(lyt_motor_check);
    
    // Initialize Pose - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_init_pose = new QVBoxLayout;
    QLabel* lbl_init_pose_text = new QLabel("Pose");
    lbl_init_pose_text->setAlignment(Qt::AlignCenter);
    lyt_init_pose->addWidget(lbl_init_pose_text);
    lyt_init_pose->addWidget(lbl_initialize_pose_status);
    lyt_status->addLayout(lyt_init_pose);
    
    // IMU - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_imu = new QVBoxLayout;
    QLabel* lbl_imu_text = new QLabel("IMU");
    lbl_imu_text->setAlignment(Qt::AlignCenter);
    lyt_imu->addWidget(lbl_imu_text);
    lyt_imu->addWidget(lbl_imu_status);
    lyt_status->addLayout(lyt_imu);
    
    // Control Start - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_control_start = new QVBoxLayout;
    QLabel* lbl_control_start_text = new QLabel("Control");
    lbl_control_start_text->setAlignment(Qt::AlignCenter);
    lyt_control_start->addWidget(lbl_control_start_text);
    lyt_control_start->addWidget(lbl_control_start_status);
    lyt_status->addLayout(lyt_control_start);
    
    // Battery - TEXT FIRST, INDICATOR BELOW
    QVBoxLayout* lyt_battery = new QVBoxLayout;
    QLabel* lbl_battery_text = new QLabel("Battery");
    lbl_battery_text->setAlignment(Qt::AlignCenter);
    lyt_battery->addWidget(lbl_battery_text);
    lyt_battery->addWidget(lbl_battery_voltage);
    lyt_status->addLayout(lyt_battery);
    
    grp_status->setLayout(lyt_status);
    main_layout->addWidget(grp_status);

    // Initialize group box
    QGroupBox* grp_initialize = new QGroupBox("Initialize");
    grp_initialize->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QHBoxLayout* lyt_initialize = new QHBoxLayout;
    auto* bt_autoStart = new QPushButton("Auto start");
    bt_autoStart->setStyleSheet(defaultButtonStyle);
    QObject::connect(bt_autoStart, &QPushButton::clicked, [=]() { publisher->pub_autoStart(); });
    lyt_initialize->addWidget(bt_autoStart);

    auto* bt_controlMode = new QPushButton("HighLevel\nControl");
    bt_controlMode->setObjectName("controlModeButton");   // found by switchControlMode()/updateExtJoyButtonColor()
    bt_controlMode->setCheckable(true);
    QObject::connect(bt_controlMode, &QPushButton::clicked, [=]() { switchControlMode(); });
    lyt_initialize->addWidget(bt_controlMode);
    grp_initialize->setLayout(lyt_initialize);
    main_layout->addWidget(grp_initialize);

    // Gait Select group box
    QGroupBox* grp_gait = new QGroupBox("Gait Select");
    grp_gait->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QVBoxLayout* lyt_gait = new QVBoxLayout;
    
    // first row: basic gait buttons
    QHBoxLayout* lyt_gait_row1 = new QHBoxLayout;
    lyt_gait_row1->addWidget(makeGaitButton("Sit",    0));
    lyt_gait_row1->addWidget(makeGaitButton("Stand",  1));
    lyt_gait_row1->addWidget(makeGaitButton("Walk",   3));
    lyt_gait_row1->addWidget(makeGaitButton("Wave",   5));
    lyt_gait_row1->addWidget(makeGaitButton("Run",    6));
    lyt_gait_row1->addWidget(makeGaitButton("Stairs", 4));
    lyt_gait_row1->addWidget(makeGaitButton("Dock",   10));
    lyt_gait->addLayout(lyt_gait_row1);
    
    QHBoxLayout* lyt_gait_row2 = new QHBoxLayout;
    lyt_gait_row2->addWidget(makeGaitButton("RL\nTrot",  30));
    lyt_gait_row2->addWidget(makeGaitButton("RL\nBound", 35));
    lyt_gait_row2->addWidget(makeGaitButton("RL\nPace",  36));
    lyt_gait_row2->addWidget(makeGaitButton("RL\nPronk", 37));
    lyt_gait->addLayout(lyt_gait_row2);
    
    // third row: additional RL gait buttons
    QHBoxLayout* lyt_gait_row3 = new QHBoxLayout;
    lyt_gait_row3->addWidget(makeGaitButton("RL\nTrot Vision", 42));
    lyt_gait_row3->addWidget(makeGaitButton("RL\nTrot Run",    45));
    lyt_gait_row3->addWidget(makeGaitButton("RL\nSilent",      46));
    lyt_gait_row3->addWidget(makeGaitButton("RL\nFront Walk",  31));
    lyt_gait_row3->addWidget(makeGaitButton("RL\nStairs",      47));
    lyt_gait->addLayout(lyt_gait_row3);
    
    grp_gait->setLayout(lyt_gait);
    main_layout->addWidget(grp_gait);

    // Power group box - 3x3 grid + 1 button at bottom
    QGroupBox* grp_power = new QGroupBox("Power");
    grp_power->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #cccccc; border-radius: 5px; margin-top: 10px; padding-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; }");
    QVBoxLayout* lyt_power = new QVBoxLayout;
    
    // first row: Power Leg, Power Arm, Power Ext52V
    QHBoxLayout* lyt_power_row1 = new QHBoxLayout;
    lyt_power_row1->addWidget(makePowerButton("Leg",    0x00));
    lyt_power_row1->addWidget(makePowerButton("Arm",    0x01));
    lyt_power_row1->addWidget(makePowerButton("Ext52V", 0x02));
    lyt_power->addLayout(lyt_power_row1);
    
    // second row: Power VisionPC, Power UsbHub, Power IrLEDs
    QHBoxLayout* lyt_power_row2 = new QHBoxLayout;
    lyt_power_row2->addWidget(makePowerButton("VisionPC", 0x10));
    lyt_power_row2->addWidget(makePowerButton("UsbHub",   0x21));
    lyt_power_row2->addWidget(makePowerButton("IrLEDs",   0x15));
    lyt_power->addLayout(lyt_power_row2);
    
    // third row: Power Cctv, Power Thermal, Power Lidar
    QHBoxLayout* lyt_power_row3 = new QHBoxLayout;
    lyt_power_row3->addWidget(makePowerButton("Cctv",    0x13));
    lyt_power_row3->addWidget(makePowerButton("Thermal", 0x14));
    lyt_power_row3->addWidget(makePowerButton("Lidar",   0x12));
    lyt_power->addLayout(lyt_power_row3);
    
    // fourth row: Power Comm (full width)
    QHBoxLayout* lyt_power_row4 = new QHBoxLayout;
    auto* bt_powerComm = makePowerButton("Comm", 0x11);
    bt_powerComm->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    lyt_power_row4->addWidget(bt_powerComm);
    lyt_power->addLayout(lyt_power_row4);

    grp_power->setLayout(lyt_power);
    main_layout->addWidget(grp_power);

    // Push all groups to the top: absorbs extra vertical space when the panel is tall.
    main_layout->addStretch(1);

    return main_tab;
}

QWidget* RbqPanel::createAdvancedTab()
{
    QWidget* advanced_tab = new QWidget();
    QVBoxLayout* advanced_layout = new QVBoxLayout(advanced_tab);
    
    // create Joint Table
    QTableWidget* joint_table = new QTableWidget(12, 10);
    joint_table->setHorizontalHeaderLabels({
        "Status", "Error", "Temp", "Angle Ref", "Angle Enc",
        "Torque Ref", "Current Sen", "KP (Nm/rad)", "KD (Nm/rad/s)", "Owner (id)"
    });
    // Vertical headers (joint names) are filled from LegJointInfo.joint[].name at runtime.

    // table setup
    joint_table->setAlternatingRowColors(true);
    joint_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    joint_table->setSelectionMode(QAbstractItemView::SingleSelection);
    joint_table->setSortingEnabled(false);
    
    // header setup
    joint_table->horizontalHeader()->setStretchLastSection(true);
    joint_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    joint_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // auto-resize row height
    joint_table->resizeRowsToContents();
    
    // table size - auto-adjust to contents
    joint_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // initial data setup
    for (int row = 0; row < 12; ++row) {
        for (int col = 0; col < 10; ++col) {
            QTableWidgetItem* item = new QTableWidgetItem();
            
            switch (col) {
                case 0: // Status
                    item->setText("N/-/-/0");
                    item->setBackground(QColor(255, 200, 200)); // light red
                    break;
                case 1: // Error
                    item->setText("0/0");
                    break;
                case 2: // Temp
                    item->setText("0/0");
                    break;
                case 3: // Angle Ref
                    item->setText("0.000");
                    break;
                case 4: // Angle Enc
                    item->setText("0.000");
                    break;
                case 5: // Torque Ref
                    item->setText("0.000");
                    break;
                case 6: // Current Sen
                    item->setText("0.000");
                    break;
                case 7: // KP
                    item->setText("0.000");
                    break;
                case 8: // KD
                    item->setText("nan");
                    break;
                case 9: // Owner
                    item->setText("-1");
                    break;
            }
            
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable); // read-only
            joint_table->setItem(row, col, item);
        }
    }
    
    // style setup
    joint_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 1px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // wrap the Joint Table in a GroupBox
    QGroupBox* joint_group = new QGroupBox("Joint States");
    joint_group->setObjectName("Joint States");
    QVBoxLayout* joint_group_layout = new QVBoxLayout(joint_group);
    joint_group_layout->addWidget(joint_table);
    advanced_layout->addWidget(joint_group);
    
    // create Position Table
    QTableWidget* position_table = new QTableWidget(1, 3);
    position_table->setHorizontalHeaderLabels({
        "X", "Y", "Z"
    });
    
    // row name setup
    QStringList position_names = {"Position"};
    position_table->setVerticalHeaderLabels(position_names);
    
    // create Orientation Table
    QTableWidget* orientation_table = new QTableWidget(1, 4);
    orientation_table->setHorizontalHeaderLabels({
        "X", "Y", "Z", "W"
    });
    
    // row name setup
    QStringList orientation_names = {"Orientation"};
    orientation_table->setVerticalHeaderLabels(orientation_names);
    
    // Position table setup
    position_table->setAlternatingRowColors(true);
    position_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    position_table->setSelectionMode(QAbstractItemView::SingleSelection);
    position_table->setSortingEnabled(false);
    
    // Position header setup
    position_table->horizontalHeader()->setStretchLastSection(false);
    position_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    position_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // Position auto-resize row height
    position_table->resizeRowsToContents();
    
    // Position table size
    position_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // Position initial data setup
    for (int col = 0; col < 3; ++col) {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setText("0.000");
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable); // read-only
        position_table->setItem(0, col, item);
    }
    
    // Orientation table setup
    orientation_table->setAlternatingRowColors(true);
    orientation_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    orientation_table->setSelectionMode(QAbstractItemView::SingleSelection);
    orientation_table->setSortingEnabled(false);
    
    // Orientation header setup
    orientation_table->horizontalHeader()->setStretchLastSection(false);
    orientation_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    orientation_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // Orientation auto-resize row height
    orientation_table->resizeRowsToContents();
    
    // Orientation table size
    orientation_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // Orientation initial data setup
    for (int col = 0; col < 4; ++col) {
        QTableWidgetItem* item = new QTableWidgetItem();
        item->setText("0.000");
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable); // read-only
        orientation_table->setItem(0, col, item);
    }
    
    // Position style setup
    position_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 1px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // Orientation style setup
    orientation_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 1px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // table size
    position_table->setMinimumHeight(60);
    position_table->setMaximumHeight(80);
    orientation_table->setMinimumHeight(60);
    orientation_table->setMaximumHeight(80);
    
    // create Robot Pose GroupBox (contains Position and Orientation)
    QGroupBox* robot_pose_group = new QGroupBox("Robot Pose");
    robot_pose_group->setObjectName("Robot Pose");
    QVBoxLayout* robot_pose_group_layout = new QVBoxLayout(robot_pose_group);
    
    // arrange Position and Orientation vertically
    robot_pose_group_layout->addWidget(position_table);
    robot_pose_group_layout->addWidget(orientation_table);
    
    advanced_layout->addWidget(robot_pose_group);
    
    // create IMU Table
    QTableWidget* imu_table = new QTableWidget(3, 3);
    imu_table->setHorizontalHeaderLabels({
        "angle", "gyro", "acc"
    });
    
    // row name setup
    QStringList imu_names = {"R(X)", "P(Y)", "Y(Z)"};
    imu_table->setVerticalHeaderLabels(imu_names);
    
    // table setup
    imu_table->setAlternatingRowColors(true);
    imu_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    imu_table->setSelectionMode(QAbstractItemView::SingleSelection);
    imu_table->setSortingEnabled(false);
    
    // header setup
    imu_table->horizontalHeader()->setStretchLastSection(false);
    imu_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    imu_table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // auto-resize row height
    imu_table->resizeRowsToContents();
    
    // table size - auto-adjust to contents
    imu_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    
    // initial data setup
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            QTableWidgetItem* item = new QTableWidgetItem();
            item->setText("0.000");
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable); // read-only
            imu_table->setItem(row, col, item);
        }
    }
    
    // style setup
    imu_table->setStyleSheet(
        "QTableWidget {"
        "    gridline-color: #d0d0d0;"
        "    background-color: white;"
        "    alternate-background-color: #f5f5f5;"
        "    selection-background-color: #3daee9;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 10px;"
        "}"
        "QTableWidget::item {"
        "    padding: 2px;"
        "    border: none;"
        "}"
        "QHeaderView::section {"
        "    background-color: #e0e0e0;"
        "    padding: 1px;"
        "    border: 1px solid #d0d0d0;"
        "    font-weight: bold;"
        "}"
    );
    
    // table size
    imu_table->setMinimumHeight(120);
    imu_table->setMaximumHeight(160);
    
    // wrap the IMU Table in a GroupBox
    QGroupBox* imu_group = new QGroupBox("IMU Data");
    imu_group->setObjectName("IMU Data");
    QVBoxLayout* imu_group_layout = new QVBoxLayout(imu_group);
    imu_group_layout->addWidget(imu_table);
    advanced_layout->addWidget(imu_group);
    
    return advanced_tab;
}

void
RbqPanel::save(rviz_common::Config config) const
{
    Panel::save(config);
}

void
RbqPanel::load(const rviz_common::Config & config)
{
    Panel::load(config);
}

}  // namespace rbq_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rbq_rviz_plugins::RbqPanel, rviz_common::Panel)
