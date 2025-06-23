#ifndef RBQ_RVIZ_PLUGINS__NAV2_PANEL_HPP_
#define RBQ_RVIZ_PLUGINS__NAV2_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

class QPushButton;

namespace rbq_rviz_plugins
{

/// Panel to interface to the nav2 stack
class RbqPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit RbqPanel(QWidget * parent = 0);
    virtual ~RbqPanel();

    void onInitialize() override;

    /// Load and save configuration data
    void load(const rviz_common::Config & config) override;
    void save(rviz_common::Config config) const override;

private Q_SLOTS:
    void startThread();
    void onStartup();
    void onShutdown();
    void onCancel();
    void onPause();
    void onResume();
    void onAccumulatedWp();
    void onAccumulatedNTP();
    void onAccumulating();
    void onNewGoal(double x, double y, double theta, QString frame);

    // void onAutoStart       ();
    // void onStand           ();
    // void onSit             ();

    void autoStart         ();
    void canCheck          ();
    void findHome          ();
    void sit               ();
    void stand             ();
    void walk              ();
    void walkSlow          ();
    void run               ();
    void calibrateImu      ();
    void staticLock        ();
    void staticReady       ();
    void staticGround      ();
    void recoveryErrorClear();
    void recoveryFlex      ();
    void emergency         ();

    void switchGamepadPort (const bool &powerON = false);
    void powerLeg          (const bool &powerON = false);
    void powerArm          (const bool &powerON = false);
    void powerVisionPC     (const bool &powerON = false);
    void powerUsbHub       (const bool &powerON = false);
    void powerCctv         (const bool &powerON = false);
    void powerThermal      (const bool &powerON = false);
    void powerLidar        (const bool &powerON = false);
    void powerExt52V       (const bool &powerON = false);
    void powerIrLEDs       (const bool &powerON = false);
    void powerComm         (const bool &powerON = false);
    void setBodyHeight     (const int  &height  = false);
    void setFootHeight     (const int  &height  = false);
    void setMaxSpeed       (const int  &speed   = false);
    void comEstimation     (const int  &stage   = -1);

private:
    void loadLogFiles();
    void onCancelButtonPressed();
    void timerEvent(QTimerEvent * event) override;

    int unique_id {0};

    // A timer used to check on the completion status of the action
    QBasicTimer timer_;

    QPushButton* bt_autoStart          = nullptr;
    QPushButton* bt_canCheck           = nullptr;
    QPushButton* bt_findHome           = nullptr;
    QPushButton* bt_sit                = nullptr;
    QPushButton* bt_stand              = nullptr;
    QPushButton* bt_walk               = nullptr;
    QPushButton* bt_walkSlow           = nullptr;
    QPushButton* bt_run                = nullptr;
    QPushButton* bt_calibrateImu       = nullptr;
    QPushButton* bt_staticLock         = nullptr;
    QPushButton* bt_staticReady        = nullptr;
    QPushButton* bt_staticGround       = nullptr;
    QPushButton* bt_recoveryErrorClear = nullptr;
    QPushButton* bt_recoveryFlex       = nullptr;
    QPushButton* bt_emergency          = nullptr;
    QPushButton* bt_switchGamepadPort  = nullptr;
    QPushButton* bt_powerLeg           = nullptr;
    QPushButton* bt_powerArm           = nullptr;
    QPushButton* bt_powerVisionPC      = nullptr;
    QPushButton* bt_powerUsbHub        = nullptr;
    QPushButton* bt_powerCctv          = nullptr;
    QPushButton* bt_powerThermal       = nullptr;
    QPushButton* bt_powerLidar         = nullptr;
    QPushButton* bt_powerExt52V        = nullptr;
    QPushButton* bt_powerIrLEDs        = nullptr;
    QPushButton* bt_powerComm          = nullptr;
    QPushButton* bt_setBodyHeight      = nullptr;
    QPushButton* bt_setFootHeight      = nullptr;
    QPushButton* bt_setMaxSpeed        = nullptr;
    QPushButton* bt_comEstimation      = nullptr;

    QPushButton* start_reset_button_{nullptr};
    QPushButton* pause_resume_button_{nullptr};
    QPushButton* navigation_mode_button_{nullptr};

    QLabel* navigation_status_indicator_{nullptr};
    QLabel* localization_status_indicator_{nullptr};
    QLabel* navigation_goal_status_indicator_{nullptr};
    QLabel* navigation_feedback_indicator_{nullptr};

    QStateMachine state_machine_;

    QState* pre_initial_{nullptr};
    QState* initial_{nullptr};
    QState* idle_{nullptr};
    QState* reset_{nullptr};
    QState* paused_{nullptr};
    QState* resumed_{nullptr};
    // The following states are added to allow for the state of the button to only expose reset
    // while the NavigateToPoses action is not active. While running, the user will be allowed to
    // cancel the action. The ROSActionTransition allows for the state of the action to be detected
    // and the button state to change automatically.
    QState* running_{nullptr};
    QState* canceled_{nullptr};
    // The following states are added to allow to collect several poses to perform a waypoint-mode
    // navigation or navigate through poses mode.
    QState* accumulating_{nullptr};
    QState* accumulated_wp_{nullptr};
    QState* accumulated_nav_through_poses_{nullptr};

    // Publish the visual markers with the waypoints
    void updateWpNavigationMarkers();

    // Create unique id numbers for markers
    int getUniqueId();

    void resetUniqueId();

    // round off double to the specified precision and convert to string
    static inline std::string toString(double val, int precision = 0);

    template<typename T>
    static inline std::string toLabel(T & msg);
};

}  // namespace rbq_rviz_plugins

#endif  //  RBQ_RVIZ_PLUGINS__NAV2_PANEL_HPP_
