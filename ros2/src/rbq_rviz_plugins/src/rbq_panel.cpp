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

#include "Publisher.h"

std::shared_ptr<Publisher> publisher;

using namespace std::chrono_literals;

namespace rbq_rviz_plugins
{

RbqPanel::RbqPanel(QWidget * parent)
    : Panel(parent)
{
    // Create the control button and its tooltip

    start_reset_button_ = new QPushButton;
    pause_resume_button_ = new QPushButton;
    navigation_mode_button_ = new QPushButton;
    navigation_status_indicator_ = new QLabel;
    localization_status_indicator_ = new QLabel;
    navigation_goal_status_indicator_ = new QLabel;
    navigation_feedback_indicator_ = new QLabel;

    publisher = std::make_shared<Publisher>("rbq_rviz_panel");

    bt_autoStart          = new QPushButton;
    bt_canCheck           = new QPushButton;
    bt_findHome           = new QPushButton;
    bt_sit                = new QPushButton;
    bt_stand              = new QPushButton;
    bt_walk               = new QPushButton;
    bt_walkSlow           = new QPushButton;
    bt_run                = new QPushButton;
    bt_calibrateImu       = new QPushButton;
    bt_staticLock         = new QPushButton;
    bt_staticReady        = new QPushButton;
    bt_staticGround       = new QPushButton;
    bt_recoveryErrorClear = new QPushButton;
    bt_recoveryFlex       = new QPushButton;
    bt_emergency          = new QPushButton;
    bt_switchGamepadPort  = new QPushButton;
    bt_powerLeg           = new QPushButton;
    bt_powerArm           = new QPushButton;
    bt_powerVisionPC      = new QPushButton;
    bt_powerUsbHub        = new QPushButton;
    bt_powerCctv          = new QPushButton;
    bt_powerThermal       = new QPushButton;
    bt_powerLidar         = new QPushButton;
    bt_powerExt52V        = new QPushButton;
    bt_powerIrLEDs        = new QPushButton;
    bt_powerComm          = new QPushButton;
    bt_setBodyHeight      = new QPushButton;
    bt_setFootHeight      = new QPushButton;
    bt_setMaxSpeed        = new QPushButton;
    bt_comEstimation      = new QPushButton;

    QSlider* sldr_setBodyHeight = new QSlider;
    sldr_setBodyHeight->setTickInterval(10);
    sldr_setBodyHeight->setSliderPosition(50);
    sldr_setBodyHeight->setRange(0, 100);
    sldr_setBodyHeight->setValue(50);
    sldr_setBodyHeight->setSingleStep(10);
    QObject::connect(sldr_setBodyHeight, &QSlider::valueChanged, this, &RbqPanel::setBodyHeight );

    bt_autoStart         ->setText( "Auto\nStart"        );
    bt_canCheck          ->setText( "Can\nCheck"         );
    bt_findHome          ->setText( "Find\nHome"         );
    bt_sit               ->setText( "Sit"                );
    bt_stand             ->setText( "Stand"              );
    bt_walk              ->setText( "Walk"               );
    bt_walkSlow          ->setText( "Walk\nWave"         );
    bt_run               ->setText( "Run"                );
    bt_staticLock        ->setText( "Static\nLock"       );
    bt_staticReady       ->setText( "Static\nReady"      );
    bt_staticGround      ->setText( "Static\nGround"     );
    bt_emergency         ->setText( "Emergency"          );
    bt_recoveryErrorClear->setText( "Recovery\nError Clear");
    bt_recoveryFlex      ->setText( "Recovery\nFlex"     );
    bt_switchGamepadPort ->setText( "Switch\nGamepadPort");
    bt_powerLeg          ->setText( "Power\nLeg"         );
    bt_powerArm          ->setText( "Power\nArm"         );
    bt_powerVisionPC     ->setText( "Power\nVisionPC"    );
    bt_powerUsbHub       ->setText( "Power\nUsbHub"      );
    bt_powerCctv         ->setText( "Power\nCctv"        );
    bt_powerThermal      ->setText( "Power\nThermal"     );
    bt_powerLidar        ->setText( "Power\nLidar"       );
    bt_powerExt52V       ->setText( "Power\nExt52V"      );
    bt_powerIrLEDs       ->setText( "Power\nIrLEDs"      );
    bt_powerComm         ->setText( "Power\nComm"        );
    bt_setBodyHeight     ->setText( "Set Body Height"    );
    bt_setFootHeight     ->setText( "Set Foot Height"    );
    bt_setMaxSpeed       ->setText( "Set Max Speed"      );
    bt_comEstimation     ->setText( "CoM Estimation"     );
    bt_calibrateImu      ->setText( "Calibrate Imu"      );

    bt_switchGamepadPort ->setCheckable(true);
    bt_switchGamepadPort ->setCheckable(true);
    bt_powerLeg          ->setCheckable(true);
    bt_powerArm          ->setCheckable(true);
    bt_powerVisionPC     ->setCheckable(true);
    bt_powerUsbHub       ->setCheckable(true);
    bt_powerCctv         ->setCheckable(true);
    bt_powerThermal      ->setCheckable(true);
    bt_powerLidar        ->setCheckable(true);
    bt_powerExt52V       ->setCheckable(true);
    bt_powerIrLEDs       ->setCheckable(true);
    bt_powerComm         ->setCheckable(true);

    bt_switchGamepadPort ->setChecked(false);
    bt_switchGamepadPort ->setChecked(false);
    bt_powerLeg          ->setChecked(false);
    bt_powerArm          ->setChecked(false);
    bt_powerVisionPC     ->setChecked(false);
    bt_powerUsbHub       ->setChecked(false);
    bt_powerCctv         ->setChecked(false);
    bt_powerThermal      ->setChecked(false);
    bt_powerLidar        ->setChecked(false);
    bt_powerExt52V       ->setChecked(false);
    bt_powerIrLEDs       ->setChecked(false);
    bt_powerComm         ->setChecked(false);

    QObject::connect(bt_autoStart         , &QPushButton::clicked, this, &RbqPanel::autoStart         );
    QObject::connect(bt_canCheck          , &QPushButton::clicked, this, &RbqPanel::canCheck          );
    QObject::connect(bt_findHome          , &QPushButton::clicked, this, &RbqPanel::findHome          );
    QObject::connect(bt_sit               , &QPushButton::clicked, this, &RbqPanel::sit               );
    QObject::connect(bt_stand             , &QPushButton::clicked, this, &RbqPanel::stand             );
    QObject::connect(bt_walk              , &QPushButton::clicked, this, &RbqPanel::walk              );
    QObject::connect(bt_walkSlow          , &QPushButton::clicked, this, &RbqPanel::walkSlow          );
    QObject::connect(bt_run               , &QPushButton::clicked, this, &RbqPanel::run               );
    QObject::connect(bt_calibrateImu      , &QPushButton::clicked, this, &RbqPanel::calibrateImu      );
    QObject::connect(bt_staticLock        , &QPushButton::clicked, this, &RbqPanel::staticLock        );
    QObject::connect(bt_staticReady       , &QPushButton::clicked, this, &RbqPanel::staticReady       );
    QObject::connect(bt_staticGround      , &QPushButton::clicked, this, &RbqPanel::staticGround      );
    QObject::connect(bt_recoveryErrorClear, &QPushButton::clicked, this, &RbqPanel::recoveryErrorClear);
    QObject::connect(bt_recoveryFlex      , &QPushButton::clicked, this, &RbqPanel::recoveryFlex      );
    QObject::connect(bt_emergency         , &QPushButton::clicked, this, &RbqPanel::emergency         );
    QObject::connect(bt_switchGamepadPort , &QPushButton::clicked, this, &RbqPanel::switchGamepadPort );
    QObject::connect(bt_powerLeg          , &QPushButton::clicked, this, &RbqPanel::powerLeg          );
    QObject::connect(bt_powerArm          , &QPushButton::clicked, this, &RbqPanel::powerArm          );
    QObject::connect(bt_powerVisionPC     , &QPushButton::clicked, this, &RbqPanel::powerVisionPC     );
    QObject::connect(bt_powerUsbHub       , &QPushButton::clicked, this, &RbqPanel::powerUsbHub       );
    QObject::connect(bt_powerCctv         , &QPushButton::clicked, this, &RbqPanel::powerCctv         );
    QObject::connect(bt_powerThermal      , &QPushButton::clicked, this, &RbqPanel::powerThermal      );
    QObject::connect(bt_powerLidar        , &QPushButton::clicked, this, &RbqPanel::powerLidar        );
    QObject::connect(bt_powerExt52V       , &QPushButton::clicked, this, &RbqPanel::powerExt52V       );
    QObject::connect(bt_powerIrLEDs       , &QPushButton::clicked, this, &RbqPanel::powerIrLEDs       );
    QObject::connect(bt_powerComm         , &QPushButton::clicked, this, &RbqPanel::powerComm         );
    // QObject::connect(bt_setBodyHeight     , &QPushButton::clicked, this, &RbqPanel::setBodyHeight     );
    // QObject::connect(bt_setFootHeight     , &QPushButton::clicked, this, &RbqPanel::setFootHeight     );
    // QObject::connect(bt_setMaxSpeed       , &QPushButton::clicked, this, &RbqPanel::setMaxSpeed       );
    QObject::connect(bt_comEstimation     , &QPushButton::clicked, this, &RbqPanel::comEstimation     );

    QVBoxLayout * lyt_buttons = new QVBoxLayout;

    QHBoxLayout* lyt_initialize = new QHBoxLayout;
    lyt_initialize->addWidget(bt_autoStart  );
    lyt_initialize->addWidget(bt_canCheck   );
    lyt_initialize->addWidget(bt_findHome   );
    lyt_buttons->addLayout(lyt_initialize);

    QHBoxLayout* lyt_dynamic1 = new QHBoxLayout;
    lyt_dynamic1->addWidget(bt_sit      );
    lyt_dynamic1->addWidget(bt_stand    );
    lyt_buttons->addLayout(lyt_dynamic1);

    QHBoxLayout* lyt_dynamic2 = new QHBoxLayout;
    lyt_dynamic2->addWidget(bt_walk     );
    lyt_dynamic2->addWidget(bt_walkSlow );
    lyt_dynamic2->addWidget(bt_run      );
    lyt_buttons->addLayout(lyt_dynamic2);

    QHBoxLayout* lyt_static = new QHBoxLayout;
    lyt_static->addWidget(bt_staticLock        );
    lyt_static->addWidget(bt_staticReady       );
    lyt_static->addWidget(bt_staticGround      );
    lyt_buttons->addLayout(lyt_static);

    QHBoxLayout* lyt_reco = new QHBoxLayout;
    lyt_reco->addWidget(bt_emergency         );
    lyt_reco->addWidget(bt_recoveryErrorClear);
    lyt_reco->addWidget(bt_recoveryFlex      );
    lyt_buttons->addLayout(lyt_reco);

    QHBoxLayout* lyt_power1 = new QHBoxLayout;
    lyt_power1->addWidget(bt_powerLeg          );
    lyt_power1->addWidget(bt_powerArm          );
    lyt_power1->addWidget(bt_powerExt52V       );
    lyt_buttons->addLayout(lyt_power1);

    QHBoxLayout* lyt_vision1 = new QHBoxLayout;
    lyt_vision1->addWidget(bt_powerVisionPC     );
    lyt_vision1->addWidget(bt_powerUsbHub       );
    lyt_vision1->addWidget(bt_powerIrLEDs       );
    lyt_buttons->addLayout(lyt_vision1);

    QHBoxLayout* lyt_vision2 = new QHBoxLayout;
    lyt_vision2->addWidget(bt_powerCctv         );
    lyt_vision2->addWidget(bt_powerThermal      );
    lyt_vision2->addWidget(bt_powerLidar        );
    lyt_buttons->addLayout(lyt_vision2);



    lyt_buttons->addWidget(bt_calibrateImu      );
    lyt_buttons->addWidget(bt_switchGamepadPort );
    lyt_buttons->addWidget(bt_powerComm         );
    // lyt_buttons->addWidget(bt_setBodyHeight     );
    // lyt_buttons->addWidget(bt_setFootHeight     );
    // lyt_buttons->addWidget(bt_setMaxSpeed       );
    lyt_buttons->addWidget(bt_comEstimation     );
    lyt_buttons->addWidget(sldr_setBodyHeight   );









    {


    // Create the state machine used to present the proper control button states in the UI

    const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
    const char * shutdown_msg = "Deactivate and cleanup all nav2 lifecycle nodes";
    const char * cancel_msg = "Cancel navigation";
    const char * pause_msg = "Deactivate all nav2 lifecycle nodes";
    const char * resume_msg = "Activate all nav2 lifecycle nodes";
    const char * single_goal_msg = "Change to waypoint / nav through poses style navigation";
    const char * waypoint_goal_msg = "Start following waypoints";
    const char * nft_goal_msg = "Start navigating through poses";
    const char * cancel_waypoint_msg = "Cancel waypoint / viapoint accumulation mode";

    const QString navigation_active("<table><tr><td width=100><b>Navigation:</b></td>"
                                    "<td><font color=green>active</color></td></tr></table>");
    const QString navigation_inactive("<table><tr><td width=100><b>Navigation:</b></td>"
                                      "<td>inactive</td></tr></table>");
    const QString navigation_unknown("<table><tr><td width=100><b>Navigation:</b></td>"
                                     "<td>unknown</td></tr></table>");
    const QString localization_active("<table><tr><td width=100><b>Localization:</b></td>"
                                      "<td><font color=green>active</color></td></tr></table>");
    const QString localization_inactive("<table><tr><td width=100><b>Localization:</b></td>"
                                        "<td>inactive</td></tr></table>");
    const QString localization_unknown("<table><tr><td width=100><b>Localization:</b></td>"
                                       "<td>unknown</td></tr></table>");

    navigation_status_indicator_->setText(navigation_unknown);
    localization_status_indicator_->setText(localization_unknown);
    navigation_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    localization_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    navigation_goal_status_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    navigation_feedback_indicator_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    pre_initial_ = new QState();
    pre_initial_->setObjectName("pre_initial");
    pre_initial_->assignProperty(start_reset_button_, "text", "Startup");
    pre_initial_->assignProperty(start_reset_button_, "enabled", false);

    pre_initial_->assignProperty(pause_resume_button_, "text", "Pause");
    pre_initial_->assignProperty(pause_resume_button_, "enabled", false);

    pre_initial_->assignProperty(
        navigation_mode_button_, "text",
        "Waypoint / Nav Through Poses Mode");
    pre_initial_->assignProperty(navigation_mode_button_, "enabled", false);

    initial_ = new QState();
    initial_->setObjectName("initial");
    initial_->assignProperty(start_reset_button_, "text", "Startup");
    initial_->assignProperty(start_reset_button_, "toolTip", startup_msg);
    initial_->assignProperty(start_reset_button_, "enabled", true);

    initial_->assignProperty(pause_resume_button_, "text", "Pause");
    initial_->assignProperty(pause_resume_button_, "enabled", false);

    initial_->assignProperty(navigation_mode_button_, "text", "Waypoint / Nav Through Poses Mode");
    initial_->assignProperty(navigation_mode_button_, "enabled", false);

    // State entered when navigate_to_pose action is not active
    idle_ = new QState();
    idle_->setObjectName("idle");
    idle_->assignProperty(start_reset_button_, "text", "Reset");
    idle_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);
    idle_->assignProperty(start_reset_button_, "enabled", true);

    idle_->assignProperty(pause_resume_button_, "text", "Pause");
    idle_->assignProperty(pause_resume_button_, "enabled", true);
    idle_->assignProperty(pause_resume_button_, "toolTip", pause_msg);

    idle_->assignProperty(navigation_mode_button_, "text", "Waypoint / Nav Through Poses Mode");
    idle_->assignProperty(navigation_mode_button_, "enabled", true);
    idle_->assignProperty(navigation_mode_button_, "toolTip", single_goal_msg);

    // State entered when navigate_to_pose action is not active
    accumulating_ = new QState();
    accumulating_->setObjectName("accumulating");
    accumulating_->assignProperty(start_reset_button_, "text", "Cancel Accumulation");
    accumulating_->assignProperty(start_reset_button_, "toolTip", cancel_waypoint_msg);
    accumulating_->assignProperty(start_reset_button_, "enabled", true);

    accumulating_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
    accumulating_->assignProperty(pause_resume_button_, "enabled", true);
    accumulating_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

    accumulating_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
    accumulating_->assignProperty(navigation_mode_button_, "enabled", true);
    accumulating_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

    accumulated_wp_ = new QState();
    accumulated_wp_->setObjectName("accumulated_wp");
    accumulated_wp_->assignProperty(start_reset_button_, "text", "Cancel");
    accumulated_wp_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
    accumulated_wp_->assignProperty(start_reset_button_, "enabled", true);

    accumulated_wp_->assignProperty(pause_resume_button_, "text", "Start Nav Through Poses");
    accumulated_wp_->assignProperty(pause_resume_button_, "enabled", false);
    accumulated_wp_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

    accumulated_wp_->assignProperty(navigation_mode_button_, "text", "Start Waypoint Following");
    accumulated_wp_->assignProperty(navigation_mode_button_, "enabled", false);
    accumulated_wp_->assignProperty(navigation_mode_button_, "toolTip", waypoint_goal_msg);

    accumulated_nav_through_poses_ = new QState();
    accumulated_nav_through_poses_->setObjectName("accumulated_nav_through_poses");
    accumulated_nav_through_poses_->assignProperty(start_reset_button_, "text", "Cancel");
    accumulated_nav_through_poses_->assignProperty(start_reset_button_, "toolTip", cancel_msg);
    accumulated_nav_through_poses_->assignProperty(start_reset_button_, "enabled", true);

    accumulated_nav_through_poses_->assignProperty(
        pause_resume_button_, "text",
        "Start Nav Through Poses");
    accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "enabled", false);
    accumulated_nav_through_poses_->assignProperty(pause_resume_button_, "toolTip", nft_goal_msg);

    accumulated_nav_through_poses_->assignProperty(
        navigation_mode_button_, "text",
        "Start Waypoint Following");
    accumulated_nav_through_poses_->assignProperty(navigation_mode_button_, "enabled", false);
    accumulated_nav_through_poses_->assignProperty(
        navigation_mode_button_, "toolTip",
        waypoint_goal_msg);

    // State entered to cancel the navigate_to_pose action
    canceled_ = new QState();
    canceled_->setObjectName("canceled");

    // State entered to reset the nav2 lifecycle nodes
    reset_ = new QState();
    reset_->setObjectName("reset");

    // State entered while the navigate_to_pose action is active
    running_ = new QState();
    running_->setObjectName("running");
    running_->assignProperty(start_reset_button_, "text", "Cancel");
    running_->assignProperty(start_reset_button_, "toolTip", cancel_msg);

    running_->assignProperty(pause_resume_button_, "text", "Pause");
    running_->assignProperty(pause_resume_button_, "enabled", false);

    running_->assignProperty(navigation_mode_button_, "text", "Waypoint mode");
    running_->assignProperty(navigation_mode_button_, "enabled", false);

    // State entered when pause is requested
    paused_ = new QState();
    paused_->setObjectName("pausing");
    paused_->assignProperty(start_reset_button_, "text", "Reset");
    paused_->assignProperty(start_reset_button_, "toolTip", shutdown_msg);

    paused_->assignProperty(pause_resume_button_, "text", "Resume");
    paused_->assignProperty(pause_resume_button_, "toolTip", resume_msg);
    paused_->assignProperty(pause_resume_button_, "enabled", true);

    paused_->assignProperty(navigation_mode_button_, "text", "Start navigation");
    paused_->assignProperty(navigation_mode_button_, "toolTip", resume_msg);
    paused_->assignProperty(navigation_mode_button_, "enabled", true);

    // State entered to resume the nav2 lifecycle nodes
    resumed_ = new QState();
    resumed_->setObjectName("resuming");

    QObject::connect(initial_, SIGNAL(exited()), this, SLOT(onStartup()));
    QObject::connect(canceled_, SIGNAL(exited()), this, SLOT(onCancel()));
    QObject::connect(reset_, SIGNAL(exited()), this, SLOT(onShutdown()));
    QObject::connect(paused_, SIGNAL(entered()), this, SLOT(onPause()));
    QObject::connect(resumed_, SIGNAL(exited()), this, SLOT(onResume()));
    QObject::connect(accumulating_, SIGNAL(entered()), this, SLOT(onAccumulating()));
    QObject::connect(accumulated_wp_, SIGNAL(entered()), this, SLOT(onAccumulatedWp()));
    QObject::connect(accumulated_nav_through_poses_, SIGNAL(entered()), this, SLOT(onAccumulatedNTP()));

    // Start/Reset button click transitions
    initial_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
    idle_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
    running_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
    paused_->addTransition(start_reset_button_, SIGNAL(clicked()), reset_);
    idle_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulating_);
    accumulating_->addTransition(navigation_mode_button_, SIGNAL(clicked()), accumulated_wp_);
    accumulating_->addTransition(
        pause_resume_button_, SIGNAL(
            clicked()), accumulated_nav_through_poses_);
    accumulating_->addTransition(start_reset_button_, SIGNAL(clicked()), idle_);
    accumulated_wp_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);
    accumulated_nav_through_poses_->addTransition(start_reset_button_, SIGNAL(clicked()), canceled_);

    // Internal state transitions
    canceled_->addTransition(canceled_, SIGNAL(entered()), idle_);
    reset_->addTransition(reset_, SIGNAL(entered()), initial_);
    resumed_->addTransition(resumed_, SIGNAL(entered()), idle_);

    // Pause/Resume button click transitions
    idle_->addTransition(pause_resume_button_, SIGNAL(clicked()), paused_);
    paused_->addTransition(pause_resume_button_, SIGNAL(clicked()), resumed_);

    state_machine_.addState(pre_initial_);
    state_machine_.addState(initial_);
    state_machine_.addState(idle_);
    state_machine_.addState(running_);
    state_machine_.addState(canceled_);
    state_machine_.addState(reset_);
    state_machine_.addState(paused_);
    state_machine_.addState(resumed_);
    state_machine_.addState(accumulating_);
    state_machine_.addState(accumulated_wp_);
    state_machine_.addState(accumulated_nav_through_poses_);

    state_machine_.setInitialState(pre_initial_);

    // delay starting initial thread until state machine has started or a race occurs
    QObject::connect(&state_machine_, SIGNAL(started()), this, SLOT(startThread()));
    state_machine_.start();
    }

    // Lay out the items in the panel
    QVBoxLayout * main_layout = new QVBoxLayout;
    // main_layout->addWidget(navigation_status_indicator_);
    // main_layout->addWidget(localization_status_indicator_);
    // main_layout->addWidget(navigation_goal_status_indicator_);
    // main_layout->addWidget(navigation_feedback_indicator_);
    // main_layout->addWidget(pause_resume_button_);
    // main_layout->addWidget(start_reset_button_);
    // main_layout->addWidget(navigation_mode_button_);

    // main_layout->addWidget(lyt_buttons);
    main_layout->insertLayout(0, lyt_buttons);

    main_layout->setContentsMargins(10, 10, 10, 10);
    setLayout(main_layout);

}

RbqPanel::~RbqPanel()
{

}

void
RbqPanel::onInitialize()
{

}

void
RbqPanel::startThread()
{

}

void
RbqPanel::onPause()
{

}

void
RbqPanel::onResume()
{

}

void
RbqPanel::onStartup()
{

}

void
RbqPanel::onShutdown()
{
    timer_.stop();
}

void
RbqPanel::onCancel()
{
    QFuture<void> future =
        QtConcurrent::run(
            std::bind(
                &RbqPanel::onCancelButtonPressed,
                this));
}

void
RbqPanel::onNewGoal(double x, double y, double theta, QString frame)
{
    updateWpNavigationMarkers();
    Q_UNUSED(x);
    Q_UNUSED(y);
    Q_UNUSED(theta);
    Q_UNUSED(frame);
}

// void RbqPanel::onAutoStart()
// {
//     std::cout << "Auto Start" << std::endl;
//     publisher->pub_autoStart();
// }

// void RbqPanel::onStand()
// {
//     std::cout << "Stand" << std::endl;
//     publisher->pub_stand();
// }

// void RbqPanel::onSit()
// {
//     std::cout << "Sit" << std::endl;
//     publisher->pub_sit();
// }

void RbqPanel::autoStart         () { publisher->pub_autoStart         (); }
void RbqPanel::canCheck          () { publisher->pub_canCheck          (); }
void RbqPanel::findHome          () { publisher->pub_findHome          (); }
void RbqPanel::sit               () { publisher->pub_sit               (); }
void RbqPanel::stand             () { publisher->pub_stand             (); }
void RbqPanel::walk              () { publisher->pub_walk              (); }
void RbqPanel::walkSlow          () { publisher->pub_walkSlow          (); }
void RbqPanel::run               () { publisher->pub_run               (); }
void RbqPanel::calibrateImu      () { publisher->pub_calibrateImu      (); }
void RbqPanel::staticLock        () { publisher->pub_staticLock        (); }
void RbqPanel::staticReady       () { publisher->pub_staticReady       (); }
void RbqPanel::staticGround      () { publisher->pub_staticGround      (); }
void RbqPanel::recoveryErrorClear() { publisher->pub_recoveryErrorClear(); }
void RbqPanel::recoveryFlex      () { publisher->pub_recoveryFlex      (); }
void RbqPanel::emergency         () { publisher->pub_emergency         (); }

void RbqPanel::switchGamepadPort (const bool &powerON) { publisher->pub_switchGamepadPort (powerON); }
void RbqPanel::powerLeg          (const bool &powerON) { publisher->pub_powerLeg          (powerON); }
void RbqPanel::powerArm          (const bool &powerON) { publisher->pub_powerArm          (powerON); }
void RbqPanel::powerVisionPC     (const bool &powerON) { publisher->pub_powerVisionPC     (powerON); }
void RbqPanel::powerUsbHub       (const bool &powerON) { publisher->pub_powerUsbHub       (powerON); }
void RbqPanel::powerCctv         (const bool &powerON) { publisher->pub_powerCctv         (powerON); }
void RbqPanel::powerThermal      (const bool &powerON) { publisher->pub_powerThermal      (powerON); }
void RbqPanel::powerLidar        (const bool &powerON) { publisher->pub_powerLidar        (powerON); }
void RbqPanel::powerExt52V       (const bool &powerON) { publisher->pub_powerExt52V       (powerON); }
void RbqPanel::powerIrLEDs       (const bool &powerON) { publisher->pub_powerIrLEDs       (powerON); }
void RbqPanel::powerComm         (const bool &powerON) { publisher->pub_powerComm         (powerON); }
void RbqPanel::setBodyHeight     (const int  &height ) { publisher->pub_setBodyHeight     (height ); }
void RbqPanel::setFootHeight     (const int  &height ) { publisher->pub_setFootHeight     (height ); }
void RbqPanel::setMaxSpeed       (const int  &speed  ) { publisher->pub_setMaxSpeed       (speed  ); }
void RbqPanel::comEstimation     (const int  &stage  ) { publisher->pub_comEstimation     (stage  ); }

void
RbqPanel::onCancelButtonPressed()
{
    timer_.stop();
}

void
RbqPanel::onAccumulatedWp()
{
    std::cout << "Start waypoint" << std::endl;
}

void
RbqPanel::onAccumulatedNTP()
{
    std::cout << "Start navigate through poses" << std::endl;
}

void
RbqPanel::onAccumulating()
{

}

void
RbqPanel::timerEvent(QTimerEvent * event)
{
    Q_UNUSED(event);
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

void
RbqPanel::resetUniqueId()
{
    unique_id = 0;
}

int
RbqPanel::getUniqueId()
{
    int temp_id = unique_id;
    unique_id += 1;
    return temp_id;
}

void
RbqPanel::updateWpNavigationMarkers()
{
    resetUniqueId();
}

template<typename T>
inline std::string RbqPanel::toLabel(T & msg)
{
    return std::string(
        "<tr><td width=150>ETA:</td><td>" +
        toString(rclcpp::Duration(msg.estimated_time_remaining).seconds(), 0) + " s"
                                                                                "</td></tr><tr><td width=150>Distance remaining:</td><td>" +
        toString(msg.distance_remaining, 2) + " m"
                                              "</td></tr><tr><td width=150>Time taken:</td><td>" +
        toString(rclcpp::Duration(msg.navigation_time).seconds(), 0) + " s"
                                                                       "</td></tr><tr><td width=150>Recoveries:</td><td>" +
        std::to_string(msg.number_of_recoveries) +
        "</td></tr>");
}

inline std::string
RbqPanel::toString(double val, int precision)
{
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << val;
    return out.str();
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rbq_rviz_plugins::RbqPanel, rviz_common::Panel)
