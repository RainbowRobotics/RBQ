#ifndef RBTYPES_HPP
#define RBTYPES_HPP

#include <QObject>
#include <QString>
#include <QVector>

#include <array>

#include "half.hpp"

using half_float::half;
using namespace half_float::literal;

namespace RBQ_SDK {

// ------- ID 0~255 ----------
constexpr unsigned char ID_ROBOT_STATE_T                    = 1;
constexpr unsigned char ID_GENERAL_REQUEST                  = 2;
constexpr unsigned char ID_HIGH_LEVEL_COMMAND               = 3;
constexpr unsigned char ID_LEG_STATE_ARRAY                  = 4;

constexpr unsigned char ID_VISION_STATE_T                   = 20;

constexpr unsigned char ID_POINTCLOUD_T                     = 50;

constexpr unsigned char ID_RC_STATE                         = 255;

// ------- ID 0~255 ---------

constexpr unsigned short PORT_ROBOT_MOTION_GAMEPAD_UDP      = 28223;
constexpr unsigned short PORT_ROBOT_MOTION_GAMEPAD_EXT_UDP  = 28224;

constexpr unsigned short PORT_ROBOT_MOTION_UDP              = 56781;
constexpr unsigned short PORT_ROBOT_MOTION_TCP              = 56782;

constexpr unsigned short PORT_ROBOT_VISION_UDP              = 56791;
constexpr unsigned short PORT_ROBOT_VISION_TCP              = 56792;

constexpr uint16_t GCS_UDP_PORT_LIVESTREAM   = 28554;

constexpr static float D2R  = 0.01745329;
constexpr static float R2D  = 57.29577;
constexpr static float PI   = 3.141592;

struct HighLevelCmd_t {
    uint32_t tickWrite  = 0;
    uint32_t tickRead   = 0;

    uint32_t senderIp4Addr = 0;
    unsigned short senderPort = 0;
    bool senderTcp      = false;
    bool accepted       = false;

    float roll              = 0;
    float pitch             = 0;
    float yaw               = 0;
    float vel_x             = 0;
    float vel_y             = 0;
    float omega_z           = 0;
    float delta_body_h      = 0;
    float delta_foot_h      = 0;
    int gaitID              = 0;

    /// CommandContainer_t type identifier
    const unsigned char typeIdentifier = ID_HIGH_LEVEL_COMMAND;

    const unsigned char tail1 =  128;
    const unsigned char tail2 =  127;

    HighLevelCmd_t() {}
    HighLevelCmd_t(const HighLevelCmd_t& p)
    {
        tickWrite       = p.tickWrite;
        tickRead        = p.tickRead;
        senderIp4Addr   = p.senderIp4Addr;
        senderPort      = p.senderPort;
        senderTcp       = p.senderTcp;
        accepted        = p.accepted;

        roll            = p.roll;
        pitch           = p.pitch;
        yaw             = p.yaw;
        vel_x           = p.vel_x;
        vel_y           = p.vel_y;
        omega_z         = p.omega_z;
        delta_body_h    = p.delta_body_h;
        delta_foot_h    = p.delta_foot_h;
        gaitID          = p.gaitID;
    }
    HighLevelCmd_t& operator=(const HighLevelCmd_t& p)
    {
        tickWrite       = p.tickWrite;
        tickRead        = p.tickRead;
        senderIp4Addr   = p.senderIp4Addr;
        senderPort      = p.senderPort;
        senderTcp       = p.senderTcp;
        accepted        = p.accepted;

        roll            = p.roll;
        pitch           = p.pitch;
        yaw             = p.yaw;
        vel_x           = p.vel_x;
        vel_y           = p.vel_y;
        omega_z         = p.omega_z;
        delta_body_h    = p.delta_body_h;
        delta_foot_h    = p.delta_foot_h;
        gaitID          = p.gaitID;
        return *this;
    }
};

struct Request_t {
    const unsigned char head1 =  255;
    const unsigned char head2 =  254;
    /// CommandContainer_t type identifier
    const unsigned char typeIdentifier = ID_GENERAL_REQUEST;

    int requestID = 0;

    uint32_t requestNumber = 0;

    bool requestAccepted = false;

    unsigned char receiverProgramID = 0;

    const static uint8_t containerSize = 10;

    bool            containerBools  [containerSize] = {0,};
    unsigned char   containerUchars [containerSize] = {0,};
    int             containerInts   [containerSize] = {0,};
    float           containerFloats [containerSize] = {0,};

    const unsigned char tail1 =  128;
    const unsigned char tail2 =  127;

    Request_t() {}
    Request_t(const Request_t& p)
    {
        requestID = p.requestID;
        requestNumber = p.requestNumber;
        requestAccepted = p.requestAccepted;
        memcpy(containerBools, p.containerBools, sizeof(bool)*containerSize);
        memcpy(containerUchars, p.containerUchars, sizeof(char)*containerSize);
        memcpy(containerInts, p.containerInts, sizeof(int)*containerSize);
        memcpy(containerFloats, p.containerFloats, sizeof(float)*containerSize);
    }
    Request_t& operator=(const Request_t& p)
    {
        requestID = p.requestID;
        requestNumber = p.requestNumber;
        requestAccepted = p.requestAccepted;
        memcpy(containerBools, p.containerBools, sizeof(bool)*containerSize);
        memcpy(containerUchars, p.containerUchars, sizeof(char)*containerSize);
        memcpy(containerInts, p.containerInts, sizeof(int)*containerSize);
        memcpy(containerFloats, p.containerFloats, sizeof(float)*containerSize);
        return *this;
    }
};

class Motion : public QObject {
    Q_OBJECT
public:

    struct MotorStatus_t {
        unsigned    FET:1;	 	// FET ON
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;  // Nonius Count err
        unsigned    BAT:1;      // Low Battery
        unsigned    CALIB:1;    // Calibration Mode
        unsigned    MT_ERR:1;   // Reply Status

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error
        unsigned    TMP:1;      // Temperature Error
        unsigned    PS1:1;		// Position Limit Error (Lower)
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    KP:16;
        unsigned    KD:16;
    };

    struct JointState_t {
        bool m_connected      = 0;
        char  temperature   = 0;
        MotorStatus_t status;
        half position       = 0.0_h;
        half torque         = 0.0_h;
    };

    struct BatteryState_t {
        std::array<bool, 2>     charging    = {false,};
        std::array<bool, 2>     m_connected   = {false,};
        std::array<uint8_t, 2>  percentage  = {0,};
        std::array<uint8_t, 2>  voltage     = {0,};
        std::array<uint8_t, 2>  temperature = {0,};
        std::array<half, 2>     current     = {0.0_h,};
    };

    struct PduState_t {
        std::array<bool, 6>     port_05V;
        std::array<bool, 6>     port_12V;
        std::array<bool, 6>     port_52V;
    };

    struct ImuState_t {
        std::array<half, 4> quaternion;    // Quaternion [w,x,y,z]
        std::array<half, 3> rpy;           // Euler angles. Unit is in radians 'rad'
        std::array<half, 3> gyroscope;     // Angular velocity. Unit is in radians per second 'rad/s'
        std::array<half, 3> accelerometer; // Acceleration. Unit is in radians per second squared 'rad/s^2'
        int8_t temperature;                 // Sensor temperature. Unit is in celcius '°C'
    };

    struct GamepadState_t {
        /// Gamepad status. True if m_connected
        bool status = false;
        /// Left Joytick. Float array length of 2
        /// first is vertical 'up-down' axis value. Range: [-1, 1]
        /// second is horizontal 'left-right' axis value. Range: [-1, 1]
        std::array<half, 2> leftJoystick{0.0_h,};
        /// Right Joytick. Float array length of 2
        /// first is vertical 'up-down' axis value. Range: [-1, 1]
        /// second is horizontal 'left-right' axis value. Range: [-1, 1]
        std::array<half, 2> rightJoystick{0.0_h,};

        half leftTrigger = 0.0_h;
        float rightTrigger = 0.0_h;

        /// 20 buttons' state. True if pressed
        std::array<bool, 20>  buttons{false,};

        /// Commander ipv4 address
        uint32_t senderIp4Addr = 0;
    };

    enum StateIdentifier_t : unsigned char {
        undefinedState,
        currentState,
        controlRefState,
        commandRefState,
    };

    struct RobotStatus_t {
        bool    CON_START       = 0;
        bool    READY_POS       = 0;
        bool    GROUND_POS      = 0;
        bool    FORCE_CON       = 0;
        bool    EXT_JOY         = 0;
        bool    IS_STANDING     = 0;
        bool    CAN_CHECK       = 0;
        bool    FIND_HOME       = 0;

        int8_t GAIT_ID          = 0;
        bool    IS_FALL         = 0;
        bool    reserve3;
        bool    reserve4;
        bool    reserve5;
        bool    reserve6;
        bool    reserve7;
        bool    reserve8;

        bool    att00Connected  = 0;    // arm
        bool    att01Connected  = 0;    // att1
        bool    att02Connected  = 0;    // att2
        bool    att03Connected  = 0;    // cctv
        bool    att04Connected  = 0;    // thermal
        bool    att05Connected  = 0;    //
        bool    att06Connected  = 0;
        bool    att07Connected  = 0;

        bool    reserve17;
        bool    reserve18;
        bool    reserve19;
        bool    reserve20;
        bool    reserve21;
        bool    reserve22;
        bool    reserve23;
        bool    reserve24;
    };
#ifndef RB_SHARED_MEMORY_H
    struct Velocity_t {
        std::array<half, 3> linear{0.0_h,};
        std::array<half, 3> angular{0.0_h,};
    };

    struct Pose_t {
        std::array<half, 3> position{0.0_h,};
        std::array<half, 3> rpy{0.0_h,};
        std::array<half, 4> quaternion{0.0_h,};
    };

    struct Odometry_t {
        Pose_t pose;
        Velocity_t velocity;
    };

    struct Odometries_t {
        /// Body Velocity wrt Body frame
        Velocity_t velocity;
        /// Body Odometry wrt World frame
        Odometry_t odomWrtWorld;
        /// Body Odometry wrt World frame
        Odometry_t odomWrtMap;
        /// Body Odometry wrt Ground frame
        /// pose.position[2] : body height
        /// pose.rpy : body rotation wrt ground
        /// velocity.angular[2] : omega wrt ground
        /// velocity.linear[0,1] : linear velocity
        Odometry_t odomWrtGround;
    };
#endif
    struct RobotState_t {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;
        double time = 0;

        uint32_t senderIp4Addr = 0;
        unsigned short senderPort = 0;
        bool senderTcp      = false;
        bool accepted       = false;

        /// Defines Robot's Status
        RobotStatus_t robotStatus;

        /// Battery state
        BatteryState_t batteryState;

        /// Power Delivery Unit state
        PduState_t pduState;

        /// Imu sensor's current state.
        ImuState_t imuState;

        /// Odometries
        Odometries_t odometries;

        /// Active joint count. Max: 20
        uint8_t jointCount = 20;

        /// Joints' state.
        /// 12 [0-11] for legs' joints.
        ///  7 [12-18] for manipulators' joints.
        ///  1 [19] for reserved joint.
        std::array<JointState_t, 20> jointStates;

        /// Accepted remote controller gamepad's current state.
        // GamepadState_t gamepadState;
        /// replacing gamepad with flexible data.
        std::array<char, 40> flexibleData;

        /// Defines if the state is currentState, or referenceState ..
        StateIdentifier_t stateID;

        /// RobotState_t type identifier
        const unsigned char identifier = ID_ROBOT_STATE_T;

        const unsigned char tail1 =  128;
        const unsigned char tail2 =  127;
    };

    enum RequestId_e {
        undefinedRequest                = 0,
        ping                            = 1,
        registerAsNewGcs                = 2,
        sendbackRobotState              = 3,
        REQ_SENDBACK_DEVICES_STATE      = 4,
        REQ_SENDBACK_LEG_STATE_ARRAY    = 5,
    };

    struct RobotPose_t
    {
        float   time        = 0;
        float   x           = 0;
        float   y           = 0;
        float   z           = 0;
        float   rx          = 0;
        float   ry          = 0;
        float   rz          = 0;
        float   fl_leg_0[3] = {0,};
        float   fl_leg_1[3] = {0,};
        float   fl_leg_2[3] = {0,};
        float   fr_leg_0[3] = {0,};
        float   fr_leg_1[3] = {0,};
        float   fr_leg_2[3] = {0,};
        float   rl_leg_0[3] = {0,};
        float   rl_leg_1[3] = {0,};
        float   rl_leg_2[3] = {0,};
        float   rr_leg_0[3] = {0,};
        float   rr_leg_1[3] = {0,};
        float   rr_leg_2[3] = {0,};

        RobotPose_t() { }
        RobotPose_t(const RobotPose_t& p)
        {
            time = p.time;

            x = p.x;
            y = p.y;
            z = p.z;
            rx = p.rx;
            ry = p.ry;
            rz = p.rz;

            memcpy(fl_leg_0, p.fl_leg_0, sizeof(float)*3);
            memcpy(fl_leg_1, p.fl_leg_1, sizeof(float)*3);
            memcpy(fl_leg_2, p.fl_leg_2, sizeof(float)*3);
            memcpy(fr_leg_0, p.fr_leg_0, sizeof(float)*3);
            memcpy(fr_leg_1, p.fr_leg_1, sizeof(float)*3);
            memcpy(fr_leg_2, p.fr_leg_2, sizeof(float)*3);
            memcpy(rl_leg_0, p.rl_leg_0, sizeof(float)*3);
            memcpy(rl_leg_1, p.rl_leg_1, sizeof(float)*3);
            memcpy(rl_leg_2, p.rl_leg_2, sizeof(float)*3);
            memcpy(rr_leg_0, p.rr_leg_0, sizeof(float)*3);
            memcpy(rr_leg_1, p.rr_leg_1, sizeof(float)*3);
            memcpy(rr_leg_2, p.rr_leg_2, sizeof(float)*3);
        }
        RobotPose_t& operator=(const RobotPose_t& p)
        {
            time = p.time;

            x = p.x;
            y = p.y;
            z = p.z;
            rx = p.rx;
            ry = p.ry;
            rz = p.rz;

            memcpy(fl_leg_0, p.fl_leg_0, sizeof(float)*3);
            memcpy(fl_leg_1, p.fl_leg_1, sizeof(float)*3);
            memcpy(fl_leg_2, p.fl_leg_2, sizeof(float)*3);
            memcpy(fr_leg_0, p.fr_leg_0, sizeof(float)*3);
            memcpy(fr_leg_1, p.fr_leg_1, sizeof(float)*3);
            memcpy(fr_leg_2, p.fr_leg_2, sizeof(float)*3);
            memcpy(rl_leg_0, p.rl_leg_0, sizeof(float)*3);
            memcpy(rl_leg_1, p.rl_leg_1, sizeof(float)*3);
            memcpy(rl_leg_2, p.rl_leg_2, sizeof(float)*3);
            memcpy(rr_leg_0, p.rr_leg_0, sizeof(float)*3);
            memcpy(rr_leg_1, p.rr_leg_1, sizeof(float)*3);
            memcpy(rr_leg_2, p.rr_leg_2, sizeof(float)*3);
            return *this;
        }
    };

    struct LegState_t {
        std::array<JointState_t, 3> joint_states;
        std::array<half, 3> foot_target_position_rt_body{0.0_h,};
        std::array<half, 3> foot_position_rt_body{0.0_h,};
        std::array<half, 3> foot_velocity_rt_body{0.0_h,};
        std::array<half, 3> foot_force_vec_rt_body{0.0_h,};
        half foot_force = 0.0_h;
        /// \brief contact
        /// CONTACT_UNKNOWN = 0
        /// CONTACT_MADE = 1
        /// CONTACT_LOST = 2
        uint8_t contact;
    };

    struct LegStateArray_t {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;

        double time = 0.0;

        std::array<LegState_t, 4> leg_states;

        /// CommandContainer_t type identifier
        const unsigned char typeIdentifier = ID_LEG_STATE_ARRAY;

        const unsigned char tail1 =  128;
        const unsigned char tail2 =  127;
    };

}; // Motion

} // namespace RBQ_SDK

#endif // RBTYPES_HPP

