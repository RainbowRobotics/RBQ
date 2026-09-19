#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <rbq_sdk/nlohmann/json.hpp>
#include <rbq_sdk/dds/Client.hpp>

namespace rbq_sdk {

// service identity (mirrors unitree::robot::a2::ROBOT_SPORT_*)
const std::string ROBOT_SPORT_SERVICE_NAME = "sport";
const std::string ROBOT_SPORT_API_VERSION  = "1.0.0.1";

// api ids
constexpr int32_t ROBOT_SPORT_API_ID_DAMP            = 1001;
constexpr int32_t ROBOT_SPORT_API_ID_BALANCESTAND    = 1002;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE        = 1003;
constexpr int32_t ROBOT_SPORT_API_ID_STANDUP         = 1004;
constexpr int32_t ROBOT_SPORT_API_ID_STANDDOWN       = 1005;
constexpr int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND   = 1006;
constexpr int32_t ROBOT_SPORT_API_ID_EULER           = 1007;
constexpr int32_t ROBOT_SPORT_API_ID_MOVE            = 1008;
constexpr int32_t ROBOT_SPORT_API_ID_BODYPOSITION    = 1009;
constexpr int32_t ROBOT_SPORT_API_ID_SWITCHGAIT      = 1011;
constexpr int32_t ROBOT_SPORT_API_ID_BODYHEIGHT      = 1013;
constexpr int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL      = 1015;
constexpr int32_t ROBOT_SPORT_API_ID_LEFTSIDEGAIT    = 1016;
constexpr int32_t ROBOT_SPORT_API_ID_RIGHTSIDEGAIT   = 1017;
constexpr int32_t ROBOT_SPORT_API_ID_HANDSTAND       = 1018;
constexpr int32_t ROBOT_SPORT_API_ID_BIPEDSTAND      = 1019;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTFLIP       = 1020;
constexpr int32_t ROBOT_SPORT_API_ID_BACKFLIP        = 1021;
constexpr int32_t ROBOT_SPORT_API_ID_GETSTATE        = 1034;
constexpr int32_t ROBOT_SPORT_API_ID_SETAUTORECOVERY = 1040;

// gait/mode ids — argument values for SwitchGait
constexpr int ID_DEFAULT_MODE     = 0;
constexpr int ID_RUNNING_MODE     = 1;
constexpr int ID_CLIMB_MODE       = 2;

constexpr int ID_LEFT_SIDE_GAIT   = 6;
constexpr int ID_RIGHT_SIDE_GAIT  = 7;
constexpr int ID_HANDSTAND        = 8;
constexpr int ID_BIPED_STAND      = 9;
constexpr int ID_FRONT_FLIP       = 10;
constexpr int ID_BACK_FLIP        = 11;
constexpr int ID_RECOVERY         = 12;
constexpr int ID_BASE_HEIGHT_CTRL = 13;
constexpr int ID_POSE             = 14;
constexpr int ID_EULER            = 15;

// Default behavior is Unitree-compatible (blocking on every command, 1 s
// RPC timeout).  Call SetNoReply(true) — inherited from Client — to make
// every void-returning command fire-and-forget, useful for streaming over
// WiFi where the 1 s timeout would otherwise collapse the loop rate.
// GetState always blocks regardless of the toggle, since it needs the
// response payload to fill the map.
class SportClient : public Client {
public:
    SportClient() : Client(ROBOT_SPORT_SERVICE_NAME, /*enable_lease=*/false) {
        Init();
    }

    void Init() {
        SetApiVersion(ROBOT_SPORT_API_VERSION);
        for (int32_t id : { ROBOT_SPORT_API_ID_DAMP,          ROBOT_SPORT_API_ID_BALANCESTAND,
                            ROBOT_SPORT_API_ID_STOPMOVE,      ROBOT_SPORT_API_ID_STANDUP,
                            ROBOT_SPORT_API_ID_STANDDOWN,     ROBOT_SPORT_API_ID_RECOVERYSTAND,
                            ROBOT_SPORT_API_ID_EULER,         ROBOT_SPORT_API_ID_MOVE,
                            ROBOT_SPORT_API_ID_BODYPOSITION,  ROBOT_SPORT_API_ID_SWITCHGAIT,
                            ROBOT_SPORT_API_ID_BODYHEIGHT,    ROBOT_SPORT_API_ID_SPEEDLEVEL,
                            ROBOT_SPORT_API_ID_LEFTSIDEGAIT,  ROBOT_SPORT_API_ID_RIGHTSIDEGAIT,
                            ROBOT_SPORT_API_ID_HANDSTAND,     ROBOT_SPORT_API_ID_BIPEDSTAND,
                            ROBOT_SPORT_API_ID_FRONTFLIP,     ROBOT_SPORT_API_ID_BACKFLIP,
                            ROBOT_SPORT_API_ID_SETAUTORECOVERY,
                            ROBOT_SPORT_API_ID_GETSTATE }) {
            RegistApi(id, 0);
        }
    }

    int32_t Damp()          { return Call(ROBOT_SPORT_API_ID_DAMP,          std::string{}); }
    int32_t BalanceStand()  { return Call(ROBOT_SPORT_API_ID_BALANCESTAND,  std::string{}); }
    int32_t StopMove()      { return Call(ROBOT_SPORT_API_ID_STOPMOVE,      std::string{}); }
    int32_t StandUp()       { return Call(ROBOT_SPORT_API_ID_STANDUP,       std::string{}); }
    int32_t StandDown()     { return Call(ROBOT_SPORT_API_ID_STANDDOWN,     std::string{}); }
    int32_t RecoveryStand() { return Call(ROBOT_SPORT_API_ID_RECOVERYSTAND, std::string{}); }
    int32_t FrontFlip()     { return Call(ROBOT_SPORT_API_ID_FRONTFLIP,     std::string{}); }
    int32_t BackFlip()      { return Call(ROBOT_SPORT_API_ID_BACKFLIP,      std::string{}); }

    int32_t Euler(float roll, float pitch, float yaw) {
        return CallJson(ROBOT_SPORT_API_ID_EULER,
                        { {"x", roll}, {"y", pitch}, {"z", yaw} });
    }

    int32_t Move(float vx, float vy, float vyaw) {
        return CallJson(ROBOT_SPORT_API_ID_MOVE,
                        { {"x", vx}, {"y", vy}, {"z", vyaw} });
    }

    int32_t BodyPosition(float x, float y, float z, float yaw) {
        return CallJson(ROBOT_SPORT_API_ID_BODYPOSITION,
                        { {"x", x}, {"y", y}, {"z", z}, {"yaw", yaw} });
    }

    int32_t SwitchGait(int gait_type)     { return CallData(ROBOT_SPORT_API_ID_SWITCHGAIT,      gait_type); }
    int32_t BodyHeight(float height)      { return CallData(ROBOT_SPORT_API_ID_BODYHEIGHT,      height);    }
    int32_t SpeedLevel(int level)         { return CallData(ROBOT_SPORT_API_ID_SPEEDLEVEL,      level);     }
    int32_t LeftSideGait(int enter)       { return CallData(ROBOT_SPORT_API_ID_LEFTSIDEGAIT,    enter);     }
    int32_t RightSideGait(int enter)      { return CallData(ROBOT_SPORT_API_ID_RIGHTSIDEGAIT,   enter);     }
    int32_t HandStand(int enter)          { return CallData(ROBOT_SPORT_API_ID_HANDSTAND,       enter);     }
    int32_t BipedStand(int enter)         { return CallData(ROBOT_SPORT_API_ID_BIPEDSTAND,      enter);     }
    int32_t SetAutoRecovery(int switch_on){ return CallData(ROBOT_SPORT_API_ID_SETAUTORECOVERY, switch_on); }

    int32_t GetState(std::map<std::string, std::string>& state_map) {
        std::string data;
        int32_t code = Call(ROBOT_SPORT_API_ID_GETSTATE, std::string{}, data);
        if (code == RPC_OK && !data.empty()) {
            try {
                auto j = nlohmann::json::parse(data);
                for (auto& el : j.items())
                    state_map[el.key()] = el.value().get<std::string>();
            } catch (...) {
                // leave state_map untouched on malformed payload
            }
        }
        return code;
    }

private:
    template <typename T>
    int32_t CallData(int32_t api_id, T value) {
        return CallJson(api_id, { {"data", value} });
    }

    int32_t CallJson(int32_t api_id, const nlohmann::json& j) {
        return Call(api_id, j.dump());
    }
};

} // namespace rbq_sdk
