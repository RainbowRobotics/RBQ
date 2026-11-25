#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rbq_sdk/Joystick.hpp>
#include <rbq_sdk/dds/ArmClient.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/idl/rbq/ArmJointInfo_.hpp>
#include <rbq_sdk/idl/rbq/JointOwnershipCmd_.hpp>
#include <rbq_sdk/idl/rbq/MotionRef_.hpp>
#include <rbq_sdk/idl/rbq/TaskInfo_.hpp>

// Shared by the numbered RBA-1 arm examples (01_ ... 06_), like RB-Y1's 00_helper:
// every example starts the arm itself when it is not running, so each one runs on its own.
//
// The arm service answers when it accepts a command, not when the motion ends, so the waits below
// poll ArmClient::GetState. They go away once the service reports completion.
namespace arm_helper {

constexpr const char *kRobotAddress = "192.168.0.10";   // every RBQ ships at this address

// Printed by every example's usage.
constexpr const char *kTargetUsage =
    "  target     sim (-s)    the simulator on this PC - the default, so it can be left out\n"
    "             robot (-r)  the robot at 192.168.0.10, through whichever network device reaches it\n"
    "             or a network device name such as enp3s0, as in the rbq examples\n";

/// Network device that reaches `address`: asks the OS which local address it would send from (no packet goes
/// out), then finds the device holding that address on the same subnet. "lo" when `address` is this PC.
inline std::string InterfaceFor(const char *address)
{
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(9);
    if (inet_pton(AF_INET, address, &target.sin_addr) != 1) return "";
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return "";
    sockaddr_in local{};
    socklen_t len = sizeof(local);
    const bool routed = connect(fd, reinterpret_cast<sockaddr *>(&target), sizeof(target)) == 0 &&
                        getsockname(fd, reinterpret_cast<sockaddr *>(&local), &len) == 0;
    close(fd);
    if (!routed) return "";
    if (local.sin_addr.s_addr == target.sin_addr.s_addr) return "lo";

    std::string name;
    ifaddrs *list = nullptr;
    if (getifaddrs(&list) != 0) return "";
    for (ifaddrs *i = list; i; i = i->ifa_next) {
        if (!i->ifa_addr || !i->ifa_netmask || i->ifa_addr->sa_family != AF_INET) continue;
        const auto addr = reinterpret_cast<sockaddr_in *>(i->ifa_addr)->sin_addr.s_addr;
        const auto mask = reinterpret_cast<sockaddr_in *>(i->ifa_netmask)->sin_addr.s_addr;
        // same subnet as the robot - otherwise the route is a default gateway, not the robot's network
        if (addr == local.sin_addr.s_addr && (addr & mask) == (target.sin_addr.s_addr & mask)) {
            name = i->ifa_name;
            break;
        }
    }
    freeifaddrs(list);
    return name;
}

inline bool IsTarget(const std::string &t)
{
    return t == "sim" || t == "-s" || t == "robot" || t == "-r" ||
           (!t.empty() && t[0] != '-' && if_nametoindex(t.c_str()) != 0);
}

inline bool HelpAsked(int argc, char *argv[])
{
    return argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help");
}

/// True when the command line had no target and TargetGiven put "sim" in its place.
inline bool &TargetOmitted()
{
    static bool omitted = false;
    return omitted;
}

/// After a rejected first argument with no target given: says it was read as an argument, not as a target. A word
/// that is neither (a typo, a device this PC does not have) would otherwise look like a wrong argument only.
inline void PrintTargetHint(char *argv[])
{
    if (TargetOmitted())
        std::printf("[arm] no target given, so '%s' was read as an argument - a target is sim, robot or a network "
                    "device of this PC (ip -br link)\n", argv[2]);
}

/// First check of every example. `usage` is the example's full description: with -h / --help it is printed and the
/// example should exit (0). Without a target in argv[1] it puts "sim" there, so the rest of the example reads its
/// arguments from argv[2] on either way - only robot / -r (or a device name) reaches the robot.
inline bool TargetGiven(int &argc, char **&argv, const std::string &usage = "")
{
    if (HelpAsked(argc, argv)) {
        std::cout << usage;
        return false;
    }
    if (argc > 1 && IsTarget(argv[1])) return true;
    TargetOmitted() = true;
    static char sim[] = "sim";
    static std::vector<char *> withSim;
    withSim.assign(argv, argv + argc);
    withSim.insert(withSim.begin() + 1, sim);
    withSim.push_back(nullptr);
    argc += 1;
    argv = withSim.data();
    return true;
}

/// A wrong argument: says which one and why, then only the usage lines. Returns the exit code (1).
inline int ArgError(char *argv[], const std::string &what, const std::string &usage)
{
    std::printf("[arm] %s\n", what.c_str());
    // the target hint only when the first argument is the one rejected (the message quotes it) - not for 'rel 0.1'
    if (argv[1] != nullptr && argv[2] != nullptr && what.find(std::string("'") + argv[2] + "'") != std::string::npos)
        PrintTargetHint(argv);
    std::printf("\n%s\n  (%s -h for the full description)\n", usage.substr(0, usage.find("\n\n")).c_str(), argv[0]);
    return 1;
}

/// Whole-string number parsing - "1.5x" or "" is not a number.
inline bool ParseFloat(const char *s, float &out)
{
    try { size_t n = 0; out = std::stof(s, &n); return n == std::string(s).size(); }
    catch (...) { return false; }
}
/// A `key=<number>` option such as vel=30: true and the number when `arg` is one for `key`.
inline bool ParseOption(const char *arg, const char *key, float &out)
{
    const std::string a(arg), k = std::string(key) + "=";
    return a.compare(0, k.size(), k) == 0 && ParseFloat(a.c_str() + k.size(), out);
}

/// The joint range [deg] MoveJ by angles accepts - provisional until the actuator specifications are final.
/// 1 deg in rad - the examples take degrees on the command line, ArmClient takes radians (like SportClient)
constexpr float kDeg2Rad = 0.0174532925f;
constexpr float kJointMinDeg[6] = {-360.f,   0.f, -180.f, -360.f, -360.f, -360.f};
constexpr float kJointMaxDeg[6] = { 360.f, 180.f,    2.f,  360.f,  360.f,  360.f};

inline bool ParseInt(const char *s, int &out)
{
    try { size_t n = 0; out = std::stoi(s, &n); return n == std::string(s).size(); }
    catch (...) { return false; }
}

/// A number as printf %g - the dry-run line below prints numbers the way Python's f"{v:g}" does.
inline std::string Num(float v)
{
    char b[32];
    std::snprintf(b, sizeof b, "%g", v);
    return b;
}

/// With RBQ_ARM_DRY_RUN=1 in the environment (rbq_sdk/scripts/arm_examples_check.py): prints the call the example would
/// make - "[dry-run] <target> <call>" - and returns true, so the example exits there without connecting. Each example
/// calls it after reading its arguments; the Python examples print the same line.
inline bool DryRun(const std::string &target, const std::string &call)
{
    const char *on = std::getenv("RBQ_ARM_DRY_RUN");
    if (on == nullptr || std::string(on) != "1") return false;
    const std::string t = (target == "sim" || target == "-s") ? "sim" : (target == "robot" || target == "-r") ? "robot" : target;
    std::printf("[dry-run] %s %s\n", t.c_str(), call.c_str());
    return true;
}

/// Points the SDK at `target` (sim / -s, robot / -r, or a device name). Prints where it connects; false if it cannot.
inline bool InitChannel(const std::string &target)
{
    std::string iface;
    if (target == "sim" || target == "-s") {
        iface = "lo";
        std::printf("[arm] target: simulator (lo)\n");
    } else if (target == "robot" || target == "-r") {
        iface = InterfaceFor(kRobotAddress);
        if (iface.empty()) {
            std::printf("[arm] no network device on the robot's network (%s) - connect to the robot's Wi-Fi or cable\n",
                        kRobotAddress);
            return false;
        }
        std::printf("[arm] target: robot %s via %s\n", kRobotAddress, iface.c_str());
    } else {
        if (target.empty() || target[0] == '-' || if_nametoindex(target.c_str()) == 0) {
            std::printf("[arm] unknown target '%s'\n%s", target.c_str(), kTargetUsage);
            return false;
        }
        iface = target;
        std::printf("[arm] target: network device %s\n", iface.c_str());
    }
    rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, iface);
    return true;
}

inline const char *CodeText(int32_t code)
{
    switch (code) {
    case rbq_sdk::RPC_OK: return "ok";
    case 3204: return "bad parameter";
    case 3205: return "rejected: arm joints held by a joint-level app, or E-STOP";
    case rbq_sdk::ARM_ERR_BUSY: return "busy: the arm is still moving";
    case rbq_sdk::ARM_ERR_NOT_READY: return "not ready: task-space control is off";
    case rbq_sdk::ARM_ERR_CANCELED: return "canceled by Stop";
    case rbq_sdk::ARM_ERR_WAIT_TIMEOUT: return "still moving when the wait ran out";
    case rbq_sdk::ARM_ERR_COLLISION: return "stopped: collision torque limit (GetState collision_joint)";
    case rbq_sdk::ARM_ERR_PATH_BLOCKED: return "not started: the path would hit the robot body or legs";
    case rbq_sdk::ARM_ERR_WORKSPACE: return "stopped: gripper workspace limit";
    case rbq_sdk::ARM_ERR_ESTOP: return "arm emergency stop - clear it: 02_arm_power [target] clear";
    case rbq_sdk::ARM_ERR_TOO_FAST: return "too fast: the time given is shorter than the speed limits allow";
    case rbq_sdk::RPC_ERR_CLIENT_API_TIMEOUT: return "no answer - Network running? right target?";
    default: return "error";
    }
}

/// After a move refused with ARM_ERR_TOO_FAST: the time given was within the range but would move the arm faster than
/// `limits` allow - says so, and the shortest time that move needs (GetState motion_time).
inline void PrintTooFast(rbq_sdk::ArmClient &arm, int32_t code, float seconds, const char *limits)
{
    std::map<std::string, std::string> state;
    if (code != rbq_sdk::ARM_ERR_TOO_FAST || arm.GetState(state) != rbq_sdk::RPC_OK) return;
    std::printf("[arm] %s s is too fast for this move - it would go past the speed limits (max %s).\n"
                "      This move needs at least %.2f s: give that or longer, or leave the time out and the robot picks it\n",
                Num(seconds).c_str(), limits, std::stof(state["motion_time"]));
}

/// Prints "<what> -> <code> (<meaning>)" and returns true when the code is 0.
inline bool Check(const char *what, int32_t code)
{
    std::printf("[arm] %s -> %d (%s)\n", what, code, CodeText(code));
    return code == rbq_sdk::RPC_OK;
}

/// Check for a step the user did not ask for (the state read before a move): prints only when it fails.
inline bool CheckQuiet(const char *what, int32_t code)
{
    return code == rbq_sdk::RPC_OK || Check(what, code);
}

/// Speed options of Move(): the time the caller gave (0 = the robot picked it) and the maximums it is held to, for the
/// "too fast" explanation. timed: the robot reports this move's time (MoveJ by angles, MoveL family) - shown when done.
struct MoveInfo {
    bool        timed   = false;
    float       seconds = 0.f;
    const char *limits  = nullptr;
};

/// A motion call and its end. Done: one line, "[arm] <what> done" (+ "in 0.91 s" for a timed move). Refused or stopped:
/// the code, what it means and - when the time given was too short - the time the move needs.
inline bool Move(rbq_sdk::ArmClient &arm, const std::string &what, int32_t code, const MoveInfo &info = {}, float waitSec = 65.f)
{
    if (code != rbq_sdk::RPC_OK) {
        Check(what.c_str(), code);
        if (info.limits) PrintTooFast(arm, code, info.seconds, info.limits);
        return false;
    }
    const int32_t done = arm.Wait(waitSec);
    if (done != rbq_sdk::RPC_OK) return Check((what + " did not finish").c_str(), done);
    std::map<std::string, std::string> state;
    if (info.timed && arm.GetState(state) == rbq_sdk::RPC_OK)
        std::printf("[arm] %s done in %.2f s\n", what.c_str(), std::atof(state["motion_time"].c_str()));
    else
        std::printf("[arm] %s done\n", what.c_str());
    return true;
}

/// Polls GetState until state[key] == value or the timeout passes.
inline bool WaitFor(rbq_sdk::ArmClient &arm, const std::string &key, const std::string &value, double seconds)
{
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        std::map<std::string, std::string> state;
        if (arm.GetState(state) == rbq_sdk::RPC_OK && state[key] == value) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::printf("[arm] gave up waiting for %s = %s after %.0f s\n", key.c_str(), value.c_str(), seconds);
    return false;
}

/// MoveJ poses as the examples accept them: a number, a short name, or the pose name itself (any case).
struct PoseChoice { const char *number; const char *shortName; const char *pose; const char *what; };
// Joint angles J0..J5 [deg] of RBA-1 (the arm controller's pose table, read back in the simulator 11-18).
inline const PoseChoice kPoses[] = {
    {"0", "home",     rbq_sdk::ARM_POSE_FOLDING,      "0 0 0 0 0 0           home / walking pose"},
    {"1", "ready",    rbq_sdk::ARM_POSE_READY,        "0 140 -70 0 -70 0     start pose for task-space moves"},
    {"2", "straight", rbq_sdk::ARM_POSE_STRAIGHT,     "0 90 -180 0 0 0       arm stretched straight up"},
    {"3", "pickup",   rbq_sdk::ARM_POSE_PICKUP_READY, "0 150 -100 0 40 0     to pick up an object from the ground"},
};

inline std::string Lower(std::string s)
{
    for (auto &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

/// Pose name for "0" / "home" / "Folding" / "folding" ...; empty when it is none of them.
inline std::string PoseFromArg(const std::string &arg)
{
    const std::string a = Lower(arg);
    for (const auto &p : kPoses)
        if (a == p.number || a == p.shortName || a == Lower(p.pose)) return p.pose;
    return "";
}

/// "    0  home      0 0 0 0 0 0           home / walking pose" ... for the usage text: number, name, J0..J5 [deg], what.
inline std::string PoseUsage()
{
    std::string text;
    char line[128];
    for (const auto &p : kPoses) {
        std::snprintf(line, sizeof(line), "    %s  %-9s %s\n", p.number, p.shortName, p.what);
        text += line;
    }
    return text;
}


/// First GetState of a fresh client. Right after start-up DDS has not yet matched the client with the robot's
/// service, so the first answers can be lost (3104); asks again for up to `seconds` before giving up.
inline bool FirstState(rbq_sdk::ArmClient &arm, std::map<std::string, std::string> &state, double seconds = 3.0)
{
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    int32_t code;
    while ((code = arm.GetState(state)) == rbq_sdk::RPC_ERR_CLIENT_API_TIMEOUT && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return CheckQuiet("GetState", code);
}

/// Starts the arm if it is not running (AutoStart), then moves it to Ready when toReady is set. With onlyIfFolded it
/// goes to Ready only from the folded pose (Folding / Packing, also where AutoStart leaves it) and otherwise starts
/// where the arm is - for a joint-level test right after a move to the pose you want.
/// In an emergency stop it never calls AutoStart (that would end the stop): it returns false with a hint, or true
/// without doing anything when inEstopOk is set (the gripper still works then).
inline bool InitializeArm(rbq_sdk::ArmClient &arm, bool toReady = true, bool inEstopOk = false, bool onlyIfFolded = false)
{
    std::map<std::string, std::string> state;
    if (!FirstState(arm, state)) return false;
    if (state["sdk_slot"] != "-1") {
        std::printf("[arm] app slot %s holds the arm joints - stop that app or call ReleaseJoints()\n",
                    state["sdk_slot"].c_str());
        return false;
    }
    if (state["estop"] == "1") {
        if (inEstopOk) return true;
        std::printf("[arm] the arm is in emergency stop - clear it first: 02_arm_power [target] clear\n");
        return false;
    }
    if (state["con_start"] != "1") {
        if (!CheckQuiet("AutoStart", arm.AutoStart())) return false;
        if (!WaitFor(arm, "con_start", "1", 60.0)) return false;
        std::printf("[arm] arm started (AutoStart)\n");
        state.clear();
        arm.GetState(state);
    }
    const bool folded = state["pose"] == rbq_sdk::ARM_POSE_FOLDING || state["pose"] == rbq_sdk::ARM_POSE_PACKING;
    if (toReady && state["pose"] != rbq_sdk::ARM_POSE_READY && (!onlyIfFolded || folded))
        return Move(arm, "MoveJ Ready", arm.MoveJ(rbq_sdk::ARM_POSE_READY));
    return true;
}

/// Task-space moves (MoveL family) need task-space control on, and the arm turns it on only in the Ready pose.
/// Goes through Ready and ControlStart only when it is off; when it is already on, the arm moves from where it is.
inline bool StartTaskControl(rbq_sdk::ArmClient &arm)
{
    std::map<std::string, std::string> state;
    if (!CheckQuiet("GetState", arm.GetState(state))) return false;
    if (state["task_control"] == "1") return true;
    if (state["pose"] != rbq_sdk::ARM_POSE_READY && !Move(arm, "MoveJ Ready", arm.MoveJ(rbq_sdk::ARM_POSE_READY))) return false;
    if (!CheckQuiet("ControlStart", arm.ControlStart()) || !WaitFor(arm, "task_control", "1", 3.0)) return false;
    std::printf("[arm] task-space control on (ControlStart)\n");
    return true;
}

// ── Joint-level (Lv0) stream, used by 07_arm_low_level ──────────────────────────────────────────────────────────

constexpr int   kArmJoints   = 6;      // J0..J5 - the gripper (GRP) is not a joint-level joint, see JointStream::Gripper
constexpr float kPosRateMax  = 2.0f;   // [rad/s] the robot moves a joint reference no faster than this

/// One snapshot of the arm: rt/rbq/arm_joint for the six arm joints and the gripper, rt/rbq/arm_ee for the end effector.
struct ArmState {
    float q[kArmJoints];         // measured position [rad]
    float dq[kArmJoints];        // measured velocity [rad/s]
    float torque[kArmJoints];    // measured torque [Nm]
    float qRef[kArmJoints];      // reference the arm follows now [rad]
    float kp[kArmJoints];        // gains the arm applies now [Nm/rad]
    float kd[kArmJoints];        //                           [Nm*s/rad]
    int   owner[kArmJoints];     // app slot that holds the joint
    float gripper;               // gripper link angle [rad]
    float eePos[3];              // measured end-effector position [m], body frame - compare your own FK with it
    float eeRpy[3];              // measured end-effector roll / pitch / yaw [rad], body frame
    bool  eeValid;               // false until the first rt/rbq/arm_ee has arrived
    float gravity[kArmJoints];   // torque that holds the arm against gravity at q [Nm] (rt/rbq/arm_gravity) - the
                                 // robot's arm model with the body taken as level; add it in SendTorque()
    bool  gravityValid;          // false until the first rt/rbq/arm_gravity has arrived
};

/// Damping SendTorque() applies when no kd is given [Nm*s/rad] - about a quarter of the arm's default kd, enough to
/// stop the arm drifting on its own and soft enough to push by hand.
static constexpr float kTorqueModeKd[kArmJoints] = {5.0f, 5.0f, 2.5f, 2.5f, 2.5f, 2.5f};

/// Takes the six arm joints as one app slot and streams joint references to them. Does the parts every joint-level
/// program needs - start the arm, reference before ownership, keep the arm's gains, give the joints back where they
/// are - so a program only computes joint references and calls Send().
///
/// The robot moves each reference at most kPosRateMax toward what you send, so a jump (say 90 deg in one tick) makes
/// the arm travel there at that speed instead of snapping. Send() prints a warning when it sees one.
class JointStream {
public:
    explicit JointStream(int slot = 22)
        : pubRef_("rt/rbq/ref/motion/_" + std::to_string(slot)),
          pubOwner_("rt/rbq/cmd/motion/owner/_" + std::to_string(slot)),
          subState_([this](const rbq_msgs::msg::dds_::ArmJointInfo_ &m) {
                        std::lock_guard<std::mutex> lock(mx_);
                        state_ = m;
                        got_ = true;
                    }, "rt/rbq/arm_joint"),
          subEe_([this](const rbq_msgs::msg::dds_::TaskInfo_ &m) {
                     std::lock_guard<std::mutex> lock(mx_);
                     ee_ = m;
                     gotEe_ = true;
                 }, "rt/rbq/arm_ee"),
          subGravity_([this](const rbq_msgs::msg::dds_::ArmJointInfo_ &m) {
                          std::lock_guard<std::mutex> lock(mx_);
                          gravity_ = m;
                          gotGravity_ = true;
                      }, "rt/rbq/arm_gravity"),
          slot_(slot)
    {
        rpc_.SetTimeout(2.0f);
    }
    ~JointStream() { Stop(); }

    /// Starts the arm (and goes to Ready when toReady), waits for the arm state, then takes the joints with the
    /// reference the arm already follows and the gains it already applies. False, with a message, when it cannot.
    /// onlyIfFolded: go to Ready only from the folded pose, else take the joints where the arm is (InitializeArm).
    bool Start(bool toReady = true, bool onlyIfFolded = false)
    {
        if (!InitializeArm(rpc_, toReady, false, onlyIfFolded)) return false;
        for (int i = 0; i < 100; i++) {   // DDS discovery and the first arm state
            {
                std::lock_guard<std::mutex> lock(mx_);
                if (got_) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        const ArmState s = State();
        {
            std::lock_guard<std::mutex> lock(mx_);
            if (!got_) {
                std::printf("[arm] nothing on rt/rbq/arm_joint (Network not running, or the wrong target?)\n");
                return false;
            }
        }
        for (int i = 0; i < kArmJoints; i++) {
            if (s.kp[i] <= 0.0f) {
                std::printf("[arm] J%d has no position gain (kp = 0): the arm is not holding a pose\n", i);
                return false;
            }
            kp_[i] = s.kp[i];
            kd_[i] = s.kd[i];
        }
        // reference first, then ownership - the arm never sees a stale reference
        Send(s.qRef);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        SetOwnership(true);
        started_ = true;
        std::printf("[arm] arm joints taken as app slot %d\n", slot_);
        return true;
    }

    ArmState State()
    {
        std::lock_guard<std::mutex> lock(mx_);
        ArmState s{};
        for (int i = 0; i < kArmJoints; i++) {
            const auto &j = state_.joint()[i];
            s.q[i] = j.pos();  s.dq[i] = j.vel();  s.torque[i] = j.torque();  s.qRef[i] = j.ref_position();
            s.kp[i] = j.kp();  s.kd[i] = j.kd();   s.owner[i] = j.owner();
        }
        s.gripper = state_.joint()[kArmJoints].pos();
        for (int i = 0; i < 3; i++) {
            s.eePos[i] = ee_.pos()[i];
            s.eeRpy[i] = ee_.rpy()[i];
        }
        s.eeValid = gotEe_;
        for (int i = 0; i < kArmJoints; i++) s.gravity[i] = gravity_.joint()[i].torque();
        s.gravityValid = gotGravity_;
        return s;
    }

    /// Gains for the next Send() calls [Nm/rad, Nm*s/rad] - the stiffness and damping of each joint around the reference.
    /// Start() sets the ones the arm already applies (State().kp / kd); scale those rather than starting from nothing.
    /// The robot clamps kp to 0..~53 000 (J0, J1) / ~24 000 (J2..J5) Nm/rad and kd to 0..~420 / ~190 Nm*s/rad.
    void SetGains(const float kp[kArmJoints], const float kd[kArmJoints])
    {
        std::copy(kp, kp + kArmJoints, kp_);
        std::copy(kd, kd + kArmJoints, kd_);
    }

    /// Joint references [rad], and optionally feed-forward velocity [rad/s] and torque [Nm] (0 when left out).
    void Send(const float q[kArmJoints], const float *vel = nullptr, const float *torque = nullptr)
    {
        rbq_msgs::msg::dds_::MotionRef_ m;
        for (int i = 0; i < kArmJoints; i++) {
            auto &r = m.arm_joint()[i];
            r.pos(q[i]);
            r.vel(vel ? vel[i] : 0.0f);
            r.torque(torque ? torque[i] : 0.0f);
            r.kp(kp_[i]);
            r.kd(kd_[i]);
        }
        WarnOnJump(q);
        pubRef_.write(m);
        torqueMode_ = false;
    }

    /// Torque mode: joint torques [Nm] with no position gain (kp 0) and damping kd [Nm*s/rad] (kTorqueModeKd when
    /// left out). Nothing holds the arm but what you send - add State().gravity, or the arm falls. The robot clamps each
    /// torque to +-60 Nm (J0, J1) / +-25 Nm (J2..J5). If the torques stop for 0.2 s (program hung or killed) the robot
    /// sends the gravity torque with damping, so the arm floats where it is. Send() goes back to position control.
    void SendTorque(const float torque[kArmJoints], const float *kd = nullptr)
    {
        const ArmState s = State();
        rbq_msgs::msg::dds_::MotionRef_ m;
        for (int i = 0; i < kArmJoints; i++) {
            auto &r = m.arm_joint()[i];
            r.pos(s.q[i]);   // no position gain, so this only keeps the reference next to the arm for when Send() takes over
            r.vel(0.0f);
            r.torque(torque[i]);
            r.kp(0.0f);
            r.kd(kd ? kd[i] : kTorqueModeKd[i]);
            lastQ_[i] = s.q[i];
        }
        pubRef_.write(m);
        torqueMode_ = true;
    }

    /// Gripper open / close / stop through ArmClient. Returns at once - the call runs on its own thread so a
    /// 500 Hz loop is not held up; the result is printed. A call made while the previous one is still running is skipped.
    enum class Grip { Open, Close, Stop };
    void Gripper(Grip g)
    {
        if (grip_.valid() && grip_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            std::printf("[arm] gripper call skipped - the previous one has not been answered yet\n");
            return;
        }
        grip_ = std::async(std::launch::async, [this, g] {
            switch (g) {
            case Grip::Open:  CheckQuiet("GripperOpen",  rpc_.GripperOpen());  break;
            case Grip::Close: CheckQuiet("GripperClose", rpc_.GripperClose()); break;
            case Grip::Stop:  CheckQuiet("GripperStop",  rpc_.GripperStop());  break;
            }
        });
    }

    /// Holds the last reference for 0.2 s, then gives the joints back - the arm stays where the last reference is.
    /// After SendTorque() that is where the arm is, held with the position gains and the gravity torque.
    void Stop()
    {
        if (grip_.valid()) grip_.wait();
        if (!started_) return;
        started_ = false;
        float hold[kArmJoints] = {};
        if (torqueMode_) {
            const ArmState s = State();
            std::copy(s.gravity, s.gravity + kArmJoints, hold);
        }
        for (int t = 0; t < 100; t++) {
            Send(lastQ_, nullptr, hold);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        SetOwnership(false);
        std::printf("[arm] arm joints handed back to the arm controller\n");
    }

private:
    void SetOwnership(bool on)
    {
        rbq_msgs::msg::dds_::JointOwnershipCmd_ own;
        for (int i = 0; i < kArmJoints; i++) own.arm_joint()[i] = on;
        pubOwner_.write(own);
    }

    // A reference that moves faster than the robot follows is almost always a mistake (a wrong unit, an IK branch
    // flip). Says so once per joint until the references are back to a speed the arm follows.
    void WarnOnJump(const float q[kArmJoints])
    {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::max(std::chrono::duration<float>(now - lastSend_).count(), 0.002f);
        for (int i = 0; i < kArmJoints; i++) {
            const float step = std::abs(q[i] - lastQ_[i]);
            const bool jump = sentOnce_ && step > kPosRateMax * dt + 0.01f;
            if (jump && !jumpWarned_[i])
                std::printf("[arm] J%d reference jumped %.1f deg in %.0f ms - the robot moves it at %.0f deg/s\n",
                            i, step * 57.2958f, dt * 1000.0f, kPosRateMax * 57.2958f);
            jumpWarned_[i] = jump;
            lastQ_[i] = q[i];
        }
        lastSend_ = now;
        sentOnce_ = true;
    }

    rbq_sdk::ArmClient rpc_;
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::MotionRef_>         pubRef_;
    rbq_sdk::Publisher<rbq_msgs::msg::dds_::JointOwnershipCmd_> pubOwner_;
    std::mutex mx_;
    rbq_msgs::msg::dds_::ArmJointInfo_ state_;
    bool got_ = false;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::ArmJointInfo_> subState_;
    rbq_msgs::msg::dds_::TaskInfo_ ee_;
    bool gotEe_ = false;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::TaskInfo_> subEe_;
    rbq_msgs::msg::dds_::ArmJointInfo_ gravity_;
    bool gotGravity_ = false;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::ArmJointInfo_> subGravity_;
    const int slot_;
    bool  started_ = false;
    bool  torqueMode_ = false;
    float kp_[kArmJoints]{}, kd_[kArmJoints]{};
    float lastQ_[kArmJoints]{};
    bool  jumpWarned_[kArmJoints]{};
    bool  sentOnce_ = false;
    std::chrono::steady_clock::time_point lastSend_{};
    std::future<void> grip_;
};

// ---- Gamepad on this PC, for a Controller that reads one (my_arm) ----------------------------------------------

/// One gamepad reading for a Controller, taken once per tick. With no gamepad everything is 0 / false and connected
/// is false. Button numbers are rbq_sdk/Joystick.hpp's: PAD_A, PAD_B, PAD_X, PAD_Y, PAD_LB, PAD_RB, PAD_BACK, PAD_START
/// (xpad / Logitech F710 in X mode - another pad may number them differently; print axis[] and held[] to see).
struct Pad {
    static constexpr int kButtons = 16;
    static constexpr int kAxes = 8;
    bool  connected = false;
    float lx = 0, ly = 0;              // left stick  -1..1 after a 0.1 dead zone: + = right, + = up
    float rx = 0, ry = 0;              // right stick
    float axis[kAxes] = {};            // every axis as the driver reports it, -1..1 (Linux: stick up is -; a trigger
                                       // is -1 released .. 1 pressed)
    bool  held[kButtons] = {};         // true while the button is down
    bool  pressed[kButtons] = {};      // true on the one tick the button went down
};

namespace {   // Joystick (rbq_sdk/Joystick.hpp) lives in an unnamed namespace, so the class that holds one does too

/// Reads a Linux gamepad (/dev/input/js*) without blocking - call Read() once per tick.
class Gamepad {
public:
    /// device: a path such as /dev/input/js0, or "" for $JOY or the first gamepad found. False, with a message, when
    /// there is none.
    bool Open(const std::string &device)
    {
        std::string arg = device;
        char *argv[] = {nullptr, nullptr, arg.data()};
        const std::string path = PickJoyPath(device.empty() ? 2 : 3, argv);
        if (path.empty()) {
            PrintNoJoystickHint();
            return false;
        }
        joy_ = std::make_unique<Joystick>(path);
        if (!joy_->Open()) {
            joy_.reset();
            return false;
        }
        return true;
    }

    Pad Read()
    {
        Pad p;
        if (!joy_) return p;
        joy_->Poll();
        p.connected = true;
        for (int i = 0; i < Pad::kAxes; i++) p.axis[i] = joy_->Ax(i);
        p.lx =  Deadzone(joy_->Ax(AXIS_LEFT_X), kDead);
        p.ly = -Deadzone(joy_->Ax(AXIS_LEFT_Y), kDead);
        p.rx =  Deadzone(joy_->Ax(AXIS_RIGHT_X), kDead);
        p.ry = -Deadzone(joy_->Ax(kAxisRightY), kDead);
        for (int i = 0; i < Pad::kButtons; i++) {
            p.held[i] = joy_->Btn(i);
            p.pressed[i] = joy_->Edge(i);
        }
        joy_->EndFrame();
        return p;
    }

private:
    static constexpr float kDead = 0.1f;
    static constexpr int kAxisRightY = 4;   // Joystick.hpp names the other three stick axes
    std::unique_ptr<Joystick> joy_;
};

} // namespace

} // namespace arm_helper
