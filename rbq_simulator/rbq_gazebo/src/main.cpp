#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double_v.pb.h>

#include <rbq_sdk/dds/ChannelFactory.hpp>

#include "Bridge.h"
#include "VisionPublisher.h"

// Registers the embedded RBQ system plugin
namespace rbq_gazebo_system { bool addToServer(gz::sim::Server &server, const rbq_gazebo::RobotSpec &spec); }

namespace fs = std::filesystem;

namespace
{
std::atomic<bool> g_running{true};
void onSignal(int) { g_running = false; }

std::string readFile(const std::string &path)
{
    std::ifstream f(path);
    if (!f) return {};
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

void prependEnv(const char *key, const std::string &value)
{
    const char *cur = std::getenv(key);
    std::string v = value;
    if (cur && cur[0] != '\0') v += std::string(":") + cur;
    setenv(key, v.c_str(), 1);
}

// ---- Camera extrinsics for the DDS TF publisher ------------------------------
bool camPoseFromSdf(const std::string &poseText, rbq_gazebo::CamPose &out)
{
    std::array<double, 6> v{};
    std::stringstream ss(poseText);
    for (double &val : v) {
        if (!(ss >> val)) return false;
    }
    // Optical frame axes expressed in the gz camera frame.
    Eigen::Matrix3d gzToOpt;
    gzToOpt << 0,  0, 1,
              -1,  0, 0,
               0, -1, 0;
    const Eigen::Matrix3d R = (Eigen::AngleAxisd(v[5], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(v[4], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(v[3], Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix() *
                              gzToOpt;
    const Eigen::Quaterniond q(R);
    out.p[0] = v[0]; out.p[1] = v[1]; out.p[2] = v[2];
    out.q[0] = q.w(); out.q[1] = q.x(); out.q[2] = q.y(); out.q[3] = q.z();
    return true;
}

// Extract each camera's depth/color sensor pose from the spawned camera fragment.
rbq_gazebo::CamPoseTable parseCamPoses(const std::string &camFrag)
{
    rbq_gazebo::CamPoseTable table{};
    static const char *const kCamNames[rbq_gazebo::kNumTfCams] = {"BT0", "BT1", "BT2",
                                                                  "BT3", "FT0", "RR0"};
    auto poseOf = [&](const std::string &sensor, rbq_gazebo::CamPose &out) {
        const auto s = camFrag.find("<sensor name=\"" + sensor + "\"");
        if (s == std::string::npos) return false;
        const auto b = camFrag.find("<pose>", s);
        const auto e = camFrag.find("</pose>", b);
        if (b == std::string::npos || e == std::string::npos) return false;
        return camPoseFromSdf(camFrag.substr(b + 6, e - (b + 6)), out);
    };
    for (int i = 0; i < rbq_gazebo::kNumTfCams; i++) {
        table[i].valid = poseOf(std::string(kCamNames[i]) + "_depth", table[i].depth) &&
                         poseOf(std::string(kCamNames[i]) + "_color", table[i].color);
        if (!table[i].valid)
            std::fprintf(stderr, "[Gazebo] camera %s pose not found; TF not published\n",
                         kCamNames[i]);
    }
    return table;
}

// Parse <world name="..."> from the SDF text.
std::string parseWorldName(const std::string &sdf, const std::string &fallback)
{
    const std::string tag = "<world";
    auto p = sdf.find(tag);
    if (p == std::string::npos) return fallback;
    auto np = sdf.find("name=\"", p);
    if (np == std::string::npos) return fallback;
    np += 6;
    auto e = sdf.find('"', np);
    if (e == std::string::npos) return fallback;
    return sdf.substr(np, e - np);
}

// Spawn pose: x y z roll pitch yaw. Only the drop height is variant-specific.
void spawnPose(double spawnZ, double out[6])
{
    const double pose[6] = {0.0, 0.0, spawnZ, 0.0, 0.0, 0.0};
    std::memcpy(out, pose, sizeof(pose));
}

void rpyToQuat(double r, double p, double y, double &qw, double &qx, double &qy, double &qz)
{
    const double cy = std::cos(y * 0.5), sy = std::sin(y * 0.5);
    const double cp = std::cos(p * 0.5), sp = std::sin(p * 0.5);
    const double cr = std::cos(r * 0.5), sr = std::sin(r * 0.5);
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

bool spawnRobot(gz::transport::Node &node, const std::string &worldName,
                const std::string &robotUrdfPath, const std::string &modelName,
                const double pose[6])
{
    const std::string createSrv = "/world/" + worldName + "/create";

    gz::msgs::EntityFactory req;
    req.set_sdf_filename(robotUrdfPath);
    req.set_name(modelName);
    req.set_allow_renaming(false);
    auto *p = req.mutable_pose();
    p->mutable_position()->set_x(pose[0]);
    p->mutable_position()->set_y(pose[1]);
    p->mutable_position()->set_z(pose[2]);
    double qw, qx, qy, qz;
    rpyToQuat(pose[3], pose[4], pose[5], qw, qx, qy, qz);
    p->mutable_orientation()->set_w(qw);
    p->mutable_orientation()->set_x(qx);
    p->mutable_orientation()->set_y(qy);
    p->mutable_orientation()->set_z(qz);

    // Wait for the create service, then request the spawn.
    for (int attempt = 0; attempt < 50 && g_running; ++attempt) {
        gz::msgs::Boolean rep;
        bool result = false;
        if (node.Request(createSrv, req, 2000, rep, result) && result && rep.data()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

bool waitPluginReady(gz::transport::Node &node)
{
    gz::msgs::Double_V rep;
    for (int attempt = 0; attempt < 100 && g_running; ++attempt) {
        bool result = false;
        if (node.Request("/rbq/read_states", 1000, rep, result) && result) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

}  // namespace

int main(int argc, char *argv[])
{
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGHUP, onSignal);

    std::string world = "empty";
    std::string lidar = "none";
    std::string iface = "lo";
    bool headless = false;
    rbq_gazebo::RobotSpec spec = rbq_gazebo::rbqSpec();
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--world" && i + 1 < argc) world = argv[++i];
        else if (a == "--wheel") spec = rbq_gazebo::rbqWheelSpec();
        else if (a == "--livox") lidar = "livox";
        else if (a == "--ouster") lidar = "ouster";
        else if ((a == "-i" || a == "--interface") && i + 1 < argc) iface = argv[++i];
        else if (a == "--headless") headless = true;
        else if (a == "--partition" && i + 1 < argc) setenv("IGN_PARTITION", argv[++i], 1);
        else if (a == "--cyclonedds-uri" && i + 1 < argc) setenv("CYCLONEDDS_URI", argv[++i], 1);
        else if (a == "--help") {
            std::printf("Usage: Gazebo [--wheel] [--world <name>] [--livox|--ouster] "
                        "[-i <iface>] [--headless]\n"
                        "  --wheel: wheeled RBQ (12 leg joints + 4 wheels); default is the quadruped.\n"
                        "  LiDAR is opt-in: default none, --livox (2x) or --ouster (1x).\n"
                        "  (Add sensors by editing resources/models/sensor/user_sensor.urdf — spliced automatically.)\n");
            return 0;
        }
    }

    // Init rbq_sdk DDS channel (domain 0 + iface)
    rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, iface);
    std::printf("[Gazebo] DDS init (domain 0, interface '%s').\n", iface.c_str());

    // Bring up bridge DDS endpoints
    rbq_gazebo::startBridgeDdsEndpoints();
    std::printf("[Gazebo] bridge DDS endpoints up early (warming before world load).\n");

    // Resolve paths relative to the executable.
    fs::path exeDir;
    {
        std::error_code ec;
        fs::path self = fs::read_symlink("/proc/self/exe", ec);
        exeDir = ec ? fs::current_path() : self.parent_path();
    }
    // Locate the package resources: worlds + URDFs + model.config + sensor meshes.
    fs::path resourcesDir;
    const std::vector<fs::path> candidates = {
        exeDir / ".." / "rbq_simulator" / "rbq_gazebo" / "resources",
        exeDir / ".." / "resources",
        exeDir / "resources",
#ifdef RBQ_GAZEBO_RESOURCE_DIR
        fs::path(RBQ_GAZEBO_RESOURCE_DIR),
#endif
    };
    // Pick the first candidate with a worlds/ subdir.
    for (const auto &c : candidates) {
        if (fs::exists(c / "worlds")) { resourcesDir = fs::weakly_canonical(c); break; }
    }
#ifdef RBQ_GAZEBO_RESOURCE_DIR
    if (resourcesDir.empty()) resourcesDir = fs::weakly_canonical(fs::path(RBQ_GAZEBO_RESOURCE_DIR));
#endif
    if (resourcesDir.empty())
        resourcesDir = fs::weakly_canonical(exeDir / ".." / "rbq_simulator" / "rbq_gazebo" / "resources");
    // Resource tree carrying meshes/ + model/ 
    fs::path centralResources = resourcesDir;
    if (!fs::exists(centralResources / "meshes"))
        centralResources = fs::weakly_canonical(resourcesDir / ".." / ".." / ".." / "resources");
    if (!fs::exists(centralResources / "meshes"))
        centralResources = fs::weakly_canonical(exeDir / ".." / "resources");

    const fs::path worldSdf = resourcesDir / "worlds" / (world + ".sdf");
    const fs::path robotUrdfPath = resourcesDir / "models" / spec.name / "rbq.urdf";

    if (!fs::exists(worldSdf)) {
        std::fprintf(stderr, "[Gazebo] world SDF not found: %s\n", worldSdf.c_str());
        return 1;
    }
    std::string robotUrdf = readFile(robotUrdfPath.string());
    if (robotUrdf.empty()) {
        std::fprintf(stderr, "[Gazebo] failed to read robot URDF: %s\n", robotUrdfPath.c_str());
        return 1;
    }

    // Splice sensor URDF fragments
    rbq_gazebo::CamPoseTable camPoses{};
    {
        const fs::path sensorDir = resourcesDir / "models" / "sensor";

        // Split fragment
        auto fragParts = [](const std::string &f, std::string &vis, std::string &sen) {
            const std::string SM = "<!-- @SENSOR@ -->", VM = "<!-- @VISUAL@ -->";
            const auto sp = f.find(SM);
            if (sp == std::string::npos) { vis.clear(); sen = f; return; }
            const auto vp = f.find(VM);
            vis = (vp != std::string::npos && vp < sp)
                      ? f.substr(vp + VM.size(), sp - (vp + VM.size())) : std::string{};
            sen = f.substr(sp + SM.size());
        };

        std::string visuals, sensors;
        auto add = [&](const fs::path &frag) {
            if (!fs::exists(frag)) return false;
            std::string v, s;
            fragParts(readFile(frag.string()), v, s);
            visuals += v;
            sensors += s;
            return true;
        };

        // Cameras: required (6 RGB-D).
        const std::string camFrag = readFile((sensorDir / "camera.urdf").string());
        if (camFrag.empty()) {
            std::fprintf(stderr, "[Gazebo] camera.urdf not found (spawning without cameras)\n");
        } else {
            camPoses = parseCamPoses(camFrag);
            std::string v, s;
            fragParts(camFrag, v, s);
            visuals += v;
            sensors += s;
        }
        if (lidar != "none") {
            const fs::path lidarFrag = sensorDir / (lidar == "ouster" ? "ouster.urdf" : "livox.urdf");
            if (!add(lidarFrag))
                std::fprintf(stderr, "[Gazebo] LiDAR fragment not found: %s (no LiDAR)\n",
                             lidarFrag.c_str());
        }

        // User sensors
        const fs::path userFrag = sensorDir / "user_sensor.urdf";
        if (fs::exists(userFrag)) {
            const std::string us = readFile(userFrag.string());
            std::string scan = us;
            size_t c;
            while ((c = scan.find("<!--")) != std::string::npos) {
                const size_t e = scan.find("-->", c);
                if (e == std::string::npos) { scan.erase(c); break; }
                scan.erase(c, (e + 3) - c);
            }
            if (scan.find("<sensor") != std::string::npos) {
                std::string v, s;
                fragParts(us, v, s);
                visuals += v;
                sensors += s;
            }
        }

        // Replace each marker comment (up to its closing -->) with the accumulated content.
        auto spliceMarker = [&](const std::string &tag, const std::string &content) {
            const auto mpos = robotUrdf.find(tag);
            if (mpos == std::string::npos) return;
            const auto endC = robotUrdf.find("-->", mpos);
            const auto end = (endC == std::string::npos) ? mpos : endC + 3;
            robotUrdf.replace(mpos, end - mpos, content);
        };
        spliceMarker("<!-- @SENSOR_VISUAL@", visuals);
        spliceMarker("<!-- @SENSORS@", sensors);
        std::printf("[Gazebo] spliced sensors: cameras + lidar=%s + user\n", lidar.c_str());
    }

    // Write spliced URDF to a per-UID temp file for gz create (sdf_filename).
    const fs::path spawnUrdf =
        fs::temp_directory_path() / ("rbq_gazebo_spawn." + std::to_string(getuid()) + ".urdf");
    {
        std::ofstream o(spawnUrdf);
        if (!o) {
            std::fprintf(stderr, "[Gazebo] cannot write spawn URDF: %s\n", spawnUrdf.c_str());
            return 1;
        }
        o << robotUrdf;
    }

    // Set resource search paths before Server ctor. RBQ plugin is compiled in + registered
    // in-process (below), not loaded as a .so. model:// URIs resolve across the trees below.
    prependEnv("GZ_SIM_RESOURCE_PATH", (resourcesDir / "models").string());      // package sensor models
    prependEnv("GZ_SIM_RESOURCE_PATH", (centralResources / "meshes").string());  // meshes + env textures

    const std::string worldName = parseWorldName(readFile(worldSdf.string()), world);

    std::printf("[Gazebo] robot=%s, world=%s (%s), lidar=%s, headless=%d\n",
                spec.name.c_str(), worldName.c_str(), worldSdf.c_str(), lidar.c_str(), headless);
    // Server ctor blocks downloading Fuel models on first run; flush so it isn't read as a hang.
    std::printf("[Gazebo] loading world '%s' (first run may download Fuel models; please wait)...\n",
                worldName.c_str());
    std::fflush(stdout);

    // Embed gz::sim::Server, start UNPAUSED for spawn + plugin PreUpdate init; bridge later pausePhysics() and single-steps.
    gz::sim::ServerConfig cfg;
    cfg.SetSdfFile(worldSdf.string());
    gz::sim::Server server(cfg);
    std::printf("[Gazebo] world loaded.\n");
    std::fflush(stdout);

    // Register RBQ plugin in-process before Run(); Configure() detects the model by scanning
    // the ECM in PreUpdate.
    if (!rbq_gazebo_system::addToServer(server, spec)) {
        std::fprintf(stderr, "[Gazebo] failed to register embedded RBQ system plugin\n");
        return 1;
    }

    server.Run(/*blocking=*/false, /*iterations=*/0, /*paused=*/false);

    // Spawn the robot.
    gz::transport::Node node;
    double pose[6];
    spawnPose(spec.spawnZ, pose);
    if (!spawnRobot(node, worldName, spawnUrdf.string(), spec.name, pose)) {
        std::fprintf(stderr, "[Gazebo] robot spawn failed\n");
        return 1;
    }
    std::printf("[Gazebo] %s spawned.\n", spec.name.c_str());

    // Wait for plugin init (model detected).
    if (!waitPluginReady(node)) {
        std::fprintf(stderr, "[Gazebo] plugin /rbq/read_states not ready\n");
        return 1;
    }
    std::printf("[Gazebo] RBQ plugin ready.\n");

    // Start threads.
    std::thread bridgeT(rbq_gazebo::bridgeLoop, worldName, std::ref(g_running), spec);
    // Camera thread
    std::thread visionT(rbq_gazebo::visionRun, std::ref(g_running), camPoses);

    // GUI as child process with package gui.config (sets MinimalScene <horizontal_fov>).
#ifdef RBQ_GAZEBO_GUI_CONFIG
    fs::path guiConfig = fs::path(RBQ_GAZEBO_GUI_CONFIG);
    if (!fs::exists(guiConfig)) guiConfig = exeDir / "gui.config";
#else
    fs::path guiConfig = exeDir / "gui.config";
#endif
    const std::string guiConfigStr = guiConfig.string();
    const bool haveGuiConfig = fs::exists(guiConfig);
    if (!haveGuiConfig)
        std::fprintf(stderr, "[Gazebo] gui.config not found at %s; using ign default GUI.\n",
                     guiConfigStr.c_str());
    pid_t guiPid = -1;
    if (!headless) {
        guiPid = fork();
        if (guiPid == 0) {
            if (haveGuiConfig)
                execlp("ign", "ign", "gazebo", "-g", "--gui-config", guiConfigStr.c_str(),
                       static_cast<char *>(nullptr));
            else
                execlp("ign", "ign", "gazebo", "-g", static_cast<char *>(nullptr));
            std::perror("[Gazebo] failed to exec 'ign gazebo -g'");
            _exit(127);
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGHUP, onSignal);

    std::printf("[Gazebo] running. Ctrl+C or close the GUI to stop.\n");
    while (g_running) {
        if (guiPid > 0) {
            int status = 0;
            const pid_t r = waitpid(guiPid, &status, WNOHANG);
            if (r == guiPid) { guiPid = -1; g_running = false; break; }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Shutdown.
    g_running = false;
    if (guiPid > 0) { kill(guiPid, SIGTERM); waitpid(guiPid, nullptr, 0); }
    if (bridgeT.joinable()) bridgeT.join();
    if (visionT.joinable()) visionT.join();
    std::printf("[Gazebo] shutdown complete.\n");
    std::fflush(stdout);
    // Force-exit
    _exit(0);
}
