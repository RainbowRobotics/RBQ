#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <atomic>
#include <cmath>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <thread>
#include <sched.h>
#include <pthread.h>

#include <mujoco/mujoco.h>

#include <Eigen/Dense>

#include "gl/glfw_adapter.h"
#include "utils/simulate.h"
#include "utils/array_safety.h"

#include "Robot.h"
#include "utils/Camera.h"
#include "utils/VisionPublisher.h"
#include "utils/MjLidar.h"
#include "utils/LidarPublisher.h"

#include <rbq_sdk/rbq_sdk.hpp>
#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/dds/LidarDds.hpp>
#include <rbq_sdk/idl/rbq/MotionRef_.hpp>
#include <rbq_sdk/idl/rbq/SimInfo_.hpp>
#include <rbq_sdk/idl/ros2/TFMessage_.hpp>

// Arm actuator counts.
static constexpr int MAX_MANI_MC = 6;
static constexpr int MAX_GRP     = 1;

// Minimal wall-clock timer.
class Timer {
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>(clock_::now() - beg_).count();
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1>> second_;
    std::chrono::time_point<clock_> beg_;
};

void CameraThread(mujoco::Simulate* sim, const std::string &cameraName);
void LidarThread(mujoco::Simulate* sim, const std::string &siteName,
                 const std::string &cloudTopic, const std::string &imuTopic);
void PhysicsLoop(mujoco::Simulate* sim, const char* filename, const rbq_mujoco::RobotSpec &spec);

// LiDAR is opt-in via --livox / --ouster (same convention as rbq_gazebo). The sites and
// their IMU sensors are always in the model; the flag draws the housings, which live in
// geom groups 4/5 that MuJoCo hides by default, and starts the Livox raytracer threads.
static constexpr int kLivoxGeomGroup  = 4;
static constexpr int kOusterGeomGroup = 5;
static bool g_livoxVisible  = false;
static bool g_ousterVisible = false;

static inline void applyLidarGroups(mjvOption &opt) {
    opt.geomgroup[kLivoxGeomGroup]  = g_livoxVisible  ? 1 : 0;
    opt.geomgroup[kOusterGeomGroup] = g_ousterVisible ? 1 : 0;
}

// Headless self-test (smoke test) shared state.
static std::atomic<bool> g_selftest{false};
static std::atomic<long> g_physicsSteps{0};
static std::atomic<int>  g_camFrames[6];  // BT0,BT1,BT2,BT3,FT0,RR0
extern mjModel *Model;
extern mjData  *Data;

static inline std::string getExecutableDir() {
    constexpr char kPathSep = '/';
    const char *path = "/proc/self/exe";

    std::string realpath = [&]() -> std::string
    {
        std::unique_ptr<char[]> realpath(nullptr);
        std::uint32_t buf_size = 128;
        bool success = false;

        while (!success)
        {
            realpath.reset(new (std::nothrow) char[buf_size]);
            if (!realpath)
            {
                std::cerr << "cannot allocate memory to store executable path\n";
                return "";
            }

            ssize_t written = readlink(path, realpath.get(), buf_size);
            if (written < buf_size)
            {
                realpath.get()[written] = '\0';
                success = true;
            }
            else if (written == -1)
            {
                if (errno == EINVAL)
                {
                    // path is not a symlink, just return it
                    return path;
                }

                std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
                return "";
            }
            else
            {
                // buffer too small, retry with larger size
                buf_size *= 2;
            }
        }
        return realpath.get();
    }();

    if (realpath.empty())
        return "";

    // Find the last '/'
    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
        if (realpath[i] == kPathSep)
        {
            return realpath.substr(0, i);
        }
    }

    return "";
}

int main(int argc, char *argv[])
{
    std::string path;
    std::string interfaceName = "lo";
    double selftestSec = 0.0;
    rbq_mujoco::RobotSpec spec = rbq_mujoco::rbqSpec();
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-i" || arg == "--interface") && i + 1 < argc) {
            interfaceName = argv[++i];
            std::cout << " using network interface: " << interfaceName << std::endl;
        }
        else if ((arg == "-p" || arg == "--path") && i + 1 < argc) {
            path = argv[++i];
            std::cout << " using path: " << path << std::endl;
        }
        else if (arg == "--wheel") {
            spec = rbq_mujoco::rbqWheelSpec();
            std::cout << " using robot variant: " << spec.name << std::endl;
        }
        else if (arg == "--livox") {
            g_livoxVisible = true;
            std::cout << " running Livox Mid360 pair (front/rear)" << std::endl;
        }
        else if (arg == "--ouster") {
            g_ousterVisible = true;
            std::cout << " showing Ouster OS1 housing" << std::endl;
        }
        else if (arg == "--selftest") {
            g_selftest.store(true);
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                selftestSec = std::atof(argv[++i]);
            }
            if (selftestSec <= 0.0) selftestSec = 3.0;
            std::cout << " running self-test for " << selftestSec << " s" << std::endl;
        }
        else if (arg == "--help") {
            std::printf("Usage: rbq_mujoco [--wheel] [--livox|--ouster] [-p <model.xml>] [-i <iface>] [--selftest [sec]]\n"
                        "  --wheel     wheeled RBQ (12 leg joints + 4 wheels); default is the quadruped.\n"
                        "  --livox     run the Livox Mid360 pair (front/rear) and draw their housings.\n"
                        "  --ouster    draw the Ouster OS1 housing (rear deck).\n"
                        "  -p          MJCF model file; defaults to the variant's resources/model/*.xml.\n");
            return 0;
        }
    }
    // -p overrides the file only: the joint layout still comes from the selected variant.
    if (path.empty()) path = getExecutableDir() + "/../resources/model/" + spec.modelFile;
    rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, interfaceName);

    int nplugin = mjp_pluginCount();
    if (nplugin) {
        std::printf("Built-in plugins:\n");
        for (int i = 0; i < nplugin; ++i) {
            std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
    }
    const std::string executable_dir = getExecutableDir();
    if (!executable_dir.empty()) {
        const std::string plugin_dir = executable_dir + "/mujoco_plugin";
        mj_loadAllPluginLibraries(plugin_dir.c_str(), +[](const char *filename, int first, int count) {
            std::printf("Plugins registered by library '%s':\n", filename);
            for (int i = first; i < first + count; ++i) {
                std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
            }
        });
    }

    mjvCamera cam;
    mjv_defaultCamera(&cam);
    mjvOption opt;
    mjv_defaultOption(&opt);
    applyLidarGroups(opt);
    mjvPerturb pert;
    mjv_defaultPerturb(&pert);
    auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam, &opt, &pert, /* is_passive = */ false);
    sim->ui0_enable = 0; 
    sim->ui1_enable = 0;  

    std::vector<std::thread*> threads;
    threads.push_back(new std::thread(PhysicsLoop, sim.get(), path.c_str(), spec));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    threads.push_back(new std::thread(CameraThread, sim.get(), "BT0"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threads.push_back(new std::thread(CameraThread, sim.get(), "BT1"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threads.push_back(new std::thread(CameraThread, sim.get(), "BT2"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threads.push_back(new std::thread(CameraThread, sim.get(), "BT3"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threads.push_back(new std::thread(CameraThread, sim.get(), "FT0"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threads.push_back(new std::thread(CameraThread, sim.get(), "RR0"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // Behind the flag: each sensor costs a thread whether or not anyone subscribes,
    // which is enough to starve a camera thread on a small CI runner.
    if (g_livoxVisible) {
        threads.push_back(new std::thread(LidarThread, sim.get(), "lidar_front",
                                          "rt/rbq/lidar/lidar_front", "rt/rbq/lidar/lidar_front/imu"));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        threads.push_back(new std::thread(LidarThread, sim.get(), "lidar_rear",
                                          "rt/rbq/lidar/lidar_rear", "rt/rbq/lidar/lidar_rear/imu"));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (g_selftest.load()) {
        // Hard-exit below instead of joining the camera threads: GLFW's teardown
        // races across threads (segfault in glfwDestroyWindow).

        // The model loads asynchronously and a load warning leaves the sim paused;
        // wait for it, then un-pause so physics steps during the test window.
        for (int i = 0; i < 500 && Model == nullptr; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        sim->run = 1;
        std::this_thread::sleep_for(std::chrono::duration<double>(selftestSec));

        const char* camNames[6] = {"BT0", "BT1", "BT2", "BT3", "FT0", "RR0"};
        const long steps = g_physicsSteps.load();
        bool modelLoaded = false;
        bool finite = false;
        {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            modelLoaded = (Model != nullptr && Data != nullptr);
            finite = modelLoaded;
            if (modelLoaded) {
                for (int i = 0; i < Model->nq; ++i)
                    if (!std::isfinite(Data->qpos[i])) finite = false;
                for (int i = 0; i < Model->nv; ++i)
                    if (!std::isfinite(Data->qvel[i])) finite = false;
            }
        }
        bool camsOk = true;
        for (int i = 0; i < 6; ++i) {
            const int frames = g_camFrames[i].load();
            if (frames <= 0) camsOk = false;
            std::printf("  camera %-3s : %d frames%s\n", camNames[i], frames,
                        frames > 0 ? "" : "  <-- FAIL");
        }
        std::printf("  model loaded : %s\n", modelLoaded ? "yes" : "no  <-- FAIL");
        std::printf("  physics steps: %ld%s\n", steps, steps > 0 ? "" : "  <-- FAIL");
        std::printf("  state finite : %s\n", finite ? "yes" : "no  <-- FAIL");
        const bool pass = modelLoaded && steps > 0 && finite && camsOk;
        std::printf("SELFTEST %s\n", pass ? "PASS" : "FAIL");
        std::fflush(stdout);
        _exit(pass ? 0 : 1);
    }

    sim->RenderLoop();

    for(auto thread : threads) {
        if (thread->joinable()) {
            thread->join();
        }
        delete thread;
    }
    return 0;
}

static inline double Limit(double input, double upper, double lower) {
    if (input >= upper) return upper;
    if (input <= lower) return lower;
    return input;
}

static inline Eigen::Vector3f GetWorldContactForceFromCfrcExt(const mjModel* m, const mjData* d, int body_id) {
    if (!m || !d || body_id < 0 || body_id >= m->nbody) return Eigen::Vector3f::Zero();
    const mjtNum* f = d->cfrc_ext + 6 * body_id + 3; // cfrc_ext is [Tx,Ty,Tz,Fx,Fy,Fz] — force is the last 3
    return Eigen::Vector3f(static_cast<float>(f[0]), static_cast<float>(f[1]), static_cast<float>(f[2]));
}

static inline Eigen::Vector3f GetWorldContactTorqueFromCfrcExt(const mjModel* m, const mjData* d, int body_id) {
    if (!m || !d || body_id < 0 || body_id >= m->nbody) return Eigen::Vector3f::Zero();
    const mjtNum* f = d->cfrc_ext + 6 * body_id; // cfrc_ext is [Tx,Ty,Tz,Fx,Fy,Fz] — torque is the first 3
    return Eigen::Vector3f(static_cast<float>(f[0]), static_cast<float>(f[1]), static_cast<float>(f[2]));
}

mjModel *Model  = nullptr;
mjData  *Data  = nullptr;

int physicsFps = 0;
int sensorsFps[6] = {0,};
int sensorsTargetFps[6] = {15,15,15,15,15,15}; // BT0,BT1,BT2,BT3,FT0,RR0};

mjModel *LoadModel(const char *file, mujoco::Simulate &sim)
{
    char filename[mujoco::Simulate::kMaxFilenameLength];
    mujoco::sample_util::strcpy_arr(filename, file);
    if (!filename[0]) {
        return nullptr;
    }
    const int kErrorLength = 1024; // load error string length
    char loadError[kErrorLength] = "";
    mjModel *model = 0;
    if (mujoco::sample_util::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mujoco::sample_util::strlen_arr(filename) - 4, ".mjb",
                      mujoco::sample_util::sizeof_arr(filename) - mujoco::sample_util::strlen_arr(filename) + 4)) {
        model = mj_loadModel(filename, nullptr);
        if (!model) {
            mujoco::sample_util::strcpy_arr(loadError, "could not load binary model");
        }
    } else {
        model = mj_loadXML(filename, nullptr, loadError, kErrorLength);
        if (loadError[0]) {
            int error_length = mujoco::sample_util::strlen_arr(loadError);
            if (loadError[error_length - 1] == '\n') {
                loadError[error_length - 1] = '\0';
            }
        }
    }
    mujoco::sample_util::strcpy_arr(sim.load_error, loadError);
    if (!model) {
        std::printf("%s\n", loadError);
        return nullptr;
    }
    if (loadError[0]) {
        std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
        sim.run = 0;
    }
    return model;
}

void CameraThread(mujoco::Simulate* sim, const std::string &cameraName)
{
    const int frame_width   = 640;
    const int frame_height  = 360;

    // Initialize GLFW in this thread
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW in camera thread \n";
        return;
    }

    // Create a hidden window for OpenGL context
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(frame_width, frame_height, "Hidden", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window in camera thread \n";
        glfwTerminate();
        return;
    }

    // Make the OpenGL context current
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize scene and context
    mjvScene scn;
    mjrContext con;
    mjvOption opt;

    mjv_defaultOption(&opt);
    applyLidarGroups(opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Wait until model and data are initialized
    while (!Model || !Data) {
        if (sim->exitrequest.load()) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Create scene and context after model is loaded
    mjv_makeScene(Model, &scn, 2000);
    mjr_makeContext(Model, &con, mjFONTSCALE_150);

    while (!Model || !Data) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Get RGB and depth data
    mjr_setBuffer(mjFB_OFFSCREEN, &con);

    // Render scene from camera perspective
    mjvCamera robot_cam;
    robot_cam.type = mjCAMERA_FIXED;
    robot_cam.fixedcamid = mj_name2id(Model, mjOBJ_CAMERA, cameraName.c_str());
    if (robot_cam.fixedcamid < 0) {
        return;
    }
    int sensorId;
    if(cameraName == "BT0") {
        sensorId = 0;
    } else if(cameraName == "BT1") {
        sensorId = 1;
    } else if(cameraName == "BT2") {
        sensorId = 2;
    } else if(cameraName == "BT3") {
        sensorId = 3;
    } else if(cameraName == "FT0") {
        sensorId = 4;
    } else if(cameraName == "RR0") {
        sensorId = 5;
    } else {
        return;
    }
    Camera camera;
    VisionPublisher publisher(sensorId);

    const int targetFps = sensorsTargetFps[static_cast<int>(sensorId)];
    const long PERIOD_US = static_cast<long>(1000 * 1000.0 / targetFps);
    struct timespec TIME_NEXT;
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    std::cout << " Camera Thread Started: " << cameraName << ", Target FPS: " << targetFps << std::endl;

    while (!sim->exitrequest.load()) {
        if (!Model || !Data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(Model, Data, &opt, NULL, &robot_cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        camera.setIntrinsics(Model, robot_cam, viewport);
        camera.getBuffer(Model, viewport, &con);
        camera.releaseBuffer();
        // cv::imshow(cameraName+" Color View", camera.color());
        // cv::Mat depth_image;
        // camera.depth().convertTo(depth_image, CV_8U, 255.0 / 1000.0);  // Normalize to 0-255
        // cv::imshow(cameraName+" Depth View", depth_image);
        // if (cv::waitKey(1) == 27) break;

        // Camera intrinsics (fx,fy,ppx,ppy); MuJoCo renders without lens distortion.
        const float intr[4] = { camera.intrinsics(0), camera.intrinsics(1),
                                camera.intrinsics(2), camera.intrinsics(3) };
        static const float kZeroCoeffs[8] = {0};

        // Camera->base TF; MuJoCo cam frame (x-right/y-up/z-back) -> optical (z-fwd).
        double pose[7];
        const double *posePtr = nullptr;
        const int base_body_id = mj_name2id(Model, mjOBJ_BODY, "base_link");
        if (base_body_id > -1) {
            Eigen::Matrix4d T_cam_world  = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d T_base_world = Eigen::Matrix4d::Identity();
            const mjtNum *cam_pos  = Data->cam_xpos + 3 * robot_cam.fixedcamid;
            const mjtNum *cam_mat  = Data->cam_xmat + 9 * robot_cam.fixedcamid;
            const mjtNum *base_pos = Data->xpos + 3 * base_body_id;
            const mjtNum *base_mat = Data->xmat + 9 * base_body_id;
            for (int i = 0; i < 3; ++i) {
                T_cam_world(i, 3)  = cam_pos[i];
                T_base_world(i, 3) = base_pos[i];
                for (int j = 0; j < 3; ++j) {
                    T_cam_world(i, j)  = cam_mat[3 * i + j];
                    T_base_world(i, j) = base_mat[3 * i + j];
                }
            }
            Eigen::Matrix4d rotate_;
            rotate_ << 1,  0,  0, 0,
                       0, -1,  0, 0,
                       0,  0, -1, 0,
                       0,  0,  0, 1;
            const Eigen::Matrix4d tf = T_base_world.inverse() * T_cam_world * rotate_;
            const Eigen::Quaterniond q(Eigen::Matrix3d(tf.block<3, 3>(0, 0)));
            pose[0] = tf(0, 3); pose[1] = tf(1, 3); pose[2] = tf(2, 3);
            pose[3] = q.w(); pose[4] = q.x(); pose[5] = q.y(); pose[6] = q.z();
            posePtr = pose;
        }
        {
            cv::Mat frame = camera.depth();
            const int width_  = frame.cols;
            const int height_ = frame.rows;
            const uint16_t* depth_data = frame.ptr<uint16_t>();
            if (depth_data && width_ != 0 && height_ != 0) {
                publisher.publishDepth(width_, height_, depth_data, intr, kZeroCoeffs, posePtr);
            }
        }
        if(cameraName == "BT0" || cameraName == "BT1" || cameraName == "BT2" || cameraName == "BT3") {
            cv::Mat colorframe = camera.color();
            cv::Mat gray;
            cv::cvtColor(colorframe, gray, cv::COLOR_BGR2GRAY);

            const int width_  = gray.cols;
            const int height_ = gray.rows;
            const uint8_t* ir_data = gray.ptr<uint8_t>();
            if (ir_data && width_ != 0 && height_ != 0) {
                publisher.publishIr(width_, height_, ir_data, intr, kZeroCoeffs, posePtr);
            }
        }
        if(cameraName == "FT0" || cameraName == "RR0") {
            cv::Mat color = camera.color();
            publisher.publishRgb(color.cols, color.rows, color.data, intr, kZeroCoeffs, posePtr);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();

        sensorsFps[static_cast<int>(sensorId)]++;
        g_camFrames[sensorId].fetch_add(1, std::memory_order_relaxed);

        TIME_NEXT.tv_nsec += PERIOD_US * 1000;
        if (TIME_NEXT.tv_nsec > 1000000000) {
            TIME_NEXT.tv_nsec -= 1000000000;
            TIME_NEXT.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    }
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
}

// One thread per lidar: cloud 10 Hz, the site's IMU 200 Hz and the base_link->sensor
// TF 2 Hz, rate-divided off one 200 Hz loop. Per-lidar so a missing site or sensor on
// one does not stop the other.
void LidarThread(mujoco::Simulate* sim, const std::string &siteName,
                 const std::string &cloudTopic, const std::string &imuTopic)
{
    // Faithful Livox Mid360: 24000 rays/frame, 50 m range, 10 Hz cloud.
    const std::string patternPath = getExecutableDir() + "/../resources/model/lidar/mid360.bin";
    MjLidar lidar(patternPath, /*samplesPerFrame=*/24000, /*cutoff=*/50.0f, /*framePeriod=*/0.1f);
    if (!lidar.valid()) {
        std::cerr << "[LidarThread] scan pattern unavailable; " << siteName << " disabled.\n";
        return;
    }
    LidarPublisher                                     cloudPub(cloudTopic, siteName);
    rbq_sdk::Publisher<sensor_msgs::msg::dds_::Imu_>    imuPub(imuTopic);
    rbq_sdk::Publisher<tf2_msgs::msg::dds_::TFMessage_> tfPub("rt/tf");

    const std::string gyroName = siteName + "_gyro";
    const std::string accName  = siteName + "_acc";
    std::vector<float> xyz, intensity, times;  // cloud scratch, reused

    auto stampNow = [](uint32_t &sec, uint32_t &nsec) {
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch());
        sec  = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(ns).count());
        nsec = static_cast<uint32_t>(ns.count() % 1000000000ULL);
    };

    // 200 Hz: read the site's gyro+accelerometer, publish sensor_msgs/Imu (SI).
    bool warnedNoImu = false;
    auto doImu = [&](int gyroId, int accId) {
        if (!imuPub.hasSubscribers()) return;
        if (gyroId < 0 || accId < 0) {
            if (!warnedNoImu) { std::cerr << "[LidarThread] no IMU sensors for " << siteName << ".\n"; warnedNoImu = true; }
            return;
        }
        double gyr[3] = {0, 0, 0}, acc[3] = {0, 0, 0};
        {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            const int ga = Model->sensor_adr[gyroId];
            const int aa = Model->sensor_adr[accId];
            for (int k = 0; k < 3; ++k) { gyr[k] = Data->sensordata[ga + k]; acc[k] = Data->sensordata[aa + k]; }
        }
        uint32_t sec, nsec; stampNow(sec, nsec);
        sensor_msgs::msg::dds_::Imu_ msg;
        LidarDds::fillImu(msg, siteName, sec, nsec, gyr, acc);
        imuPub.write(msg);
    };

    // 10 Hz: raytrace the scan, publish PointCloud2 (+ lidar->base pose for /tf).
    auto doCloud = [&](int siteId, int baseId) {
        if (!cloudPub.hasSubscribers()) return;
        int n = 0;
        Eigen::Matrix4d T_site_world = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_base_world = Eigen::Matrix4d::Identity();
        bool haveTf = false;
        {
            // mj_multiRay writes ray scratch in Data; serialize against mj_step.
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            n = lidar.scan(Model, Data, siteId, baseId, xyz, intensity, times);
            const mjtNum *sp = Data->site_xpos + 3 * siteId;
            const mjtNum *sm = Data->site_xmat + 9 * siteId;
            for (int i = 0; i < 3; ++i) {
                T_site_world(i, 3) = sp[i];
                for (int j = 0; j < 3; ++j) T_site_world(i, j) = sm[3 * i + j];
            }
            if (baseId > -1) {
                const mjtNum *bp = Data->xpos + 3 * baseId;
                const mjtNum *bm = Data->xmat + 9 * baseId;
                for (int i = 0; i < 3; ++i) {
                    T_base_world(i, 3) = bp[i];
                    for (int j = 0; j < 3; ++j) T_base_world(i, j) = bm[3 * i + j];
                }
                haveTf = true;
            }
        }
        double pose[7];
        const double *posePtr = nullptr;
        if (haveTf) {
            const Eigen::Matrix4d T = T_base_world.inverse() * T_site_world;  // lidar->base
            const Eigen::Quaterniond q(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
            pose[0] = T(0, 3); pose[1] = T(1, 3); pose[2] = T(2, 3);
            pose[3] = q.w();   pose[4] = q.x();   pose[5] = q.y(); pose[6] = q.z();
            posePtr = pose;
        }
        cloudPub.publish(n, xyz.data(), intensity.data(), times.data(), posePtr);
    };

    // 2 Hz: base_link -> site TF; site_pos/site_quat are already relative to base_link.
    auto doTf = [&](int siteId) {
        double pos[3], quat[4];
        {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            for (int k = 0; k < 3; ++k) pos[k]  = Model->site_pos[3 * siteId + k];
            for (int k = 0; k < 4; ++k) quat[k] = Model->site_quat[4 * siteId + k];  // w,x,y,z
        }
        uint32_t sec, nsec; stampNow(sec, nsec);
        tf2_msgs::msg::dds_::TFMessage_ tf;
        geometry_msgs::msg::dds_::TransformStamped_ ts;
        ts.header().stamp().sec()     = sec;
        ts.header().stamp().nanosec() = nsec;
        ts.header().frame_id()        = "base_link";
        ts.child_frame_id()           = siteName;
        ts.transform().translation().x() = pos[0];
        ts.transform().translation().y() = pos[1];
        ts.transform().translation().z() = pos[2];
        ts.transform().rotation().w()    = quat[0];
        ts.transform().rotation().x()    = quat[1];
        ts.transform().rotation().y()    = quat[2];
        ts.transform().rotation().z()    = quat[3];
        tf.transforms().push_back(ts);
        tfPub.write(tf);
    };

    // 200 Hz base loop; cloud every 20th tick (10 Hz), TF every 100th (2 Hz).
    constexpr long PERIOD_US   = 5 * 1000;  // 200 Hz
    constexpr int  CLOUD_EVERY = 20;
    constexpr int  TF_EVERY    = 100;
    struct timespec TIME_NEXT;
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);

    mjModel* lastModel = nullptr;
    int siteId = -1, baseId = -1, gyroId = -1, accId = -1;
    int tick = 0;
    bool warnedNoSite = false;
    std::cout << " Lidar Thread Started (" << siteName << ": cloud 10Hz, imu 200Hz, tf 2Hz)." << std::endl;

    while (!sim->exitrequest.load()) {
        if (!Model || !Data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        // Resolve ids once per model (re-resolve on reload).
        if (Model != lastModel) {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            lastModel = Model;
            siteId = mj_name2id(Model, mjOBJ_SITE,   siteName.c_str());
            baseId = mj_name2id(Model, mjOBJ_BODY,   "base_link");
            gyroId = mj_name2id(Model, mjOBJ_SENSOR, gyroName.c_str());
            accId  = mj_name2id(Model, mjOBJ_SENSOR, accName.c_str());
        }
        if (siteId < 0) {
            if (!warnedNoSite) {
                std::cerr << "[LidarThread] no '" << siteName << "' site in model; disabled.\n";
                warnedNoSite = true;
            }
            return;
        }

        doImu(gyroId, accId);
        if (tick % CLOUD_EVERY == 0) doCloud(siteId, baseId);
        if (tick % TF_EVERY == 0)    doTf(siteId);
        ++tick;

        TIME_NEXT.tv_nsec += PERIOD_US * 1000;
        if (TIME_NEXT.tv_nsec > 1000000000) {
            TIME_NEXT.tv_nsec -= 1000000000;
            TIME_NEXT.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    }
}

void PhysicsLoop(mujoco::Simulate* sim, const char* filename, const rbq_mujoco::RobotSpec &spec)
{
    const double target_dt = 0.001;
    const double syncMisalign = 0.1;
    const double simRefreshFraction = 0.7;
    auto next_wake_time = std::chrono::high_resolution_clock::now();
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set thread priority" << std::endl;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);  // Use CPU core 0
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Warning: Failed to set thread affinity" << std::endl;
    }
    // control noise variables
    mjtNum *ctrlnoise = nullptr;
    if (filename != nullptr) {
        sim->LoadMessage(filename);
        Model = LoadModel(filename, *sim);
        if (Model)
            Data = mj_makeData(Model);
        if (Data) {
            // sim->Load() blocks on the render thread, which the headless self-test has none of.
            if (!g_selftest.load())
                sim->Load(Model, Data, filename);
            mj_forward(Model, Data);
            free(ctrlnoise);
            ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * Model->nu));
            mju_zero(ctrlnoise, Model->nu);
        } else {
            sim->LoadMessageClear();
        }
    }

    const int JOINT_NUM = static_cast<int>(spec.joints.size());
    // initialize state and control
    if (Data && Model) {
        // The variant owns qpos[0..6+JOINT_NUM]; the environment's free-joint props follow.
        // Abort on a joint-table/MJCF mismatch rather than driving the wrong actuators.
        for (int i = 0; i < JOINT_NUM; ++i) {
            const int jointId = mj_name2id(Model, mjOBJ_JOINT, spec.joints[i].name.c_str());
            if (jointId < 0 || Model->jnt_qposadr[jointId] != 7 + i) {
                std::cerr << "Error: joint '" << spec.joints[i].name << "' missing or at qpos "
                          << (jointId < 0 ? -1 : Model->jnt_qposadr[jointId]) << ", expected "
                          << (7 + i) << " — model does not match variant '" << spec.name << "'"
                          << std::endl;
                // _exit, not exit: camera threads are already running against Model/Data —
                // exit()'s static teardown would race their in-flight MuJoCo/GL calls.
                _exit(1);
            }
        }
        // cfrc_ext is read by body id; an unresolved name would publish zero contact forever.
        for (const std::string &body : spec.contactBodies) {
            if (mj_name2id(Model, mjOBJ_BODY, body.c_str()) < 0) {
                std::cerr << "Error: contact body '" << body << "' not in model — variant '"
                          << spec.name << "'" << std::endl;
                _exit(1);  // see _exit note above
            }
        }
        memcpy(Data->qpos, spec.initialQpos.data(), spec.initialQpos.size() * sizeof(double));
        mju_zero(Data->qvel, Model->nv);
    }

    // cpu-sim syncronization point
    std::chrono::time_point<mujoco::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    while (!sim->exitrequest.load())
    {
        if (sim->droploadrequest.load()) {
            sim->LoadMessage(sim->dropfilename);
            mjModel *model = LoadModel(sim->dropfilename, *sim);
            sim->droploadrequest.store(false);

            mjData *dnew = nullptr;
            if (model)
                dnew = mj_makeData(model);
            if (dnew) {
                sim->Load(model, dnew, sim->dropfilename);

                mj_deleteData(Data);
                mj_deleteModel(Model);

                Model = model;
                Data = dnew;
                mj_forward(Model, Data);

                // allocate ctrlnoise
                free(ctrlnoise);
                ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * Model->nu);
                mju_zero(ctrlnoise, Model->nu);
            } else {
                sim->LoadMessageClear();
            }
        }

        if (sim->uiloadrequest.load()) {
            sim->uiloadrequest.fetch_sub(1);
            sim->LoadMessage(sim->filename);
            mjModel *model = LoadModel(sim->filename, *sim);
            mjData *dnew = nullptr;
            if (model)
                dnew = mj_makeData(model);
            if (dnew) {
                sim->Load(model, dnew, sim->filename);

                mj_deleteData(Data);
                mj_deleteModel(Model);

                Model = model;
                Data = dnew;
                mj_forward(Model, Data);

                // allocate ctrlnoise
                free(ctrlnoise);
                ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * Model->nu));
                mju_zero(ctrlnoise, Model->nu);
            } else {
                sim->LoadMessageClear();
            }
        }

        if (sim->run && sim->busywait)
            std::this_thread::yield();
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

            if (Model) {
                if (sim->run) {
                    bool stepped = false;

                    // record cpu time at start of iteration
                    const auto startCPU = mujoco::Simulate::Clock::now();

                    // elapsed CPU and simulation time since last sync
                    const auto elapsedCPU = startCPU - syncCPU;
                    double elapsedSim = Data->time - syncSim;

                    // inject noise
                    if (sim->ctrl_noise_std) {
                        // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
                        mjtNum rate = mju_exp(-Model->opt.timestep / mju_max(sim->ctrl_noise_rate, mjMINVAL));
                        mjtNum scale = sim->ctrl_noise_std * mju_sqrt(1 - rate * rate);
                        for (int i = 0; i < Model->nu; i++) {
                            // update noise
                            ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);
                            // apply noise
                            Data->ctrl[i] = ctrlnoise[i];
                        }
                    }

                    // requested slow-down factor
                    const double slowdown = 100 / sim->percentRealTime[sim->real_time_index];

                    // misalignment condition: distance from target sim time is bigger than syncmisalign
                    const bool misaligned = mju_abs(std::chrono::duration<double>(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

                    // out-of-sync (for any reason): reset sync times, step
                    if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned || sim->speed_changed) {
                        // re-sync
                        syncCPU = startCPU;
                        syncSim = Data->time;
                        sim->speed_changed = false;

                        // run single step, let next iteration deal with timing
                        mj_step(Model, Data);
                        g_physicsSteps.fetch_add(1, std::memory_order_relaxed);
                        stepped = true;
                    } else { // in-sync: step until ahead of cpu
                        bool measured = false;
                        mjtNum prevSim = Data->time;

                        double refreshTime = simRefreshFraction / sim->refresh_rate;

                        // step while sim lags behind cpu and within refreshTime
                        while (std::chrono::duration<double>((Data->time - syncSim) * slowdown) < mujoco::Simulate::Clock::now() - syncCPU &&
                               mujoco::Simulate::Clock::now() - startCPU < std::chrono::duration<double>(refreshTime))
                        {
                            // measure slowdown before first step
                            if (!measured && elapsedSim) {
                                sim->measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                                measured = true;
                            }

                            static rbq_msgs::msg::dds_::MotionRef_ msg_motionRef;
                            static rbq_sdk::Subscriber<rbq_msgs::msg::dds_::MotionRef_>
                                    sub_motionRef(&msg_motionRef, "rt/rbq/ref/motion/_0");
                            static rbq_sdk::Publisher<rbq_msgs::msg::dds_::SimInfo_> pub_simInfo("rt/rbq/_sim");
                            rbq_msgs::msg::dds_::SimInfo_ msg_simInfo;


                            // IMU data
                            {
                                // Wall-clock epoch, not elapsed-since-start: subscribers stamp their own
                                // TFs with epoch and there is no /clock here, so an offset clock would put
                                // two time domains in one tf tree.
                                auto sim_ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                   std::chrono::system_clock::now().time_since_epoch());
                                const uint32_t sim_sec     = std::chrono::duration_cast<std::chrono::seconds>(sim_ns).count();
                                const uint32_t sim_nanosec = sim_ns.count() % 1000000000ULL;
                                msg_simInfo.imu().header().stamp().sec() = sim_sec;
                                msg_simInfo.imu().header().stamp().nanosec() = sim_nanosec;
                                msg_simInfo.imu().orientation().w() = Data->qpos[3];
                                msg_simInfo.imu().orientation().x() = Data->qpos[4];
                                msg_simInfo.imu().orientation().y() = Data->qpos[5];
                                msg_simInfo.imu().orientation().z() = Data->qpos[6];
                                int sensor_id = mj_name2id(Model, mjOBJ_SENSOR, "imu_acc");
                                if (sensor_id == -1) {
                                    std::cerr << "Error: imu_acc sensor not found in XML file" << std::endl;
                                } else {
                                    int sensor_adr = Model->sensor_adr[sensor_id];
                                    msg_simInfo.imu().linear_acceleration().x() = Data->sensordata[sensor_adr + 0];
                                    msg_simInfo.imu().linear_acceleration().y() = Data->sensordata[sensor_adr + 1];
                                    msg_simInfo.imu().linear_acceleration().z() = Data->sensordata[sensor_adr + 2];
                                }
                                sensor_id = mj_name2id(Model, mjOBJ_SENSOR, "imu_gyro");
                                if (sensor_id == -1) {
                                    std::cerr << "Error: imu_gyro sensor not found in XML file" << std::endl;
                                } else {
                                    int sensor_adr = Model->sensor_adr[sensor_id];
                                    msg_simInfo.imu().angular_velocity().x() = Data->sensordata[sensor_adr + 0];
                                    msg_simInfo.imu().angular_velocity().y() = Data->sensordata[sensor_adr + 1];
                                    msg_simInfo.imu().angular_velocity().z() = Data->sensordata[sensor_adr + 2];
                                }

                            }
                            // Joint state + PD, clamped to the real motor rating so telemetry matches.
                            for (int i = 0; i < JOINT_NUM; i++) {
                                const rbq_mujoco::JointSpec &joint = spec.joints[i];
                                const bool isWheel = (joint.group == rbq_mujoco::JointGroup::Wheel);
                                const int  idx     = joint.index;
                                const double q  = Data->qpos[7 + i];
                                const double dq = Data->qvel[6 + i];
                                const auto &r = isWheel ? msg_motionRef.whl_joint().at(idx)
                                                        : msg_motionRef.leg_joint().at(idx);
                                double torque = (r.pos() - q) * r.kp() + (-dq) * r.kd() + r.torque();
                                torque = Limit(torque, joint.torqueLim, -joint.torqueLim);
                                Data->ctrl[i] = torque;
                                auto &js = isWheel ? msg_simInfo.whl_joint()[idx]
                                                   : msg_simInfo.leg_joint()[idx];
                                js.pos()    = q;
                                js.vel()    = dq;
                                js.torque() = torque;
                            }
                            // GT qpos[0:3]=pos, qpos[3:7]=quat(x,y,z,w); qvel[0:3]=linvel, qvel[3:6]=angvel.
                            {
                                msg_simInfo.body_task().pos()[0] = Data->qpos[0];
                                msg_simInfo.body_task().pos()[1] = Data->qpos[1];
                                msg_simInfo.body_task().pos()[2] = Data->qpos[2];
                                msg_simInfo.body_task().quat()[0] = Data->qpos[6];
                                msg_simInfo.body_task().quat()[1] = Data->qpos[3];
                                msg_simInfo.body_task().quat()[2] = Data->qpos[4];
                                msg_simInfo.body_task().quat()[3] = Data->qpos[5];
                                msg_simInfo.body_task().vel()[0] = Data->qvel[0];
                                msg_simInfo.body_task().vel()[1] = Data->qvel[1];
                                msg_simInfo.body_task().vel()[2] = Data->qvel[2];
                                msg_simInfo.body_task().omega()[0] = Data->qvel[3];
                                msg_simInfo.body_task().omega()[1] = Data->qvel[4];
                                msg_simInfo.body_task().omega()[2] = Data->qvel[5];
                                for (int i=0; i<4; i++) {
                                    const int contactBody = mj_name2id(Model, mjOBJ_BODY, spec.contactBodies[i].c_str());
                                    Eigen::Vector3f force = GetWorldContactForceFromCfrcExt(Model, Data, contactBody);
                                    msg_simInfo.leg_contact()[i].force()[0] = force(0);
                                    msg_simInfo.leg_contact()[i].force()[1] = force(1);
                                    msg_simInfo.leg_contact()[i].force()[2] = force(2);
                                    Eigen::Vector3f torque = GetWorldContactTorqueFromCfrcExt(Model, Data, contactBody);
                                    msg_simInfo.leg_contact()[i].torque()[0] = torque(0);
                                    msg_simInfo.leg_contact()[i].torque()[1] = torque(1);
                                    msg_simInfo.leg_contact()[i].torque()[2] = torque(2);
                                }
                            }
                            pub_simInfo.write(msg_simInfo);
                            
                            mj_step(Model, Data);
                            g_physicsSteps.fetch_add(1, std::memory_order_relaxed);
                            stepped = true;
                            physicsFps++;

                            // break if reset
                            if (Data->time < prevSim)
                                break;
                        }
                    }

                    // save current state to history buffer
                    if (stepped)
                        sim->AddToHistory();
                }
                // paused
                else {
                    // run mj_forward, to update rendering and joint sliders
                    mj_forward(Model, Data);
                    sim->speed_changed = true;
                }
            }
        }

        if (sim->run) {
            next_wake_time += std::chrono::microseconds(static_cast<int>(target_dt * 1000000));
            auto now = std::chrono::high_resolution_clock::now();
            if (now < next_wake_time) {
                std::this_thread::sleep_until(next_wake_time);
            } else {
                next_wake_time = now + std::chrono::microseconds(static_cast<int>(target_dt * 1000000));
            }

            // static Timer fps_timer;
            // if (fps_timer.elapsed() > 1.0f) {
            //     fps_timer.reset();
            //     printf("Sim FPS: ");
            //     printf("phys: %d, ", physicsFps);
            //     physicsFps = 0;
            //     for (int i=0; i<6; i++) {
            //         printf("\tcam_%d: %d,", i, sensorsFps[i]);
            //         sensorsFps[i] = 0;
            //     }
            //     printf("\n");
            // }
        }
    }

    mj_deleteData(Data);
    mj_deleteModel(Model);
    exit(0);
}

