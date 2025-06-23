#ifndef Vision_IPC_H
#define Vision_IPC_H

#include <QObject>
#include <QSharedMemory>
#include <QDebug>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

typedef std::chrono::high_resolution_clock Clock_t;
typedef std::chrono::high_resolution_clock::time_point TimePoint_t;
typedef std::chrono::duration<double, std::ratio<1> > Seconds_t;

class Vision_IPC : public QObject
{
    Q_OBJECT

public:
    constexpr static int MAX_SENSOR_DEVICE = 20;

    enum Sensors_e : uint8_t {
        Bottom0         = 0,
        Bottom1         = 1,
        Bottom2         = 2,
        Bottom3         = 3,
        Front           = 4,
        Rear            = 5,
        Left            = 6,
        Right           = 7,
        CctvSensor      = 8,
        ThermalSensor   = 9,
        SlamRender      = 10,
        VioRender       = 11,
    };
    Q_ENUM(Sensors_e);

    enum Error_e : unsigned char {
        noError = 0,
        shmLockError = 1,
        sizeError = 2,
        idError = 2,
    };

    struct Depth_t
    {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;

        double time = 0;

        uint16_t width      = 0;
        uint16_t height     = 0;

        const static int MAX_DIMENSION  = 1920 * 1080;
        const static int CHANNELS_SIZE  = 1;

        uint16_t data [MAX_DIMENSION*CHANNELS_SIZE]  = {0,};

        float intrinsics[4]     = {0,}; // fx,fy,ppx,ppy
        float coeffs    [8]     = {0,}; // intrinsics / Distortion coefficients.
        float extrinsics[12]    = {0,}; // rotation[9], translation[3]

        // TF matrix w.r. base (body center)
        Eigen::Matrix4d TF = Eigen::Matrix4d::Identity();

        TimePoint_t timePointWrite;

        Depth_t()
        {
            timePointWrite = Clock_t::now();
        }
        Depth_t(const Depth_t &p)
        {
            time             = p.time;
            tickWrite        = p.tickWrite;
            tickRead         = p.tickRead;
            width            = p.width;
            height           = p.height;
            TF               = p.TF;
            timePointWrite   = p.timePointWrite;
            memcpy(intrinsics, p.intrinsics,  sizeof(intrinsics));
            memcpy(coeffs,     p.coeffs,      sizeof(coeffs));
            memcpy(extrinsics, p.extrinsics,  sizeof(extrinsics));
            memcpy(data,       p.data,        std::min<int>(CHANNELS_SIZE * width * height * sizeof(uint16_t), sizeof(data)));
        }

        Depth_t &operator=(const Depth_t &p)
        {
            time             = p.time;
            tickWrite        = p.tickWrite;
            tickRead         = p.tickRead;
            width            = p.width;
            height           = p.height;
            TF               = p.TF;
            timePointWrite   = p.timePointWrite;
            memcpy(intrinsics, p.intrinsics,  sizeof(intrinsics));
            memcpy(coeffs,     p.coeffs,      sizeof(coeffs));
            memcpy(extrinsics, p.extrinsics,  sizeof(extrinsics));
            memcpy(data,       p.data,        std::min<int>(CHANNELS_SIZE * width * height * sizeof(uint16_t), sizeof(data)));

            return *this;
        }

        double elapsed() const
        {
            return std::chrono::duration_cast<Seconds_t>
                (Clock_t::now() - timePointWrite).count();
        }
    };

    struct ImageColor_t
    {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;

        double time = 0;

        uint16_t width      = 0;
        uint16_t height     = 0;

        const static int MAX_DIMENSION      = 1920 * 1080;
        const static int CHANNELS_SIZE      = 3;

        uint8_t data    [MAX_DIMENSION*CHANNELS_SIZE]  = {0,};

        float intrinsics[4]     = {0,}; // fx,fy,ppx,ppy
        float coeffs    [8]     = {0,}; // intrinsics / Distortion coefficients.
        float extrinsics[12]    = {0,}; // rotation[9], translation[3]

        // TF matrix w.r. base (body center)
        Eigen::Matrix4d TF = Eigen::Matrix4d::Identity();

        TimePoint_t timePointWrite;

        ImageColor_t()
        {
            timePointWrite = Clock_t::now();
        }
        ImageColor_t(const ImageColor_t& p)
        {
            time             = p.time;
            tickWrite        = p.tickWrite;
            tickRead         = p.tickRead;
            width            = p.width;
            height           = p.height;
            TF               = p.TF;
            timePointWrite   = p.timePointWrite;
            memcpy(intrinsics, p.intrinsics,  sizeof(intrinsics));
            memcpy(coeffs,     p.coeffs,      sizeof(coeffs));
            memcpy(extrinsics, p.extrinsics,  sizeof(extrinsics));
            memcpy(data,       p.data,        std::min<int>(CHANNELS_SIZE * width * height * sizeof(uint8_t), sizeof(data)));
        }
        ImageColor_t& operator=(const ImageColor_t& p)
        {
            time             = p.time;
            tickWrite        = p.tickWrite;
            tickRead         = p.tickRead;
            width            = p.width;
            height           = p.height;
            TF               = p.TF;
            timePointWrite   = p.timePointWrite;
            memcpy(intrinsics, p.intrinsics,  sizeof(intrinsics));
            memcpy(coeffs,     p.coeffs,      sizeof(coeffs));
            memcpy(extrinsics, p.extrinsics,  sizeof(extrinsics));
            memcpy(data,       p.data,        std::min<int>(CHANNELS_SIZE * width * height * sizeof(uint8_t), sizeof(data)));
            return *this;
        }
        double elapsed() const
        {
            return std::chrono::duration_cast<Seconds_t>
                (Clock_t::now() - timePointWrite).count();
        }
    };

    struct ImageIR_t
    {
        uint32_t tickWrite  = 0;
        uint32_t tickRead   = 0;

        double time = 0;

        uint16_t width      = 0;
        uint16_t height     = 0;

        const static int MAX_DIMENSION      = 1920 * 1080;
        const static int CHANNELS_SIZE      = 1;

        uint8_t data    [MAX_DIMENSION*CHANNELS_SIZE]  = {0,};

        float intrinsics[4]     = {0,}; // fx,fy,ppx,ppy
        float coeffs    [8]     = {0,}; // intrinsics / Distortion coefficients.
        float extrinsics[12]    = {0,}; // rotation[9], translation[3]

        // TF matrix w.r. base (body center)
        Eigen::Matrix4d TF = Eigen::Matrix4d::Identity();

        TimePoint_t timePointWrite;

        ImageIR_t()
        {
            timePointWrite = Clock_t::now();
        }
        ImageIR_t(const ImageIR_t& p)
        {
            time             = p.time;
            tickWrite        = p.tickWrite;
            tickRead         = p.tickRead;
            width            = p.width;
            height           = p.height;
            TF               = p.TF;
            timePointWrite   = p.timePointWrite;
            memcpy(intrinsics, p.intrinsics,  sizeof(intrinsics));
            memcpy(coeffs,     p.coeffs,      sizeof(coeffs));
            memcpy(extrinsics, p.extrinsics,  sizeof(extrinsics));
            memcpy(data,       p.data,        std::min<int>(CHANNELS_SIZE * width * height * sizeof(uint8_t), sizeof(data)));
        }
        ImageIR_t& operator=(const ImageIR_t& p)
        {
            time             = p.time;
            tickWrite        = p.tickWrite;
            tickRead         = p.tickRead;
            width            = p.width;
            height           = p.height;
            TF               = p.TF;
            timePointWrite   = p.timePointWrite;
            memcpy(intrinsics, p.intrinsics,  sizeof(intrinsics));
            memcpy(coeffs,     p.coeffs,      sizeof(coeffs));
            memcpy(extrinsics, p.extrinsics,  sizeof(extrinsics));
            memcpy(data,       p.data,        std::min<int>(CHANNELS_SIZE * width * height * sizeof(uint8_t), sizeof(data)));
            return *this;
        }
        double elapsed() const
        {
            return std::chrono::duration_cast<Seconds_t>
                (Clock_t::now() - timePointWrite).count();
        }
    };

public:
    explicit Vision_IPC(QObject *parent = nullptr);
    ~Vision_IPC();

    uint32_t getTickWrite(QSharedMemory* shm);
    uint32_t getTickRead(QSharedMemory* shm);

    double getTime();
    int64_t getTimeSeconds();
    int64_t getUpTimeSeconds();
private:
    std::chrono::steady_clock::time_point m_startTime       = std::chrono::steady_clock::now(); // App start time

    // default vision sensors -------------------------------------------------------------------
    const static int shm_sensors_total = MAX_SENSOR_DEVICE;
public:
    uint32_t shm_depth_tickRead   [shm_sensors_total]     = {0,};
    uint32_t shm_depth_tickWrite  [shm_sensors_total]     = {0,};
    const QString shm_depth_key[shm_sensors_total]  = { "shm_depth_0",  "shm_depth_1",  "shm_depth_2",  "shm_depth_3",  "shm_depth_4",
                                                      "shm_depth_5",  "shm_depth_6",  "shm_depth_7",  "shm_depth_8",  "shm_depth_9",
                                                      "shm_depth_10", "shm_depth_11", "shm_depth_12", "shm_depth_13", "shm_depth_14",
                                                      "shm_depth_15", "shm_depth_16", "shm_depth_17", "shm_depth_18", "shm_depth_19"};
    QSharedMemory *shm_depth[shm_sensors_total]    = {nullptr, };
    Vision_IPC::Error_e setDepth(Vision_IPC::Sensors_e id, Vision_IPC::Depth_t &newDepth);
    Vision_IPC::Error_e getDepth(Vision_IPC::Sensors_e id, Vision_IPC::Depth_t &outDepth);

    uint32_t shm_color_tickRead   [shm_sensors_total]     = {0,};
    uint32_t shm_color_tickWrite  [shm_sensors_total]     = {0,};
    const QString shm_color_key[shm_sensors_total]  = { "shm_color_0",  "shm_color_1",  "shm_color_2",  "shm_color_3",  "shm_color_4",
                                                      "shm_color_5",  "shm_color_6",  "shm_color_7",  "shm_color_8",  "shm_color_9",
                                                      "shm_color_10", "shm_color_11", "shm_color_12", "shm_color_13", "shm_color_14",
                                                      "shm_color_15", "shm_color_16", "shm_color_17", "shm_color_18", "shm_color_19"};
    QSharedMemory *shm_color[shm_sensors_total]    = {nullptr, };
    Vision_IPC::Error_e setImageColor(Vision_IPC::Sensors_e id, Vision_IPC::ImageColor_t &newImageColor);
    Vision_IPC::Error_e getImageColor(Vision_IPC::Sensors_e id, Vision_IPC::ImageColor_t &outImageColor);

    uint32_t shm_ir_tickRead   [shm_sensors_total]     = {0,};
    uint32_t shm_ir_tickWrite  [shm_sensors_total]     = {0,};
    const QString shm_ir_key[shm_sensors_total]  = {"shm_ir_0",  "shm_ir_1",  "shm_ir_2",  "shm_ir_3",  "shm_ir_4",
                                                   "shm_ir_5",  "shm_ir_6",  "shm_ir_7",  "shm_ir_8",  "shm_ir_9",
                                                   "shm_ir_10", "shm_ir_11", "shm_ir_12", "shm_ir_13", "shm_ir_14",
                                                   "shm_ir_15", "shm_ir_16", "shm_ir_17", "shm_ir_18", "shm_ir_19"};
    QSharedMemory *shm_ir[shm_sensors_total]    = {nullptr, };
    Vision_IPC::Error_e setImageIR(Vision_IPC::Sensors_e id, Vision_IPC::ImageIR_t &newImageIR);
    Vision_IPC::Error_e getImageIR(Vision_IPC::Sensors_e id, Vision_IPC::ImageIR_t &outImageIR);
    // -------------------------------------------------------------------------------------------
};

#endif // Vision_IPC_H
