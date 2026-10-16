#pragma once
//
// Shared sensor_msgs/PointCloud2 (+ Imu) helpers so LiDAR producers and consumers
// cannot drift apart, and stay readable by generic ROS2 tooling.
//
// Producer layout, byte-packed, point_step 28:
//   x,y,z,intensity,time : float32   (time = seconds offset from header.stamp)
//   timestamp            : float64   (absolute seconds)
// Both time aliases are emitted so Velodyne-style `time` and Livox-style `timestamp`
// consumers each find the one they expect.
//
// Consumers parse by field name/offset/datatype, never a fixed layout, so the same
// code also ingests Livox, Ouster (`t`, u32 ns) and Velodyne clouds.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <rbq_sdk/idl/ros2/PointCloud2_.hpp>
#include <rbq_sdk/idl/ros2/PointField_.hpp>
#include <rbq_sdk/idl/ros2/Imu_.hpp>

namespace LidarDds
{
// sensor_msgs::PointField datatypes.
enum : uint8_t { INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4,
                 INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8 };

// ---- Producer -------------------------------------------------------------

constexpr uint32_t POINT_STEP = 28;  // x,y,z,intensity,time (5*f32) + timestamp (f64)

inline sensor_msgs::msg::dds_::PointField_ makeField(const std::string &name, uint32_t offset,
                                                     uint8_t datatype)
{
    sensor_msgs::msg::dds_::PointField_ f;
    f.name()     = name;
    f.offset()   = offset;
    f.datatype() = datatype;
    f.count()    = 1;
    return f;
}

// Build a PointCloud2 from parallel arrays. xyz has 3*n floats; intensity and
// times each have n floats (times = per-point seconds offset from the stamp).
inline void fillPointCloud2(sensor_msgs::msg::dds_::PointCloud2_ &msg,
                            const std::string &frameId, uint32_t sec, uint32_t nanosec,
                            uint32_t n, const float *xyz, const float *intensity,
                            const float *times)
{
    msg.header().stamp().sec()     = sec;
    msg.header().stamp().nanosec() = nanosec;
    msg.header().frame_id()        = frameId;
    msg.height()       = 1;
    msg.width()        = n;
    msg.fields()       = {makeField("x", 0, FLOAT32),   makeField("y", 4, FLOAT32),
                          makeField("z", 8, FLOAT32),   makeField("intensity", 12, FLOAT32),
                          makeField("time", 16, FLOAT32), makeField("timestamp", 20, FLOAT64)};
    msg.is_bigendian() = false;
    msg.point_step()   = POINT_STEP;
    msg.row_step()     = POINT_STEP * n;
    msg.is_dense()     = true;

    const double stampAbs = static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9;
    std::vector<uint8_t> &data = msg.data();
    data.resize(static_cast<size_t>(n) * POINT_STEP);
    for (uint32_t i = 0; i < n; ++i) {
        uint8_t *p = data.data() + static_cast<size_t>(i) * POINT_STEP;
        const double tsAbs = stampAbs + static_cast<double>(times[i]);
        std::memcpy(p + 0,  &xyz[3 * i + 0], sizeof(float));
        std::memcpy(p + 4,  &xyz[3 * i + 1], sizeof(float));
        std::memcpy(p + 8,  &xyz[3 * i + 2], sizeof(float));
        std::memcpy(p + 12, &intensity[i],   sizeof(float));
        std::memcpy(p + 16, &times[i],       sizeof(float));
        std::memcpy(p + 20, &tsAbs,          sizeof(double));
    }
}

// Build an Imu in SI units (gyro rad/s, accel m/s^2). Orientation marked unknown
// per REP-145 so consumers estimate it themselves.
inline void fillImu(sensor_msgs::msg::dds_::Imu_ &msg, const std::string &frameId,
                    uint32_t sec, uint32_t nanosec, const double gyr[3], const double acc[3])
{
    msg.header().stamp().sec()     = sec;
    msg.header().stamp().nanosec() = nanosec;
    msg.header().frame_id()        = frameId;
    msg.orientation().w() = 1.0;
    msg.orientation().x() = 0.0;
    msg.orientation().y() = 0.0;
    msg.orientation().z() = 0.0;
    msg.orientation_covariance()[0]  = -1.0;  // orientation unknown (REP-145)
    msg.angular_velocity().x()    = gyr[0];
    msg.angular_velocity().y()    = gyr[1];
    msg.angular_velocity().z()    = gyr[2];
    msg.linear_acceleration().x() = acc[0];
    msg.linear_acceleration().y() = acc[1];
    msg.linear_acceleration().z() = acc[2];
}

// ---- Consumer (generic, layout-agnostic) ----------------------------------

struct FieldRef {
    int     offset   = -1;
    uint8_t datatype = 0;
    bool    ok() const { return offset >= 0; }
};

inline FieldRef findField(const sensor_msgs::msg::dds_::PointCloud2_ &m, const std::string &name)
{
    for (const auto &f : m.fields())
        if (f.name() == name) return {static_cast<int>(f.offset()), f.datatype()};
    return {};
}

inline double readScalar(const uint8_t *base, const FieldRef &f)
{
    const uint8_t *p = base + f.offset;
    switch (f.datatype) {
        case FLOAT32: { float    v; std::memcpy(&v, p, 4); return v; }
        case FLOAT64: { double   v; std::memcpy(&v, p, 8); return v; }
        case UINT32:  { uint32_t v; std::memcpy(&v, p, 4); return static_cast<double>(v); }
        case INT32:   { int32_t  v; std::memcpy(&v, p, 4); return static_cast<double>(v); }
        case UINT16:  { uint16_t v; std::memcpy(&v, p, 2); return static_cast<double>(v); }
        case INT16:   { int16_t  v; std::memcpy(&v, p, 2); return static_cast<double>(v); }
        case UINT8:   { return static_cast<double>(*p); }
        case INT8:    { int8_t   v; std::memcpy(&v, p, 1); return static_cast<double>(v); }
        default:      return 0.0;
    }
}

// Resolves x/y/z/intensity + a per-point time field by name once, then reads
// points from any PointCloud2 (ours, Livox, Ouster, Velodyne). Time is returned
// as a seconds offset from the header stamp regardless of source convention.
struct CloudReader {
    const sensor_msgs::msg::dds_::PointCloud2_ *m = nullptr;
    uint32_t n = 0, step = 0;
    double   stampAbs = 0.0;
    FieldRef fx, fy, fz, fint, ftime;
    int      timeMode = 0;  // 0 none, 1 rel-sec, 2 abs-sec, 3 rel-nanosec

    explicit CloudReader(const sensor_msgs::msg::dds_::PointCloud2_ &msg) : m(&msg)
    {
        n    = msg.width() * (msg.height() ? msg.height() : 1);
        step = msg.point_step();
        stampAbs = static_cast<double>(msg.header().stamp().sec()) +
                   static_cast<double>(msg.header().stamp().nanosec()) * 1e-9;
        fx   = findField(msg, "x");
        fy   = findField(msg, "y");
        fz   = findField(msg, "z");
        fint = findField(msg, "intensity");
        if (!fint.ok()) fint = findField(msg, "reflectivity");
        // Time aliases, in priority order.
        ftime = findField(msg, "time");            // Velodyne/LIO-SAM (rel sec)
        if (ftime.ok()) { timeMode = 1; }
        else {
            ftime = findField(msg, "timestamp");   // Livox PC2 (abs sec)
            if (ftime.ok()) { timeMode = 2; }
            else {
                ftime = findField(msg, "t");       // Ouster (rel ns)
                timeMode = ftime.ok() ? 3 : 0;
            }
        }
    }

    bool valid() const { return m && step >= 12 && fx.ok() && fy.ok() && fz.ok(); }

    const uint8_t *pt(uint32_t i) const { return m->data().data() + static_cast<size_t>(i) * step; }

    void xyz(uint32_t i, double out[3]) const
    {
        const uint8_t *p = pt(i);
        out[0] = readScalar(p, fx);
        out[1] = readScalar(p, fy);
        out[2] = readScalar(p, fz);
    }

    double intensity(uint32_t i) const { return fint.ok() ? readScalar(pt(i), fint) : 0.0; }

    // Per-point time as seconds offset from the header stamp (0 if no time field).
    double timeOffset(uint32_t i) const
    {
        if (timeMode == 0) return 0.0;
        const double v = readScalar(pt(i), ftime);
        if (timeMode == 2) return v - stampAbs;   // abs -> rel
        if (timeMode == 3) return v * 1e-9;        // ns  -> sec
        return v;                                   // already rel-sec
    }
};
}  // namespace LidarDds
