#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

// Livox Mid360 non-repetitive scan pattern + MuJoCo CPU raytracer. The pattern is a
// float32 [theta, phi] table; each frame casts a rolling window of it with mj_multiRay.
class MjLidar
{
public:
    // patternPath holds interleaved [theta, phi] float32 pairs; cutoff is the max range
    // in metres and framePeriod the scan period, used to synthesize per-point times.
    MjLidar(const std::string &patternPath, int samplesPerFrame, float cutoff, float framePeriod);

    bool valid() const { return nRays_ > 0; }

    // Casts this frame's rays from site siteId, ignoring bodyExclude's geoms, and advances
    // the ring buffer. Fills xyz (3N, sensor frame), intensity (N, = range) and times (N,
    // offset within the frame); returns N. Caller holds the sim mutex — this writes into d.
    int scan(const mjModel *m, mjData *d, int siteId, int bodyExclude,
             std::vector<float> &xyz, std::vector<float> &intensity,
             std::vector<float> &times);

    int samplesPerFrame() const { return samples_; }

private:
    std::vector<float>  pattern_;   // interleaved theta,phi; size = 2 * nRays_
    int                 nRays_   = 0;
    int                 samples_ = 0;
    float               cutoff_  = 0.f;
    float               framePeriod_ = 0.f;
    int                 cursor_  = 0;   // ring-buffer start (in ray units)

    // Reused per-frame scratch, sized to samples_.
    std::vector<mjtNum> vec_;       // 3 * samples_ world-frame ray dirs
    std::vector<mjtNum> dist_;      // samples_ hit distances
    std::vector<int>    geomid_;    // samples_ hit geom ids
    std::vector<float>  localX_;    // samples_ sensor-frame unit dirs (for hit pts)
    std::vector<float>  localY_;
    std::vector<float>  localZ_;
};
