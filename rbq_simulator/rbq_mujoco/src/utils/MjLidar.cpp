#include "MjLidar.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>

MjLidar::MjLidar(const std::string &patternPath, int samplesPerFrame, float cutoff, float framePeriod)
    : samples_(samplesPerFrame), cutoff_(cutoff), framePeriod_(framePeriod)
{
    std::ifstream f(patternPath, std::ios::binary | std::ios::ate);
    if (!f) {
        std::cerr << "[MjLidar] cannot open scan pattern: " << patternPath << "\n";
        return;
    }
    const std::streamsize bytes = f.tellg();
    if (bytes <= 0 || bytes % (2 * sizeof(float)) != 0) {
        std::cerr << "[MjLidar] bad scan pattern size: " << bytes << " bytes \n";
        return;
    }
    f.seekg(0);
    pattern_.resize(static_cast<size_t>(bytes) / sizeof(float));
    f.read(reinterpret_cast<char *>(pattern_.data()), bytes);
    nRays_ = static_cast<int>(pattern_.size() / 2);

    if (samples_ > nRays_) samples_ = nRays_;
    vec_.resize(static_cast<size_t>(samples_) * 3);
    dist_.resize(samples_);
    geomid_.resize(samples_);
    localX_.resize(samples_);
    localY_.resize(samples_);
    localZ_.resize(samples_);

    std::cout << "[MjLidar] loaded " << nRays_ << " rays, " << samples_
              << " samples/frame from " << patternPath << "\n";
}

int MjLidar::scan(const mjModel *m, mjData *d, int siteId, int bodyExclude,
                  std::vector<float> &xyz, std::vector<float> &intensity,
                  std::vector<float> &times)
{
    if (nRays_ <= 0 || siteId < 0) return 0;

    const mjtNum *pos = d->site_xpos + 3 * siteId;
    const mjtNum *mat = d->site_xmat + 9 * siteId;  // row-major 3x3
    const mjtNum pnt[3] = {pos[0], pos[1], pos[2]};

    // site_xmat is all zeros until the first mj_forward, and zero-length ray dirs abort
    // mj_multiRay; a populated rotation row has norm 1.
    const double rowNorm2 = mat[0] * mat[0] + mat[1] * mat[1] + mat[2] * mat[2];
    if (rowNorm2 < 0.5) return 0;

    // Build this frame's world-frame ray directions from the rolling window.
    for (int i = 0; i < samples_; ++i) {
        const int r = (cursor_ + i) % nRays_;
        const float theta = pattern_[2 * r + 0];
        const float phi   = pattern_[2 * r + 1];
        const float cphi  = std::cos(phi);
        const float lx = cphi * std::cos(theta);
        const float ly = cphi * std::sin(theta);
        const float lz = std::sin(phi);
        localX_[i] = lx;
        localY_[i] = ly;
        localZ_[i] = lz;
        // world = R * local, R row-major.
        vec_[3 * i + 0] = mat[0] * lx + mat[1] * ly + mat[2] * lz;
        vec_[3 * i + 1] = mat[3] * lx + mat[4] * ly + mat[5] * lz;
        vec_[3 * i + 2] = mat[6] * lx + mat[7] * ly + mat[8] * lz;
    }
    cursor_ = (cursor_ + samples_) % nRays_;

    mj_multiRay(m, d, pnt, vec_.data(), /*geomgroup=*/nullptr, /*flg_static=*/1,
                bodyExclude, geomid_.data(), dist_.data(), samples_,
                static_cast<mjtNum>(cutoff_));

    xyz.clear();
    intensity.clear();
    times.clear();
    xyz.reserve(static_cast<size_t>(samples_) * 3);
    intensity.reserve(samples_);
    times.reserve(samples_);
    const float invSamples = samples_ > 0 ? framePeriod_ / static_cast<float>(samples_) : 0.f;
    int n = 0;
    for (int i = 0; i < samples_; ++i) {
        const float dist = static_cast<float>(dist_[i]);
        if (geomid_[i] < 0 || dist <= 0.f || dist >= cutoff_) continue;
        xyz.push_back(dist * localX_[i]);
        xyz.push_back(dist * localY_[i]);
        xyz.push_back(dist * localZ_[i]);
        intensity.push_back(dist);  // no material reflectivity available; use range
        times.push_back(static_cast<float>(i) * invSamples);  // acquisition offset in frame
        ++n;
    }
    return n;
}
