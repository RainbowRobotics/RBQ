#pragma once

#include <chrono>

namespace rbq_sdk {

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
                (clock_::now() - beg_).count(); }
    static double GetTime() {
        std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
        return (timestamp*1.0e-9);
    }
    static double GetTimeSince(const double &past) {
        std::chrono::time_point<std::chrono::system_clock> t = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
        return (timestamp*1.0e-9 - past);
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

} // namespace rbq_sdk
