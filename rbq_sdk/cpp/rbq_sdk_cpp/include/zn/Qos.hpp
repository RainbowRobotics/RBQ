#pragma once

#include <chrono>
#include <cstdint>

namespace rbq_sdk::zn {

/// How a topic is delivered. There is deliberately no "transient local" durability: nothing is cached for
/// late joiners — a subscriber only sees what is published after it starts.
struct Qos {
    enum class Reliability { Reliable, BestEffort };

    Reliability reliability = Reliability::Reliable;
    /// Advertised history depth (ROS graph). Delivery itself keeps no queue beyond the callback.
    uint32_t depth = 10;
    /// Block the publisher instead of dropping when the network can't keep up. Off for anything periodic.
    bool block_when_congested = false;
    /// Subscriber side: drop samples whose source timestamp is older than this. 0 = keep everything (the default).
    /// It compares the publisher's wall clock with the subscriber's — set it only when both run on one host or on
    /// synced clocks; across unsynced PCs a skew drops every sample (or lets stale ones through).
    std::chrono::nanoseconds max_age{0};

    /// Commands: every one must arrive (reliable). Pass max_age only between processes that share a clock.
    static Qos command(std::chrono::milliseconds max_age = std::chrono::milliseconds(0))
    {
        Qos q;
        q.reliability = Reliability::Reliable;
        q.depth = 10;
        q.max_age = max_age;
        return q;
    }
    /// Frames and other high-rate streams: newest wins, a slow subscriber loses frames rather than lagging.
    static Qos stream(std::chrono::milliseconds max_age = std::chrono::milliseconds(0))
    {
        Qos q;
        q.reliability = Reliability::BestEffort;
        q.depth = 1;
        q.max_age = max_age;
        return q;
    }
    /// State republished at a fixed rate: a late subscriber is current after one period.
    static Qos state(std::chrono::milliseconds max_age = std::chrono::milliseconds(0))
    {
        Qos q;
        q.reliability = Reliability::Reliable;
        q.depth = 1;
        q.max_age = max_age;
        return q;
    }
};

} // namespace rbq_sdk::zn
