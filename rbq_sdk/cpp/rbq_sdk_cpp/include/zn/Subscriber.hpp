#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "Cdr.hpp"
#include "Node.hpp"
#include "Qos.hpp"
#include "TypeInfoTable.hpp"

namespace rbq_sdk::zn {

/// Where a sample came from — the rmw_zenoh attachment.
struct MessageInfo {
    int64_t sequence_number = 0;
    int64_t source_timestamp_ns = 0;   ///< publisher's wall clock (system_clock, ns since epoch)
    int64_t received_timestamp_ns = 0; ///< this process's wall clock at arrival
    std::array<uint8_t, 16> publisher_gid{};
};

/// Untyped half of Subscriber<T>. Samples without a valid attachment, or older than Qos::max_age, never reach
/// the callback.
class SubscriberBase {
public:
    using RawCallback = std::function<void(const uint8_t* data, size_t size, const MessageInfo& info)>;

    SubscriberBase(Node& node, const std::string& topic, const char* dds_type, const char* type_hash, const Qos& qos,
                   RawCallback cb);
    ~SubscriberBase();
    SubscriberBase(const SubscriberBase&) = delete;
    SubscriberBase& operator=(const SubscriberBase&) = delete;

    const std::string& topic() const { return topic_; }
    const std::string& keyExpr() const { return key_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::string topic_;
    std::string key_;
};

/// Calls cb for every sample of T published on topic after this subscriber exists — nothing published before is
/// replayed. The callback runs on a Zenoh thread: keep it short, and synchronize anything it shares.
template <typename T>
class Subscriber {
public:
    using Callback = std::function<void(const T& msg, const MessageInfo& info)>;

    Subscriber(Node& node, const std::string& topic, Callback cb, const Qos& qos = Qos())
        : base_(node, topic, TypeInfo<T>::dds_name, TypeInfo<T>::hash, qos,
                [cb = std::move(cb)](const uint8_t* data, size_t size, const MessageInfo& info) {
                    T msg;
                    if (cdr::deserialize(data, size, msg)) cb(msg, info);
                })
    {
    }

    const std::string& topic() const { return base_.topic(); }

private:
    SubscriberBase base_;
};

} // namespace rbq_sdk::zn
