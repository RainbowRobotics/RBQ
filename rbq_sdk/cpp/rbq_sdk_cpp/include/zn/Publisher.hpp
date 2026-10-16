#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "Cdr.hpp"
#include "Node.hpp"
#include "Qos.hpp"
#include "TypeInfoTable.hpp"

namespace rbq_sdk::zn {

/// Untyped half of Publisher<T>: the Zenoh publisher, its ROS graph token and the per-sample attachment.
class PublisherBase {
public:
    PublisherBase(Node& node, const std::string& topic, const char* dds_type, const char* type_hash, const Qos& qos);
    ~PublisherBase();
    PublisherBase(const PublisherBase&) = delete;
    PublisherBase& operator=(const PublisherBase&) = delete;

    /// Send one CDR payload (header included). Takes the buffer to avoid a copy.
    bool put(std::vector<uint8_t>&& cdr);
    /// True while at least one subscriber (RBQ app or ROS node) matches this topic.
    bool hasSubscribers() const;

    const std::string& topic() const { return topic_; }
    const std::string& keyExpr() const { return key_; }
    /// The id every sample carries (MessageInfo::publisher_gid on the receiving side).
    const std::array<uint8_t, 16>& gid() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::string topic_;
    std::string key_;
};

/// Publishes T on a topic. publish() is thread-safe; it returns false only if T can't be serialized or Zenoh
/// rejects the sample.
template <typename T>
class Publisher {
public:
    Publisher(Node& node, const std::string& topic, const Qos& qos = Qos())
        : base_(node, topic, TypeInfo<T>::dds_name, TypeInfo<T>::hash, qos) {}

    bool publish(const T& msg)
    {
        std::vector<uint8_t> buf;
        if (!cdr::serialize(msg, buf)) return false;
        return base_.put(std::move(buf));
    }

    bool hasSubscribers() const { return base_.hasSubscribers(); }
    const std::string& topic() const { return base_.topic(); }
    const std::array<uint8_t, 16>& gid() const { return base_.gid(); }

private:
    PublisherBase base_;
};

} // namespace rbq_sdk::zn
