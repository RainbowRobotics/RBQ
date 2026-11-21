#pragma once

#include <memory>
#include <string>

#include "Session.hpp"

namespace rbq_sdk::zn {

/// A named participant, as ROS 2 sees one: `ros2 node list` shows it and its publishers/subscribers. Topic names
/// resolve like rclcpp's — "/abs" stays, "rel" goes under the node namespace.
class Node {
public:
    explicit Node(const std::string& name, const std::string& ns = "/",
                  std::shared_ptr<Session> session = Session::global());
    ~Node();
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    const std::string& name() const { return name_; }
    const std::string& ns() const { return ns_; }
    uint64_t id() const { return id_; }
    Session& session() const { return *session_; }
    const std::shared_ptr<Session>& sessionPtr() const { return session_; }

    /// Fully qualified topic name ("/ns/topic").
    std::string resolve(const std::string& topic) const;

private:
    struct Token;
    std::shared_ptr<Session> session_;
    std::string name_;
    std::string ns_;
    uint64_t id_ = 0;
    std::unique_ptr<Token> token_;
};

} // namespace rbq_sdk::zn
