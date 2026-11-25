#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rbq_sdk::zn {

/// How a session joins the Zenoh network. The defaults are rmw_zenoh's: a peer that reaches the others through the
/// router on this host (tcp/localhost:7447), so ROS 2 nodes on rmw_zenoh and RBQ apps see each other unchanged.
/// The same environment variables rmw_zenoh reads apply here too:
///   ROS_DOMAIN_ID              domain (default 0)
///   ZENOH_SESSION_CONFIG_URI   a full json5 config file, used instead of the defaults
///   ZENOH_CONFIG_OVERRIDE      "key=value;key=value" (json5 values), applied last
struct SessionConfig {
    std::string mode = "peer";
    std::vector<std::string> connect = {"tcp/localhost:7447"};
    std::vector<std::string> listen = {"tcp/localhost:0"};
    /// Exchange large payloads through Zenoh shared memory with peers on this host that enable it too.
    bool shared_memory = false;
    int domain_id = 0;
    std::string config_file;
    std::string overrides;

    static SessionConfig fromEnv();
    /// What rbq_zenohd runs: a router on every interface, the one process each host's sessions connect to.
    static SessionConfig router(const std::string& listen = "tcp/[::]:7447");
};

class Session {
public:
    struct Impl;

    /// Open a session. Throws std::runtime_error if Zenoh refuses the config.
    static std::shared_ptr<Session> open(const SessionConfig& config = SessionConfig::fromEnv());
    /// The process-wide session, opened from the environment on first use.
    static std::shared_ptr<Session> global();
    /// Open a router (SessionConfig::router) unless something already listens on its port on this host (another
    /// app's router, rbq_zenohd, rmw_zenohd) — then return null and let this host's sessions use that one.
    static std::shared_ptr<Session> openRouterIfAbsent(const SessionConfig& config);

    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    int domainId() const { return domain_id_; }
    /// Zenoh session id as rmw_zenoh prints it in liveliness keys (32 hex digits).
    const std::string& zid() const { return zid_str_; }
    const std::array<uint8_t, 16>& zidBytes() const { return zid_; }
    /// Unique per session; rmw_zenoh numbers nodes and entities from one counter the same way.
    uint64_t nextEntityId() { return next_id_++; }

    Impl& impl() { return *impl_; }

private:
    explicit Session(std::unique_ptr<Impl> impl, int domain_id);

    std::unique_ptr<Impl> impl_;
    int domain_id_ = 0;
    std::array<uint8_t, 16> zid_{};
    std::string zid_str_;
    std::atomic<uint64_t> next_id_{0};
};

} // namespace rbq_sdk::zn
