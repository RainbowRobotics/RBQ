// rbq_sdk Zenoh layer: sessions, nodes, publishers and subscribers that ROS 2 nodes on rmw_zenoh (Jazzy) see as
// ordinary ROS entities. Every wire detail below follows rmw_zenoh_cpp 0.2.x (docs/design.md):
//   data key       <domain>/<topic without leading '/'>/<dds type>/<RIHS01 hash>
//   attachment     int64 seq LE | int64 source ns LE | 0x10 | 16-byte gid   (zenoh-ext serialization, 33 bytes)
//   graph tokens   @ros2_lv/<domain>/<zid>/<nid>/<eid>/<NN|MP|MS>/<enclave>/<ns>/<node>[/<topic>/<type>/<hash>/<qos>]
//                  with every '/' inside a field written as '%'

#include "zn/Node.hpp"
#include "zn/Publisher.hpp"
#include "zn/Session.hpp"
#include "zn/Subscriber.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <zenoh.h>

namespace rbq_sdk::zn {

namespace {

constexpr size_t kAttachmentSize = 33;

int64_t nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string mangle(const std::string& s)
{
    std::string out = s;
    for (char& c : out)
        if (c == '/') c = '%';
    return out;
}

/// rmw_zenoh's strip_slashes: one leading and one trailing '/'.
std::string stripSlashes(std::string s)
{
    if (!s.empty() && s.front() == '/') s.erase(0, 1);
    if (!s.empty() && s.back() == '/') s.pop_back();
    return s;
}

/// rmw_zenoh qos_to_keyexpr: a field equal to rmw_zenoh's default (reliable, volatile, keep-last, depth 42,
/// infinite deadline/lifespan/lease, automatic liveliness) is left empty.
std::string qosString(const Qos& q)
{
    std::ostringstream s;
    s << (q.reliability == Qos::Reliability::BestEffort ? "2" : "") << "::,";
    if (q.depth != 42) s << q.depth;
    s << ":,:,:,,";
    return s.str();
}

std::array<uint8_t, 16> makeGid(const Session& session, uint64_t eid)
{
    std::array<uint8_t, 16> gid = session.zidBytes();
    for (int i = 0; i < 8; ++i) gid[i] ^= uint8_t(eid >> (8 * i));
    return gid;
}

void putLE64(uint8_t* p, int64_t v)
{
    for (int i = 0; i < 8; ++i) p[i] = uint8_t(uint64_t(v) >> (8 * i));
}

int64_t getLE64(const uint8_t* p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= uint64_t(p[i]) << (8 * i);
    return int64_t(v);
}

bool insertJson5(z_loaned_config_t* cfg, const std::string& key, const std::string& value)
{
    if (zc_config_insert_json5(cfg, key.c_str(), value.c_str()) < 0) {
        std::cerr << "[rbq_sdk::zn] invalid config " << key << "=" << value << std::endl;
        return false;
    }
    return true;
}

std::string jsonList(const std::vector<std::string>& v)
{
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) s += (i ? ",\"" : "\"") + v[i] + "\"";
    return s + "]";
}

} // namespace

namespace detail {
/// One liveliness token: declared on construction, undeclared on destruction.
class LivelinessToken {
public:
    LivelinessToken(Session& session, const std::string& key);
    ~LivelinessToken() { if (ok_) z_drop(z_move(token_)); }
    LivelinessToken(const LivelinessToken&) = delete;
    LivelinessToken& operator=(const LivelinessToken&) = delete;

private:
    z_owned_liveliness_token_t token_;
    bool ok_ = false;
};
} // namespace detail

using detail::LivelinessToken;

// ---------------------------------------------------------------------------------------------------------------
// Session

struct Session::Impl {
    z_owned_session_t session;
};

SessionConfig SessionConfig::fromEnv()
{
    SessionConfig c;
    if (const char* d = std::getenv("ROS_DOMAIN_ID")) c.domain_id = std::atoi(d);
    if (const char* f = std::getenv("ZENOH_SESSION_CONFIG_URI")) c.config_file = f;
    if (const char* o = std::getenv("ZENOH_CONFIG_OVERRIDE")) c.overrides = o;
    return c;
}

SessionConfig SessionConfig::router(const std::string& listen)
{
    SessionConfig c = fromEnv();
    c.config_file.clear();
    c.mode = "router";
    c.connect.clear();
    c.listen = {listen};
    return c;
}

std::shared_ptr<Session> Session::open(const SessionConfig& config)
{
    static std::once_flag logInit;
    std::call_once(logInit, [] { zc_init_log_from_env_or("error"); });

    z_owned_config_t cfg;
    if (!config.config_file.empty()) {
        if (zc_config_from_file(&cfg, config.config_file.c_str()) < 0)
            throw std::runtime_error("zenoh: cannot read config file " + config.config_file);
    } else {
        z_config_default(&cfg);
        z_loaned_config_t* c = z_loan_mut(cfg);
        insertJson5(c, "mode", "\"" + config.mode + "\"");
        insertJson5(c, "connect/endpoints", jsonList(config.connect));
        insertJson5(c, "listen/endpoints", jsonList(config.listen));
        insertJson5(c, "scouting/multicast/enabled", "false");
        // zenoh-c 1.x turns shared memory on by default; keep it off unless asked, as rmw_zenoh does, so a small
        // /dev/shm (containers) never decides how a session behaves.
        insertJson5(c, "transport/shared_memory/enabled", config.shared_memory ? "true" : "false");
        // A router stamps samples with its HLC and logs every sample stamped >500 ms ahead of its own clock, so two
        // PCs with skewed clocks flood the log at the publish rate. Nothing here reads Zenoh timestamps (MessageInfo
        // carries the publisher's clock in the attachment), so routers stamp nothing.
        insertJson5(c, "timestamping/enabled", "false");
    }
    // ZENOH_CONFIG_OVERRIDE: "key=value;key=value", as rmw_zenoh reads it.
    std::istringstream ov(config.overrides);
    for (std::string kv; std::getline(ov, kv, ';');) {
        const auto eq = kv.find('=');
        if (eq != std::string::npos) insertJson5(z_loan_mut(cfg), kv.substr(0, eq), kv.substr(eq + 1));
    }

    auto impl = std::make_unique<Impl>();
    if (z_open(&impl->session, z_move(cfg), nullptr) < 0) throw std::runtime_error("zenoh: z_open failed");

    std::shared_ptr<Session> s(new Session(std::move(impl), config.domain_id));
    return s;
}

std::shared_ptr<Session> Session::openRouterIfAbsent(const SessionConfig& config)
{
    // The listen endpoint's port ("tcp/[::]:7447" -> 7447); a local TCP connect tells whether it is taken.
    const std::string ep = config.listen.empty() ? std::string("tcp/[::]:7447") : config.listen.front();
    const int port = std::atoi(ep.substr(ep.rfind(':') + 1).c_str());
    if (port > 0) {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(uint16_t(port));
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        const bool taken = fd >= 0 && ::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
        if (fd >= 0) ::close(fd);
        if (taken) return nullptr;
    }
    try {
        return open(config);
    } catch (const std::exception&) {
        return nullptr;   // lost a race with another router starting
    }
}

std::shared_ptr<Session> Session::global()
{
    static std::mutex m;
    static std::shared_ptr<Session> s;
    std::lock_guard<std::mutex> lock(m);
    if (!s) s = open();
    return s;
}

Session::Session(std::unique_ptr<Impl> impl, int domain_id) : impl_(std::move(impl)), domain_id_(domain_id)
{
    const z_id_t id = z_info_zid(z_loan(impl_->session));
    std::memcpy(zid_.data(), id.id, 16);
    z_owned_string_t str;
    z_id_to_string(&id, &str);
    zid_str_.assign(z_string_data(z_loan(str)), z_string_len(z_loan(str)));
    z_drop(z_move(str));
}

Session::~Session()
{
    z_drop(z_move(impl_->session));
}

LivelinessToken::LivelinessToken(Session& session, const std::string& key)
{
    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, key.c_str()) < 0) {
        std::cerr << "[rbq_sdk::zn] invalid liveliness key " << key << std::endl;
        return;
    }
    ok_ = z_liveliness_declare_token(z_loan(session.impl().session), &token_, z_loan(ke), nullptr) >= 0;
    if (!ok_) std::cerr << "[rbq_sdk::zn] liveliness token failed: " << key << std::endl;
}

namespace {
std::string graphPrefix(const Node& node, uint64_t eid)
{
    std::ostringstream s;
    s << "@ros2_lv/" << node.session().domainId() << "/" << node.session().zid() << "/" << node.id() << "/" << eid;
    return s.str();
}

std::string nodeFields(const Node& node)
{
    return "%/" + mangle(node.ns()) + "/" + node.name();   // enclave "/" -> "%"
}

std::string dataKey(const Node& node, const std::string& fqTopic, const char* dds, const char* hash)
{
    std::ostringstream s;
    s << node.session().domainId() << "/" << stripSlashes(fqTopic) << "/" << dds << "/" << hash;
    return s.str();
}

std::string entityToken(const Node& node, uint64_t eid, const char* kind, const std::string& fqTopic,
                        const char* dds, const char* hash, const Qos& qos)
{
    return graphPrefix(node, eid) + "/" + kind + "/" + nodeFields(node) + "/" + mangle(fqTopic) + "/" + dds + "/" +
           hash + "/" + qosString(qos);
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------
// Node

struct Node::Token : LivelinessToken {
    using LivelinessToken::LivelinessToken;
};

Node::Node(const std::string& name, const std::string& ns, std::shared_ptr<Session> session)
    : session_(std::move(session)), name_(name), ns_(ns.empty() ? "/" : ns)
{
    if (ns_.front() != '/') ns_.insert(0, "/");
    if (ns_.size() > 1 && ns_.back() == '/') ns_.pop_back();
    id_ = session_->nextEntityId();
    token_ = std::make_unique<Token>(*session_, graphPrefix(*this, id_) + "/NN/" + nodeFields(*this));
}

Node::~Node() = default;

std::string Node::resolve(const std::string& topic) const
{
    if (!topic.empty() && topic.front() == '/') return topic;
    return (ns_ == "/" ? "" : ns_) + "/" + topic;
}

// ---------------------------------------------------------------------------------------------------------------
// Publisher

struct PublisherBase::Impl {
    std::shared_ptr<Session> session;
    z_owned_publisher_t publisher;
    std::unique_ptr<LivelinessToken> token;
    std::array<uint8_t, 16> gid;
    std::mutex seq_mutex;
    int64_t seq = 0;
};

PublisherBase::PublisherBase(Node& node, const std::string& topic, const char* dds_type, const char* type_hash,
                             const Qos& qos)
    : impl_(std::make_unique<Impl>()), topic_(node.resolve(topic))
{
    impl_->session = node.sessionPtr();
    key_ = dataKey(node, topic_, dds_type, type_hash);
    const uint64_t eid = node.session().nextEntityId();
    impl_->gid = makeGid(node.session(), eid);

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, key_.c_str()) < 0) throw std::runtime_error("zenoh: invalid key " + key_);
    z_publisher_options_t opts;
    z_publisher_options_default(&opts);
    opts.congestion_control = qos.block_when_congested ? Z_CONGESTION_CONTROL_BLOCK : Z_CONGESTION_CONTROL_DROP;
    opts.reliability = qos.reliability == Qos::Reliability::BestEffort ? Z_RELIABILITY_BEST_EFFORT
                                                                       : Z_RELIABILITY_RELIABLE;
    if (z_declare_publisher(z_loan(node.session().impl().session), &impl_->publisher, z_loan(ke), &opts) < 0)
        throw std::runtime_error("zenoh: cannot declare publisher on " + key_);
    impl_->token = std::make_unique<LivelinessToken>(
        node.session(), entityToken(node, eid, "MP", topic_, dds_type, type_hash, qos));
}

PublisherBase::~PublisherBase()
{
    impl_->token.reset();
    z_drop(z_move(impl_->publisher));
}

bool PublisherBase::put(std::vector<uint8_t>&& cdr)
{
    uint8_t att[kAttachmentSize];
    {
        std::lock_guard<std::mutex> lock(impl_->seq_mutex);
        putLE64(att, ++impl_->seq);
    }
    putLE64(att + 8, nowNs());
    att[16] = 0x10;
    std::memcpy(att + 17, impl_->gid.data(), 16);

    // Hand the buffer to Zenoh without copying; it frees it when the sample is sent.
    auto* owned = new std::vector<uint8_t>(std::move(cdr));
    z_owned_bytes_t payload;
    if (z_bytes_from_buf(&payload, owned->data(), owned->size(),
                         [](void*, void* ctx) { delete static_cast<std::vector<uint8_t>*>(ctx); }, owned) < 0) {
        delete owned;
        return false;
    }
    z_owned_bytes_t attachment;
    z_bytes_copy_from_buf(&attachment, att, kAttachmentSize);
    z_publisher_put_options_t opts;
    z_publisher_put_options_default(&opts);
    opts.attachment = z_move(attachment);
    return z_publisher_put(z_loan(impl_->publisher), z_move(payload), &opts) >= 0;
}

const std::array<uint8_t, 16>& PublisherBase::gid() const
{
    return impl_->gid;
}

bool PublisherBase::hasSubscribers() const
{
    z_matching_status_t status{};
    if (z_publisher_get_matching_status(z_loan(impl_->publisher), &status) < 0) return false;
    return status.matching;
}

// ---------------------------------------------------------------------------------------------------------------
// Subscriber

namespace {
/// State the Zenoh callback reaches. Owned by the closure: Zenoh frees it once no callback can run any more, so
/// a callback in flight while the subscriber is destroyed never touches freed memory.
struct SubscriberContext {
    SubscriberBase::RawCallback cb;
    int64_t max_age_ns = 0;
};

void onSample(z_loaned_sample_t* sample, void* arg)
{
    auto* ctx = static_cast<SubscriberContext*>(arg);
    MessageInfo info;
    info.received_timestamp_ns = nowNs();

    const z_loaned_bytes_t* att = z_sample_attachment(sample);
    if (!att) return;   // rmw_zenoh drops these too
    {
        z_owned_slice_t s;
        if (z_bytes_to_slice(att, &s) < 0) return;
        const uint8_t* p = z_slice_data(z_loan(s));
        const bool ok = z_slice_len(z_loan(s)) == kAttachmentSize && p[16] == 0x10;
        if (ok) {
            info.sequence_number = getLE64(p);
            info.source_timestamp_ns = getLE64(p + 8);
            std::memcpy(info.publisher_gid.data(), p + 17, 16);
        }
        z_drop(z_move(s));
        if (!ok) return;
    }
    if (ctx->max_age_ns > 0 && info.received_timestamp_ns - info.source_timestamp_ns > ctx->max_age_ns) return;

    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    z_view_slice_t view;
    if (z_bytes_get_contiguous_view(payload, &view) >= 0) {
        ctx->cb(z_slice_data(z_loan(view)), z_slice_len(z_loan(view)), info);
        return;
    }
    z_owned_slice_t s;   // fragmented payload: one copy
    if (z_bytes_to_slice(payload, &s) < 0) return;
    ctx->cb(z_slice_data(z_loan(s)), z_slice_len(z_loan(s)), info);
    z_drop(z_move(s));
}
} // namespace

struct SubscriberBase::Impl {
    std::shared_ptr<Session> session;
    z_owned_subscriber_t subscriber;
    std::unique_ptr<LivelinessToken> token;
};

SubscriberBase::SubscriberBase(Node& node, const std::string& topic, const char* dds_type, const char* type_hash,
                               const Qos& qos, RawCallback cb)
    : impl_(std::make_unique<Impl>()), topic_(node.resolve(topic))
{
    impl_->session = node.sessionPtr();
    key_ = dataKey(node, topic_, dds_type, type_hash);
    const uint64_t eid = node.session().nextEntityId();

    z_view_keyexpr_t ke;
    if (z_view_keyexpr_from_str(&ke, key_.c_str()) < 0) throw std::runtime_error("zenoh: invalid key " + key_);
    auto* ctx = new SubscriberContext{std::move(cb), qos.max_age.count()};
    z_owned_closure_sample_t closure;
    z_closure_sample(&closure, onSample, [](void* c) { delete static_cast<SubscriberContext*>(c); }, ctx);
    if (z_declare_subscriber(z_loan(node.session().impl().session), &impl_->subscriber, z_loan(ke),
                             z_move(closure), nullptr) < 0)
        throw std::runtime_error("zenoh: cannot declare subscriber on " + key_);
    impl_->token = std::make_unique<LivelinessToken>(
        node.session(), entityToken(node, eid, "MS", topic_, dds_type, type_hash, qos));
}

SubscriberBase::~SubscriberBase()
{
    impl_->token.reset();
    z_drop(z_move(impl_->subscriber));
}

} // namespace rbq_sdk::zn
