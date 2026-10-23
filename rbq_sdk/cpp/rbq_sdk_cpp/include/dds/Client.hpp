#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <dds/dds.hpp>

#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/idl/api/Request_.hpp>
#include <rbq_sdk/idl/api/Response_.hpp>

namespace rbq_sdk {

// Unitree-compatible RPC error codes (mirror unitree_sdk2py.rpc.internal).
constexpr int32_t RPC_OK                       = 0;
constexpr int32_t RPC_ERR_UNKNOWN              = 3001;
constexpr int32_t RPC_ERR_CLIENT_SEND          = 3102;
constexpr int32_t RPC_ERR_CLIENT_API_NOT_REG   = 3103;
constexpr int32_t RPC_ERR_CLIENT_API_TIMEOUT   = 3104;
constexpr int32_t RPC_ERR_CLIENT_API_NOT_MATCH = 3105;

// IDs <= 100 are internal/reserved and skip the api map check.
constexpr int32_t RPC_INTERNAL_API_ID_MAX = 100;

// Service request/response channels are reliable; QoS must match the Unitree
// server, which writes Response_ with Reliable + KeepLast.
inline dds::pub::qos::DataWriterQos service_writer_qos() {
    dds::pub::qos::DataWriterQos qos;
    qos << dds::core::policy::Reliability::Reliable()
        << dds::core::policy::History::KeepLast(8)
        << dds::core::policy::ResourceLimits(32, 32, 8);
    return qos;
}
inline dds::sub::qos::DataReaderQos service_reader_qos() {
    dds::sub::qos::DataReaderQos qos;
    qos << dds::core::policy::Reliability::Reliable()
        << dds::core::policy::History::KeepLast(8)
        << dds::core::policy::ResourceLimits(32, 32, 8);
    return qos;
}

class Client {
public:
    using Request_  = ::api::msg::dds_::Request_;
    using Response_ = ::api::msg::dds_::Response_;

    explicit Client(const std::string& service_name, bool enable_lease = false)
        : service_name_(service_name), enable_lease_(enable_lease)
    {
        const std::string base = "rt/api/" + service_name_;
        pub_ = Publisher<Request_>(base + "/request", service_writer_qos());
        sub_ = Subscriber<Response_>(
            [this](const Response_& r) { OnResponse(r); },
            base + "/response",
            service_reader_qos());
    }

    virtual ~Client() = default;

    void SetTimeout(float seconds)         { timeout_us_ = static_cast<int64_t>(seconds * 1e6); }
    void SetTimeout(int64_t microseconds)  { timeout_us_ = microseconds; }

    // Toggle whether the no-data Call() overload waits for a response.
    // Default false = Unitree-compatible blocking behavior.
    // Set true to make every call without an output buffer fire-and-forget
    // — useful for streaming over WiFi where the 1 s RPC timeout would stall
    // the loop.  The with-data Call(..., data) overload always blocks.
    void SetNoReply(bool no_reply) { no_reply_default_ = no_reply; }
    bool NoReply() const           { return no_reply_default_; }

    const std::string& GetApiVersion() const { return api_version_; }

    // Blocking request/response.  Returns response.header.status.code on success,
    // or one of RPC_ERR_* on local error / timeout.  Always blocks regardless
    // of the SetNoReply flag — the caller needs the response payload.
    int32_t Call(int32_t api_id, const std::string& parameter, std::string& data) {
        return CallBlocking(api_id, parameter, data);
    }

    // Honors the SetNoReply flag: fire-and-forget when true, otherwise the
    // same as the with-data overload with the response discarded.
    int32_t Call(int32_t api_id, const std::string& parameter) {
        if (no_reply_default_) return CallNoReply(api_id, parameter);
        std::string discarded;
        return CallBlocking(api_id, parameter, discarded);
    }

    // Fire-and-forget — sets policy.noreply = true; returns RPC_OK on successful write.
    int32_t CallNoReply(int32_t api_id, const std::string& parameter) {
        int32_t priority = 0;
        if (!CheckApi(api_id, priority)) return RPC_ERR_CLIENT_API_NOT_REG;

        Request_ req{};
        FillHeader(req, NextId(), api_id, priority, /*no_reply=*/true);
        req.parameter(parameter);
        return pub_.write(req) ? RPC_OK : RPC_ERR_CLIENT_SEND;
    }

protected:
    void SetApiVersion(const std::string& v) { api_version_ = v; }

    void RegistApi(int32_t api_id, int32_t priority = 0) {
        std::lock_guard<std::mutex> lk(api_mx_);
        api_map_[api_id] = priority;
    }

private:
    struct Pending {
        std::condition_variable cv;
        bool ready{false};
        Response_ response;
    };

    int32_t CallBlocking(int32_t api_id, const std::string& parameter, std::string& data) {
        int32_t priority = 0;
        if (!CheckApi(api_id, priority)) return RPC_ERR_CLIENT_API_NOT_REG;

        const int64_t id = NextId();
        auto pending = std::make_shared<Pending>();
        {
            std::lock_guard<std::mutex> lk(pending_mx_);
            pending_.emplace(id, pending);
        }

        Request_ req{};
        FillHeader(req, id, api_id, priority, /*no_reply=*/false);
        req.parameter(parameter);

        if (!pub_.write(req)) {
            std::lock_guard<std::mutex> lk(pending_mx_);
            pending_.erase(id);
            return RPC_ERR_CLIENT_SEND;
        }

        std::unique_lock<std::mutex> lk(pending_mx_);
        const bool got = pending->cv.wait_for(
            lk, std::chrono::microseconds(timeout_us_.load()),
            [&] { return pending->ready; });
        pending_.erase(id);
        if (!got) return RPC_ERR_CLIENT_API_TIMEOUT;

        if (pending->response.header().identity().api_id() != api_id)
            return RPC_ERR_CLIENT_API_NOT_MATCH;

        data = pending->response.data();
        return static_cast<int32_t>(pending->response.header().status().code());
    }

    bool CheckApi(int32_t api_id, int32_t& priority) {
        if (api_id <= RPC_INTERNAL_API_ID_MAX) {
            priority = 0;
            return true;
        }
        std::lock_guard<std::mutex> lk(api_mx_);
        auto it = api_map_.find(api_id);
        if (it == api_map_.end()) return false;
        priority = it->second;
        return true;
    }

    void OnResponse(const Response_& r) {
        const int64_t id = r.header().identity().id();
        std::lock_guard<std::mutex> lk(pending_mx_);
        auto it = pending_.find(id);
        if (it == pending_.end()) return;
        auto p = it->second;
        p->response = r;
        p->ready = true;
        p->cv.notify_all();
    }

    static int64_t NextId() {
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }

    static void FillHeader(Request_& req, int64_t id, int32_t api_id,
                           int32_t priority, bool no_reply) {
        req.header().identity().id()       = id;
        req.header().identity().api_id()   = api_id;
        req.header().lease().id()          = 0;
        req.header().policy().priority()   = priority;
        req.header().policy().noreply()    = no_reply;
    }

    std::string service_name_;
    bool        enable_lease_;
    std::string api_version_;

    std::mutex api_mx_;
    std::unordered_map<int32_t, int32_t> api_map_;

    std::mutex pending_mx_;
    std::unordered_map<int64_t, std::shared_ptr<Pending>> pending_;

    std::atomic<int64_t> timeout_us_{1'000'000};  // 1 s, matches Unitree default
    std::atomic<bool>    no_reply_default_{false};  // SetNoReply: false = Unitree-compat blocking

    // Declared last so they are destroyed first — stops DDS callbacks before
    // pending_/pending_mx_ go away.
    Publisher<Request_>   pub_;
    Subscriber<Response_> sub_;
};

} // namespace rbq_sdk
