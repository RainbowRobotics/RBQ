#pragma once

#include <cstdlib>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include <dds/dds.hpp>

namespace rbq_sdk {

class ChannelFactory {
public:
    static ChannelFactory& Instance() {
        static ChannelFactory s;
        return s;
    }

    void Init(int domainId = 0, const std::string& networkInterface = "lo") {
        std::lock_guard<std::mutex> lock(mtx_);
        if (dp_) return;

        if (!networkInterface.empty()) {
            writeCycloneConfig(networkInterface);
        }
        dp_ = std::make_shared<dds::domain::DomainParticipant>(domainId);
        domain_id_ = domainId;
        interface_ = networkInterface;
    }

    void Release() {
        std::lock_guard<std::mutex> lock(mtx_);
        dp_.reset();
    }

    dds::domain::DomainParticipant& participant() {
        if (!dp_) Init();
        return *dp_;
    }

    bool initialized() const { return static_cast<bool>(dp_); }
    int  domainId()   const { return domain_id_; }
    const std::string& networkInterface() const { return interface_; }

    ChannelFactory(const ChannelFactory&) = delete;
    ChannelFactory& operator=(const ChannelFactory&) = delete;

private:
    ChannelFactory() = default;
    ~ChannelFactory() = default;

    // Inline XML in CYCLONEDDS_URI
    static void writeCycloneConfig(const std::string& iface) {
        const char* existing = std::getenv("CYCLONEDDS_URI");
        if (existing && existing[0] != '\0') return;

        // lo can't multicast and Cyclone turns it off there anyway - say so up front (as the Python SDK does, channel.py),
        // or Cyclone 0.10 prints "selected interface "lo" is not multicast-capable: disabling multicast" on every start.
        // Without multicast the processes find each other by unicast to the fixed ports of localhost (participant index
        // auto) - give that peer too: multicast off in the config alone left every process unfound (sim, 11-18).
        const bool lo = iface == "lo";
        const std::string xml =
            "<CycloneDDS><Domain Id=\"any\">"
            "<General>"
            "<Interfaces><NetworkInterface name=\"" + iface + "\" priority=\"default\" multicast=\""
            + (lo ? "false" : "default") + "\"/></Interfaces>"
            "<AllowMulticast>" + (lo ? "false" : "spdp") + "</AllowMulticast>"
            "</General>"
            // Participant index budget is per host+domain — the robot runs ~11 DDS processes plus restart churn
            "<Discovery>"
            + std::string(lo ? "<ParticipantIndex>auto</ParticipantIndex><Peers><Peer address=\"localhost\"/></Peers>" : "")
            + "<MaxAutoParticipantIndex>60</MaxAutoParticipantIndex></Discovery>"
            "</Domain></CycloneDDS>";
        setenv("CYCLONEDDS_URI", xml.c_str(), 1);
    }

    std::shared_ptr<dds::domain::DomainParticipant> dp_;
    int         domain_id_ = 0;
    std::string interface_;
    std::mutex  mtx_;
};

} // namespace rbq_sdk
