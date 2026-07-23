#pragma once

#include <cstdlib>
#include <cstdio>
#include <fstream>
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

    static void writeCycloneConfig(const std::string& iface) {
        const char* existing = std::getenv("CYCLONEDDS_URI");
        if (existing && existing[0] != '\0') return;

        const char* tmpdir = std::getenv("TMPDIR");
        std::string dir   = tmpdir ? tmpdir : "/tmp";
        std::string path  = dir + "/rbq_sdk_cyclonedds_" + iface + ".xml";

        std::ofstream xml(path);
        xml << R"(<?xml version="1.0" encoding="utf-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name=")" << iface << R"(" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>spdp</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
)";
        std::string uri = "file://" + path;
        setenv("CYCLONEDDS_URI", uri.c_str(), 1);
    }

    std::shared_ptr<dds::domain::DomainParticipant> dp_;
    int         domain_id_ = 0;
    std::string interface_;
    std::mutex  mtx_;
};

} // namespace rbq_sdk
