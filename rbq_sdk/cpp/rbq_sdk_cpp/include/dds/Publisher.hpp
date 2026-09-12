#pragma once

#include <memory>
#include <string>

#include <dds/dds.hpp>

#include "ChannelFactory.hpp"
#include "Settings.hpp"

namespace rbq_sdk {

template <typename T>
class Publisher {
public:
    Publisher() = default;

    explicit Publisher(const std::string& topic_name,
                       dds::pub::qos::DataWriterQos writer_qos = default_writer_qos())
    {
        auto& dp = ChannelFactory::Instance().participant();
        pub_    = std::make_shared<dds::pub::Publisher>(dp);
        topic_  = std::make_shared<dds::topic::Topic<T>>(dp, topic_name);
        writer_ = std::make_shared<dds::pub::DataWriter<T>>(*pub_, *topic_, writer_qos);
    }

    ~Publisher() = default;

    bool write(const T& msg) {
        if (!writer_) return false;
        writer_->write(msg);
        return true;
    }

    void close() {
        writer_.reset();
        topic_.reset();
        pub_.reset();
    }

    bool hasSubscribers() {
        try {
            return writer_->publication_matched_status().current_count() > 0;
        } catch (...) {
            return false;
        }
    }

    dds::pub::Publisher&      publisher() { return *pub_; }
    dds::topic::Topic<T>&     topic()     { return *topic_; }
    dds::pub::DataWriter<T>&  writer()    { return *writer_; }

private:
    std::shared_ptr<dds::pub::Publisher>     pub_;
    std::shared_ptr<dds::topic::Topic<T>>    topic_;
    std::shared_ptr<dds::pub::DataWriter<T>> writer_;
};

} // namespace rbq_sdk
