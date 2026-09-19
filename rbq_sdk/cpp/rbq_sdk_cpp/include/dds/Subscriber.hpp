#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <dds/dds.hpp>

#include "ChannelFactory.hpp"
#include "Settings.hpp"

namespace rbq_sdk {

template <typename T>
class Subscriber {
    struct Listener : dds::sub::DataReaderListener<T> {
        T* target_{nullptr};
        std::function<void(const T&)> on_sample_;

        Listener(T* t, std::function<void(const T&)> cb)
            : target_(t), on_sample_(std::move(cb)) {}

        void on_data_available(dds::sub::DataReader<T>& r) override {
            auto samples = r.take();
            for (const auto& s : samples) {
                if (!s.info().valid()) continue;
                const auto& m = s.data();
                if (target_) *target_ = m;
                if (on_sample_) on_sample_(m);
            }
        }

        void on_requested_deadline_missed(dds::sub::DataReader<T>&,
                                          const dds::core::status::RequestedDeadlineMissedStatus&) override {}
        void on_requested_incompatible_qos(dds::sub::DataReader<T>&,
                                           const dds::core::status::RequestedIncompatibleQosStatus&) override {}
        void on_sample_rejected(dds::sub::DataReader<T>&,
                                const dds::core::status::SampleRejectedStatus&) override {}
        void on_liveliness_changed(dds::sub::DataReader<T>&,
                                   const dds::core::status::LivelinessChangedStatus&) override {}
        void on_subscription_matched(dds::sub::DataReader<T>&,
                                     const dds::core::status::SubscriptionMatchedStatus&) override {}
        void on_sample_lost(dds::sub::DataReader<T>&,
                            const dds::core::status::SampleLostStatus&) override {}
    };

public:
    Subscriber() = default;

    Subscriber(T* target_ptr,
               const std::string& topic_name,
               dds::sub::qos::DataReaderQos reader_qos = default_reader_qos(),
               std::function<void(const T&)> on_sample = {})
    {
        build(target_ptr, std::move(on_sample), topic_name, std::move(reader_qos));
    }

    Subscriber(std::function<void(const T&)> on_sample,
               const std::string& topic_name,
               dds::sub::qos::DataReaderQos reader_qos = default_reader_qos())
    {
        build(nullptr, std::move(on_sample), topic_name, std::move(reader_qos));
    }

    ~Subscriber() = default;

    void set_target(T* target_ptr) { if (listener_) listener_->target_ = target_ptr; }

    void close() {
        reader_.reset();
        listener_.reset();
        topic_.reset();
        sub_.reset();
    }

    dds::sub::Subscriber&    subscriber() { return *sub_; }
    dds::topic::Topic<T>&    topic()      { return *topic_; }
    dds::sub::DataReader<T>& reader()     { return *reader_; }

private:
    void build(T* target_ptr,
               std::function<void(const T&)> on_sample,
               const std::string& topic_name,
               dds::sub::qos::DataReaderQos reader_qos)
    {
        auto& dp = ChannelFactory::Instance().participant();
        sub_      = std::make_shared<dds::sub::Subscriber>(dp);
        topic_    = std::make_shared<dds::topic::Topic<T>>(dp, topic_name);
        listener_ = std::make_shared<Listener>(target_ptr, std::move(on_sample));
        reader_   = std::make_shared<dds::sub::DataReader<T>>(
            *sub_, *topic_, reader_qos, listener_.get(),
            dds::core::status::StatusMask::data_available());
    }

    std::shared_ptr<dds::sub::Subscriber>    sub_;
    std::shared_ptr<dds::topic::Topic<T>>    topic_;
    std::shared_ptr<Listener>                listener_;
    std::shared_ptr<dds::sub::DataReader<T>> reader_;
};

} // namespace rbq_sdk
