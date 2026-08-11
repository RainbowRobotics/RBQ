#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/idl/ros2/CompressedImage_.hpp>

static std::atomic<bool> g_running{true};

int main(int argc, char** argv)
{
    const std::string topic =
        (argc > 1) ? argv[1] : "rt/rbq/vision/sensor_4/rgb/compressed";

    std::signal(SIGINT,  [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });

    rbq_sdk::ChannelFactory::Instance().Init(0, "enp45s0");

    std::atomic<uint64_t> count{0};
    std::atomic<size_t>   last_bytes{0};
    std::string           last_format;
    std::mutex            fmt_mu;

    rbq_sdk::Subscriber<sensor_msgs::msg::dds_::CompressedImage_> sub(
        [&](const sensor_msgs::msg::dds_::CompressedImage_& msg) {
            count.fetch_add(1, std::memory_order_relaxed);
            last_bytes.store(msg.data().size(), std::memory_order_relaxed);
            std::lock_guard<std::mutex> lk(fmt_mu);
            last_format = msg.format();
        },
        topic);

    std::printf("[rbq_image_echo] subscribed to '%s'\n", topic.c_str());
    std::printf("[rbq_image_echo] waiting for samples (Ctrl+C to exit)...\n");

    uint64_t prev = 0;
    auto     t0   = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const uint64_t now_count = count.load();
        const uint64_t delta     = now_count - prev;
        prev                     = now_count;

        std::string fmt_copy;
        {
            std::lock_guard<std::mutex> lk(fmt_mu);
            fmt_copy = last_format;
        }
        std::printf("[rbq_image_echo] %.1fs  total=%lu  rate=%lu Hz  last=%zu bytes  fmt='%s'\n",
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count(),
                    static_cast<unsigned long>(now_count),
                    static_cast<unsigned long>(delta),
                    last_bytes.load(),
                    fmt_copy.c_str());
    }

    std::printf("[rbq_image_echo] exiting, total samples = %lu\n",
                static_cast<unsigned long>(count.load()));
    return 0;
}
