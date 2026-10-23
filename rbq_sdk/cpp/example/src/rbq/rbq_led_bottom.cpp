#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <rbq_sdk/dds/ChannelFactory.hpp>
#include <rbq_sdk/dds/Publisher.hpp>
#include <rbq_sdk/dds/Subscriber.hpp>
#include <rbq_sdk/idl/rbq/LedBottom_.hpp>
#include <rbq_sdk/idl/rbq/LedBottomState_.hpp>

// side / mode constants (mirror rbq_msgs/LedBottom.msg)
static constexpr int SIDE_RIGHT = 1, SIDE_LEFT = 2, SIDE_BOTH = 3;
static constexpr int MODE_OFF = 0, MODE_ON = 1, MODE_BLINK = 2;

static void printUsage()
{
    std::cout <<
        "Usage: rbq_led_bottom <interface> [side] [mode] [r] [g] [b] [on_ms] [off_ms] [count]\n"
        "\n"
        "  interface     CycloneDDS interface, e.g. lo (on the robot) or eth0 (another host)\n"
        "  side          right / left / both (the other LED keeps its setting)   (default both)\n"
        "  mode          off / on / blink                                       (default on)\n"
        "  r g b         colour 0-255                                           (default 0 0 255)\n"
        "  on_ms off_ms  blink on / off time 10-2550 ms (10 ms steps)           (default 500 500)\n"
        "  count         blink repeats, 0 = forever                             (default 0)\n"
        "\n"
        "Example (right LED blinking red, 0.5 s on / 0.5 s off, forever):\n"
        "  rbq_led_bottom eth0 right blink 255 0 0 500 500 0\n";
}

static int sideOf(const std::string &s)
{
    return s == "right" ? SIDE_RIGHT : s == "left" ? SIDE_LEFT : s == "both" ? SIDE_BOTH : -1;
}

static int modeOf(const std::string &s)
{
    return s == "off" ? MODE_OFF : s == "on" ? MODE_ON : s == "blink" ? MODE_BLINK : -1;
}

static void printSide(const char *name, const rbq_msgs::msg::dds_::LedBottomSide_ &s)
{
    static const char *const kMode[] = {"off", "on", "blink"};
    std::cout << "  " << name << ": mode=" << (s.mode() < 3 ? kMode[s.mode()] : "?")
              << " rgb=" << int(s.rgb()[0]) << "," << int(s.rgb()[1]) << "," << int(s.rgb()[2])
              << " on_ms=" << s.on_ms() << " off_ms=" << s.off_ms() << " count=" << int(s.count())
              << " actual=" << int(s.actual()[0]) << "," << int(s.actual()[1]) << "," << int(s.actual()[2])
              << (s.limited() ? " limited" : "") << (s.legacy() ? " legacy" : "")
              << " lpf_w=" << s.lpf_w() << "\n";
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printUsage();
        return 1;
    }

    const std::string interface = argv[1];
    const std::string sideName  = (argc > 2) ? argv[2] : "both";
    const std::string modeName  = (argc > 3) ? argv[3] : "on";
    const int side   = sideOf(sideName);
    const int mode   = modeOf(modeName);
    const int r      = (argc > 4) ? std::stoi(argv[4]) : 0;
    const int g      = (argc > 5) ? std::stoi(argv[5]) : 0;
    const int b      = (argc > 6) ? std::stoi(argv[6]) : 255;
    const int on_ms  = (argc > 7) ? std::stoi(argv[7]) : 500;
    const int off_ms = (argc > 8) ? std::stoi(argv[8]) : 500;
    const int count  = (argc > 9) ? std::stoi(argv[9]) : 0;
    if (side < 0 || mode < 0) {
        printUsage();
        return 1;
    }

    rbq_sdk::ChannelFactory::Instance().Init(/*domainId=*/0, interface);

    rbq_sdk::Publisher<rbq_msgs::msg::dds_::LedBottom_> pub("rt/rbq/cmd/led_bottom");

    // the robot reports the LED state once a second — keep the latest
    std::mutex mx;
    bool got = false;
    rbq_msgs::msg::dds_::LedBottomState_ latest;
    rbq_sdk::Subscriber<rbq_msgs::msg::dds_::LedBottomState_> sub(
        [&](const rbq_msgs::msg::dds_::LedBottomState_ &s) {
            std::lock_guard<std::mutex> lock(mx);
            latest = s;
            got = true;
        }, "rt/rbq/led_bottom");

    // wait for DDS discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rbq_msgs::msg::dds_::LedBottom_ msg;
    msg.side(static_cast<uint8_t>(side));
    msg.mode(static_cast<uint8_t>(mode));
    msg.rgb()[0] = static_cast<uint8_t>(r);
    msg.rgb()[1] = static_cast<uint8_t>(g);
    msg.rgb()[2] = static_cast<uint8_t>(b);
    msg.on_ms(static_cast<uint16_t>(on_ms));
    msg.off_ms(static_cast<uint16_t>(off_ms));
    msg.count(static_cast<uint8_t>(count));

    pub.write(msg);

    std::cout << "[rbq_led_bottom] published:"
              << " side=" << sideName
              << " mode=" << modeName
              << " rgb=" << r << "," << g << "," << b
              << " on_ms=" << on_ms
              << " off_ms=" << off_ms
              << " count=" << count
              << std::endl;

    // the IF board applies the setting and reports once a second; Network relays it at 1 Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    std::lock_guard<std::mutex> lock(mx);
    if (!got) {
        std::cout << "[rbq_led_bottom] no state on rt/rbq/led_bottom (Network not running, or not a CAN FD robot)"
                  << std::endl;
        return 0;
    }
    std::cout << "[rbq_led_bottom] state:"
              << " age_ms=" << latest.age_ms()
              << " configured=" << latest.configured()
              << " if_fw=" << latest.if_fw()
              << "\n";
    printSide("right", latest.right());
    printSide("left",  latest.left());

    return 0;
}
