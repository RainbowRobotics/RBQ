#pragma once

#include <cmath>
#include <cerrno>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>


namespace
{

constexpr int kAxisCount = 8;
constexpr int kBtnCount = 16;

constexpr int AXIS_LEFT_X = 0;
constexpr int AXIS_LEFT_Y = 1;
constexpr int AXIS_LT = 2;
constexpr int AXIS_RIGHT_X = 3;
constexpr int AXIS_RT = 5;

constexpr int PAD_A = 0;
constexpr int PAD_B = 1;
constexpr int PAD_X = 2;
constexpr int PAD_Y = 3;
constexpr int PAD_LB = 4;
constexpr int PAD_RB = 5;
constexpr int PAD_BACK = 6;
constexpr int PAD_START = 7;

static float Deadzone(float v, float dz)
{
    if (std::fabs(v) < dz)
        return 0.f;
    const float s = v > 0.f ? 1.f : -1.f;
    return s * (std::fabs(v) - dz) / (1.f - dz);
}

static std::vector<std::string> ListJsNodes()
{
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory("/dev/input", ec))
        return out;
    for (const auto &e : fs::directory_iterator("/dev/input", ec))
    {
        if (ec)
            break;
        const std::string name = e.path().filename().string();
        if (name.size() >= 2 && name.compare(0, 2, "js") == 0)
            out.push_back(e.path().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

static bool JoyNameLooksLikePad(const char *name)
{
    std::string n(name);
    for (char &c : n)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return n.find("logitech") != std::string::npos || n.find("gamepad") != std::string::npos ||
           n.find("x-box") != std::string::npos || n.find("xbox") != std::string::npos ||
           n.find("wireless") != std::string::npos;
}

static std::string TryOpenJoy(const std::string &p)
{
    const int fd = ::open(p.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
        return {};
    ::close(fd);
    return p;
}

static std::string PickJoyPath(int argc, char **argv)
{
    if (argc >= 3)
        return TryOpenJoy(argv[2]);

    if (const char *e = std::getenv("JOY"))
    {
        const std::string r = TryOpenJoy(e);
        if (!r.empty())
            return r;
    }

    std::string fallback;
    for (const auto &p : ListJsNodes())
    {
        const int fd = ::open(p.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            continue;
        char name[128] = {};
        (void)::ioctl(fd, JSIOCGNAME(sizeof(name)), name);
        ::close(fd);
        if (JoyNameLooksLikePad(name))
            return p;
        if (fallback.empty())
            fallback = p;
    }
    if (!fallback.empty())
        return fallback;
    return TryOpenJoy("/dev/input/js0");
}

static void PrintNoJoystickHint()
{
    std::cerr << "Joystick not detected on /dev/input/js*. Check if USB dongle is connected and joystick is on.\n";
    for (const auto &p : ListJsNodes())
        std::cerr << "  " << p << '\n';
}

class Joystick
{
public:
    explicit Joystick(std::string path) : path_(std::move(path)), fd_(-1) {}

    ~Joystick()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    bool Open()
    {
        fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0)
        {
            std::cerr << "조이스틱 열기 실패: " << path_ << '\n';
            return false;
        }
        char name[128] = {};
        if (::ioctl(fd_, JSIOCGNAME(sizeof(name)), name) >= 0)
            std::cout << "조이스틱: " << name << " (" << path_ << ")\n";
        return true;
    }

    void Poll()
    {
        struct js_event ev;
        for (;;)
        {
            const ssize_t n = ::read(fd_, &ev, sizeof(ev));
            if (n != static_cast<ssize_t>(sizeof(ev)))
            {
                if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
                    std::cerr << "조이 read 오류\n";
                break;
            }
            const bool is_init = (ev.type & JS_EVENT_INIT) != 0;
            const uint8_t k = static_cast<uint8_t>(ev.type & ~JS_EVENT_INIT);
            if (k == JS_EVENT_AXIS && ev.number < kAxisCount)
            {
                float v = static_cast<float>(ev.value) / 32767.f;
                if (v > 1.f)
                    v = 1.f;
                if (v < -1.f)
                    v = -1.f;
                ax_[ev.number] = v;
            }
            else if (k == JS_EVENT_BUTTON && ev.number < kBtnCount)
            {
                const bool on = ev.value != 0;
                btn_[ev.number] = on;
                // INIT 동기화 이벤트에서는 edge 금지 (첫 프레임에 댐프/정지 등 오동작 방지)
                if (on && !prev_[ev.number] && !is_init)
                    edge_[ev.number] = true;
            }
        }
    }

    void EndFrame()
    {
        for (int i = 0; i < kBtnCount; ++i)
        {
            prev_[i] = btn_[i];
            edge_[i] = false;
        }
    }

    float Ax(int i) const { return (i >= 0 && i < kAxisCount) ? ax_[i] : 0.f; }
    bool Btn(int i) const { return (i >= 0 && i < kBtnCount) ? btn_[i] : false; }
    bool Edge(int i) const { return (i >= 0 && i < kBtnCount) ? edge_[i] : false; }

private:
    std::string path_;
    int fd_;
    float ax_[kAxisCount] = {};
    bool btn_[kBtnCount] = {};
    bool prev_[kBtnCount] = {};
    bool edge_[kBtnCount] = {};
};

} // namespace
