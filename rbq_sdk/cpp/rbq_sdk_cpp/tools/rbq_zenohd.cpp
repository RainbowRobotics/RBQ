// rbq_zenohd — the Zenoh router of one host. Every rbq_sdk session (and every ROS 2 node on rmw_zenoh) connects to
// tcp/localhost:7447 by default and reaches the others through it; routers on different hosts connect to each
// other with --connect. Same role and default port as ROS 2's rmw_zenohd, so a host runs one or the other.
//
//   rbq_zenohd [--listen tcp/[::]:7447] [--connect tcp/<other-host>:7447]...
// ROS_DOMAIN_ID, ZENOH_CONFIG_OVERRIDE apply as for any session; ZENOH_ROUTER_CONFIG_URI replaces the defaults.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#include "zn/Session.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
}

int main(int argc, char** argv)
{
    std::string listen = "tcp/[::]:7447";
    std::vector<std::string> connect;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--listen") && i + 1 < argc) listen = argv[++i];
        else if (!std::strcmp(argv[i], "--connect") && i + 1 < argc) connect.push_back(argv[++i]);
        else {
            std::cerr << "usage: rbq_zenohd [--listen <endpoint>] [--connect <endpoint>]..." << std::endl;
            return 2;
        }
    }

    auto cfg = rbq_sdk::zn::SessionConfig::router(listen);
    cfg.connect = connect;
    if (const char* f = std::getenv("ZENOH_ROUTER_CONFIG_URI")) cfg.config_file = f;

    std::signal(SIGINT, [](int) { g_stop = 1; });
    std::signal(SIGTERM, [](int) { g_stop = 1; });

    try {
        auto session = rbq_sdk::zn::Session::open(cfg);
        std::cout << "rbq_zenohd " << session->zid() << " listening on " << listen << std::endl;
        while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } catch (const std::exception& e) {
        std::cerr << "rbq_zenohd: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
