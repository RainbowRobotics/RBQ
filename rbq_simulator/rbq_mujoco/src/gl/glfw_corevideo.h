#pragma once

#ifndef __APPLE__
#error "This header only works on macOS."
#endif

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "glfw_dispatch.h"

#ifdef __OBJC__
#import <CoreVideo/CoreVideo.h>
#else
typedef void *CVDisplayLinkRef;
#endif

// Workaround for perpertually broken OpenGL VSync on macOS,
// most recently https://github.com/glfw/glfw/issues/2249.
namespace mujoco
{
class GlfwCoreVideo
{
public:
    GlfwCoreVideo(GLFWwindow *window);
    ~GlfwCoreVideo();

    void WaitForDisplayRefresh();
    int DisplayLinkCallback();
    void UpdateDisplayLink();

private:
    GLFWwindow *window_;
    CVDisplayLinkRef display_link_;

    std::atomic_bool waiting_;
    std::mutex mu_;
    std::condition_variable cond_;
};
} // namespace mujoco

