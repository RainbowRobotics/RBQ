// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "glfw_adapter.h"

#include <cstdlib>
#include <utility>
#include <fstream>
#include <filesystem>
#include <string>
#include <iostream>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>
#include "glfw_dispatch.h"
#include <nlohmann/json.hpp>

#ifdef __APPLE__
#include "glfw_corevideo.h"
#endif

namespace mujoco
{
namespace
{
int MaybeGlfwInit()
{
    static const int is_initialized = []()
    {
        auto success = Glfw().glfwInit();
        if (success == GLFW_TRUE)
        {
            std::atexit(Glfw().glfwTerminate);
        }
        return success;
    }();
    return is_initialized;
}

GlfwAdapter &GlfwAdapterFromWindow(GLFWwindow *window)
{
    return *static_cast<GlfwAdapter *>(Glfw().glfwGetWindowUserPointer(window));
}
} // namespace

std::string GlfwAdapter::GetConfigPath()
{
    // Use XDG Base Directory Specification for Linux
    // $XDG_CONFIG_HOME (defaults to ~/.config) for configuration files
    std::string config_dir;
    
    // Check XDG_CONFIG_HOME environment variable
    const char* xdg_config_home = std::getenv("XDG_CONFIG_HOME");
    if (xdg_config_home && std::strlen(xdg_config_home) > 0) {
        config_dir = xdg_config_home;
    } else {
        // Default to ~/.config
        const char* home = std::getenv("HOME");
        if (home && std::strlen(home) > 0) {
            config_dir = std::string(home) + "/.config";
        } else {
            // Fallback to current directory if HOME is not set
            return "../configs/Mujoco/window_state.json";
        }
    }
    
    config_dir += "/RBQ/";
    
    return config_dir + "rbq_mujoco.json";
}

void GlfwAdapter::LoadWindowState(int& x, int& y, int& width, int& height) const
{
    std::string config_path = GetConfigPath();
    std::ifstream file(config_path);
    
    if (!file.is_open()) {
        // No saved state, use defaults
        return;
    }
    
    try {
        nlohmann::json j;
        file >> j;
        
        if (j.contains("window_x")) x = j["window_x"];
        if (j.contains("window_y")) y = j["window_y"];
        if (j.contains("window_width")) width = j["window_width"];
        if (j.contains("window_height")) height = j["window_height"];
    } catch (const std::exception& e) {
        // Invalid JSON, use defaults
        // Silently fail - this is not critical
    }
}

void GlfwAdapter::SaveWindowState() const
{
    if (!window_) {
        return;
    }
    
    int x, y, width, height;
    Glfw().glfwGetWindowPos(window_, &x, &y);
    Glfw().glfwGetWindowSize(window_, &width, &height);
    
    // Don't save if window is minimized or in invalid state
    if (width <= 0 || height <= 0) {
        return;
    }
    
    nlohmann::json j;
    j["window_x"] = x;
    j["window_y"] = y;
    j["window_width"] = width;
    j["window_height"] = height;
    
    std::string config_path = GetConfigPath();
    std::filesystem::path path(config_path);
    std::filesystem::create_directories(path.parent_path());
    
    std::ofstream file(config_path);
    if (file.is_open()) {
        file << j.dump(4);
    }
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
GlfwAdapter::GlfwAdapter()
{
    if (MaybeGlfwInit() != GLFW_TRUE)
    {
        mju_error("could not initialize GLFW");
    }

    // multisampling
    Glfw().glfwWindowHint(GLFW_SAMPLES, 4);
    Glfw().glfwWindowHint(GLFW_VISIBLE, 1);

    // get video mode and save
    vidmode_ = *Glfw().glfwGetVideoMode(Glfw().glfwGetPrimaryMonitor());

    // Load saved window state
    int window_x = 0, window_y = 0;
    int window_width = (2 * vidmode_.width) / 3;
    int window_height = (2 * vidmode_.height) / 3;
    bool loaded_state = false;
    
    // Try to load saved state
    {
        int saved_x = 0, saved_y = 0;
        int saved_width = window_width, saved_height = window_height;
        LoadWindowState(saved_x, saved_y, saved_width, saved_height);
        
        // Validate loaded dimensions
        if (saved_width > 0 && saved_width <= vidmode_.width &&
            saved_height > 0 && saved_height <= vidmode_.height) {
            window_width = saved_width;
            window_height = saved_height;
            loaded_state = true;
        }
        
        // Validate and use saved position
        if (saved_x >= 0 && saved_y >= 0) {
            window_x = saved_x;
            window_y = saved_y;
        }
    }

    // create window
    window_ = Glfw().glfwCreateWindow(window_width, window_height,
                                      "MuJoCo", nullptr, nullptr);
    if (!window_)
    {
        mju_error("could not create window");
    }
    
    // Set window position if we loaded valid state
    if (loaded_state) {
        Glfw().glfwSetWindowPos(window_, window_x, window_y);
    }

    // save window position and size
    Glfw().glfwGetWindowPos(window_, &window_pos_.first, &window_pos_.second);
    Glfw().glfwGetWindowSize(window_, &window_size_.first, &window_size_.second);

    // set callbacks
    Glfw().glfwSetWindowUserPointer(window_, this);
    Glfw().glfwSetDropCallback(
        window_, +[](GLFWwindow *window, int count, const char **paths)
        { GlfwAdapterFromWindow(window).OnFilesDrop(count, paths); });
    Glfw().glfwSetKeyCallback(
        window_, +[](GLFWwindow *window, int key, int scancode, int act, int mods)
        { GlfwAdapterFromWindow(window).OnKey(key, scancode, act); });
    Glfw().glfwSetMouseButtonCallback(
        window_, +[](GLFWwindow *window, int button, int act, int mods)
        { GlfwAdapterFromWindow(window).OnMouseButton(button, act); });
    Glfw().glfwSetCursorPosCallback(
        window_, +[](GLFWwindow *window, double x, double y)
        { GlfwAdapterFromWindow(window).OnMouseMove(x, y); });
    Glfw().glfwSetScrollCallback(
        window_, +[](GLFWwindow *window, double xoffset, double yoffset)
        { GlfwAdapterFromWindow(window).OnScroll(xoffset, yoffset); });
    Glfw().glfwSetWindowRefreshCallback(
        window_, +[](GLFWwindow *window)
        {
#ifdef __APPLE__
            auto& core_video = GlfwAdapterFromWindow(window).core_video_;
            if (core_video.has_value()) {
                core_video->UpdateDisplayLink();
            }
#endif
            GlfwAdapterFromWindow(window).OnWindowRefresh(); });
    Glfw().glfwSetWindowSizeCallback(
        window_, +[](GLFWwindow *window, int width, int height)
        { 
            GlfwAdapterFromWindow(window).OnWindowResize(width, height);
            GlfwAdapterFromWindow(window).SaveWindowState();
        });
    
    // Add window position callback to save state when window is moved
    Glfw().glfwSetWindowPosCallback(
        window_, +[](GLFWwindow *window, int x, int y)
        { GlfwAdapterFromWindow(window).SaveWindowState(); });

    // make context current
    Glfw().glfwMakeContextCurrent(window_);
}

GlfwAdapter::~GlfwAdapter()
{
    // Save window state before destroying
    SaveWindowState();
    
    FreeMjrContext();
    Glfw().glfwMakeContextCurrent(nullptr);
    Glfw().glfwDestroyWindow(window_);
}

std::pair<double, double> GlfwAdapter::GetCursorPosition() const
{
    double x, y;
    Glfw().glfwGetCursorPos(window_, &x, &y);
    return {x, y};
}

double GlfwAdapter::GetDisplayPixelsPerInch() const
{
    int width_mm, height_mm;
    Glfw().glfwGetMonitorPhysicalSize(
        Glfw().glfwGetPrimaryMonitor(), &width_mm, &height_mm);
    return 25.4 * vidmode_.width / width_mm;
}

std::pair<int, int> GlfwAdapter::GetFramebufferSize() const
{
    int width, height;
    Glfw().glfwGetFramebufferSize(window_, &width, &height);
    return {width, height};
}

std::pair<int, int> GlfwAdapter::GetWindowSize() const
{
    int width, height;
    Glfw().glfwGetWindowSize(window_, &width, &height);
    return {width, height};
}

bool GlfwAdapter::IsGPUAccelerated() const
{
    return true;
}

void GlfwAdapter::PollEvents()
{
    Glfw().glfwPollEvents();
}

void GlfwAdapter::SetClipboardString(const char *text)
{
    Glfw().glfwSetClipboardString(window_, text);
}

void GlfwAdapter::SetVSync(bool enabled)
{
#ifdef __APPLE__
    Glfw().glfwSwapInterval(0);
    if (enabled && !core_video_.has_value())
    {
        core_video_.emplace(window_);
    }
    else if (!enabled && core_video_.has_value())
    {
        core_video_.reset();
    }
#else
    Glfw().glfwSwapInterval(enabled);
#endif
}

void GlfwAdapter::SetWindowTitle(const char *title)
{
    Glfw().glfwSetWindowTitle(window_, title);
}

bool GlfwAdapter::ShouldCloseWindow() const
{
    return Glfw().glfwWindowShouldClose(window_);
}

void GlfwAdapter::SwapBuffers()
{
#ifdef __APPLE__
    if (core_video_.has_value())
    {
        core_video_->WaitForDisplayRefresh();
    }
#endif
    Glfw().glfwSwapBuffers(window_);
}

void GlfwAdapter::ToggleFullscreen()
{
    // currently full screen: switch to windowed
    if (Glfw().glfwGetWindowMonitor(window_))
    {
        // restore window from saved data
        Glfw().glfwSetWindowMonitor(window_, nullptr, window_pos_.first, window_pos_.second,
                                    window_size_.first, window_size_.second, 0);
    }

    // currently windowed: switch to full screen
    else
    {
        // save window data
        Glfw().glfwGetWindowPos(window_, &window_pos_.first, &window_pos_.second);
        Glfw().glfwGetWindowSize(window_, &window_size_.first,
                                 &window_size_.second);

        // switch
        Glfw().glfwSetWindowMonitor(window_, Glfw().glfwGetPrimaryMonitor(), 0,
                                    0, vidmode_.width, vidmode_.height,
                                    vidmode_.refreshRate);
    }
}

bool GlfwAdapter::IsLeftMouseButtonPressed() const
{
    return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
}

bool GlfwAdapter::IsMiddleMouseButtonPressed() const
{
    return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

bool GlfwAdapter::IsRightMouseButtonPressed() const
{
    return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
}

bool GlfwAdapter::IsAltKeyPressed() const
{
    return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
           Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
}

bool GlfwAdapter::IsCtrlKeyPressed() const
{
    return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
           Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

bool GlfwAdapter::IsShiftKeyPressed() const
{
    return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
           Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool GlfwAdapter::IsMouseButtonDownEvent(int act) const
{
    return act == GLFW_PRESS;
}

bool GlfwAdapter::IsKeyDownEvent(int act) const { return act == GLFW_PRESS; }

int GlfwAdapter::TranslateKeyCode(int key) const { return key; }

mjtButton GlfwAdapter::TranslateMouseButton(int button) const
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        return mjBUTTON_LEFT;
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        return mjBUTTON_RIGHT;
    }
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    {
        return mjBUTTON_MIDDLE;
    }
    return mjBUTTON_NONE;
}
} // namespace mujoco
