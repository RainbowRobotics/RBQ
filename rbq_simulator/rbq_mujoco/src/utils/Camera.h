#include <cstdio>
#include <mujoco/mujoco.h>

#include <opencv2/opencv.hpp>

class Camera {
public:

    /// @brief Compute fx/fy/cx/cy from FOV + viewport; recall on viewport resize.
    /// @param camera mujoco camera must be set up first
    void setIntrinsics(const mjModel* model, const mjvCamera camera, const mjrRect viewport) {
        // vertical FOV
        double fovy = model->cam_fovy[camera.fixedcamid] / 180 * M_PI / 2;

        // focal length, fx = fy
        fx = viewport.height / 2 / tan(fovy);
        fy = viewport.height / 2 / tan(fovy);

        // principal points
        cx = viewport.width / 2;
        cy = viewport.height / 2;
    }

    /// @brief Read GL color/depth buffers into m_color (BGR) and m_depth (mm).
    /// NOTE: call releaseBuffer() each loop to avoid leaking the malloc'd buffers.
    void getBuffer(const mjModel* model, const mjrRect viewport, const mjrContext* context) {
        // Use preallocated buffer to fetch color buffer and depth buffer in OpenGL
        color_buffer = (uchar*) malloc(viewport.height*viewport.width * 3);
        depth_buffer = (float*) malloc(viewport.height*viewport.width * 4);
        mjr_readPixels(color_buffer, depth_buffer, viewport, context);

        cv::Size size(viewport.width, viewport.height);
        cv::Mat rgb(size, CV_8UC3, color_buffer);
        cv::flip(rgb, rgb, 0);
        cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR); // Convert color format
        rgb.copyTo(m_color);

        cv::Mat depth_(size, CV_32F, depth_buffer);
        cv::flip(depth_, depth_, 0);
        linearizeDepth(depth_, model->vis.map.znear, model->vis.map.zfar, model->stat.extent).copyTo(m_depth);
    }

    /// @brief free memory at the end of loop
    inline void releaseBuffer() {
        free(color_buffer);
        free(depth_buffer);
    }

    /// @return color image (BGR)
    inline cv::Mat color() { return m_color; }
    /// @return depth image in millimeters
    inline cv::Mat depth() { return m_depth; }

    /// @return intrinsic by id: 0=fx 1=fy 2=cx 3=cy
    float intrinsics(const int &id) {
        switch (id) {
        case 0: return fx;
        case 1: return fy;
        case 2: return cx;
        case 3: return cy;
        default: return 0;
        }
    }

private:
    /// @brief Linearize GL depth buffer to mm; returns CV_16U [0~65535].
    cv::Mat linearizeDepth(const cv::Mat& depth, const double &z_near, const double &z_far, const double &extent) {
        cv::Mat depth_img(depth.size(), CV_16U, cv::Scalar(0));
        for (uint i = 0; i < depth_img.rows; i++) {
            auto* raw_depth_ptr = depth.ptr<float>(i);
            uint16_t* depth_ptr = depth_img.ptr<uint16_t>(i);
            for (uint j = 0; j < depth_img.cols; j++) {
                depth_ptr[j] = (z_near * z_far * extent / (z_far - raw_depth_ptr[j] * (z_far - z_near))) * 1000;
            }
        }
        return depth_img;
    }

private:
    uchar* color_buffer;
    float* depth_buffer;

    // camera intrinsics
    double fx, fy; // focal length
    int cx, cy; // principal points

    cv::Mat m_color;
    cv::Mat m_depth;
};
