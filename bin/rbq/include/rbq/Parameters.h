#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>
#include <array>

#define RT_MS       2

class Parameters {

public:
    Parameters();

    static int Initialize(const std::string &filename);

    static constexpr float gravity = 9.81;
    static constexpr float dt_ = RT_MS/1000.0f;
    static constexpr int freq_ = (int)1000/RT_MS;

    static float mu;
    static float body_mass;
    static float hip_mass;
    static float thigh_mass;
    static float calf_mass;
    static float total_mass;
    static std::array<float, 3> body_com;
    static std::array<float, 3> hip_com;
    static std::array<float, 3> thigh_com;
    static std::array<float, 3> calf_com;
    static float I_rotor;
    static float I_rotor_knee;
    static float body_Ixx;
    static float body_Iyy;
    static float body_Izz;
    static float body_Ixy;
    static float body_Iyz;
    static float body_Izx;
    static float hip_Ixx;
    static float hip_Iyy;
    static float hip_Izz;
    static float hip_Ixy;
    static float hip_Iyz;
    static float hip_Izx;
    static float thigh_Ixx;
    static float thigh_Iyy;
    static float thigh_Izz;
    static float thigh_Ixy;
    static float thigh_Iyz;
    static float thigh_Izx;
    static float calf_Ixx;
    static float calf_Iyy;
    static float calf_Izz;
    static float calf_Ixy;
    static float calf_Iyz;
    static float calf_Izx;
    static std::array<float, 3> bd2imu;
    static std::array<float, 3> bd2hip;
    static float roll2pitch;
    static float foot_radius;
    static float thigh_length;
    static float calf_length;
    static float init_angle;
    static float default_height;
    static std::array<float, 3> gear_ratio;
    static float torque_constant;
    static float torque_constant_knee;
    static float torque_lim;
    static float torque_lim_knee;
    static std::array<float, 12> joint_upper_lim;
    static std::array<float, 12> joint_lower_lim;
};

#endif // PARAMETERS_H
