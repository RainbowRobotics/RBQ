#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>

class Kinematics {
public:

    static Eigen::VectorXf PelvIK(const Eigen::Vector3f W_body_pos,
                                  const Eigen::Matrix3f W_body_rot,
                                  const Eigen::Vector3f W_HR_leg_pos,
                                  const Eigen::Vector3f W_HL_leg_pos,
                                  const Eigen::Vector3f W_FR_leg_pos,
                                  const Eigen::Vector3f W_FL_leg_pos);

    static Eigen::VectorXf PelvIK(const Eigen::Vector3f W_body_pos,
                                  const Eigen::Vector4f W_body_quat,
                                  const Eigen::Vector3f W_HR_leg_pos,
                                  const Eigen::Vector3f W_HL_leg_pos,
                                  const Eigen::Vector3f W_FR_leg_pos,
                                  const Eigen::Vector3f W_FL_leg_pos);

    static int R_or_L(int lnum){
        return (2 * (lnum % 2) - 1); // HR, FR = -1    HL, FL = +1
    }
    static int F_or_H(int lnum){
        return (2*(lnum > 1) - 1); // FR, FL = +1    HR, HL = -1
    }

    static int Side_Leg(int lnum){
        if(lnum == 0) return 1;
        else if(lnum == 1) return 0;
        else if(lnum == 2) return 3;
        else if(lnum == 3) return 2;
        else return lnum;
    }

    static Eigen::Matrix3f getJacobian(const int lnum, // 0:HR 1:HL 2:FR 3:FL
                                const Eigen::Vector3f &joint_ang);

    static Eigen::Matrix3f getJacobian(const int lnum, // 0:HR 1:HL 2:FR 3:FL
                                const Eigen::Matrix3f &body_rot,
                                const Eigen::Vector3f &joint_ang);

    static Eigen::Vector3f getFootPosition(int lnum,
                                    const Eigen::Vector3f &joint_ang);
    static Eigen::Vector3f getFootPosition2(int lnum,
                                            const Eigen::Vector3f &joint_ang);
    static Eigen::Vector3f getFootPosition(int lnum,
                                    const Eigen::Matrix3f &body_rot,
                                    const Eigen::Vector3f &joint_ang);
    static Eigen::Vector3f getFootPosition2(int lnum,
                                    const Eigen::Matrix3f &body_rot,
                                    const Eigen::Vector3f &joint_ang);

    static Eigen::Vector3f getHipPosition(int lnum);

    static Eigen::Vector3f getHipPosition(int lnum,
                                          const Eigen::Matrix3f &body_rot);

    static Eigen::Vector3f getShdPosition(int lnum,
                                          const Eigen::Vector3f &joint_ang);
    static Eigen::Vector3f getShdPosition(int lnum,
                                          const Eigen::Matrix3f &body_rot,
                                          const Eigen::Vector3f &joint_ang);

    static Eigen::Vector3f getElbPosition(int lnum,
                                          const Eigen::Vector3f &joint_ang);
    static Eigen::Vector3f getElbPosition(int lnum,
                                          const Eigen::Matrix3f &body_rot,
                                          const Eigen::Vector3f &joint_ang);

    static Eigen::Vector3f getFootVelocity(int lnum,
                                    const Eigen::Vector3f &joint_ang,
                                    const Eigen::Vector3f &joint_speed);
    static Eigen::Vector3f getFootVelocity(int lnum,
                                    const Eigen::Matrix3f &body_rot,
                                    const Eigen::Vector3f &W_ang_vel,
                                    const Eigen::Vector3f &joint_ang,
                                    const Eigen::Vector3f &joint_speed);

    static Eigen::Vector3f getIMU2FootPosition(int lnum,
                                        const Eigen::Matrix3f &body_rot,
                                        const Eigen::Vector3f &joint_ang);

    static Eigen::Vector3f getIMU2FootVelocity(int lnum,
                                        const Eigen::Matrix3f &body_rot,
                                        const Eigen::Vector3f &W_ang_vel,
                                        const Eigen::Vector3f &joint_ang,
                                        const Eigen::Vector3f &joint_speed);

    static Eigen::Vector3f getLegJointAngle(int lnum,
                                            const Eigen::Vector3f &FootPos);

    static Eigen::Vector3f getLegJointAngle(int lnum,
                                            const Eigen::Vector3f &B_bd2ft, Eigen::Vector3f old_angle_rad);

    static void imu2Bd(const Eigen::Matrix3f &body_rot,
                const Eigen::Vector3f &W_ang_vel,
                const Eigen::Vector3f &W_imu_pos,
                const Eigen::Vector3f &W_imu_vel,
                Eigen::Vector3f &W_bd_pos,
                Eigen::Vector3f &W_bd_vel);

};

#endif
