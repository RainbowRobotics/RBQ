#ifndef OPERATION_H
#define OPERATION_H

#include <Eigen/Dense>

inline float signf(float a){return (a<0 ? -1.0 : 1.0);}

/*!
 * frequently-used Vector, Matrix
 */
using Vec2f  = Eigen::Matrix<float, 2, 1>;      using Vec2d  = Eigen::Matrix<double, 2, 1>;
using Vec3f  = Eigen::Matrix<float, 3, 1>;      using Vec3d  = Eigen::Matrix<double, 3, 1>;
using Vec4f  = Eigen::Matrix<float, 4, 1>;      using Vec4d  = Eigen::Matrix<double, 4, 1>;
using Vec5f  = Eigen::Matrix<float, 5, 1>;      using Vec5d  = Eigen::Matrix<double, 5, 1>;
using Vec6f  = Eigen::Matrix<float, 6, 1>;      using Vec6d  = Eigen::Matrix<double, 6, 1>;
using Vec12f = Eigen::Matrix<float, 12, 1>;     using Vec12d = Eigen::Matrix<double, 12, 1>;
using Vec18f = Eigen::Matrix<float, 18, 1>;     using Vec18d = Eigen::Matrix<double, 18, 1>;

using Mat2f = Eigen::Matrix<float, 2, 2>;       using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat3f = Eigen::Matrix<float, 3, 3>;       using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4f = Eigen::Matrix<float, 4, 4>;       using Mat4d = Eigen::Matrix<double, 4, 4>;
using Mat6f = Eigen::Matrix<float, 6, 6>;       using Mat6d = Eigen::Matrix<double, 6, 6>;

namespace Eigen
{
typedef Eigen::Matrix<double,6,1> Vector6d;
}

class Operation {
public:
    static Eigen::Vector4f rot2QUAT(Eigen::Matrix3f m);

    static Eigen::Matrix3f quat2ROT(const Eigen::Vector4f quat);

    static Eigen::Vector3f quat2RPY(const Eigen::Vector4f quat);

    static Eigen::Vector3f quat2RPY2(const Eigen::Vector4f quat);

    static Eigen::Vector4f rpy2QUAT(Eigen::Vector3f rpy);

    static Eigen::Matrix3f rpy2ROT(Eigen::Vector3f rpy);

    static Eigen::Vector4f quatProduct(const Eigen::Vector4f &a, const Eigen::Vector4f &b);

    static Eigen::Matrix3f diagonalize3f(const float a, const float b, const float c);

    static Eigen::Matrix3f vec2HAT(Eigen::Vector3f vec);

    static Eigen::Vector3f rot2RPY(Eigen::Matrix3f mat);

    static Eigen::Vector3f rot2RPY_2(Eigen::Matrix3f mat);

    static Eigen::Vector4f rot2AXAN(Eigen::Matrix3f mat);

    static Eigen::Matrix3f axan2ROT(Eigen::Vector4f axan);

    static double Norm_double(Eigen::Vector3d s1, Eigen::Vector3d s2);

    static Eigen::Matrix3f normal2ROT(Eigen::Vector3f n);

    static Eigen::Vector3f proj(Eigen::Vector3f a, Eigen::Vector3f b);

    static Eigen::Vector3f orientationERR(Eigen::Matrix3f Rdes, Eigen::Matrix3f R);

    static Eigen::Matrix3f rotz2MAT(float rotz_rad);

    static Eigen::Matrix3f roty2MAT(float roty_rad);

    static Eigen::Matrix3f rotx2MAT(float rotx_rad);

    // Convert 2D cartesian points to polar angles in range 0 ~ 2pi
    static float point2rad(float x, float y);

    static Eigen::Vector3f global2local_point(float rotz_rad, Eigen::Vector3f _pCenter, Eigen::Vector3f _pGlobal);

    static Eigen::Vector3f local2global_point(float rotz_rad, Eigen::Vector3f _pCenter, Eigen::Vector3f _pLocal);

    static Eigen::Matrix4f getTransformationMat(Eigen::Matrix3f _rot, Eigen::Vector3f _vec_offset);

    static Eigen::Matrix3f averageRotation(const Eigen::Matrix3f &R1, const Eigen::Matrix3f &R2);

    static bool isInsideTriangle(const Eigen::Vector3f& point1,
                                 const Eigen::Vector3f& point2,
                                 const Eigen::Vector3f& point3,
                                 const Eigen::Vector3f& _X);

    static bool isInsideTriangleWithinDistance(const Eigen::Vector3f& point1,
                                               const Eigen::Vector3f& point2,
                                               const Eigen::Vector3f& point3,
                                               const Eigen::Vector3f& _X,
                                               float distance);

    static Eigen::Vector3f saturatePoint(const Eigen::Vector3f& centorPoint, Eigen::Vector3f& inputPoint, float radius);

    static float saturateValue(const float& centerValue, float& inputValue, float range);

    static Eigen::Vector3f findIntersection(Eigen::Vector3f P0, Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3);

    ///
    /// \brief ZYX_to_TF
    /// \param tx in meters
    /// \param ty in meters
    /// \param tz in meters
    /// \param rx in radian
    /// \param ry in radian
    /// \param rz in radian
    /// \return TF
    ///
    static Eigen::Matrix4d ZYX_to_TF(const double &tx, const double &ty, const double &tz, const double &rx, const double &ry, const double &rz);

    ///
    /// \brief TF_to_ZYX
    /// \param TF
    /// \return ZYX is euler "tx,ty,tz,rx,ry,rz" (meter, radian)
    ///
    static Eigen::Vector6d TF_to_ZYX(const Eigen::Matrix4d &TF);

    ///
    /// \brief string_to_TF
    /// \param string is ZYX euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    /// \return TF
    ///
    static Eigen::Matrix4d string_to_TF(const std::string &string);

    ///
    /// \brief string_to_TF
    /// \param str is ZYX euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    /// \return true if success
    ///
    static bool string_to_TF(const std::string &_string, Eigen::Matrix4d &_out_TF);


    ///
    /// \brief TF_to_string
    /// \param TF
    /// \return string is ZYX euler "tx,ty,tz,rx,ry,rz" (meter, deg)
    ///
    static std::string TF_to_string(const Eigen::Matrix4d &TF);

    ///
    /// \brief generatePoints between given end-points
    /// \param p0 end-point 0
    /// \param p1 end-point 1
    /// \param p2 end-point 2
    /// \return
    ///
    static std::vector<Eigen::Vector3d> generatePoints(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);

}; // namespace Operation

class Filter {
public:
    static float LPF(float input, float output, float f_cut, float ts);

    static Eigen::Vector3f LPF3d(Eigen::Vector3f input, Eigen::Vector3f output, float f_cut, float ts);

    static Eigen::Vector3f LPF3f(Eigen::Vector3f input, Eigen::Vector3f output, float f_cut, float ts);

    static float HPF(float input, float input_pre, float output, float f_cut, float ts);

    static float Limit(float input, float low_limit, float upper_limit);

    static Eigen::Vector3f Limit3f(Eigen::Vector3f input, Eigen::Vector3f low_limit, Eigen::Vector3f upper_limit);

    static float WindowFilter(float input, float output, float size);

    static float DeadZone(float input, float low_clamp, float upper_clamp);

    static float DeadZone2(float input, float low_clamp, float upper_clamp);

    static float Max(float input1, float input2);

    static float Min(float input1, float input2);

    static float Max3(float input1, float input2, float input3);

    static float Min3(float input1, float input2, float input3);

    static float Max4(float input1, float input2, float input3, float input4);

    static float VelLimitByAcc(float desiredVel, float previousVel, float desiredAcc, float desiredDcc, float dt_);

    static float MaxFilter(float input, float output, float decaying_rate, float ts);

};

class ConvexPolygon {
public:
    template <typename T>
    static T clamp(const T& value, const T& minVal, const T& maxVal);

    // Computes the cross product of vectors AB and AC
    static float crossProduct(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& C);

    // Computes the convex hull of a set of points
    static std::vector<Eigen::Vector2f> computeConvexHull(std::vector<Eigen::Vector2f>& points);

    // Computes the distance from a point to a line segment
    static float pointToSegmentDistance(const Eigen::Vector2f& A, const Eigen::Vector2f& B, const Eigen::Vector2f& P);

    // Main function: Checks if point X is inside the convex hull of points A, B, C, D
    // or computes the distance to the convex hull if outside
    static float isPointInsideOrDistance(const Eigen::Vector3f& A, const Eigen::Vector3f& B, const Eigen::Vector3f& C, const Eigen::Vector3f& D, const Eigen::Vector3f& X);

};

#endif
