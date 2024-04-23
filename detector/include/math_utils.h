#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

#define pi 3.14159269398979

Eigen::Vector4d quatInverse(const Eigen::Vector4d &q)
{
    Eigen::Vector4d inv_quat;
    inv_quat << q(0), -q(1), -q(2), -q(3);
    return inv_quat;
}

Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p)
{
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
        p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
        p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

Eigen::Vector3d quat_VectorMultiplication(const Eigen::Vector4d &q, const Eigen::Vector3d &v)
{
    Eigen::Vector3d vec;
    vec << (1 - 2 * q(2) * q(2) - 2 * q(3) * q(3)) * v(0) + 2 * (q(1) * q(2) - q(0) * q(3)) * v(1) + 2 * (q(1) * q(3) + q(0) * q(2)) * v(2),
        2 * (q(1) * q(2) + q(0) * q(3)) * v(0) + (1 - 2 * q(1) * q(1) - 2 * q(3) * q(3)) * v(1) + 2 * (q(2) * q(3) - q(0) * q(1)) * v(2),
        2 * (q(1) * q(3) - q(0) * q(2)) * v(0) + 2 * (q(2) * q(3) + q(0) * q(1)) * v(1) + (1 - 2 * q(1) * q(1) - 2 * q(2) * q(2)) * v(2);
    return vec;
}

// 四元数转欧拉角
Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond &q)
{
    // YPR - ZYX
    return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
    // YPR - ZYX
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()));
}

// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

Eigen::Vector3d quaternion_to_euler(const Eigen::Vector4d &q)
{
    float quat[4];
    quat[0] = q(0);
    quat[1] = q(1);
    quat[2] = q(2);
    quat[3] = q(3);

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

// 旋转矩阵转欧拉角
Eigen::Vector3d rotation_to_euler(Eigen::Matrix3d dcm)
{
    Eigen::Vector3d euler_angle;

    double phi_val = atan2(dcm(2, 1), dcm(2, 2));
    double theta_val = asin(-dcm(2, 0));
    double psi_val = atan2(dcm(1, 0), dcm(0, 0));
    // double pi = M_PI;

    if (fabs(theta_val - pi / 2.0) < 1.0e-3)
    {
        phi_val = 0.0;
        psi_val = atan2(dcm(1, 2), dcm(0, 2));
    }
    else if (fabs(theta_val + pi / 2.0) < 1.0e-3)
    {
        phi_val = 0.0;
        psi_val = atan2(-dcm(1, 2), -dcm(0, 2));
    }

    euler_angle(0) = phi_val;
    euler_angle(1) = theta_val;
    euler_angle(2) = psi_val;

    return euler_angle;
}

// constrain_function
float constrain_function(float data, float Max)
{
    if (abs(data) > Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
    return 0.0;
}

// constrain_function2
float constrain_function2(float data, float Min, float Max)
{
    if (data > Max)
    {
        return Max;
    }
    else if (data < Min)
    {
        return Min;
    }
    else
    {
        return data;
    }
    return 0.0;
}

// sign_function
float sign_function(float data)
{
    if (data > 0)
    {
        return 1.0;
    }
    else if (data < 0)
    {
        return -1.0;
    }
    else if (data == 0)
    {
        return 0.0;
    }
    return 0.0;
}

// min function
float min(float data1, float data2)
{
    if (data1 >= data2)
    {
        return data2;
    }
    else
    {
        return data1;
    }
    return 0.0;
}

#endif
