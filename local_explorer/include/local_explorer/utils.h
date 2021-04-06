#ifndef __UTILS_H__
#define __UTILS_H__

#include "local_explorer/common.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <cstdio>
#include <cstring>

namespace local_explorer
{

inline double Random(double min_val = 0.0, double max_val = 1.0)
{
    return (double(rand())/RAND_MAX) * (max_val - min_val) + min_val;
}

inline void PrintSpace(int space_count)
{
    if (space_count <= 0)
        return;
    char *str = new char[space_count+1]; 
    memset(str, ' ', space_count*sizeof(char));
    str[space_count] = '\0';
    printf("%s", str);
    delete[]str;
}

inline Eigen::Vector3f Invert(Eigen::Vector3f pos, Eigen::Vector3f origin, double param = INVERT_PARAM)
{
    return (pos-origin)/pow((pos-origin).norm(), param);
}

inline bool InBoxRange(Eigen::Vector3f pos)
{
    for (int i = 0; i < 3; i++)
    {
        if (pos[i] > BOARDER_DIMENSION[i] || pos[i] < -BOARDER_DIMENSION[i])
            return false;
    }
    return true;
}

// get eigenvalues and corresponding eigenvectors, with eigenvalues sorted
void CalcEigen(Eigen::Matrix3f mat, float evalue[3], Eigen::Vector3f evector[3])  // get eigenvalues and corresponding eigenvectors, with eigenvalues sorted
{
    Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(mat);
    auto evalue_us = eigen_solver.eigenvalues().real();
    auto evector_us = eigen_solver.eigenvectors().real();
    if (evalue_us[0] > evalue_us[1] && evalue_us[0] > evalue_us[2])
    {
        evalue[0] = evalue_us[0];
        evector[0] = evector_us.col(0);
        if (evalue_us[1] > evalue_us[2])
        {
            evalue[1] = evalue_us[1];
            evector[1] = evector_us.col(1);
            evalue[2] = evalue_us[2];
            evector[2] = evector_us.col(2);
        }
        else
        {
            evalue[1] = evalue_us[2];
            evector[1] = evector_us.col(2);
            evalue[2] = evalue_us[1];
            evector[2] = evector_us.col(1);
        }
    }
    else if (evalue_us[1] > evalue_us[2])
    {
        evalue[0] = evalue_us[1];
        evector[0] = evector_us.col(1);
        if (evalue_us[0] > evalue_us[2])
        {
            evalue[1] = evalue_us[0];
            evector[1] = evector_us.col(0);
            evalue[2] = evalue_us[2];
            evector[2] = evector_us.col(2);
        }
        else
        {
            evalue[1] = evalue_us[2];
            evector[1] = evector_us.col(2);
            evalue[2] = evalue_us[0];
            evector[2] = evector_us.col(0);
        }
    }
    else
    {
        evalue[0] = evalue_us[2];
        evector[0] = evector_us.col(2);
        if (evalue_us[0] > evalue_us[1])
        {
            evalue[1] = evalue_us[0];
            evector[1] = evector_us.col(0);
            evalue[2] = evalue_us[1];
            evector[2] = evector_us.col(1);
        }
        else
        {
            evalue[1] = evalue_us[1];
            evector[1] = evector_us.col(1);
            evalue[2] = evalue_us[0];
            evector[2] = evector_us.col(0);
        }
    }
}

double MaxElementd(Eigen::Vector3d v)
{
    return (v[0] > v[1] && v[0] > v[2]) ? v[0] : (v[1] > v[2] ? v[1] : v[2]);
}

float MaxElementf(Eigen::Vector3f v)
{
    return (v[0] > v[1] && v[0] > v[2]) ? v[0] : (v[1] > v[2] ? v[1] : v[2]);
}

double MinElementd(Eigen::Vector3d v)
{
    return (v[0] < v[1] && v[0] < v[2]) ? v[0] : (v[1] < v[2] ? v[1] : v[2]);
}

float MinElementf(Eigen::Vector3f v)
{
    return (v[0] < v[1] && v[0] < v[2]) ? v[0] : (v[1] < v[2] ? v[1] : v[2]);
}

double distxyd(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    return sqrt((v2[0]-v1[0])*(v2[0]-v1[0])+(v2[1]-v1[1])*(v2[1]-v1[1]));
}

float distxyf(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    return sqrt((v2[0]-v1[0])*(v2[0]-v1[0])+(v2[1]-v1[1])*(v2[1]-v1[1]));
}

Eigen::Vector3d EulerAngleDiff(Eigen::Quaterniond q1, Eigen::Quaterniond q2) // q1/q2
{
    Eigen::Quaterniond q = q1*q2.conjugate();
    double roll, pitch, yaw;
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(roll, pitch, yaw) / M_PI * 180;
}

bool InCameraRange(Eigen::Vector3f pt) // pt is in robot's frame
{
    if (pt[0] <= 0)
        return false;
    float u, v;
    u = -FX*pt[1]/pt[0] + CX;
    v = -FY*pt[1]/pt[0] + CY;
    //return (u > 0 && u < float(CAM_RES[0]) && v > 0 && v < float(CAM_RES[1]));
    return (u > 0 && u < float(CAM_RES[0]));  // ignore v and erase frontier vertices at any height
}

} // namespace local_explorer

#endif