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

double DirectionYaw(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    return double(atan2f(v2[1]-v1[1], v2[0]-v1[0]));
}

Eigen::Quaterniond DirectionQuatHorizonal(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    double yaw = DirectionYaw(v1, v2);
    return Eigen::Quaterniond(cos(yaw/2), 0, 0, sin(yaw/2));
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
    float u;
    //float v;
    u = -FX*pt[1]/pt[0] + CX;
    //v = -FY*pt[1]/pt[0] + CY;
    //return (u > 0 && u < float(CAM_RES[0]) && v > 0 && v < float(CAM_RES[1]));
    return (u > 0 && u < float(CAM_RES[0]));  // ignore v and erase frontier vertices at any height
}

// functions for plucker coordinate
void GetPlucker(Eigen::Vector3f a, Eigen::Vector3f b, float* L)
{
    L[0] = a[0] * b[1] - b[0] * a[1];
    L[1] = a[0] * b[2] - b[0] * a[2];
    L[2] = a[0] - b[0];
    L[3] = a[1] * b[2] - b[1] * a[2];
    L[4] = a[2] - b[2];
    L[5] = b[1] - a[1];
}

float SideOp(float* L1, float* L2)
{
    return L1[0] * L2[4] + L1[1] * L2[5] + L1[2] * L2[3] + L1[3] * L2[2] + L1[4] * L2[0] + L1[5] * L2[1];
}
// end of functions for plucker coordinate

// check if a line segment intersects with a triangle
// https://members.loria.fr/SLazard/ARC-Visi3D/Pant-project/files/Line_Segment_Triangle.html
bool IsLineSegIntersectTri(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f start, Eigen::Vector3f end)
{
    float s1, s2, s3;
    float *L1 = new float[6];
    float *e1 = new float[6];
    float *e2 = new float[6];
    float *e3 = new float[6];
    GetPlucker(start, end, L1);
    GetPlucker(p1, p2, e1);
    GetPlucker(p2, p3, e2);
    GetPlucker(p3, p1, e3);
    s1 = SideOp(L1, e1);
    s2 = SideOp(L1, e2);
    s3 = SideOp(L1, e3);
    delete []L1;
    delete []e2;
    delete []e3;
    if (!(s1*s2 >= 0 && s1*s3 >= 0))
    {
        delete []e1;
        return false;
    }

    float *L2 = new float[6];
    float *L3 = new float[6];
    GetPlucker(start, p3, L2);
    GetPlucker(p3, end, L3);
    s1 = SideOp(e1, L2);
    s2 = SideOp(e1, L3);
    delete []e1;
    delete []L2;
    delete []L3;
    return (s1*s2 > 0);
}

bool IsHeightBetweenLimit(Eigen::Vector3f pt)
{
    return !((pt[2]-BOARDER_Z_RANGE[0]) < HEIGHT_DIFF_THRESHOLD || 
                    (-pt[2]+BOARDER_Z_RANGE[1]) < HEIGHT_DIFF_THRESHOLD);
}

} // namespace local_explorer

#endif