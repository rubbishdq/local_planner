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


} // namespace local_explorer

#endif