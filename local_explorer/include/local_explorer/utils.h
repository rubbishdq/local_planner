#ifndef __UTILS_H__
#define __UTILS_H__

#include "Eigen/Core"
#include "Eigen/Dense"
#include <cstdio>

namespace local_explorer
{
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