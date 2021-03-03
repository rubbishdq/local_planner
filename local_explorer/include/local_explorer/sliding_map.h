#include "common.h"

#include "Eigen/Core"

#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>


namespace localExplorer
{

class PointVoxelized
{
public:
    int n;  // amount of points
    Eigen::Vector3d mu;
    Eigen::Matrix3d sigma;
};

}

