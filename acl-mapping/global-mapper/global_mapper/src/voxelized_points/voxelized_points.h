// created by dqs
#pragma once

#include <array>
#include <vector>
#include <cstring>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>

#include "voxel_grid/voxel_grid.h"

namespace voxelized_points
{
class VoxelizedPoint
{
public:
    VoxelizedPoint();
    VoxelizedPoint(const std::vector<Eigen::Vector3f>& points);
    void CalculateParams(const std::vector<Eigen::Vector3f>& points);
    void InsertPoint(Eigen::Vector3f point);
    VoxelizedPoint& operator =(VoxelizedPoint &p2);

    int n_;
    Eigen::Vector3f mu_;
    Eigen::Matrix3f sigma_;
};
std::ostream& operator <<(std::ostream &output, VoxelizedPoint &p);

class VoxelizedPoints : public voxel_grid::VoxelGrid<VoxelizedPoint>
{
// TODO
public:
    VoxelizedPoints(const double origin[3], const double world_dimensions[3], const double resolution);
private:
    virtual void PreShiftOrigin(const std::vector<int>& slice_indexes) override;
    virtual void PostShiftOrigin(const std::vector<int>& slice_indexes) override;
};


}// namespace voxelized_points