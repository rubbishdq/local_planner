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
    VoxelizedPoint(int n, Eigen::Vector3f mu, Eigen::Matrix3f sigma);
    VoxelizedPoint(const std::vector<Eigen::Vector3f>& points);
    void CalculateParams(const std::vector<Eigen::Vector3f>& points);
    void InsertPoint(Eigen::Vector3f point);
    void InsertPoint(const double point[3]);
    void Clear();

    int n_;
    Eigen::Vector3f mu_;
    Eigen::Matrix3f sigma_;
};
std::ostream& operator <<(std::ostream &output, VoxelizedPoint p);

class VoxelizedPoints : public voxel_grid::VoxelGrid<VoxelizedPoint>
{
// TODO
public:
    VoxelizedPoints(const double origin[3], const double world_dimensions[3], const double resolution);
    void InsertPoint(Eigen::Vector3f point);
    void InsertPoint(const double point[3]);
private:
    virtual void PreShiftOrigin(const std::vector<int>& slice_indexes) override;
    virtual void PostShiftOrigin(const std::vector<int>& slice_indexes) override;

    static VoxelizedPoint empty_voxel_;
};
VoxelizedPoint VoxelizedPoints::empty_voxel_;

}// namespace voxelized_points