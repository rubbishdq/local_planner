#include "voxelized_points/voxelized_points.h"

namespace voxelized_points
{
VoxelizedPoint::VoxelizedPoint()
{
    n_ = 0;
    mu_ = Eigen::Vector3f::Zero();
    sigma_ = Eigen::Matrix3f::Zero();
}

VoxelizedPoint::VoxelizedPoint(int n, Eigen::Vector3f mu, Eigen::Matrix3f sigma)
{
    n_ = n;
    mu_ = mu;
    sigma_ = sigma;
}

VoxelizedPoint::VoxelizedPoint(const std::vector<Eigen::Vector3f>& points)
{
    CalculateParams(points);
}

void VoxelizedPoint::CalculateParams(const std::vector<Eigen::Vector3f>& points)
{
    n_ = points.size();
    if (n_ == 0)
        return;
    if (n_ == 1)
    {
        mu_ = points[0];
        sigma_ = Eigen::Matrix3f::Zero();
        return;
    }
    mu_.setZero();
    sigma_.setZero();
    for (auto pt : points)
    {
        mu_ += pt;
    }
    mu_ /= n_;
    for (auto pt : points)
    {
        Eigen::Vector3f diff = pt - mu_;
        sigma_ += diff*diff.transpose();
    }
    sigma_ /= n_;
}

void VoxelizedPoint::InsertPoint(Eigen::Vector3f point)
{
    if (n_ == 0)
    {
        mu_ = point;
        sigma_ = Eigen::Matrix3f::Zero();
    }
    else
    {
        Eigen::Vector3f mu_new = (n_*mu_+point) / (n_+1);
        Eigen::Matrix3f sigma_new = (n_*sigma_+point*point.transpose()
            +n_*mu_*mu_.transpose()-(n_+1)*mu_new*mu_new.transpose());
        mu_ = mu_new;
        sigma_ = sigma_new;
    }
    n_++;
}

void VoxelizedPoint::InsertPoint(const double point[3])
{
    Eigen::Vector3f point_vector((float)point[0], (float)point[1], (float)point[2]);
    InsertPoint(point_vector);
}

void VoxelizedPoint::Clear()
{
    n_  = 0;
    mu_.setZero();
    sigma_.setZero();
}

std::ostream& operator <<(std::ostream &output, VoxelizedPoint p)
{
    output << "n_:  " << p.n_ << std::endl;
    output << "mu:\n" << p.mu_.transpose() << std::endl;
    output << "sigma:\n" << p.sigma_ << std::endl;
    return output;
}

VoxelizedPoints::VoxelizedPoints(const double origin[3], const double world_dimensions[3], const double resolution)
    : voxel_grid::VoxelGrid<VoxelizedPoint>(origin, world_dimensions, resolution)
{
  double indexer_origin[3];
  GetOrigin(indexer_origin);
  UpdateOrigin(indexer_origin);
}

void VoxelizedPoints::InsertPoint(Eigen::Vector3f point)
{
    VoxelizedPoint *DataPtr = &(this->GetReference(double(point[0]), double(point[1]), double(point[2])));
    DataPtr->InsertPoint(point);
}

void VoxelizedPoints::InsertPoint(const double point[3])
{
    VoxelizedPoint *DataPtr = &(this->GetReference(point));
    DataPtr->InsertPoint(point);
}

void VoxelizedPoints::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
    // empty
}

void VoxelizedPoints::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
  for (int index : slice_indexes)
  {
    WriteValue(index, empty_voxel_);
  }
}

}// namespace voxelized_points