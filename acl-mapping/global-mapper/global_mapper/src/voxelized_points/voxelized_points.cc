#include "voxelized_points/voxelized_points.h"

namespace voxelized_points
{

VoxelizedPoint::VoxelizedPoint()
{
    n_ = 0;
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
    for (auto iter = points.begin(); iter != points.end(); iter++)
    {
        mu_ += *iter;
    }
    mu_ /= n_;
    for (auto iter = points.begin(); iter != points.end(); iter++)
    {
        Eigen::Vector3f diff = *iter - mu_;
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

 VoxelizedPoint& VoxelizedPoint::operator =(VoxelizedPoint &p2)
 {
     n_ = p2.n_;
     mu_ = p2.mu_;
     sigma_ = p2.sigma_;
 }

std::ostream& operator <<(std::ostream &output, VoxelizedPoint &p)
{
    output << "n_:  " << p.n_ << std::endl;
    output << "mu:\n" << p.mu_.transpose() << std::endl;
    output << "sigma:\n" << p.sigma_ << std::endl;
}

void VoxelizedPoints::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
    //TODO
}

void VoxelizedPoints::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
    //TODO
}

}// namespace voxelized_points