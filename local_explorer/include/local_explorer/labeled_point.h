# ifndef __LABELED_POINT_H__
#define __LABELED_POINT_H__

#include "voxelized_points/voxelized_points.h"
#include "Eigen/Core"
#include <cmath>

namespace local_explorer
{

enum PointType {
    NORMAL, ADDITIONAL, BOARDER
};

class LabeledPoint : public voxelized_points::VoxelizedPoint
{
public:

    LabeledPoint(PointType point_type = NORMAL) : voxelized_points::VoxelizedPoint(), type_(point_type) {}
    LabeledPoint(int n, Eigen::Vector3f mu, Eigen::Matrix3f sigma, PointType point_type = NORMAL) 
        : voxelized_points::VoxelizedPoint(n, mu, sigma), type_(point_type)
    {
    }
    LabeledPoint(voxelized_points::VoxelizedPoint point, PointType point_type = NORMAL) 
        : voxelized_points::VoxelizedPoint(point.n_, point.mu_, point.sigma_), type_(point_type)
    {
        // there is no copy constructor for voxelized_points::VoxelizedPoint, or bugs will occur during compilation
        // TODO: fix these bugs
    }
    void Invert(Eigen::Vector3f origin, double param)
    {
        mu_ = (mu_-origin)/pow((mu_-origin).norm(), param)+origin;
    }

    void Reinvert(Eigen::Vector3f origin, double param)
    {
        mu_ = (mu_-origin)*pow((mu_-origin).norm(), 1/(1-param)-1)+origin;
    }

    PointType type_;
};

} // namespace local_explorer

#endif