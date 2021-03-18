#include "local_explorer/viewpoint_generator.h"

namespace local_explorer
{

ViewpointGenerator::ViewpointGenerator()
{
    viewpoint_ptr_ = std::make_shared<Viewpoint>();
    is_initialized_ = false;
}

void ViewpointGenerator::PreprocessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr, 
    std::vector<LabeledPoint> &cloud)
{
    origin_ << msg_ptr->origin.x, msg_ptr->origin.y, msg_ptr->origin.z;
    int n;
    Eigen::Vector3f mu;
    Eigen::Matrix3f sigma;
    for (auto &point : msg_ptr->points)
    {
        n = point.n;
        mu << point.mu[0], point.mu[1], point.mu[2];
        sigma << point.sigma[0], point.sigma[1], point.sigma[2], 
            point.sigma[1], point.sigma[3], point.sigma[4], 
            point.sigma[2], point.sigma[4], point.sigma[5];
        if ((origin_-mu).norm() <= MIN_RANGE_FOR_EXTENSION)
        {
            continue;
        }
        cloud.push_back(LabeledPoint(n, mu, sigma, NORMAL)); 
    }
    AddBoarderPoints(cloud);
}

void ViewpointGenerator::AddBoarderPoints(std::vector<LabeledPoint> &cloud)
{
    double lowest_vertex[3] = {origin_[0]-DIMENSION[0]/2, origin_[1]-DIMENSION[1]/2, origin_[2]-DIMENSION[2]/2};
    double highest_vertex[3] = {origin_[0]+DIMENSION[0]/2, origin_[1]+DIMENSION[1]/2, origin_[2]+DIMENSION[2]/2};
    Eigen::Vector3f mu;
    for (double diff_x = RESOLUTION; diff_x < DIMENSION[0]; diff_x+=RESOLUTION)
    {
        for (double diff_y = RESOLUTION; diff_y < DIMENSION[1]; diff_y+=RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]+diff_y), (float)(highest_vertex[2]);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
    for (double diff_x = RESOLUTION; diff_x < DIMENSION[0]; diff_x+=RESOLUTION)
    {
        for (double diff_z = RESOLUTION; diff_z < DIMENSION[2]; diff_z+=RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(lowest_vertex[0]+diff_x), (float)(highest_vertex[1]), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
    for (double diff_y = RESOLUTION; diff_y < DIMENSION[1]; diff_y+=RESOLUTION)
    {
        for (double diff_z = RESOLUTION; diff_z < DIMENSION[2]; diff_z+=RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(highest_vertex[0]), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
}

void ViewpointGenerator::InvertPoints(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud)
{
    inverted_cloud.assign(cloud.begin(), cloud.end());
    for (auto &point : inverted_cloud)
    {
        point.invert(INVERT_PARAM);
    }
}

void ViewpointGenerator::ProcessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    processed_cloud_ptr_ = std::make_shared<std::vector<LabeledPoint>>();
    inverted_cloud_ptr_ = std::make_shared<std::vector<LabeledPoint>>();

    PreprocessVoxelizedPoints(msg_ptr, *processed_cloud_ptr_);

    InvertPoints(*processed_cloud_ptr_, *inverted_cloud_ptr_);

    viewpoint_ptr_->GenerateViewpoint(*processed_cloud_ptr_, *inverted_cloud_ptr_);
    is_initialized_ = true;
}

std::shared_ptr<Viewpoint> ViewpointGenerator::GetViewpointPtr()
{
    return viewpoint_ptr_;
}

bool ViewpointGenerator::IsGenerated()
{
    if (is_initialized_)
        return viewpoint_ptr_->IsGenerated();
    return false;
}

} // namespace local_explorer



