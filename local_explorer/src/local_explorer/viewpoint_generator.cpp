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
            float evalue[3]; 
            Eigen::Vector3f evector[3];
            CalcEigen(sigma, evalue, evector);
            /*
            std::cout << "n: " << n << std::endl;
            std::cout << "mu: " << mu << std::endl;
            std::cout << "sigma: " << sigma << std::endl;
            */
            for (int i = 0; i < 3; i++)  // sometimes eigenvalue will be negative due to computational error
            {
                /*
                std::cout << "eigenvalue " << i << ":  " << evalue[i] << std::endl;
                std::cout << "eigenvector " << i << ":  " << evector[i] << std::endl;
                */
                if (evalue[i] < 0)
                {
                    evalue[i] = 0;
                }
            }
            for (int i = -EXTENSION_COUNT; i <= EXTENSION_COUNT; i++)
            {
                for (int j = -EXTENSION_COUNT; j <= EXTENSION_COUNT; j++)
                {
                    if ((evalue[1] == 0 && i != 0) || (evalue[2] == 0 && j != 0))
                    {
                        continue;
                    }
                    Eigen::Vector3f mu_new = mu+(0.5*i*sqrt(evalue[1]))*evector[1]+(0.5*j*sqrt(evalue[2]))*evector[2];
                    //std::cout << mu_new << std::endl;
                    cloud.push_back(LabeledPoint(n, mu_new, sigma, ADDITIONAL)); 
                }
            }
        }
        cloud.push_back(LabeledPoint(n, mu, sigma, NORMAL)); 
    }
    AddBoarderPoints(cloud);
}

void ViewpointGenerator::AddBoarderPoints(std::vector<LabeledPoint> &cloud)
{
    double lowest_vertex[3] = {origin_[0]-BOARDER_DIMENSION[0]/2, origin_[1]-BOARDER_DIMENSION[1]/2, origin_[2]-BOARDER_DIMENSION[2]/2};
    double highest_vertex[3] = {origin_[0]+BOARDER_DIMENSION[0]/2, origin_[1]+BOARDER_DIMENSION[1]/2, origin_[2]+BOARDER_DIMENSION[2]/2};
    if (lowest_vertex[2] < BOARDER_HEIGHT_RANGE[0])
    {
        lowest_vertex[2] = BOARDER_HEIGHT_RANGE[0];
    }
    if (highest_vertex[2] < BOARDER_HEIGHT_RANGE[1])
    {
        highest_vertex[2] = BOARDER_HEIGHT_RANGE[1];
    }
    Eigen::Vector3f mu;
    
    for (double diff_x = BOARDER_RESOLUTION; diff_x < BOARDER_DIMENSION[0]; diff_x+=BOARDER_RESOLUTION)
    {
        for (double diff_y = BOARDER_RESOLUTION; diff_y < BOARDER_DIMENSION[1]; diff_y+=BOARDER_RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]+diff_y), (float)(highest_vertex[2]);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
    for (double diff_x = BOARDER_RESOLUTION; diff_x < BOARDER_DIMENSION[0]; diff_x+=BOARDER_RESOLUTION)
    {
        for (double diff_z = BOARDER_RESOLUTION; diff_z < BOARDER_DIMENSION[2]; diff_z+=BOARDER_RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(lowest_vertex[0]+diff_x), (float)(highest_vertex[1]), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
    for (double diff_y = BOARDER_RESOLUTION; diff_y < BOARDER_DIMENSION[1]; diff_y+=BOARDER_RESOLUTION)
    {
        for (double diff_z = BOARDER_RESOLUTION; diff_z < BOARDER_DIMENSION[2]; diff_z+=BOARDER_RESOLUTION)
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
        point.invert(origin_, INVERT_PARAM);
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

std::shared_ptr<std::vector<LabeledPoint>> ViewpointGenerator::GetProcessedCloudPtr()
{
    return processed_cloud_ptr_;
}

std::shared_ptr<std::vector<LabeledPoint>> ViewpointGenerator::GetInvertedCloudPtr()
{
    return inverted_cloud_ptr_;
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



