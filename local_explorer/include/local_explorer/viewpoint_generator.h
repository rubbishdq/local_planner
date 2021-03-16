# ifndef __VIEWPOINT_GENERATOR_H__
#define __VIEWPOINT_GENERATOR_H__

#include "local_explorer/viewpoint.h"

#include "global_mapper_ros/VoxelizedPoints.h"
#include "ros/ros.h"

namespace local_explorer
{
class ViewpointGenerator
{
public:
    ViewpointGenerator();

    void PreprocessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr, 
        std::vector<LabeledPoint> &cloud);
    void AddBoarderPoints(std::vector<LabeledPoint> &cloud, double origin[3]);
    void InvertPoints(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud);

    void ProcessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);

    std::shared_ptr<Viewpoint> GetViewpointPtr();
    bool IsGenerated();

private:
    std::shared_ptr<std::vector<LabeledPoint>> processed_cloud_ptr_;
    std::shared_ptr<std::vector<LabeledPoint>> inverted_cloud_ptr_;
    //char qhull_input_str_[500000];
    std::shared_ptr<Viewpoint> viewpoint_ptr_;

};

} // namespace local_explorer

#endif