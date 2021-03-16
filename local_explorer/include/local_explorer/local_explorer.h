#ifndef __LOCAL_EXPLORER_H__
#define __LOCAL_EXPLORER_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/viewpoint_generator.h"

#include "sensor_msgs/PointCloud2.h"
#include "global_mapper_ros/VoxelizedPoints.h"
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>

#include "Eigen/Core"
#include <sstream>
#include <vector>
#include <memory>

namespace local_explorer
{

class LocalExplorer
{
public:
    LocalExplorer();

private:
    void RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void PublishViewpoint(Viewpoint &viewpoint);
    void PublishCandidateFrontier(Viewpoint &viewpoint);

    void VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);

    ros::NodeHandle n_;

    ros::Publisher voxelized_points_pub_;
    ros::Publisher viewpoint_pub_;
    ros::Publisher frontier_pub_;

    ros::Subscriber voxelized_points_sub_;

    std::unique_ptr<ViewpointGenerator> viewpoint_generator_ptr_;
};

} // namespace local_explorer

#endif
