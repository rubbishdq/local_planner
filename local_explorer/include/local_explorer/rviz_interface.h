//#include "voxelized_points/voxelized_points.h"

#include "sensor_msgs/PointCloud2.h"
#include "global_mapper_ros/VoxelizedPoints.h"

#include <pcl_ros/point_cloud.h>
#include "ros/ros.h"

#include <iostream>

namespace local_explorer
{

class RvizInterfaceNode
{
public:
    RvizInterfaceNode();
private:
    void RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    ros::NodeHandle n_;

    ros::Publisher voxelized_points_pub_;

    ros::Subscriber voxelized_points_sub_;

};

} // namespace local_explorer