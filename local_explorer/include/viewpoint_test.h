// similar to early version of local_explorer
// test visibility checking ability of viewpoint's kd-tree
#ifndef __VIEWPOINT_TEST_H__
#define __VIEWPOINT_TEST_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/viewpoint_generator.h"

#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "global_mapper_ros/VoxelizedPoints.h"
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>

#include "Eigen/Core"
#include <sstream>
#include <vector>
#include <list>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <stdarg.h>

namespace local_explorer
{

class ViewpointTest
{
public:
    ViewpointTest();

private:
    void RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void PublishViewpoint(Viewpoint &viewpoint);
    void PublishPoint(const ros::TimerEvent& event);

    void VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void PointCallback(const geometry_msgs::Point32::ConstPtr& msg_ptr);

    Eigen::Vector3f point_;

    ros::NodeHandle n_;

    ros::Publisher voxelized_points_pub_;
    ros::Publisher viewpoint_pub_;
    ros::Publisher point_pub_;

    ros::Subscriber voxelized_points_sub_;
    ros::Subscriber point_sub_;

    ros::Timer publishloop_timer_;

    std::unique_ptr<ViewpointGenerator> viewpoint_generator_ptr_;
};

} // namespace local_explorer

#endif
