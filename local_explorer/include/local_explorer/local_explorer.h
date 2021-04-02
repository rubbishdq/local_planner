#ifndef __LOCAL_EXPLORER_H__
#define __LOCAL_EXPLORER_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/viewpoint_generator.h"

#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
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

class LocalExplorer
{
public:
    LocalExplorer();

private:
    void InitFrontierColor();
    void RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void PublishInvertedCloud(ViewpointGenerator &viewpoint_generator);
    void PublishConvexHull(ConvexHull &convex_hull);  // inverted
    void PublishColoredConvexHull(ConvexHull &convex_hull);  // inverted
    void PublishViewpoint(Viewpoint &viewpoint);
    void PublishSingleFrontierCluster(Viewpoint &viewpoint);

    void VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);

    std::list<std::shared_ptr<Viewpoint>> viewpoint_list_;

    float frontier_color_[FRONTIER_COLOR_COUNT][3];  // used to visualize frontier clusters

    ros::NodeHandle n_;

    ros::Publisher voxelized_points_pub_;
    ros::Publisher inverted_cloud_pub_;
    ros::Publisher convex_hull_pub_;
    ros::Publisher colored_convex_hull_pub_;
    ros::Publisher viewpoint_pub_;
    ros::Publisher single_frontier_cluster_list_pub_;

    ros::Subscriber voxelized_points_sub_;

    std::unique_ptr<ViewpointGenerator> viewpoint_generator_ptr_;
};

} // namespace local_explorer

#endif
