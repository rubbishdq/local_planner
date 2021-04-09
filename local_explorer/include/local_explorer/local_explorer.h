#ifndef __LOCAL_EXPLORER_H__
#define __LOCAL_EXPLORER_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/viewpoint_generator.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "global_mapper_ros/VoxelizedPoints.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>

#include "Eigen/Core"
#include <sstream>
#include <vector>
#include <list>
#include <queue>
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
    int DetermineOperation();
    void RemoveRedundantBoarder(Viewpoint &viewpoint, bool last_viewpoint); // if last_viewpoint is false, last added viewpoint's boarder will not be removed
    void ProcessNewViewpoint(std::shared_ptr<Viewpoint> viewpoint_ptr);
    void UpdateTopologicalMap(std::shared_ptr<Viewpoint> viewpoint_ptr); // viewpoint_ptr should not be in this->viewpoint_list_

    void RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void PublishInvertedCloud(ViewpointGenerator &viewpoint_generator);
    void PublishConvexHull(ConvexHull &convex_hull);  // inverted
    void PublishColoredConvexHull(ConvexHull &convex_hull);  // inverted
    void PublishViewpoint(Viewpoint &viewpoint, bool flagged_only);
    void PublishColoredViewpoint(Viewpoint &viewpoint);
    void PublishSingleFrontierCluster(Viewpoint &viewpoint);
    void PublishFrontier();

    void VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ptr);
    void RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg_ptr);
    void DisplayedNumCallback(const std_msgs::Int32::ConstPtr& msg_ptr);

    bool is_record_;  // record viewpoints if true
    bool pose_init_, last_pose_init_;

    Eigen::Vector3d last_pos_, pos_;
    Eigen::Quaterniond last_rot_, rot_;  // use Eigen::Quaternionf?

    std::vector<std::shared_ptr<Viewpoint>> viewpoint_list_;

    float frontier_color_[FRONTIER_COLOR_COUNT][3];  // used to visualize frontier clusters
    int displayed_viewpoint_num_;

    ros::NodeHandle n_;

    ros::Publisher voxelized_points_pub_;
    ros::Publisher inverted_cloud_pub_;
    ros::Publisher convex_hull_pub_;
    ros::Publisher colored_convex_hull_pub_;
    ros::Publisher viewpoint_pub_;
    ros::Publisher colored_viewpoint_pub_;
    ros::Publisher single_frontier_cluster_list_pub_;
    ros::Publisher frontier_pub_;

    ros::Subscriber voxelized_points_sub_;
    ros::Subscriber mav_pose_sub_;
    ros::Subscriber record_command_sub_;
    ros::Subscriber displayed_num_sub_;

    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
    tf2_ros::Buffer tf_buffer_;

    std::unique_ptr<ViewpointGenerator> viewpoint_generator_ptr_;
};

} // namespace local_explorer

#endif
