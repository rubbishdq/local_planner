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
#include <thread>
#include <mutex>
#include <vector>
#include <list>
#include <queue>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <stdarg.h>

namespace local_explorer
{

enum NavState
{
    NAV_IN_PATH, NAV_TO_LOCAL_FRONTIER, REACHED_GOAL
};

class LocalExplorer
{
public:
    LocalExplorer();

private:
    void InitFrontierColor();
    int DetermineOperation();
    void RemoveRedundantBoarder(Viewpoint &viewpoint, bool last_viewpoint); // if last_viewpoint is false, last added viewpoint's boarder will not be removed
    void ProcessNewViewpoint(std::shared_ptr<Viewpoint> viewpoint_ptr);
    bool Replan();
    void UpdateTopologicalMap(std::shared_ptr<Viewpoint> viewpoint_ptr); // viewpoint_ptr should not be in this->viewpoint_list_
    std::deque<std::shared_ptr<Viewpoint>> GetTopologicalPath(
        std::shared_ptr<Viewpoint> start, std::shared_ptr<Viewpoint> end, float* cost);
    bool GetNearestViewpoint(Eigen::Vector3f pos, std::shared_ptr<Viewpoint>& viewpoint_ptr);  // return false if no available viewpoints found
    bool GetNearestFrontierCluster(Eigen::Vector3f pos, FrontierCluster*& fc_ptr);  // return false if no available frontiers found
    bool GetNearestFrontierCluster(Eigen::Vector3f pos, FrontierCluster*& fc_ptr, std::shared_ptr<Viewpoint>& vptr);  // also gets the viewpoint this fc belongs to

    void RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void PublishInvertedCloud(ViewpointGenerator &viewpoint_generator);
    void PublishConvexHull(ConvexHull &convex_hull);  // inverted
    void PublishColoredConvexHull(ConvexHull &convex_hull);  // inverted
    void PublishViewpoint(Viewpoint &viewpoint, bool flagged_only);
    void PublishColoredViewpoint(Viewpoint &viewpoint);
    void PublishKdTreeRT(Viewpoint &viewpoint);
    void PublishSingleFrontierCluster(FrontierCluster &fc);
    void PublishSingleViewpointFrontier(Viewpoint &viewpoint);
    void PublishFrontier();
    void PublishGlobalNavGoal(Eigen::Vector3f pos, Eigen::Quaterniond rot);
    void PublishLocalNavGoal(Eigen::Vector3f pos, Eigen::Quaterniond rot);
    void PublishTopologicalPath();

    void VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);
    void UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ptr);
    void RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg_ptr);
    void DisplayedNumCallback(const std_msgs::Int32::ConstPtr& msg_ptr);
    void DroneStatusCallback(const std_msgs::Int32::ConstPtr& msg_ptr);

    void NavCommandCallback(const ros::TimerEvent& event);
    void PublishTopologicalPathCallback(const ros::TimerEvent& event);

    bool is_record_;  // record viewpoints if true
    bool pose_init_, last_pose_init_;
    int drone_status_;
    bool drone_status_updated_;
    NavState nav_state_;

    std::deque<std::shared_ptr<Viewpoint>> topological_path_;

    Eigen::Vector3d last_pos_, pos_;
    Eigen::Vector3f goal_pos_;
    Eigen::Quaterniond last_rot_, rot_, goal_rot_;  // use Eigen::Quaternionf?

    std::vector<std::shared_ptr<Viewpoint>> viewpoint_list_;
    FrontierCluster* target_fc_;
    bool navigated_to_target_viewpoint_;

    float frontier_color_[FRONTIER_COLOR_COUNT][3];  // used to visualize frontier clusters
    int displayed_viewpoint_num_;

    ros::NodeHandle n_;

    ros::Publisher voxelized_points_pub_;
    ros::Publisher inverted_cloud_pub_;
    ros::Publisher convex_hull_pub_;
    ros::Publisher colored_convex_hull_pub_;
    ros::Publisher viewpoint_pub_;
    ros::Publisher colored_viewpoint_pub_;
    ros::Publisher kd_tree_rt_pub_;
    ros::Publisher single_frontier_cluster_pub_;
    ros::Publisher single_viewpoint_frontier_pub_;
    ros::Publisher frontier_pub_;
    ros::Publisher global_nav_goal_pub_;
    ros::Publisher local_nav_goal_pub_;
    ros::Publisher topological_path_pub_;

    ros::Subscriber voxelized_points_sub_;
    ros::Subscriber mav_pose_sub_;
    ros::Subscriber record_command_sub_;
    ros::Subscriber displayed_num_sub_;
    ros::Subscriber drone_status_sub_;

    ros::Timer nav_command_timer_;
    ros::Timer topological_path_pub_timer_;

    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
    tf2_ros::Buffer tf_buffer_;

    std::unique_ptr<ViewpointGenerator> viewpoint_generator_ptr_;

    std::mutex topological_path_mutex_;
};

} // namespace local_explorer

#endif
