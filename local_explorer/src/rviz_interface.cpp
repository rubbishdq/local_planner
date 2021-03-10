#include "rviz_interface.h"

namespace local_explorer
{

RvizInterfaceNode::RvizInterfaceNode()
{
    voxelized_points_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/rviz_interface", 1);

    voxelized_points_sub_ = n_.subscribe("/iris/global_mapper_ros/voxelized_points", 1, &RvizInterfaceNode::VoxelizedPointsCallback, this, ros::TransportHints().tcpNoDelay());  

    ROS_INFO("Rviz interface node started.");

    ros::spin();
}


void RvizInterfaceNode::RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (auto point : msg_ptr->points)
    {
        cloud.push_back(pcl::PointXYZ(point.mu[0], point.mu[1], point.mu[2]));
    }
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    voxelized_points_pub_.publish(cloud_msg);
}

void RvizInterfaceNode::VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    RepublishVoxelizedPoints(msg_ptr);
}

} // namespace local_explorer

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_interface_node");
    local_explorer::RvizInterfaceNode rfn;
    return 0;
}
