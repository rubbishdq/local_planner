#include "pcd_saver.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

using namespace std;

PCDSaverNode::PCDSaverNode()
{
    ready_to_save_ = false;

    pointcloudSub_ = n_.subscribe("point_cloud", 1, &PCDSaverNode::pointcloudCallback, this, ros::TransportHints().tcpNoDelay());  
    savecommandSub_ = n_.subscribe("save_command", 1, &PCDSaverNode::savecommandCallback, this, ros::TransportHints().tcpNoDelay());  
}

void PCDSaverNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (ready_to_save_)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ> pcl_pc;
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc);
        pcl::io::savePCDFileASCII ("/home/dqs/pcd/complex_room.pcd", pcl_pc);
    }
}

void PCDSaverNode::savecommandCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ready_to_save_ = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core_controller_node");

    PCDSaverNode psn;

    if(!ros::ok())
    {
	    return 0;
    }
    ROS_INFO("flight controller node set up already.");
    ros::spin();

    return 0;
}
