#include "pcd_publisher.h"

using namespace std;

PCDPublisherNode::PCDPublisherNode() : cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
	//pcl::io::loadPCDFile("/home/dqs/pcd/complex_room.pcd", *cloud_);
    pcl::io::loadPCDFile("/home/dqs/pcd/sparse_pillars.pcd", *cloud_);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < cloud_->size(); i++)
    {
        pcl::PointXYZ pt(cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z);
        double z_max = 2.49, z_min = 0.51;
        if (pt.z > z_max || pt.z < z_min)
        {
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud_);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_);

    pointcloudPub_ = n_.advertise<sensor_msgs::PointCloud2>("global_map", 1);
    publish_timer_ = n_.createTimer(ros::Duration(1.0), &PCDPublisherNode::PublishPointCloud, this);
}

void PCDPublisherNode::PublishPointCloud(const ros::TimerEvent& event)
{  
    sendTFData();

    sensor_msgs::PointCloud2 pointcloud;
    pcl::toROSMsg(*cloud_, pointcloud);
    pointcloud.header.frame_id = "map";
    pointcloudPub_.publish(pointcloud);
}

void PCDPublisherNode::sendTFData()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0,0,0));
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_publisher_node");

    PCDPublisherNode ppn;

    if(!ros::ok())
    {
	    return 0;
    }
    ROS_INFO("PCD publisher node set up already.");
    ros::spin();

    return 0;
}
