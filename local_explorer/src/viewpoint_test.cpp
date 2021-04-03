#include "viewpoint_test.h"

namespace local_explorer
{
ViewpointTest::ViewpointTest()
{
    point_ << 0, 0, 0;

    voxelized_points_pub_ = n_.advertise<sensor_msgs::PointCloud2>("viewpoint_test/voxelized_pointcloud", 1);
    viewpoint_pub_ = n_.advertise<sensor_msgs::PointCloud2>("viewpoint_test/viewpoint", 1);
    point_pub_ = n_.advertise<geometry_msgs::PointStamped>("viewpoint_test/point_stamped", 1);

    voxelized_points_sub_ = n_.subscribe("global_mapper_ros/voxelized_points", 1, &ViewpointTest::VoxelizedPointsCallback, this, ros::TransportHints().tcpNoDelay());  
    point_sub_ = n_.subscribe("viewpoint_test/point", 1, &ViewpointTest::PointCallback, this, ros::TransportHints().tcpNoDelay());  

    publishloop_timer_ = n_.createTimer(ros::Duration(0.1), &ViewpointTest::PublishPoint, this);

    ROS_INFO("Local explorer node started.");

    ros::spin();
}

void ViewpointTest::RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (auto &point : msg_ptr->points)
    {
        cloud.push_back(pcl::PointXYZ(point.mu[0], point.mu[1], point.mu[2]));
    }
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    voxelized_points_pub_.publish(cloud_msg);
}

void ViewpointTest::PublishViewpoint(Viewpoint &viewpoint)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<std::shared_ptr<Vertex>> vertex_data = viewpoint.GetConvexHullPtr()->vertex_list_;
    for (auto &vertex : vertex_data)
    {
        cloud.push_back(pcl::PointXYZ(vertex->pos_[0], vertex->pos_[1], vertex->pos_[2]));
    }
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    viewpoint_pub_.publish(cloud_msg);
}

void ViewpointTest::VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    ROS_INFO("Voxelized points message received.");
    viewpoint_generator_ptr_ = std::unique_ptr<ViewpointGenerator>(new ViewpointGenerator());
    viewpoint_generator_ptr_->ProcessVoxelizedPoints(msg_ptr);

    if (viewpoint_generator_ptr_->IsGenerated())
    {
        std::shared_ptr<Viewpoint> viewpoint_ptr = viewpoint_generator_ptr_->GetViewpointPtr();
        ROS_INFO("Viewpoint successfully generated.");
        PublishViewpoint(*viewpoint_ptr);
        if (viewpoint_ptr->Visible(point_))
        {
            printf("Point(%f, %f, %f) visible.\n", point_[0], point_[1], point_[2]);
        }
        else
        {
            printf("Point(%f, %f, %f) not visible.\n", point_[0], point_[1], point_[2]);
        }
    }
    RepublishVoxelizedPoints(msg_ptr);
}

void ViewpointTest::PointCallback(const geometry_msgs::Point32::ConstPtr& msg_ptr)
{
    point_ << msg_ptr->x, msg_ptr->y, msg_ptr->z;
}

void ViewpointTest::PublishPoint(const ros::TimerEvent& event)
{
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.point.x = (double)point_[0];
    msg.point.y = (double)point_[1];
    msg.point.z = (double)point_[2];
    point_pub_.publish(msg);
}

} // namespace local_explorer

int main(int argc, char** argv)
{
    ros::init(argc, argv, "viewpoint_test_node");
    local_explorer::ViewpointTest vtn;
    return 0;
}
