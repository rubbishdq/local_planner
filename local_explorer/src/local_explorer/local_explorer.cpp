#include "local_explorer/local_explorer.h"

namespace local_explorer
{

LocalExplorer::LocalExplorer()
{
    srand(time(0));
    InitFrontierColor();

    is_record_ = false;

    // make_unique is a C++14 feature
    //viewpoint_generator_ptr_ = std::make_unique<ViewpointGenerator>();
    //viewpoint_generator_ptr_ = std::unique_ptr<ViewpointGenerator>(new ViewpointGenerator());

    voxelized_points_pub_ = n_.advertise<sensor_msgs::PointCloud2>("local_explorer/voxelized_pointcloud", 1);
    inverted_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("local_explorer/inverted_pointcloud", 1);
    convex_hull_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/convex_hull", 1);
    colored_convex_hull_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/colored_convex_hull", 1);
    viewpoint_pub_ = n_.advertise<sensor_msgs::PointCloud2>("local_explorer/viewpoint", 1);
    single_frontier_cluster_list_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/single_frontier_cluster_list", 1);
    frontier_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/frontier", 1);

    voxelized_points_sub_ = n_.subscribe("global_mapper_ros/voxelized_points", 1, &LocalExplorer::VoxelizedPointsCallback, this, ros::TransportHints().tcpNoDelay());  
    record_command_sub_ = n_.subscribe("record_command", 1, &LocalExplorer::RecordCommandCallback, this, ros::TransportHints().tcpNoDelay());  

    ROS_INFO("Local explorer node started.");

    ros::spin();
}

void LocalExplorer::InitFrontierColor()
{
    for (int i = 0; i < FRONTIER_COLOR_COUNT; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            frontier_color_[i][j] = float(rand()) / RAND_MAX;
        }
    }
}

void LocalExplorer::RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
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

void LocalExplorer::PublishInvertedCloud(ViewpointGenerator &viewpoint_generator)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto inverted_cloud_ptr = viewpoint_generator.GetInvertedCloudPtr();
    for (auto &point : *inverted_cloud_ptr)
    {
        cloud.push_back(pcl::PointXYZ(INVERT_CLOUD_VISUALIZE_PARAM*point.mu_[0], 
            INVERT_CLOUD_VISUALIZE_PARAM*point.mu_[1], 
            INVERT_CLOUD_VISUALIZE_PARAM*point.mu_[2]));
    }
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "iris/base_link";
    inverted_cloud_pub_.publish(cloud_msg);
}

void LocalExplorer::PublishConvexHull(ConvexHull &convex_hull)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    for (auto facet_ptr : convex_hull.facet_list_)
    {
        color_rgba.r = 1.0;
        color_rgba.g = 0.0;
        color_rgba.b = 1.0;
        color_rgba.a = MARKER_ALPHA;
        marker.colors.push_back(color_rgba);
        for (int i = 0; i < 3; i++)
        {
            auto vertex_ptr = facet_ptr->vertices_[i];
            point.x = INVERT_CLOUD_VISUALIZE_PARAM*vertex_ptr->pos_inverted_[0]; 
            point.y = INVERT_CLOUD_VISUALIZE_PARAM*vertex_ptr->pos_inverted_[1]; 
            point.z = INVERT_CLOUD_VISUALIZE_PARAM*vertex_ptr->pos_inverted_[2];
            marker.points.push_back(point);
        }
    }
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    
    marker.color.a = MARKER_ALPHA;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    convex_hull_pub_.publish(marker);
}

void LocalExplorer::PublishColoredConvexHull(ConvexHull &convex_hull)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    for (auto facet_ptr : convex_hull.facet_list_)
    {
        int color_index = rand() % FRONTIER_COLOR_COUNT;
        color_rgba.r = frontier_color_[color_index][0];
        color_rgba.g = frontier_color_[color_index][1];
        color_rgba.b = frontier_color_[color_index][2];
        color_rgba.a = MARKER_ALPHA;
        marker.colors.push_back(color_rgba);
        for (int i = 0; i < 3; i++)
        {
            auto vertex_ptr = facet_ptr->vertices_[i];
            point.x = INVERT_CLOUD_VISUALIZE_PARAM*vertex_ptr->pos_inverted_[0]; 
            point.y = INVERT_CLOUD_VISUALIZE_PARAM*vertex_ptr->pos_inverted_[1]; 
            point.z = INVERT_CLOUD_VISUALIZE_PARAM*vertex_ptr->pos_inverted_[2];
            marker.points.push_back(point);
        }
    }
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    
    marker.color.a = MARKER_ALPHA;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    colored_convex_hull_pub_.publish(marker);
}

void LocalExplorer::PublishViewpoint(Viewpoint &viewpoint)
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

void LocalExplorer::PublishSingleFrontierCluster(Viewpoint &viewpoint)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    std::vector<FrontierCluster> frontier_cluster_list = viewpoint.GetFrontierClusterList();
    ROS_INFO("FrontierClusterCount: %d", viewpoint.GetFrontierClusterCount());
    for (auto &frontier_cluster : frontier_cluster_list)
    {
        int color_index = frontier_cluster.id_ % FRONTIER_COLOR_COUNT;
        for (auto facet_ptr : frontier_cluster.facet_list_)
        {
            color_rgba.r = frontier_color_[color_index][0];
            color_rgba.g = frontier_color_[color_index][1];
            color_rgba.b = frontier_color_[color_index][2];
            color_rgba.a = MARKER_ALPHA;
            marker.colors.push_back(color_rgba);
            for (int i = 0; i < 3; i++)
            {
                auto vertex_ptr = facet_ptr->vertices_[i];
                point.x = vertex_ptr->pos_[0]; point.y = vertex_ptr->pos_[1]; point.z = vertex_ptr->pos_[2];
                marker.points.push_back(point);
            }
        }
    }
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    
    marker.color.a = MARKER_ALPHA;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    
    //marker.lifetime = ros::Duration(0.2);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    single_frontier_cluster_list_pub_.publish(marker);
}

void LocalExplorer::PublishFrontier()
{
    if (viewpoint_list_.empty())
        return;
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    ROS_INFO("Viewpoint count: %d", int(viewpoint_list_.size()));
    int k = 0;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        int color_index = k % FRONTIER_COLOR_COUNT;
        std::vector<FrontierCluster> frontier_cluster_list = viewpoint_ptr->GetFrontierClusterList();
        for (auto &frontier_cluster : frontier_cluster_list)
        {
            for (auto facet_ptr : frontier_cluster.facet_list_)
            {
                color_rgba.r = frontier_color_[color_index][0];
                color_rgba.g = frontier_color_[color_index][1];
                color_rgba.b = frontier_color_[color_index][2];
                color_rgba.a = MARKER_ALPHA;
                marker.colors.push_back(color_rgba);
                for (int i = 0; i < 3; i++)
                {
                    auto vertex_ptr = facet_ptr->vertices_[i];
                    point.x = vertex_ptr->pos_[0]; point.y = vertex_ptr->pos_[1]; point.z = vertex_ptr->pos_[2];
                    marker.points.push_back(point);
                }
            }
        }
        k++;
    }
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    
    marker.color.a = MARKER_ALPHA;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    
    //marker.lifetime = ros::Duration(0.2);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    frontier_pub_.publish(marker);
}

void LocalExplorer::VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    ROS_INFO("Voxelized points message received.");
    viewpoint_generator_ptr_ = std::unique_ptr<ViewpointGenerator>(new ViewpointGenerator());
    viewpoint_generator_ptr_->ProcessVoxelizedPoints(msg_ptr);

    if (viewpoint_generator_ptr_->IsGenerated())
    {
        std::shared_ptr<Viewpoint> viewpoint_ptr = viewpoint_generator_ptr_->GetViewpointPtr();
        ROS_INFO("Viewpoint successfully generated.");
        //PublishInvertedCloud(*viewpoint_generator_ptr_);
        //PublishConvexHull(*(viewpoint_generator_ptr_->GetViewpointPtr()->GetConvexHullPtr()));
        //PublishColoredConvexHull(*(viewpoint_generator_ptr_->GetViewpointPtr()->GetConvexHullPtr()));
        PublishViewpoint(*viewpoint_ptr);
        PublishSingleFrontierCluster(*viewpoint_ptr);

        // check frontier points visibility
        for (auto old_viewpoint_ptr : viewpoint_list_)
        {
            viewpoint_ptr->CheckVisibility(*old_viewpoint_ptr);
            old_viewpoint_ptr->CheckVisibility(*viewpoint_ptr);
        }
        if (is_record_)
            viewpoint_list_.push_back(viewpoint_ptr);
        PublishFrontier();        
    }
    RepublishVoxelizedPoints(msg_ptr);
}

void LocalExplorer::RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg_ptr)
{
    is_record_ = msg_ptr->data;
}

} // namespace local_explorer

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_explorer_node");
    local_explorer::LocalExplorer len;
    return 0;
}
