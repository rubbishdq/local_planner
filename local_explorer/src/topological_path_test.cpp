#include "topological_path_test.h"

namespace local_explorer
{

TopologicalPathTest::TopologicalPathTest()
{
    srand(time(0));
    InitFrontierColor();

    is_record_ = false;
    pose_init_ = false;
    last_pose_init_ = false;
    displayed_viewpoint_num_ = -1;
    drone_status_ = -1; // not initialized
    drone_status_updated_ = false;
    nav_state_ = NavState::REACHED_GOAL;
    is_replanning_ = false;

    // make_unique is a C++14 feature
    //viewpoint_generator_ptr_ = std::make_unique<ViewpointGenerator>();
    //viewpoint_generator_ptr_ = std::unique_ptr<ViewpointGenerator>(new ViewpointGenerator());

    tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

    voxelized_points_pub_ = n_.advertise<sensor_msgs::PointCloud2>("local_explorer/voxelized_pointcloud", 1);
    inverted_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("local_explorer/inverted_pointcloud", 1);
    convex_hull_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/convex_hull", 1);
    colored_convex_hull_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/colored_convex_hull", 1);
    viewpoint_pub_ = n_.advertise<sensor_msgs::PointCloud2>("local_explorer/viewpoint", 1);
    colored_viewpoint_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/colored_viewpoint", 1);
    single_frontier_cluster_list_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/single_frontier_cluster_list", 1);
    frontier_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/frontier", 1);
    global_nav_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("local_explorer/global_nav_goal", 1);
    local_nav_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("local_explorer/local_nav_goal", 1);
    topological_path_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/topological_path", 1);

    voxelized_points_sub_ = n_.subscribe("global_mapper_ros/voxelized_points", 1, &TopologicalPathTest::VoxelizedPointsCallback, this, ros::TransportHints().tcpNoDelay());  
    mav_pose_sub_ = n_.subscribe("pose", 1, &TopologicalPathTest::UavPoseCallback, this, ros::TransportHints().tcpNoDelay());  
    record_command_sub_ = n_.subscribe("record_command", 1, &TopologicalPathTest::RecordCommandCallback, this, ros::TransportHints().tcpNoDelay());
    replan_command_sub_ = n_.subscribe("replan_command", 1, &TopologicalPathTest::ReplanCommandCallback, this, ros::TransportHints().tcpNoDelay());
    displayed_num_sub_ = n_.subscribe("displayed_num", 1, &TopologicalPathTest::DisplayedNumCallback, this, ros::TransportHints().tcpNoDelay());
    drone_status_sub_ = n_.subscribe("faster/drone_status", 1, &TopologicalPathTest::DroneStatusCallback, this, ros::TransportHints().tcpNoDelay());

    nav_command_timer_ = n_.createTimer(ros::Duration(NAV_COMMAND_TIMEVAL), &TopologicalPathTest::NavCommandCallback, this);
    //topological_path_pub_timer_ = n_.createTimer(ros::Duration(TOPOLOGICAL_PATH_PUB_TIMEVAL), &TopologicalPathTest::PublishTopologicalPathCallback, this);

    ROS_INFO("Local explorer node started.");

    ros::spin();
}

void TopologicalPathTest::InitFrontierColor()
{
    for (int i = 0; i < FRONTIER_COLOR_COUNT; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            frontier_color_[i][j] = float(rand()) / RAND_MAX;
        }
    }
}

int TopologicalPathTest::DetermineOperation()
{
    if (!pose_init_)
        return 0;
    int operation_id; // 0: do nothing; 1: push back viewpoint; 2: erase last viewpoint data and push back viewpoint
    if (!last_pose_init_)
    {
        operation_id = 0;
        last_pose_init_ = true;
    }
    else
    {
        if (distxyd(pos_, last_pos_) > NEW_VIEWPOINT_DIST_THRESHOLD)
            operation_id = 1;
        else
        {
            Eigen::Vector3d angle_diff = EulerAngleDiff(rot_, last_rot_);
            double max_angle = 0;
            for (int i = 0; i < 3; i++)
            {
                if (fabs(angle_diff[i]) > max_angle)
                    max_angle = fabs(angle_diff[i]);
            }
            //if (distxyd(pos_, last_pos_) < UPDATE_VIEWPOINT_DIST_THRESHOLD 
            //    && max_angle > UPDATE_VIEWPOINT_ANGLE_THRESHOLD)
            if (false)
            {
                if (!viewpoint_list_.empty())
                    operation_id = 1;  // may be 2
                else
                    operation_id = 1;
            }
            else
                operation_id = 0;
        }
    }
    return operation_id;
}

void TopologicalPathTest::RemoveRedundantBoarder(Viewpoint &viewpoint, bool last_viewpoint = false)
{
    geometry_msgs::TransformStamped transform_stamped;
    Eigen::Vector3f pos_cam;
    Eigen::Quaternionf rot_cam;
    int erase_count = 0;

    try
    {
        transform_stamped = tf_buffer_.lookupTransform("world", "iris/base_link", ros::Time(0), ros::Duration(0.02)); // TODO: use /iris/camera
        pos_cam << (float)transform_stamped.transform.translation.x,
            (float)transform_stamped.transform.translation.y,
            (float)transform_stamped.transform.translation.z;
        rot_cam = Eigen::Quaternionf((float)transform_stamped.transform.rotation.w, 
            (float)transform_stamped.transform.rotation.x, 
            (float)transform_stamped.transform.rotation.y, 
            (float)transform_stamped.transform.rotation.z);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("[world_database_master_ros] OnGetTransform failed with %s", ex.what());
        return;
    }

    int ind = 0;
    for (auto viewpoint_iter = viewpoint_list_.begin(); viewpoint_iter != viewpoint_list_.end(); viewpoint_iter++)
    {
        if (!last_viewpoint && ind == int(viewpoint_list_.size())-1)
        {
            break;
        }
        auto old_viewpoint_ptr = *viewpoint_iter;
        Eigen::Vector3f old_origin = old_viewpoint_ptr->GetOrigin();
        printf("%d. (%f, %f, %f) ", ind, old_origin[0], old_origin[1], old_origin[2]);
        if (!viewpoint.Visible(old_origin))
        {
            printf("not visible\n");
            ind++;
            continue;
        }
        else
        {
            printf("visible\n");
        }
        for (auto &fc : old_viewpoint_ptr->frontier_cluster_list_)
        {
            auto facet_iter = fc.facet_list_.begin();
            while (facet_iter != fc.facet_list_.end())
            {
                bool is_frontier = false;
                for (auto vertex_ptr : (*facet_iter)->vertices_)
                {
                    if (vertex_ptr->flag_ == 2)
                    {
                        Eigen::Vector3f pos_new = rot_cam.conjugate()*(vertex_ptr->pos_-pos_cam);
                        if (InCameraRange(pos_new))
                        {
                            vertex_ptr->flag_ = 0;
                        }
                    }
                    if (vertex_ptr->flag_ != 0)
                    {
                        is_frontier = true;
                    }
                }
                if (!is_frontier)
                {
                    facet_iter = fc.facet_list_.erase(facet_iter);
                    erase_count++;
                }
                else
                {
                    facet_iter++;
                }
            }
        }
        ind++;
    }
    printf("Erased %d frontier facets in TopologicalPathTest::RemoveRedundantBoarder().\n", erase_count);
}

void TopologicalPathTest::ProcessNewViewpoint(std::shared_ptr<Viewpoint> viewpoint_ptr)
{
    // if is_record_ == false, only execute CheckVisibility() and RemoveRedundantBoarder()
    int operation_id = DetermineOperation();
    if (operation_id != 0)
    {
        if (is_record_)
        {
            if (operation_id == 2)
            {
                auto iter = viewpoint_list_.end();
                iter--;
                viewpoint_list_.erase(iter);
            }
        }
        // check frontier points visibility
        for (auto old_viewpoint_ptr : viewpoint_list_)
        {
            viewpoint_ptr->CheckVisibility(*old_viewpoint_ptr);
            old_viewpoint_ptr->CheckVisibility(*viewpoint_ptr);
        }
        RemoveRedundantBoarder(*viewpoint_ptr, true);
    }
    else
    {
        RemoveRedundantBoarder(*viewpoint_ptr, false);
    }
    if (is_record_)
    {
        if (operation_id != 0)
        {
            UpdateTopologicalMap(viewpoint_ptr);
            viewpoint_list_.push_back(viewpoint_ptr);
            last_pos_ = pos_;
            last_rot_ = rot_;
            //viewpoint_list_[0]->PrintFrontierData(0);
            ROS_INFO("Operation ID: %d", operation_id);
        }
    }
}

bool TopologicalPathTest::Replan()
{
    Eigen::Vector3f current_pos;
    current_pos << (float)pos_[0], pos_[1], pos_[2];
    std::shared_ptr<Viewpoint> start, end;
    end = viewpoint_list_[0];
    if (!GetNearestViewpoint(current_pos, start))
    {
        ROS_INFO("No viewpoints found.");
        return false;
    }
    std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
    topological_path_ = GetTopologicalPath(start, end);
    is_replanning_ = false;
    return true;
}

void TopologicalPathTest::UpdateTopologicalMap(std::shared_ptr<Viewpoint> viewpoint_ptr)
{
    int ind = 0, count = 0;
    for (auto old_viewpoint_ptr : viewpoint_list_)
    {
        if (ind == int(viewpoint_list_.size())-1 ||
            (viewpoint_ptr->Distance(*old_viewpoint_ptr) <= SENSOR_RANGE && viewpoint_ptr->Visible(old_viewpoint_ptr->GetOrigin())))
        {
            viewpoint_ptr->AddNeighbor(old_viewpoint_ptr);
            old_viewpoint_ptr->AddNeighbor(viewpoint_ptr);
            count++;
        }
        ind++;
    }
    ROS_INFO("New viewpoint connected with %d viewpoints.", count);
}

// Dijkstra algorithm
std::deque<std::shared_ptr<Viewpoint>> TopologicalPathTest::GetTopologicalPath(
    std::shared_ptr<Viewpoint> start, std::shared_ptr<Viewpoint> end)
{
    typedef std::pair<float, std::shared_ptr<Viewpoint>> q_ele;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        viewpoint_ptr->InitDijkstraData();
    }
    start->dist_ = 0;
    std::priority_queue<q_ele, std::vector<q_ele>, std::greater<q_ele>> pq;
    pq.push(q_ele(start->dist_, start));

    std::shared_ptr<Viewpoint> node_ptr;
    while (!pq.empty())
    {
        node_ptr = pq.top().second;
        pq.pop();
        if (node_ptr == end)
        {
            break;
        }
        if (node_ptr->is_visited_)
        {
            continue;
        }
        node_ptr->is_visited_ = true;
        // update distance
        for (NeighborViewpoint &neighbor : node_ptr->neighbor_list_)
        {
            auto neighbor_ptr = neighbor.viewpoint_ptr_.lock();
            if (neighbor_ptr->is_visited_)
            {
                continue;
            }
            if (neighbor_ptr->dist_ > node_ptr->dist_ + neighbor.dist_+DIJKSTRA_PENALTY)
            {
                neighbor_ptr->dist_ = node_ptr->dist_ + neighbor.dist_+DIJKSTRA_PENALTY;
                neighbor_ptr->last_viewpoint_ = node_ptr;
                pq.push(q_ele(neighbor_ptr->dist_, neighbor_ptr));
            }
        }
    }
    
    // read topological path
    std::deque<std::shared_ptr<Viewpoint>> path;
    node_ptr = end;
    while (node_ptr != start)
    {
        path.push_back(node_ptr);
        node_ptr = node_ptr->last_viewpoint_.lock();
    }
    path.push_back(start);
    return path;
}

bool TopologicalPathTest::GetNearestViewpoint(Eigen::Vector3f pos, std::shared_ptr<Viewpoint>& vptr)
{
    float min_dist = FLT_MAX;
    std::shared_ptr<Viewpoint> nearest_viewpoint_ptr;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        if (!viewpoint_ptr->Visible(pos))
            continue;
        Eigen::Vector3f origin = viewpoint_ptr->GetOrigin();
        float dist = (origin-pos).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_viewpoint_ptr = viewpoint_ptr;
        }
    }
    vptr = nearest_viewpoint_ptr;
    if (nearest_viewpoint_ptr)
        return true;
    else
        return false;
}

// naive strategy for selecting next frontier to navigate to
bool TopologicalPathTest::GetNearestFrontierCluster(Eigen::Vector3f pos, FrontierCluster*& fc_ptr)
{
    float min_dist = FLT_MAX;
    FrontierCluster* nearest_fc_ptr = nullptr;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        for (auto &fc : viewpoint_ptr->frontier_cluster_list_)
        {
            if (fc.IsEmpty())
            {
                continue;
            }
            float dist = (fc.GetCenter()-pos).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_fc_ptr = &fc;
            }
        }
    }
    fc_ptr = nearest_fc_ptr;
    return !(nearest_fc_ptr == nullptr);
}

bool TopologicalPathTest::GetNearestFrontierCluster(Eigen::Vector3f pos, FrontierCluster*& fc_ptr, std::shared_ptr<Viewpoint>& vptr)
{
    float min_dist = FLT_MAX;
    FrontierCluster* nearest_fc_ptr = nullptr;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        for (auto &fc : viewpoint_ptr->frontier_cluster_list_)
        {
            if (fc.IsEmpty())
            {
                continue;
            }
            float dist = (fc.GetCenter()-pos).norm();
            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_fc_ptr = &fc;
                vptr = viewpoint_ptr;
            }
        }
    }
    fc_ptr = nearest_fc_ptr;
    return !(nearest_fc_ptr == nullptr);
}

void TopologicalPathTest::RepublishVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
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

void TopologicalPathTest::PublishInvertedCloud(ViewpointGenerator &viewpoint_generator)
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

void TopologicalPathTest::PublishConvexHull(ConvexHull &convex_hull)
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

void TopologicalPathTest::PublishColoredConvexHull(ConvexHull &convex_hull)
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

void TopologicalPathTest::PublishViewpoint(Viewpoint &viewpoint, bool flagged_only = false)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<std::shared_ptr<Vertex>> vertex_data = viewpoint.GetConvexHullPtr()->vertex_list_;
    for (auto &vertex : vertex_data)
    {
        if (!flagged_only || vertex->flag_)
            cloud.push_back(pcl::PointXYZ(vertex->pos_[0], vertex->pos_[1], vertex->pos_[2]));
    }
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    viewpoint_pub_.publish(cloud_msg);
}

void TopologicalPathTest::PublishColoredViewpoint(Viewpoint &viewpoint)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    std::vector<std::shared_ptr<Vertex>> vertex_data = viewpoint.GetConvexHullPtr()->vertex_list_;
    for (auto &vertex : vertex_data)
    {
        point.x = vertex->pos_[0]; point.y = vertex->pos_[1]; point.z = vertex->pos_[2];
        switch(vertex->flag_)
        {
            case 0:
                color_rgba.r = 1;
                color_rgba.g = 0;
                color_rgba.b = 0;
                color_rgba.a = MARKER_ALPHA;
                break;
            case 1:
                color_rgba.r = 0;
                color_rgba.g = 1;
                color_rgba.b = 0;
                color_rgba.a = MARKER_ALPHA;
                break;
            case 2:
                color_rgba.r = 1;
                color_rgba.g = 1;
                color_rgba.b = 1;
                color_rgba.a = MARKER_ALPHA;
                break;
            default:
                break;
        }
        marker.points.push_back(point);
        marker.colors.push_back(color_rgba);
    }
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 1;
    
    marker.color.a = MARKER_ALPHA;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    
    //marker.lifetime = ros::Duration(0.2);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    colored_viewpoint_pub_.publish(marker);
}

void TopologicalPathTest::PublishSingleFrontierCluster(Viewpoint &viewpoint)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    std::vector<FrontierCluster> frontier_cluster_list = viewpoint.GetFrontierClusterList();
    //ROS_INFO("FrontierClusterCount: %d", viewpoint.GetFrontierClusterCount());
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

void TopologicalPathTest::PublishFrontier()
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

void TopologicalPathTest::PublishGlobalNavGoal(Eigen::Vector3f pos, Eigen::Quaterniond rot)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = double(pos[0]);
    msg.pose.position.y = double(pos[1]);
    msg.pose.position.z = double(pos[2]);
    msg.pose.orientation.x = rot.x();
    msg.pose.orientation.y = rot.y();
    msg.pose.orientation.z = rot.z();
    msg.pose.orientation.w = rot.w();
    global_nav_goal_pub_.publish(msg);
}

void TopologicalPathTest::PublishLocalNavGoal(Eigen::Vector3f pos, Eigen::Quaterniond rot)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = double(pos[0]);
    msg.pose.position.y = double(pos[1]);
    msg.pose.position.z = double(pos[2]);
    msg.pose.orientation.x = rot.x();
    msg.pose.orientation.y = rot.y();
    msg.pose.orientation.z = rot.z();
    msg.pose.orientation.w = rot.w();
    local_nav_goal_pub_.publish(msg);
}

void TopologicalPathTest::PublishTopologicalPath()
{
    std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
    if (topological_path_.empty())
    {
        ROS_WARN("Topological path empty.");
        return;
    }
    else
    {
        ROS_INFO("Publishing topological path.");
    }
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    int path_length = int(topological_path_.size());
    ROS_INFO("Topological path size: %d", path_length);
    for (int k = 0; k < path_length; k++)
    {
        Eigen::Vector3f path_node;
        path_node = topological_path_[k]->GetOrigin();
        point.x = path_node[0]; point.y = path_node[1]; point.z = path_node[2];
        marker.points.push_back(point);
    }

    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    
    marker.color.a = MARKER_ALPHA;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    
    //marker.lifetime = ros::Duration(0.2);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    topological_path_pub_.publish(marker);
}

void TopologicalPathTest::VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    //ROS_INFO("Voxelized points message received.");
    viewpoint_generator_ptr_ = std::unique_ptr<ViewpointGenerator>(new ViewpointGenerator());
    viewpoint_generator_ptr_->ProcessVoxelizedPoints(msg_ptr);

    if (viewpoint_generator_ptr_->IsGenerated())
    {
        std::shared_ptr<Viewpoint> viewpoint_ptr = viewpoint_generator_ptr_->GetViewpointPtr();
        std::shared_ptr<Viewpoint> displayed_viewpoint_ptr;
        //ROS_INFO("Viewpoint successfully generated.");
        PublishInvertedCloud(*viewpoint_generator_ptr_);
        PublishConvexHull(*(viewpoint_generator_ptr_->GetViewpointPtr()->GetConvexHullPtr()));
        PublishColoredConvexHull(*(viewpoint_generator_ptr_->GetViewpointPtr()->GetConvexHullPtr()));
        PublishViewpoint(*viewpoint_ptr);
        PublishSingleFrontierCluster(*viewpoint_ptr);

        if (displayed_viewpoint_num_ >= 0 && displayed_viewpoint_num_ < int(viewpoint_list_.size()))
        {
            PublishColoredViewpoint(*viewpoint_list_[displayed_viewpoint_num_]);
        }

        ProcessNewViewpoint(viewpoint_ptr);

        PublishFrontier();
    }
    RepublishVoxelizedPoints(msg_ptr);
}

void TopologicalPathTest::UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ptr)
{
    /*
    pos_ << (float)msg_ptr->pose.position.x, (float)msg_ptr->pose.position.y, (float)msg_ptr->pose.position.z;
    rot_ = Eigen::Quaternionf((float)msg_ptr->pose.orientation.w, (float)msg_ptr->pose.orientation.x, 
        (float)msg_ptr->pose.orientation.y, (float)msg_ptr->pose.orientation.z);
    */
    pos_ << msg_ptr->pose.position.x, msg_ptr->pose.position.y, msg_ptr->pose.position.z;
    rot_ = Eigen::Quaterniond(msg_ptr->pose.orientation.w, msg_ptr->pose.orientation.x, msg_ptr->pose.orientation.y, 
        msg_ptr->pose.orientation.z);
    pose_init_ = true;
}

void TopologicalPathTest::RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg_ptr)
{
    is_record_ = msg_ptr->data;
}

void TopologicalPathTest::ReplanCommandCallback(const std_msgs::Bool::ConstPtr& msg_ptr)
{
    is_replanning_ = msg_ptr->data;
}

void TopologicalPathTest::DisplayedNumCallback(const std_msgs::Int32::ConstPtr& msg_ptr)
{
    displayed_viewpoint_num_ = msg_ptr->data;
}

void TopologicalPathTest::DroneStatusCallback(const std_msgs::Int32::ConstPtr& msg_ptr)
{
    drone_status_ = msg_ptr->data;
    drone_status_updated_ = true;
}

// naive strategy
void TopologicalPathTest::NavCommandCallback(const ros::TimerEvent& event)
{
    if (!is_replanning_)
    {
        return;
    }
    Eigen::Vector3f current_pos;
    current_pos << (float)pos_[0], (float)pos_[1], (float)pos_[2];
    switch (nav_state_)
    {
        case NavState::NAV_IN_PATH:
        {
            ROS_INFO("Current state: NAV_IN_PATH");
            if (drone_status_ == 3 && drone_status_updated_)  // REACHED_GOAL in faster
            {
                std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
                if (topological_path_.empty())
                {
                    nav_state_ = NavState::REACHED_GOAL;
                }
                else
                {
                    auto next_vptr = topological_path_.front();
                    topological_path_.pop_front();
                    goal_pos_ = next_vptr->GetOrigin();
                    goal_rot_ = DirectionQuatHorizonal(current_pos, goal_pos_);
                    PublishLocalNavGoal(goal_pos_, goal_rot_);
                }
                drone_status_updated_ = false;
            }
            break;
        }
        case NavState::REACHED_GOAL:
        {
            ROS_INFO("Current state: REACHED_GOAL");
            // try replanning
            if (Replan())
            {
                PublishGlobalNavGoal(viewpoint_list_[0]->GetOrigin(), DirectionQuatHorizonal(current_pos, viewpoint_list_[0]->GetOrigin()));
                nav_state_ = NavState::NAV_IN_PATH;
                PublishTopologicalPath();
                auto next_vptr = topological_path_.front();
                topological_path_.pop_front();
                goal_pos_ = next_vptr->GetOrigin();
                goal_rot_ = DirectionQuatHorizonal(current_pos, goal_pos_);
                PublishLocalNavGoal(goal_pos_, goal_rot_);
                //drone_status_updated_ = false;
            }
        }
            break;
        default:
            break;
    }
}

void TopologicalPathTest::PublishTopologicalPathCallback(const ros::TimerEvent& event)
{
    PublishTopologicalPath();
}

} // namespace local_explorer

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_explorer_node");
    local_explorer::TopologicalPathTest len;
    return 0;
}
