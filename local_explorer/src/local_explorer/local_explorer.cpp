#include "local_explorer/local_explorer.h"

namespace local_explorer
{

LocalExplorer::LocalExplorer()
{
    srand(time(0));
    InitFrontierColor();

    is_record_ = false;
    pose_init_ = false;
    last_pose_init_ = false;
    displayed_viewpoint_num_ = -1;
    drone_status_ = -1; // not initialized
    drone_status_updated_ = false;
    faster_nav_status_ = -1; // not initialized
    faster_nav_status_updated_ = false;
    target_fc_ = nullptr;
    nav_state_ = NavState::REACHED_GOAL;
    navigated_to_target_viewpoint_ = false;
    is_le_timer_on_ = false;

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
    kd_tree_rt_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/kd_tree_rt", 1);
    single_frontier_cluster_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/single_frontier_cluster", 1);
    single_viewpoint_frontier_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/single_viewpoint_frontier", 1);
    frontier_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/frontier", 1);
    global_nav_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("local_explorer/global_nav_goal", 1);
    local_nav_goal_pub_ = n_.advertise<geometry_msgs::PoseStamped>("local_explorer/local_nav_goal", 1);
    topological_path_pub_ = n_.advertise<visualization_msgs::Marker>("local_explorer/topological_path", 1);

    voxelized_points_sub_ = n_.subscribe("global_mapper_ros/voxelized_points", 1, &LocalExplorer::VoxelizedPointsCallback, this, ros::TransportHints().tcpNoDelay());  
    //mav_pose_sub_ = n_.subscribe("pose", 1, &LocalExplorer::UavPoseCallback, this, ros::TransportHints().tcpNoDelay());  
    mav_state_sub_ = n_.subscribe("state", 1, &LocalExplorer::UavStateCallback, this, ros::TransportHints().tcpNoDelay());  
    record_command_sub_ = n_.subscribe("record_command", 1, &LocalExplorer::RecordCommandCallback, this, ros::TransportHints().tcpNoDelay());
    displayed_num_sub_ = n_.subscribe("displayed_num", 1, &LocalExplorer::DisplayedNumCallback, this, ros::TransportHints().tcpNoDelay());
    drone_status_sub_ = n_.subscribe("faster/drone_status", 1, &LocalExplorer::DroneStatusCallback, this, ros::TransportHints().tcpNoDelay());
    faster_nav_status_sub_ = n_.subscribe("faster/nav_status", 1, &LocalExplorer::FasterNavStatusCallback, this, ros::TransportHints().tcpNoDelay());

    nav_command_timer_ = n_.createTimer(ros::Duration(NAV_COMMAND_TIMEVAL), &LocalExplorer::NavCommandCallback, this);
    //topological_path_pub_timer_ = n_.createTimer(ros::Duration(TOPOLOGICAL_PATH_PUB_TIMEVAL), &LocalExplorer::PublishTopologicalPathCallback, this);

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

int LocalExplorer::DetermineOperation(std::shared_ptr<Viewpoint> viewpoint_ptr)
{
    if (!pose_init_)
        return 0;
    //Eigen::Vector3f current_pos(float(pos_[0]), float(pos_[1]), float(pos_[2]));
    int operation_id; // 0: do nothing; 1: push back viewpoint; 2: erase last viewpoint data and push back viewpoint
    if (!last_pose_init_)
    {
        operation_id = 0;
        last_pose_init_ = true;
    }
    else if (viewpoint_ptr->IsCloseToBoarder(viewpoint_ptr->GetOrigin()))
    {
        operation_id = 0;
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

void LocalExplorer::RemoveRedundantBoarder(Viewpoint &viewpoint, bool last_viewpoint = false)
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

    // strategy 1: remove a boarder frontier if it is in camera's FoV && current position is visible by the viewpoint it belongs to 
    // (to ensure that current position is not too far from this viewpoint)
    // to naive, often remove frontiers that should not be removed
    /*
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
                    fc.RemoveFacet(facet_iter);
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
    */

   // strategy 2: use current viewpoint's kd_tree_rt_ptr to check if there is any obstacle between an old viewpoint and its boarder frontiers
   // if so, remove the boarder frontier that is covered by obstacles
    int ind = 0;
    for (auto viewpoint_iter = viewpoint_list_.begin(); viewpoint_iter != viewpoint_list_.end(); viewpoint_iter++)
    {
        if (!last_viewpoint && ind == int(viewpoint_list_.size())-1)
        {
            break;
        }
        auto old_viewpoint_ptr = *viewpoint_iter;
        Eigen::Vector3f old_origin = old_viewpoint_ptr->GetOrigin();
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
                        if (viewpoint.IntersectWithObstacle(old_origin, vertex_ptr->pos_))
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
                    fc.RemoveFacet(facet_iter);
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

    if (erase_count > 0)
        printf("Erased %d frontier facets in LocalExplorer::RemoveRedundantBoarder().\n", erase_count);
}

void LocalExplorer::ProcessNewViewpoint(std::shared_ptr<Viewpoint> viewpoint_ptr)
{
    // if is_record_ == false, only execute CheckVisibility() and RemoveRedundantBoarder()
    int operation_id = DetermineOperation(viewpoint_ptr);
    if (operation_id != 0)
    {
        if (is_record_)
        {
            if (operation_id == 2)
            {
                auto iter = viewpoint_list_.end();
                iter--;
                (*iter)->ResetNeighbor();
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
        //RemoveRedundantBoarder(*viewpoint_ptr, false);
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
    // remove small frontier clusters
    for (auto old_viewpoint_ptr : viewpoint_list_)  // Note: viewpoint_ptr has been added to viewpoint_list_
    {
        old_viewpoint_ptr->RemoveSmallFrontierCluster(MIN_FRONTIER_CLUSTER_AREA);  // parameter used in this function can be better adjusted
    }
    for (auto old_viewpoint_ptr : viewpoint_list_)
    {
        old_viewpoint_ptr->RecalcFrontierClusterRange();
    }
}

ReplanResult LocalExplorer::Replan(Eigen::Vector3f current_pos, Eigen::Vector3f current_vel, Eigen::Quaterniond current_rot)
{
    ReplanResult result;
    FrontierCluster* fc_ptr = nullptr;
    std::shared_ptr<Viewpoint> start, end;
    if (!GetNearestViewpoint(current_pos, start))
    {
        ROS_INFO("No viewpoints found.");
        result.value = 0;
        return result;
    }
    if (!GetNextFrontierCluster(current_pos, current_vel, current_rot, fc_ptr, end))
    {
        ROS_INFO("No new frontier clusters found.");
        result.value = 0;
        return result;
    }
    target_fc_ = fc_ptr;
    target_viewpoint_ = end;
    // if current position is visible by "end", navigate to target_fc directly
    if (end->Visible(current_pos))
    {
        result.value = 1;
        result.common_vptr = end;
        return result;
    }
    std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
    topological_path_ = GetTopologicalPath(start, end, nullptr);
    navigated_to_target_viewpoint_ = false;
    // result.value = 2;
    result.value = 1;
    result.common_vptr = end;
    return result;
}

void LocalExplorer::UpdateTopologicalMap(std::shared_ptr<Viewpoint> viewpoint_ptr)
{
    int ind = 0, count = 0;
    for (auto old_viewpoint_ptr : viewpoint_list_)
    {
        // connect new viewpoint with last viewpoint in the array
        // for other old viewpoints, connect if they are visible to each other
        if (ind == int(viewpoint_list_.size())-1 ||
            (viewpoint_ptr->Distance(*old_viewpoint_ptr) <= SENSOR_RANGE && 
            viewpoint_ptr->Visible(old_viewpoint_ptr->GetOrigin()) &&
            old_viewpoint_ptr->Visible(viewpoint_ptr->GetOrigin())))
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
std::deque<std::shared_ptr<Viewpoint>> LocalExplorer::GetTopologicalPath(
    std::shared_ptr<Viewpoint> start, std::shared_ptr<Viewpoint> end, float* cost)
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
            if (cost != nullptr)
            {
                *cost = node_ptr->dist_;
            }
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
        path.push_front(node_ptr);
        node_ptr = node_ptr->last_viewpoint_.lock();
    }
    path.push_front(start);
    return path;
}

bool LocalExplorer::GetNearestViewpoint(Eigen::Vector3f pos, std::shared_ptr<Viewpoint>& vptr)
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
bool LocalExplorer::GetNearestFrontierCluster(Eigen::Vector3f pos, FrontierCluster*& fc_ptr)
{
    int empty_count = 0;
    float min_dist = FLT_MAX;
    FrontierCluster* nearest_fc_ptr = nullptr;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        for (auto &fc : viewpoint_ptr->frontier_cluster_list_)
        {
            if (fc.IsEmpty())
            {
                empty_count++;
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
    ROS_INFO("Empty frontier cluster count: %d", empty_count);
    fc_ptr = nearest_fc_ptr;
    return !(nearest_fc_ptr == nullptr);
}

bool LocalExplorer::GetNearestFrontierCluster(Eigen::Vector3f pos, FrontierCluster*& fc_ptr, std::shared_ptr<Viewpoint>& vptr)
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

bool LocalExplorer::GetNextFrontierCluster(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Quaterniond rot, FrontierCluster*& fc_ptr, std::shared_ptr<Viewpoint>& vptr)
{
    // Strategy 1: find nearest frontier cluster
    //return GetNearestFrontierCluster(pos, fc_ptr, vptr);

    // Strategy 2:depending on UAV's current position, rotation and velocity, as well as some properties of frontier clusters
    float max_score = -FLT_MAX;
    float max_score_a = -FLT_MAX, max_score_d = -FLT_MAX, max_score_v = -FLT_MAX, max_score_r = -FLT_MAX, max_score_bonus = -FLT_MAX;
    FrontierCluster* next_fc_ptr = nullptr;
    for (auto viewpoint_ptr : viewpoint_list_)
    {
        bool is_visible = viewpoint_ptr->Visible(pos);
        for (auto &fc : viewpoint_ptr->frontier_cluster_list_)
        {
            if (fc.IsEmpty())
            {
                continue;
            }
            float score = 0;
            float score_a, score_d, score_v, score_r, score_bonus;
            score_a = NEXTFC_K_A * std::min(fc.area_, NEXTFC_MAX_AREA);
            Eigen::Vector3f pos_diff = fc.GetCenter()-pos;
            score_d = NEXTFC_K_D * std::max(pos_diff.norm(), NEXTFC_MIN_DIST);
            Eigen::Vector3f pos_diff_unit = pos_diff / pos_diff.norm();
            score_v = NEXTFC_K_V * (pos_diff_unit.dot(vel));
            Eigen::Vector3d euler_angle_diff = EulerAngleDiff(rot, DirectionQuatHorizonal(pos, fc.GetCenter()));
            score_r = NEXTFC_K_R * cos(euler_angle_diff[2]);
            score_bonus = is_visible ? NEXTFC_VISIBLE_BONUS : 0;
            score = score_a + score_d + score_v + score_r + score_bonus;
            if (score > max_score)
            {
                max_score = score;
                max_score_a = score_a;
                max_score_d = score_d;
                max_score_v = score_v;
                max_score_r = score_r;
                max_score_bonus = score_bonus;
                next_fc_ptr = &fc;
                vptr = viewpoint_ptr;
            }
        }
    }
    std::cout << "max_score_a: " << max_score_a << std::endl;
    std::cout << "max_score_d: " << max_score_d << std::endl;
    std::cout << "max_score_v: " << max_score_v << std::endl;
    std::cout << "max_score_r: " << max_score_r << std::endl;
    std::cout << "max_score_bonus: " << max_score_bonus << std::endl;
    std::cout << "max_score: " << max_score << std::endl;
    fc_ptr = next_fc_ptr;
    return !(next_fc_ptr == nullptr);
}

void LocalExplorer::RemoveCurrentTarget()
{
    if (target_fc_ != nullptr)
    {
        target_fc_->SetEmpty();
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

void LocalExplorer::PublishViewpoint(Viewpoint &viewpoint, bool flagged_only = false)
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

void LocalExplorer::PublishColoredViewpoint(Viewpoint &viewpoint)
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

void LocalExplorer::PublishKdTreeRT(Viewpoint &viewpoint)
{
    if (!viewpoint.kd_tree_rt_ptr_)
        return;
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    for (auto fb_ptr : viewpoint.kd_tree_rt_ptr_->fb_list_)
    {
        for (int i = 0; i < 3; i++)
        {
            auto vertex = fb_ptr->vertices_[i];
            point.x = vertex[0]; 
            point.y = vertex[1]; 
            point.z = vertex[2];
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
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    kd_tree_rt_pub_.publish(marker);
}

void LocalExplorer::PublishSingleFrontierCluster(FrontierCluster &fc)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_rgba;
    //ROS_INFO("FrontierClusterCount: %d", viewpoint.GetFrontierClusterCount());
    for (auto facet_ptr : fc.facet_list_)
    {
        color_rgba.r = 1.0;
        color_rgba.g = 1.0;
        color_rgba.b = 1.0;
        color_rgba.a = MARKER_ALPHA;
        marker.colors.push_back(color_rgba);
        for (int i = 0; i < 3; i++)
        {
            auto vertex_ptr = facet_ptr->vertices_[i];
            point.x = vertex_ptr->pos_[0]; point.y = vertex_ptr->pos_[1]; point.z = vertex_ptr->pos_[2];
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
    
    //marker.lifetime = ros::Duration(0.2);

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    single_frontier_cluster_pub_.publish(marker);
}

void LocalExplorer::PublishSingleViewpointFrontier(Viewpoint &viewpoint)
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
    single_viewpoint_frontier_pub_.publish(marker);
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

void LocalExplorer::PublishGlobalNavGoal(Eigen::Vector3f pos, Eigen::Quaterniond rot)
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

void LocalExplorer::PublishLocalNavGoal(Eigen::Vector3f pos, Eigen::Quaterniond rot)
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

void LocalExplorer::PublishTopologicalPath()
{
    //std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
    if (topological_path_.empty())
    {
        ROS_WARN("Topological path empty.");
        return;
    }
    if (target_fc_ == nullptr)
    {
        ROS_WARN("No target frontier cluster selected.");
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
    for (int k = 0; k <= path_length; k++)
    {
        Eigen::Vector3f path_node;
        if (k == path_length)
        {
            path_node = target_fc_->GetCenter();
        }
        else
        {
            path_node = topological_path_[k]->GetOrigin();
        }
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

void LocalExplorer::VoxelizedPointsCallback(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
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
        if (target_fc_ != nullptr)
        {
            PublishSingleFrontierCluster(*target_fc_);
        }
        PublishSingleViewpointFrontier(*viewpoint_ptr);
        PublishKdTreeRT(*viewpoint_ptr);

        if (displayed_viewpoint_num_ >= 0 && displayed_viewpoint_num_ < int(viewpoint_list_.size()))
        {
            PublishColoredViewpoint(*viewpoint_list_[displayed_viewpoint_num_]);
        }

        ProcessNewViewpoint(viewpoint_ptr);
        viewpoint_ptr->ResetKdTreeRT();

        PublishFrontier();
    }
    RepublishVoxelizedPoints(msg_ptr);
}

void LocalExplorer::UavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ptr)
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

void LocalExplorer::UavStateCallback(const snapstack_msgs::State::ConstPtr& msg_ptr)
{
    pos_ << msg_ptr->pos.x, msg_ptr->pos.y, msg_ptr->pos.z;
    rot_ = Eigen::Quaterniond(msg_ptr->quat.w, msg_ptr->quat.x, msg_ptr->quat.y, 
        msg_ptr->quat.z);
    vel_ << msg_ptr->vel.x, msg_ptr->vel.y, msg_ptr->vel.z;
    pose_init_ = true;
}

void LocalExplorer::RecordCommandCallback(const std_msgs::Bool::ConstPtr& msg_ptr)
{
    is_record_ = msg_ptr->data;
}

void LocalExplorer::DisplayedNumCallback(const std_msgs::Int32::ConstPtr& msg_ptr)
{
    displayed_viewpoint_num_ = msg_ptr->data;
}

void LocalExplorer::DroneStatusCallback(const std_msgs::Int32::ConstPtr& msg_ptr)
{
    drone_status_ = msg_ptr->data;
    drone_status_updated_ = true;
}

void LocalExplorer::FasterNavStatusCallback(const std_msgs::Int32::ConstPtr& msg_ptr)
{
    faster_nav_status_ = msg_ptr->data;
    faster_nav_status_updated_ = true;
}

// naive strategy
void LocalExplorer::NavCommandCallback(const ros::TimerEvent& event)
{
    Eigen::Vector3f current_pos, current_vel;
    Eigen::Quaterniond current_rot;
    current_pos << (float)pos_[0], (float)pos_[1], (float)pos_[2];
    current_vel << (float)vel_[0], (float)vel_[1], (float)vel_[2];
    current_rot = rot_;
    switch (nav_state_)
    {
        case NavState::NAV_IN_PATH:
        {
            if (target_fc_ == nullptr || target_fc_->IsEmpty())
            {
                nav_state_ = NavState::REACHED_GOAL;  // stop navigation and replan
                std::cout << "REACHED_GOAL in line 1124\n";
                break;
            }
            if (faster_nav_status_ > 0 && faster_nav_status_updated_)  // navigation exception in faster
            {
                if (!is_le_timer_on_)
                {
                    le_timer_ = ros::Time::now();
                    is_le_timer_on_ = true;
                }
                else if (ros::Time::now() - le_timer_ > ros::Duration(LRG_TIMER_MAX_DURATION))
                {
                    is_le_timer_on_ = false;
                    RemoveCurrentTarget();
                    nav_state_ = NavState::REACHED_GOAL;  // stop navigation and replan
                    std::cout << "REACHED_GOAL in line 1139\n";
                    faster_nav_status_updated_ = false;
                    break;
                }
            }
            ROS_INFO("Current state: NAV_IN_PATH");
            if ((drone_status_ == 3 && drone_status_updated_)
                || (!topological_path_.empty() && topological_path_.front()->Visible(current_pos)))
            // REACHED_GOAL in faster or can navigate to next node directly
            {
                std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
                if (topological_path_.empty())
                {
                    //if (!navigated_to_target_viewpoint_ && target_fc_->is_boarder_) // yaw first
                    if (false)
                    {
                        goal_pos_ = current_pos + 0.01*(target_fc_->GetCenter()-current_pos);
                        goal_rot_ = DirectionQuatHorizonal(current_pos, target_fc_->GetCenter());
                        navigated_to_target_viewpoint_ = true;
                    }
                    else
                    {
                        nav_state_ = NavState::NAV_TO_LOCAL_FRONTIER;
                        goal_pos_ = target_fc_->GetCenter();
                        goal_rot_ = DirectionQuatHorizonal(current_pos, goal_pos_);
                    }
                    PublishLocalNavGoal(goal_pos_, goal_rot_);
                }
                else
                {
                    auto next_vptr = topological_path_.front();
                    topological_path_.pop_front();
                    goal_pos_ = next_vptr->GetOrigin();
                    //goal_rot_ = DirectionQuatHorizonal(current_pos, goal_pos_);
                    if (!topological_path_.empty())
                    {
                        auto next_vptr_2 = topological_path_.front();
                        goal_rot_ = DirectionQuatHorizonal(goal_pos_, next_vptr_2->GetOrigin());
                    }
                    else
                    {
                        goal_rot_ = DirectionQuatHorizonal(goal_pos_, target_fc_->GetCenter());
                    }
                    PublishLocalNavGoal(goal_pos_, goal_rot_);
                }
                drone_status_updated_ = false;
            }
            break;
        }
        case NavState::NAV_TO_LOCAL_FRONTIER:
        {
            if (target_fc_ == nullptr || target_fc_->IsEmpty())
            {
                nav_state_ = NavState::REACHED_GOAL;  // stop navigation and replan
                std::cout << "REACHED_GOAL in line 1193\n";
                break;
            }
            if (faster_nav_status_ > 0 && faster_nav_status_updated_)  // navigation exception in faster
            {
                if (!is_le_timer_on_)
                {
                    le_timer_ = ros::Time::now();
                    is_le_timer_on_ = true;
                }
                else if (ros::Time::now() - le_timer_ > ros::Duration(LRG_TIMER_MAX_DURATION))
                {
                    is_le_timer_on_ = false;
                    RemoveCurrentTarget();
                    nav_state_ = NavState::REACHED_GOAL;  // stop navigation and replan
                    std::cout << "REACHED_GOAL in line 1208\n";
                    faster_nav_status_updated_ = false;
                    break;
                }
            }
            ROS_INFO("Current state: NAV_TO_LOCAL_FRONTIER");
            if (drone_status_ == 3 && drone_status_updated_)  // REACHED_GOAL in faster
            {
                nav_state_ = NavState::REACHED_GOAL;
                std::cout << "REACHED_GOAL in line 1217\n";
                drone_status_updated_ = false;
                RemoveCurrentTarget();
            }
            break;
        }
        case NavState::REACHED_GOAL:
        {
            ROS_INFO("Current state: REACHED_GOAL");
            // try replanning
            ReplanResult replan_result = Replan(current_pos, current_vel, current_rot);
            switch (replan_result.value)
            {
                case 1:
                {
                    nav_state_ = NavState::NAV_TO_LOCAL_FRONTIER;
                    goal_pos_ = goal_pos_ = target_fc_->GetCenter();
                    goal_rot_ = DirectionQuatHorizonal(replan_result.common_vptr->GetOrigin(), goal_pos_);
                    PublishLocalNavGoal(goal_pos_, goal_rot_);
                    drone_status_updated_ = false;
                    break;
                }
                case 2:
                {
                    std::lock_guard<std::mutex> topological_path_lock(topological_path_mutex_);
                    PublishGlobalNavGoal(target_fc_->GetCenter(), DirectionQuatHorizonal(current_pos, target_fc_->GetCenter()));
                    PublishTopologicalPath();
                    nav_state_ = NavState::NAV_IN_PATH;
                    auto next_vptr = topological_path_.front();
                    topological_path_.pop_front();
                    goal_pos_ = next_vptr->GetOrigin();
                    //goal_rot_ = DirectionQuatHorizonal(current_pos, goal_pos_);
                    if (!topological_path_.empty())
                    {
                        auto next_vptr_2 = topological_path_.front();
                        goal_rot_ = DirectionQuatHorizonal(goal_pos_, next_vptr_2->GetOrigin());
                    }
                    else
                    {
                        goal_rot_ = DirectionQuatHorizonal(goal_pos_, target_fc_->GetCenter());
                    }
                    PublishLocalNavGoal(goal_pos_, goal_rot_);
                    drone_status_updated_ = false;
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
    }
}

void LocalExplorer::PublishTopologicalPathCallback(const ros::TimerEvent& event)
{
    PublishTopologicalPath();
}

} // namespace local_explorer

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_explorer_node");
    local_explorer::LocalExplorer len;
    return 0;
}
