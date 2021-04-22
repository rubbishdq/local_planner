#include "local_explorer/viewpoint.h"

namespace local_explorer
{

FrontierCluster::FrontierCluster(int id)
{
    id_ = id;
    area_ = 0.0;
}

void FrontierCluster::AddFacet(std::shared_ptr<Facet> facet_ptr)
{
    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3f p = facet_ptr->vertices_[i]->pos_;
        if (IsEmpty())
        {
            for (int j = 0; j < 3; j++)
            {
                xyz_range_[0][j] = p[j];
                xyz_range_[1][j] = p[j];
            }
        }
        else
        {
            for (int j = 0; j < 3; j++)
            {
                if (xyz_range_[0][j] > p[j])
                    xyz_range_[0][j] = p[j];
                if (xyz_range_[1][j] < p[j])
                    xyz_range_[1][j] = p[j];
            }
        }
    }
    facet_list_.push_back(facet_ptr);
    area_ += facet_ptr->area_;
}

void FrontierCluster::SetFacetFlag(flag_t flag)
{
    for (auto facet_ptr : facet_list_)
    {
        facet_ptr->flag_ = flag;
    }
}

bool FrontierCluster::IsEmpty()
{
    return facet_list_.empty();
}

Eigen::Vector3f FrontierCluster::GetCenter()
{
    return Eigen::Vector3f((xyz_range_[0][0]+xyz_range_[1][0]) / 2, 
        (xyz_range_[0][1]+xyz_range_[1][1]) / 2,
        (xyz_range_[0][2]+xyz_range_[1][2]) / 2);
}

Viewpoint::Viewpoint(Eigen::Vector3f origin)
{
    convex_hull_ptr_ = std::make_shared<ConvexHull>(origin);
    frontier_cluster_count_ = 0;
    is_generated_ = false;
}

void Viewpoint::GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud, Eigen::Vector3f origin)
{
    int cloud_size = int(inverted_cloud.size());
    if (cloud_size >= 4)
    {
        convex_hull_ptr_ = std::make_shared<ConvexHull>(origin);
        
        double *arr = new double[3*cloud_size];
        Points2Array(inverted_cloud, arr);
        convex_hull_ptr_->CalcConvexHull(cloud_size, arr);
        convex_hull_ptr_->ReadQhullGlobalData();
        
        delete []arr;
        
        /*
        printf("Convex hull vertex count: %d    facet count: %d    ridge count: %d    good ridge count: %d\n", 
            convex_hull_ptr_->VertexCount(), 
            convex_hull_ptr_->FacetCount(), 
            convex_hull_ptr_->RidgeCount(), 
            convex_hull_ptr_->GoodRidgeCount());
        */
        
        // generate kd-tree
        std::vector<FacetBox*> fb_list;
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            FacetBox* new_fb_ptr = new FacetBox(*facet_ptr);
            fb_list.push_back(new_fb_ptr);
        }
        kd_tree_ptr_ = std::make_shared<KdTree>(fb_list);

        for (auto vertex_ptr : convex_hull_ptr_->vertex_list_)
        {
            vertex_ptr->pos_ = cloud[vertex_ptr->flag_].mu_;
            if (cloud[vertex_ptr->flag_].type_ == BOARDER)
                vertex_ptr->flag_ = 2;
            else
                vertex_ptr->flag_ = 0;
            // now vertex_ptr->flag_ represents if this vertex is a frontier vertex
        }
        /*
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            facet_ptr->CheckRidgeStatus();
        }
        */ // for debug
        convex_hull_ptr_->DivideLongRidge();
        /*
        printf("Convex hull (after long ridges division) vertex count: %d    facet count: %d    ridge count: %d    good ridge count: %d\n", 
            convex_hull_ptr_->VertexCount(), 
            convex_hull_ptr_->FacetCount(), 
            convex_hull_ptr_->RidgeCount(), 
            convex_hull_ptr_->GoodRidgeCount());
        */
        
        // calculate facets' area
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            facet_ptr->CalcArea();
        }

        // marked other frontier vertices of a boarder triangle as boarder vertices
        convex_hull_ptr_->ClearFacetFlag();
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            for (auto vertex_ptr : facet_ptr->vertices_)
            {
                if (vertex_ptr->flag_ == 2)
                {
                    facet_ptr->flag_ = 1;
                    break;
                }
            }
        }
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            if (facet_ptr->flag_ != 1)
                continue;
            for (auto vertex_ptr : facet_ptr->vertices_)
            {
                if (vertex_ptr->flag_ != 0)
                {
                    vertex_ptr->flag_ = 2;
                }
            }
        }
        
        // select frontier facets
        convex_hull_ptr_->ClearFacetFlag();
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            if (facet_ptr->IsFlaggedFacet())
            {
                facet_ptr->flag_ = -1;  // labeled as frontier facet
            }
        }
        // cluster frontier facets
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            if (facet_ptr->flag_ == -1)
            {
                frontier_cluster_list_.push_back(FrontierCluster(frontier_cluster_count_));
                frontier_cluster_count_++;
                ClusterSingleFacet(facet_ptr, frontier_cluster_count_);
            }
        }
        /*
        frontier_cluster_list_.resize(frontier_cluster_count_);
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            if (facet_ptr->flag_ > 0)
            {
                frontier_cluster_list_[facet_ptr->flag_-1].AddFacet(facet_ptr);
            }
        }
        */
        // remove facet cluster that is too small
        auto fc_iter = frontier_cluster_list_.begin();
        while (fc_iter != frontier_cluster_list_.end())
        {
            if (fc_iter->area_ < MIN_FRONTIER_CLUSTER_AREA)
            {
                fc_iter->SetFacetFlag(0);
                fc_iter = frontier_cluster_list_.erase(fc_iter);
                frontier_cluster_count_--;
            }
            else
            {
                fc_iter++;
            }
        }

        is_generated_ = true;
    }
}

// useless in non-reentrant qhull
void Viewpoint::Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num)
{
    // not widely using sprintf because sprintf is slow, but will add zero
    sprintf(str, "3 %zu ", pts.size());
    int len = strlen(str);
    double coor;
    for (auto &point : pts)
    {
        for (int i = 0; i <= 2; i++)
        {
            coor = point.mu_[i];
            if (coor < 0)
            {
                str[len] = '-';
                ++len;
                coor = -coor;
            }
            //coor *= pow10(dec_num);
            for (int j = 0; j < dec_num; j++)
            {
                coor *= 10.0;
            }
            int coor_temp = int(coor);
            char digit;
            int pow10 = 1;
            for (int j = 0; j < int_num + dec_num - 1; j++)
            {
                pow10 *= 10;
            }
            for (int j = int_num; j >= -dec_num; j--)
            {
                if (j == 0)
                {
                    str[len] = '.';
                    ++len;
                    continue;
                }
                else
                {
                    digit = char(coor_temp / pow10) + char('0');
                    str[len] = digit;
                    ++len;
                    coor_temp %= pow10;
                    pow10 /= 10;
                }
            }
            str[len] = ' ';
            ++len;
        }
    }
    str[len] = '\0';
}

void Viewpoint::Points2Array(std::vector<LabeledPoint> &pts, double* arr)
{
    int ind = 0;
    for (auto &point : pts)
    {
        for (int i = 0; i < 3; i++)
        {
            arr[ind+i] = (double)point.mu_[i];
        }
        ind += 3;
    }
}

void Viewpoint::ClusterSingleFacet(std::shared_ptr<Facet> facet_ptr, int cluster_id)
{
    facet_ptr->flag_ = cluster_id;
    frontier_cluster_list_[cluster_id-1].AddFacet(facet_ptr);
    for (int k = 0; k < 3; k++)
    {
        auto next_facet_ptr = facet_ptr->neighbors_[k].lock();
        if (next_facet_ptr->flag_ < 0)
        {
            // check if adding this facet will exceed frontier cluster size limit
            bool loop_flag = true;  // if loop_flag == false, next_facet_ptr will not be added to this cluster
            for (int i = 0; i < 3 && loop_flag; i++)
            {
                Eigen::Vector3f p = next_facet_ptr->vertices_[i]->pos_;
                for (int j = 0; j < 3 && loop_flag; j++)
                {
                    if (p[j] > frontier_cluster_list_[cluster_id-1].xyz_range_[0][j]+FRONTIER_CLUSTER_SIZE_LIMIT[j]
                        || p[j] < frontier_cluster_list_[cluster_id-1].xyz_range_[1][j]-FRONTIER_CLUSTER_SIZE_LIMIT[j])
                    {
                        loop_flag = false;
                    }
                }
            }
            if (loop_flag)
            {
                ClusterSingleFacet(next_facet_ptr, cluster_id);
            }
        }
    }
}

bool Viewpoint::Visible(Eigen::Vector3f pt)
{
    Eigen::Vector3f diff = pt - GetOrigin();
    if (diff.norm() > KD_TREE_IN_THRESHOLD)
    {
        pt = pt - (diff/diff.norm())*KD_TREE_IN_THRESHOLD;
    }
    Eigen::Vector3f pt_inverted = Invert(pt, GetOrigin());
    return !kd_tree_ptr_->In(pt_inverted);
}

void Viewpoint::CheckVisibility(Viewpoint &v2)
{
    int erase_count = 0;
    for (auto &fc : frontier_cluster_list_)
    {
        auto facet_iter = fc.facet_list_.begin();
        while (facet_iter != fc.facet_list_.end())
        {
            bool is_frontier = false;
            for (auto vertex_ptr : (*facet_iter)->vertices_)
            {
                if (vertex_ptr->flag_ == 0)
                {
                    continue;
                }
                Eigen::Vector3f pos = vertex_ptr->pos_, v2_origin = v2.GetOrigin();
                if (!InBoxRange(pos-v2_origin))
                {
                    continue;
                }
                if (v2.Visible(pos))
                {
                    vertex_ptr->flag_ = 0;
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
    //printf("Erased %d frontier facets in Viewpoint::CheckVisibility().\n", erase_count);
}

void Viewpoint::PrintFrontierData(int id = 0)
{
    int frontier_point_count = 0, frontier_facet_count = 0;
    for (auto &fc : frontier_cluster_list_)
    {
        frontier_facet_count += fc.facet_list_.size();
        for (auto facet_ptr : fc.facet_list_)
        {
            for (auto vertex_ptr : facet_ptr->vertices_)
            {
                if (vertex_ptr->flag_ != 0)
                {
                    frontier_point_count++;
                }
            }
        }
    }
    printf("Viewpoint %d frontier points: %d, frontier facets: %d\n", id, frontier_point_count, frontier_facet_count);
}

void Viewpoint::AddNeighbor(std::shared_ptr<Viewpoint> viewpoint_ptr)
{
    NeighborViewpoint nv;
    nv.viewpoint_ptr_ = viewpoint_ptr;
    nv.dist_ = Distance(*viewpoint_ptr);
    neighbor_list_.push_back(nv);
}

float Viewpoint::Distance(Viewpoint &v2)
{
    Eigen::Vector3f origin = GetOrigin(), v2_origin = v2.GetOrigin();
    return (origin - v2_origin).norm();
}

void Viewpoint::SetOrigin(Eigen::Vector3f origin)
{
    convex_hull_ptr_->origin_ = origin;
}

Eigen::Vector3f Viewpoint::GetOrigin()
{
    return convex_hull_ptr_->origin_;
}


std::shared_ptr<ConvexHull> Viewpoint::GetConvexHullPtr()
{
    return convex_hull_ptr_;
}

std::vector<FrontierCluster>& Viewpoint::GetFrontierClusterList()
{
    return frontier_cluster_list_;
}

int Viewpoint::GetFrontierClusterCount()
{
    return frontier_cluster_count_;
}

bool Viewpoint::IsGenerated()
{
    return is_generated_;
}

void Viewpoint::InitDijkstraData()
{
    is_visited_ = false;
    dist_ = FLT_MAX;
    last_viewpoint_.reset();
}

bool Viewpoint::operator <(const Viewpoint& v2)
{
    return (dist_ < v2.dist_);
}


} // namespace local_explorer