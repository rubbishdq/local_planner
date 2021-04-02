#include "local_explorer/viewpoint.h"

namespace local_explorer
{

FrontierCluster::FrontierCluster(int id)
{
    id_ = id;
    area_ = 0.0;
    is_empty_ = true;
}

void FrontierCluster::AddFacet(std::shared_ptr<Facet> facet_ptr)
{
    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3f p = facet_ptr->vertices_[i]->pos_;
        if (is_empty_)
        {
            for (int j = 0; j < 3; j++)
            {
                xyz_range_[0][j] = p[j];
                xyz_range_[1][j] = p[j];
            }
            is_empty_ = false;
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

bool FrontierCluster::IsEmpty()
{
    return is_empty_;
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
        
        printf("Convex hull vertex count: %d    facet count: %d    ridge count: %d    good ridge count: %d\n", 
            convex_hull_ptr_->VertexCount(), 
            convex_hull_ptr_->FacetCount(), 
            convex_hull_ptr_->RidgeCount(), 
            convex_hull_ptr_->GoodRidgeCount());
        
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
                vertex_ptr->flag_ = 1;
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
        convex_hull_ptr_->DevideLongRidge();
        printf("Convex hull (after long ridges division) vertex count: %d    facet count: %d    ridge count: %d    good ridge count: %d\n", 
            convex_hull_ptr_->VertexCount(), 
            convex_hull_ptr_->FacetCount(), 
            convex_hull_ptr_->RidgeCount(), 
            convex_hull_ptr_->GoodRidgeCount());
        
        // calculate facets' area
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            facet_ptr->CalcArea();
        }
        
        // select frontier facets
        convex_hull_ptr_->ClearFacetFlag();
        for (auto facet_ptr : convex_hull_ptr_->facet_list_)
        {
            if (IsFrontierFacet(*facet_ptr))
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

bool Viewpoint::IsFrontierFacet(Facet &facet)
{
    if (USE_HEIGHT_DIFF_THRESHOLD)
    {
        /*
        Eigen::Vector3f centroid = facet.GetCentroid();
        if ((centroid[2]-BOARDER_HEIGHT_RANGE[0]) < HEIGHT_DIFF_THRESHOLD || 
            (-centroid[2]+BOARDER_HEIGHT_RANGE[1]) < HEIGHT_DIFF_THRESHOLD)
        {
            return false;
        }*/
        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector3f p = facet.vertices_[i]->pos_;
            if ((p[2]-BOARDER_HEIGHT_RANGE[0]) < HEIGHT_DIFF_THRESHOLD || 
                (-p[2]+BOARDER_HEIGHT_RANGE[1]) < HEIGHT_DIFF_THRESHOLD)
            {
                return false;
            }
        }
    }
    for (int i = 0; i < 3; i++)
    {
        if (facet.vertices_[i]->flag_)
        {
            return true;
        }
    }
    return false;
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

bool Viewpoint::In(Eigen::Vector3f pt)
{
    return kd_tree_ptr_->In(pt);
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


} // namespace local_explorer