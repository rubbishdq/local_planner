# ifndef __VIEWPOINT_H__
#define __VIEWPOINT_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/common.h"
#include "local_explorer/qhull_bridge.h"
#include "local_explorer/kd_tree.h"

#include <list>
#include <iterator>
#include <cfloat>

namespace local_explorer
{
class Viewpoint;

class FrontierCluster
{
public:
    FrontierCluster(int id = 0, bool is_boarder_ = false);
    void AddFacet(std::shared_ptr<Facet> facet_ptr);
    void CalcXYZRange();
    void SetFacetFlag(flag_t flag);
    bool IsEmpty();
    void SetEmpty();
    Eigen::Vector3f GetCenter();
    void RemoveFacet(std::list<std::shared_ptr<Facet>>::iterator &iter);

    int id_;  // start from 1
    std::list<std::shared_ptr<Facet>> facet_list_;
    float xyz_range_[2][3];
    float area_;
    bool is_boarder_;
};

// topological map's node are based on Viewpoint, edge are based on NeighborViewpoint
struct NeighborViewpoint
{
    std::weak_ptr<Viewpoint> viewpoint_ptr_;
    float dist_;
};

class Viewpoint : public std::enable_shared_from_this<Viewpoint>
{
public:
    Viewpoint(Eigen::Vector3f origin = Eigen::Vector3f::Zero());
    void GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud, Eigen::Vector3f origin);
    void Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num); // convert std::vector<LabeledPoint> to qhull's input format
    void Points2Array(std::vector<LabeledPoint> &pts, double* arr);
    void ClusterSingleFacet(std::shared_ptr<Facet> facet_ptr, int cluster_id, int clustering_flag);
    bool Visible(Eigen::Vector3f pt);
    bool IntersectWithObstacle(Eigen::Vector3f start, Eigen::Vector3f end);
    void CheckVisibility(Viewpoint &v2); // removed frontier points visible in v2
    void PrintFrontierData(int id);
    void AddNeighbor(std::shared_ptr<Viewpoint> viewpoint_ptr);
    void ResetNeighbor();
    void RemoveSmallFrontierCluster(float min_area);
    void RecalcFrontierClusterRange();

    float Distance(Viewpoint &v2);
    void SetOrigin(Eigen::Vector3f origin);
    Eigen::Vector3f GetOrigin();
    std::shared_ptr<ConvexHull> GetConvexHullPtr();
    std::vector<FrontierCluster>& GetFrontierClusterList();
    int GetFrontierClusterCount();
    bool IsGenerated();
    void InitDijkstraData();
    void ResetKdTreeRT();
    bool operator <(const Viewpoint& v2);

public:
    std::shared_ptr<ConvexHull> convex_hull_ptr_;
    std::shared_ptr<KdTree> kd_tree_ptr_;
    std::shared_ptr<KdTreeRT> kd_tree_rt_ptr_;  // kd-tree used for raytracing, will be released soon after viewpoint is constructed
    std::vector<FrontierCluster> frontier_cluster_list_;  // candidate frontier
    std::list<NeighborViewpoint> neighbor_list_;
    int frontier_cluster_count_;
    bool is_generated_;

    // data used in Dijkstra algorithm
    bool is_visited_;
    float dist_;
    std::weak_ptr<Viewpoint> last_viewpoint_;

};


} // namespace local_explorer


#endif