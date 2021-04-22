# ifndef __VIEWPOINT_H__
#define __VIEWPOINT_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/common.h"
#include "local_explorer/qhull_bridge.h"
#include "local_explorer/kd_tree.h"

#include <list>
#include <cfloat>

namespace local_explorer
{
class Viewpoint;

class FrontierCluster
{
public:
    FrontierCluster(int id = 0);
    void AddFacet(std::shared_ptr<Facet> facet_ptr);
    void SetFacetFlag(flag_t flag);
    bool IsEmpty();
    Eigen::Vector3f GetCenter();

    int id_;  // start from 1
    std::list<std::shared_ptr<Facet>> facet_list_;
    float xyz_range_[2][3];
    float area_;
};

// topological map's node are based on Viewpoint, edge are based on NeighborViewpoint
struct NeighborViewpoint
{
    std::weak_ptr<Viewpoint> viewpoint_ptr_;
    float dist_;
};

class Viewpoint
{
public:
    Viewpoint(Eigen::Vector3f origin = Eigen::Vector3f::Zero());
    void GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud, Eigen::Vector3f origin);
    void Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num); // convert std::vector<LabeledPoint> to qhull's input format
    void Points2Array(std::vector<LabeledPoint> &pts, double* arr);
    void ClusterSingleFacet(std::shared_ptr<Facet> facet_ptr, int cluster_id);
    bool Visible(Eigen::Vector3f pt);
    void CheckVisibility(Viewpoint &v2); // removed frontier points visible in v2
    void PrintFrontierData(int id);
    void AddNeighbor(std::shared_ptr<Viewpoint> viewpoint_ptr);

    float Distance(Viewpoint &v2);
    void SetOrigin(Eigen::Vector3f origin);
    Eigen::Vector3f GetOrigin();
    std::shared_ptr<ConvexHull> GetConvexHullPtr();
    std::vector<FrontierCluster>& GetFrontierClusterList();
    int GetFrontierClusterCount();
    bool IsGenerated();
    void InitDijkstraData();
    bool operator <(const Viewpoint& v2);

public:
    std::shared_ptr<ConvexHull> convex_hull_ptr_;
    std::shared_ptr<KdTree> kd_tree_ptr_;
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