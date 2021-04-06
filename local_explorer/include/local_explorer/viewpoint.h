# ifndef __VIEWPOINT_H__
#define __VIEWPOINT_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/common.h"
#include "local_explorer/qhull_bridge.h"
#include "local_explorer/kd_tree.h"

#include <list>

namespace local_explorer
{

class FrontierCluster
{
public:
    FrontierCluster(int id = 0);
    void AddFacet(std::shared_ptr<Facet> facet_ptr);
    void SetFacetFlag(flag_t flag);
    bool IsEmpty();

    int id_;  // start from 1
    std::list<std::shared_ptr<Facet>> facet_list_;
    float xyz_range_[2][3];
    float area_;
    bool is_empty_;
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

    void SetOrigin(Eigen::Vector3f origin);
    Eigen::Vector3f GetOrigin();
    std::shared_ptr<ConvexHull> GetConvexHullPtr();
    std::vector<FrontierCluster>& GetFrontierClusterList();
    int GetFrontierClusterCount();
    bool IsGenerated();

public:
    std::shared_ptr<ConvexHull> convex_hull_ptr_;
    std::shared_ptr<KdTree> kd_tree_ptr_;
    std::vector<FrontierCluster> frontier_cluster_list_;  // candidate frontier
    int frontier_cluster_count_;
    bool is_generated_;
};


} // namespace local_explorer


#endif