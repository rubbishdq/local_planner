# ifndef __VIEWPOINT_H__
#define __VIEWPOINT_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/common.h"
#include "local_explorer/qhull_bridge.h"

namespace local_explorer
{

class FrontierCluster
{
public:
    FrontierCluster(int id = 0);
    void AddFacet(std::shared_ptr<Facet> facet_ptr);
    bool IsEmpty();

    int id_;  // start from 1
    std::vector<std::shared_ptr<Facet>> facet_list_;
    float xyz_range_[2][3];
    float area_;
    bool is_empty_;
};
class Viewpoint
{
public:
    Viewpoint();
    void GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud);
    void Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num); // convert std::vector<LabeledPoint> to qhull's input format
    void Points2Array(std::vector<LabeledPoint> &pts, double* arr);
    bool IsFrontierFacet(Facet &facet);
    void ClusterSingleFacet(std::shared_ptr<Facet> facet_ptr, int cluster_id);

    std::shared_ptr<ConvexHull> GetConvexHullPtr();
    std::vector<LabeledPoint>& GetVertexData();
    std::vector<FrontierCluster>& GetFrontierClusterList();
    int GetFrontierClusterCount();
    bool IsGenerated();

private:
    std::shared_ptr<ConvexHull> convex_hull_ptr_;
    std::vector<LabeledPoint> vertex_data_;   // original position of convex hull's vertexs(not inverted)
    std::vector<FrontierCluster> frontier_cluster_list_;  // candidate frontier
    int frontier_cluster_count_;
    bool is_generated_;
};


} // namespace local_explorer


#endif