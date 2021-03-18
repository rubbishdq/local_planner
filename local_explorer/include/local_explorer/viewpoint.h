# ifndef __VIEWPOINT_H__
#define __VIEWPOINT_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/common.h"
#include "local_explorer/qhull_bridge.h"

namespace local_explorer
{

class Viewpoint
{
public:
    Viewpoint();
    void GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud);
    void Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num); // convert std::vector<LabeledPoint> to qhull's input format
    void Points2Array(std::vector<LabeledPoint> &pts, double* arr);

    std::shared_ptr<ConvexHull> GetConvexHullPtr();
    std::vector<LabeledPoint>& GetVertexData();
    bool IsGenerated();

private:
    std::shared_ptr<ConvexHull> convex_hull_ptr_;
    std::vector<LabeledPoint> vertex_data_;   // original position of convex hull's vertexs(not inverted)
    bool is_generated_;
};


} // namespace local_explorer


#endif