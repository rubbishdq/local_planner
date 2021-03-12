# ifndef __VIEWPOINT_GENERATOR_H__
#define __VIEWPOINT_GENERATOR_H__

#include "local_explorer/labeled_point.h"
#include "local_explorer/common.h"

#include "RboxPoints.h"
#include "QhullError.h"
#include "Qhull.h"
#include "QhullQh.h"
#include "QhullFacet.h"
#include "QhullFacetList.h"
#include "QhullLinkedList.h"
#include "QhullVertex.h"
#include "QhullSet.h"
#include "QhullVertexSet.h"

#include "global_mapper_ros/VoxelizedPoints.h"
#include "ros/ros.h"

#include "Eigen/Core"
#include <sstream>
#include <vector>
#include <memory>

namespace local_explorer
{

class ViewpointGenerator
{
public:
    ViewpointGenerator();

    void PreprocessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr, 
        std::vector<LabeledPoint> &cloud);
    void AddBoarderPoints(std::vector<LabeledPoint> &cloud, double origin[3]);
    void InvertPoints(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud);
    void Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num); // convert std::vector<LabeledPoint> to qhull's input format

    void ProcessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr);

private:
    std::shared_ptr<std::vector<LabeledPoint>> processed_cloud_ptr_;
    std::shared_ptr<std::vector<LabeledPoint>> inverted_cloud_ptr_;
    char qhull_input_str_[500000];
};

} // namespace local_explorer

#endif