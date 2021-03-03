#include "common.h"

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

#include "Eigen/Core"

#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <sys/time.h>

namespace localExplorer
{

class ViewPoint
{
public:
    ViewPoint() {}
    ViewPoint(std::vector<Eigen::Vector3d> pcl_data) : pcl_original(pcl_data) { }
    
    void preprocessPCL();

    std::vector<Eigen::Vector3d> pcl_original;
    //std::vector<PointVoxelized> pcl_preprocessed;

};


}