// based on non-reentrant qhull v7.2.0
// Gazebo used non-reentrant qhull, thus reentrant qhull is unusable in this project

# ifndef __QHULL_BRIDGE_H__
#define __QHULL_BRIDGE_H__

extern "C" {
#include "qhull_a.h"
}
#include "Eigen/Core"
#include "ros/ros.h"

#include <memory>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdarg.h>

namespace local_explorer
{

class Vertex
{
public:
    Vertex(int id = 0, Eigen::Vector3f pos = Eigen::Vector3f::Zero()) : id_(id), pos_(pos) {original_id_ = 0;}

    int id_;
    Eigen::Vector3f pos_;
    int original_id_;
};

class Facet
{
public:
    Facet(int id = 0) : id_(id) {}

    int id_;
    std::vector<std::shared_ptr<Vertex>> vertices_;
    std::vector<std::shared_ptr<Facet>> neighbors_;
};

class ConvexHull
{
public:
    ConvexHull();
    int CalcConvexHull(int numpoints, double *points);
    void ReadQhullGlobalData();  // non-reentrant qhull's global data will be released at the end of this function
    int VertexCount();
    int FacetCount();

    std::vector<std::shared_ptr<Facet>> facet_list_;
    std::vector<std::shared_ptr<Vertex>> vertex_list_;
};


} // namespace local_explorer

#endif