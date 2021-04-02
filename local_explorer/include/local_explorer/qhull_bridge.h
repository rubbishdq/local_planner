// based on non-reentrant qhull v7.2.0
// Gazebo used non-reentrant qhull, thus reentrant qhull is unusable in this project

# ifndef __QHULL_BRIDGE_H__
#define __QHULL_BRIDGE_H__

#include "local_explorer/common.h"
#include "local_explorer/utils.h"

extern "C" {
#include "qhull_a.h"
}
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <memory>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include <stdarg.h>

namespace local_explorer
{

class Vertex;
class Facet;
class Ridge;

class Vertex
{
public:
    Vertex(int id = 0, Eigen::Vector3f pos = Eigen::Vector3f::Zero()) : id_(id), pos_inverted_(pos) {flag_ = 0;} // pos_ is not initialized here
    void Print();
    void PrintVerbose(bool use_inverted_pos);

    int id_;
    Eigen::Vector3f pos_inverted_;
    Eigen::Vector3f pos_;  // non-inverted position
    flag_t flag_;
};

class Facet
{
public:
    Facet(int id = 0) : id_(id), area_(0.0) {flag_ = 0;}
    void Print();
    void PrintVerbose(int space_count);
    void CalcArea(); // must be used after vertices_ are initialized
    Eigen::Vector3f GetCentroid();
    bool Contain(std::shared_ptr<Vertex> vertex_ptr);
    std::shared_ptr<Ridge> FindLongestRidge();
    std::shared_ptr<Vertex> FindLastVertex(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2);
    std::shared_ptr<Facet> FindNeighborFacet(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2); // find neighbor facet that contains v1 but doesn't contains v2
    std::shared_ptr<Ridge> FindOtherRidge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2); // find ridge that contains v1 but doesn't contains v2
    bool IsFlaggedFacet(); // return false position of facet is out of range, return true if some of its vertices' are frontier vertices, otherwise return false

    void CheckRidgeStatus(); // only used for debug

    int id_;
    float area_;
    flag_t flag_;
    std::shared_ptr<Vertex> vertices_[3];
    std::weak_ptr<Facet> neighbors_[3];
    std::weak_ptr<Ridge> ridges_[3];
};

class Ridge
{
public:
    Ridge(int id = 0, bool is_good = false) : id_(id), is_good_(is_good) {}
    void Print();
    void PrintVerbose(int space_count);
    bool Contain(std::shared_ptr<Vertex> vertex_ptr);
    float GetLength();
    Eigen::Vector3f GetMidPoint();

    int id_;
    std::shared_ptr<Vertex> vertices_[2];
    std::weak_ptr<Facet> facets_[2];
    bool is_good_; // A good ridge is included by exactly 2 facets. Bad ridges often result from points that are too close
};

class ConvexHull
{
public:
    ConvexHull(Eigen::Vector3f origin);
    int CalcConvexHull(int numpoints, double *points);
    void ReadQhullGlobalData();  // non-reentrant qhull's global data will be released at the end of this function
    void DevideLongRidge();  // devide long ridges and mark frontier vertices
    int VertexCount();
    int FacetCount();
    int RidgeCount();
    int GoodRidgeCount();
    void ClearFacetFlag();
    void ClearVertexFlag();
    void ReleaseRidge();

    Eigen::Vector3f origin_;
    std::vector<std::shared_ptr<Facet>> facet_list_;
    std::vector<std::shared_ptr<Vertex>> vertex_list_;
    std::vector<std::shared_ptr<Ridge>> ridge_list_;
};


} // namespace local_explorer

#endif