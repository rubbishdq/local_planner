#ifndef __KD_TREE_H__
#define __KD_TREE_H__

#include "local_explorer/common.h"
#include "local_explorer/utils.h"
#include "local_explorer/qhull_bridge.h"

#include <utility>
#include <algorithm>

namespace local_explorer
{
// positions in KdTree are relative positions to origin

class FacetBoxRT;

struct AxisPlane
{
    AxisPlane(unsigned char dim = 0, float pos = 0.0) : dim_(dim), pos_(pos) {}

    unsigned char dim_; // 0: x, 1: y, 2: z
    float pos_;
};

struct AABB
{
    AABB();
    AABB(std::vector<FacetBoxRT*>& fb_list);
    bool Intersect(Eigen::Vector3f start, Eigen::Vector3f end);
    std::pair<AABB, AABB> Cut(AxisPlane plane);
    float Area();
    float min_[3], max_[3];
};

class FacetBox
{
public:
    FacetBox();
    FacetBox(Facet &facet);  // NOTE: facet needs to be good (none of its ridges is bad)
    bool In(Eigen::Vector3f pt);
    bool Between(AxisPlane &plane);

    AABB aabb_;
    Eigen::Vector3f normal_;
    float offset_;
};


class KdTreeNode
{
public:
    KdTreeNode(KdTreeNode* np1, KdTreeNode* np2);
    ~KdTreeNode();
    bool Leaf();
    bool In(Eigen::Vector3f pt);
    int FacetBoxCount();
    void Print(int depth, char* pos_str);

    KdTreeNode* child_[2];
    union
    {
        AxisPlane plane_;  // used in non-leaf node
        std::vector<FacetBox*> fb_list_;  // used in leaf node
    };
    #ifdef __DEBUG__
    int fb_count_;
    #endif
};

class KdTree
{
public:
    KdTree();
    KdTree(std::vector<FacetBox*> &fb_list);
    ~KdTree();
    KdTreeNode* Build(std::vector<FacetBox*> &fb_list, int depth);
    AxisPlane Cut(std::vector<FacetBox*> &fb_list, unsigned char dim, bool use_max);
    bool In(Eigen::Vector3f pt);
    void Print();

    KdTreeNode* root_;
    std::vector<FacetBox*> fb_list_;
};

// temporary kd-tree data structure using original position, for raytracing
class FacetBoxRT
{
public:
    FacetBoxRT();
    FacetBoxRT(Facet &facet);  // NOTE: facet needs to be good (none of its ridges is bad)
    bool Intersect(Eigen::Vector3f start, Eigen::Vector3f end);
    bool Between(AxisPlane &plane);

    AABB aabb_;
    Eigen::Vector3f vertices_[3];
};

class KdTreeRTNode
{
public:
    KdTreeRTNode(KdTreeRTNode* np1, KdTreeRTNode* np2);
    ~KdTreeRTNode();
    bool Leaf();
    bool Intersect(Eigen::Vector3f start, Eigen::Vector3f end);
    int FacetBoxCount();
    void Print(int depth, char* pos_str);

    AABB aabb_;
    KdTreeRTNode* child_[2];
    union
    {
        AxisPlane plane_;  // used in non-leaf node
        std::vector<FacetBoxRT*> fb_list_;  // used in leaf node
    };
    #ifdef __DEBUG__
    int fb_count_;
    #endif
};

class KdTreeRT
{
public:
    KdTreeRT();
    KdTreeRT(std::vector<FacetBoxRT*> &fb_list);
    ~KdTreeRT();
    KdTreeRTNode* Build(std::vector<FacetBoxRT*> &fb_list, AABB box, int depth);
    bool Intersect(Eigen::Vector3f start, Eigen::Vector3f end);
    void Print();

    KdTreeRTNode* root_;
    std::vector<FacetBoxRT*> fb_list_;
};

} // namespace local_explorer

#endif