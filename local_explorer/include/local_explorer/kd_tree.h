#ifndef __KD_TREE_H__
#define __KD_TREE_H__

#include "local_explorer/common.h"
#include "local_explorer/utils.h"
#include "local_explorer/qhull_bridge.h"

#include <utility>
#include <algorithm>

namespace local_explorer
{
// positions in kd-tree are relative positions to origin

struct AxisPlane
{
    AxisPlane(unsigned char dim = 0, float pos = 0.0) : dim_(dim), pos_(pos) {}

    unsigned char dim_; // 0: x, 1: y, 2: z
    float pos_;
};

class FacetBox
{
public:
    FacetBox();
    FacetBox(Facet &facet);  // NOTE: facet needs to be good (none of its ridges is bad)
    bool In(Eigen::Vector3f pt);
    bool Between(AxisPlane &plane);

    float min_[3];
    float max_[3];
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
        AxisPlane plane_;  // used in leaf node
        std::vector<FacetBox*> fb_list_;  // used in non-leaf node
    };
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

} // namespace local_explorer

#endif