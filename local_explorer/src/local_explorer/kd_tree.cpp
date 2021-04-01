#include "local_explorer/kd_tree.h"

namespace local_explorer
{
FacetBox::FacetBox()
{
    for (int i = 0; i < 3; i++)
    {
        min_[i] = max_[i] = 0;
    }
    offset_ = 0;
}

FacetBox::FacetBox(Facet &facet)
{
    Eigen::Vector3f p[3];
    for (int i = 0; i < 3; i++)
        p[i] = facet.vertices_[i]->pos_inverted_;
    Eigen::Vector3f perp_vec = (p[1]-p[0]).cross(p[2]-p[0]);
    if (perp_vec.norm() != 0)
        normal_ = perp_vec / perp_vec.norm();
    offset_ = normal_.dot(p[0]);
    // ensure origin is at the opposite direction of normal_, that is to say origin.dot(normal_) < offset_
    if (offset_ < 0)
    {
        normal_ = -normal_;
        offset_ = -offset_;
    }
    for (int i = 0; i < 3; i++)
    {
        min_[i] = (p[0][i] < p[1][i] && p[0][i] < p[2][i]) ? p[0][i] : (p[1][i] < p[2][i] ? p[1][i] : p[2][i]);
        max_[i] = (p[0][i] > p[1][i] && p[0][i] > p[2][i]) ? p[0][i] : (p[1][i] > p[2][i] ? p[1][i] : p[2][i]);
    }
}

bool FacetBox::In(Eigen::Vector3f pt)
{
    return pt.dot(normal_) < offset_;
}

bool FacetBox::Between(AxisPlane &plane)
{
    unsigned char dim = plane.dim_;
    return (plane.pos_ > min_[dim]) && (plane.pos_ < max_[dim]);
}

KdTreeNode::KdTreeNode(KdTreeNode* np1 = nullptr, KdTreeNode* np2 = nullptr)
    : child_{np1, np2}, fb_list_()
{
    // empty
}

KdTreeNode::~KdTreeNode()
{
    for (int i = 0; i < 2; i++)
    {
        delete child_[i];
    }
    if (Leaf())
    {
        fb_list_.~vector<FacetBox*>();
    }
}

bool KdTreeNode::Leaf()
{
    return (child_[0] == nullptr) && (child_[1] == nullptr);
}

bool KdTreeNode::In(Eigen::Vector3f pt)
{
    if (Leaf())
    {
        for (auto fb_ptr : fb_list_)
        {
            if (!fb_ptr->In(pt))
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        if (pt[plane_.dim_] < plane_.pos_)
        {
            return (child_[0] == nullptr) || child_[0]->In(pt);
        }
        else
        {
            return (child_[1] == nullptr) || child_[1]->In(pt);
        }
    }
}

int KdTreeNode::FacetBoxCount()
{
    #ifdef __DEBUG__
    return fb_count_;

    #else
    if (Leaf())
        return fb_list_.size();
    else
    {
        int count = 0;
        if (child_[0] != nullptr)
            count += child_[0]->FacetBoxCount();
        if (child_[1] != nullptr)
            count += child_[1]->FacetBoxCount();
        return count;
    }
    #endif
}

void KdTreeNode::Print(int depth = 0, char* pos_str = nullptr)
{
    if (depth == 0)
    {
        pos_str = new char[KD_TREE_MAX_DEPTH+1];
        printf("kd-tree layer %d, ", depth);
    }
    else
    {
        PrintSpace(depth);
        printf("kd-tree layer %d, position: %s, ", depth, pos_str);
    }
    printf("Facet box count: %d\n", FacetBoxCount());
    if (Leaf())
    {
        PrintSpace(depth);
        printf("Is leaf node\n");
    }
    else
    {
        PrintSpace(depth);
        printf("Left child: ");
        if (child_[0] == nullptr)
        {
            printf("nullptr\n");
        }
        else
        {
            printf("\n");
            pos_str[depth] = 'l';
            pos_str[depth+1] = '\0';
            child_[0]->Print(depth+1, pos_str);
            pos_str[depth] = '\0';
        }
        PrintSpace(depth);
        printf("Right child: ");
        if (child_[1] == nullptr)
        {
            printf("nullptr\n");
        }
        else
        {
            printf("\n");
            pos_str[depth] = 'r';
            pos_str[depth+1] = '\0';
            child_[1]->Print(depth+1, pos_str);
            pos_str[depth] = '\0';
        }
    }
    if (depth == 0)
    {
        delete []pos_str;
    }
}

KdTree::KdTree()
{
    root_ = nullptr;
}

KdTree::KdTree(std::vector<FacetBox*> &fb_list)
{
    fb_list_.assign(fb_list.begin(), fb_list.end());
    root_ = Build(fb_list, 0);
}

KdTree::~KdTree()
{
    delete root_;
    for (auto fb_ptr : fb_list_)
    {
        delete fb_ptr;
    }
}

KdTreeNode* KdTree::Build(std::vector<FacetBox*> &fb_list, int depth)
{
    // a naive kd-tree building algorithm
    int fb_count = fb_list.size();
    if (fb_count == 0 || depth >= KD_TREE_MAX_DEPTH)
    {
        return nullptr;
    }
    KdTreeNode* new_node_ptr = new KdTreeNode;

    #ifdef __DEBUG__
    new_node_ptr->fb_count_ = fb_count;
    #endif

    if (fb_count < KD_TREE_MIN_OBJ_COUNT || depth == KD_TREE_MAX_DEPTH-1)
    {
        new_node_ptr->fb_list_.assign(fb_list.begin(), fb_list.end());
        return new_node_ptr;
    }
    // aim at least facets that belong to both of the child trees
    bool use_max = (depth % 2 == 0);
    int common_facetbox_count, least_count = INT_MAX;
    AxisPlane best_plane;
    unsigned char dim;
    float pos;
    for (dim = 0; dim < 3; dim++)
    {
        common_facetbox_count = 0;
        AxisPlane div_plane = Cut(fb_list, dim, use_max);
        for (auto fb_ptr : fb_list)
        {
            if (fb_ptr->Between(div_plane))
            {
                common_facetbox_count++;
            }
        }
        if (common_facetbox_count < least_count)
        {
            least_count = common_facetbox_count;
            best_plane = div_plane;
        }
    }
    if (least_count == fb_count) // unable to divide FacetBox
    {
        new_node_ptr->fb_list_.assign(fb_list.begin(), fb_list.end());
        return new_node_ptr;
    }
    dim = best_plane.dim_;
    pos = best_plane.pos_;
    std::vector<FacetBox*> fb_list_left, fb_list_right;
    for (auto fb_ptr : fb_list)
    {
        if (fb_ptr->min_[dim] < pos)
        {
            fb_list_left.push_back(fb_ptr);
        }
        if (fb_ptr->max_[dim] > pos)
        {
            fb_list_right.push_back(fb_ptr);
        }
    }
    new_node_ptr->plane_ = best_plane;
    new_node_ptr->child_[0] = Build(fb_list_left, depth+1);
    new_node_ptr->child_[1] = Build(fb_list_right, depth+1);
    return new_node_ptr;
}

AxisPlane KdTree::Cut(std::vector<FacetBox*> &fb_list, unsigned char dim, bool use_max = false)
{
    AxisPlane div_plane(dim);
    std::vector<float> pos_list;
    for (auto fb_ptr : fb_list)
    {
        if (use_max)
        {
            pos_list.push_back(fb_ptr->max_[dim]);
        }
        else
        {
            pos_list.push_back(fb_ptr->min_[dim]);
        }
    }
    std::nth_element(pos_list.begin(), pos_list.begin() + pos_list.size() / 2, pos_list.end());
    if (use_max)
        div_plane.pos_ = pos_list[pos_list.size() / 2] - 2 * EPS_F;
    else
        div_plane.pos_ = pos_list[pos_list.size() / 2] + 2 * EPS_F;
    return div_plane;
}

bool KdTree::In(Eigen::Vector3f pt)
{
    return root_->In(pt);
}

void KdTree::Print()
{
    root_->Print();
}

} // namespace local_explorer