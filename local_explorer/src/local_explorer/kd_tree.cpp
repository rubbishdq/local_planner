#include "local_explorer/kd_tree.h"

namespace local_explorer
{
AABB::AABB()
{
    for (int i = 0; i < 3; i++)
    {
        min_[i] = max_[i] = 0;
    }
}

AABB::AABB(std::vector<FacetBoxRT*>& fb_list)
{
    if (fb_list.empty())
    {
        for (int i = 0; i < 3; i++)
        {
            min_[i] = max_[i] = 0;
        }
    }
    else
    {
        int ind = 0;
        for (auto fb_ptr : fb_list)
        {
            if (ind == 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    min_[i] = fb_ptr->aabb_.min_[i];
                    max_[i] = fb_ptr->aabb_.max_[i];
                }
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    if (min_[i] > fb_ptr->aabb_.min_[i])
                        min_[i] = fb_ptr->aabb_.min_[i];
                    if (max_[i] < fb_ptr->aabb_.max_[i])
                        max_[i] = fb_ptr->aabb_.max_[i];
                }
            }
            ind++;
        }
    }
}

bool AABB::Intersect(Eigen::Vector3f start, Eigen::Vector3f end)
{
    // naive method: check if line segment intersects with triangle for 12 times
    Eigen::Vector3f vertices[8];
    int index[12][3] = {{0,1,2}, {1,2,6},
        {0,2,3}, {2,3,4},
        {0,1,3}, {1,3,5},
        {1,5,6}, {5,6,7}, 
        {3,4,5}, {4,5,7}, 
        {2,4,6}, {4,6,7}};
    vertices[0] << min_[0], min_[1], min_[2];
    vertices[1] << max_[0], min_[1], min_[2];
    vertices[2] << min_[0], max_[1], min_[2];
    vertices[3] << min_[0], min_[1], max_[2];
    vertices[4] << min_[0], max_[1], max_[2];
    vertices[5] << max_[0], min_[1], max_[2];
    vertices[6] << max_[0], max_[1], min_[2];
    vertices[7] << max_[0], max_[1], max_[2];
    for (int i = 0; i < 12; i++)
    {
        if (IsLineSegIntersectTri(vertices[index[i][0]], vertices[index[i][1]], vertices[index[i][2]], start, end))
        {
            return true;
        }
    }
    return false;
}

std::pair<AABB, AABB> AABB::Cut(AxisPlane plane)
{
    std::pair<AABB, AABB> res;
    res.first = *this;
    res.second = *this;
    res.first.max_[(int)plane.dim_] = plane.pos_;
    res.second.min_[(int)plane.dim_] = plane.pos_;
    return res;
}

float AABB::Area()
{
    Eigen::Vector3f diff(max_[0]-min_[0], max_[1]-min_[1], max_[2]-min_[2]);
    return 2*(diff[0]*diff[1]+diff[1]*diff[2]+diff[2]*diff[0]);
}

FacetBox::FacetBox()
{
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
        aabb_.min_[i] = (p[0][i] < p[1][i] && p[0][i] < p[2][i]) ? p[0][i] : (p[1][i] < p[2][i] ? p[1][i] : p[2][i]);
        aabb_.max_[i] = (p[0][i] > p[1][i] && p[0][i] > p[2][i]) ? p[0][i] : (p[1][i] > p[2][i] ? p[1][i] : p[2][i]);
    }
}

bool FacetBox::In(Eigen::Vector3f pt)
{
    return pt.dot(normal_) < offset_;
}

bool FacetBox::Between(AxisPlane &plane)
{
    unsigned char dim = plane.dim_;
    return (plane.pos_ > aabb_.min_[dim]) && (plane.pos_ < aabb_.max_[dim]);
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
        if (fb_ptr->aabb_.min_[dim] < pos)
        {
            fb_list_left.push_back(fb_ptr);
        }
        if (fb_ptr->aabb_.max_[dim] > pos)
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
            pos_list.push_back(fb_ptr->aabb_.max_[dim]);
        }
        else
        {
            pos_list.push_back(fb_ptr->aabb_.min_[dim]);
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
    if (root_)
        root_->Print();
}

FacetBoxRT::FacetBoxRT()
{
    // empty
}

FacetBoxRT::FacetBoxRT(Facet &facet)
{
    Eigen::Vector3f p[3];
    for (int i = 0; i < 3; i++)
        p[i] = facet.vertices_[i]->pos_;
    for (int i = 0; i < 3; i++)
    {
        aabb_.min_[i] = (p[0][i] < p[1][i] && p[0][i] < p[2][i]) ? p[0][i] : (p[1][i] < p[2][i] ? p[1][i] : p[2][i]);
        aabb_.max_[i] = (p[0][i] > p[1][i] && p[0][i] > p[2][i]) ? p[0][i] : (p[1][i] > p[2][i] ? p[1][i] : p[2][i]);
        vertices_[i] = p[i];
    }
}

bool FacetBoxRT::Intersect(Eigen::Vector3f start, Eigen::Vector3f end)
{
    return IsLineSegIntersectTri(vertices_[0], vertices_[1], vertices_[2], start, end);
}

bool FacetBoxRT::Between(AxisPlane &plane)
{
    unsigned char dim = plane.dim_;
    return (plane.pos_ > aabb_.min_[dim]) && (plane.pos_ < aabb_.max_[dim]);
}

KdTreeRTNode::KdTreeRTNode(KdTreeRTNode* np1 = nullptr, KdTreeRTNode* np2 = nullptr)
    : child_{np1, np2}, fb_list_()
{
    // empty
}

KdTreeRTNode::~KdTreeRTNode()
{
    for (int i = 0; i < 2; i++)
    {
        delete child_[i];
    }
    if (Leaf())
    {
        fb_list_.~vector<FacetBoxRT*>();
    }
}

bool KdTreeRTNode::Leaf()
{
    return (child_[0] == nullptr) && (child_[1] == nullptr);
}

bool KdTreeRTNode::Intersect(Eigen::Vector3f start, Eigen::Vector3f end)
{
    // start-end line segment is known to intersect with aabb_
    if (Leaf())
    {
        for (auto fb_ptr : fb_list_)
        {
            if (fb_ptr->Intersect(start, end))
            {
                return true;
            }
        }
        return false;
    }
    for (int i = 0; i < 2; i++)
    {
        if (!child_[i]->aabb_.Intersect(start, end))
        {
            continue;
        }
        if (child_[i]->Intersect(start, end))
        {
            return true;
        }
    }
    return false;
}

int KdTreeRTNode::FacetBoxCount()
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

void KdTreeRTNode::Print(int depth = 0, char* pos_str = nullptr)
{
    if (depth == 0)
    {
        pos_str = new char[KD_TREE_RT_MAX_DEPTH+1];
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

KdTreeRT::KdTreeRT()
{
    root_ = nullptr;
}

KdTreeRT::KdTreeRT(std::vector<FacetBoxRT*> &fb_list)
{
    struct timeval t1, t2;
    double deltaT;
    gettimeofday(&t1, nullptr);

    fb_list_.assign(fb_list.begin(), fb_list.end());
    AABB box(fb_list);
    root_ = Build(fb_list, box, 0);

    gettimeofday(&t2, nullptr);
    deltaT = double(t2.tv_sec - t1.tv_sec) + double(t2.tv_usec - t1.tv_usec) / 1e6;
    printf("Time usage for KdTreeRT building: %lf\n", deltaT);
}

KdTreeRT::~KdTreeRT()
{
    delete root_;
    for (auto fb_ptr : fb_list_)
    {
        delete fb_ptr;
    }
}

KdTreeRTNode* KdTreeRT::Build(std::vector<FacetBoxRT*> &fb_list, AABB box, int depth)
{
    // a naive kd-tree building algorithm depending on SAH
    int fb_count = fb_list.size();
    if (fb_count == 0 || depth >= KD_TREE_RT_MAX_DEPTH)
    {
        return nullptr;
    }
    KdTreeRTNode* new_node_ptr = new KdTreeRTNode;
    new_node_ptr->aabb_ = box;

    #ifdef __DEBUG__
    new_node_ptr->fb_count_ = fb_count;
    #endif

    if (fb_count < KD_TREE_RT_MIN_OBJ_COUNT || depth == KD_TREE_RT_MAX_DEPTH-1)
    {
        new_node_ptr->fb_list_.assign(fb_list.begin(), fb_list.end());
        return new_node_ptr;
    }
    
    //int common_facetbox_count, least_count = INT_MAX;
    AxisPlane best_plane(3);
    std::pair<AABB, AABB> cut_box;
    float min_cost = FLT_MAX;
    unsigned char dim;
    float pos;
    
    typedef std::pair<float, bool> pfb;
	auto gen_cand_list = [&fb_list, &box](unsigned char d)
    {
		std::vector<pfb> cand_list;
		for (auto fb_ptr : fb_list)
        {
            if (fb_ptr->aabb_.min_[int(d)] > box.min_[int(d)])
			    cand_list.push_back(pfb(fb_ptr->aabb_.min_[int(d)] - EPS_F, true));
            if (fb_ptr->aabb_.max_[int(d)] < box.max_[int(d)])
			    cand_list.push_back(pfb(fb_ptr->aabb_.max_[int(d)] + EPS_F, false));
        }
		std::sort(cand_list.begin(), cand_list.end());
		return cand_list;
	};

    for (unsigned char d = 0; d < 3; d++)
    {
		std::vector<pfb> cand_list = gen_cand_list(d);

		int lcnt = 0, rcnt = fb_count;
		auto ptr = cand_list.begin();

		do {
			AxisPlane plane(d, ptr->first);
			try {
				cut_box = box.Cut(plane);
				float cost = cut_box.first.Area() * lcnt + cut_box.second.Area() * rcnt;
				if (lcnt == 0 or rcnt == 0 or (lcnt + rcnt == fb_count - 1 && rcnt == 1))
					cost *= 0.8;		// this is a hack
				if (cost < min_cost)
                {
                    best_plane = plane;
                    min_cost = cost;
                }
			} catch (...) {}

			if (ptr->second) lcnt ++; else rcnt --;

			auto old = ptr++;
			while (ptr != cand_list.end() && ptr->first - old->first < EPS_F) {
				if (ptr->second) lcnt ++;
				else rcnt --;
				old = ptr++;
			}
		} while (ptr != cand_list.end());
    }

    if (best_plane.dim_ >= 3)  // failed to find best plane
    {
        new_node_ptr->fb_list_.assign(fb_list.begin(), fb_list.end());
        return new_node_ptr;
    }

    dim = best_plane.dim_;
    pos = best_plane.pos_;
    cut_box = box.Cut(best_plane);
    std::vector<FacetBoxRT*> fb_list_left, fb_list_right;
    for (auto fb_ptr : fb_list)
    {
        if (fb_ptr->aabb_.min_[dim] < pos)
        {
            fb_list_left.push_back(fb_ptr);
        }
        if (fb_ptr->aabb_.max_[dim] > pos)
        {
            fb_list_right.push_back(fb_ptr);
        }
    }
    if (int(fb_list_left.size()) == fb_count || int(fb_list_right.size()) == fb_count)  // division is useless
    {
        new_node_ptr->fb_list_.assign(fb_list.begin(), fb_list.end());
        return new_node_ptr;
    }

    new_node_ptr->plane_ = best_plane;
    new_node_ptr->child_[0] = Build(fb_list_left, cut_box.first, depth+1);
    new_node_ptr->child_[1] = Build(fb_list_right, cut_box.second, depth+1);
    return new_node_ptr;
}

bool KdTreeRT::Intersect(Eigen::Vector3f start, Eigen::Vector3f end)
{
    if (!root_)
        return false;
    if (!root_->aabb_.Intersect(start, end))
        return false;
    return root_->Intersect(start, end);
}

void KdTreeRT::Print()
{
    if (root_)
        root_->Print();
}


} // namespace local_explorer