#include "local_explorer/qhull_bridge.h"

namespace local_explorer
{

void Facet::CalcArea()
{
    Eigen::Vector3f p1, p2, p3;
    p1 = vertices_[0]->pos_;
    p2 = vertices_[1]->pos_;
    p3 = vertices_[2]->pos_;
    area_ = 0.5*(p2-p1.cross(p3-p1)).norm();
}

float Facet::RidgeMaxLength()
{
    Eigen::Vector3f p1, p2, p3;
    float l1, l2, l3;
    p1 = vertices_[0]->pos_;
    p2 = vertices_[1]->pos_;
    p3 = vertices_[2]->pos_;
    l1 = (p2-p1).norm();
    l2 = (p3-p1).norm();
    l3 = (p3-p2).norm();
    return (l1 > l2 && l1 > l3) ? l1 : (l2 > l3 ? l2 : l3);  
}

ConvexHull::ConvexHull()
{
    // empty
}

// modified code of function qh_new_qhull
int ConvexHull::CalcConvexHull(int numpoints, double *points) {
    int exitcode;
    static boolT firstcall = True;
    char qhull_cmd[] = "qhull ";

    if (firstcall) {
        qh_meminit(stderr);
        firstcall= False;
    } else {
        qh_memcheck();
    }
    qh_initqhull_start(NULL, NULL, NULL);
    if(numpoints==0 && points==NULL){
        trace1((qh ferr, 1047, "qh_new_qhull: initialize Qhull\n"));
        return 0;
    }
    trace1((qh ferr, 1044, "qh_new_qhull: build new Qhull for %d %d-d points with %s\n", numpoints, 3, qhull_cmd));
    exitcode = setjmp(qh errexit);
    if (!exitcode)
    {
        qh NOerrexit = False;
        qh_initflags(qhull_cmd);

        qh_init_B(points, numpoints, 3, 0);
        qh_qhull();
    }
    qh NOerrexit = True;
    return exitcode;
} /* new_qhull */

void ConvexHull::ReadQhullGlobalData()
{
    std::map<int, std::shared_ptr<Vertex>> vertex_map;
    std::map<int, std::shared_ptr<Facet>> facet_map;
    //initialize vertices
    int i = 0;
    for (auto ptr = qh vertex_list; ptr != nullptr && i < qh num_vertices; ptr = ptr->next)
    {
        std::shared_ptr<Vertex> vertex_ptr(std::make_shared<Vertex>(i, Eigen::Vector3f(ptr->point[0], ptr->point[1], ptr->point[2])));
        vertex_ptr->original_id_ = qh_pointid(ptr->point);
        vertex_list_.push_back(vertex_ptr);
        vertex_map[(int)ptr->id] = vertex_ptr;
        i++;
    }
    //initialize facets
    i = 0;
    for (auto ptr = qh facet_list; ptr != nullptr && i < qh num_facets; ptr = ptr->next)
    {
        std::shared_ptr<Facet> facet_ptr(std::make_shared<Facet>(i));
        facet_list_.push_back(facet_ptr);
        facet_map[(int)ptr->id] = facet_ptr;
        i++;
    }
    i = 0;
    for (auto ptr = qh facet_list; ptr != nullptr && i < qh num_facets; ptr = ptr->next)
    {
        for (int j = 0; j < 3; j++)
        {
            facet_list_[i]->vertices_.push_back(vertex_map[(int)((vertexT*)(ptr->vertices->e[j].p))->id]);
            facet_list_[i]->neighbors_.push_back(facet_map[(int)((facetT*)(ptr->neighbors->e[j].p))->id]);
        }
        i++;
    }
    qh_freeqhull2(1);
}

int ConvexHull::VertexCount()
{
    return vertex_list_.size();
}

int ConvexHull::FacetCount()
{
    return facet_list_.size();
}

void ConvexHull::ClearFacetFlag()
{
    for (auto facet_ptr : facet_list_)
    {
        facet_ptr->flag_ = 0;
    }
}

} // namespace local_explorer