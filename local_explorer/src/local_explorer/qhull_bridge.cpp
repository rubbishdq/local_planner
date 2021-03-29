#include "local_explorer/qhull_bridge.h"

namespace local_explorer
{
void Vertex::Print()
{
    printf("v%d  ", id_);
}

void Vertex::PrintVerbose(bool use_inverted_pos = false)
{
    Eigen::Vector3f pos;
    if (use_inverted_pos)
        pos = pos_inverted_;
    else
        pos = pos_;
    printf("v%d(%.3f, %.3f, %.3f)  ", id_, pos[0], pos[1], pos[2]);
}

void Facet::Print()
{
    printf("f%d  ", id_);
}

void Facet::PrintVerbose(int space_count = 0)
{
    PrintSpace(space_count);
    printf("f%d:\n", id_);
    PrintSpace(space_count+2);
    printf("--vertices: ");
    for (int i = 0; i < 3; i++)
    {
        vertices_[i]->PrintVerbose();
    }
    printf("\n");
    PrintSpace(space_count+2);
    printf("--neighbors: ");
    for (int i = 0; i < 3; i++)
    {
        auto neighbor_ptr = neighbors_[i].lock();
        neighbor_ptr->Print();
    }
    printf("\n");
    PrintSpace(space_count+2);
    printf("--ridges: ");
    for (int i = 0; i < 3; i++)
    {
        auto ridge_ptr = ridges_[i].lock();
        ridge_ptr->Print();
    }
    printf("\n");
}

void Facet::CalcArea()
{
    Eigen::Vector3f p1, p2, p3;
    p1 = vertices_[0]->pos_;
    p2 = vertices_[1]->pos_;
    p3 = vertices_[2]->pos_;
    area_ = 0.5*(p2-p1.cross(p3-p1)).norm();
}

Eigen::Vector3f Facet::GetCentroid()
{
    Eigen::Vector3f p1, p2, p3;
    p1 = vertices_[0]->pos_;
    p2 = vertices_[1]->pos_;
    p3 = vertices_[2]->pos_;
    return (p1+p2+p3)/3;
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

bool Facet::Contain(std::shared_ptr<Vertex> vertex_ptr)
{
    for (int i = 0; i < 3; i++)
    {
        if (vertices_[i] == vertex_ptr)
            return true;
    }
    return false;
}

std::shared_ptr<Vertex> Facet::FindLastVertex(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
    for (int i = 0; i < 3; i++)
    {
        if (vertices_[i] != v1 && vertices_[i] != v2)
            return vertices_[i];
    }
    printf("Error in Facet::FindLastVertex: last vertex not found.\n");
    return std::shared_ptr<Vertex>();
}

std::shared_ptr<Facet> Facet::FindNeighborFacet(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
    for (int i = 0; i < 3; i++)
    {
        auto facet_ptr = neighbors_[i].lock();
        if (facet_ptr->Contain(v1) && !facet_ptr->Contain(v2))
            return facet_ptr;
    }
    printf("Error in Facet::FindNeighborFacet: neighbor facet not found.\n");
    return std::shared_ptr<Facet>();
}

// only used for debug
void Facet::CheckRidgeStatus()
{
    bool is_good = true;
    for (int i = 0; i < 3; i++)
    {
        auto ridge_ptr = ridges_[i].lock();
        if (!ridge_ptr->is_good_)
        {
            is_good = false;
            break;
        }
    }
    if (is_good)
        return;
    PrintVerbose();
    printf("  --neighbors' details:\n");
    for (int i = 0; i < 3; i++)
    {
        auto neighbor_ptr = neighbors_[i].lock();
        neighbor_ptr->PrintVerbose(4);
    }
}

void Ridge::Print()
{
    char is_good;
    if (is_good_)
        is_good = 'g';
    else
        is_good = 'b';
    printf("r%d(v%d, v%d, %c)  ", id_, vertices_[0]->id_, vertices_[1]->id_, is_good);
}

void Ridge::PrintVerbose(int space_count = 0)
{
    int facet_count;
    PrintSpace(space_count);
    printf("r%d:", id_);
    if (is_good_)
    {
        printf("(good)\n");
        facet_count = 2;
    }
    else
    {
        printf("(not good)\n");
        facet_count = 1;
    }
    PrintSpace(space_count+2);
    printf("--vertices: ");
    for (int i = 0; i < 2; i++)
    {
        vertices_[i]->Print();
    }
    printf("\n");
    PrintSpace(space_count+2);
    printf("--facets: ");
    for (int i = 0; i < facet_count; i++)
    {
        auto facet_ptr = facets_[i].lock();
        facet_ptr->Print();
    }
    printf("\n");
}

float Ridge::GetLength()
{
    Eigen::Vector3f p1, p2;
    p1 = vertices_[0]->pos_;
    p2 = vertices_[1]->pos_;
    return (p2-p1).norm();
}

Eigen::Vector3f Ridge::GetMidPoint()
{
    Eigen::Vector3f p1, p2;
    p1 = vertices_[0]->pos_;
    p2 = vertices_[1]->pos_;
    return (p2-p1)/2;
}

ConvexHull::ConvexHull()
{
    // empty
}

// modified code of function qh_new_qhull
int ConvexHull::CalcConvexHull(int numpoints, double *points) {
    //printf("numpoints: %d\n", numpoints);
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
    struct timeval t1, t2;
    double deltaT;
    gettimeofday(&t1, nullptr);
    std::map<int, std::shared_ptr<Vertex>> vertex_map;
    std::map<int, std::shared_ptr<Facet>> facet_map;
    std::map<int, std::shared_ptr<Ridge>> ridge_map;
    //initialize vertices
    int i = 0;
    for (auto ptr = qh vertex_list; ptr != nullptr && i < qh num_vertices; ptr = ptr->next)
    {
        std::shared_ptr<Vertex> vertex_ptr(std::make_shared<Vertex>(i, Eigen::Vector3f(ptr->point[0], ptr->point[1], ptr->point[2])));
        vertex_ptr->flag_ = qh_pointid(ptr->point);  // represent original id of vertex here
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
            facet_list_[i]->vertices_[j] = (vertex_map[(int)((vertexT*)(ptr->vertices->e[j].p))->id]);
            facet_list_[i]->neighbors_[j] = (facet_map[(int)((facetT*)(ptr->neighbors->e[j].p))->id]);
        }
        i++;
    }
    // initialize ridges
    // it seems that qh doesn't contain ridge data for convex hull
    // NOTE: algorithm below is only suitable for points whose number is below sqrt(INT_MAX)
    i = 0;
    for (auto facet_ptr : facet_list_)
    {
        for (int j = 0; j < 3; j++)
        {
            int vertex_id[2], ridge_id;
            vertex_id[0] = facet_ptr->vertices_[j]->id_;
            vertex_id[1] = facet_ptr->vertices_[(j+1)%3]->id_;
            if (vertex_id[0] > vertex_id[1])
            {
                int temp = vertex_id[0]; vertex_id[0] = vertex_id[1]; vertex_id[1] = temp;
            }
            // suppose x = VertexCount, then 0 1 -> 1, 0 2 -> 2, ... 0 x-1 -> x-1, 1 2 -> x,  1 3 -> x+1, etc  
            // p q -> p*(2*x-1-p)/2+q-p
            if (vertex_id[0] % 2 == 0)
                ridge_id = (vertex_id[0]/2)*(2*VertexCount()-1-vertex_id[0]) + vertex_id[1] - vertex_id[0];
            else
                ridge_id = vertex_id[0]*((2*VertexCount()-1-vertex_id[0])/2) + vertex_id[1] - vertex_id[0];
            auto map_iter = ridge_map.find(ridge_id);
            if (map_iter == ridge_map.end())
            {
                std::shared_ptr<Ridge> ridge_ptr = std::make_shared<Ridge>(i);
                ridge_ptr->vertices_[0] = facet_ptr->vertices_[j];
                ridge_ptr->vertices_[1] = facet_ptr->vertices_[(j+1)%3];
                ridge_ptr->facets_[0] = facet_ptr;
                facet_ptr->ridges_[j] = ridge_ptr;
                ridge_list_.push_back(ridge_ptr);
                ridge_map[ridge_id] = ridge_ptr;
                i++;
            }
            else
            {
                map_iter->second->facets_[1] = facet_ptr;
                map_iter->second->is_good_ = true;
                facet_ptr->ridges_[j] = map_iter->second;
            }
        }
    } 
    gettimeofday(&t2, nullptr);
    deltaT = double(t2.tv_sec - t1.tv_sec) + double(t2.tv_usec - t1.tv_usec) / 1e6;
    printf("Time usage for ConvexHull::ReadQhullGlobalData(): %lf\n", deltaT);
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

int ConvexHull::RidgeCount()
{
    return ridge_list_.size();
}

int ConvexHull::GoodRidgeCount()
{
    int count = 0;
    for (auto ridge_ptr : ridge_list_)
    {
        if (ridge_ptr->is_good_)
            count++;
    }
    return count;
}

void ConvexHull::ClearFacetFlag()
{
    for (auto facet_ptr : facet_list_)
    {
        facet_ptr->flag_ = 0;
    }
}

void ConvexHull::ClearVertexFlag()
{
    for (auto vertex_ptr : vertex_list_)
    {
        vertex_ptr->flag_ = 0;
    }
}

} // namespace local_explorer