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

bool Facet::Contain(std::shared_ptr<Vertex> vertex_ptr)
{
    for (int i = 0; i < 3; i++)
    {
        if (vertices_[i] == vertex_ptr)
            return true;
    }
    return false;
}

std::shared_ptr<Ridge> Facet::FindLongestRidge()
{
    float max_length = 0;
    std::shared_ptr<Ridge> longest_ridge_ptr;
    for (int i = 0; i < 3; i++)
    {
        auto ridge_ptr = ridges_[i].lock();
        if (ridge_ptr->GetLength() > max_length)
        {
            max_length = ridge_ptr->GetLength();
            longest_ridge_ptr = ridge_ptr;
        }
    }
    return longest_ridge_ptr;
}

std::shared_ptr<Vertex> Facet::FindLastVertex(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
    for (int i = 0; i < 3; i++)
    {
        if (vertices_[i] != v1 && vertices_[i] != v2)
            return vertices_[i];
    }
    printf("Error in Facet::FindLastVertex: last vertex not found.\n");
    printf("Vertices ID: v1(%d), v2(%d).\n", v1->id_, v2->id_);
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
    printf("Vertices ID: v1(%d), v2(%d).\n", v1->id_, v2->id_);
    return std::shared_ptr<Facet>();
}

std::shared_ptr<Ridge> Facet::FindOtherRidge(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2)
{
    for (int i = 0; i < 3; i++)
    {
        auto ridge_ptr = ridges_[i].lock();
        if (ridge_ptr->Contain(v1) && !ridge_ptr->Contain(v2))
            return ridge_ptr;
    }
    printf("Error in Facet::FindOtherRidge: the other ridge not found.\n");
    printf("Vertices ID: v1(%d), v2(%d).\n", v1->id_, v2->id_);
    return std::shared_ptr<Ridge>();
}

int Facet::FacetType()
{
    if (USE_HEIGHT_DIFF_THRESHOLD)
    {
        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector3f p = vertices_[i]->pos_;
            if ((p[2]-BOARDER_Z_RANGE[0]) < HEIGHT_DIFF_THRESHOLD || 
                (-p[2]+BOARDER_Z_RANGE[1]) < HEIGHT_DIFF_THRESHOLD)
            {
                return -1;
            }
        }
    }
    bool is_frontier = false;
    for (int i = 0; i < 3; i++)
    {
        if (vertices_[i]->flag_ == 2)
        {
            return 2;  // boarder frontier
        }
        else if (vertices_[i]->flag_ == 1)
        {
            is_frontier = true;
        }
    }
    if (is_frontier)
    {
        return 1; //discontinuity frontier
    }
    else
    {
        return 0;
    }
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

bool Ridge::Contain(std::shared_ptr<Vertex> vertex_ptr)
{
    for (int i = 0; i < 2; i++)
    {
        if (vertices_[i] == vertex_ptr)
            return true;
    }
    return false;
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
    return (p2+p1)/2;
}

ConvexHull::ConvexHull(Eigen::Vector3f origin = Eigen::Vector3f::Zero()) : origin_(origin)
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
    //printf("Time usage for ConvexHull::ReadQhullGlobalData(): %lf\n", deltaT);
    qh_freeqhull2(1);
}

void ConvexHull::DivideLongRidge()
{
    struct timeval t1, t2;
    double deltaT;
    gettimeofday(&t1, nullptr);
    int k = 0;
    while (k < RidgeCount())
    {
        auto ridge_ptr = ridge_list_[k];
        if (!ridge_ptr->is_good_ 
            || ridge_ptr->GetLength() < MIN_FRONTIER_RIDGE_LENGTH)  // only divide good ridges
        {
            k++;
            continue;
        }
        // divide longest ridge of triangle first
        for (int i = 0; i < 2; i++)
        {
            std::shared_ptr<Facet> facet_ptr = ridge_ptr->facets_[i].lock();
            std::shared_ptr<Ridge> longest_ridge_ptr = facet_ptr->FindLongestRidge();
            if (longest_ridge_ptr != ridge_ptr)
            {
                ridge_ptr = longest_ridge_ptr;
                break;
            }
        }
        /*
        printf("Current ridge length: %f\n", ridge_ptr->GetLength());
        for (int i = 0; i < 2; i++)
        {
            std::shared_ptr<Facet> facet_ptr = ridge_ptr->facets_[i].lock();
            facet_ptr->PrintVerbose();
        }
        */
        std::shared_ptr<Vertex> ridge_vertex_ptr[2], last_vertex_ptr[2];  // last_vertex: neighbor's third vertex besides ridge vertices
        std::shared_ptr<Facet> neighbor_facet_ptr[2], nn_facet_ptr[2][2]; // nn: neighbor's neighbor
        std::shared_ptr<Ridge> neighbor_other_ridge_ptr[2][2];
        for (int i = 0; i < 2; i++)
        {
            ridge_vertex_ptr[i] = ridge_ptr->vertices_[i];
            neighbor_facet_ptr[i] = ridge_ptr->facets_[i].lock();
        }
        for (int i = 0; i < 2; i++)
        {
            last_vertex_ptr[i] = neighbor_facet_ptr[i]->FindLastVertex(ridge_vertex_ptr[0], ridge_vertex_ptr[1]);
            nn_facet_ptr[i][0] = neighbor_facet_ptr[i]->FindNeighborFacet(ridge_vertex_ptr[0], ridge_vertex_ptr[1]);
            nn_facet_ptr[i][1] = neighbor_facet_ptr[i]->FindNeighborFacet(ridge_vertex_ptr[1], ridge_vertex_ptr[0]);
            neighbor_other_ridge_ptr[i][0] = neighbor_facet_ptr[i]->FindOtherRidge(ridge_vertex_ptr[0], ridge_vertex_ptr[1]);
            neighbor_other_ridge_ptr[i][1] = neighbor_facet_ptr[i]->FindOtherRidge(ridge_vertex_ptr[1], ridge_vertex_ptr[0]);
        }
        Eigen::Vector3f midpoint = ridge_ptr->GetMidPoint();
        std::shared_ptr<Vertex> new_vertex_ptr = std::make_shared<Vertex>(VertexCount(), Invert(midpoint, origin_));
        new_vertex_ptr->pos_ = midpoint;
        std::shared_ptr<Facet> new_facet_ptr[2];
        std::shared_ptr<Ridge> new_ridge_ptr[3];
        for (int i = 0; i < 2; i++)
        {
            new_facet_ptr[i] = std::make_shared<Facet>(FacetCount()+i);
        }
        for (int i = 0; i < 3; i++)
        {
            new_ridge_ptr[i] = std::make_shared<Ridge>(RidgeCount()+i, true);
        }
        if (ridge_vertex_ptr[0]->flag_ == 2 || ridge_vertex_ptr[1]->flag_ == 2) // &&?
            new_vertex_ptr->flag_ = 2;  // mark as boarder
        else
            new_vertex_ptr->flag_ = 1;  // mark as frontier
        // new_vertex_ptr's data is all set
        for (int i = 0; i < 2; i++)
        {
            new_ridge_ptr[i]->vertices_[0] = new_vertex_ptr;
            new_ridge_ptr[i]->vertices_[1] = last_vertex_ptr[i];
            new_ridge_ptr[i]->facets_[0] = new_facet_ptr[i];
            new_ridge_ptr[i]->facets_[1] = neighbor_facet_ptr[i];
        }
        ridge_ptr->vertices_[0] = ridge_vertex_ptr[0];
        ridge_ptr->vertices_[1] = new_vertex_ptr;
        //ridge_ptr->facets_[0] = neighbor_facet_ptr[0];
        //ridge_ptr->facets_[1] = neighbor_facet_ptr[1];
        new_ridge_ptr[2]->vertices_[0] = new_vertex_ptr;
        new_ridge_ptr[2]->vertices_[1] = ridge_vertex_ptr[1];
        new_ridge_ptr[2]->facets_[0] = new_facet_ptr[0];
        new_ridge_ptr[2]->facets_[1] = new_facet_ptr[1];
        // new_ridge_ptr's data is all set
        for (int i = 0; i < 2; i++)
        {
            neighbor_facet_ptr[i]->vertices_[0] = ridge_vertex_ptr[0];
            neighbor_facet_ptr[i]->vertices_[1] = last_vertex_ptr[i];
            neighbor_facet_ptr[i]->vertices_[2] = new_vertex_ptr;
            neighbor_facet_ptr[i]->ridges_[0] = ridge_ptr;
            neighbor_facet_ptr[i]->ridges_[1] = neighbor_other_ridge_ptr[i][0];
            neighbor_facet_ptr[i]->ridges_[2] = new_ridge_ptr[i];
            neighbor_facet_ptr[i]->neighbors_[0] = neighbor_facet_ptr[(i+1)%2];
            neighbor_facet_ptr[i]->neighbors_[1] = nn_facet_ptr[i][0];
            neighbor_facet_ptr[i]->neighbors_[2] = new_facet_ptr[i];
        }
        // old facets' data (neighbor_facet_ptr) is all set (except flag_)
        for (int i = 0; i < 2; i++)
        {
            new_facet_ptr[i]->vertices_[0] = ridge_vertex_ptr[1];
            new_facet_ptr[i]->vertices_[1] = last_vertex_ptr[i];
            new_facet_ptr[i]->vertices_[2] = new_vertex_ptr;
            new_facet_ptr[i]->ridges_[0] = new_ridge_ptr[2];
            new_facet_ptr[i]->ridges_[1] = neighbor_other_ridge_ptr[i][1];
            new_facet_ptr[i]->ridges_[2] = new_ridge_ptr[i];
            new_facet_ptr[i]->neighbors_[0] = new_facet_ptr[(i+1)%2];
            new_facet_ptr[i]->neighbors_[1] = nn_facet_ptr[i][1];
            new_facet_ptr[i]->neighbors_[2] = neighbor_facet_ptr[i];
        }
        // new_facet_ptr's data is all set (except flag_)
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                if (neighbor_other_ridge_ptr[i][1]->facets_[j].lock() == neighbor_facet_ptr[i])
                {
                    neighbor_other_ridge_ptr[i][1]->facets_[j] = new_facet_ptr[i];
                    break;
                }
            }
        }
        // neighbor_other_ridge_ptr's data is reset
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (nn_facet_ptr[i][1]->neighbors_[j].lock() == neighbor_facet_ptr[i])
                {
                    nn_facet_ptr[i][1]->neighbors_[j] = new_facet_ptr[i];
                    break;
                }
            }
        }
        // nn_facet_ptr's data is reset
        vertex_list_.push_back(new_vertex_ptr);
        for (int i = 0; i < 2; i++)
        {
            facet_list_.push_back(new_facet_ptr[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            ridge_list_.push_back(new_ridge_ptr[i]);
        }
    }
    gettimeofday(&t2, nullptr);
    deltaT = double(t2.tv_sec - t1.tv_sec) + double(t2.tv_usec - t1.tv_usec) / 1e6;
    //printf("Time usage for ConvexHull::DivideLongRidge(): %lf\n", deltaT);
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

void ConvexHull::ReleaseRidge()
{
    std::vector<std::shared_ptr<Ridge>> vector_temp;
    vector_temp.swap(ridge_list_);
}

} // namespace local_explorer