#include "local_explorer/viewpoint.h"

namespace local_explorer
{

Viewpoint::Viewpoint()
{
    convex_hull_ptr_ = std::make_shared<ConvexHull>();
    is_generated_ = false;
}

void Viewpoint::GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud)
{
    int cloud_size = int(inverted_cloud.size());
    if (cloud_size >= 4)
    {
        convex_hull_ptr_ = std::make_shared<ConvexHull>();
        
        double *arr = new double[3*cloud_size];
        Points2Array(inverted_cloud, arr);
        convex_hull_ptr_->CalcConvexHull(cloud_size, arr);
        convex_hull_ptr_->ReadQhullGlobalData();
        
        delete []arr;
        
        int vertex_count = convex_hull_ptr_->VertexCount();
        vertex_data_.resize(vertex_count);
        for (auto vertex_ptr : convex_hull_ptr_->vertex_list_)
        {
            vertex_data_[vertex_ptr->id_] = cloud[vertex_ptr->original_id_];
            vertex_ptr->pos_ = cloud[vertex_ptr->original_id_].mu_;
        }
        is_generated_ = true;
    }
}

// useless in non-reentrant qhull
void Viewpoint::Points2Str(std::vector<LabeledPoint> &pts, char* str, int int_num, int dec_num)
{
    // not widely using sprintf because sprintf is slow, but will add zero
    sprintf(str, "3 %zu ", pts.size());
    int len = strlen(str);
    double coor;
    for (auto &point : pts)
    {
        for (int i = 0; i <= 2; i++)
        {
            coor = point.mu_[i];
            if (coor < 0)
            {
                str[len] = '-';
                ++len;
                coor = -coor;
            }
            //coor *= pow10(dec_num);
            for (int j = 0; j < dec_num; j++)
            {
                coor *= 10.0;
            }
            int coor_temp = int(coor);
            char digit;
            int pow10 = 1;
            for (int j = 0; j < int_num + dec_num - 1; j++)
            {
                pow10 *= 10;
            }
            for (int j = int_num; j >= -dec_num; j--)
            {
                if (j == 0)
                {
                    str[len] = '.';
                    ++len;
                    continue;
                }
                else
                {
                    digit = char(coor_temp / pow10) + char('0');
                    str[len] = digit;
                    ++len;
                    coor_temp %= pow10;
                    pow10 /= 10;
                }
            }
            str[len] = ' ';
            ++len;
        }
    }
    str[len] = '\0';
}

void Viewpoint::Points2Array(std::vector<LabeledPoint> &pts, double* arr)
{
    int ind = 0;
    for (auto &point : pts)
    {
        for (int i = 0; i < 3; i++)
        {
            arr[ind+i] = (double)point.mu_[i];
        }
        ind += 3;
    }
}

std::shared_ptr<ConvexHull> Viewpoint::GetConvexHullPtr()
{
    return convex_hull_ptr_;
}

std::vector<LabeledPoint>& Viewpoint::GetVertexData()
{
    return vertex_data_;
}

bool Viewpoint::IsGenerated()
{
    return is_generated_;
}


} // namespace local_explorer