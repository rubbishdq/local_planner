#include "local_explorer/viewpoint.h"

namespace local_explorer
{

Viewpoint::Viewpoint()
{
    is_generated_ = false;
}

void Viewpoint::GenerateViewpoint(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud)
{
    int cloud_size = int(inverted_cloud.size());
    if (cloud_size >= 4)
    {
        qhull_ptr_ = std::make_shared<orgQhull::Qhull>();
        /*
        Points2Str(*inverted_cloud_ptr_, qhull_input_str_, 4, 3);
        std::istringstream is(qhull_input_str_);
        orgQhull::RboxPoints rbox;
        rbox.appendPoints(is);
        qhull_ptr_->runQhull(rbox, "");
        */
       
        double *arr = new double[3*cloud_size];
        //Points2Array(inverted_cloud, arr);
        arr[0] = 0; arr[1] = 0; arr[2] = 0;
        arr[3] = 1; arr[4] = 0; arr[5] = 0;
        arr[6] = 0; arr[7] = 1; arr[8] = 0;
        arr[9] = 0; arr[10] = 0; arr[11] = 1;
        
        //qhull_ptr_->runQhull("rbox", 3, cloud_size, arr, "");
        //qhull_ptr_->runQhull("rbox", 3, 4, arr, "");
        printf("2222222222222222222222222222222\n");
        
        //delete []arr;
        /*
        printf("3333333333333333333333333333333\n");

        int vertex_count = qhull_ptr_->vertexCount();
        vertex_data_.resize(vertex_count);
        printf("0000000000000000000000000000000\n");
        std::vector<orgQhull::QhullVertex> vertex_list = qhull_ptr_->vertexList().toStdVector();
        printf("4444444444444444444444444444444\n");
        for (auto vertex : vertex_list)
        {
            orgQhull::QhullPoint p = vertex.point();
            int p_id = p.id(), v_id = vertex.id();
            vertex_data_[v_id-1] = cloud[p_id];
        }*/
        //is_generated_ = true;
    }
}

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

std::shared_ptr<orgQhull::Qhull> Viewpoint::GetQhullPtr()
{
    return qhull_ptr_;
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