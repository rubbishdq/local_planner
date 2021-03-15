#include "local_explorer/viewpoint_generator.h"

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
        Points2Array(inverted_cloud, arr);
        qhull_ptr_->runQhull("rbox", 3, cloud_size, arr, "");
        delete []arr;

        int vertex_count = qhull_ptr_->vertexCount();
        vertex_pos_.resize(vertex_count);
        std::vector<orgQhull::QhullVertex> vertex_list = qhull_ptr_->vertexList().toStdVector();
        for (auto vertex : vertex_list)
        {
            orgQhull::QhullPoint p = vertex.point();
            int p_id = p.id(), v_id = vertex.id();
            vertex_pos_[v_id-1] = cloud[p_id].mu_;
        }
        is_generated_ = true;
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

bool Viewpoint::IsGenerated()
{
    return is_generated_;
}

ViewpointGenerator::ViewpointGenerator()
{
}

void ViewpointGenerator::PreprocessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr, 
    std::vector<LabeledPoint> &cloud)
{
    double origin[3] = {(double)msg_ptr->origin.x, (double)msg_ptr->origin.y, (double)msg_ptr->origin.z};
    Eigen::Vector3f origin_vec(msg_ptr->origin.x, msg_ptr->origin.y, msg_ptr->origin.z);
    int n;
    Eigen::Vector3f mu;
    Eigen::Matrix3f sigma;
    for (auto &point : msg_ptr->points)
    {
        n = point.n;
        mu << point.mu[0], point.mu[1], point.mu[2];
        sigma << point.sigma[0], point.sigma[1], point.sigma[2], 
            point.sigma[1], point.sigma[3], point.sigma[4], 
            point.sigma[2], point.sigma[4], point.sigma[5];
        if ((origin_vec-mu).norm() <= MIN_RANGE_FOR_EXTENSION)
        {
            continue;
        }
        cloud.push_back(LabeledPoint(n, mu, sigma, NORMAL)); 
    }
    AddBoarderPoints(cloud, origin);
}

void ViewpointGenerator::AddBoarderPoints(std::vector<LabeledPoint> &cloud, double origin[3])
{
    double lowest_vertex[3] = {origin[0]-DIMENSION[0]/2, origin[1]-DIMENSION[1]/2, origin[2]-DIMENSION[2]/2};
    double highest_vertex[3] = {origin[0]+DIMENSION[0]/2, origin[1]+DIMENSION[1]/2, origin[2]+DIMENSION[2]/2};
    Eigen::Vector3f mu;
    for (double diff_x = RESOLUTION; diff_x < DIMENSION[0]; diff_x+=RESOLUTION)
    {
        for (double diff_y = RESOLUTION; diff_y < DIMENSION[1]; diff_x+=RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]+diff_y), (float)(highest_vertex[2]);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
    for (double diff_x = RESOLUTION; diff_x < DIMENSION[0]; diff_x+=RESOLUTION)
    {
        for (double diff_z = RESOLUTION; diff_z < DIMENSION[2]; diff_z+=RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]+diff_x), (float)(lowest_vertex[1]), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(lowest_vertex[0]+diff_x), (float)(highest_vertex[1]), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
    for (double diff_y = RESOLUTION; diff_y < DIMENSION[1]; diff_y+=RESOLUTION)
    {
        for (double diff_z = RESOLUTION; diff_z < DIMENSION[2]; diff_z+=RESOLUTION)
        {
            mu << (float)(lowest_vertex[0]), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
            mu << (float)(highest_vertex[0]), (float)(lowest_vertex[1]+diff_y), (float)(lowest_vertex[2]+diff_z);
            cloud.push_back(LabeledPoint(1, mu, Eigen::Matrix3f::Zero(), BOARDER)); 
        }
    }
}

void ViewpointGenerator::InvertPoints(std::vector<LabeledPoint> &cloud, std::vector<LabeledPoint> &inverted_cloud)
{
    inverted_cloud.assign(cloud.begin(), cloud.end());
    for (auto &point : inverted_cloud)
    {
        point.invert(INVERT_PARAM);
    }
}

void ViewpointGenerator::ProcessVoxelizedPoints(const global_mapper_ros::VoxelizedPoints::ConstPtr& msg_ptr)
{
    processed_cloud_ptr_ = std::make_shared<std::vector<LabeledPoint>>();
    inverted_cloud_ptr_ = std::make_shared<std::vector<LabeledPoint>>();

    PreprocessVoxelizedPoints(msg_ptr, *processed_cloud_ptr_);

    InvertPoints(*processed_cloud_ptr_, *inverted_cloud_ptr_);
    viewpoint_ptr_->GenerateViewpoint(*processed_cloud_ptr_, *inverted_cloud_ptr_);

}

std::shared_ptr<Viewpoint> ViewpointGenerator::GetViewpointPtr()
{
    return viewpoint_ptr_;
}

bool ViewpointGenerator::IsGenerated()
{
    return viewpoint_ptr_->IsGenerated();
}

} // namespace local_explorer



