#include "frontier_generator.h"

namespace localExplorer
{

void ViewPoint::preprocessPCL()
{
    /*
    std::vector<Eigen::Vector3d>(this->pcl_preprocessed).swap(this->pcl_preprocessed);  // clear the vector
    for (auto iter = this->pcl_original.begin(); iter != this->pcl_original.end(); iter++)
    {
        Eigen::Vector3d pt = *iter;
        if (pt.norm() < MIN_RANGE_FOR_EXTENSION)
        {
            1;
        }
        else
        {
            2;
        }
    }
    return;
    */
}

}


int main()
{
    Eigen::Vector3d a(1,1,0), b(0,0,1);
    std::cout << a << "  " << b << std::endl;
    b = a;
    std::cout << a << "  " << b << std::endl;
    b[2] = 2;
    std::cout << a << "  " << b << std::endl;
    std::cout << a.norm() << std::endl;

    return 0;
}

