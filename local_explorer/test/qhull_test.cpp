
#include "ros/ros.h"

#include "RboxPoints.h"
#include "QhullError.h"
#include "Qhull.h"
#include "QhullQh.h"
#include "QhullFacet.h"
#include "QhullFacetList.h"
#include "QhullLinkedList.h"
#include "QhullVertex.h"
#include "QhullSet.h"
#include "QhullVertexSet.h"

#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <sys/time.h>

using namespace orgQhull;
using namespace std;

class Pt3d{
public:
    Pt3d() {
        coor[0] = 0.0;
        coor[1] = 0.0;
        coor[2] = 0.0;
    }
    Pt3d(double x, double y, double z) {
        coor[0] = x;
        coor[1] = y;
        coor[2] = z;
    }
    double coor[3];
};

inline double random(double min_val, double max_val)
{
    return (double(rand())/RAND_MAX) * (max_val - min_val) + min_val;
}

void vec2str(vector<Pt3d> pts, char* str)
{
    // sprintf is really slow
    sprintf(str, "3 %zu ", pts.size());
    for (auto iter = pts.begin(); iter != pts.end(); iter++)
    {
        sprintf(str + strlen(str), "%.2lf %.2lf %.2lf ", iter->coor[0], iter->coor[1], iter->coor[2]);
    }
}

void vec2str2(vector<Pt3d> pts, char* str, int int_num, int dec_num)
{
    // not using sprintf, but will add zero
    sprintf(str, "3 %zu ", pts.size());
    int len = strlen(str);
    double coor;
    for (auto iter = pts.begin(); iter != pts.end(); iter++)
    {
        for (int i = 0; i <= 2; i++)
        {
            coor = iter->coor[i];
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

int main() {
    struct timeval t1, t2;
    double deltaT;
    srand(time(0));

    RboxPoints rbox;
    //rbox.appendPoints("6");


    //istringstream is("3 4 0 0 0 0 0 1 0 1 0 1 0 0");
    //rbox.appendPoints(is);
    /*
    char str[] = "3 4 0 0 0 0 0 1 0 1 0 1 0 0";
    istringstream is(str);
    rbox.appendPoints(is);
     */

    vector<Pt3d> pts;
    
    pts.push_back(Pt3d(0, 0, 0));
    pts.push_back(Pt3d(0, 0, 1));
    pts.push_back(Pt3d(0, 1, 0));
    pts.push_back(Pt3d(0.2, 0.2, 0.2));
    pts.push_back(Pt3d(1, 0, 0));
     
     /*
    for (int i = 0; i < 10000; i++)
    {
        pts.push_back(Pt3d(random(0, 99), random(0, 99), random(0, 99)));
    }*/

    char str[5000000];
    gettimeofday(&t1, nullptr);
    //vec2str(pts, str);
    vec2str2(pts, str, 4, 3);

    istringstream is(str);

    gettimeofday(&t2, nullptr);
    deltaT = double(t2.tv_sec-t1.tv_sec) + double(t2.tv_usec-t1.tv_usec)/1e6;
    cout << "Time usage: " << deltaT << " s" << endl;

    //cout << str << endl;
    rbox.appendPoints(is);

    Qhull qhull;
    qhull.runQhull(rbox, "");
    gettimeofday(&t2, nullptr);
    deltaT = double(t2.tv_sec-t1.tv_sec) + double(t2.tv_usec-t1.tv_usec)/1e6;
    cout << "Time usage: " << deltaT << " s" << endl;
    
    cout << "vertexList(): \n";
    cout << qhull.vertexList();
    cout << "facetList(): \n";
    cout << qhull.facetList();
    cout << "TestVertex: \n";
    auto vertexList = qhull.vertexList().toStdVector();
    for (auto iter = vertexList.begin(); iter != vertexList.end(); iter++)
    {
        auto coor = iter->point().toStdVector();
        cout << coor[0] << " " << coor[1] << " " << coor[2] << endl;
        auto pt = iter->point();
        cout << pt[0] << " " << pt[1] << " " << pt[2] << endl;
        cout << "id: " << iter->id() << endl;
    }
    cout << "TestFacet: \n";
     
    return 0;
}

