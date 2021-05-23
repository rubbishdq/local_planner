#include<pcl/visualization/cloud_viewer.h>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("/home/dqs/pcd/sparse_pillars.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
    return 0;
}