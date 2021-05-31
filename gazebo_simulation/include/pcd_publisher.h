#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cstdlib>
#include <cmath>
using namespace std;

class PCDPublisherNode
{
public:
    PCDPublisherNode();
    void PublishPointCloud(const ros::TimerEvent& event);
    void sendTFData();
    
    // ROS相关内容
    ros::NodeHandle n_;

    ros::Publisher pointcloudPub_;

    ros::Timer publish_timer_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    bool ready_to_save_;
};


