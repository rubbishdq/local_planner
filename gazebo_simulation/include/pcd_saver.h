#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cstdlib>
#include <cmath>
using namespace std;

class PCDSaverNode
{
public:
    PCDSaverNode();
    
private:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void savecommandCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // ROS相关内容
    ros::NodeHandle n_;

    ros::Subscriber pointcloudSub_;
    ros::Subscriber savecommandSub_;

    bool ready_to_save_;
};


