#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>

ros::Publisher pub;
ros::Subscriber sub;
std::vector<livox_ros_driver2::CustomMsg::ConstPtr> livox_data;

void cb(const livox_ros_driver2::CustomMsg::ConstPtr livox_msg_in)
{
    livox_data.push_back(livox_msg_in);
    if(livox_data.size() < 1){
        return;
    }

    pcl::PointCloud<pcl::PointXYZINormal> pcl_in;
    for(std::size_t i = 0; i < livox_data.size(); ++i){
        auto livox_msg = livox_data[i];
        auto time_end = livox_msg->points.back().offset_time;
        
        for(int j = 0; j < livox_msg->point_num; ++j){
            pcl::PointXYZINormal pt;
            pt.x = livox_msg->points[j].x;
            pt.y = livox_msg->points[j].y;
            pt.z = livox_msg->points[j].z;
            pt.intensity = livox_msg->points[j].line + livox_msg->points[j].reflectivity / 10000.0;
            pt.curvature = static_cast<float>(livox_msg->points[j].offset_time / (float)time_end) * 0.1;
            pcl_in.push_back(pt); 
        }
    }

    unsigned long timebase_ns = livox_data[0]->timebase;
    ros::Time timestamp;
    timestamp.fromNSec(timebase_ns);

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
    pcl_ros_msg.header.frame_id = livox_msg_in->header.frame_id;
    pub.publish(pcl_ros_msg);
    livox_data.clear();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "livox_to_pointcloud2");
    ros::NodeHandle nh;
    sub = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox_horizon_points", 100, cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 100);
    ros::spin();
    return 0;
}
