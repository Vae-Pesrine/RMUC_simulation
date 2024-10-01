#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>

std::vector<livox_ros_driver2::CustomMsg::ConstPtr> livox_data;

void cb(const livox_ros_driver2::CustomMsg::ConstPtr livox_msg_in)
{
    livox_data.push_back(livox_msg_in);
    if(livox_data.size() < 1){
        return;
    }

    pcl::PointCloud<pcl::PointXYZINormal> pcl_in;
    for(auto i : livox_data.size()){
        auto livox_msg = livox_data[i];
        auto time_end = livox_msg->points.back().offset_time;

        for(auto j : livox_msg->point_num){
            pcl::PointCloud<pcl::PointXYZINormal> pt;
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
    pcl_ros_msg.header.frame_id = "/livox";
    pub.publish(  v0-0-p09-09o0-98io09yu7ioyu7iop bnm,);
    livox_data.clear();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "livox_to_pointcloud2");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<livox_ros_driver2::CustomMsg>("/livox_horizon_points", 100, cb);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 100);
    ros::spin();
    return 0;
}
