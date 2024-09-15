//ros
#include <ros/ros.h>

//message
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

//PCL library
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>


//tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
class Scan2MapLocation
{

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    ros::Subscriber sub_scan_;

    ros::Publisher pub_map_;

    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    Eigen::Isometry3d map_to_base_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d map_to_lidar_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_to_lidar_ = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d match_result_ = Eigen::Isometry3d::Identity();     //GICP匹配结果
    Eigen::Isometry3d last_match_result_ = Eigen::Isometry3d::Identity();     //上一帧GICP匹配结果


    //加载pcd地图
    std::string mapfile;
    double OverallMapVoxelSize = 0;
    sensor_msgs::PointCloud2 OverallMap2;
    pcl::VoxelGrid<pcl::PointXYZ> OverallMapDwzFilter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr OverallMapCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr OverallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>);

    //if initialized
    bool map_initialized_ = false;
    bool scan_initialized_ = false;
    bool odom_initialized_ = false;
    bool relocalization_switch = false;
    bool relocalization_result = false;

    bool debug_switch_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string lidar_frame_;
    
    //计算时间
    std::chrono::steady_clock::time_point scan_start_time_;
    std::chrono::steady_clock::time_point scan_end_time_;
    std::chrono::steady_clock::time_point scan_last_end_time_;
    std::chrono::duration<double> scan_time_used_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    double match_time_;        //当前匹配的时间
    double last_match_time_;   //上一帧匹配的时间

    double scan_time_;  //当前用于匹配的雷达数据时间
    double AGE_THRESHOLD_;

    void InitParams();

    bool GetTransform(const Eigeh::Isometry3d &trans, const std::string parent_frame, const std::string child_frame, const ros::Time stamp)

    bool GetOdomTransform(Eigen::Isometry3d &trans, double start_time, double end_time)

    bool ScanMatchWithGICP(Eigen::Isometry &trans, pcl::PointXYZ::Ptr &cloud_scan_msg, pcl::PointXYZ::Ptr &cloud_map_msg)
    

public:
    Scan2MapLocation();
    ~Scan2MapLocation();

    void ScanCallback(const sensor_msgs::PointCloud2::ConstPtr scan_msg);

}
