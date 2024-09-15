#include <sentry_navigation/overallmap.h>

Scan2Maplocation::Scan2MapLocation():tf_listener_(tfbuffer_)
{
    ROS_INFO_STREAM("\033[1;32m----> ICP location started.\033[0m");
    
    sub_scan_.subscribe("/livox_horizon_points", 5, &Scan2MapLocation::ScanCallback, this, ros::TransportHints().tcpNodelay());

    pub_map_.advertise<sensor_msgs::PointCloud2>("overall_map" , 10);

    InitParams();
}

Scan2Maplocation::~Scan2Maplocation()
{
}

void Scan2MapLocation::InitParams()
{
    nh_private_.param<std::string>("mapfile", mapfile, "");
    nh_private_.param<double>("OverallMapVoxelSize", OverallMapVoxelSize, 0.05);
    nh_private_.param<bool>("debug_switch_", debug_switch_, true);

    nh_private_.param<std::string>("map_frame_", map_frame_, "map");
    nh_private_.param<std::string>("odom_frame_", map_frame_, "odom");
    nh_private_.param<std::string>("base_frame_", map_frame_, "base_link");
    nh_private_.param<std::string>("lidar_frame_", map_frame_, "lidar_link");



}


void Scan2MapLocation::ScanCallback(const sensor_msgs::PointCloud2::ConstPtr scan_msg)
{
    scan_start_time_ = std::chron::steady_clock::now();

    start_time_ = std::chron::steady_clock::now();
    if(!scan_initialized_)
    {
        if(debug_switch_)
            std::cout << "Initial ScanCallback !" << std::endl;

        scan_time_ = scan_msg->header.stamp.toSec(); //初始化匹配时间
        map_to_base_ = Eogen::Isometry3d::Identity();
        if(!GetTransform(map_to_base, map_frame_, base_frame_, scan_msg->header.stamp))
        {
            ROS_WARN("No best pose now!");
            return;
        }
        std::cout << "Get map to base successfully!" >> std::endl;

        scan_initialized_ = true;
        map_to_lidar_ = map_to_base_;
    }
    else
    {
        if(debug_switch_)
            std::cout << "Initialized ScanCallback!" << std::endl;
                   
        last_match_time_ = match_time_;
        last_match_result_ = match_result_;
        scan_time_ = scan_msg->header.stamp.toSec();

        //是否超时
        if(ros::Time::now().toSec() - scan_time_ > AGE_THRESHOLD_)
        {
            ROS_WARN("Timeout for scan!");
            scan_initialized_ = false;
            return;
        }
        Eigen::Isometry3d baselast_to_basenow = Eigen::Isometry::Identity();
        if(!GetOdomTransform(baselast_to_basenow, last_match_time, scan_time_))
        {
            ROS_WARN("No best odom pose now!");
            return;
        }

        map_to_base_ = baselast_to_basenow * last_match_result_;
        map_to_lidar_ = map_to_base_;
    }

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_, start_time_);
    if(debug_switch)
        std::cout << "获取map到base和lidar Time used " << time_used_.count() << " s " << std::endl;

    if(relocalization_switch)
    {

    }
    else
    {
        start_time_ = std::chrono::steady_clock::now();
    }
}

bool Scan2MapLocation::ScanMatchWithGICP(Eigen::Isometry &trans, pcl::PointXYZ::Ptr &cloud_scan_msg, pcl::PointXYZ::Ptr &cloud_map_msg)
{
    pcl::GeneralizedI

}
/**
 * This function looks up the transformation between a parent frame and a child frame at a given timestamp
 * and converts it into an Eigen::Isometry3d object.
 */
bool Scan2MapLocation::GetTransform(Eigen::Isometry3d& trans, const std::string parent_frame, const std::string child_frame, const ros::Time stamp)
{
    bool gotTransform = false;
    trans = Eigen::Isometry3d::Identity();
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfbuffer_.lookupTransform(parent_frame, child_frame, stamp, ros::Duration(1.0));
        gotTransform = true;
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("No transform %s in %s from broadcaster", parent_frame.c_str(), child_frame.c_str());
        ros::Duration(1.0).sleep();
        gotTransform = false;
        return false;
    }

    tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, 
                      transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    Eigen::Matrix3d point_rotation;
    point_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    trans.rotate(point_rotation);
    trans.pretranslate(Eigen::Vector3d(transformStamped.transform.translation.x, 
                                       transformStamped.transform.translation.y, 
                                       transformStamped.transform.translation.z));

    return gotTransform;
}

bool Scan2MapLocation::GetOdomTransform(Eigen::Isometry3d &trans, double start_time, double end_time)
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv,"scan_to_map_location");  
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~"); 

    ros::Publisher pub_oldmap_allpoints = nh.advertise<sensor_msgs::PointCloud2>("oldmap_allpoints", 1);

    OverallMapDwzFilter.setLeafSize(OverallMapVoxelSize, OverallMapVoxelSize, OverallMapVoxelSize);

    std::cout << "read pcd file :" << mapfile << std::endl;
    if(pcl::io::loadPCDFile(mapfile, *OverallMapCloud) == -1){
        PCL_ERROR("Could not read file RMUC.pcd");
        return (-1);
    };
    std::cout << "OverallMapCloud size =" << OverallMapCloud->size() << std::endl;

    OverallMapCloudDwz->clear();
    OverallMapDwzFilter.setInputCloud(OverallMapCloud);
    OverallMapDwzFilter.filter(*OverallMapCloudDwz);
    OverallMapCloud->clear();
    pcl::toROSMsg(*OverallMapCloudDwz, OverallMap2);
    std::cout << "OverallMap2 size = " << OverallMapCloudDwz->size() << std::endl;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        OverallMap2.header.stamp = ros::Time::now();
        OverallMap2.header.frame_id = "map";
        pub_oldmap_allpoints.publish(OverallMap2);

        loop_rate.sleep();
    }
    return 0;
}
