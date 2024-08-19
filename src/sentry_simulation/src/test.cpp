#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

//message
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

bool use_gazebo_time = false;
ros::Time odom_time;
const systenDelay = 5;
bool systemInited = false;
int systemIintCount = 0;

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehicleYawStack[stackNum];
float vehiclePitchStack[stackNum];
float terrianRollStack[stackNum];
float terrianYawStack[stackNum];
float terrianPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;



void Point2Callback(const sensor_msgs::PointCloud2::ConstPtr& point2_msg)
{
    if(!systemInited){
        systemIintCount++;
        if(systemIintCount > systenDelay){
            systemInited = true;
            ROS_INFO("System initialized");
        }
        return ;
    }

    double scan_time = point2_msg->header.stamp.toSec();

    if(odomSendIDPointer < 0){
        return;
    }
    while(odomTimeStack[(odomRecIDPointer + 1)] % stackNum < scan_time &&
          odomRecIDPointer!= (odomSendIDPointer + 1) % stackNum)
    {
        odomRecIDPointer = (odomRecIDPointer +1) % stackNum;
    }

    double odomRecTime = odom_time.toSec();
    float vehicleRecX = vehicleX;
    float vehicleRecY = vehicleY;
    float vehicleRecZ = vehicleZ;
    float vehicleRecRoll = vehicleRoll;
    float vehicleRecYaw = vehicleYaw;
    float vehicleRecPitch = vehiclePitch;

    if(use_gazebo_time){
        odomRecTime = odomTimeStack[odomRecIDPointer];
        vehicleRecX = vehicleXStack[odomRecIDPointer];
        vehicleRecY = vehicleYStack[odomRecIDPointer];
        vehicleRecZ = vehicleZStack[odomRecIDPointer];
        vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
        vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
        vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv,"test");   
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_ = ros::NodeHandle("~");

    ros::Subscriber point2_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox_horizon_points", 2, Point2Callback);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/state_estimation", 5);
    return 0
}
