#include <iostream>
#include <cstring>
#include <fstream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "ros/console.h"
using namespace std;
ros::Subscriber initPosSub;
ros::Subscriber deltaTimeSub;

ros::Publisher  odomPub;
ros::Publisher  destPub;
nav_msgs::Odometry  odom;
geometry_msgs::Point     dest;

//

const int _move_along_x = 1;
const int _move_to_dest = 2;

int bridge_mode;
double test_distance = 0.0;

void getDest()
{
    if (bridge_mode == _move_along_x)
    {
        dest    = odom.pose.pose.position;
        dest.x  += test_distance;
    }else if(bridge_mode == _move_to_dest)
    {
	ros::NodeHandle handle("~");
	string destFile;
	handle.getParam("/init/dest_file", destFile);
	ROS_WARN("[dest_file] = %s", destFile.c_str());
	ifstream fin(destFile);
	fin >> dest.x >> dest.y >> dest.z;
	fin.close();
    }
    return ;
}

void InitPosCallback(const nav_msgs::Odometry & msg)
{
    odom    = msg;
    odom.header.stamp   = ros::Time(0.0);

    getDest();
    ROS_WARN("[dest] = %.3lf %.3lf %.3lf", dest.x, dest.y, dest.z);

    ros::Rate loop_rate(0.5);

    while (ros::ok() && odomPub.getNumSubscribers() == 0) 
        loop_rate.sleep();
    odomPub.publish(odom);

    while (ros::ok() && destPub.getNumSubscribers() == 0)
        loop_rate.sleep();
    destPub.publish(dest);
}

void DeltaTimeCallback(const std_msgs::Float64 & msg)
{
    odom.header.stamp   = ros::Time(msg.data);
    //ROS_WARN("The received time : %.3lf.", msg.data);
    odomPub.publish(odom);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "voxel_bridge");
    ros::NodeHandle handle("~");

    handle.param("/test/test_distance", test_distance, 1.0);

    handle.param("/bridge_mode", bridge_mode, _move_along_x);

    initPosSub  = 
        handle.subscribe("init_pos", 2, & InitPosCallback);

    deltaTimeSub   = 
        handle.subscribe("delta_time", 2, & DeltaTimeCallback);

    odomPub =
        handle.advertise<nav_msgs::Odometry>("odom", 2);

    destPub = 
        handle.advertise<geometry_msgs::Point>("dest", 2);

    ros::spin();

    return 0;
}
