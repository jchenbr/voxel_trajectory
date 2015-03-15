#include <cstring>
#include <string>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "ros/console.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

void default_mode(ros::NodeHandle & handle);

nav_msgs::Odometry odom;

void OdomCallback(const nav_msgs::Odometry & msg)
{
    odom = msg;
}

void real_quad(ros::NodeHandle & handle)
{
    default_mode(handle);

    ros::Publisher  posPub = 
        handle.advertise<nav_msgs::Odometry>("init_pos", 2);

    ros::Publisher  timePub = 
        handle.advertise<std_msgs::Float64>("delta_time", 2);

    ros::Subscriber odomeSub =
        handle.subscribe("odom", 2, &OdomCallback);

    odom.header.stamp   = ros::Time(0.0);
    odom.pose.pose.orientation.w = 1.0;

    ros::Rate   wait_rate(1);
    ros::Rate   loop_rate(100);

    while (ros::ok() && posPub.getNumSubscribers() == 0) 
        wait_rate.sleep();
    posPub.publish(odom);

    double init_time = ros::Time::now().toSec();
    while (ros::ok() && timePub.getNumSubscribers() == 0)
        wait_rate.sleep();

    std_msgs::Float64   deltaTime;
    while (ros::ok())
    {
        deltaTime.data  = (ros::Time::now().toSec()) - init_time;
        timePub.publish(deltaTime);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

void pcl_pcd(ros::NodeHandle & handle)
{
    ros::Publisher pc_pub = 
        handle.advertise<sensor_msgs::PointCloud2>(
            "/trajectory_generator/obstacle_point_cloud", 2);

    string pc_file;
    handle.getParam("/point_cloud_file", pc_file);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pc_file.c_str(), *cloud) == -1)
    {
        clog << "[ERROR] can't find point cloud file:\n"<<pc_file<<endl;
        return ;
    }

    float * pc  = new float[cloud->points.size() * 3];

    for (size_t idx = 0; idx < cloud->points.size() ; ++idx)
    {
        pc[idx * 3 + 0] = cloud->points[idx].x;
        pc[idx * 3 + 1] = cloud->points[idx].y;
        pc[idx * 3 + 2] = cloud->points[idx].z;
    }

    sensor_msgs::PointCloud2 ptCloud;

    ptCloud.header.frame_id = "/map";
    ptCloud.header.stamp    = ros::Time::now();

    ptCloud.height  = 1;
    ptCloud.width   = cloud->points.size();

    ptCloud.is_bigendian    = false;
    ptCloud.is_dense    = true;

    ptCloud.point_step  = 3 * 4;
    ptCloud.row_step    = ptCloud.point_step * ptCloud.width;

    sensor_msgs::PointField field;

    field.name      = "x";
    field.offset    = 0;
    field.datatype  = 7;
    field.count     = 1;
    ptCloud.fields.push_back(field);

    field.name      = "y";
    field.offset    += 4;
    ptCloud.fields.push_back(field);

    field.name      = "z";
    field.offset    += 4;
    ptCloud.fields.push_back(field);

    ptCloud.data.reserve(ptCloud.row_step);
    uint8_t * pc_   = (uint8_t*)  pc;

    for (size_t idx = 0; idx < ptCloud.row_step; ++idx)
    {ptCloud.data.push_back(pc_[idx]);}

    ros::Rate loop_rate(1);

    clog<<"Data prepared!"<<endl;

    while (ros::ok() && pc_pub.getNumSubscribers() == 0)
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    clog<<"publishing... n_sub="<<pc_pub.getNumSubscribers()<<endl;
    pc_pub.publish(ptCloud);
        ros::spinOnce();
    //for (int i = 0; i < ptCloud.width; i++) clog<<ptCloud.data[i];clog<<endl;
    clog<<"published... size="<< ptCloud.width<<endl;

    while (ros::ok() )
    {
        loop_rate.sleep();
    }

    delete [] pc;
}

void obs_blk_dest(ros::NodeHandle & handle)
{
    ros::Publisher dest_pub =
        handle.advertise<geometry_msgs::Point>("/trajectory_generator/dest", 2);

    ros::Publisher blk_pub = 
        handle.advertise<sensor_msgs::PointCloud2>("/trajectory_generator/obstacle_block_cloud", 2);

    geometry_msgs::Point pt;
    handle.getParam("/dest_x", pt.x);
    handle.getParam("/dest_y", pt.y);
    handle.getParam("/dest_z", pt.z);

    string blk_file;
    handle.getParam("/blk_cloud_file", blk_file);
    ifstream fin(blk_file.c_str());

    int n_blk = 0;
    fin>>n_blk;

    ROS_WARN("[BLK_FILE] %s [tot] %d", blk_file.c_str(), n_blk);

    
    float * blk = new float[ n_blk * 6];
    for (size_t idx = 0; idx < n_blk; ++ idx)
    {
        for (int j = 0; j < 6; j++) fin >> blk[idx * 6 + j];
#if 1
        ROS_WARN("[tot] %d [blk] %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
            n_blk,
            blk[idx * 6 + 0], blk[idx * 6 + 1], 
            blk[idx * 6 + 2], blk[idx * 6 + 3], 
            blk[idx * 6 + 4], blk[idx * 6 + 5]);
#endif
    }
    
    sensor_msgs::PointCloud2 blkCloud;

    blkCloud.header.frame_id    = "/map";
    blkCloud.header.stamp   = ros::Time::now();

    blkCloud.height = 1;
    blkCloud.width  = n_blk;

    blkCloud.is_bigendian    = false;
    blkCloud.is_dense    = true;

    blkCloud.point_step  = 6 * 4;
    blkCloud.row_step   = blkCloud.point_step * blkCloud.width;

    sensor_msgs::PointField field;

    field.name  = "x";
    field.offset    = 0;
    field.datatype  = 7;
    field.count = 1;
    blkCloud.fields.push_back(field);

    field.name  = "X";
    field.offset  += 4;
    blkCloud.fields.push_back(field);

    field.name  = "y";
    field.offset  += 4;
    blkCloud.fields.push_back(field);

    field.name  = "Y";
    field.offset  += 4;
    blkCloud.fields.push_back(field);

    field.name  = "z";
    field.offset  += 4;
    blkCloud.fields.push_back(field);

    field.name  = "Z";
    field.offset  += 4;
    blkCloud.fields.push_back(field);

    blkCloud.data.reserve(blkCloud.row_step);
    uint8_t * blk_ = (uint8_t *) blk;

    for (size_t idx = 0; idx < blkCloud.row_step; ++idx)
    {blkCloud.data.push_back(blk_[idx]);}

    ros::Rate loop_rate(1);
    
    while (ros::ok() && blk_pub.getNumSubscribers() == 0)
        loop_rate.sleep();

    blk_pub.publish(blkCloud);

    while (ros::ok())
    {
        if (dest_pub.getNumSubscribers()>0)
        {
            dest_pub.publish(pt);
            break;
        }
        loop_rate.sleep();
    }
    ros::spinOnce();
    while (ros::ok() )
    {
        loop_rate.sleep();
    }
    delete[] blk;
}

ros::Publisher pointCloudPub;

void default_mode(ros::NodeHandle & handle)
{
    pointCloudPub   = 
        handle.advertise<sensor_msgs::PointCloud2>("/trajectory_generator/obstacle_point_cloud", 2);
    string point_cloud_file;
    handle.getParam("/point_cloud_file", point_cloud_file);
    ifstream fin(point_cloud_file.c_str());
    int n;
    fin >> n ;
    ROS_WARN("n = %d",n);

    float * pt32 = new float[n * 3];
    for (size_t idx = 0; idx < n; ++idx)
    {
        fin >> pt32[ idx * 3 + 0] 
            >> pt32[ idx * 3 + 1]
            >> pt32[ idx * 3 + 2];
        ROS_WARN("pt = {%lf, %lf, %lf}",
                            pt32[idx * 3 + 0],
                            pt32[idx * 3 + 1],
                            pt32[idx * 3 + 2]);
    }

    sensor_msgs::PointCloud2 ptCloud;

    ptCloud.header.frame_id = "/map";
    ptCloud.header.stamp    = ros::Time::now();

    ptCloud.height  = 1;
    ptCloud.width   = n;

    ptCloud.is_bigendian    = false;
    ptCloud.is_dense    = true;

    ptCloud.point_step  = 3 * 4;
    ptCloud.row_step    = ptCloud.point_step * ptCloud.width;

    sensor_msgs::PointField field;

    field.name      = "x";
    field.offset    = 0;
    field.datatype  = 7;
    field.count     = 1;
    ptCloud.fields.push_back(field);

    field.name      = "y";
    field.offset    += 4;
    ptCloud.fields.push_back(field);

    field.name      = "z";
    field.offset    += 4;
    ptCloud.fields.push_back(field);

    ptCloud.data.reserve(ptCloud.row_step);
    uint8_t * pt32it    = (uint8_t *)pt32;
    for (size_t idx = 0; idx < ptCloud.row_step; ++idx)
    {ptCloud.data.push_back(pt32it[idx]);}

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        if (pointCloudPub.getNumSubscribers()>0)
        {
            pointCloudPub.publish(ptCloud);
            ros::spinOnce();
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    delete[] pt32;
}

int main(int argc, char ** argv)
{
    char node_name[] = "mission_commander_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle handle("~");
    ros::NodeHandle root("/");


    int mode;
    //handle.param("test_mode", mode, 0);
    root.param("test_mode", mode, 0);

    ROS_WARN("[work_mode] %d", mode);

    if (mode == 3)
    {
        real_quad(handle);
    }
    else if (mode == 2)
    {
        pcl_pcd(handle);
    }
    else if (mode== 1)
    {
        obs_blk_dest(handle);
    }
    else
    {
        default_mode(handle);
    }
    return 0;
}
