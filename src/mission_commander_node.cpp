#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <random>

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

sensor_msgs::PointCloud2 getPointCloud(float * pt, int n)
{
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
    uint8_t * pc_   = (uint8_t*)  pt;

    for (size_t idx = 0; idx < ptCloud.row_step; ++idx)
    {ptCloud.data.push_back(pc_[idx]);}
    
    return ptCloud;
}

sensor_msgs::PointCloud2 getBlockCloud(float * blk, int n)
{
    sensor_msgs::PointCloud2 blkCloud;

    blkCloud.header.frame_id    = "/map";
    blkCloud.header.stamp   = ros::Time::now();

    blkCloud.height = 1;
    blkCloud.width  = n;

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

    return blkCloud;
}

void deployRandomForest(ros::NodeHandle & handle)
{
    int n_rf;
    double x, y, h, r, H, W;

    double bdy[6];

    handle.getParam("/map_x_lower_bound", bdy[0]);
    handle.getParam("/map_x_upper_bound", bdy[1]);
    handle.getParam("/map_y_lower_bound", bdy[2]);
    handle.getParam("/map_y_upper_bound", bdy[3]);
    handle.getParam("/map_z_lower_bound", bdy[4]);
    handle.getParam("/map_z_upper_bound", bdy[5]);

    double h_mean, h_var;
    double r_mean, r_var;
    double H_mean, H_var;
    double W_mean, W_var;

    handle.getParam("/random_forest/tot_num", n_rf);

    handle.getParam("/random_forest/r_mean", r_mean);
    handle.getParam("/random_forest/r_var", r_var);

    handle.getParam("/random_forest/h_mean", h_mean);
    handle.getParam("/random_forest/h_var", h_var);

    handle.getParam("/random_forest/H_mean", H_mean);
    handle.getParam("/random_forest/H_var", H_var);
    
    handle.getParam("/random_forest/W_mean", W_mean);
    handle.getParam("/random_forest/W_var", W_var);

    float * blk = new float[n_rf * 6 * 2];

    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<double>  rand_x(bdy[0], bdy[1]);
    std::uniform_real_distribution<double>  rand_y(bdy[2], bdy[3]);

    std::normal_distribution<double>    rand_r(r_mean, r_var);
    std::normal_distribution<double>    rand_h(h_mean, h_var);
    std::normal_distribution<double>    rand_H(H_mean, H_var);
    std::normal_distribution<double>    rand_W(W_mean, W_var);

    for (int idx = 0; idx < n_rf; ++ idx)
    {
        x   = rand_x(eng);
        y   = rand_y(eng);
        r   = abs(rand_r(eng));
        h   = abs(rand_h(eng));
        H   = abs(rand_H(eng));
        W   = abs(rand_W(eng));

        ROS_WARN("[%.3lf %.3lf], [%.3lf %.3lf], [%.3lf %.3lf]",
            x, y, r, h, W, H);

        blk[ idx * 12 + 0 ] = x - r;
        blk[ idx * 12 + 1 ] = x + r;
        blk[ idx * 12 + 2 ] = y - r;
        blk[ idx * 12 + 3 ] = y + r;
        blk[ idx * 12 + 4 ] = bdy[4];
        blk[ idx * 12 + 5 ] = bdy[4] + h;

        blk[ idx * 12 + 6 ] = x - W;
        blk[ idx * 12 + 7 ] = x + W;
        blk[ idx * 12 + 8 ] = y - W;
        blk[ idx * 12 + 9 ] = y + W;
        blk[ idx * 12 + 10] = bdy[4] + h;
        blk[ idx * 12 + 11] = bdy[4] + h + H;
    }

    sensor_msgs::PointCloud2 blkCloud = getBlockCloud(blk, n_rf * 2);

    ros::Publisher pub = 
        handle.advertise<sensor_msgs::PointCloud2>(
            "/trajectory_generator/obstacle_block_cloud", 2);

    ros::Rate wait_rate(1);
    while (ros::ok() && pub.getNumSubscribers() == 0) 
        wait_rate.sleep();

    pub.publish(blkCloud);
    delete[] blk;
}

void randomForesetMode(ros::NodeHandle & handle)
{
    deployRandomForest(handle);
}

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

    sensor_msgs::PointCloud2 ptCloud = getPointCloud(pc, cloud->points.size());

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


    bool isDest = false;
    handle.getParam("/dest_signal", isDest);
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
    
    sensor_msgs::PointCloud2 blkCloud = getBlockCloud(blk, n_blk);

    ros::Rate loop_rate(1);
    
    while (ros::ok() && blk_pub.getNumSubscribers() == 0)
        loop_rate.sleep();

    blk_pub.publish(blkCloud);

    if (isDest)
    {
        while (ros::ok())
        {
            if (dest_pub.getNumSubscribers()>0)
            {
                dest_pub.publish(pt);
                break;
            }
            loop_rate.sleep();
        }
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

    sensor_msgs::PointCloud2 ptCloud = getPointCloud(pt32, n);

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

    if (mode == 4)
    {
        randomForesetMode(handle);
    }
    else if (mode == 3)
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
