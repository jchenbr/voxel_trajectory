
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <map>
#include <algorithm>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"
#include "mavlink_message/quad_state.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define _USE_DEBUG_PRINT_
using namespace std;

enum WorkMode {RcvObs, SetTraj, SendOdom, Zombie};
const int _CMD_12 = 0x01;
const int _CMD_23 = 0x02;
const int _CMD_32 = 0x03;
const int _CMD_0  = 0x04;
WorkMode workMode;

ros::Subscriber oSubMode;
ros::Subscriber oSubObs;
ros::Subscriber oSubDest;
ros::Subscriber oSubTime;
ros::Subscriber oSubOdom;

ros::Publisher oPubObsVis;
ros::Publisher iPubBlk;
ros::Publisher iPubDest;
ros::Publisher iPubOdom;

vector<Eigen::Vector3d> vecBlk;
string blkTypeObs;
map<string, double> blkSize;
double blkResolution;
int blkID;

visualization_msgs::Marker  msgPubMarker;
sensor_msgs::PointCloud2    msgPubBlk;
geometry_msgs::Point        msgPubDest;
nav_msgs::Odometry          msgPubOdom;
nav_msgs::Path              msgPubPath;

void ModeCallback(const mavlink_message::quad_state & msgRcvMode);
void ObsCallback(const geometry_msgs::PoseStamped & msgRcvObs);
void DestCallback(const geometry_msgs::PoseStamped & msgRcvDest);
void TimeCallback(const std_msgs::Float64 & msgRcvTime);
void OdomCallback(const nav_msgs::Odometry & msgRcvOdom);

inline void Initialize()
{
    ros::NodeHandle handle("~");
    // set-up publishers
    oSubMode    = handle.subscribe(
            "mode_rcv", 2, &ModeCallback);
    oSubObs     = handle.subscribe(
            "obs_rcv", 2, &ObsCallback);
    oSubDest    = handle.subscribe(
            "dest_rcv", 2, &DestCallback);
    oSubTime    = handle.subscribe(
            "time_rcv", 2, &TimeCallback);
    oSubOdom    = handle.subscribe(
            "odom_rcv", 2, &OdomCallback);

    // set-up subscribers
    oPubObsVis  = handle.advertise<
            visualization_msgs::Marker>(
            "obs_vis_pub", 2);
    iPubBlk     = handle.advertise<
            sensor_msgs::PointCloud2>(
            "obs_blk_pub", 2);
    iPubDest    = handle.advertise<
            nav_msgs::Path>(
            "dest_pub", 2);
    iPubOdom    = handle.advertise<
            nav_msgs::Odometry>(
            "odom_pub", 2);

    // set-up variables 
    msgPubMarker.header.frame_id    = "/map";
    msgPubMarker.header.stamp   = ros::Time::now();
    msgPubMarker.ns     = "voxel_path";
    msgPubMarker.type   = visualization_msgs::Marker::CUBE;
    msgPubMarker.action = visualization_msgs::Marker::ADD;
    msgPubMarker.color.a = 0.5;
    msgPubMarker.color.b = 0.0;
    msgPubMarker.color.r = 0.0;
    msgPubMarker.color.g = 1.0;
    blkID   = 0;

    handle.param("/obs/resolution", blkResolution, 0.005);
    handle.getParam("/obs/size", blkSize);
    vecBlk.resize(0);
#ifdef _USE_DEBUG_PRINT_
    ROS_WARN("[blk size]");
    for (auto item : blkSize)
        ROS_WARN("[map] {%s: %.3lf}", item.first.c_str(), item.second);
#endif
    
    workMode = RcvObs;
}

inline void SetMsgPubBlk()
{
    const int _TOT_DIM = 3;
    assert(vecBlk.size() % _TOT_DIM == 0);

    msgPubBlk.header    = msgPubMarker.header;

    msgPubBlk.height    = 1;
    msgPubBlk.width     = vecBlk.size() ;
    msgPubBlk.is_dense  = true;
    msgPubBlk.is_bigendian  = false;

    msgPubBlk.point_step    = _TOT_DIM * 4;
    msgPubBlk.row_step  = msgPubBlk.point_step * msgPubBlk.width;
    
    sensor_msgs::PointField f;

    msgPubBlk.fields.resize(_TOT_DIM);
    string f_name[] ={string("x"), string("y"), string("z")}; 
    for (size_t idx = 0; idx < _TOT_DIM; ++idx)
    {
        f.name      = f_name[idx];
        f.offset    = idx << 2;
        f.datatype  = 7;
        f.count     = 1;
        msgPubBlk.fields[idx] = f;
    }

    float * data = new float[vecBlk.size() * _TOT_DIM];
    for (size_t idx = 0; idx < vecBlk.size(); ++idx)
        for (size_t dim = 0; dim < _TOT_DIM; ++dim)
        data[idx * _TOT_DIM + dim] = vecBlk[idx](dim);

    msgPubBlk.data.resize(msgPubBlk.row_step);
    uint8_t * dataRaw = reinterpret_cast<uint8_t*>(data);

    for (size_t idx = 0; idx < msgPubBlk.row_step; ++idx)
       msgPubBlk.data[idx] = dataRaw[idx]; 

    delete [] data;
}

void ModeCallback(const mavlink_message::quad_state & msgRcvMode)
{
    // change & do-sth-meanwhile work mode
    if (msgRcvMode.offboard_state == _CMD_12) 
    {
        if (workMode != RcvObs) return ;

        SetMsgPubBlk();
        iPubBlk.publish(msgPubBlk);

        workMode = SetTraj;
    }
    else if (msgRcvMode.offboard_state == _CMD_23)
    {
        if (workMode != SetTraj) return ;

        //iPubDest.publish(msgPubDest);

        workMode = SendOdom;
    }
    else if (msgRcvMode.offboard_state == _CMD_32)
    {
        if (workMode != SendOdom) return ;

        msgPubOdom.header.stamp = ros::Time(1e4);
        iPubOdom.publish(msgPubOdom);

        workMode = SetTraj;
    }else if (msgRcvMode.offboard_state == _CMD_0)
    {
        workMode = Zombie;
    }
    ROS_WARN("[MODE] %d", workMode);
}

void ObsCallback(const geometry_msgs::PoseStamped & msgRcvObs)
{
    // add & vis obstacle when in revice-obstacle mode 
    if (workMode != RcvObs) return ;
    
    Eigen::Vector3d ctr(
            msgRcvObs.pose.position.x,
            msgRcvObs.pose.position.y,
            msgRcvObs.pose.position.z);

    Eigen::Quaterniond att(
            msgRcvObs.pose.orientation.w,
            msgRcvObs.pose.orientation.x,
            msgRcvObs.pose.orientation.y,
            msgRcvObs.pose.orientation.z);

#ifdef _USE_DEBUG_PRINT_

    ROS_WARN("[ctr] %.3lf %.3lf %.3lf", ctr.x(), ctr.y(), ctr.z());
    ROS_WARN("[att] %.3lf %.3lf %.3lf %.3lf", att.w(), att.x(), att.y(), att.z());
#endif

    double length = blkSize[msgRcvObs.header.frame_id+"_length"];
    double width  = blkSize[msgRcvObs.header.frame_id+"_width"];
    double height = blkSize[msgRcvObs.header.frame_id+"_height"];
    // add
    const double eps = 1e-11;
    for (double 
            x = -0.5 * length; 
            x <= 0.5 * length + eps; 
            x += blkResolution)
        for (double 
                y = -0.5 * width;
                y <= 0.5 * width + eps;
                y += blkResolution)
            for (double 
                    z = -0.5 * height;
                    z <= 0.5 * height + eps;
                    z += blkResolution)
            {
                vecBlk.push_back(
                        ctr + att * Eigen::Vector3d(x,y,z));
                //clog<<"[to_add] \n"<< ctr + att * Eigen::Vector3d(x,y,z)<<endl;
            }
    //ROS_WARN("[tot_] %d", (int)vecBlk.size());
    ROS_WARN("[length] %.3lf", length); 
    ROS_WARN("[width] %.3lf", width); 
    ROS_WARN("[height] %.3lf", height); 
    // vis
    msgPubMarker.pose   = msgRcvObs.pose;
    msgPubMarker.id     = blkID++;
    msgPubMarker.scale.x = length;
    msgPubMarker.scale.y = width;
    msgPubMarker.scale.z = height;
    oPubObsVis.publish(msgPubMarker);
}

void DestCallback(const geometry_msgs::PoseStamped & msgRcvDest)
{
    // add dest postion when in set-trajectory mode
    if (workMode != SetTraj) return ;
    if (msgRcvDest.pose.position.z < 0.0)
        iPubDest.publish(msgPubPath);
    else
        msgPubPath.poses.push_back(msgRcvDest);
}

void OdomCallback(const nav_msgs::Odometry & msgRcvOdom)
{
    // add start position when in set-trajectory mode; 
    if (workMode != SetTraj) return ;

    msgPubOdom = msgRcvOdom;
    msgPubOdom.twist.twist.linear.x = 0.0; 
    msgPubOdom.twist.twist.linear.y = 0.0; 
    msgPubOdom.twist.twist.linear.z = 0.0; 

    msgPubOdom.header.stamp     = ros::Time(0.0);
    iPubOdom.publish(msgPubOdom);
}

void TimeCallback(const std_msgs::Float64 & msgRcvTime)
{
    // call the kernel when in send-odom mode
    if (workMode != SendOdom) return ;

    msgPubOdom.header.stamp     = ros::Time(msgRcvTime.data);

    iPubOdom.publish(msgPubOdom);
} 

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "voxel_bridge");

    Initialize();

    ros::spin();
    return 0;
}
