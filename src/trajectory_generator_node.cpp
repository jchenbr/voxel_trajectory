
#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <deque>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "mavlink_message/PositionCommand.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "voxel_trajectory/voxelserver.h"


using namespace std;

ros::Publisher desiredStatePub;
ros::Publisher mavlinkStatePub;
ros::Publisher mapPointsPub;
ros::Publisher trajectoryPub;

// the server
bool isOdom    = false;
bool isMap = false;
bool isTraj = false;
bool isInit = false;
VoxelTrajectory::VoxelServer * server = NULL;
VoxelTrajectory::VoxelServer * innerServer = NULL; 

// for the map
double mapBdy[_TOT_BDY];

// for the odometry
double estPos[_TOT_DIM];
double estVel[_TOT_DIM];
double estAcc[_TOT_DIM];
double yaw = 0.0, yaw_dot = 0.0;

ros::Time odomTime;
double finalTime;

quadrotor_msgs::PositionCommand posCMD;
mavlink_message::PositionCommand posCMD_ml;

// for the mission
double safeMargin;
double maxAcc;
double maxVel;
double mapVoxelResolution;
deque<double> ptBuff;

void VisualizeMap()
{
    if (!isMap) return ;

    vector<double> pt   = server->getPointCloud();
    float * pt32 = new float[pt.size()];

    for (size_t idx = 0; idx < pt.size(); idx++)
        pt32[idx]   = (float)pt[idx];

    //for (size_t idx = 0; idx < pt.size()/3; idx++) 
    //    ROS_WARN("Send: [%.3lf %.3lf %.3lf]", pt[idx*3 + 0], pt[idx*3 + 1], pt[idx*3 + 2]);

    sensor_msgs::PointCloud2 ptCloud;

    ptCloud.header.frame_id = "/map";
    ptCloud.header.stamp    = odomTime;

    ptCloud.height  = 1;
    ptCloud.width   = pt.size() / 3;

    ptCloud.is_bigendian    = false;
    ptCloud.is_dense    = true;

    ptCloud.point_step  = 4 * _TOT_DIM;
    ptCloud.row_step    = ptCloud.point_step * ptCloud.width;

    sensor_msgs::PointField field;
    field.name      = "x";
    field.offset    = 0;
    field.datatype  = 7; // float
    field.count     = 1;
    ptCloud.fields.push_back(field);

    field.name      = "y";
    field.offset    += 4;
    ptCloud.fields.push_back(field);

    field.name      = "z";
    field.offset    += 4;
    ptCloud.fields.push_back(field);

    ptCloud.data.reserve(ptCloud.row_step);
    uint8_t * pt32it = (uint8_t *)pt32;
    for (size_t idx = 0; idx < ptCloud.row_step; idx++)
    {
        ptCloud.data.push_back(*pt32it);
        ++pt32it;
    }
    ros::Rate loop_rate(1);
    while (ros::ok() && mapPointsPub.getNumSubscribers() == 0)
        loop_rate.sleep();
    mapPointsPub.publish(ptCloud);
    delete[] pt32;
}

void PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr & cloud)
{
    ROS_WARN("[Receive TOTAL]: %d", cloud->width); 
    if (! isMap)
    {   
        isMap = true;
    }

    float * data = (float *) cloud -> data.data();
    vector<double > pt( cloud->width  * _TOT_BDY );
    ROS_WARN("[Receive TOTAL]: %d", cloud->width); 
    for (size_t idx = 0; idx < cloud->width; ++idx)
    {
        pt[idx * _TOT_BDY + _BDY_x] = data[idx * _TOT_DIM + _DIM_x] - safeMargin;
        pt[idx * _TOT_BDY + _BDY_X] = data[idx * _TOT_DIM + _DIM_x] + safeMargin;
        pt[idx * _TOT_BDY + _BDY_y] = data[idx * _TOT_DIM + _DIM_y] - safeMargin;
        pt[idx * _TOT_BDY + _BDY_Y] = data[idx * _TOT_DIM + _DIM_y] + safeMargin;
        pt[idx * _TOT_BDY + _BDY_z] = data[idx * _TOT_DIM + _DIM_z] - safeMargin;
        pt[idx * _TOT_BDY + _BDY_Z] = data[idx * _TOT_DIM + _DIM_z] + safeMargin;
    }

    server->addMapBlock(pt);
    VisualizeMap();
}

void blockCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr & cloud)
{   
    if (! isMap)
    {   
        isMap = true;
    }
    ROS_WARN("<BUILD SUCCEED> %.3lf", mapVoxelResolution);
    float * data = (float *) cloud -> data.data();
    vector<double> blk( cloud->width * _TOT_BDY);
    for (size_t idx = 0; idx < cloud->width; ++idx)
    {
        for (size_t j = 0; j < _TOT_BDY; j++)
        if (j & 1)
           blk[idx * _TOT_BDY + j] = data[ idx * _TOT_BDY + j] + safeMargin;
        else
           blk[idx * _TOT_BDY + j] = data[ idx * _TOT_BDY + j] - safeMargin;

            ROS_WARN("Receive: [%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf]", 
            blk[idx * _TOT_BDY + 0], blk[idx * _TOT_BDY + 1],
            blk[idx * _TOT_BDY + 2], blk[idx * _TOT_BDY + 3],
            blk[idx * _TOT_BDY + 4], blk[idx * _TOT_BDY + 5]);
    }
    server->addMapBlock(blk);
    ROS_WARN("[ADD BLOCK SUCCEED!]");
    VisualizeMap();
}

void VisualizeTraj();

void trySetTraj()
{
    if (ptBuff.empty()) return;
    vector<double> begState(_TOT_DIM * 3);
    vector<double> endState(_TOT_DIM * 3, 0.0);

    begState[0 * _TOT_DIM + _DIM_x] = estPos[_DIM_x];
    begState[0 * _TOT_DIM + _DIM_y] = estPos[_DIM_y];
    begState[0 * _TOT_DIM + _DIM_z] = estPos[_DIM_z];

    if (!isTraj){
        begState[1 * _TOT_DIM + _DIM_x] = estVel[_DIM_x];
        begState[1 * _TOT_DIM + _DIM_y] = estVel[_DIM_y];
        begState[1 * _TOT_DIM + _DIM_z] = estVel[_DIM_z];

        begState[2 * _TOT_DIM + _DIM_x] = estAcc[_DIM_x];
        begState[2 * _TOT_DIM + _DIM_y] = estAcc[_DIM_y];
        begState[2 * _TOT_DIM + _DIM_z] = estAcc[_DIM_z];
    }else
    {
        vector<double> state = server -> getDesiredState(odomTime.toSec());

        begState[1 * _TOT_DIM + _DIM_x] = state[1 * _TOT_DIM + _DIM_x];
        begState[1 * _TOT_DIM + _DIM_y] = state[1 * _TOT_DIM + _DIM_y];
        begState[1 * _TOT_DIM + _DIM_z] = state[1 * _TOT_DIM + _DIM_z];

        begState[2 * _TOT_DIM + _DIM_x] = state[2 * _TOT_DIM + _DIM_x];
        begState[2 * _TOT_DIM + _DIM_y] = state[2 * _TOT_DIM + _DIM_y];
        begState[2 * _TOT_DIM + _DIM_z] = state[2 * _TOT_DIM + _DIM_z];
    }

    endState[0 * _TOT_DIM + _DIM_x] = ptBuff[0]; 
    endState[0 * _TOT_DIM + _DIM_y] = ptBuff[1];
    endState[0 * _TOT_DIM + _DIM_z] = ptBuff[2];

    isTraj  = false;
    int server_ret = server->setPoints(begState, endState, odomTime.toSec());
    
    ROS_WARN("The traj status is %d.", server_ret);
    if (server_ret == 0) return ; 
    //erase the current dest
    if (server_ret == 2)
    {
        ptBuff.pop_front();
        ptBuff.pop_front();
        ptBuff.pop_front();
    }

    isTraj  = true;
    VisualizeTraj();
}

void pubMavlinkCMD()
{
    posCMD_ml.header    = posCMD.header;
    posCMD_ml.position  = posCMD.position;
    posCMD_ml.velocity  = posCMD.velocity;
    posCMD_ml.accelerate  = posCMD.acceleration;
    posCMD_ml.yaw   = posCMD.yaw;
    posCMD_ml.kx    = posCMD.kx;
    posCMD_ml.kv    = posCMD.kv;

    mavlinkStatePub.publish(posCMD_ml);
}

void OdometryCallback(
    const nav_msgs::Odometry::ConstPtr & odom)
{
    //ROS_WARN("[Recevid Odometry Time] %.3lf", odom->header.stamp.toSec());
    isOdom = true;

    odomTime   = odom->header.stamp;

    estPos[_DIM_x] = odom->pose.pose.position.x;
    estPos[_DIM_y] = odom->pose.pose.position.y;
    estPos[_DIM_z] = odom->pose.pose.position.z;

    estVel[_DIM_x] = odom->twist.twist.linear.x;
    estVel[_DIM_y] = odom->twist.twist.linear.y;
    estVel[_DIM_z] = odom->twist.twist.linear.z;

    estAcc[_DIM_x] = 0.0;
    estAcc[_DIM_y] = 0.0;
    estAcc[_DIM_z] = 0.0;
#if 0
    ROS_WARN("[Orientation] %.3lf %.3lf %.3lf %.3lf", 
        odom->pose.pose.orientation.x, 
        odom->pose.pose.orientation.y, 
        odom->pose.pose.orientation.z, 
        odom->pose.pose.orientation.w 
        );
#endif
    yaw = tf::getYaw(odom->pose.pose.orientation);

    if (isInit)
    {
        desiredStatePub.publish(posCMD);
    }
#if 1
    if (isTraj)
    {
        vector<double> state = server->getDesiredState(odomTime.toSec());

        if (odomTime.toSec() > server->getFinalTime()) 
        {
            isTraj = false;
            state[3 + _DIM_x] = state[3 + _DIM_y] = state[3 + _DIM_z] = 0.0;
            state[6 + _DIM_x] = state[6 + _DIM_y] = state[6 + _DIM_z] = 0.0;
            trySetTraj();
        }

        posCMD.position.x  = state[0 + _DIM_x];
        posCMD.position.y  = state[0 + _DIM_y];
        posCMD.position.z  = state[0 + _DIM_z];

        posCMD.velocity.x  = state[3 + _DIM_x];
        posCMD.velocity.y  = state[3 + _DIM_y];
        posCMD.velocity.z  = state[3 + _DIM_z];

        posCMD.acceleration.x  = state[6 + _DIM_x];
        posCMD.acceleration.y  = state[6 + _DIM_y];
        posCMD.acceleration.z  = state[6 + _DIM_z];

#if 0
        ROS_WARN("Chakki, Chakki! [%.3lf %.3lf %.3lf, %.3lf %.3lf %.3lf, %.3lf %.3lf %.3lf]\n", 
            posCMD.position.x, posCMD.position.y, posCMD.position.z, 
            posCMD.velocity.x, posCMD.velocity.y, posCMD.velocity.z,
            posCMD.acceleration.x, posCMD.acceleration.y, posCMD.acceleration.z);
#endif

        posCMD.yaw = yaw;
        posCMD.yaw_dot = 0;

        desiredStatePub.publish(posCMD);
        pubMavlinkCMD();
    }
#endif
}


inline double getTotAcc(double x, double y , double z)
{
    return sqrt( x * x + y * y + z * z);
}

void VisualizeTraj()
{
    if (!isTraj) return ;

    double t_begin = server->getBeginTime();
    double t_final = server->getFinalTime();

    nav_msgs::Path path;
    path.header.stamp   = odomTime;
    path.header.frame_id    = string("/map");

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;

    vector<double> state;

    int count = 0, iSafe = 0;
    for (double t = t_begin; t < t_final; t += 0.01, count += 1)
    {
        state = server->getDesiredState(t);
        pose.pose.position.x    = state[0 * _TOT_DIM + _DIM_x];
        pose.pose.position.y    = state[0 * _TOT_DIM + _DIM_y];
        pose.pose.position.z    = state[0 * _TOT_DIM + _DIM_z];
#ifdef _FLAG_ACC_CHECK_
        if (getTotAcc(state[3], state[4], state[5]) > maxAcc * 1.5)
        {
            iSafe = count;
            break;
        }
#endif

        path.poses.push_back(pose);
    }

    ROS_WARN("<<GOT TRAJ!>>");
    trajectoryPub.publish(path);
    
#ifdef _FLAG_ACC_CHECK_
    if (iSafe)
    {
        ROS_WARN("<<TRAJ EXCEED THE ACC LIM!>>");
        isTraj = false;

        state   = server->getDesiredState(t_final);
        ptBuff.push_front(state[0 * _TOT_DIM + _DIM_z]);
        ptBuff.push_front(state[0 * _TOT_DIM + _DIM_y]);
        ptBuff.push_front(state[0 * _TOT_DIM + _DIM_x]);

        state   = server->getDesiredState(t_begin + 0.01 * iSafe);
        ptBuff.push_front(state[0 * _TOT_DIM + _DIM_z]);
        ptBuff.push_front(state[0 * _TOT_DIM + _DIM_y]);
        ptBuff.push_front(state[0 * _TOT_DIM + _DIM_x]);

        trySetTraj();
    }
#endif
}

void MissionCallback(
    const geometry_msgs::Point::ConstPtr & pt)
{
    isInit  = false;
    ptBuff.clear();
    if (!isMap) return ;

    ROS_WARN("[ONE POINT] %.3lf %.3lf %.3lf", pt->x, pt->y, pt->z);
    ptBuff.push_back(pt->x);
    ptBuff.push_back(pt->y);
    ptBuff.push_back(pt->z);

    trySetTraj();
}

void WaypointCallback(
    const nav_msgs::Path::ConstPtr & waypoints)
{
    isInit  = false;
    ptBuff.clear();
    if (!isMap) return ;
    
    for (size_t idx = 0; idx < waypoints->poses.size(); ++idx)
    {
        ptBuff.push_back( waypoints->poses[idx].pose.position.x );
        ptBuff.push_back( waypoints->poses[idx].pose.position.y );
        ptBuff.push_back( waypoints->poses[idx].pose.position.z );
    }

    trySetTraj();
}
//void command_callback()

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle handle("~");
    ros::NodeHandle root("/");

    posCMD.header.frame_id = "/voxel_trajectory";

    handle.param("/init/signal", isInit, false);
    handle.param("/init/pos/x", posCMD.position.x, 0.0);
    handle.param("/init/pos/y", posCMD.position.y, 0.0);
    handle.param("/init/pos/z", posCMD.position.z, 0.0);

    clog << "x = " << posCMD.position.x << endl;
    clog << "y = " << posCMD.position.y << endl;
    clog << "z = " << posCMD.position.z << endl;
    clog << "signal = " << isInit <<endl;

    handle.param("/init/vel/x", posCMD.velocity.x, 0.0);
    handle.param("/init/vel/y", posCMD.velocity.y, 0.0);
    handle.param("/init/vel/z", posCMD.velocity.z, 0.0);
    handle.param("/init/acc/x", posCMD.acceleration.x, 0.0);
    handle.param("/init/acc/y", posCMD.acceleration.y, 0.0);
    handle.param("/init/acc/z", posCMD.acceleration.z, 0.0);
    handle.param("/init/yaw", posCMD.yaw, 0.0);
    handle.param("/init/yaw_dot", posCMD.yaw_dot, 0.0);

    handle.param("/gains/pos/x", posCMD.kx[_DIM_x], 3.7);
    handle.param("/gains/pos/y", posCMD.kx[_DIM_y], 3.7);
    handle.param("/gains/pos/z", posCMD.kx[_DIM_z], 5.2);
    handle.param("/gains/vel/x", posCMD.kv[_DIM_x], 2.4);
    handle.param("/gains/vel/y", posCMD.kv[_DIM_y], 2.4);
    handle.param("/gains/vel/z", posCMD.kv[_DIM_z], 3.0);

    root.param("map_x_lower_bound", mapBdy[_BDY_x], -100.0);
    root.param("map_y_lower_bound", mapBdy[_BDY_y], -100.0);
    root.param("map_z_lower_bound", mapBdy[_BDY_z], 0.0);
    root.param("map_x_upper_bound", mapBdy[_BDY_X], 100.0);
    root.param("map_y_upper_bound", mapBdy[_BDY_Y], 100.0);
    root.param("map_z_upper_bound", mapBdy[_BDY_Z], 200.0);

    handle.param("/voxel/maxVelocity", maxVel, 1.0);
    handle.param("/voxel/maxAccleration", maxAcc, 1.0);

    handle.param("/voxel/safeMargin", safeMargin, 0.3);
    handle.param("/voxel/resolution",  mapVoxelResolution, 0.05);
        
        server = new VoxelTrajectory::VoxelServer;
        server->setResolution(mapVoxelResolution);
        server->setMapBoundary(mapBdy);
        server->setMaxAcceleration(maxVel);
        server->setMaxVelocity(maxAcc);

        innerServer = new VoxelTrajectory::VoxelServer;
        innerServer->setMapBoundary(mapBdy);
        innerServer->setResolution(mapVoxelResolution);

    ros::Subscriber pointCloudSub = 
        handle.subscribe("obstacle_point_cloud", 2, & PointCloudCallback);
    ros::Subscriber blockCloudSub = 
        handle.subscribe("obstacle_block_cloud", 2, & blockCloudCallback);
    ros::Subscriber odometrySub    =
        handle.subscribe("odometry", 10, & OdometryCallback);
    ros::Subscriber destSub =
        handle.subscribe("dest", 2, & MissionCallback);
    ros::Subscriber waypointSub = 
        handle.subscribe("waypoints", 2, & WaypointCallback);
    //ros::Subscriber command_sub = 
    //    handle.subscribe("command", & command_callback);

    desiredStatePub = handle.advertise
        <quadrotor_msgs::PositionCommand>("desired_state", 10);

    mavlinkStatePub = handle.advertise
        <mavlink_message::PositionCommand>("mavlink_state", 50);

    mapPointsPub  = handle.advertise
        <sensor_msgs::PointCloud2>("map_points", 2);

    trajectoryPub  = handle.advertise
        <nav_msgs::Path>("trajectory", 2);

    ros::spin();

    ROS_WARN("WHAT'S WRONG!?");
    return 0;
}

