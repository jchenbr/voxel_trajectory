
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <algorithm>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "mavlink_message/PositionCommand.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "voxel_trajectory/voxelserver.h"


using namespace std;

ros::Publisher desiredStatePub;
ros::Publisher mavlinkStatePub;
ros::Publisher mapPointsPub;
ros::Publisher trajectoryPub;
ros::Publisher voxelPathPub;
ros::Publisher lineStripPub;
ros::Publisher checkPointPub;

// the server
bool isOdom    = false;
bool isMap = false;
bool isTraj = false;
bool isInit = false;
bool isVis  = true;

//for dump
bool isDump = false;
ros::Time timeDump(0.0);
ofstream fout;



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
vector<double> ptBuff;

void VisualizeMap()
{
    if (!isMap || !isVis) return ;

    vector<double> pt   = server->getPointCloud();

    float * pt32 = new float[pt.size()];

    for (size_t idx = 0; idx < pt.size(); idx++)
        pt32[idx]   = (float)pt[idx];

    //for (size_t idx = 0; idx < pt.size()/3; idx++) 
    //    ROS_WARN("Send: [%.3lf %.3lf %.3lf]", pt[idx*3 + 0], pt[idx*3 + 1], pt[idx*3 + 2]);
    ROS_WARN("The number of Grid: %d", static_cast<int>(pt.size() / 3));
    double bdy[_TOT_BDY] = {1e9, -1e9, 1e9, -1e9};
    for (size_t idx = 0; idx * 3 < pt.size(); idx++)
    {
        bdy[_BDY_x] = min(bdy[_BDY_x], pt[idx * 3 + _BDY_x]);
        bdy[_BDY_y] = min(bdy[_BDY_y], pt[idx * 3 + _BDY_y]);
        bdy[_BDY_z] = min(bdy[_BDY_z], pt[idx * 3 + _BDY_z]);

        bdy[_BDY_X] = max(bdy[_BDY_X], pt[idx * 3 + _BDY_X]);
        bdy[_BDY_Y] = max(bdy[_BDY_Y], pt[idx * 3 + _BDY_Y]);
        bdy[_BDY_Z] = max(bdy[_BDY_Z], pt[idx * 3 + _BDY_Z]);
    }
    ROS_WARN("The boundary of Map: [%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf]",
            bdy[_BDY_x], bdy[_BDY_X], bdy[_BDY_y], bdy[_BDY_Y], bdy[_BDY_z], bdy[_BDY_Z]);

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
    vector<double > pt;
    pt.reserve( cloud->width  * _TOT_BDY );
    ROS_WARN("[Receive TOTAL]: %d", cloud->width); 
    double bdy[6] = {-86.863, -36.186, -69.212, 9.111, 0 ,0}; 
    double x, y;
    for (size_t idx = 0; idx < cloud->width; ++idx)
    {
        x = data[idx * _TOT_DIM + _DIM_x];
        y = data[idx * _TOT_DIM + _DIM_y];
        if (x < bdy[_BDY_x] || x > bdy[_BDY_X] || y < bdy[_BDY_y] || y > bdy[_BDY_Y])
            continue;
        x -= bdy[_BDY_x];
        y -= bdy[_BDY_y];
        pt.push_back(x - safeMargin);
        pt.push_back(x + safeMargin);
        pt.push_back(y - safeMargin);
        pt.push_back(y + safeMargin);
        pt.push_back(data[idx * _TOT_DIM + _DIM_z] - safeMargin);
        pt.push_back(data[idx * _TOT_DIM + _DIM_z] + safeMargin);
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

    vector<double> wp;
    wp.reserve(ptBuff.size());
    for (auto & pt : ptBuff) wp.push_back(pt);
    ptBuff.clear();

    isTraj  = false;
    ros::Time begin = ros::Time::now();

    int server_ret = server->setWayPoints(begState, wp, odomTime.toSec());

    ROS_WARN("[The time of trajectory generation : %.9lf]", (ros::Time::now() - begin).toSec());
    
    ROS_WARN("The traj status is %d.", server_ret);
    //erase the current dest
   
   if (server_ret == 2) 
   {    
       isTraj  = true;
       VisualizeTraj();
   }
   else 
       isTraj = false;
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

    posCMD_ml.header.stamp = odomTime;
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
    else if (server->isTraj() == false)
    {
        posCMD.position.x = estPos[_DIM_x];
        posCMD.position.y = estPos[_DIM_y];
        posCMD.position.z = estPos[_DIM_z];

        posCMD.velocity.x = 0.0;
        posCMD.velocity.y = 0.0;
        posCMD.velocity.z = 0.0;
        
        posCMD.acceleration.x = 0.0;
        posCMD.acceleration.y = 0.0;
        posCMD.acceleration.z = 0.0;

        posCMD.yaw = yaw;
        posCMD.yaw_dot = 0.0;
    }
    else
    {
        vector<double> state;
        if (isTraj){
            state = server->getDesiredState(
                    odomTime.toSec());

            if (fout.is_open())
            {
                fout << odom->pose.pose.position.x << " ";
                fout << odom->pose.pose.position.y << " ";
                fout << odom->pose.pose.position.z << " ";
                fout << odom->twist.twist.linear.x << " ";
                fout << odom->twist.twist.linear.y << " ";
                fout << odom->twist.twist.linear.z << " ";
                fout << state[0] << " ";
                fout << state[1] << " ";
                fout << state[2] << " ";
                fout << state[3] << " ";
                fout << state[4] << " ";
                fout << state[5] << " ";
                fout << state[6] << " ";
                fout << state[7] << " ";
                fout << state[8] << " ";
                fout << (odom->header.stamp.toSec() - server->getBeginTime())<< endl;
            }
        }else 
        {
            if (fout.is_open()) fout.close();
                
            state = server->getDesiredState(
                    server->getFinalTime());
        }

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

        posCMD.header.stamp = odomTime;
        desiredStatePub.publish(posCMD);
        pubMavlinkCMD();
    }
}


inline double getTotAcc(double x, double y , double z)
{
    return sqrt( x * x + y * y + z * z);
}

visualization_msgs::MarkerArray arrMarkers;

void VisualizePath()
{
    if (!isTraj || !isVis) return; 
    Eigen::MatrixXd    path    = server->getVoxelPath();
    //clog << "<PATH>:\n" << path;
    
    int M   = path.rows() >> 1;

    for (auto & it : arrMarkers.markers)
        it.action  = visualization_msgs::Marker::DELETE;

    voxelPathPub.publish(arrMarkers);
    arrMarkers.markers.resize(0);
    arrMarkers.markers.reserve(M);

    visualization_msgs::Marker  marker;
    marker.header.frame_id  = "/map";
    marker.header.stamp     = odomTime;
    marker.ns               = "voxel_path";
    marker.type             = visualization_msgs::Marker::CUBE;
    marker.action           = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x   = 0.0;
    marker.pose.orientation.y   = 0.0;
    marker.pose.orientation.z   = 0.0;
    marker.pose.orientation.w   = 1.0;
    marker.color.a  = 0.2;
    marker.color.b  = 0.0;
    marker.color.r  = 0.0;
    marker.color.g  = 4.0;

    for (int idx  = 1; idx <= M; ++idx)
    {
        Eigen::RowVectorXd box = path.row(idx);
        
        marker.id   = idx;

        marker.pose.position.x    = (box(_BDY_x) + box(_BDY_X)) * 0.5;
        marker.pose.position.y    = (box(_BDY_y) + box(_BDY_Y)) * 0.5;
        marker.pose.position.z    = (box(_BDY_z) + box(_BDY_Z)) * 0.5;

        marker.scale.x  = box(_BDY_X) - box(_BDY_x);
        marker.scale.y  = box(_BDY_Y) - box(_BDY_y);
        marker.scale.z  = box(_BDY_Z) - box(_BDY_z);
        
        arrMarkers.markers.push_back(marker);
    }

    voxelPathPub.publish(arrMarkers);
}

void VisualizeCheckPoint()
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = odomTime;
    cloud.header.frame_id = string("/map");
    
    vector<double> pt = server->getCheckPoint();
    if (fout.is_open())
    {
        for (int i = 0; i < pt.size() / 3; i++)
            fout << pt[i * 3 + 0] << " " << pt[i * 3 + 1] << " " << pt[i * 3 +2] << endl;
        fout << "-" << endl;
    }

    cloud.height = 1;
    cloud.width  = pt.size() / 3;
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    
    cloud.point_step = _TOT_DIM * 4;
    cloud.row_step = cloud.point_step * cloud.width;

    sensor_msgs::PointField f;

    cloud.fields.resize(_TOT_DIM);
    string f_name[] = {string("x"), string("y"), string("z")};

    for (size_t idx = 0; idx < _TOT_DIM; ++idx)
    {
        f.name      = f_name[idx];
        f.offset    = idx << 2;
        f.datatype  = 7;
        f.count     = 1;
        cloud.fields[idx] = f;
    } 

    float * data = new float[pt.size()];

    for (size_t idx = 0; idx < pt.size(); ++idx)
        data[idx] = pt[idx];

    cloud.data.resize(cloud.row_step);
    uint8_t * dataRaw = (uint8_t *) data;

    for (size_t idx = 0; idx < cloud.row_step; ++idx)
        cloud.data[idx] = dataRaw[idx];

    checkPointPub.publish(cloud);

    delete [] data;
}

visualization_msgs::Marker line_strip;
void VisualizeTraj()
{
    if (!isTraj || !isVis) return ;

    line_strip.header.stamp     = ros::Time::now();
    line_strip.header.frame_id  = "/map";

    line_strip.action   = visualization_msgs::Marker::DELETE;
    lineStripPub.publish(line_strip);

    line_strip.ns = "line_strip";
    line_strip.id = 0;
    line_strip.type     = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.action   = visualization_msgs::Marker::ADD;
    line_strip.scale.x  = 0.1;
    line_strip.scale.y  = 0.1;
    line_strip.scale.z  = 0.1;

    line_strip.pose.orientation.x = 0.0;
    line_strip.pose.orientation.y = 0.0;
    line_strip.pose.orientation.z = 0.0;
    line_strip.pose.orientation.w = 1.0;

    line_strip.color.r  = 1.0;
    line_strip.color.g  = 0.0;
    line_strip.color.b  = 0.0;
    line_strip.color.a  = 1.0;



    double t_begin = server->getBeginTime();
    double t_final = server->getFinalTime();

    nav_msgs::Path path;
    path.header.stamp   = odomTime;
    path.header.frame_id    = string("/map");

    geometry_msgs::PoseStamped pose;
    pose.header = path.header;

    vector<double> state;
    
    line_strip.points.resize(0);
    line_strip.points.reserve(static_cast<int>((t_final - t_begin) * 100 + 0.5));
    geometry_msgs::Point pt;
    Eigen::Vector3d cur, pre; 

    double _tot_length = 0.0;

    int count = 0, iSafe = 0;
    for (double t = t_begin; t < t_final; t += 0.01, count += 1)
    {
        state = server->getDesiredState(t);
        cur(0) = pt.x = pose.pose.position.x    = state[0 * _TOT_DIM + _DIM_x];
        cur(1) = pt.y = pose.pose.position.y    = state[0 * _TOT_DIM + _DIM_y];
        cur(2) = pt.z = pose.pose.position.z    = state[0 * _TOT_DIM + _DIM_z];

        line_strip.points.push_back(pt);
        
        if (count)
            _tot_length += (pre - cur).norm();

        pre = cur;



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
    ROS_WARN("The length of the trajectory: %.9lf", _tot_length);

    lineStripPub.publish(line_strip);
    trajectoryPub.publish(path);

    VisualizePath();
    VisualizeCheckPoint();
    
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

    string dump_file;
    handle.param("/dump_file", dump_file, string(""));
    if (dump_file.length() > 0)
        fout.open(dump_file.c_str());

    handle.param("/init/is_vis", isVis, true);
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
        server->setMaxAcceleration(maxAcc);
        server->setMaxVelocity(maxVel);

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

    mapPointsPub    = handle.advertise
        <sensor_msgs::PointCloud2>("map_points", 2);

    trajectoryPub   = handle.advertise
        <nav_msgs::Path>("trajectory", 2);

    voxelPathPub    = handle.advertise
        <visualization_msgs::MarkerArray>("voxel_path", 2);

    lineStripPub    = handle.advertise
        <visualization_msgs::Marker>("line_strip", 2);

    checkPointPub   = handle.advertise
        <sensor_msgs::PointCloud2>("check_point", 2);

    ros::Publisher inflated_grid_pub = handle.advertise
        <visualization_msgs::Marker>("inflated_grid", 2);
    server->setGridPublisher(inflated_grid_pub);

    ros::spin();

    ROS_WARN("WHAT'S WRONG!?");
    return 0;
}

