
#define _TRAJECTORY_USE_VISUALIZATION_

//#define _FLAG_USE_NO_INFLATION_MAP_

#include "voxel_trajectory/voxelserver.h"
#include "voxel_trajectory/voxelmacro.h"
#include "voxel_trajectory/traj_utils.h"

// ros tools
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>

#include <tf/transform_datatypes.h>
#include "voxel_trajectory/CheckObstacleByPoints.h"

// standard messages
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// desired states messages
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
//#include "mavlink_message/PositionCommand.h"

#include <voxel_map/voxel_map.h>
#include <vector>
#include <set>
#include <deque>
#include <sstream>
#include <cmath>


using namespace std;
using namespace voxel_trajectory;

// to do: 
//  -1, set up waypoints assignment
// 
//  0. work states, including,
//      0) staying on ground,
//      1) taking off,
//      2) end-to-end,
//      3) landing,
//      4) hovering;
//
//  1. set up argments for specifying,
//      0) setting take_off point, velocity, acceleration
//      1) land_off point, velocity, acceleration
//      2) trajectory destination, preffered velocity and destination 
//
//  2. staying on ground stage:
//      0) wait for trigger, if so, go to => 3.
//
//  3. taking off stage:
//      0) fix yaw angle, and x, y coordinates, 
//      1) set up filter for filtering (x,y).
//      2) set destination and try to generate trajectory to take off
//          -> if false, go back to on ground state =>2.
//          -> if succeed, send the trajectory
//      3) check arrival by odometry and stamp
//          -> if yes, go forward to execute the end-to-end trajectory => 3.
//
//  4. end to end trajectory stage:
//      0) fixed yaw angle, and z coordinate,
//      1) try generate the trajectory to the destination(s ?):
//          -> if false, terminate the trajectory, go to hover => 6.
//          -> if succeed, send the trajectory;
//      2) check halfway after insertion of the laser scan,
//          -> if collide with newly detected obstacle, try regenrate => 4.1;
//          -> if not, pass
//      3) check arrival by odometry and stamp
//          -> if yes, go forward to landing stage.
//
//  5. landing stage:
//      0) fix yaw, and x, y coordiante, set up filter;
//      1) set destination and try to generate to land
//          -> if false, go to hovering stage => 6.
//          -> if suceed, send it
//      2) check arrival, 
//          -> if yes, go forward to execute the on ground stage.
//
//  6. hovering stage:
class TrajectoryGenerator
{
private:
    // _core server
    VoxelTrajectory::VoxelServer * _core = new VoxelTrajectory::VoxelServer();

    enum State {taking_off, landing, on_ground, hovering, doing_task};
    enum StateSignal {trigger, arrival, trajectory_failure};

    State _state = on_ground;
    // for changing the states of quadrotor
    inline void alterState(StateSignal signal_val);

    inline bool isArrived();

    // task action
    void configOnGround();
    void configTakingOff();
    void configAssignTask();
    void configLanding();
    void configHovering();

    // task param
    double _arrival_thld = 0.1;
    double _take_off_hgt = 1.3, _land_hgt = -0.1;
    double _take_off_vel = 1.0, _land_vel = 1.0;
    double _take_off_acc = 1.0, _land_acc = 1.0;
    double _task_vel = 1.0, _task_acc = 1.0;
    double _task_upper_hgt = 0.2, _task_lower_hgt = -0.3;
    vector<double> _task_waypoints {4.0, 0.0, 1.3};

    const double _EPS = 1e-9;
    const double _EPS_POS = 1e-1;
    const double _PI = acos(-1);

    // interface
    // service
    
    ros::ServiceServer _check_point_srv;

    // subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _obs_pt_sub, _obs_blk_sub, _obs_blk1_sub;
    ros::Subscriber _dest_pt_sub, _dest_pts_sub; 
    ros::Subscriber _laser_sub;
    ros::Subscriber _3d_laser_sub;
    ros::Subscriber _trigger_sub;

    // publishers
    ros::Publisher _desired_state_pub;
    ros::Publisher _traj_pub;

    ros::Publisher _map_vis_pub;
    ros::Publisher _map_no_inflation_vis_pub;
    ros::Publisher _traj_vis_pub;
    ros::Publisher _check_point_vis_pub;
    
    ros::Publisher _path_vis_pub;
    ros::Publisher _inflated_path_vis_pub;
    ros::Publisher _win_ctr_vis_pub;

    ros::Publisher _laser_vis_pub;

    ros::Publisher _debug_pub;

    // flags 
    bool _has_odom = false;
    bool _has_traj = false;
    bool _is_vis = true;

    double _extra_obstacle_height = 0.0;
    double _allowed_ground_height = 0.3;
    double _flight_height_limit = 100.0;

    // trajectory
    uint32_t _traj_id = 1;
    uint8_t _traj_status = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    vector<double> _waypoints;
    vector<double> _arr_time;

    const size_t _odom_queue_size = 500;
    deque<nav_msgs::Odometry> _odom_queue;

    // input messages
    nav_msgs::Odometry _odom;
    ros::Time _final_time = ros::TIME_MIN;

    // output messages
    quadrotor_msgs::PositionCommand _pos_cmd;
    quadrotor_msgs::PolynomialTrajectory _traj;

    //VoxelTrajectory::VoxelServer * _core_no_inflation = new VoxelTrajectory::VoxelServer();
    //VoxelTrajectory::VoxelServer * _core_empty = new VoxelTrajectory::VoxelServer();
    visualization_msgs::Marker _traj_vis;
    visualization_msgs::MarkerArray _path_vis, _inflated_path_vis;
    sensor_msgs::PointCloud2 _map_vis;
    sensor_msgs::PointCloud2 _check_point_vis;
    sensor_msgs::LaserScan _laser_scan;
    nav_msgs::Path _win_ctr_vis;
    
    // otheros
    ros::Timer _vis_map_timer;
    ros::Timer _vis_map_no_inflation_timer;

    ros::Time _last_scan_stamp;
    ros::Time _map_stamp = ros::TIME_MIN;
    ros::Duration _map_duration = ros::Duration(2.0);
    double _laser_scan_step = 0.2;
    double _laser_scan_resolution = 0.05;
    double _laser_radius = 1000.0;
    
    double _ratio_init_z_velocity = 0.5;

    double _vis_traj_width = 0.1;

public:
    TrajectoryGenerator(ros::NodeHandle & handle);

    bool checkObstacleByPoints(voxel_trajectory::CheckObstacleByPoints::Request & req, 
            voxel_trajectory::CheckObstacleByPoints::Response & res);

    // receive odometry
    void rcvOdometry(const nav_msgs::Odometry &);
    // receive obstacles
    void rcvGlobalPointsCloud2(const sensor_msgs::PointCloud2 &);
    void rcvGlobalBlocksCloud2(const sensor_msgs::PointCloud2 &);
    void rcvGlobalBlocksCloud1(const sensor_msgs::PointCloud &);
    void rcvLaserScan(const sensor_msgs::LaserScan &);
    void rcv3DLaserScan(const sensor_msgs::PointCloud2 &);
    void rcvTrigger(const geometry_msgs::PoseStamped &);

    // receive waypoints
    void rcvWaypoints(const nav_msgs::Path &);

    void initMap();
    void genTrajectory();

    // check halfway obstacle
    void checkHalfWay();

    // misc
    void setGains(const double[NUM_DIM] , const double[NUM_DIM]);
    void visMap(const ros::TimerEvent& evt);
    void visTrajectory();
    void visPath();
    void visInflatedPath();
    void visCheckpoint();
    void visWindowCenter();
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "trajectory_server_node");
    ros::NodeHandle handle("~");

    TrajectoryGenerator generator(handle);

    ros::spin();
    return 0;
}

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle & handle)
{
    //service
   
    _check_point_srv = 
        handle.advertiseService("CheckObstacleByPoints", &TrajectoryGenerator::checkObstacleByPoints, this);

    // subscribe odometry
    _odom_sub = 
        handle.subscribe("odometry", 50, &TrajectoryGenerator::rcvOdometry, this);

    // add obstacle
    _obs_pt_sub =
        handle.subscribe("obstacle_points", 2, &TrajectoryGenerator::rcvGlobalPointsCloud2, this);
    _obs_blk_sub = 
        handle.subscribe("obstacle_blocks", 2, &TrajectoryGenerator::rcvGlobalBlocksCloud2, this);
    _obs_blk1_sub = 
        handle.subscribe("obstacle_blocks1", 2, &TrajectoryGenerator::rcvGlobalBlocksCloud1, this);
    _laser_sub = 
        handle.subscribe("laser_scan", 5, &TrajectoryGenerator::rcvLaserScan, this);
    _3d_laser_sub = 
        handle.subscribe("laser_scan_3d", 1, &TrajectoryGenerator::rcv3DLaserScan, this);

    // set destination
    _dest_pts_sub = 
        handle.subscribe("waypoints", 2, &TrajectoryGenerator::rcvWaypoints, this);

    // trigger for executing the mission
    _trigger_sub = 
        handle.subscribe("trigger", 2, &TrajectoryGenerator::rcvTrigger, this);

    // publish desired state
    _traj_pub = 
        handle.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);

    // pulish visualizatioin
    _map_vis_pub =
        handle.advertise<sensor_msgs::PointCloud2>("map_vis", 2);
    _map_no_inflation_vis_pub = 
        handle.advertise<sensor_msgs::PointCloud2>("map_no_inflation_vis", 2);
    _path_vis_pub =
        handle.advertise<visualization_msgs::MarkerArray>("path_vis", 2);
    _inflated_path_vis_pub = 
        handle.advertise<visualization_msgs::MarkerArray>("inflated_path_vis", 2);
    _traj_vis_pub =
        handle.advertise<visualization_msgs::Marker>("trajectory_vis", 2);
    _check_point_vis_pub =
        handle.advertise<sensor_msgs::PointCloud2>("checkpoints_vis", 2);
    _laser_vis_pub = 
        handle.advertise<visualization_msgs::Marker>("laser_points_vis", 2);
    _win_ctr_vis_pub = 
        handle.advertise<nav_msgs::Path>("window_center_points_vis", 2);
    _debug_pub = 
        handle.advertise<std_msgs::String>("debug_info", 50);

    // others
    double _vis_drt = 1.0;
    handle.param("map/visualization_duration", _vis_drt, _vis_drt);
    _vis_map_timer = 
        handle.createTimer(ros::Duration(_vis_drt), &TrajectoryGenerator::visMap, this);

    handle.param("flag/visualization", _is_vis, true);
    { // parameter for setting log odds
        double log_odd[5];
        handle.param("map/log_odd/min", log_odd[0], -2.0);
        handle.param("map/log_odd/max", log_odd[1], 4.5);
        handle.param("map/log_odd/occupy", log_odd[2], 0.0);
        handle.param("map/log_odd/hit", log_odd[3], 0.85);
        handle.param("map/log_odd/miss", log_odd[4], -0.4);
        _core->setLogOdd(log_odd);
        double ob_radius;
        handle.param("map/observation_radius", ob_radius, 10.0);
        _laser_radius = ob_radius;
        _core->setObservationRadius(ob_radius);
    }
    {
        double tmp;
        handle.param("map/dislike_margin",  tmp, 0.0);
        _core->setDislikeMargin(tmp);
        handle.param("map/path_resolution", tmp, 0.2);
        _core->setPathResolution(tmp);
    }
    {
        double bdy[NUM_BDY];
        handle.param("map/boundary/lower_x", bdy[LX], -100.0);
        handle.param("map/boundary/upper_x", bdy[RX], 100.0);
        handle.param("map/boundary/lower_y", bdy[LY], -100.0);
        handle.param("map/boundary/upper_y", bdy[RY], 100.0);
        handle.param("map/boundary/lower_z", bdy[LZ], -100.0);
        handle.param("map/boundary/upper_z", bdy[RZ], 200.0);
        _core->setMapBoundary(bdy);
    }
    {
        double resolution, margin, max_vel, max_acc, f_vel, f_acc;
        handle.param("map/resolution", resolution, 0.4);
        handle.param("map/safe_margin", margin, 0.3);
        handle.param("max_velocity", max_vel, 1.0);
        handle.param("max_acceleration", max_acc, 1.0);
        handle.param("flight_velocity", f_vel, max_vel);
        handle.param("flight_acceleration", f_acc, max_acc);
        _core->setResolution(resolution);
        _core->setMargin(margin);
        _core->setMaxVelocity(max_vel);
        _core->setMaxAcceleration(max_vel);
        _core->setFlightVelocity(f_vel);
        _core->setFlightAcceleration(f_acc);
        _core->initMap();    
        double pos_gain[NUM_DIM] = {3.7, 3.7, 5.2};
        double vel_gain[NUM_DIM] = {2.4, 2.4, 3.0};
        setGains(pos_gain, vel_gain);
    }
    {
        handle.param("setting/flight_height_limit", _flight_height_limit, 1e7);
        handle.param("setting/extra_obstacle_height", _extra_obstacle_height, 0.0);
        handle.param("setting/allowed_ground_height", _allowed_ground_height, 1e7);
        handle.param("setting/laser_scan_step", _laser_scan_step, 0.2);
        handle.param("setting/laser_scan_resolution", _laser_scan_resolution, 0.05);
        handle.param("setting/ratio_z_init_velocity", _ratio_init_z_velocity, 0.5);
    }
    { // for task setting
        handle.param("setting/take_off/height", _take_off_hgt, _take_off_hgt);
        handle.param("setting/take_off/velocity", _take_off_vel, _take_off_vel);
        handle.param("setting/take_off/acceleration", _take_off_acc, _take_off_acc);

        handle.param("setting/land/height", _land_hgt, _land_hgt);
        handle.param("setting/land/velocity", _land_vel, _land_vel);
        handle.param("setting/land/acceleration", _land_acc, _land_acc);

        handle.param("setting/task/lower_height", _task_lower_hgt, _task_lower_hgt);
        handle.param("setting/task/upper_height", _task_upper_hgt, _task_upper_hgt);
        handle.param("setting/task/velocity", _task_vel, _task_vel);
        handle.param("setting/task/acceleration", _task_acc, _task_acc);

        handle.param("setting/task/arrive_threshold", _arrival_thld, _arrival_thld);
        int num_wps = 0;
        handle.param("setting/task/waypoints/num", num_wps, num_wps);
        _task_waypoints.clear();
        for (int idx = 0; idx < num_wps; ++idx)
        {
            vector<double> wp;
            handle.getParam("setting/task/waypoints/point_" + to_string(idx), wp);
            assert(wp.size() == NUM_DIM);
            _task_waypoints.insert(_task_waypoints.end(), wp.begin(), wp.end());
        }
    } 

    double map_duration;
    handle.param("map/map_duration", map_duration, 2.0);
    this->_map_duration = ros::Duration(map_duration);
    this->_map_stamp = ros::Time::now();

    _last_scan_stamp = ros::TIME_MIN;
    handle.param("vis/trajectory_with", _vis_traj_width, 0.1);
}

void TrajectoryGenerator::setGains(const double pos_gain[NUM_DIM], const double vel_gain[NUM_DIM])
{
    _pos_cmd.kx[X] = pos_gain[X];
    _pos_cmd.kx[Y] = pos_gain[Y];
    _pos_cmd.kx[Z] = pos_gain[Z];

    _pos_cmd.kv[X] = vel_gain[X];
    _pos_cmd.kv[Y] = vel_gain[Y];
    _pos_cmd.kv[Z] = vel_gain[Z];
}

void TrajectoryGenerator::rcvOdometry(const nav_msgs::Odometry & odom)
{
    // invalid odomtry
    if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return ;

    _odom = odom;

    _has_odom = true;
    _odom_queue.push_back(odom);
    while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();

    if (isArrived()) alterState(arrival);
}

void TrajectoryGenerator::rcvTrigger(const geometry_msgs::PoseStamped &)
{
    alterState(trigger);
}

void TrajectoryGenerator::checkHalfWay()
{
    if (_has_traj && _odom.header.stamp < _final_time 
            && _core->checkHalfWayObstacle_BrutalForce(_odom.header.stamp.toSec()))
    {
        _final_time = ros::TIME_MIN;
        
        nav_msgs::Path path;
        { // set up waypoints
            size_t valid_id = 0;
            for (valid_id = 0; valid_id < _arr_time.size(); ++valid_id)
            {
                if (_odom.header.stamp.toSec() < _arr_time[valid_id]) break;
            }
#if 0
            ROS_WARN_STREAM("current segment id = " << valid_id);
            ROS_WARN_STREAM("current time = " << _odom.header.stamp.toSec());
            for (size_t ind = 0; ind < _arr_time.size(); ++ind)
                ROS_WARN_STREAM(" id = " << ind << ", stamp = "
                        << _arr_time[ind] - _odom.header.stamp.toSec());
#endif

            path.header.stamp = _odom.header.stamp;
            path.header.frame_id = "/map";
            path.poses.resize(_arr_time.size() - valid_id);

            for (size_t idx = valid_id; idx * NUM_DIM < _waypoints.size(); ++idx)
            {
                path.poses[idx - valid_id].pose.position.x = _waypoints[idx * NUM_DIM + X];
                path.poses[idx - valid_id].pose.position.y = _waypoints[idx * NUM_DIM + Y];
                path.poses[idx - valid_id].pose.position.z = _waypoints[idx * NUM_DIM + Z];
            }
        }
        { // set up the orientation (yaw)
            auto quad = tf::createQuaternionFromYaw(_traj.final_yaw);
            path.poses.back().pose.orientation.w = quad.w();
            path.poses.back().pose.orientation.x = quad.x();
            path.poses.back().pose.orientation.y = quad.y();
            path.poses.back().pose.orientation.z = quad.z();
        }
        rcvWaypoints(path);
    }
    if (_traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD 
           &&  _odom.header.stamp >= _final_time)
    {
        _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
        _traj_pub.publish(_traj);
    }
    // ROS_INFO("[GENERATOR] HALF_WAY_CHECK DONE!!");
}

bool TrajectoryGenerator::checkObstacleByPoints(
        voxel_trajectory::CheckObstacleByPoints::Request & req,
        voxel_trajectory::CheckObstacleByPoints::Response & res)
{
    // ROS_WARN("[GENERATOR] POINTS_CHECK DONE!!");
    res.is_occupied.resize(req.size);
    for (size_t idx = 0; idx < req.size; ++idx)
    {
        res.is_occupied[idx] = _core->isValid3DPoint(
                req.x[idx], req.y[idx], req.z[idx]);
    }
    return true;
}

void TrajectoryGenerator::rcvGlobalPointsCloud2(const sensor_msgs::PointCloud2 & cloud)
{
    vector<voxel_map::Point> pts;
    if (!retPointsFromCloud2Msg(cloud, pts)) return;

    _core->addPoints(pts);
    checkHalfWay();
}

void TrajectoryGenerator::rcvGlobalBlocksCloud2(const sensor_msgs::PointCloud2 & cloud)
{
    vector<voxel_map::Box> boxes;
    if (!retBoxesFromCloud2Msg(cloud, boxes)) return;
    
    _core->addBoxes(boxes);
    checkHalfWay();
}

void TrajectoryGenerator::rcvGlobalBlocksCloud1(const sensor_msgs::PointCloud & cloud)
{
    vector<voxel_map::Box> boxes;
    if (!retBoxesFromCloud1Msg(cloud, boxes)) return;
    
    _core->addBoxes(boxes);
    checkHalfWay();
}
    
void TrajectoryGenerator::initMap()
{
    if (ros::Time::now() - _map_stamp > _map_duration)
    {
        _map_stamp = ros::Time::now();

        _core->initMap();
        ROS_WARN("[GENERATOR] map initalized.");
    }
}

void TrajectoryGenerator::rcv3DLaserScan(const sensor_msgs::PointCloud2 & cloud)
{
    if (_odom_queue.empty()) return ;
    if (cloud.header.stamp - _last_scan_stamp < ros::Duration(_laser_scan_step)) return ;
    _last_scan_stamp = cloud.header.stamp;

    auto laser_odom = _odom_queue.back();

    for (auto & odom: _odom_queue)
    {
        if (odom.header.stamp > cloud.header.stamp)
        {
            laser_odom = odom;
            break;
        }
    }

    vector<voxel_map::Ray> rays;
    if (!retRaysFromCloud2Odom(cloud, laser_odom, rays, 
                _allowed_ground_height,
                _laser_scan_resolution, 1)) return;
    _core->addRays(rays);

    vector<voxel_map::Box> boxes;
    for (auto & ray: rays)
    {
        if (ray.target()(2) <= _allowed_ground_height) continue;
        if (ray.length() > _laser_radius) continue;
        voxel_map::Box box(ray.target(), ray.target());
        box.lower(2) -= _extra_obstacle_height;
        box.upper(2) += _extra_obstacle_height;
        boxes.push_back(box);
    }
    _core->addBoxes(boxes);
    checkHalfWay();
}

void TrajectoryGenerator::rcvLaserScan(const sensor_msgs::LaserScan & scan)
{
    if (_odom_queue.empty()) return;
    if (scan.header.stamp - _last_scan_stamp < ros::Duration(_laser_scan_step)) return ;

    _last_scan_stamp = scan.header.stamp;

    auto laser_odom = _odom_queue.back();
    for (auto & odom: _odom_queue)
    {
        if (odom.header.stamp > scan.header.stamp) 
        {
            laser_odom = odom;
            break;
        }
    }
    vector<voxel_map::Ray> rays;
    if (!retRaysFromScanOdom(scan, laser_odom, rays, 
                _allowed_ground_height,
                _laser_scan_resolution, 1)) return ;
    _core->addRays(rays);

    vector<voxel_map::Box> boxes;
    for (auto & ray: rays)
    {
        if (ray.target()(2) <= _allowed_ground_height) continue;
        if (ray.length() > _laser_radius) continue;
        voxel_map::Box box(ray.target(), ray.target());
        box.lower(2) -= _extra_obstacle_height;
        box.upper(2) += _extra_obstacle_height;
        boxes.push_back(box);
    }
    _core->addBoxes(boxes);
    checkHalfWay();
}

void TrajectoryGenerator::rcvWaypoints(const nav_msgs::Path & wp)
{
    if (_state != doing_task) return ;

    _waypoints.clear();
    _waypoints.reserve(wp.poses.size() * NUM_DIM);

    for (auto & pose: wp.poses) 
    {
        _waypoints.push_back(pose.pose.position.x);
        _waypoints.push_back(pose.pose.position.y);
        _waypoints.push_back(pose.pose.position.z);
    }
    genTrajectory();
}

void TrajectoryGenerator::genTrajectory()
{
    // to do
    ROS_WARN("rcv waypoints");
    if (!_has_odom || _waypoints.empty()) return ;
    
    // choose a kernel, 
    VoxelTrajectory::VoxelServer * p_core = _core;


    vector<double> state = 
    {
        _odom.pose.pose.position.x + _EPS,
        _odom.pose.pose.position.y + _EPS,
        _odom.pose.pose.position.z + _EPS,
        _odom.twist.twist.linear.x,
        _odom.twist.twist.linear.y,
        _odom.twist.twist.linear.z * _ratio_init_z_velocity,
        0.0, 0.0, 0.0
    };

    ROS_INFO("[GENERATOR] Generating the trajectory.");

    vector<double> cost_time;
    if (p_core->setWayPointsRec(state, _waypoints, _odom.header.stamp.toSec(), 
                _arr_time, cost_time) != 2)
    { // fail to generate trajectory
        _arr_time.clear();
        // illegal _waypoints
        _waypoints.clear();
        ROS_INFO("[GENERATOR] Generating the trajectory failed!");
        _final_time = ros::TIME_MIN;

        _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(_traj);
        _has_traj = false;

        alterState(trajectory_failure);
    }
    else
    { // trajectory generation succeed.
        ROS_INFO("[GENERATOR] Generating the trajectory succeed!");

        _traj_id += 1;
        _final_time = ros::Time(p_core->getFinalTime());

        _traj = p_core->getTraj();
#if 1
        { // fix yaw
            _traj.start_yaw = tf::getYaw(_odom.pose.pose.orientation);
            _traj.final_yaw = tf::getYaw(_odom.pose.pose.orientation);

            { // in case of assigning yaw, make absolute of yaw difference less than Pi
                double d_yaw = fmod(_traj.final_yaw - _traj.start_yaw, 2 * _PI);
                if (d_yaw < -_PI) d_yaw += 2 * _PI;
                if (d_yaw > +_PI) d_yaw -= 2 * _PI;
            }
        }
#endif

        _traj.header.frame_id = "/map";
        _traj.trajectory_id = _traj_id;
        _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
        _traj_pub.publish(_traj);
        ROS_INFO("[GENERATOR] Published the trajectory.");

        {
            std_msgs::String debug_info;
            stringstream sin;
            sin << "[NEW_TRAJ] The_Trajectory_Generation_Duration: " 
                << cost_time[X] << ", " 
                << cost_time[Y] << ", " 
                << cost_time[Z] << ".\n"
                << "[NEW_TRAJ] The new trajectory cost: " 
                << p_core->qp_cost[X] << ", "
                << p_core->qp_cost[Y] << ", "
                << p_core->qp_cost[Z] << ".";
            debug_info.data = sin.str();
            _debug_pub.publish(debug_info);
        }
        _has_traj = true;
    }

    ROS_INFO("[GENERATOR] Starting visualization.");
    visTrajectory();
    ROS_INFO("[GENERATOR] Trajectory visualzation finished.");
    visPath();
    ROS_INFO("[GENERATOR] Path visualzation finished.");
    visInflatedPath();
    ROS_INFO("[GENERATOR] Inflated path visualzation finished.");
    visCheckpoint();
    ROS_INFO("[GENERATOR] Checkpoints visualzation finished.");
    //visWindowCenter();
    //ROS_INFO("[GENERATOR] Windows' center points published.");
}

void TrajectoryGenerator::visMap(const ros::TimerEvent& evt)
{
    //ROS_INFO("[GENERATOR] Map visualization start... flag : is_vis = %d, has_map = %d",
           // _is_vis, _has_map);

    if (!_is_vis) return ;
    vector<double> pt = _core->getPointCloud();
    //ROS_INFO("[GENERATOR] Here are %lu occupied grid(s) in the octormap.", pt.size() / 3);

    vector<float> pt32;
    pt32.resize(pt.size());
    for (size_t idx = 0; idx < pt.size(); ++idx) pt32[idx] = static_cast<float>(pt[idx]);
    //ROS_INFO("[GENERATOR] float points alredy.");

    const double * bdy = _core->getBdy();
    //ROS_INFO("[GENERATOR] The boundary of the map: [%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf].",
    //        bdy[LX], bdy[LY], bdy[LZ], bdy[RX], bdy[RY], bdy[RZ]);

    _map_vis.header.frame_id = "/map";
    _map_vis.header.stamp = _odom.header.stamp;

    _map_vis.height = 1;
    _map_vis.width = pt.size() / NUM_DIM;
    _map_vis.is_bigendian = false;
    _map_vis.is_dense = true;

    _map_vis.point_step = 4 * NUM_DIM;
    _map_vis.row_step = _map_vis.point_step * _map_vis.width;

    sensor_msgs::PointField field;
    _map_vis.fields.resize(NUM_DIM);
    string f_name[NUM_DIM] = {"x", "y", "z"};
    for (size_t idx = 0; idx < NUM_DIM; ++idx)
    {
        field.name = f_name[idx];
        field.offset = idx * 4;
        field.datatype = sensor_msgs::PointField::FLOAT32;
        field.count = 1;
        _map_vis.fields[idx] = field;
    }

    _map_vis.data.clear();
    _map_vis.data.reserve(_map_vis.row_step);
    uint8_t * pt_int = reinterpret_cast<uint8_t *>(pt32.data());
    for (size_t idx = 0; idx < _map_vis.row_step; ++idx)
    {
        _map_vis.data.push_back(pt_int[idx]);
    }

    _map_vis_pub.publish(_map_vis);
    //ROS_INFO("[GENERATOR] Map visualization finished. Total number of points : %d.", 
    //        _map_vis.point_step);

}

void TrajectoryGenerator::visTrajectory()
{
    if (!_is_vis || !_has_traj) return ;
    if (_odom.header.stamp > _final_time) return;
    _traj_vis.header.stamp       = _odom.header.stamp;
    _traj_vis.header.frame_id    = "/map";

    _traj_vis.ns = "trajectory/trajectory";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;
    _traj_vis.color.a = 0.7;

    double traj_len = 0.0;
    int count = 0;
    Eigen::Vector3d cur, pre;

    double t_begin = _core->getBeginTime(), t_final = _core->getFinalTime();
    _traj_vis.points.clear();
    // ROS_INFO("[GENERATOR] Trajectory time : %.3lf %.3lf", t_begin, t_final);
    _traj_vis.points.reserve(static_cast<int>((t_final - t_begin) * 100 + 0.5));
    vector<double> state;
    geometry_msgs::Point pt;

    // ROS_INFO("[GENERATOR] Trajectory visualization prepared.");

    for (double t = t_begin; t < t_final; t += 0.02, count += 1)
    {
        state = _core->getDesiredState(t);
        cur(X) = pt.x = state[X];
        cur(Y) = pt.y = state[Y];
        cur(Z) = pt.z = state[Z];
        _traj_vis.points.push_back(pt);

        if (count) traj_len += (pre - cur).norm();
        pre = cur;
    }

    // ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
    _traj_vis_pub.publish(_traj_vis);
}

void TrajectoryGenerator::visPath()
{
    if (!_is_vis) return ;

    for (auto & mk: _path_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;
    _path_vis_pub.publish(_path_vis);

    const Eigen::MatrixXd & path = _core->getPathConstRef();
    size_t M = path.rows() >> 1;

    _path_vis.markers.clear();
    _path_vis.markers.reserve(M);

    visualization_msgs::Marker mk;
    mk.header.frame_id = "/map";
    mk.header.stamp = _odom.header.stamp;
    mk.ns = "trajectory/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 0.2;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;

    for (size_t idx = 1; idx <= M; ++idx)
    {
        mk.id = idx;
        mk.pose.position.x = (path(idx, LX) + path(idx, RX)) * 0.5;
        mk.pose.position.y = (path(idx, LY) + path(idx, RY)) * 0.5;
        mk.pose.position.z = (path(idx, LZ) + path(idx, RZ)) * 0.5;
        mk.scale.x = path(idx, RX) - path(idx, LX);
        mk.scale.y = path(idx, RY) - path(idx, LY);
        mk.scale.z = path(idx, RZ) - path(idx, LZ);

        _path_vis.markers.push_back(mk);
    }
    _path_vis_pub.publish(_path_vis);
}

void TrajectoryGenerator::visInflatedPath()
{
    if (!_is_vis || !_has_traj) return ;

    for (auto & mk: _inflated_path_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;
    _inflated_path_vis_pub.publish(_inflated_path_vis);

    const Eigen::MatrixXd & path = _core->getInflatedPathConstRef();
    size_t M = path.rows();

    _inflated_path_vis.markers.clear();
    _inflated_path_vis.markers.reserve(M);

    visualization_msgs::Marker mk;
    mk.header.frame_id = "/map";
    mk.header.stamp = _odom.header.stamp;
    mk.ns = "trajectory/inflated_grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 0.2;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;

    for (size_t idx = 0; idx < M; ++idx)
    {
        mk.id = idx;
        mk.pose.position.x = (path(idx, LX) + path(idx, RX)) * 0.5;
        mk.pose.position.y = (path(idx, LY) + path(idx, RY)) * 0.5;
        mk.pose.position.z = (path(idx, LZ) + path(idx, RZ)) * 0.5;
        mk.scale.x = path(idx, RX) - path(idx, LX);
        mk.scale.y = path(idx, RY) - path(idx, LY);
        mk.scale.z = path(idx, RZ) - path(idx, LZ);

        _inflated_path_vis.markers.push_back(mk);
    }
    _inflated_path_vis_pub.publish(_inflated_path_vis);
}

void TrajectoryGenerator::visCheckpoint()
{
    if (!_is_vis || !_has_traj) return ;
    _check_point_vis.header.stamp = _odom.header.stamp;
    _check_point_vis.header.frame_id = string("/map");

    vector<double> pt = _core->getCheckPoint();

    _check_point_vis.height = 1;
    _check_point_vis.width = pt.size() / 3;
    _check_point_vis.is_bigendian = false;
    _check_point_vis.is_dense = true;

    _check_point_vis.point_step = NUM_DIM * 4;
    _check_point_vis.row_step = _check_point_vis.point_step * _check_point_vis.width;

    sensor_msgs::PointField field;
    _check_point_vis.fields.resize(NUM_DIM);
    string f_name [] = {"x", "y", "z"};
    for (size_t idx = 0; idx < NUM_DIM; ++idx)
    {
        field.name = f_name[idx];
        field.offset = idx * 4;
        field.datatype = 7;;
        field.count = 1;
        _check_point_vis.fields[idx] =field;
    }

    float pt32[pt.size()];
    for (size_t idx = 0; idx < pt.size(); ++idx) pt32[idx] = pt[idx];

    _check_point_vis.data.resize(_check_point_vis.row_step);
    uint8_t * pt_int = reinterpret_cast<uint8_t *>(pt32);

    for (size_t idx = 0; idx < _check_point_vis.row_step; ++idx)
        _check_point_vis.data[idx] = pt_int[idx];

    _check_point_vis_pub.publish(_check_point_vis);
}

void TrajectoryGenerator::visWindowCenter()
{
    const Eigen::MatrixXd & m_path = _core->getPathConstRef();
    int m = m_path.rows() >> 1;
    nav_msgs::Path & win_ctr = _win_ctr_vis;
    geometry_msgs::PoseStamped pose;

    win_ctr.header.frame_id = "/map";
    win_ctr.header.stamp = _odom.header.stamp;
    pose.header = win_ctr.header;
    win_ctr.poses.clear();
    win_ctr.poses.reserve(m);;
    
    for (int idx = m + 1; idx < (m + m); ++ idx)
    {
        pose.pose.position.x = 0.5 * (m_path(idx, LX) + m_path(idx, RX));
        pose.pose.position.y = 0.5 * (m_path(idx, LY) + m_path(idx, RY));
        pose.pose.position.z = 0.5 * (m_path(idx, LZ) + m_path(idx, RZ));
        win_ctr.poses.push_back(pose);
    }
    pose.pose.position.x = m_path(0, 3);
    pose.pose.position.y = m_path(0, 4);
    pose.pose.position.z = m_path(0, 5);

    win_ctr.poses.push_back(pose);

    _win_ctr_vis_pub.publish(win_ctr);
}

inline bool TrajectoryGenerator::isArrived()
{
    if (!_arr_time.empty() && _odom.header.stamp > _final_time)
    {
        Eigen::Vector3d pos, dest;
        pos << 
            _odom.pose.pose.position.x,
            _odom.pose.pose.position.y,
            _odom.pose.pose.position.z;
        dest << 
            _waypoints[_waypoints.size() - NUM_DIM + X], 
            _waypoints[_waypoints.size() - NUM_DIM + Y], 
            _waypoints[_waypoints.size() - NUM_DIM + Z]; 

        bool ret = (pos-dest).norm() < _arrival_thld;

        if (ret)
        {
            _waypoints.clear();
            _arr_time.clear();
            _final_time = ros::TIME_MIN;
            ROS_WARN("[GENERATOR] arrive the goal position");
        }
        
        return ret;
    }
    return false;
}

inline void TrajectoryGenerator::alterState(StateSignal signal_val)
{
    if (signal_val == trigger && _state == on_ground)
    {
        _state = taking_off;
        // take_off
        configTakingOff();
    }
    else if (signal_val == arrival && _state == taking_off)
    {
        _state = doing_task;
        // assign task
        configAssignTask();
    }
    else if (signal_val == arrival && _state == doing_task)
    {
        _state = landing;
        // land
        configLanding();
    }
    else if (signal_val == arrival && _state == landing)
    {
        _state = on_ground;
        // finished
        configOnGround();
    }
    else if (_state != hovering && signal_val == trajectory_failure)
    {
        _state = hovering;
        ROS_WARN_STREAM("[GENERATOR] Task failed!");
        // abort trajectory!
        configHovering();
    }
    else if (_state != hovering)
    {
        ROS_WARN_STREAM("[GENERATOR] Unexpected Situation, state = " 
                << _state << ", signal = " << signal_val);
        _state = hovering;
        // abort trajectory and hovering !
        configHovering();
    }   
}

void TrajectoryGenerator::configOnGround()
{
    _traj.header.stamp = _odom.header.stamp;
    _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
    _traj_pub.publish(_traj);
}

void TrajectoryGenerator::configTakingOff()
{
    ROS_WARN("[GENERATOR] configure taking off...");
    _waypoints = vector<double> 
    {
        _odom.pose.pose.position.x, 
        _odom.pose.pose.position.y,
        _take_off_hgt
    };

    _core->setFlightVelocity(_take_off_vel);
    _core->setFlightAcceleration(_take_off_acc);
    {
        // setting up the filter for search flight corridor
        //  fix (x, y)
        _core->setPathVoxelFilter(
                [&](const voxel_map::Box & box) -> bool
                {
                    return
                        box.contain(voxel_map::Point(
                            _odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            box.lower(Z)
                        ));
                }
        );
    }
    genTrajectory();
}

void TrajectoryGenerator::configAssignTask()
{
    ROS_WARN("[GENERATOR] configure task...");
    _waypoints = _task_waypoints;
    for (size_t idx = 0; idx < _waypoints.size(); idx += NUM_DIM)
    {
       _waypoints[idx + Z] = _odom.pose.pose.position.z; 
    }

    _core->setFlightVelocity(_task_vel);
    _core->setFlightAcceleration(_task_acc);
    {
        // setting up the filter for search flight corridor
        //  fix height (z)
        //  no obstale within a specifed height window (to do)
        _core->setPathVoxelFilter(
                [&](const voxel_map::Box & box) -> bool
                {
                    auto test_box = box;
                    test_box.lower(Z) += _task_lower_hgt;
                    test_box.upper(Z) += _task_upper_hgt;

                    return
                        box.contain(voxel_map::Point(
                            box.lower(X),
                            box.lower(Y),
                            _odom.pose.pose.position.z
                        )) 
                        && 
                        _core->isValidBox(test_box);
                }
        );
    }
    genTrajectory();
}

void TrajectoryGenerator::configLanding()
{
    ROS_WARN("[GENERATOR] configure landing...");
    _waypoints = vector<double>
    {
        _odom.pose.pose.position.x,
        _odom.pose.pose.position.y,
        _land_hgt
    };
    _core->setFlightVelocity(_land_vel);
    _core->setFlightAcceleration(_land_acc);
    {
        // setting up the filter for search flight corridor
        //  fix (x, y)
        _core->setPathVoxelFilter(
                [&](const voxel_map::Box & box) -> bool
                {
                    return
                        box.contain(voxel_map::Point(
                            _odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            box.lower(Z)
                        ));
                }
        );
    }
    genTrajectory();
}

void TrajectoryGenerator::configHovering()
{
    ROS_WARN("[GENERATOR] configure hovering...");
    _waypoints = vector<double>
    {
        _odom.pose.pose.position.x,
        _odom.pose.pose.position.y,
        _odom.pose.pose.position.z
    };
    { // setting up the filter, fix x, y, z 
        _core->setPathVoxelFilter(
                [&](const voxel_map::Box & box) -> bool
                {
                    return
                        box.contain(voxel_map::Point(
                            _odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z
                        ));
                }
        );
    }
    genTrajectory();
}
