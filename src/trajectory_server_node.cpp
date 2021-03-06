
//#define _TRAJECTORY_TRAJECTROY_USE_MAVLINK_MSG_
#define _TRAJECTORY_USE_VISUALIZATION_
#define _TRAJECTORY_GENERATOR_FLAG_NO_INFLATION_

#include "voxel_trajectory/voxelserver.h"
#include "voxel_trajectory/voxelmacro.h"
#include "voxel_trajectory//rrt_planner.h"

// ros tools
#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"

#include "tf/transform_datatypes.h"
//#include "pcl/io/pcd_io.h"
//#include "pcl/point_types.h"
// standard messages
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
// desired states messages
#include "quadrotor_msgs/PositionCommand.h"
//#include "mavlink_message/PositionCommand.h"
#include "laser_geometry/laser_geometry.h"

#include <vector>
#include <set>
#include <deque>
#include <sstream>



using namespace std;

// server state
class NodeServer
{
private:
    // _core server
    VoxelTrajectory::VoxelServer * _core = new VoxelTrajectory::VoxelServer();
    const double _EPS = 1e-9;
    const double _EPS_POS = 1e-1;

    // interface
    // subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _obs_pt_sub, _obs_blk_sub;
    ros::Subscriber _dest_pt_sub, _dest_pts_sub; 
    ros::Subscriber _laser_sub;

    // publishers
    ros::Publisher _desired_state_pub;

    ros::Publisher _map_vis_pub;
    ros::Publisher _traj_vis_pub;
    ros::Publisher _check_point_vis_pub;
    ros::Publisher _path_vis_pub;
    ros::Publisher _inflated_path_vis_pub;
    ros::Publisher _debug_pub;

    ros::Publisher _laser_vis_pub;
    ros::Publisher _rrt_path_pub;

    // flags 
    bool _has_odom = false;
    bool _has_traj = false;
    bool _has_map = false;
    bool _is_vis = true;

    // configuration
    double _resolution = 0.4;        // unit - m^3
    double _safe_margin = 0.1;       // unit - m
    double _max_acc = 1.0;           // unit - m/s
    double _max_vel = 1.0;           // unit - m/(s^2)

    double _extra_obstacle_height = 0.0;
    double _allowed_ground_height = 0.3;
    double _flight_height_limit = 100.0;

    // trajectory
    uint32_t _traj_id = 1;
    uint8_t _traj_status = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    vector<double> _waypoints;
    vector<double> _arr_time;

    const size_t _odom_queue_size = 200;
    deque<nav_msgs::Odometry> _odom_queue;

    // input messages
    nav_msgs::Odometry _odom;
    nav_msgs::Odometry _last_dest;
    ros::Time _final_time = ros::TIME_MIN;

    // output messages
    quadrotor_msgs::PositionCommand _pos_cmd;

    visualization_msgs::Marker _traj_vis;
    visualization_msgs::MarkerArray _path_vis, _inflated_path_vis;
    sensor_msgs::PointCloud2 _map_vis;
    sensor_msgs::PointCloud2 _check_point_vis;
    sensor_msgs::LaserScan _laser_scan;
    
    // otheros
    ros::Timer _vis_map_timer;

    VoxelTrajectory::RRTPlannerManager * _planner;

    ros::Time _last_scan_stamp;
    double _laser_scan_step = 0.2;
    double _laser_scan_resolution = 0.05;

    double _vis_traj_width = 0.10;

public:

    NodeServer(ros::NodeHandle & handle)
    {
        // subscribe odometry
        _odom_sub = 
            handle.subscribe("odometry", 50, &NodeServer::rcvOdometryCallbck, this);

        // add obstacle
        _obs_pt_sub =
            handle.subscribe("obstacle_points", 2, &NodeServer::rcvGlobalPointCloudCallback, this);
        _obs_blk_sub = 
            handle.subscribe("obstacle_blocks", 2, &NodeServer::rcvGlobalBlocksCloudCallback, this);
        _laser_sub = 
            handle.subscribe("laser_scan", 5, &NodeServer::rcvLaserScanCallback, this);

        // set destination
        _dest_pt_sub = 
            handle.subscribe("goal_point", 2, &NodeServer::rcvDestinationCallback, this);
        _dest_pts_sub = 
            handle.subscribe("waypoints", 2, &NodeServer::rcvWaypointsCallback, this);

        // publish desired state
        _desired_state_pub = 
            handle.advertise<quadrotor_msgs::PositionCommand>("desired_state", 10);


        // pulish visualizatioin
        _map_vis_pub =
            handle.advertise<sensor_msgs::PointCloud2>("map_vis", 2);
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
        _rrt_path_pub = 
            handle.advertise<nav_msgs::Path>("rrt_path", 2);
        _debug_pub = 
            handle.advertise<std_msgs::String>("debug_info", 2);

        // others
        _vis_map_timer = 
            handle.createTimer(ros::Duration(1.0), &NodeServer::visMap, this);

        handle.param("vis/trajectory_width", _vis_traj_width, 0.10);

        _last_scan_stamp = ros::TIME_MIN;

        ROS_WARN("[NO_INFLATION] prepraring rrt planner ..");
        auto validator = [&](double x, double y, double z)->bool
        {
            return _core->isVaild3DPoint(x, y, z);
        };
        ROS_WARN("[NO_INFLATION] setting up rrt planner ..");

        _planner = new VoxelTrajectory::RRTPlannerManager(validator);

        ROS_WARN("[NO_INFLATION] rrt planner already.");
    }

    void buildMap(double bdy[_TOT_BDY], double resolution = 0.4, double safe_margin = 0.1, 
            double max_vel = 1.0, double max_acc = 1.0,
            double flight_vel = 1.0, double flight_acc = 1.0)
    {
        _core->setMapBoundary(bdy);
        _core->setResolution(_resolution = resolution);
        _core->setMargin(_safe_margin = safe_margin);
        _core->setMaxVelocity(_max_vel = max_vel);
        _core->setMaxAcceleration(_max_acc = max_acc);
        _core->setFlightVelocity(flight_vel);
        _core->setFlightAcceleration(flight_acc);
        _has_map = true;
    
        double pos_gain[_TOT_DIM] = {3.7, 3.7, 5.2};
        double vel_gain[_TOT_DIM] = {2.4, 2.4, 3.0};
        setGains(pos_gain, vel_gain);
    }
    
    void setGains(double pos_gain[_TOT_DIM], double vel_gain[_TOT_DIM])
    {
        _pos_cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        _pos_cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        _pos_cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        _pos_cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        _pos_cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        _pos_cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    void publishDesiredState()
    {
       //ROS_WARN("[TRAJ_SERVER] BEFORE go to publish desired state!");
       if (!_has_odom) return ; 
       //ROS_WARN("[TRAJ_SERVER] go to publish desired state!");
       // to do : get the desired state from the _core
       if (_has_traj)
       {
           // quadrotor_msgs desired state
           _pos_cmd.header = _odom.header;

           if (_odom.header.stamp < _final_time)
           {
               vector<double> state = _core->getDesiredState(_odom.header.stamp.toSec());
               _pos_cmd.trajectory_flag = 
                   quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
               _pos_cmd.position.x = state[_TOT_DIM * 0 + _DIM_x];
               _pos_cmd.position.y = state[_TOT_DIM * 0 + _DIM_y];
               _pos_cmd.position.z = state[_TOT_DIM * 0 + _DIM_z];

               _pos_cmd.velocity.x = state[_TOT_DIM * 1 + _DIM_x];
               _pos_cmd.velocity.y = state[_TOT_DIM * 1 + _DIM_y];
               _pos_cmd.velocity.z = state[_TOT_DIM * 1 + _DIM_z];

               _pos_cmd.acceleration.x = state[_TOT_DIM * 2 + _DIM_x];
               _pos_cmd.acceleration.y = state[_TOT_DIM * 2 + _DIM_y];
               _pos_cmd.acceleration.z = state[_TOT_DIM * 2 + _DIM_z];

               _pos_cmd.yaw = tf::getYaw(_odom.pose.pose.orientation);
               _pos_cmd.yaw_dot = 0.0;
           }
           else
           {
#if 0
               if (!(_final_time > ros::TIME_MIN)) return ;
               ROS_WARN("[TRAJ] last dest: [%.3lf %.3lf %.3lf]", 
                       _last_dest.pose.pose.position.x,
                       _last_dest.pose.pose.position.y,
                       _last_dest.pose.pose.position.z);

               ROS_WARN("[TRAJ] last desired: \n[%.3lf %.3lf %.3lf]\n[%.3lf %.3lf %.3lf]\n[%.3lf %.3lf %.3lf]", 
                       _pos_cmd.position.x, _pos_cmd.position.y, _pos_cmd.position.z,
                       _pos_cmd.velocity.x, _pos_cmd.velocity.y, _pos_cmd.velocity.z,
                       _pos_cmd.acceleration.x, _pos_cmd.acceleration.y, _pos_cmd.acceleration.z);

               vector<double> state = _core->getDesiredState(_odom.header.stamp.toSec());
               ROS_WARN("[TRAJ] desired dest: [%.3lf %.3lf %.3lf]\n[%.3lf %.3lf %.3lf]\n[%.3lf %.3lf %.3lf]", 
                       state[0], state[1], state[2], 
                       state[3], state[4], state[5], 
                       state[6], state[7], state[8]);
#endif
               _final_time = ros::TIME_MIN;
               
               _pos_cmd.trajectory_flag = 
                   quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
               _pos_cmd.position = _last_dest.pose.pose.position;

               _pos_cmd.velocity.x = 0.0;
               _pos_cmd.velocity.y = 0.0;
               _pos_cmd.velocity.z = 0.0;

               _pos_cmd.acceleration.x = 0.0;
               _pos_cmd.acceleration.y = 0.0;
               _pos_cmd.acceleration.z = 0.0;

               _pos_cmd.yaw = tf::getYaw(_last_dest.pose.pose.orientation);
               _pos_cmd.yaw_dot = 0.0;
           }

           _desired_state_pub.publish(_pos_cmd);
       //    ROS_INFO("[TRAJ] Published desired states, at [%.3lf %.3lf %.3lf]", 
       //           _pos_cmd.position.x, _pos_cmd.position.y, _pos_cmd.position.z);
       }
    }

    void rcvOdometryCallbck(const nav_msgs::Odometry odom)
    {
        if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return ;
        _odom = odom;
        _has_odom = true;
        //ROS_INFO("[TRAJ] Received odometry message. now at : [%.3lf %.3lf %.3lf]",
        //        odom.pose.pose.position.x,
        //        odom.pose.pose.position.y,
        //        odom.pose.pose.position.z);
        _odom_queue.push_back(odom);
        while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();
        
        publishDesiredState();
    }

    void checkHalfWay()
    {
        //ROS_INFO("[TRAJ] Checking halfway obstacle!!");
        if (_has_traj && _odom.header.stamp < _final_time && _core->checkHalfWayObstacle_BrutalForce())
        {
#if 0
            ROS_INFO("[TRAJ] Replan. Will set up %lu waypoint(s).", _waypoints.size());
            ROS_INFO("[TRAJ] Replan. %lu arr_time.", _arr_time.size());
            for (size_t i = 0; i < _arr_time.size(); ++i) 
            {
                ROS_INFO("t = %.3lf, dest = [%.3lf %.3lf %.3lf]", _arr_time[i], 
                        _waypoints[i * _TOT_DIM + _DIM_x],
                        _waypoints[i * _TOT_DIM + _DIM_y],
                        _waypoints[i * _TOT_DIM + _DIM_z]); 
            }
#endif

            _final_time = ros::TIME_MIN;
            
            if (_arr_time.size() * _TOT_DIM != _waypoints.size())
            {
                ROS_WARN("[TRAJ] Replan failed. arr_time size = %lu, waypoints size = %lu",
                        _arr_time.size(), _waypoints.size());
                return ;
            }

            size_t vaild_id;
            for (vaild_id = 0; vaild_id < _arr_time.size(); ++vaild_id)
            {
                if (_odom.header.stamp.toSec() < _arr_time[vaild_id]) break;
            }

            //ROS_INFO("[TRAJ] now time = %.3lf", _odom.header.stamp.toSec());
            //ROS_INFO("[TRAJ] vaild id = %lu", vaild_id);

            if (vaild_id >= _arr_time.size()) return ;

            nav_msgs::Path path;
            path.poses.resize(_arr_time.size() - vaild_id);

            for (size_t idx = vaild_id; idx < _arr_time.size(); ++idx)
            {
                path.poses[idx - vaild_id].pose.position.x = _waypoints[idx * _TOT_DIM + _DIM_x];
                path.poses[idx - vaild_id].pose.position.y = _waypoints[idx * _TOT_DIM + _DIM_y];
                path.poses[idx - vaild_id].pose.position.z = _waypoints[idx * _TOT_DIM + _DIM_z];
            }

            rcvWaypointsCallback(path);
        }
        //ROS_INFO("[TRAJ] HALF_WAY_CHECK DONE!!");
    }

    void rcvGlobalPointCloudCallback(const sensor_msgs::PointCloud2 & cloud)
    {
        if (!_has_map) return ;

        const float * data = reinterpret_cast<const float *>(cloud.data.data());
        vector<double> pt;
        pt.reserve(cloud.width * _TOT_BDY);
        for (size_t idx = 0; idx < cloud.width; ++idx)
        {
            pt.push_back(data[idx * _TOT_DIM + _DIM_x] - _safe_margin);
            pt.push_back(data[idx * _TOT_DIM + _DIM_x] + _safe_margin);
            pt.push_back(data[idx * _TOT_DIM + _DIM_y] - _safe_margin);
            pt.push_back(data[idx * _TOT_DIM + _DIM_y] + _safe_margin);
            pt.push_back(data[idx * _TOT_DIM + _DIM_z] - _safe_margin);
            pt.push_back(data[idx * _TOT_DIM + _DIM_z] + _safe_margin);
        }

        _core->addMapBlock(pt);

        checkHalfWay();

        // visMap();
    }

    void rcvGlobalBlocksCloudCallback(const sensor_msgs::PointCloud2 & cloud)
    {
        if (!_has_map) return ;

        const float * data = reinterpret_cast<const float *>(cloud.data.data());
        vector<double> blk;
        blk.reserve(cloud.width * _TOT_BDY);
        for (size_t idx = 0; idx < cloud.width; ++idx)
        {
            blk.push_back(data[idx * _TOT_BDY + _BDY_x] - _safe_margin);    
            blk.push_back(data[idx * _TOT_BDY + _BDY_X] + _safe_margin);    
            blk.push_back(data[idx * _TOT_BDY + _BDY_y] - _safe_margin);    
            blk.push_back(data[idx * _TOT_BDY + _BDY_Y] + _safe_margin);    
            blk.push_back(data[idx * _TOT_BDY + _BDY_z] - _safe_margin);    
            blk.push_back(data[idx * _TOT_BDY + _BDY_Z] + _safe_margin);    
        }
        _core->addMapBlock(blk);

        checkHalfWay();

        // visMap();
    }

    void rcvLaserScanCallback(const sensor_msgs::LaserScan & scan)
    {
        if (_odom_queue.empty()) return;
        if (scan.header.stamp - _last_scan_stamp < ros::Duration(_laser_scan_step)) return ;
        _last_scan_stamp = scan.header.stamp;

        for (auto & odom: _odom_queue)
        {
            if (odom.header.stamp > scan.header.stamp) 
            {
                insertLaserScan(scan, odom);       
                return ;
            }
        }
        insertLaserScan(scan, _odom_queue.back());
    }

    void insertLaserScan(
            const sensor_msgs::LaserScan & scan, 
            const nav_msgs::Odometry & odom)
    {
        if (!_has_map) return ;

        sensor_msgs::PointCloud cloud;
        laser_geometry::LaserProjection projector;
        projector.projectLaser(scan, cloud);

        // prepare the transform matrix
        Eigen::Quaterniond quad(
                odom.pose.pose.orientation.w,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z);
        auto rotate = quad.toRotationMatrix();

        Eigen::Vector3d trans(
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z);

        // get the local coordinate & store in Eigen vector
        vector<double> blk;
        blk.reserve(scan.ranges.size() * _TOT_BDY);

#if 0
        visualization_msgs::Marker mk;
        mk.ns = "laser_points";
        mk.header.stamp = scan.header.stamp;
        mk.header.frame_id = "map";
        mk.type = visualization_msgs::Marker::CUBE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.color.b = 1.0;
        mk.color.g = 0.0;
        mk.color.r = 0.0;
        mk.color.a = 1.0;
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;
        mk.pose.orientation.w = 1.0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.position.x = 0.0;
        mk.pose.position.y = 0.0;
        mk.pose.position.z = 0.0;
        mk.points.reserve(scan.ranges.size());
#endif

        bool _is_first_pt = true;
        Eigen::Vector3d _last_pt;
        for (auto lp: cloud.points)
        {
            auto pt = rotate * Eigen::Vector3d(lp.x, lp.y, lp.z) + trans;

            if (_is_first_pt)
            {
                _is_first_pt = false;
            }
            else
            {
                if ((pt - _last_pt).norm() < _laser_scan_resolution) continue;
            }
            _last_pt = pt;

            geometry_msgs::Point vis_pt;
            vis_pt.x = pt(_DIM_x);
            vis_pt.y = pt(_DIM_y);
            vis_pt.z = pt(_DIM_z);
            //mk.points.push_back(vis_pt); 

            blk.push_back(pt(_DIM_x) - _safe_margin);
            blk.push_back(pt(_DIM_x) + _safe_margin);
            blk.push_back(pt(_DIM_y) - _safe_margin);
            blk.push_back(pt(_DIM_y) + _safe_margin);

            if (pt(_DIM_z) < _allowed_ground_height)
            {
                blk.push_back(pt(_DIM_z) - _safe_margin);
                blk.push_back(pt(_DIM_z) + _safe_margin);
            }
            else
            {
                blk.push_back(pt(_DIM_z) - _safe_margin - _extra_obstacle_height);
                blk.push_back(pt(_DIM_z) + _safe_margin + _extra_obstacle_height);
            }
        }
        //_laser_vis_pub.publish(mk);
        _core->addMapBlock(blk);

        checkHalfWay();

        // visMap();
    }

    void rcvLocalPointCloud(const sensor_msgs::PointCloud2 cloud)
    {
        if (_odom_queue.empty()) return ;
        for (auto & odom: _odom_queue)
        {
            if (odom.header.stamp > cloud.header.stamp)
            {
                insertBodyPointCloud(cloud, odom);
                return ;
            }
        }
        insertBodyPointCloud(cloud, _odom_queue.back());
    }

    void insertBodyPointCloud(
            const sensor_msgs::PointCloud2 & cloud, 
            const nav_msgs::Odometry & odom)
    {
        if (!_has_map) return ;

        // prepare the transform matrix
        Eigen::Quaterniond quad(
                odom.pose.pose.orientation.w,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z);
        auto rotate = quad.toRotationMatrix();

        Eigen::Vector3d trans(
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z);

        // get the local coordinate & store in Eigen vector
        vector<double> blk;
        blk.reserve(cloud.width * _TOT_BDY);

#if 0
        visualization_msgs::Marker mk;
        mk.ns = "laser_points";
        mk.header.stamp = scan.header.stamp;
        mk.header.frame_id = "map";
        mk.type = visualization_msgs::Marker::CUBE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.color.b = 1.0;
        mk.color.g = 0.0;
        mk.color.r = 0.0;
        mk.color.a = 1.0;
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;
        mk.pose.orientation.w = 1.0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.position.x = 0.0;
        mk.pose.position.y = 0.0;
        mk.pose.position.z = 0.0;
        mk.points.reserve(scan.ranges.size());
#endif
        const float * pts = reinterpret_cast<const float *>(cloud.data.data());

        for (size_t idx = 0; idx < cloud.width; ++idx)
        {
            auto pt = trans + rotate * Eigen::Vector3d(
                    pts[idx * _TOT_DIM + _DIM_x],
                    pts[idx * _TOT_DIM + _DIM_y],
                    pts[idx * _TOT_DIM + _DIM_z]);

            geometry_msgs::Point vis_pt;
            vis_pt.x = pt(_DIM_x);
            vis_pt.y = pt(_DIM_y);
            vis_pt.z = pt(_DIM_z);
            //mk.points.push_back(vis_pt); 

            blk.push_back(pt(_DIM_x) - _safe_margin);
            blk.push_back(pt(_DIM_x) + _safe_margin);
            blk.push_back(pt(_DIM_y) - _safe_margin);
            blk.push_back(pt(_DIM_y) + _safe_margin);
            blk.push_back(pt(_DIM_z) - _safe_margin);
            blk.push_back(pt(_DIM_z) + _safe_margin);
        }
        //_laser_vis_pub.publish(mk);
        _core->addMapBlock(blk);

        checkHalfWay();
        //visMap();
    }

    void rcvDestinationCallback(const geometry_msgs::Point & pt)
    {
        if (!_has_map || !_has_odom) return ;
        _pos_cmd.trajectory_id = ++_traj_id;
        
        vector<double> state = 
        {
            _odom.pose.pose.position.x + _EPS,
            _odom.pose.pose.position.y + _EPS,
            _odom.pose.pose.position.z + _EPS,
            _odom.twist.twist.linear.x,
            _odom.twist.twist.linear.y,
            _odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0
        };

        _waypoints = vector<double> {pt.x, pt.y, pt.z};

        if (_core->setWayPoints(state, _waypoints, _odom.header.stamp.toSec(), _arr_time) != 2)
        {
            _arr_time.clear();
            _waypoints.clear();
            ROS_WARN("[TRAJ] Generating the trajectory failed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _final_time = ros::TIME_MIN;
        }
        else
        {
            _last_dest.pose.pose = _odom.pose.pose;
            _last_dest.pose.pose.position = pt;
            _final_time = ros::Time(_core->getFinalTime());
        }

        _has_traj = true;
         ROS_INFO("[TRAJ] Starting visualization.");
        visTrajectory();
        ROS_INFO("[TRAJ] Trajectory visualzation finished.");
        visPath();
        ROS_INFO("[TRAJ] Path visualzation finished.");
        visInflatedPath();
        ROS_INFO("[TRAJ] Inflated path visualzation finished.");
        visCheckpoint();
        ROS_INFO("[TRAJ] Checkpoints visualzation finished.");
    }

    void rcvWaypointsCallback(const nav_msgs::Path & wp)
    {
        if (!_has_map || !_has_odom) return ;
        _pos_cmd.trajectory_id = ++_traj_id;

        vector<double> state = 
        {
            _odom.pose.pose.position.x + _EPS,
            _odom.pose.pose.position.y + _EPS,
            _odom.pose.pose.position.z + _EPS,
            _odom.twist.twist.linear.x,
            _odom.twist.twist.linear.y,
            _odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0
        };

        _waypoints.clear();
        _waypoints.reserve(wp.poses.size() * _TOT_DIM);

        for (auto & pose: wp.poses) 
        {
            _waypoints.push_back(pose.pose.position.x);
            _waypoints.push_back(pose.pose.position.y);
            _waypoints.push_back(pose.pose.position.z);
        }

        ROS_INFO("[TRAJ] Generating the trajectory.");

        vector<double> cost_time;

        if (_core->setWayPointsRec(state, _waypoints, _odom.header.stamp.toSec(), 
                    _arr_time, cost_time) != 2)
        {
            _arr_time.clear();
            _waypoints.clear();
            ROS_INFO("[TRAJ] Generating the trajectory failed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _final_time = ros::TIME_MIN;
        }
        else
        {
            ROS_INFO("[TRAJ] Generating the trajectory succeed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _last_dest.pose.pose.position = wp.poses.back().pose.position;
            _final_time = ros::Time(_core->getFinalTime());

            stringstream sin;
            sin << "[OLD_TRAJ] The old trajectory cost : "
                << _core->qp_cost[_DIM_x] << ", "
                << _core->qp_cost[_DIM_y] << ", "
                << _core->qp_cost[_DIM_z] << "." << endl
                << cost_time[_DIM_x] << ", "
                << cost_time[_DIM_y] << ", " 
                << cost_time[_DIM_z] << ".";
            std_msgs::String debug_info;
            debug_info.data = sin.str();
            _debug_pub.publish(debug_info);

        }

        _has_traj = true;

        ROS_INFO("[TRAJ] Starting visualization.");
        visTrajectory();
        ROS_INFO("[TRAJ] Trajectory visualzation finished.");
        visPath();
        ROS_INFO("[TRAJ] Path visualzation finished.");
        visInflatedPath();
        ROS_INFO("[TRAJ] Inflated path visualzation finished.");
        visCheckpoint();
        ROS_INFO("[TRAJ] Checkpoints visualzation finished.");
        //publishRRTWaypoints(wp);
    }

    void publishRRTWaypoints(const nav_msgs::Path & wp)
    {
        if (wp.poses.size() == 0) return ;

        Eigen::Vector3d src(
             _odom.pose.pose.position.x,
             _odom.pose.pose.position.y,
             _odom.pose.pose.position.z);

        Eigen::Vector3d dest(
                wp.poses[0].pose.position.x,
                wp.poses[0].pose.position.y,
                wp.poses[0].pose.position.z);

        vector<Eigen::Vector3d> pts;
        if (!_planner->planWithSrcDest(src, dest, pts)) return ;

        clog << "tot num : " << pts.size() << endl;
        clog << "src : \n " << src << "\nbeg : \n" << pts[0] << endl; 
        clog << "dest : \n " << dest << "\nend : \n" << pts.back() << endl; 
        clog << "dist " << (dest - pts.back()).norm() << endl;

        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;

        path.header.frame_id = "/map";
        path.header.stamp = ros::Time::now();
        pose.header = path.header;

        path.poses.clear();
        path.poses.reserve(pts.size());
        for (size_t idx = 0; idx < pts.size(); ++idx)
        {
            if ((pts[idx] - src).norm() < 0.1) continue;
            pose.pose.position.x = pts[idx][_DIM_x];
            pose.pose.position.y = pts[idx][_DIM_y];
            pose.pose.position.z = pts[idx][_DIM_z];
            path.poses.push_back(pose);
        }
        //pose.pose.position.x = src[_DIM_x];
        //pose.pose.position.y = src[_DIM_y];
        //pose.pose.position.z = src[_DIM_z];
        //path.poses.push_back(pose);
        _rrt_path_pub.publish(path);
    }

    void disableVisualization()
    {
        _is_vis = false;
    }

    void enableVisualization()
    {
        _is_vis = true;
    }

    void visMap(const ros::TimerEvent& evt)
    {
        //ROS_INFO("[TRAJ] Map visualization start... flag : is_vis = %d, has_map = %d",
               // _is_vis, _has_map);

        if (!_is_vis || !_has_map) return ;
        vector<double> pt = _core->getPointCloud();
        //ROS_INFO("[TRAJ] Here are %lu occupied grid(s) in the octormap.", pt.size() / 3);

        vector<float> pt32;
        pt32.resize(pt.size());
        for (size_t idx = 0; idx < pt.size(); ++idx) pt32[idx] = static_cast<float>(pt[idx]);
        //ROS_INFO("[TRAJ] float points alredy.");

        const double * bdy = _core->getBdy();
        ROS_INFO("[TRAJ] The boundary of the map: [%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf].",
                bdy[_BDY_x], bdy[_BDY_y], bdy[_BDY_z], bdy[_BDY_X], bdy[_BDY_Y], bdy[_BDY_Z]);

        _map_vis.header.frame_id = "/map";
        _map_vis.header.stamp = _odom.header.stamp;

        _map_vis.height = 1;
        _map_vis.width = pt.size() / _TOT_DIM;
        _map_vis.is_bigendian = false;
        _map_vis.is_dense = true;

        _map_vis.point_step = 4 * _TOT_DIM;
        _map_vis.row_step = _map_vis.point_step * _map_vis.width;

        sensor_msgs::PointField field;
        _map_vis.fields.resize(_TOT_DIM);
        string f_name[_TOT_DIM] = {"x", "y", "z"};
        for (size_t idx = 0; idx < _TOT_DIM; ++idx)
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
        ROS_INFO("[TRAJ] Map visualization finished. Total number of points : %d.", 
                _map_vis.point_step);
    }

    void visTrajectory()
    {
        if (!_is_vis || !_has_traj) return ;
        if (_odom.header.stamp > _final_time) return;
        _traj_vis.header.stamp       = _odom.header.stamp;
        _traj_vis.header.frame_id    = "/map";

        _traj_vis.ns = "no_inflation/trajectory";
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
        _traj_vis.color.g = 0.5;
        _traj_vis.color.b = 0.0;
        _traj_vis.color.a = 0.5;



        double traj_len = 0.0;
        int count = 0;
        Eigen::Vector3d cur, pre;

        double t_begin = _core->getBeginTime(), t_final = _core->getFinalTime();
        _traj_vis.points.clear();
        ROS_INFO("[TRAJ] Trajectory time : %.3lf %.3lf", t_begin, t_final);
        _traj_vis.points.reserve(static_cast<int>((t_final - t_begin) * 100 + 0.5));
        vector<double> state;
        geometry_msgs::Point pt;

        ROS_INFO("[TRAJ] Trajectory visualization prepared.");

        for (double t = t_begin; t < t_final; t += 0.02, count += 1)
        {
            state = _core->getDesiredState(t);
            cur(_DIM_x) = pt.x = state[_DIM_x];
            cur(_DIM_y) = pt.y = state[_DIM_y];
            cur(_DIM_z) = pt.z = state[_DIM_z];
            _traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }

        ROS_INFO("[TRAJ] The length of the trajectory; %.3lfm.", traj_len);
        _traj_vis_pub.publish(_traj_vis);
    }

    void visPath()
    {
        if (!_is_vis || !_has_traj) return ;

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
            mk.pose.position.x = (path(idx, _BDY_x) + path(idx, _BDY_X)) * 0.5;
            mk.pose.position.y = (path(idx, _BDY_y) + path(idx, _BDY_Y)) * 0.5;
            mk.pose.position.z = (path(idx, _BDY_z) + path(idx, _BDY_Z)) * 0.5;
            mk.scale.x = path(idx, _BDY_X) - path(idx, _BDY_x);
            mk.scale.y = path(idx, _BDY_Y) - path(idx, _BDY_y);
            mk.scale.z = path(idx, _BDY_Z) - path(idx, _BDY_z);

            _path_vis.markers.push_back(mk);
        }
        _path_vis_pub.publish(_path_vis);
    }

    void visInflatedPath()
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
            mk.pose.position.x = (path(idx, _BDY_x) + path(idx, _BDY_X)) * 0.5;
            mk.pose.position.y = (path(idx, _BDY_y) + path(idx, _BDY_Y)) * 0.5;
            mk.pose.position.z = (path(idx, _BDY_z) + path(idx, _BDY_Z)) * 0.5;
            mk.scale.x = path(idx, _BDY_X) - path(idx, _BDY_x);
            mk.scale.y = path(idx, _BDY_Y) - path(idx, _BDY_y);
            mk.scale.z = path(idx, _BDY_Z) - path(idx, _BDY_z);

            _inflated_path_vis.markers.push_back(mk);
        }
        _inflated_path_vis_pub.publish(_inflated_path_vis);
    }

    void visCheckpoint()
    {
        if (!_is_vis || !_has_traj) return ;
        _check_point_vis.header.stamp = _odom.header.stamp;
        _check_point_vis.header.frame_id = string("/map");

        vector<double> pt = _core->getCheckPoint();

        _check_point_vis.height = 1;
        _check_point_vis.width = pt.size() / 3;
        _check_point_vis.is_bigendian = false;
        _check_point_vis.is_dense = true;

        _check_point_vis.point_step = _TOT_DIM * 4;
        _check_point_vis.row_step = _check_point_vis.point_step * _check_point_vis.width;

        sensor_msgs::PointField field;
        _check_point_vis.fields.resize(_TOT_DIM);
        string f_name [] = {"x", "y", "z"};
        for (size_t idx = 0; idx < _TOT_DIM; ++idx)
        {
            field.name = f_name[idx];
            field.offset = idx * 4;
            field.datatype = 7;
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
    
    void setLaserScanFilterSetting(double step, double resolution)
    {
        this->_laser_scan_step = step;
        this->_laser_scan_resolution = resolution;
    }

    void set2DLaserDataSetting(double extra_height, double ground_height)
    {
        this->_extra_obstacle_height = extra_height;
        this->_allowed_ground_height = ground_height;
    }

    void setFlightHeightLimit(double height_limit)
    {
        if (!_has_map) return ;

        _flight_height_limit = height_limit;

        vector<double> blk = {
            -1e5, 1e5,
            -1e5, 1e5,
            height_limit, height_limit + 1e-7};
        _core->addMapBlock(blk);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "trajectory_server_node");
    ros::NodeHandle handle("~");

    NodeServer server(handle);

    bool is_vis;
    double bdy[_TOT_BDY], resolution, safe_margin;
    double max_vel, max_acc;
    double flight_vel, flight_acc;
    double flight_height_limit;
    double extra_obstacle_heigt;
    double allowed_ground_height;
    double laser_resolution, laser_step;

    handle.param("flag/visualization", is_vis, true);
    handle.param("map/boundary/lower_x", bdy[_BDY_x], -100.0);
    handle.param("map/boundary/upper_x", bdy[_BDY_X], 100.0);
    handle.param("map/boundary/lower_y", bdy[_BDY_y], -100.0);
    handle.param("map/boundary/upper_y", bdy[_BDY_Y], 100.0);
    handle.param("map/boundary/lower_z", bdy[_BDY_z], -100.0);
    handle.param("map/boundary/upper_z", bdy[_BDY_Z], 200.0);
    handle.param("map/resolution", resolution, 0.4);
    handle.param("map/safe_margin", safe_margin, 0.3);
    handle.param("max_velocity", max_vel, 1.0);
    handle.param("max_acceleration", max_acc, 1.0);
    handle.param("flight_velocity", flight_vel, max_vel);
    handle.param("flight_acceleration", flight_acc, max_acc);
    handle.param("setting/flight_height_limit", flight_height_limit, -1e7);
    handle.param("setting/extra_obstacle_height", extra_obstacle_heigt, 0.0);
    handle.param("setting/allowed_ground_height", allowed_ground_height, 1e7);
    handle.param("setting/laser_scan_step", laser_step, 0.2);
    handle.param("setting/laser_scan_resolution", laser_resolution, 0.05);


    ROS_INFO("[TRAJ_SERVER] vel = %.3lf acc = %.3lf", max_vel, max_acc);

    server.buildMap(bdy, resolution, safe_margin, max_vel, max_acc, flight_vel, flight_acc);
    server.setFlightHeightLimit(flight_height_limit);
    server.set2DLaserDataSetting(extra_obstacle_heigt, allowed_ground_height);
    server.setLaserScanFilterSetting(laser_step, laser_resolution);

    if (is_vis) 
        server.enableVisualization();
    else
        server.disableVisualization();

    // ros::Rate work_rate(1.0);
    // while (ros::ok())
    // {
    //     server.visMap();
    //     work_rate.sleep();
    //     ros::spinOnce();
    // }

    ros::spin();
    return 0;
}
