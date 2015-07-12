
//#define _TRAJECTORY_TRAJECTROY_USE_MAVLINK_MSG_
#define _TRAJECTORY_USE_VISUALIZATION_

#include "voxel_trajectory/voxelserver.h"
#include "voxel_trajectory/voxelmacro.h"

// ros tools
#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"
//#include "pcl/io/pcd_io.h"
//#include "pcl/point_types.h"
// standard messages
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
// desired states messages
#include "quadrotor_msgs/PositionCommand.h"
//#include "mavlink_message/PositionCommand.h"

using namespace std;

// server state
class NodeServer
{
private:
    // _core server
    VoxelTrajectory::VoxelServer * _core = new VoxelTrajectory::VoxelServer();
    const double _EPS = 1e-9;
    const double _EPS_POS = 1e-6;

    // interface
    // subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _obs_pt_sub, _obs_blk_sub;
    ros::Subscriber _dest_pt_sub, _dest_pts_sub; 

    // publishers
    ros::Publisher _desired_state_pub;

    ros::Publisher _map_vis_pub;
    ros::Publisher _traj_vis_pub;
    ros::Publisher _check_point_vis_pub;
    ros::Publisher _path_vis_pub;
    ros::Publisher _inflated_path_vis_pub;

    // flags 
    bool _has_odom = false;
    bool _has_traj = false;
    bool _has_map = false;
    bool _is_vis = true;

    // configuration
    double _resolution = 0.4;        // unit - m^3
    double _safe_margin = 0.4;       // unit - m
    double _max_acc = 1.0;           // unit - m/s
    double _max_vel = 1.0;           // unit - m/(s^2)

    // trajectory
    uint32_t _traj_id = 0;
    uint8_t _traj_status = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    vector<double> _waypoints;

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

public:

    NodeServer(ros::NodeHandle & handle)
    {
        // subscribe odometry
        _odom_sub = 
            handle.subscribe("odometry", 50, &NodeServer::odometryCallback, this);

        // add obstacle
        _obs_pt_sub =
            handle.subscribe("obstacle_points", 2, &NodeServer::addObstaclePoints, this);
        _obs_blk_sub = 
            handle.subscribe("obstacle_blocks", 2, &NodeServer::addObstacleBlocks, this);

        // set destination
        _dest_pt_sub = 
            handle.subscribe("goal_point", 2, &NodeServer::setDestination, this);
        _dest_pts_sub = 
            handle.subscribe("waypoints", 2, &NodeServer::setDestinationWaypoints, this);

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

    }

    void buildMap(double bdy[_TOT_BDY], double resolution = 0.4, double safe_margin = 0.4, 
            double max_vel = 1.0, double max_acc = 1.0)
    {
        _core->setMapBoundary(bdy);
        _core->setResolution(_resolution = resolution);
        _core->setMargin(_safe_margin = safe_margin);
        _core->setMaxVelocity(_max_vel = max_vel);
        _core->setMaxAcceleration(_max_acc = max_acc);
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
       if (!_has_odom) return ; 
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

               if (_waypoints.size() > 0 &&
                    abs(state[_DIM_x] - _waypoints[_DIM_x]) < _EPS_POS &&
                    abs(state[_DIM_y] - _waypoints[_DIM_y]) < _EPS_POS &&
                    abs(state[_DIM_z] - _waypoints[_DIM_z]) < _EPS_POS)
               {
                   for (size_t idx = 0; idx + _TOT_DIM < _waypoints.size(); ++idx)
                   {
                       _waypoints[idx] = _waypoints[idx + _TOT_DIM];
                   }
                   _waypoints.pop_back();
                   _waypoints.pop_back();
                   _waypoints.pop_back();
               }


               _pos_cmd.yaw = tf::getYaw(_odom.pose.pose.orientation);
               _pos_cmd.yaw_dot = 0.0;
           }
           else
           {
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
           //ROS_INFO("[TRAJ] Published desired states, at [%.3lf %.3lf %.3lf]", 
           //        _pos_cmd.position.x, _pos_cmd.position.y, _pos_cmd.position.z);
       }
    }

    void odometryCallback(const nav_msgs::Odometry odom)
    {
        _odom = odom;
        _has_odom = true;
        //ROS_INFO("[TRAJ] Received odometry message. now at : [%.3lf %.3lf %.3lf]",
        //        odom.pose.pose.position.x,
        //        odom.pose.pose.position.y,
        //        odom.pose.pose.position.z);
        
        publishDesiredState();
    }

    void checkHalfWay()
    {
        if (_odom.header.stamp < _final_time && _core->checkHalfWayObstacle_BrutalForce())
        {
            if (_waypoints.size() == 0) return ;
            nav_msgs::Path path;
            path.poses.resize(_waypoints.size() / _TOT_DIM);
            for (size_t idx = 0; idx * _TOT_DIM < _waypoints.size(); ++idx)
            {
                path.poses[idx].pose.position.x = _waypoints[idx * _TOT_DIM + _DIM_x];
                path.poses[idx].pose.position.y = _waypoints[idx * _TOT_DIM + _DIM_y];
                path.poses[idx].pose.position.z = _waypoints[idx * _TOT_DIM + _DIM_z];
            }

            setDestinationWaypoints(path);
        }
    }

    void addObstaclePoints(const sensor_msgs::PointCloud2 & cloud)
    {
        if (!_has_map) return ;

        const float * data = reinterpret_cast<const float *>(cloud.data.data());
        vector<double> pt;
        pt.reserve(cloud.width * _TOT_DIM);
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

        visMap();
    }

    void addObstacleBlocks(const sensor_msgs::PointCloud2 & cloud)
    {
        if (!_has_map) return ;

        const float * data = reinterpret_cast<const float *>(cloud.data.data());
        vector<double> blk;
        blk.reserve(cloud.width * _TOT_DIM);
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

        visMap();
    }

    void setDestination(const geometry_msgs::Point & pt)
    {
        if (!_has_map) return ;
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

        vector<double> waypoints = {pt.x, pt.y, pt.z};

        _waypoints.clear();
        if (_core->setWayPoints(state, waypoints, _odom.header.stamp.toSec()) != 2)
        {
            ROS_WARN("[TRAJ] Generating the trajectory failed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _final_time = ros::TIME_MIN;
        }
        else
        {
            _waypoints = waypoints;
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

    void setDestinationWaypoints(const nav_msgs::Path & wp)
    {
        if (!_has_map) return ;
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

        vector<double> waypoints;
        waypoints.reserve(wp.poses.size() * _TOT_DIM);

        for (auto & pose: wp.poses) 
        {
            waypoints.push_back(pose.pose.position.x);
            waypoints.push_back(pose.pose.position.y);
            waypoints.push_back(pose.pose.position.z);
        }

        _waypoints.clear();

        if (_core->setWayPoints(state, waypoints, _odom.header.stamp.toSec()) != 2)
        {
            ROS_INFO("[TRAJ] Generating the trajectory failed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _final_time = ros::TIME_MIN;
        }
        else
        {
            ROS_INFO("[TRAJ] Generating the trajectory succeed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _last_dest.pose.pose.position = wp.poses.back().pose.position;
            _waypoints = waypoints;
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

    void disableVisualization()
    {
        _is_vis = false;
    }

    void enableVisualization()
    {
        _is_vis = true;
    }

    void visMap()
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
        _map_vis.width = pt.size() / 3;
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
            field.datatype = 7;
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

        ros::Rate loop_rate(1);
        for (int i = 1; i < 3; i++) 
        {
            if (_map_vis_pub.getNumSubscribers() > 0) break;
            loop_rate.sleep();
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

        _traj_vis.ns = "trajectory/trajectory";
        _traj_vis.id = 0;
        _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        _traj_vis.action = visualization_msgs::Marker::ADD;
        _traj_vis.scale.x = 0.05;
        _traj_vis.scale.y = 0.05;
        _traj_vis.scale.z = 0.05;
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

        for (size_t idx = 1; idx <= M; ++idx)
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
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "trajectory_server_node");
    ros::NodeHandle handle("~");

    NodeServer server(handle);

    bool is_vis;
    double bdy[_TOT_BDY], resolution, safe_margin;
    double max_vel, max_acc;

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


    ROS_INFO("[TRAJ] vel = %.3lf acc = %.3lf", max_vel, max_acc);
    server.buildMap(bdy, resolution, safe_margin, max_vel, max_acc);

    if (is_vis) 
        server.enableVisualization();
    else
        server.disableVisualization();

    ros::spin();
    return 0;
}
