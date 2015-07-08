
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
    vector<double> _goal_points;

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
    }

    void publishDesiredState()
    {
       if (!_has_odom) return ; 
       // to do : get the desired state from the _core
       if (_has_traj)
       {
           vector<double> state = _core->getDesiredState(_odom.header.stamp.toSec());

           // quadrotor_msgs desired state
           _pos_cmd.header = _odom.header;

           if (_odom.header.stamp < _final_time)
           {
               _pos_cmd.trajectory_flag = 
                   quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
               _pos_cmd.position.x = state[_TOT_DIM * 0 + _DIM_x];
               _pos_cmd.position.y = state[_TOT_DIM * 0 + _DIM_y];
               _pos_cmd.position.z = state[_TOT_DIM * 0 + _DIM_z];

               _pos_cmd.velocity.x = state[_TOT_BDY * 1 + _DIM_x];
               _pos_cmd.velocity.y = state[_TOT_BDY * 1 + _DIM_y];
               _pos_cmd.velocity.z = state[_TOT_BDY * 1 + _DIM_z];

               _pos_cmd.acceleration.x = state[_TOT_BDY * 2 + _DIM_x];
               _pos_cmd.acceleration.y = state[_TOT_BDY * 2 + _DIM_y];
               _pos_cmd.acceleration.z = state[_TOT_BDY * 2 + _DIM_z];

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
       }
    }

    void odometryCallback(const nav_msgs::Odometry odom)
    {
        _odom = odom;
        _has_odom = true;
        
        publishDesiredState();
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
    }

    void setDestination(const geometry_msgs::Point & pt)
    {
        if (!_has_map) return ;
        _pos_cmd.trajectory_id = ++_traj_id;
        
        vector<double> state = 
        {
            _odom.pose.pose.position.x,
            _odom.pose.pose.position.y,
            _odom.pose.pose.position.z,
            _odom.twist.twist.linear.x,
            _odom.twist.twist.linear.y,
            _odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0
        };

        vector<double> waypoints = {pt.x, pt.y, pt.z};

        if (_core->setWayPoints(state, waypoints, _odom.header.stamp.toSec()) != 2)
        {
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
    }

    void setDestinationWaypoints(const nav_msgs::Path & wp)
    {
        if (!_has_map) return ;
        _pos_cmd.trajectory_id = ++_traj_id;

        vector<double> state = 
        {
            _odom.pose.pose.position.x,
            _odom.pose.pose.position.y,
            _odom.pose.pose.position.z,
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

        if (_core->setWayPoints(state, waypoints, _odom.header.stamp.toSec()) != 2)
        {
            ROS_WARN("[TRAJ] Generating the trajectory failed!");
            _last_dest.pose.pose = _odom.pose.pose;
            _final_time = ros::TIME_MIN;
        }
        else
        {
            _last_dest.pose.pose = _odom.pose.pose;
            _last_dest.pose.pose.position = wp.poses.back().pose.position;
            _final_time = ros::Time(_core->getFinalTime());
        }

        _has_traj = true;
    }

    void disableVisulization()
    {
        _is_vis = false;
    }

    void enbaleVisulization()
    {
        _is_vis = true;
    }

    void visMap()
    {
        if (!_is_vis || !_has_map) return ;
        vector<double> pt = _core->getPointCloud();

        float pt32[pt.size()];
        for (size_t idx = 0; idx < pt.size(); ++idx)
            pt32[idx] = static_cast<float>(pt[idx]);

        ROS_WARN("[TRAJ] Here are %lu occupied grid(s) in the octormap.", pt.size() / 3);
        const double * bdy = _core->getBdy();
        ROS_WARN("[TRAJ] The boundary of the map: [%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf].",
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
        _map_vis.resize(_TOT_DIM);
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
        uint8_t * pt_int = reinterpret_cast<uint8_t *>(pt32);
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
    }

    void visTrajectory()
    {
        if (!_is_vis || !_has_traj) return ;
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
        _traj_vis.points.reserve(static_cast<int>((t_final - t_begin) * 100 + 0.5));
        vector<double> state;
        geometry_msgs::Point pt;

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

        ROS_DEBUG("[TRAJ] The length of the trajectory; %.3lfm.", traj_len);
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
        size_t M = path.rows() >> 1;

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

    ros::spin();
    return 0;
}
