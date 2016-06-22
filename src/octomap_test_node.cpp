
#include "voxel_trajectory/traj_utils.h"

// ros
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <set>
#include <deque>
#include <sstream>
#include <cmath>
#include <voxel_map/voxel_map.h>
using namespace std;

class MapManager
{
private:
    // subscribed topic as system input
    ros::Subscriber _odom_sub, _scan_sub;
    // advertised topic as system output
    ros::Publisher _map_vis_pub;
    ros::Publisher _debug_pub;

    // do thing repeatly 
    ros::Timer _vis_tmr;

    // odometry
    nav_msgs::Odometry _odom;
    const size_t _odom_queue_size = 500;
    deque<nav_msgs::Odometry> _odom_queue;

    // map
    octomap::OcTree *_core;
    double _resolution, _radius;
    bool _is_started = false;
    ros::Time _start_time;
public:
    MapManager(ros::NodeHandle & handle);

    void rcvOdometry(const nav_msgs::Odometry & odom);
    void rcv3DLaserScan(const sensor_msgs::PointCloud2 & cloud);

    void visMap(const ros::TimerEvent &);

};

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "octomap_node");
    ros::NodeHandle handle("~");

    MapManager manager(handle);
    ros::spin();

    return 0;
}


MapManager::MapManager(ros::NodeHandle & handle)
{
    // topic for input
    {
        _odom_sub = 
            handle.subscribe("odometry", 50, &MapManager::rcvOdometry, this);
        _scan_sub = 
            handle.subscribe("laser_scan_3d", 1, &MapManager::rcv3DLaserScan, this);
    }

    // topic for output
    {
        _map_vis_pub = 
            handle.advertise<sensor_msgs::PointCloud>("map_vis", 2);
        _debug_pub = 
            handle.advertise<std_msgs::String>("debug_info", 10);

        _vis_tmr = 
            handle.createTimer(ros::Duration(1.0), &MapManager::visMap, this);
    }

    {
        handle.param("map/resolution", _resolution, 0.1);
        handle.param("map/observation_radius", _radius, 10.0);

        double lo_hit, lo_mis, lo_min, lo_max, lo_occ;
        handle.param("map/log_odd/min", lo_min, -2.0);
        handle.param("map/log_odd/max", lo_max, 5.0);
        handle.param("map/log_odd/occupy", lo_occ, 0.0);
        handle.param("map/log_odd/hit", lo_hit, 2.45);
        handle.param("map/log_odd/miss", lo_mis, -0.4);
        _core = new octomap::OcTree(_resolution);
        _core->setClampingThresMax(octomap::probability(lo_max));
        _core->setClampingThresMin(octomap::probability(lo_min));
        _core->setOccupancyThres(octomap::probability(lo_occ));
        _core->setProbHit(octomap::probability(lo_hit));
        _core->setProbMiss(octomap::probability(lo_mis));
    }
}

void MapManager::rcvOdometry(const nav_msgs::Odometry & odom)
{
    _odom = odom;

    _odom_queue.push_back(odom);
    while (_odom_queue.size() > _odom_queue_size) _odom_queue.pop_front();
}

void MapManager::rcv3DLaserScan(const sensor_msgs::PointCloud2 & cloud)
{
    if (!_is_started) _is_started = true, _start_time = cloud.header.stamp; 

    if (_odom_queue.empty()) return ;
    nav_msgs::Odometry laser_odom = _odom_queue.back();
    for (auto & odom: _odom_queue)
    {
        if (odom.header.stamp > cloud.header.stamp)
        {
            laser_odom = odom;
            break;
        }
    }

    vector<voxel_map::Ray> rays;
    if (!voxel_trajectory::retRaysFromCloud2Odom(
                cloud, laser_odom, rays, -100, 0.0, 1))
        return ;

    ROS_WARN_STREAM(" max range = " << _radius);

    octomap::point3d origin(
            laser_odom.pose.pose.position.x, 
            laser_odom.pose.pose.position.y, 
            laser_odom.pose.pose.position.z);
#if 0
    octomap::Pointcloud ray_cloud;

    for (auto ray: rays)
    {
        auto dest = ray.target();
        ray_cloud.push_back(dest(0), dest(1), dest(2));
    }

    _core->insertPointCloudRays(ray_cloud, origin, _radius);
#else
    ros::Time pre_time = ros::Time::now();
    for (auto ray : rays)
    {
        auto dest = ray.target();
        octomap::point3d target(dest(0), dest(1), dest(2));
        _core->insertRay(origin, target, _radius);
    }
    stringstream sin;
    sin << (cloud.header.stamp - _start_time) << " \t" << (ros::Time::now() - pre_time);
    std_msgs::String msg;
    msg.data = sin.str();
    _debug_pub.publish(msg);
#endif
}

void MapManager::visMap(const ros::TimerEvent & evt)
{
    return ;
    sensor_msgs::PointCloud cloud;
    cloud.header = _odom.header;
    cloud.header.frame_id = "/map";
    for (octomap::OcTree::leaf_iterator it = _core->begin_leafs(), end = _core->end_leafs(); 
            it != end; ++it)
        if (_core->isNodeOccupied(*it))
        {
            auto pt = it.getCoordinate();
            geometry_msgs::Point32 p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            cloud.points.emplace_back(p);
        }
    _map_vis_pub.publish(cloud);
}
